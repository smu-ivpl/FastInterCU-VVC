/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

 /** \file     StreamMergeApp.cpp
     \brief    Decoder application class
 */

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "StreamMergeApp.h"
#include "DecoderLib/AnnexBread.h"
#include "DecoderLib/NALread.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

 //! \ingroup DecoderApp
 //! \{

 // ====================================================================================================================
 // Constructor / destructor / initialization / destroy
 // ====================================================================================================================

StreamMergeApp::StreamMergeApp()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in StreamMergeApp class
 - delete allocated buffers
 - destroy internal class
 - returns the number of mismatching pictures
 */

void read2(InputNALUnit& nalu)
{
  InputBitstream& bs = nalu.getBitstream();

  nalu.m_forbiddenZeroBit   = bs.read(1);                 // forbidden zero bit
  nalu.m_nuhReservedZeroBit = bs.read(1);                 // nuh_reserved_zero_bit
  nalu.m_nuhLayerId         = bs.read(6);                 // nuh_layer_id
  nalu.m_nalUnitType        = (NalUnitType) bs.read(5);   // nal_unit_type
  nalu.m_temporalId         = bs.read(3) - 1;             // nuh_temporal_id_plus1
}

static void
_byteStreamNALUnit(
  SingleLayerStream& bs,
  std::istream& istream,
  vector<uint8_t>& nalUnit,
  AnnexBStats& stats)
{
  /* At the beginning of the decoding process, the decoder initialises its
   * current position in the byte stream to the beginning of the byte stream.
   * It then extracts and discards each leading_zero_8bits syntax element (if
   * present), moving the current position in the byte stream forward one
   * byte at a time, until the current position in the byte stream is such
   * that the next four bytes in the bitstream form the four-byte sequence
   * 0x00000001.
   */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::SStat &statBits = CodingStatistics::GetStatisticEP(STATS__NAL_UNIT_PACKING);
#endif
  while ((bs.eofBeforeNBytes(24 / 8, istream) || bs.peekBytes(24 / 8, istream) != 0x000001)
    && (bs.eofBeforeNBytes(32 / 8, istream) || bs.peekBytes(32 / 8, istream) != 0x00000001))
  {
    uint8_t leading_zero_8bits = bs.readByte(istream);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    statBits.bits += 8; statBits.count++;
#endif
    if (leading_zero_8bits != 0) { THROW("Leading zero bits not zero"); }
    stats.m_numLeadingZero8BitsBytes++;
  }

  /* 1. When the next four bytes in the bitstream form the four-byte sequence
   * 0x00000001, the next byte in the byte stream (which is a zero_byte
   * syntax element) is extracted and discarded and the current position in
   * the byte stream is set equal to the position of the byte following this
   * discarded byte.
   */
   /* NB, the previous step guarantees this will succeed -- if EOF was
    * encountered, an exception will stop execution getting this far */
  if (bs.peekBytes(24 / 8, istream) != 0x000001)
  {
    uint8_t zero_byte = bs.readByte(istream);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    statBits.bits += 8; statBits.count++;
#endif
    CHECK_(zero_byte != 0, "Zero byte not '0'");
    stats.m_numZeroByteBytes++;
  }

  /* 2. The next three-byte sequence in the byte stream (which is a
   * start_code_prefix_one_3bytes) is extracted and discarded and the current
   * position in the byte stream is set equal to the position of the byte
   * following this three-byte sequence.
   */
   /* NB, (1) guarantees that the next three bytes are 0x00 00 01 */
  uint32_t start_code_prefix_one_3bytes = bs.readBytes(24 / 8, istream);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  statBits.bits += 24; statBits.count += 3;
#endif
  if (start_code_prefix_one_3bytes != 0x000001) { THROW("Invalid code prefix"); }
  stats.m_numStartCodePrefixBytes += 3;

  /* 3. NumBytesInNALunit is set equal to the number of bytes starting with
   * the byte at the current position in the byte stream up to and including
   * the last byte that precedes the location of any of the following
   * conditions:
   *   a. A subsequent byte-aligned three-byte sequence equal to 0x000000, or
   *   b. A subsequent byte-aligned three-byte sequence equal to 0x000001, or
   *   c. The end of the byte stream, as determined by unspecified means.
   */
   /* 4. NumBytesInNALunit bytes are removed from the bitstream and the
    * current position in the byte stream is advanced by NumBytesInNALunit
    * bytes. This sequence of bytes is nal_unit( NumBytesInNALunit ) and is
    * decoded using the NAL unit decoding process
    */
    /* NB, (unsigned)x > 2 implies n!=0 && n!=1 */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::SStat &bodyStats = CodingStatistics::GetStatisticEP(STATS__NAL_UNIT_TOTAL_BODY);
#endif
  while (bs.eofBeforeNBytes(24 / 8, istream) || bs.peekBytes(24 / 8, istream) > 2)
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    uint8_t thebyte = bs.readByte(istream); bodyStats.bits += 8; bodyStats.count++;
    nalUnit.push_back(thebyte);
#else
    nalUnit.push_back(bs.readByte(istream));
#endif
  }

  /* 5. When the current position in the byte stream is:
   *  - not at the end of the byte stream (as determined by unspecified means)
   *  - and the next bytes in the byte stream do not start with a three-byte
   *    sequence equal to 0x000001
   *  - and the next bytes in the byte stream do not start with a four byte
   *    sequence equal to 0x00000001,
   * the decoder extracts and discards each trailing_zero_8bits syntax
   * element, moving the current position in the byte stream forward one byte
   * at a time, until the current position in the byte stream is such that:
   *  - the next bytes in the byte stream form the four-byte sequence
   *    0x00000001 or
   *  - the end of the byte stream has been encountered (as determined by
   *    unspecified means).
   */
   /* NB, (3) guarantees there are at least three bytes available or none */
  while ((bs.eofBeforeNBytes(24 / 8, istream) || bs.peekBytes(24 / 8, istream) != 0x000001)
    && (bs.eofBeforeNBytes(32 / 8, istream) || bs.peekBytes(32 / 8, istream) != 0x00000001))
  {
    uint8_t trailing_zero_8bits = bs.readByte(istream);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    statBits.bits += 8; statBits.count++;
#endif
    CHECK_(trailing_zero_8bits != 0, "Trailing zero bits not '0'");
    stats.m_numTrailingZero8BitsBytes++;
  }
}

/**
 * Parse an AVC AnnexB Bytestream bs to extract a single nalUnit
 * while accumulating bytestream statistics into stats.
 *
 * Returns false if EOF was reached (NB, nalunit data may be valid),
 *         otherwise true.
 */
bool
byteStreamNALUnit(
  SingleLayerStream& bs,
  std::istream& istream,
  vector<uint8_t>& nalUnit,
  AnnexBStats& stats)
{
  bool eof = false;
  try
  {
    _byteStreamNALUnit(bs, istream, nalUnit, stats);
  }
  catch (...)
  {
    eof = true;
  }
  stats.m_numBytesInNALUnit = uint32_t(nalUnit.size());
  return eof;
}

void StreamMergeApp::writeNewVPS(ostream& out, int nLayerId, int nTemporalId)
{
  //write NALU header
  OutputBitstream bsNALUHeader;
  static const uint8_t start_code_prefix[] = { 0,0,0,1 };

  int forbiddenZero = 0;
  bsNALUHeader.write(forbiddenZero, 1);   // forbidden_zero_bit
  int nuhReservedZeroBit = 0;
  bsNALUHeader.write(nuhReservedZeroBit, 1);   // nuh_reserved_zero_bit
  bsNALUHeader.write(nLayerId, 6);             // nuh_layer_id
  bsNALUHeader.write(NAL_UNIT_VPS, 5);         // nal_unit_type
  bsNALUHeader.write(nTemporalId + 1, 3);      // nuh_temporal_id_plus1

  out.write(reinterpret_cast<const char*>(start_code_prefix), 4);
  out.write(reinterpret_cast<const char*>(bsNALUHeader.getByteStream()), bsNALUHeader.getByteStreamLength());

  //write VPS
  OutputBitstream bsVPS;
  HLSWriter       m_HLSWriter;

  m_HLSWriter.setBitstream(&bsVPS);
  m_HLSWriter.codeVPS(&vps);

  out.write(reinterpret_cast<const char*>(bsVPS.getByteStream()), bsVPS.getByteStreamLength());

  return;
}

uint32_t StreamMergeApp::mergeStreams()
{
  ifstream bitstreamFileIn[MAX_VPS_LAYERS];
  ofstream bitstreamFileOut(m_bitstreamFileNameOut.c_str(), ifstream::out | ifstream::binary);
  int nNumValidStr = m_numInputStreams;

  for (int i = 0; i < m_numInputStreams; i++)
  {
    bitstreamFileIn[i].open(m_bitstreamFileNameIn[i].c_str(), ifstream::in | ifstream::binary);

    if (!bitstreamFileIn[i])
    {
      EXIT("failed to open bitstream file " << m_bitstreamFileNameIn[i].c_str() << " for reading");
    }

    bitstreamFileIn[i].clear();
    bitstreamFileIn[i].seekg(0, ios::beg);
  }

  SingleLayerStream bytestream[MAX_VPS_LAYERS];

  for (int i = 0; i < m_numInputStreams; i++)
    bytestream[i].init(bitstreamFileIn[i]);

  //set VPS which will be replicated for all layers but with differnt nul_layer_id
  vps.setMaxLayers(m_numInputStreams);
  vps.setVPSExtensionFlag(false);

  //Loop all input bitstreams to interleave their NALUs
  while (nNumValidStr)
  {
    //loop over all input streams
    for (int i = 0; i < m_numInputStreams; i++)
    {
      uint8_t layerId = i < 63 ? i : i + 1;

      if (!bitstreamFileIn[i])
        continue;

      AnnexBStats stats = AnnexBStats();

      InputNALUnit nalu;

      byteStreamNALUnit(bytestream[i], bitstreamFileIn[i], nalu.getBitstream().getFifo(), stats);

      // call actual decoding function
      if (nalu.getBitstream().getFifo().empty())
      {
        /* this can happen if the following occur:
         *  - empty input file
         *  - two back-to-back start_code_prefixes
         *  - start_code_prefix immediately followed by EOF
         */
        std::cerr << "Warning: Attempt to decode an empty NAL unit" << std::endl;
      }
      else
      {
        read2(nalu);

        if (nalu.m_nalUnitType == NAL_UNIT_VPS)
        {
          writeNewVPS(bitstreamFileOut, layerId, nalu.m_temporalId);
          printf("Write new VPS for stream %d\n", i);

          continue;
        }

        int iNumZeros = stats.m_numLeadingZero8BitsBytes + stats.m_numZeroByteBytes + stats.m_numStartCodePrefixBytes - 1;
        char ch = 0;
        for (int i = 0; i < iNumZeros; i++) { bitstreamFileOut.write(&ch, 1); }
        ch = 1; bitstreamFileOut.write(&ch, 1);

        //update the nul_layer_id
        uint8_t *p = (uint8_t*)nalu.getBitstream().getFifo().data();
        p[1] = ((layerId + 1) << 1) & 0xff;

        bitstreamFileOut.write((const char*)p, nalu.getBitstream().getFifo().size());

        printf("Merge NALU type %d from stream %d\n", nalu.m_nalUnitType, i);
      }

      if (!bitstreamFileIn[i])
        nNumValidStr--;
    }
  }

  return 0;
}

//! \}

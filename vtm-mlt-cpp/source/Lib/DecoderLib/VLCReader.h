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

/** \file     VLCWReader.h
 *  \brief    Reader for high level syntax
 */

#ifndef __VLCREADER__
#define __VLCREADER__

#include "CommonLib/Rom.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Slice.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/ParameterSetManager.h"
#include "CABACReader.h"

#if ENABLE_TRACING

#define READ_SCODE(length, code, name)    xReadSCode  ( length, code, name )
#define READ_CODE(length, code, name)     xReadCodeTr ( length, code, name )
#define READ_UVLC(        code, name)     xReadUvlcTr (         code, name )
#define READ_SVLC(        code, name)     xReadSvlcTr (         code, name )
#define READ_FLAG(        code, name)     xReadFlagTr (         code, name )

#else

#if RExt__DECODER_DEBUG_BIT_STATISTICS

#define READ_SCODE(length, code, name)    xReadSCode( length, code, name )
#define READ_CODE(length, code, name)     xReadCode ( length, code, name )
#define READ_UVLC(        code, name)     xReadUvlc (         code, name )
#define READ_SVLC(        code, name)     xReadSvlc (         code, name )
#define READ_FLAG(        code, name)     xReadFlag (         code, name )

#else

#define READ_SCODE(length, code, name)    xReadSCode ( length, code )
#define READ_CODE(length, code, name)     xReadCode ( length, code )
#define READ_UVLC(        code, name)     xReadUvlc (         code )
#define READ_SVLC(        code, name)     xReadSvlc (         code )
#define READ_FLAG(        code, name)     xReadFlag (         code )

#endif

#endif

//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class VLCReader
{
protected:
  InputBitstream*   m_pcBitstream;

  VLCReader() : m_pcBitstream (NULL) {};
  virtual ~VLCReader() {};

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  void  xReadCode    ( uint32_t   length, uint32_t& val, const char *pSymbolName );
  void  xReadUvlc    (                uint32_t& val, const char *pSymbolName );
  void  xReadSvlc    (                 int& val, const char *pSymbolName );
  void  xReadFlag    (                uint32_t& val, const char *pSymbolName );
#else
  void  xReadCode    ( uint32_t   length, uint32_t& val );
  void  xReadUvlc    (                uint32_t& val );
  void  xReadSvlc    (                 int& val );
  void  xReadFlag    (                uint32_t& val );
#endif
#if ENABLE_TRACING
  void  xReadCodeTr  ( uint32_t  length, uint32_t& rValue, const char *pSymbolName );
  void  xReadUvlcTr  (               uint32_t& rValue, const char *pSymbolName );
  void  xReadSvlcTr  (                int& rValue, const char *pSymbolName );
  void  xReadFlagTr  (               uint32_t& rValue, const char *pSymbolName );
#endif
#if RExt__DECODER_DEBUG_BIT_STATISTICS || ENABLE_TRACING
  void  xReadSCode   ( uint32_t  length, int& val, const char *pSymbolName );
#else
  void  xReadSCode   ( uint32_t  length, int& val );
#endif

public:
  void  setBitstream ( InputBitstream* p )   { m_pcBitstream = p; }
  InputBitstream* getBitstream() { return m_pcBitstream; }

protected:
  void xReadRbspTrailingBits();
  bool isByteAligned() { return (m_pcBitstream->getNumBitsUntilByteAligned() == 0 ); }
};



class AUDReader: public VLCReader
{
public:
  AUDReader() {};
  virtual ~AUDReader() {};
  void parseAccessUnitDelimiter(InputBitstream* bs, uint32_t &audIrapOrGdrAuFlag, uint32_t &picType);
};



class FDReader: public VLCReader
{
public:
  FDReader() {};
  virtual ~FDReader() {};
  void parseFillerData(InputBitstream* bs, uint32_t &fdSize);
};



class HLSyntaxReader : public VLCReader
{
public:
  HLSyntaxReader();
  virtual ~HLSyntaxReader();

protected:
  void  copyRefPicList(SPS* pcSPS, ReferencePictureList* source_rpl, ReferencePictureList* dest_rpl);
  void  parseRefPicList(SPS* pcSPS, ReferencePictureList* rpl, int rplIdx);

public:
  void  setBitstream        ( InputBitstream* p )   { m_pcBitstream = p; }
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  void  parseOPI            ( OPI* opi );
#endif
  void  parseVPS            ( VPS* pcVPS );
  void  parseDCI            ( DCI* dci );
  void  parseSPS            ( SPS* pcSPS );
  void  parsePPS            ( PPS* pcPPS );
  void  parseAPS            ( APS* pcAPS );
  void  parseAlfAps         ( APS* pcAPS );
  void  parseLmcsAps        ( APS* pcAPS );
  void  parseScalingListAps ( APS* pcAPS );
  void  parseVUI            ( VUI* pcVUI, SPS* pcSPS );
  void  parseConstraintInfo   (ConstraintInfo *cinfo);
  void  parseProfileTierLevel(ProfileTierLevel *ptl, bool profileTierPresentFlag, int maxNumSubLayersMinus1);
  void  parseOlsHrdParameters(GeneralHrdParams* generalHrd, OlsHrdParams *olsHrd, uint32_t firstSubLayer, uint32_t tempLevelHigh);
  void parseGeneralHrdParameters(GeneralHrdParams *generalHrd);
  void  parsePictureHeader  ( PicHeader* picHeader, ParameterSetManager *parameterSetManager, bool readRbspTrailingBits );
  void  checkAlfNaluTidAndPicTid(Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager);
  void  parseSliceHeader    ( Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager, const int prevTid0POC, const int prevPicPOC );
  void  getSlicePoc ( Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager, const int prevTid0POC );
  void  parseTerminatingBit ( uint32_t& ruiBit );
  void  parseRemainingBytes ( bool noTrailingBytesExpected );

  void  parsePredWeightTable( Slice* pcSlice, const SPS *sps );
  void parsePredWeightTable ( PicHeader *picHeader, const PPS *pps, const SPS *sps );
  void parseScalingList     ( ScalingList *scalingList, bool aps_chromaPresentFlag );
  void  decodeScalingList   ( ScalingList *scalingList, uint32_t scalingListId, bool isPredictor);
  void parseReshaper        ( SliceReshapeInfo& sliceReshaperInfo, const SPS* pcSPS, const bool isIntra );
  void alfFilter( AlfParam& alfParam, const bool isChroma, const int altIdx );
  void ccAlfFilter( Slice *pcSlice );
  void dpb_parameters(int maxSubLayersMinus1, bool subLayerInfoFlag, SPS *pcSPS);
private:

protected:
  bool  xMoreRbspData();
};


//! \}

#endif

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

/** \file     EncApp.h
    \brief    Encoder application class (header)
*/

#ifndef __ENCAPP__
#define __ENCAPP__

#include <list>
#include <ostream>

#include "EncoderLib/EncLib.h"
#include "Utilities/VideoIOYuv.h"
#include "CommonLib/NAL.h"
#include "EncAppCfg.h"
#if EXTENSION_360_VIDEO
#include "AppEncHelper360/TExt360AppEncTop.h"
#endif
#include "EncoderLib/EncTemporalFilter.h"

#if JVET_O0756_CALCULATE_HDRMETRICS
#include <chrono>
#endif

class EncAppCommon;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder application class
class EncApp : public EncAppCfg, public AUWriterIf
{
private:
  // class interface
  EncLib            m_cEncLib;                    ///< encoder class
  VideoIOYuv        m_cVideoIOYuvInputFile;       ///< input YUV file
  VideoIOYuv        m_cVideoIOYuvReconFile;       ///< output reconstruction file
  int               m_iFrameRcvd;                 ///< number of received frames
  uint32_t          m_essentialBytes;
  uint32_t          m_totalBytes;
  fstream&          m_bitstream;
#if JVET_O0756_CALCULATE_HDRMETRICS
  std::chrono::duration<long long, ratio<1, 1000000000>> m_metricTime;
#endif

private:
  // initialization
  void xCreateLib( std::list<PelUnitBuf*>& recBufList, const int layerId );         ///< create files & encoder class
  void xInitLibCfg ();                           ///< initialize internal variables
  void xInitLib    (bool isFieldCoding);         ///< initialize encoder class
  void xDestroyLib ();                           ///< destroy encoder class

  // file I/O
  void xWriteOutput     ( int iNumEncoded, std::list<PelUnitBuf*>& recBufList
                         );                      ///< write bitstream to file
  void rateStatsAccum   ( const AccessUnit& au, const std::vector<uint32_t>& stats);
  void printRateSummary ();
  void printChromaFormat();

  std::list<PelUnitBuf*> m_recBufList;
  int                    m_numEncoded;
  PelStorage*            m_trueOrgPic;
  PelStorage*            m_orgPic;
  PelStorage*            m_filteredOrgPic;
#if EXTENSION_360_VIDEO
  TExt360AppEncTop*      m_ext360;
#endif
  EncTemporalFilter      m_temporalFilter;
  bool m_flush;

public:
  EncApp( fstream& bitStream, EncLibCommon* encLibCommon );
  virtual ~EncApp();

  int   getMaxLayers() const { return m_maxLayers; }
  void  createLib( const int layerIdx );
  void  destroyLib();
  bool  encodePrep( bool& eos );
  bool  encode();                               ///< main encoding function

  void  outputAU( const AccessUnit& au );

#if JVET_O0756_CALCULATE_HDRMETRICS
  std::chrono::duration<long long, ratio<1, 1000000000>> getMetricTime()    const { return m_metricTime; };
#endif
  VPS * getVPS() { return m_cEncLib.getVPS(); }
  int   getChromaFormatIDC() const { return m_cEncLib.getChromaFormatIdc(); }
  int   getBitDepth() const { return m_cEncLib.getBitDepth(CHANNEL_TYPE_LUMA); }
};// END CLASS DEFINITION EncApp

//! \}

#endif // __ENCAPP__


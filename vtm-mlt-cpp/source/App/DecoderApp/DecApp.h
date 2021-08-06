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

/** \file     TAppDecLib.h
    \brief    Decoder application class (header)
*/

#ifndef __DECAPP__
#define __DECAPP__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Utilities/VideoIOYuv.h"
#include "CommonLib/Picture.h"
#include "DecoderLib/DecLib.h"
#include "DecAppCfg.h"

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder application class
class DecApp : public DecAppCfg
{
private:
  // class interface
  DecLib          m_cDecLib;                     ///< decoder class
  std::unordered_map<int, VideoIOYuv>      m_cVideoIOYuvReconFile;        ///< reconstruction YUV class

  // for output control
  int             m_iPOCLastDisplay;              ///< last POC in display order
  std::ofstream   m_seiMessageFileStream;         ///< Used for outputing SEI messages.

  std::ofstream   m_oplFileStream;                ///< Used to output log file for confomance testing

  bool            m_newCLVS[MAX_NUM_LAYER_IDS];   ///< used to record a new CLVSS


private:
  bool  xIsNaluWithinTargetDecLayerIdSet( const InputNALUnit* nalu ) const; ///< check whether given Nalu is within targetDecLayerIdSet
  bool  xIsNaluWithinTargetOutputLayerIdSet( const InputNALUnit* nalu ) const; ///< check whether given Nalu is within targetOutputLayerIdSet

public:
  DecApp();
  virtual ~DecApp         ()  {}

  uint32_t  decode            (); ///< main decoding function

private:
  void  xCreateDecLib     (); ///< create internal classes
  void  xDestroyDecLib    (); ///< destroy internal classes
  void  xWriteOutput      ( PicList* pcListPic , uint32_t tId); ///< write YUV to file
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
  void  xFlushOutput( PicList *pcListPic, const int layerId = NOT_VALID, bool noOutputOfPriorPicsFlag = false );   ///< flush all remaining decoded pictures to file
#else
  void  xFlushOutput( PicList* pcListPic, const int layerId = NOT_VALID ); ///< flush all remaining decoded pictures to file
#endif
  bool  isNewPicture(ifstream *bitstreamFile, class InputByteStream *bytestream);  ///< check if next NAL unit will be the first NAL unit from a new picture
  bool  isNewAccessUnit(bool newPicture, ifstream *bitstreamFile, class InputByteStream *bytestream);  ///< check if next NAL unit will be the first NAL unit from a new access unit

  void  writeLineToOutputLog(Picture * pcPic);

};

//! \}

#endif // __DECAPP__


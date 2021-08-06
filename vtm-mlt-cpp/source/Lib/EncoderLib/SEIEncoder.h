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

#pragma once

#ifndef __SEIENCODER__
#define __SEIENCODER__

#include "CommonLib/SEI.h"

// forward declarations
class EncCfg;
class EncLib;
class EncGOP;


//! Initializes different SEI message types based on given encoder configuration parameters
class SEIEncoder
{
public:
  SEIEncoder()
    :m_pcCfg(NULL)
    ,m_pcEncLib(NULL)
    ,m_pcEncGOP(NULL)
  ,m_isInitialized(false)
  {};
  virtual ~SEIEncoder(){};

  void init(EncCfg* encCfg, EncLib *encTop, EncGOP *encGOP)
  {
    m_pcCfg = encCfg;
    m_pcEncGOP = encGOP;
    m_pcEncLib = encTop;
    m_isInitialized = true;
  };

  // leading SEIs
  void initSEIFramePacking(SEIFramePacking *sei, int currPicNum);
  void initSEIParameterSetsInclusionIndication(SEIParameterSetsInclusionIndication* sei);
  void initSEIDependentRAPIndication(SEIDependentRAPIndication *sei);
  void initSEIBufferingPeriod(SEIBufferingPeriod *sei, bool noLeadingPictures);
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void initSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics *sei);
#endif
  void initSEIScalableNesting(SEIScalableNesting *scalableNestingSEI, SEIMessages &nestedSEIs, const std::vector<int> &targetOLSs, const std::vector<int> &targetLayers, const std::vector<uint16_t> &subpictureIDs);
  void initDecodedPictureHashSEI(SEIDecodedPictureHash *sei, PelUnitBuf& pic, std::string &rHashString, const BitDepths &bitDepths);
  void initSEIErp(SEIEquirectangularProjection *sei);
  void initSEISphereRotation(SEISphereRotation *sei);
  void initSEIOmniViewport(SEIOmniViewport *sei);
  void initSEIRegionWisePacking(SEIRegionWisePacking *sei);
  void initSEIGcmp(SEIGeneralizedCubemapProjection *sei);
  void initSEISubpictureLevelInfo(SEISubpicureLevelInfo *sei, const SPS *sps);
  void initSEISampleAspectRatioInfo(SEISampleAspectRatioInfo *sei);
  void initSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics *sei);
  void initSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume *sei);
  void initSEIContentLightLevel(SEIContentLightLevelInfo *sei);
  void initSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment *sei);
  void initSEIContentColourVolume(SEIContentColourVolume *sei);
private:
  EncCfg* m_pcCfg;
  EncLib* m_pcEncLib;
  EncGOP* m_pcEncGOP;

  bool m_isInitialized;
};


//! \}

#endif // __SEIENCODER__

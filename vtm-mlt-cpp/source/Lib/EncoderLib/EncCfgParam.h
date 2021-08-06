
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


#ifndef __ENCCFGPARAMS__
#define __ENCCFGPARAMS__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CommonLib/CommonDef.h"

namespace EncCfgParam {

class CfgVPSParameters
{
public:
  CfgVPSParameters()
#if !JVET_R0193
  : m_maxTidILRefPicsPlus1(-1)
#endif
  {}

  virtual ~CfgVPSParameters(){}

#if JVET_R0193
  std::vector<std::vector<uint32_t>> m_maxTidILRefPicsPlus1;
#else
  int m_maxTidILRefPicsPlus1;
#endif
};

class CfgSEISubpictureLevel
{
public:

  CfgSEISubpictureLevel()
  : m_enabled (false)
  , m_explicitFraction (false)
  , m_numSubpictures (1)
  , m_sliMaxSublayers(1)
  , m_sliSublayerInfoPresentFlag (false)
  {}

  virtual ~CfgSEISubpictureLevel(){}

  bool                      m_enabled;
  std::vector<Level::Name>  m_refLevels;
  bool                      m_explicitFraction;
  int                       m_numSubpictures;
  std::vector<int>          m_nonSubpicLayersFraction;
  std::vector<int>          m_fractions;
  int                       m_sliMaxSublayers;
  bool                      m_sliSublayerInfoPresentFlag;
};

}


#endif // __ENCCFGPARAMS__

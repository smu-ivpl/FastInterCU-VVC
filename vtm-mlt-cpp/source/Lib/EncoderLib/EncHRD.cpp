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

#include "EncHRD.h"

#if U0132_TARGET_BITS_SATURATION

// calculate scale value of bitrate and initial delay
int EncHRD::xCalcScale(int x)
{
  if (x==0)
  {
    return 0;
  }
  uint32_t mask = 0xffffffff;
  int scaleValue = 32;

  while ((x&mask) != 0)
  {
    scaleValue--;
    mask = (mask >> 1);
  }

  return scaleValue;
}
#endif

void EncHRD::initHRDParameters(EncCfg* encCfg)
{
  bool useSubCpbParams = encCfg->getNoPicPartitionFlag() == false;
  int  bitRate = encCfg->getTargetBitrate();
# if U0132_TARGET_BITS_SATURATION
  int cpbSize = encCfg->getCpbSize();
  CHECK_(!(cpbSize != 0), "Unspecified error");  // CPB size may not be equal to zero. ToDo: have a better default and check for level constraints
  if (!encCfg->getHrdParametersPresentFlag() && !encCfg->getCpbSaturationEnabled())
#else
  if (!encCfg->getHrdParametersPresentFlag())
#endif
  {
    return;
  }

  switch (encCfg->getFrameRate())
  {
  case 24:
    m_generalHrdParams.setNumUnitsInTick(1125000);    m_generalHrdParams.setTimeScale(27000000);
    break;
  case 25:
    m_generalHrdParams.setNumUnitsInTick(1080000);    m_generalHrdParams.setTimeScale(27000000);
    break;
  case 30:
    m_generalHrdParams.setNumUnitsInTick(900900);     m_generalHrdParams.setTimeScale(27000000);
    break;
  case 50:
    m_generalHrdParams.setNumUnitsInTick(540000);     m_generalHrdParams.setTimeScale(27000000);
    break;
  case 60:
    m_generalHrdParams.setNumUnitsInTick(450450);     m_generalHrdParams.setTimeScale(27000000);
    break;
  default:
    m_generalHrdParams.setNumUnitsInTick(1001);       m_generalHrdParams.setTimeScale(60000);
    break;
  }

  if (encCfg->getTemporalSubsampleRatio() > 1)
  {
    uint32_t temporalSubsampleRatio = encCfg->getTemporalSubsampleRatio();
    if (double(m_generalHrdParams.getNumUnitsInTick()) * temporalSubsampleRatio > std::numeric_limits<uint32_t>::max())
    {
      m_generalHrdParams.setTimeScale(m_generalHrdParams.getTimeScale() / temporalSubsampleRatio);
    }
    else
    {
      m_generalHrdParams.setNumUnitsInTick(m_generalHrdParams.getNumUnitsInTick() * temporalSubsampleRatio);
    }
  }
  bool rateCnt = (bitRate > 0);

  m_generalHrdParams.setGeneralNalHrdParametersPresentFlag(rateCnt);
  m_generalHrdParams.setGeneralVclHrdParametersPresentFlag(rateCnt);

  m_generalHrdParams.setGeneralSamePicTimingInAllOlsFlag(encCfg->getSamePicTimingInAllOLS());
  useSubCpbParams &= (m_generalHrdParams.getGeneralNalHrdParametersPresentFlag() || m_generalHrdParams.getGeneralVclHrdParametersPresentFlag());
  m_generalHrdParams.setGeneralDecodingUnitHrdParamsPresentFlag(useSubCpbParams);

  if (m_generalHrdParams.getGeneralDecodingUnitHrdParamsPresentFlag())
  {
    m_generalHrdParams.setTickDivisorMinus2(100 - 2);
  }

#if U0132_TARGET_BITS_SATURATION
  if (xCalcScale(bitRate) <= 6)
  {
    m_generalHrdParams.setBitRateScale(0);
  }
  else
  {
    m_generalHrdParams.setBitRateScale(xCalcScale(bitRate) - 6);
  }

  if (xCalcScale(cpbSize) <= 4)
  {
    m_generalHrdParams.setCpbSizeScale(0);
  }
  else
  {
    m_generalHrdParams.setCpbSizeScale(xCalcScale(cpbSize) - 4);
  }
#else
  m_generalHrdParams.setBitRateScale(4);                                       // in units of 2^( 6 + 4 ) = 1,024 bps
  m_generalHrdParams.setCpbSizeScale(6);                                       // in units of 2^( 4 + 6 ) = 1,024 bit
#endif

  m_generalHrdParams.setCpbSizeDuScale(6);                                     // in units of 2^( 4 + 6 ) = 1,024 bit
  m_generalHrdParams.setHrdCpbCntMinus1(0);


  // Note: parameters for all temporal layers are initialized with the same values
  int i, j;
  uint32_t bitrateValue, cpbSizeValue;
  uint32_t duCpbSizeValue;
  uint32_t duBitRateValue = 0;
  OlsHrdParams* olsHrdParams = getOlsHrdParametersAddr();

  for (i = 0; i < MAX_TLAYER; i++)
  {
    olsHrdParams[i].setFixedPicRateGeneralFlag(1);
    olsHrdParams[i].setFixedPicRateWithinCvsFlag(1);
    olsHrdParams[i].setElementDurationInTcMinus1(0);
    olsHrdParams[i].setLowDelayHrdFlag(0);

    //! \todo check for possible PTL violations
    // BitRate[ i ] = ( bit_rate_value_minus1[ i ] + 1 ) * 2^( 6 + bit_rate_scale )
    bitrateValue = bitRate / (1 << (6 + m_generalHrdParams.getBitRateScale()));      // bitRate is in bits, so it needs to be scaled down
                                                                              // CpbSize[ i ] = ( cpb_size_value_minus1[ i ] + 1 ) * 2^( 4 + cpb_size_scale )
#if U0132_TARGET_BITS_SATURATION
    cpbSizeValue = cpbSize / (1 << (4 + m_generalHrdParams.getCpbSizeScale()));      // using bitRate results in 1 second CPB size
#else
    cpbSizeValue = bitRate / (1 << (4 + m_generalHrdParams.getCpbSizeScale()));      // using bitRate results in 1 second CPB size
#endif


                                                                              // DU CPB size could be smaller (i.e. bitrateValue / number of DUs), but we don't know
                                                                              // in how many DUs the slice segment settings will result
    duCpbSizeValue = bitrateValue;
    duBitRateValue = cpbSizeValue;

    for (j = 0; j < (m_generalHrdParams.getHrdCpbCntMinus1() + 1); j++)
    {
      olsHrdParams[i].setBitRateValueMinus1(j, 0, (bitrateValue - 1));
      olsHrdParams[i].setCpbSizeValueMinus1(j, 0, (cpbSizeValue - 1));
      olsHrdParams[i].setDuCpbSizeValueMinus1(j, 0, (duCpbSizeValue - 1));
      olsHrdParams[i].setDuBitRateValueMinus1(j, 0, (duBitRateValue - 1));
      olsHrdParams[i].setCbrFlag(j, 0, false);

      olsHrdParams[i].setBitRateValueMinus1(j, 1, (bitrateValue - 1));
      olsHrdParams[i].setCpbSizeValueMinus1(j, 1, (cpbSizeValue - 1));
      olsHrdParams[i].setDuCpbSizeValueMinus1(j, 1, (duCpbSizeValue - 1));
      olsHrdParams[i].setDuBitRateValueMinus1(j, 1, (duBitRateValue - 1));
      olsHrdParams[i].setCbrFlag(j, 1, false);
    }
  }
}


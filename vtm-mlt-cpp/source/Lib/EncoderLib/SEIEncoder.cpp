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

#include "CommonLib/CommonDef.h"
#include "CommonLib/SEI.h"
#include "EncGOP.h"
#include "EncLib.h"

uint32_t calcMD5(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcCRC(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcChecksum(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
std::string hashToString(const PictureHash &digest, int numChar);

//! \ingroup EncoderLib
//! \{

void SEIEncoder::initSEIFramePacking(SEIFramePacking *seiFramePacking, int currPicNum)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiFramePacking!=NULL), "Unspecified error");

  seiFramePacking->m_arrangementId = m_pcCfg->getFramePackingArrangementSEIId();
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType = m_pcCfg->getFramePackingArrangementSEIType();
  CHECK_(!((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) ), "Unspecified error");
  seiFramePacking->m_quincunxSamplingFlag = m_pcCfg->getFramePackingArrangementSEIQuincunx();
  seiFramePacking->m_contentInterpretationType = m_pcCfg->getFramePackingArrangementSEIInterpretation();
  seiFramePacking->m_spatialFlippingFlag = 0;
  seiFramePacking->m_frame0FlippedFlag = 0;
  seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && (currPicNum&1) );
  seiFramePacking->m_frame0SelfContainedFlag = 0;
  seiFramePacking->m_frame1SelfContainedFlag = 0;
  seiFramePacking->m_frame0GridPositionX = 0;
  seiFramePacking->m_frame0GridPositionY = 0;
  seiFramePacking->m_frame1GridPositionX = 0;
  seiFramePacking->m_frame1GridPositionY = 0;
  seiFramePacking->m_arrangementReservedByte = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio = 0;
}


void SEIEncoder::initSEIParameterSetsInclusionIndication(SEIParameterSetsInclusionIndication* seiParameterSetsInclusionIndication)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiParameterSetsInclusionIndication != NULL), "Unspecified error");

  seiParameterSetsInclusionIndication->m_selfContainedClvsFlag = m_pcCfg->getSelfContainedClvsFlag();
}

void SEIEncoder::initSEIBufferingPeriod(SEIBufferingPeriod *bufferingPeriodSEI, bool noLeadingPictures)
{
  CHECK_(!(m_isInitialized), "bufferingPeriodSEI already initialized");
  CHECK_(!(bufferingPeriodSEI != nullptr), "Need a bufferingPeriodSEI for initialization (got nullptr)");

  uint32_t uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
  bufferingPeriodSEI->m_bpNalCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpVclCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpMaxSubLayers = m_pcCfg->getMaxTempLayer() ;
  bufferingPeriodSEI->m_bpCpbCnt = 1;
  for(int i=0; i < bufferingPeriodSEI->m_bpMaxSubLayers; i++)
  {
    for(int j=0; j < bufferingPeriodSEI->m_bpCpbCnt; j++)
    {
      bufferingPeriodSEI->m_initialCpbRemovalDelay[j][i][0] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalDelay[j][i][1] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[j][i][0] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[j][i][1] = uiInitialCpbRemovalDelay;
    }
  }
  // We don't set concatenation_flag here. max_initial_removal_delay_for_concatenation depends on the usage scenario.
  // The parameters could be added to config file, but as long as the initialisation of generic buffering parameters is
  // not controllable, it does not seem to make sense to provide settings for these.
  bufferingPeriodSEI->m_concatenationFlag = false;
  bufferingPeriodSEI->m_maxInitialRemovalDelayForConcatenation = uiInitialCpbRemovalDelay;

  bufferingPeriodSEI->m_bpDecodingUnitHrdParamsPresentFlag = m_pcCfg->getNoPicPartitionFlag() == false;
  bufferingPeriodSEI->m_decodingUnitCpbParamsInPicTimingSeiFlag = !m_pcCfg->getDecodingUnitInfoSEIEnabled();

  bufferingPeriodSEI->m_initialCpbRemovalDelayLength = 16;                  // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  // Note: The following parameters require some knowledge about the GOP structure.
  //       Using getIntraPeriod() should be avoided though, because it assumes certain GOP
  //       properties, which are only valid in CTC.
  //       Still copying this setting from HM for consistency, improvements welcome
  bool isRandomAccess  = m_pcCfg->getIntraPeriod() > 0;
  if( isRandomAccess )
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 6;                        // 32 = 2^5 (plus 1)
    bufferingPeriodSEI->m_dpbOutputDelayLength =  6;                        // 32 + 3 = 2^6
  }
  else
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 9;                        // max. 2^10
    bufferingPeriodSEI->m_dpbOutputDelayLength =  9;                        // max. 2^10
  }
  bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength = 7;               // ceil( log2( tick_divisor_minus2 + 2 ) )
  bufferingPeriodSEI->m_dpbOutputDelayDuLength = bufferingPeriodSEI->m_dpbOutputDelayLength + bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength;
  //for the concatenation, it can be set to one during splicing.
  bufferingPeriodSEI->m_concatenationFlag = 0;
  //since the temporal layer HRDParameters is not ready, we assumed it is fixed
  bufferingPeriodSEI->m_auCpbRemovalDelayDelta = 1;
  bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag = m_pcCfg->getBpDeltasGOPStructure() ;
  if (bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag)
  {
    switch (m_pcCfg->getGOPSize())
    {
      case 8:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 7;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
        }
      }
        break;
      case 16:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 4;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[5]          = 7;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[6]          = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[7]          = 14;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[8]          = 15;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 7;
        }
      }
        break;
      default:
      {
        THROW("m_cpbRemovalDelayDelta not applicable for the GOP size");
      }
        break;
    }
  }
  bufferingPeriodSEI->m_sublayerDpbOutputOffsetsPresentFlag = true;
  for(int i = 0; i < bufferingPeriodSEI->m_bpMaxSubLayers; i++)
  {
    bufferingPeriodSEI->m_dpbOutputTidOffset[i] = m_pcCfg->getMaxNumReorderPics(i) * static_cast<int>(pow(2, static_cast<double>(bufferingPeriodSEI->m_bpMaxSubLayers-1-i)));
    if(bufferingPeriodSEI->m_dpbOutputTidOffset[i] >= m_pcCfg->getMaxNumReorderPics(bufferingPeriodSEI->m_bpMaxSubLayers-1))
    {
      bufferingPeriodSEI->m_dpbOutputTidOffset[i] -= m_pcCfg->getMaxNumReorderPics(bufferingPeriodSEI->m_bpMaxSubLayers-1);
    }
    else
    {
      bufferingPeriodSEI->m_dpbOutputTidOffset[i] = 0;
    }
  }
  // A commercial encoder should track the buffer state for all layers and sub-layers
  // to ensure CPB conformance. Such tracking is required for calculating alternative
  // CPB parameters.
  // Unfortunately VTM does not have such tracking. Thus we cannot encode alternative
  // CPB parameters here.
  bufferingPeriodSEI->m_altCpbParamsPresentFlag = false;
  bufferingPeriodSEI->m_useAltCpbParamsFlag = false;
}

void SEIEncoder::initSEIErp(SEIEquirectangularProjection* seiEquirectangularProjection)
{
  CHECK_(!(m_isInitialized), "seiEquirectangularProjection already initialized");
  CHECK_(!(seiEquirectangularProjection != nullptr), "Need a seiEquirectangularProjection for initialization (got nullptr)");

  seiEquirectangularProjection->m_erpCancelFlag = m_pcCfg->getErpSEICancelFlag();
  if (!seiEquirectangularProjection->m_erpCancelFlag)
  {
    seiEquirectangularProjection->m_erpPersistenceFlag   = m_pcCfg->getErpSEIPersistenceFlag();
    seiEquirectangularProjection->m_erpGuardBandFlag     = m_pcCfg->getErpSEIGuardBandFlag();
    if (seiEquirectangularProjection->m_erpGuardBandFlag == 1)
    {
      seiEquirectangularProjection->m_erpGuardBandType       = m_pcCfg->getErpSEIGuardBandType();
      seiEquirectangularProjection->m_erpLeftGuardBandWidth  = m_pcCfg->getErpSEILeftGuardBandWidth();
      seiEquirectangularProjection->m_erpRightGuardBandWidth = m_pcCfg->getErpSEIRightGuardBandWidth();
    }
  }
}

void SEIEncoder::initSEISphereRotation(SEISphereRotation* seiSphereRotation)
{
  CHECK_(!(m_isInitialized), "seiSphereRotation already initialized");
  CHECK_(!(seiSphereRotation != nullptr), "Need a seiSphereRotation for initialization (got nullptr)");

  seiSphereRotation->m_sphereRotationCancelFlag = m_pcCfg->getSphereRotationSEICancelFlag();
  if ( !seiSphereRotation->m_sphereRotationCancelFlag )
  {
    seiSphereRotation->m_sphereRotationPersistenceFlag = m_pcCfg->getSphereRotationSEIPersistenceFlag();
    seiSphereRotation->m_sphereRotationYaw = m_pcCfg->getSphereRotationSEIYaw();
    seiSphereRotation->m_sphereRotationPitch = m_pcCfg->getSphereRotationSEIPitch();
    seiSphereRotation->m_sphereRotationRoll = m_pcCfg->getSphereRotationSEIRoll();
  }
}

void SEIEncoder::initSEIOmniViewport(SEIOmniViewport* seiOmniViewport)
{
  CHECK_(!(m_isInitialized), "seiOmniViewport already initialized");
  CHECK_(!(seiOmniViewport != nullptr), "Need a seiOmniViewport for initialization (got nullptr)");

  seiOmniViewport->m_omniViewportId = m_pcCfg->getOmniViewportSEIId();
  seiOmniViewport->m_omniViewportCancelFlag = m_pcCfg->getOmniViewportSEICancelFlag();
  if ( !seiOmniViewport->m_omniViewportCancelFlag )
  {
    seiOmniViewport->m_omniViewportPersistenceFlag = m_pcCfg->getOmniViewportSEIPersistenceFlag();
    seiOmniViewport->m_omniViewportCntMinus1 = m_pcCfg->getOmniViewportSEICntMinus1();

    seiOmniViewport->m_omniViewportRegions.resize(seiOmniViewport->m_omniViewportCntMinus1+1);
    for (uint32_t i = 0; i <= seiOmniViewport->m_omniViewportCntMinus1; i++)
    {
      SEIOmniViewport::OmniViewport &viewport = seiOmniViewport->m_omniViewportRegions[i];
      viewport.azimuthCentre   = m_pcCfg->getOmniViewportSEIAzimuthCentre(i);
      viewport.elevationCentre = m_pcCfg->getOmniViewportSEIElevationCentre(i);
      viewport.tiltCentre      = m_pcCfg->getOmniViewportSEITiltCentre(i);
      viewport.horRange        = m_pcCfg->getOmniViewportSEIHorRange(i);
      viewport.verRange        = m_pcCfg->getOmniViewportSEIVerRange(i);
    }
  }
}

void SEIEncoder::initSEIRegionWisePacking(SEIRegionWisePacking *seiRegionWisePacking)
{
  CHECK_(!(m_isInitialized), "seiRegionWisePacking already initialized");
  CHECK_(!(seiRegionWisePacking != nullptr), "Need a seiRegionWisePacking for initialization (got nullptr)");

  seiRegionWisePacking->m_rwpCancelFlag                          = m_pcCfg->getRwpSEIRwpCancelFlag();
  seiRegionWisePacking->m_rwpPersistenceFlag                     = m_pcCfg->getRwpSEIRwpPersistenceFlag();
  seiRegionWisePacking->m_constituentPictureMatchingFlag         = m_pcCfg->getRwpSEIConstituentPictureMatchingFlag();
  seiRegionWisePacking->m_numPackedRegions                       = m_pcCfg->getRwpSEINumPackedRegions();
  seiRegionWisePacking->m_projPictureWidth                       = m_pcCfg->getRwpSEIProjPictureWidth();
  seiRegionWisePacking->m_projPictureHeight                      = m_pcCfg->getRwpSEIProjPictureHeight();
  seiRegionWisePacking->m_packedPictureWidth                     = m_pcCfg->getRwpSEIPackedPictureWidth();
  seiRegionWisePacking->m_packedPictureHeight                    = m_pcCfg->getRwpSEIPackedPictureHeight();
  seiRegionWisePacking->m_rwpTransformType.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpProjRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpLeftGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpRightGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpTopGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpBottomGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandType.resize(4*seiRegionWisePacking->m_numPackedRegions);
  for( int i=0; i < seiRegionWisePacking->m_numPackedRegions; i++ )
  {
    seiRegionWisePacking->m_rwpTransformType[i]                  = m_pcCfg->getRwpSEIRwpTransformType(i);
    seiRegionWisePacking->m_rwpGuardBandFlag[i]                  = m_pcCfg->getRwpSEIRwpGuardBandFlag(i);
    seiRegionWisePacking->m_projRegionWidth[i]                   = m_pcCfg->getRwpSEIProjRegionWidth(i);
    seiRegionWisePacking->m_projRegionHeight[i]                  = m_pcCfg->getRwpSEIProjRegionHeight(i);
    seiRegionWisePacking->m_rwpProjRegionTop[i]                  = m_pcCfg->getRwpSEIRwpSEIProjRegionTop(i);
    seiRegionWisePacking->m_projRegionLeft[i]                    = m_pcCfg->getRwpSEIProjRegionLeft(i);
    seiRegionWisePacking->m_packedRegionWidth[i]                 = m_pcCfg->getRwpSEIPackedRegionWidth(i);
    seiRegionWisePacking->m_packedRegionHeight[i]                = m_pcCfg->getRwpSEIPackedRegionHeight(i);
    seiRegionWisePacking->m_packedRegionTop[i]                   = m_pcCfg->getRwpSEIPackedRegionTop(i);
    seiRegionWisePacking->m_packedRegionLeft[i]                  = m_pcCfg->getRwpSEIPackedRegionLeft(i);
    if( seiRegionWisePacking->m_rwpGuardBandFlag[i] )
    {
      seiRegionWisePacking->m_rwpLeftGuardBandWidth[i]           =  m_pcCfg->getRwpSEIRwpLeftGuardBandWidth(i);
      seiRegionWisePacking->m_rwpRightGuardBandWidth[i]          =  m_pcCfg->getRwpSEIRwpRightGuardBandWidth(i);
      seiRegionWisePacking->m_rwpTopGuardBandHeight[i]           =  m_pcCfg->getRwpSEIRwpTopGuardBandHeight(i);
      seiRegionWisePacking->m_rwpBottomGuardBandHeight[i]        =  m_pcCfg->getRwpSEIRwpBottomGuardBandHeight(i);
      seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag[i]  =  m_pcCfg->getRwpSEIRwpGuardBandNotUsedForPredFlag(i);
      for( int j=0; j < 4; j++ )
      {
        seiRegionWisePacking->m_rwpGuardBandType[i*4 + j]         =  m_pcCfg->getRwpSEIRwpGuardBandType(i*4 + j);
      }
    }
  }
}

void SEIEncoder::initSEIGcmp(SEIGeneralizedCubemapProjection* seiGeneralizedCubemapProjection)
{
  CHECK_(!(m_isInitialized), "seiGeneralizedCubemapProjection already initialized");
  CHECK_(!(seiGeneralizedCubemapProjection != nullptr), "Need a seiGeneralizedCubemapProjection for initialization (got nullptr)");

  seiGeneralizedCubemapProjection->m_gcmpCancelFlag                      = m_pcCfg->getGcmpSEICancelFlag();
  if (!seiGeneralizedCubemapProjection->m_gcmpCancelFlag)
  {
    seiGeneralizedCubemapProjection->m_gcmpPersistenceFlag               = m_pcCfg->getGcmpSEIPersistenceFlag();
    seiGeneralizedCubemapProjection->m_gcmpPackingType                   = m_pcCfg->getGcmpSEIPackingType();
    seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType           = m_pcCfg->getGcmpSEIMappingFunctionType();

    int numFace = seiGeneralizedCubemapProjection->m_gcmpPackingType == 4 || seiGeneralizedCubemapProjection->m_gcmpPackingType == 5 ? 5 : 6;
    seiGeneralizedCubemapProjection->m_gcmpFaceIndex.resize(numFace);
    seiGeneralizedCubemapProjection->m_gcmpFaceRotation.resize(numFace);
    if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
    {
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag.resize(numFace);
    }
    for (int i = 0; i < numFace; i++)
    {
      seiGeneralizedCubemapProjection->m_gcmpFaceIndex[i]                = m_pcCfg->getGcmpSEIFaceIndex(i);
      seiGeneralizedCubemapProjection->m_gcmpFaceRotation[i]             = m_pcCfg->getGcmpSEIFaceRotation(i);
      if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
      {
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU[i]           = std::max<uint8_t>(1, (uint8_t)(128.0 * m_pcCfg->getGcmpSEIFunctionCoeffU(i) + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag[i] = m_pcCfg->getGcmpSEIFunctionUAffectedByVFlag(i);
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV[i]           = std::max<uint8_t>(1, (uint8_t)(128.0 * m_pcCfg->getGcmpSEIFunctionCoeffV(i) + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag[i] = m_pcCfg->getGcmpSEIFunctionVAffectedByUFlag(i);
      }
    }

    seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag                 = m_pcCfg->getGcmpSEIGuardBandFlag();
    if (seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag)
    {
      seiGeneralizedCubemapProjection->m_gcmpGuardBandType                 = m_pcCfg->getGcmpSEIGuardBandType();
      seiGeneralizedCubemapProjection->m_gcmpGuardBandBoundaryExteriorFlag = m_pcCfg->getGcmpSEIGuardBandBoundaryExteriorFlag();
      seiGeneralizedCubemapProjection->m_gcmpGuardBandSamplesMinus1        = m_pcCfg->getGcmpSEIGuardBandSamplesMinus1();
    }
  }
}

void SEIEncoder::initSEISampleAspectRatioInfo(SEISampleAspectRatioInfo* seiSampleAspectRatioInfo)
{
  CHECK_(!(m_isInitialized), "seiSampleAspectRatioInfo already initialized");
  CHECK_(!(seiSampleAspectRatioInfo != nullptr), "Need a seiSampleAspectRatioInfo for initialization (got nullptr)");

  seiSampleAspectRatioInfo->m_sariCancelFlag = m_pcCfg->getSariCancelFlag();
  if (!seiSampleAspectRatioInfo->m_sariCancelFlag)
  {
    seiSampleAspectRatioInfo->m_sariPersistenceFlag   = m_pcCfg->getSariPersistenceFlag();
    seiSampleAspectRatioInfo->m_sariAspectRatioIdc    = m_pcCfg->getSariAspectRatioIdc();
    if (seiSampleAspectRatioInfo->m_sariAspectRatioIdc == 255)
    {
      seiSampleAspectRatioInfo->m_sariSarWidth   = m_pcCfg->getSariSarWidth();
      seiSampleAspectRatioInfo->m_sariSarHeight  = m_pcCfg->getSariSarHeight();
    }
    else
    {
      seiSampleAspectRatioInfo->m_sariSarWidth   = 0;
      seiSampleAspectRatioInfo->m_sariSarHeight  = 0;
    }
  }
}

//! initialize scalable nesting SEI message.
//! Note: The SEI message structures input into this function will become part of the scalable nesting SEI and will be
//!       automatically freed, when the nesting SEI is disposed.
//  either targetOLS or targetLayer should be active, call with empty vector for the inactive mode
void SEIEncoder::initSEIScalableNesting(SEIScalableNesting *scalableNestingSEI, SEIMessages &nestedSEIs, const std::vector<int> &targetOLSs, const std::vector<int> &targetLayers, const std::vector<uint16_t> &subpictureIDs)
{
  CHECK_(!(m_isInitialized), "Scalable Nesting SEI already initialized ");
  CHECK_(!(scalableNestingSEI != NULL), "No Scalable Nesting SEI object passed");
  CHECK_(targetOLSs.size() > 0 && targetLayers.size() > 0, "Scalable Nesting SEI can apply to either OLS or layer(s), not both");

  scalableNestingSEI->m_snOlsFlag = (targetOLSs.size() > 0) ? 1 : 0;  // If the nested SEI messages are picture buffering SEI messages, picture timing SEI messages or sub-picture timing SEI messages, nesting_ols_flag shall be equal to 1, by default case
  if (scalableNestingSEI->m_snOlsFlag)
  {
    scalableNestingSEI->m_snNumOlssMinus1 =  (uint32_t) targetOLSs.size() - 1;
    // initialize absolute indexes
    for (int i = 0; i <= scalableNestingSEI->m_snNumOlssMinus1; i++)
    {
      scalableNestingSEI->m_snOlsIdx[i] = targetOLSs[i];
    }
    // calculate delta indexes from absolute ones
    for (int i = 0; i <= scalableNestingSEI->m_snNumOlssMinus1; i++)
    {
      if (i == 0)
      {
        CHECK_(scalableNestingSEI->m_snOlsIdx[i] < 0, "OLS indexes must be  equal to or greater than 0");
        // no "-1" operation for the first index although the name implies one
        scalableNestingSEI->m_snOlsIdxDeltaMinus1[i] = scalableNestingSEI->m_snOlsIdx[i];
      }
      else
      {
        CHECK_(scalableNestingSEI->m_snOlsIdx[i] <= scalableNestingSEI->m_snOlsIdx[i - 1], "OLS indexes must be in ascending order");
        scalableNestingSEI->m_snOlsIdxDeltaMinus1[i] = scalableNestingSEI->m_snOlsIdx[i] - scalableNestingSEI->m_snOlsIdx[i - 1] - 1;
      }
    }
  }
  else
  {
    scalableNestingSEI->m_snAllLayersFlag = 0;                          // nesting is not applied to all layers
    scalableNestingSEI->m_snNumLayersMinus1 = (uint32_t) targetLayers.size() - 1;  //nesting_num_layers_minus1
    for (int i=0; i <= scalableNestingSEI->m_snNumLayersMinus1; i++ )
    {
      scalableNestingSEI->m_snLayerId[i] = targetLayers[i];
    }
  }
  if (!subpictureIDs.empty())
  {
    scalableNestingSEI->m_snSubpicFlag = 1;
    scalableNestingSEI->m_snNumSubpics = (uint32_t) subpictureIDs.size();
    scalableNestingSEI->m_snSubpicId   = subpictureIDs;
    scalableNestingSEI->m_snSubpicIdLen = max(1, ceilLog2((*std::max_element(subpictureIDs.begin(), subpictureIDs.end())) + 1));
    CHECK_( scalableNestingSEI->m_snSubpicIdLen > 15, "Subpicture ID too large. Length must be <= 15 bits");
  }
  scalableNestingSEI->m_nestedSEIs.clear();
  for (SEIMessages::iterator it = nestedSEIs.begin(); it != nestedSEIs.end(); it++)
  {
    scalableNestingSEI->m_nestedSEIs.push_back((*it));
  }
}


//! calculate hashes for entire reconstructed picture
void SEIEncoder::initDecodedPictureHashSEI(SEIDecodedPictureHash *decodedPictureHashSEI, PelUnitBuf& pic, std::string &rHashString, const BitDepths &bitDepths)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(decodedPictureHashSEI!=NULL), "Unspecified error");

  decodedPictureHashSEI->method = m_pcCfg->getDecodedPictureHashSEIType();
#if FIX_TICKET_1405
  decodedPictureHashSEI->singleCompFlag = (m_pcCfg->getChromaFormatIdc() == 0);
#endif
  switch (m_pcCfg->getDecodedPictureHashSEIType())
  {
    case HASHTYPE_MD5:
      {
        uint32_t numChar=calcMD5(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
    case HASHTYPE_CRC:
      {
        uint32_t numChar=calcCRC(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
    case HASHTYPE_CHECKSUM:
    default:
      {
        uint32_t numChar=calcChecksum(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
  }
}

void SEIEncoder::initSEIDependentRAPIndication(SEIDependentRAPIndication *seiDependentRAPIndication)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiDependentRAPIndication!=NULL), "Unspecified error");
}


template <typename T>
static void readTokenValue(T            &returnedValue, /// value returned
                           bool         &failed,        /// used and updated
                           std::istream &is,            /// stream to read token from
                           const char  *pToken)        /// token string
{
  returnedValue=T();
  if (failed)
  {
    return;
  }

  int c;
  // Ignore any whitespace
  while ((c=is.get())!=EOF && isspace(c));
  // test for comment mark
  while (c=='#')
  {
    // Ignore to the end of the line
    while ((c=is.get())!=EOF && (c!=10 && c!=13));
    // Ignore any white space at the start of the next line
    while ((c=is.get())!=EOF && isspace(c));
  }
  // test first character of token
  failed=(c!=pToken[0]);
  // test remaining characters of token
  int pos;
  for(pos=1;!failed && pToken[pos]!=0 && is.get()==pToken[pos]; pos++);
  failed|=(pToken[pos]!=0);
  // Ignore any whitespace before the ':'
  while (!failed && (c=is.get())!=EOF && isspace(c));
  failed|=(c!=':');
  // Now read the value associated with the token:
  if (!failed)
  {
    is >> returnedValue;
    failed=!is.good();
    if (!failed)
    {
      c=is.get();
      failed=(c!=EOF && !isspace(c));
    }
  }
  if (failed)
  {
    std::cerr << "Unable to read token '" << pToken << "'\n";
  }
}

template <typename T>
static void readTokenValueAndValidate(T            &returnedValue, /// value returned
                                      bool         &failed,        /// used and updated
                                      std::istream &is,            /// stream to read token from
                                      const char  *pToken,        /// token string
                                      const T      &minInclusive,  /// minimum value allowed, inclusive
                                      const T      &maxInclusive)  /// maximum value allowed, inclusive
{
  readTokenValue(returnedValue, failed, is, pToken);
  if (!failed)
  {
    if (returnedValue<minInclusive || returnedValue>maxInclusive)
    {
      failed=true;
      std::cerr << "Value for token " << pToken << " must be in the range " << minInclusive << " to " << maxInclusive << " (inclusive); value read: " << returnedValue << std::endl;
    }
  }
}

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
void SEIEncoder::initSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics *seiAltTransCharacteristics)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiAltTransCharacteristics!=NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAltTransCharacteristics->m_preferredTransferCharacteristics = m_pcCfg->getSEIPreferredTransferCharacteristics();
}
#endif
void SEIEncoder::initSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics *seiFilmGrain)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiFilmGrain != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiFilmGrain->m_filmGrainCharacteristicsCancelFlag      = m_pcCfg->getFilmGrainCharactersticsSEICancelFlag();
  seiFilmGrain->m_filmGrainCharacteristicsPersistenceFlag = m_pcCfg->getFilmGrainCharactersticsSEIPersistenceFlag();
  seiFilmGrain->m_filmGrainModelId                        = m_pcCfg->getFilmGrainCharactersticsSEIModelID();
  seiFilmGrain->m_separateColourDescriptionPresentFlag    = m_pcCfg->getFilmGrainCharactersticsSEISepColourDescPresent();
  seiFilmGrain->m_blendingModeId                          = m_pcCfg->getFilmGrainCharactersticsSEIBlendingModeID();
  seiFilmGrain->m_log2ScaleFactor                         = m_pcCfg->getFilmGrainCharactersticsSEILog2ScaleFactor();
  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    seiFilmGrain->m_compModel[i].presentFlag = m_pcCfg->getFGCSEICompModelPresent(i);
  }
}

void SEIEncoder::initSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume *seiMDCV)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiMDCV != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  for (int j = 0; j <= 1; j++)
  {
    for (int i = 0; i <= 2; i++)
    {
       seiMDCV->values.primaries[i][j] = m_pcCfg->getMasteringDisplaySEI().primaries[i][j];
    }
    seiMDCV->values.whitePoint[j] = m_pcCfg->getMasteringDisplaySEI().whitePoint[j];
  }
  seiMDCV->values.maxLuminance = m_pcCfg->getMasteringDisplaySEI().maxLuminance;
  seiMDCV->values.minLuminance = m_pcCfg->getMasteringDisplaySEI().minLuminance;
}

void SEIEncoder::initSEIContentLightLevel(SEIContentLightLevelInfo *seiCLL)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiCLL != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiCLL->m_maxContentLightLevel    = m_pcCfg->getCLLSEIMaxContentLightLevel();
  seiCLL->m_maxPicAverageLightLevel = m_pcCfg->getCLLSEIMaxPicAvgLightLevel();
}

void SEIEncoder::initSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment *seiAmbViewEnvironment)
{
  CHECK_(!(m_isInitialized), "Unspecified error");
  CHECK_(!(seiAmbViewEnvironment != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAmbViewEnvironment->m_ambientIlluminance = m_pcCfg->getAmbientViewingEnvironmentSEIIlluminance();
  seiAmbViewEnvironment->m_ambientLightX      = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightX();
  seiAmbViewEnvironment->m_ambientLightY      = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightY();
}

void SEIEncoder::initSEIContentColourVolume(SEIContentColourVolume *seiContentColourVolume)
{
  assert(m_isInitialized);
  assert(seiContentColourVolume != NULL);
  seiContentColourVolume->m_ccvCancelFlag = m_pcCfg->getCcvSEICancelFlag();
  seiContentColourVolume->m_ccvPersistenceFlag = m_pcCfg->getCcvSEIPersistenceFlag();

  seiContentColourVolume->m_ccvPrimariesPresentFlag = m_pcCfg->getCcvSEIPrimariesPresentFlag();
  seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMinLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMaxLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag = m_pcCfg->getCcvSEIAvgLuminanceValuePresentFlag();

  // Currently we are using a floor operation for setting up the "integer" values for this SEI.
  // This applies to both primaries and luminance limits.
  if (seiContentColourVolume->m_ccvPrimariesPresentFlag == true)
  {
    for (int i = 0; i < MAX_NUM_COMPONENT; i++)
    {
      seiContentColourVolume->m_ccvPrimariesX[i] = (int32_t)(50000.0 * m_pcCfg->getCcvSEIPrimariesX(i));
      seiContentColourVolume->m_ccvPrimariesY[i] = (int32_t)(50000.0 * m_pcCfg->getCcvSEIPrimariesY(i));
    }
  }

  if (seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMinLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIMinLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMaxLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIMaxLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvAvgLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIAvgLuminanceValue());
  }
}
void SEIEncoder::initSEISubpictureLevelInfo(SEISubpicureLevelInfo *sei, const SPS *sps)
{
  const EncCfgParam::CfgSEISubpictureLevel &cfgSubPicLevel = m_pcCfg->getSubpicureLevelInfoSEICfg();

  sei->m_sliSublayerInfoPresentFlag = cfgSubPicLevel.m_sliSublayerInfoPresentFlag;
  sei->m_sliMaxSublayers = cfgSubPicLevel.m_sliMaxSublayers;
  sei->m_numRefLevels = cfgSubPicLevel.m_sliSublayerInfoPresentFlag ? (int)cfgSubPicLevel.m_refLevels.size() / cfgSubPicLevel.m_sliMaxSublayers : (int)cfgSubPicLevel.m_refLevels.size();
  sei->m_numSubpics = cfgSubPicLevel.m_numSubpictures;
  sei->m_explicitFractionPresentFlag = cfgSubPicLevel.m_explicitFraction;

  // sei parameters initialization
  sei->m_nonSubpicLayersFraction.resize(sei->m_numRefLevels);
  sei->m_refLevelIdc.resize(sei->m_numRefLevels);
  for (int level = 0; level < sei->m_numRefLevels; level++)
  {
    sei->m_nonSubpicLayersFraction[level].resize(sei->m_sliMaxSublayers);
    sei->m_refLevelIdc[level].resize(sei->m_sliMaxSublayers);
    for (int sublayer = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
    {
      sei->m_refLevelIdc[level][sublayer] = Level::LEVEL15_5;
    }
  }
  if (sei->m_explicitFractionPresentFlag)
  {
    sei->m_refLevelFraction.resize(sei->m_numRefLevels);
    for (int level = 0; level < sei->m_numRefLevels; level++)
    {
      sei->m_refLevelFraction[level].resize(sei->m_numSubpics);
      for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
      {
        sei->m_refLevelFraction[level][subpic].resize(sei->m_sliMaxSublayers);
        for (int sublayer = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
        {
          sei->m_refLevelFraction[level][subpic][sublayer] = 0;
        }
      }
    }
  }

  // set sei parameters according to the configured values
  for (int sublayer = sei->m_sliSublayerInfoPresentFlag ? 0 : sei->m_sliMaxSublayers - 1, cnta = 0, cntb = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
  {
    for (int level = 0; level < sei->m_numRefLevels; level++)
    {
      sei->m_nonSubpicLayersFraction[level][sublayer] = cfgSubPicLevel.m_nonSubpicLayersFraction[cnta];
      sei->m_refLevelIdc[level][sublayer] = cfgSubPicLevel.m_refLevels[cnta++];
      if (sei->m_explicitFractionPresentFlag)
      {
        for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
        {
          sei->m_refLevelFraction[level][subpic][sublayer] = cfgSubPicLevel.m_fractions[cntb++];
        }
      }
    }
  }

  // update the inference of m_refLevelIdc[][] and m_refLevelFraction[][][]
  if (!sei->m_sliSublayerInfoPresentFlag)
  {
    for (int sublayer = sei->m_sliMaxSublayers - 2; sublayer >= 0; sublayer--)
    {
      for (int level = 0; level < sei->m_numRefLevels; level++)
      {
        sei->m_nonSubpicLayersFraction[level][sublayer] = sei->m_nonSubpicLayersFraction[level][sei->m_sliMaxSublayers - 1];
        sei->m_refLevelIdc[level][sublayer] = sei->m_refLevelIdc[level][sei->m_sliMaxSublayers - 1];
        if (sei->m_explicitFractionPresentFlag)
        {
          for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
          {
            sei->m_refLevelFraction[level][subpic][sublayer] = sei->m_refLevelFraction[level][subpic][sei->m_sliMaxSublayers - 1];
          }
        }
      }
    }
  }
}


//! \}

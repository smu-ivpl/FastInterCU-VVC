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

/** \file     EncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>

#include "EncLib.h"
#include "EncGOP.h"
#include "Analyze.h"
#include "libmd5/MD5.h"
#include "CommonLib/SEI.h"
#include "CommonLib/NAL.h"
#include "NALwrite.h"

#include <math.h>
#include <deque>
#include <chrono>
#include <cinttypes>

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/ProfileLevelTier.h"

#include "DecoderLib/DecLib.h"

#define ENCODE_SUB_SET 0

using namespace std;

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
int getLSB(int poc, int maxLSB)
{
  if (poc >= 0)
  {
    return poc % maxLSB;
  }
  else
  {
    return (maxLSB - ((-poc) % maxLSB)) % maxLSB;
  }
}

EncGOP::EncGOP()
{
  m_iLastIDR            = 0;
  m_iGopSize            = 0;
  m_iNumPicCoded        = 0; //Niko
  m_bFirst              = true;
  m_iLastRecoveryPicPOC = 0;
  m_latestDRAPPOC       = MAX_INT;
  m_lastRasPoc          = MAX_INT;

  m_pcCfg               = NULL;
  m_pcSliceEncoder      = NULL;
  m_pcListPic           = NULL;
  m_HLSWriter           = NULL;
  m_bSeqFirst           = true;
  m_audIrapOrGdrAuFlag  = false;

  m_bRefreshPending     = 0;
  m_pocCRA              = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  ::memset(m_lastBPSEI, 0, sizeof(m_lastBPSEI));
  m_rapWithLeading      = false;
  m_bufferingPeriodSEIPresentInAU = false;
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_associatedIRAPType[i] = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }
  ::memset(m_associatedIRAPPOC, 0, sizeof(m_associatedIRAPPOC));
#if W0038_DB_OPT
  m_pcDeblockingTempPicYuv = NULL;
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_ppcFrameOrg             = nullptr;
  m_ppcFrameRec             = nullptr;

  m_pcConvertFormat         = nullptr;
  m_pcConvertIQuantize      = nullptr;
  m_pcColorTransform        = nullptr;
  m_pcDistortionDeltaE      = nullptr;
  m_pcTransferFct           = nullptr;

  m_pcColorTransformParams  = nullptr;
  m_pcFrameFormat           = nullptr;

  m_metricTime = std::chrono::milliseconds(0);
#endif

  m_bInitAMaxBT         = true;
  m_bgPOC = -1;
  m_picBg = NULL;
  m_picOrig = NULL;
  m_isEncodedLTRef = false;
  m_isUseLTRef = false;
  m_isPrepareLTRef = true;
  m_lastLTRefPoc = 0;
}

EncGOP::~EncGOP()
{
  if( !m_pcCfg->getDecodeBitstream(0).empty() || !m_pcCfg->getDecodeBitstream(1).empty() )
  {
    // reset potential decoder resources
    tryDecodePicture( NULL, 0, std::string("") );
  }
#if JVET_O0756_CALCULATE_HDRMETRICS
  delete [] m_ppcFrameOrg;
  delete [] m_ppcFrameRec;

  m_ppcFrameOrg = m_ppcFrameRec = nullptr;

  delete m_pcConvertFormat;
  delete m_pcConvertIQuantize;
  delete m_pcColorTransform;
  delete m_pcDistortionDeltaE;
  delete m_pcTransferFct;
  delete m_pcColorTransformParams;
  delete m_pcFrameFormat;

  m_pcConvertFormat         = nullptr;
  m_pcConvertIQuantize      = nullptr;
  m_pcColorTransform        = nullptr;
  m_pcDistortionDeltaE      = nullptr;
  m_pcTransferFct           = nullptr;
  m_pcColorTransformParams  = nullptr;
  m_pcFrameFormat           = nullptr;
#endif
}

/** Create list to contain pointers to CTU start addresses of slice.
 */
void  EncGOP::create()
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;
}

void  EncGOP::destroy()
{
#if W0038_DB_OPT
  if (m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv->destroy();
    delete m_pcDeblockingTempPicYuv;
    m_pcDeblockingTempPicYuv = NULL;
  }
#endif
  if (m_picBg)
  {
    m_picBg->destroy();
    delete m_picBg;
    m_picBg = NULL;
  }
  if (m_picOrig)
  {
    m_picOrig->destroy();
    delete m_picOrig;
    m_picOrig = NULL;
  }
}

void EncGOP::init ( EncLib* pcEncLib )
{
  m_pcEncLib     = pcEncLib;
  m_pcCfg                = pcEncLib;
  m_seiEncoder.init(m_pcCfg, pcEncLib, this);
  m_pcSliceEncoder       = pcEncLib->getSliceEncoder();
  m_pcListPic            = pcEncLib->getListPic();
  m_HLSWriter            = pcEncLib->getHLSWriter();
  m_pcLoopFilter         = pcEncLib->getLoopFilter();
  m_pcSAO                = pcEncLib->getSAO();
  m_pcALF                = pcEncLib->getALF();
  m_pcRateCtrl           = pcEncLib->getRateCtrl();
  ::memset(m_lastBPSEI, 0, sizeof(m_lastBPSEI));
  ::memset(m_totalCoded, 0, sizeof(m_totalCoded));
  m_HRD                = pcEncLib->getHRD();

  m_AUWriterIf = pcEncLib->getAUWriterIf();

#if WCG_EXT
  if (m_pcCfg->getLmcs())
  {
    pcEncLib->getRdCost()->setReshapeInfo(m_pcCfg->getReshapeSignalType(), m_pcCfg->getBitDepth(CHANNEL_TYPE_LUMA));
    pcEncLib->getRdCost()->initLumaLevelToWeightTableReshape();
  }
  else if (m_pcCfg->getLumaLevelToDeltaQPMapping().mode)
  {
    pcEncLib->getRdCost()->setReshapeInfo(RESHAPE_SIGNAL_PQ, m_pcCfg->getBitDepth(CHANNEL_TYPE_LUMA));
    pcEncLib->getRdCost()->initLumaLevelToWeightTableReshape();
  }
  pcEncLib->getALF()->getLumaLevelWeightTable() = pcEncLib->getRdCost()->getLumaLevelWeightTable();
  int alfWSSD = 0;
  if (m_pcCfg->getLmcs() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ )
  {
    alfWSSD = 1;
  }
  pcEncLib->getALF()->setAlfWSSD(alfWSSD);
#endif
  m_pcReshaper = pcEncLib->getReshaper();

#if JVET_O0756_CALCULATE_HDRMETRICS
  const bool calculateHdrMetrics = m_pcEncLib->getCalcluateHdrMetrics();
  if(calculateHdrMetrics)
  {
    //allocate frame buffers and initialize class members
    int chainNumber = 5;

    m_ppcFrameOrg = new hdrtoolslib::Frame* [chainNumber];
    m_ppcFrameRec = new hdrtoolslib::Frame* [chainNumber];

    double* whitePointDeltaE = new double[hdrtoolslib::NB_REF_WHITE];
    for (int i=0; i<hdrtoolslib::NB_REF_WHITE; i++)
    {
      whitePointDeltaE[i] = m_pcCfg->getWhitePointDeltaE(i);
    }

    double maxSampleValue                       = m_pcCfg->getMaxSampleValue();
    hdrtoolslib::SampleRange sampleRange        = m_pcCfg->getSampleRange();
    hdrtoolslib::ChromaFormat chFmt             = hdrtoolslib::ChromaFormat(m_pcCfg->getChromaFormatIdc());
    int bitDepth = m_pcCfg->getBitDepth(CHANNEL_TYPE_LUMA);
    hdrtoolslib::ColorPrimaries colorPrimaries  = m_pcCfg->getColorPrimaries();
    bool enableTFunctionLUT                     = m_pcCfg->getEnableTFunctionLUT();
    hdrtoolslib::ChromaLocation* chromaLocation = new hdrtoolslib::ChromaLocation[2];
    for (int i=0; i<2; i++)
    {
      chromaLocation[i] = m_pcCfg->getChromaLocation(i);
    }
    int chromaUpFilter  = m_pcCfg->getChromaUPFilter();
    int cropOffsetLeft   = m_pcCfg->getCropOffsetLeft();
    int cropOffsetTop    = m_pcCfg->getCropOffsetTop();
    int cropOffsetRight  = m_pcCfg->getCropOffsetRight();
    int cropOffsetBottom = m_pcCfg->getCropOffsetBottom();

    int width  = m_pcCfg->getSourceWidth() - cropOffsetLeft + cropOffsetRight;
    int height = m_pcCfg->getSourceHeight() - cropOffsetTop  + cropOffsetBottom;

    m_ppcFrameOrg[0] = new hdrtoolslib::Frame(width, height, false, hdrtoolslib::CM_YCbCr, colorPrimaries, chFmt, sampleRange, bitDepth, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[0] = new hdrtoolslib::Frame(width, height, false, hdrtoolslib::CM_YCbCr, colorPrimaries, chFmt, sampleRange, bitDepth, false, hdrtoolslib::TF_PQ, 0);

    m_ppcFrameOrg[1] = new hdrtoolslib::Frame(m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], false, hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444, sampleRange, bitDepth, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[1] = new hdrtoolslib::Frame(m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], false, hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444, sampleRange, bitDepth, false, hdrtoolslib::TF_PQ, 0);                                // 420 to 444 conversion

    m_ppcFrameOrg[2] =  new hdrtoolslib::Frame(m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], true, hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[2] =  new hdrtoolslib::Frame(m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], true, hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_PQ, 0);                                // 444 to Float conversion

    m_ppcFrameOrg[3] = new hdrtoolslib::Frame(m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], true, hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[3] = new hdrtoolslib::Frame(m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], true, hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_PQ, 0);                                // YCbCr to RGB conversion

    m_ppcFrameOrg[4] = new hdrtoolslib::Frame(m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], true, hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_NULL, 0);
    m_ppcFrameRec[4] = new hdrtoolslib::Frame(m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], true, hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_NULL, 0);                                // Inverse Transfer Function

    m_pcFrameFormat                   = new hdrtoolslib::FrameFormat();
    m_pcFrameFormat->m_isFloat        = true;
    m_pcFrameFormat->m_chromaFormat   = hdrtoolslib::CF_UNKNOWN;
    m_pcFrameFormat->m_colorSpace     = hdrtoolslib::CM_RGB;
    m_pcFrameFormat->m_colorPrimaries = hdrtoolslib::CP_2020;
    m_pcFrameFormat->m_sampleRange    = hdrtoolslib::SR_UNKNOWN;

    m_pcConvertFormat     = hdrtoolslib::ConvertColorFormat::create(width, height, chFmt, hdrtoolslib::CF_444, chromaUpFilter, chromaLocation, chromaLocation);
    m_pcConvertIQuantize  = hdrtoolslib::Convert::create(&m_ppcFrameOrg[1]->m_format, &m_ppcFrameOrg[2]->m_format);
    m_pcColorTransform    = hdrtoolslib::ColorTransform::create(m_ppcFrameOrg[2]->m_colorSpace, m_ppcFrameOrg[2]->m_colorPrimaries, m_ppcFrameOrg[3]->m_colorSpace, m_ppcFrameOrg[3]->m_colorPrimaries, true, 1);
    m_pcDistortionDeltaE  = new hdrtoolslib::DistortionMetricDeltaE(m_pcFrameFormat, false, maxSampleValue, whitePointDeltaE, 1);
    m_pcTransferFct       = hdrtoolslib::TransferFunction::create(hdrtoolslib::TF_PQ, true, (float) maxSampleValue, 0, 0.0, 1.0, enableTFunctionLUT);
  }
#endif
}

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
int EncGOP::xWriteOPI (AccessUnit &accessUnit, const OPI *opi)
{
  OutputNALUnit nalu(NAL_UNIT_OPI);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  CHECK_( nalu.m_temporalId, "The value of TemporalId of OPI NAL units shall be equal to 0" );
  m_HLSWriter->codeOPI( opi );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}
#endif

int EncGOP::xWriteVPS (AccessUnit &accessUnit, const VPS *vps)
{
  OutputNALUnit nalu(NAL_UNIT_VPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  CHECK_( nalu.m_temporalId, "The value of TemporalId of VPS NAL units shall be equal to 0" );
  m_HLSWriter->codeVPS( vps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}
int EncGOP::xWriteDCI(AccessUnit& accessUnit, const DCI* dci)
{
  OutputNALUnit nalu(NAL_UNIT_DCI);
  m_HLSWriter->setBitstream(&nalu.m_Bitstream);
  CHECK_(nalu.m_temporalId, "The value of TemporalId of DCI NAL units shall be equal to 0");
  m_HLSWriter->codeDCI(dci);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteSPS( AccessUnit &accessUnit, const SPS *sps, const int layerId )
{
  OutputNALUnit nalu(NAL_UNIT_SPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  nalu.m_nuhLayerId = layerId;
  CHECK_( nalu.m_temporalId, "The value of TemporalId of SPS NAL units shall be equal to 0" );
  m_HLSWriter->codeSPS( sps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;

}

int EncGOP::xWritePPS( AccessUnit &accessUnit, const PPS *pps, const int layerId )
{
  OutputNALUnit nalu(NAL_UNIT_PPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  nalu.m_nuhLayerId = layerId;
  CHECK_( nalu.m_temporalId < accessUnit.temporalId, "TemporalId shall be greater than or equal to the TemporalId of the layer access unit containing the NAL unit" );
  m_HLSWriter->codePPS( pps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteAPS( AccessUnit &accessUnit, APS *aps, const int layerId, const bool isPrefixNUT )
{
  OutputNALUnit nalu( isPrefixNUT ? NAL_UNIT_PREFIX_APS : NAL_UNIT_SUFFIX_APS );
  m_HLSWriter->setBitstream(&nalu.m_Bitstream);
  nalu.m_nuhLayerId = layerId;
  nalu.m_temporalId = aps->getTemporalId();
  aps->setLayerId( layerId );
  CHECK_( nalu.m_temporalId < accessUnit.temporalId, "TemporalId shall be greater than or equal to the TemporalId of the layer access unit containing the NAL unit" );
  m_HLSWriter->codeAPS(aps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteParameterSets(AccessUnit &accessUnit, Slice *slice, const bool bSeqFirst, const int layerIdx)
{
  int actualTotalBits = 0;

  if( bSeqFirst )
  {
    if (layerIdx == 0)
    {
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
      if (m_pcCfg->getOPIEnabled())
      {
        actualTotalBits += xWriteOPI(accessUnit, m_pcEncLib->getOPI());
      }
#endif
      if (m_pcCfg->getDCIEnabled())
      {
        actualTotalBits += xWriteDCI(accessUnit, m_pcEncLib->getDCI());
      }
      if (slice->getSPS()->getVPSId() != 0)
      {
        actualTotalBits += xWriteVPS(accessUnit, m_pcEncLib->getVPS());
      }
    }
    if( m_pcEncLib->SPSNeedsWriting( slice->getSPS()->getSPSId() ) ) // Note this assumes that all changes to the SPS are made at the EncLib level prior to picture creation (EncLib::xGetNewPicBuffer).
    {
      CHECK_( !( bSeqFirst ), "Unspecified error" ); // Implementations that use more than 1 SPS need to be aware of activation issues.
      actualTotalBits += xWriteSPS( accessUnit, slice->getSPS(), m_pcEncLib->getLayerId() );
    }
  }

  if( m_pcEncLib->PPSNeedsWriting( slice->getPPS()->getPPSId() ) ) // Note this assumes that all changes to the PPS are made at the EncLib level prior to picture creation (EncLib::xGetNewPicBuffer).
  {
    actualTotalBits += xWritePPS( accessUnit, slice->getPPS(), m_pcEncLib->getLayerId() );
  }

  return actualTotalBits;
}

int EncGOP::xWritePicHeader( AccessUnit &accessUnit, PicHeader *picHeader )
{
  OutputNALUnit nalu(NAL_UNIT_PH);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  nalu.m_temporalId = accessUnit.temporalId;
  nalu.m_nuhLayerId = m_pcEncLib->getLayerId();
  m_HLSWriter->codePictureHeader( picHeader, true );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

void EncGOP::xWriteAccessUnitDelimiter (AccessUnit &accessUnit, Slice *slice)
{
  AUDWriter audWriter;
  OutputNALUnit nalu(NAL_UNIT_ACCESS_UNIT_DELIMITER);
  nalu.m_temporalId = slice->getTLayer();
  int vpsId = slice->getSPS()->getVPSId();
  if (vpsId == 0)
  {
    nalu.m_nuhLayerId = 0;
  }
  else
  {
    nalu.m_nuhLayerId = slice->getVPS()->getLayerId(0);
  }
  CHECK_( nalu.m_temporalId != accessUnit.temporalId, "TemporalId shall be equal to the TemporalId of the AU containing the NAL unit" );
  int picType = slice->isIntra() ? 0 : (slice->isInterP() ? 1 : 2);
  audWriter.codeAUD(nalu.m_Bitstream, m_audIrapOrGdrAuFlag, picType);
  accessUnit.push_front(new NALUnitEBSP(nalu));
}

void EncGOP::xWriteFillerData (AccessUnit &accessUnit, Slice *slice, uint32_t &fdSize)
{
  FDWriter fdWriter;
  OutputNALUnit nalu(NAL_UNIT_FD);
  nalu.m_temporalId = slice->getTLayer();
  int vpsId = slice->getSPS()->getVPSId();
  if (vpsId == 0)
  {
    nalu.m_nuhLayerId = 0;
  }
  else
  {
    nalu.m_nuhLayerId = slice->getVPS()->getLayerId(0);
  }
  CHECK_( nalu.m_temporalId != accessUnit.temporalId, "TemporalId shall be equal to the TemporalId of the AU containing the NAL unit" );
  fdWriter.codeFD(nalu.m_Bitstream, fdSize);
  accessUnit.push_back(new NALUnitEBSP(nalu));
}

// write SEI list into one NAL unit and add it to the Access unit at auPos
void EncGOP::xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  OutputNALUnit nalu( naluType, m_pcEncLib->getLayerId(), temporalId );
  m_seiWriter.writeSEImessages(nalu.m_Bitstream, seiMessages, *m_HRD, false, temporalId);
  auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
  auPos++;
}

void EncGOP::xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  for (SEIMessages::const_iterator sei = seiMessages.begin(); sei!=seiMessages.end(); sei++ )
  {
    SEIMessages tmpMessages;
    tmpMessages.push_back(*sei);
    OutputNALUnit nalu( naluType, m_pcEncLib->getLayerId(), temporalId );
    m_seiWriter.writeSEImessages(nalu.m_Bitstream, tmpMessages, *m_HRD, false, temporalId);
    auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
    auPos++;
  }
}

void EncGOP::xClearSEIs(SEIMessages& seiMessages, bool deleteMessages)
{
  if (deleteMessages)
  {
    deleteSEIs(seiMessages);
  }
  else
  {
    seiMessages.clear();
  }
}

// write SEI messages as separate NAL units ordered
void EncGOP::xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, bool testWrite)
{
  AccessUnit::iterator itNalu = accessUnit.begin();

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  while ((itNalu != accessUnit.end()) &&
    ((*itNalu)->m_nalUnitType == NAL_UNIT_ACCESS_UNIT_DELIMITER
      || (*itNalu)->m_nalUnitType == NAL_UNIT_OPI
      || (*itNalu)->m_nalUnitType == NAL_UNIT_VPS
      || (*itNalu)->m_nalUnitType == NAL_UNIT_DCI
      || (*itNalu)->m_nalUnitType == NAL_UNIT_SPS
      || (*itNalu)->m_nalUnitType == NAL_UNIT_PPS
      ))
#else
  while ((itNalu != accessUnit.end()) &&
    ((*itNalu)->m_nalUnitType == NAL_UNIT_ACCESS_UNIT_DELIMITER
      || (*itNalu)->m_nalUnitType == NAL_UNIT_VPS
      || (*itNalu)->m_nalUnitType == NAL_UNIT_DCI
      || (*itNalu)->m_nalUnitType == NAL_UNIT_SPS
      || (*itNalu)->m_nalUnitType == NAL_UNIT_PPS
      ))
#endif
  {
    itNalu++;
  }

  SEIMessages localMessages = seiMessages;
  SEIMessages currentMessages;

#if ENABLE_TRACING
  g_HLSTraceEnable = !testWrite;
#endif
  // The case that a specific SEI is not present is handled in xWriteSEI (empty list)


  // Buffering period SEI must always be following active parameter sets
  currentMessages = extractSeisByType(localMessages, SEI::BUFFERING_PERIOD);
  CHECK_(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
  xClearSEIs(currentMessages, !testWrite);

  // Picture timing SEI must always be following buffering period
  // Note: When general_same_pic_timing_in_all_ols_flag is equal to 1, PT SEI messages are required
  //       to be placed into separate NAL units. The code below conforms to the constraint even if
  //       general_same_pic_timing_in_all_ols_flag is equal to 0
  currentMessages = extractSeisByType(localMessages, SEI::PICTURE_TIMING);
  CHECK_(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
  xClearSEIs(currentMessages, !testWrite);

  // Decoding unit info SEI must always be following picture timing
  if (!duInfoSeiMessages.empty())
  {
    currentMessages.push_back(duInfoSeiMessages.front());
    if (!testWrite)
    {
      duInfoSeiMessages.pop_front();
    }
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
    xClearSEIs(currentMessages, !testWrite);
  }

  if (m_pcCfg->getScalableNestingSEIEnabled())
  {
    // Scalable nesting SEI must always be the following DU info
    currentMessages = extractSeisByType(localMessages, SEI::SCALABLE_NESTING);
    xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
    xClearSEIs(currentMessages, !testWrite);
  }


  // And finally everything else one by one
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, localMessages, accessUnit, itNalu, temporalId);
  xClearSEIs(localMessages, !testWrite);

  if (!testWrite)
  {
    seiMessages.clear();
  }
}


void EncGOP::xWriteLeadingSEIMessages (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, const SPS *sps, std::deque<DUData> &duData)
{
  AccessUnit testAU;
  SEIMessages picTimingSEIs = getSeisByType(seiMessages, SEI::PICTURE_TIMING);
  CHECK_(!(picTimingSEIs.size() < 2), "Unspecified error");
  SEIPictureTiming * picTiming = picTimingSEIs.empty() ? NULL : (SEIPictureTiming*) picTimingSEIs.front();

  // test writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, testAU, temporalId, true);
  // update Timing and DU info SEI
  xUpdateDuData(testAU, duData);
  xUpdateTimingSEI(picTiming, duData, sps);
  xUpdateDuInfoSEI(duInfoSeiMessages, picTiming, sps->getMaxTLayers());
  // actual writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, accessUnit, temporalId, false);

  // testAU will automatically be cleaned up when losing scope
}

void EncGOP::xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, int temporalId)
{
  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB NAL units
  AccessUnit::iterator pos = accessUnit.end();
  xWriteSEISeparately(NAL_UNIT_SUFFIX_SEI, seiMessages, accessUnit, pos, temporalId);
  deleteSEIs(seiMessages);
}

void EncGOP::xWriteDuSEIMessages (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, std::deque<DUData> &duData)
{
  if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && m_HRD->getBufferingPeriodSEI()->m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    int naluIdx = 0;
    AccessUnit::iterator nalu = accessUnit.begin();

    // skip over first DU, we have a DU info SEI there already
    while (naluIdx < duData[0].accumNalsDU && nalu!=accessUnit.end())
    {
      naluIdx++;
      nalu++;
    }

    SEIMessages::iterator duSEI = duInfoSeiMessages.begin();
    // loop over remaining DUs
    for (int duIdx = 1; duIdx < duData.size(); duIdx++)
    {
      if (duSEI == duInfoSeiMessages.end())
      {
        // if the number of generated SEIs matches the number of DUs, this should not happen
        CHECK_(!(false), "Unspecified error");
        return;
      }
      // write the next SEI
      SEIMessages tmpSEI;
      tmpSEI.push_back(*duSEI);
      xWriteSEI(NAL_UNIT_PREFIX_SEI, tmpSEI, accessUnit, nalu, temporalId);
      // nalu points to the position after the SEI, so we have to increase the index as well
      naluIdx++;
      while ((naluIdx < duData[duIdx].accumNalsDU) && nalu!=accessUnit.end())
      {
        naluIdx++;
        nalu++;
      }
      duSEI++;
    }
  }
  deleteSEIs(duInfoSeiMessages);
}


void EncGOP::xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const SPS *sps, const PPS *pps)
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if(m_pcCfg->getFramePackingArrangementSEIEnabled())
  {
    SEIFramePacking *sei = new SEIFramePacking;
    m_seiEncoder.initSEIFramePacking (sei, m_iNumPicCoded);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getParameterSetsInclusionIndicationSEIEnabled())
  {
    SEIParameterSetsInclusionIndication* sei = new SEIParameterSetsInclusionIndication;
    m_seiEncoder.initSEIParameterSetsInclusionIndication(sei);
    seiMessages.push_back(sei);
  }

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  if(m_pcCfg->getSEIAlternativeTransferCharacteristicsSEIEnable())
  {
    SEIAlternativeTransferCharacteristics *seiAlternativeTransferCharacteristics = new SEIAlternativeTransferCharacteristics;
    m_seiEncoder.initSEIAlternativeTransferCharacteristics(seiAlternativeTransferCharacteristics);
    seiMessages.push_back(seiAlternativeTransferCharacteristics);
  }
#endif
  if (m_pcCfg->getErpSEIEnabled())
  {
    SEIEquirectangularProjection *sei = new SEIEquirectangularProjection;
    m_seiEncoder.initSEIErp(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getSphereRotationSEIEnabled())
  {
    SEISphereRotation *sei = new SEISphereRotation;
    m_seiEncoder.initSEISphereRotation(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getOmniViewportSEIEnabled())
  {
    SEIOmniViewport *sei = new SEIOmniViewport;
    m_seiEncoder.initSEIOmniViewport(sei);
    seiMessages.push_back(sei);
  }
  if (m_pcCfg->getRwpSEIEnabled())
  {
    SEIRegionWisePacking *seiRegionWisePacking = new SEIRegionWisePacking;
    m_seiEncoder.initSEIRegionWisePacking(seiRegionWisePacking);
    seiMessages.push_back(seiRegionWisePacking);
  }
  if (m_pcCfg->getGcmpSEIEnabled())
  {
    SEIGeneralizedCubemapProjection *sei = new SEIGeneralizedCubemapProjection;
    m_seiEncoder.initSEIGcmp(sei);
    seiMessages.push_back(sei);
  }
  if (m_pcCfg->getSubpicureLevelInfoSEICfg().m_enabled)
  {
    SEISubpicureLevelInfo *seiSubpicureLevelInfo = new SEISubpicureLevelInfo;
    m_seiEncoder.initSEISubpictureLevelInfo(seiSubpicureLevelInfo, sps);
    seiMessages.push_back(seiSubpicureLevelInfo);
  }
  if (m_pcCfg->getSampleAspectRatioInfoSEIEnabled())
  {
    SEISampleAspectRatioInfo *seiSampleAspectRatioInfo = new SEISampleAspectRatioInfo;
    m_seiEncoder.initSEISampleAspectRatioInfo(seiSampleAspectRatioInfo);
    seiMessages.push_back(seiSampleAspectRatioInfo);
  }
  // film grain
  if (m_pcCfg->getFilmGrainCharactersticsSEIEnabled())
  {
    SEIFilmGrainCharacteristics *sei = new SEIFilmGrainCharacteristics;
    m_seiEncoder.initSEIFilmGrainCharacteristics(sei);
    seiMessages.push_back(sei);
  }

  // mastering display colour volume
  if (m_pcCfg->getMasteringDisplaySEI().colourVolumeSEIEnabled)
  {
    SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
    m_seiEncoder.initSEIMasteringDisplayColourVolume(sei);
    seiMessages.push_back(sei);
  }

  // content light level
  if (m_pcCfg->getCLLSEIEnabled())
  {
    SEIContentLightLevelInfo *seiCLL = new SEIContentLightLevelInfo;
    m_seiEncoder.initSEIContentLightLevel(seiCLL);
    seiMessages.push_back(seiCLL);
  }

  // ambient viewing environment
  if (m_pcCfg->getAmbientViewingEnvironmentSEIEnabled())
  {
    SEIAmbientViewingEnvironment *seiAVE = new SEIAmbientViewingEnvironment;
    m_seiEncoder.initSEIAmbientViewingEnvironment(seiAVE);
    seiMessages.push_back(seiAVE);
  }

  // content colour volume
  if (m_pcCfg->getCcvSEIEnabled())
  {
    SEIContentColourVolume *seiContentColourVolume = new SEIContentColourVolume;
    m_seiEncoder.initSEIContentColourVolume(seiContentColourVolume);
    seiMessages.push_back(seiContentColourVolume);
  }
}

void EncGOP::xCreatePerPictureSEIMessages (int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, Slice *slice)
{
  if ((m_pcCfg->getBufferingPeriodSEIEnabled()) && (slice->isIRAP() || slice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) &&
    slice->getNalUnitLayerId()==slice->getVPS()->getLayerId(0) &&
  (slice->getSPS()->getGeneralHrdParametersPresentFlag()))
  {
    SEIBufferingPeriod *bufferingPeriodSEI = new SEIBufferingPeriod();
    bool noLeadingPictures = ( (slice->getNalUnitType()!= NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (slice->getNalUnitType()!= NAL_UNIT_CODED_SLICE_CRA) )?(true):(false);
    m_seiEncoder.initSEIBufferingPeriod(bufferingPeriodSEI,noLeadingPictures);
    m_HRD->setBufferingPeriodSEI(bufferingPeriodSEI);
    seiMessages.push_back(bufferingPeriodSEI);
    m_bufferingPeriodSEIPresentInAU = true;

    if (m_pcCfg->getScalableNestingSEIEnabled())
    {
      SEIBufferingPeriod *bufferingPeriodSEIcopy = new SEIBufferingPeriod();
      bufferingPeriodSEI->copyTo(*bufferingPeriodSEIcopy);
      nestedSeiMessages.push_back(bufferingPeriodSEIcopy);
    }

  }

  if (m_pcEncLib->getDependentRAPIndicationSEIEnabled() && slice->isDRAP())
  {
    SEIDependentRAPIndication *dependentRAPIndicationSEI = new SEIDependentRAPIndication();
    m_seiEncoder.initSEIDependentRAPIndication(dependentRAPIndicationSEI);
    seiMessages.push_back(dependentRAPIndicationSEI);
  }

}

void EncGOP::xCreateScalableNestingSEI(SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, const std::vector<int> &targetOLSs, const std::vector<int> &targetLayers, const std::vector<uint16_t>& subpicIDs)
{
  SEIMessages tmpMessages;
  while (!nestedSeiMessages.empty())
  {
    SEI* sei = nestedSeiMessages.front();
    nestedSeiMessages.pop_front();
    tmpMessages.push_back(sei);
    SEIScalableNesting *nestingSEI = new SEIScalableNesting();
    m_seiEncoder.initSEIScalableNesting(nestingSEI, tmpMessages, targetOLSs, targetLayers, subpicIDs);
    seiMessages.push_back(nestingSEI);
    tmpMessages.clear();
  }
}


void EncGOP::xCreateFrameFieldInfoSEI  (SEIMessages& seiMessages, Slice *slice, bool isField)
{
  if (m_pcCfg->getFrameFieldInfoSEIEnabled())
  {
    SEIFrameFieldInfo *frameFieldInfoSEI = new SEIFrameFieldInfo();

    // encode only very basic information. if more feature are supported, this should be moved to SEIEncoder
    frameFieldInfoSEI->m_fieldPicFlag = isField;
    if (isField)
    {
      frameFieldInfoSEI->m_bottomFieldFlag = !slice->getPic()->topField;
    }
    seiMessages.push_back(frameFieldInfoSEI);
  }
}


void EncGOP::xCreatePictureTimingSEI  (int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, Slice *slice, bool isField, std::deque<DUData> &duData)
{
  // Picture timing depends on buffering period. When either of those is not disabled,
  // initialization would fail. Needs more cleanup after DU timing is integrated.
  if (!(m_pcCfg->getPictureTimingSEIEnabled() && m_pcCfg->getBufferingPeriodSEIEnabled()))
  {
    return;
  }

  const GeneralHrdParams *hrd = slice->getSPS()->getGeneralHrdParameters();

  // update decoding unit parameters
  if ((m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled()) && slice->getNalUnitLayerId() == slice->getVPS()->getLayerId(0))
  {
    int picSptDpbOutputDuDelay = 0;
    SEIPictureTiming *pictureTimingSEI = new SEIPictureTiming();

    // DU parameters
    if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
    {
      uint32_t numDU = (uint32_t) duData.size();
      pictureTimingSEI->m_numDecodingUnitsMinus1     = ( numDU - 1 );
      pictureTimingSEI->m_duCommonCpbRemovalDelayFlag = false;
      pictureTimingSEI->m_numNalusInDuMinus1.resize( numDU );
      const uint32_t maxNumSubLayers = slice->getSPS()->getMaxTLayers();
      pictureTimingSEI->m_duCpbRemovalDelayMinus1.resize( numDU * maxNumSubLayers );
    }
    const uint32_t cpbRemovalDelayLegth = m_HRD->getBufferingPeriodSEI()->m_cpbRemovalDelayLength;
    const uint32_t maxNumSubLayers = slice->getSPS()->getMaxTLayers();
    pictureTimingSEI->m_auCpbRemovalDelay[maxNumSubLayers-1] = std::min<int>(std::max<int>(1, m_totalCoded[maxNumSubLayers-1] - m_lastBPSEI[maxNumSubLayers-1]), static_cast<int>(pow(2, static_cast<double>(cpbRemovalDelayLegth)))); // Syntax element signalled as minus, hence the .
    CHECK_( (m_totalCoded[maxNumSubLayers-1] - m_lastBPSEI[maxNumSubLayers-1]) > pow(2, static_cast<double>(cpbRemovalDelayLegth)), " cpbRemovalDelayLegth too small for m_auCpbRemovalDelay[pt_max_sub_layers_minus1] at picture timing SEI " );
    const uint32_t temporalId = slice->getTLayer();
    if (maxNumSubLayers == 1)
    {
      pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[0] = true;
    }
    for( int i = temporalId ; i < maxNumSubLayers - 1 ; i ++ )
    {
      int indexWithinGOP = (m_totalCoded[maxNumSubLayers - 1] - m_lastBPSEI[maxNumSubLayers - 1]) % m_pcCfg->getGOPSize();
      pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[i] = true;
      if( ((m_rapWithLeading == true) && (indexWithinGOP == 0)) || (m_totalCoded[maxNumSubLayers - 1] == 0) || m_bufferingPeriodSEIPresentInAU || (slice->getPOC() + m_pcCfg->getGOPSize()) > m_pcCfg->getFramesToBeEncoded() )
      {
        pictureTimingSEI->m_cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      else
      {
        pictureTimingSEI->m_cpbRemovalDelayDeltaEnabledFlag[i] = m_HRD->getBufferingPeriodSEI()->m_cpbRemovalDelayDeltasPresentFlag;
      }
      if( pictureTimingSEI->m_cpbRemovalDelayDeltaEnabledFlag[i] )
      {
        if( m_rapWithLeading == false )
        {
          switch (m_pcCfg->getGOPSize())
          {
            case 8:
            {
              if((indexWithinGOP == 1 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if((indexWithinGOP == 2 && i == 2) || (indexWithinGOP == 6 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if((indexWithinGOP == 1 && i == 1) || (indexWithinGOP == 3 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if(indexWithinGOP == 2 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if(indexWithinGOP == 1 && i == 0)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            case 16:
            {
              if((indexWithinGOP == 1 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if((indexWithinGOP == 2 && i == 3) || (indexWithinGOP == 10 && i == 3) || (indexWithinGOP == 14 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 3 && i == 3) || (indexWithinGOP == 7 && i == 3) || (indexWithinGOP == 11 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if(indexWithinGOP == 4 && i == 3)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if((indexWithinGOP == 2 && i == 2) || (indexWithinGOP == 10 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else if(indexWithinGOP == 1 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 5;
              }
              else if(indexWithinGOP == 3 && i == 2)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 6;
              }
              else if(indexWithinGOP == 2 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 7;
              }
              else if(indexWithinGOP == 1 && i == 0)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 8;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            default:
            {
              THROW("m_cpbRemovalDelayDeltaIdx not supported for the current GOP size");
            }
              break;
          }
        }
        else
        {
          switch (m_pcCfg->getGOPSize())
          {
            case 8:
            {
              if((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 5 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if(indexWithinGOP == 2 && i == 2)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if(indexWithinGOP == 1 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            case 16:
            {
              if((indexWithinGOP == 1 && i == 3) || (indexWithinGOP == 9 && i == 3) || (indexWithinGOP == 13 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if((indexWithinGOP == 2 && i == 3) || (indexWithinGOP == 6 && i == 3) || (indexWithinGOP == 10 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 9 && i == 2) || (indexWithinGOP == 3 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if(indexWithinGOP == 2 && i == 2)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if(indexWithinGOP == 1 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            default:
            {
              THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
            }
              break;
          }
        }
      }
      else
      {
        int scaledDistToBuffPeriod = (m_totalCoded[i] - m_lastBPSEI[i]) * static_cast<int>(pow(2, static_cast<double>(maxNumSubLayers - 1 - i)));
        pictureTimingSEI->m_auCpbRemovalDelay[i] = std::min<int>(std::max<int>(1, scaledDistToBuffPeriod), static_cast<int>(pow(2, static_cast<double>(cpbRemovalDelayLegth)))); // Syntax element signalled as minus, hence the .
        CHECK_( (scaledDistToBuffPeriod) > pow(2, static_cast<double>(cpbRemovalDelayLegth)), " cpbRemovalDelayLegth too small for m_auCpbRemovalDelay[i] at picture timing SEI " );
      }
    }
    pictureTimingSEI->m_picDpbOutputDelay = slice->getSPS()->getMaxNumReorderPics(slice->getSPS()->getMaxTLayers()-1) + slice->getPOC() - m_totalCoded[maxNumSubLayers-1];
    if(m_pcCfg->getEfficientFieldIRAPEnabled() && IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
    {
      // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
      pictureTimingSEI->m_picDpbOutputDelay ++;
    }
    int factor = hrd->getTickDivisorMinus2() + 2;
    pictureTimingSEI->m_picDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
    {
      picSptDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    }
    if (m_bufferingPeriodSEIPresentInAU)
    {
      for( int i = temporalId ; i < maxNumSubLayers ; i ++ )
      {
        m_lastBPSEI[i] = m_totalCoded[i];
      }
      if( (slice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL)||(slice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA) )
      {
        m_rapWithLeading = true;
      }
    }


    if( m_pcCfg->getPictureTimingSEIEnabled() )
    {
      seiMessages.push_back(pictureTimingSEI);

      if (m_pcCfg->getScalableNestingSEIEnabled() && !m_pcCfg->getSamePicTimingInAllOLS())
      {
        SEIPictureTiming *pictureTimingSEIcopy = new SEIPictureTiming();
        pictureTimingSEI->copyTo(*pictureTimingSEIcopy);
        nestedSeiMessages.push_back(pictureTimingSEIcopy);
      }
    }

    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
    {
      for( int i = 0; i < ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 ); i ++ )
      {
        SEIDecodingUnitInfo *duInfoSEI = new SEIDecodingUnitInfo();
        duInfoSEI->m_decodingUnitIdx = i;
        for( int j = temporalId; j <= maxNumSubLayers; j++ )
          duInfoSEI->m_duSptCpbRemovalDelayIncrement[j] = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i*maxNumSubLayers+j] + 1;
        duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
        duInfoSEI->m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;

        duInfoSeiMessages.push_back(duInfoSEI);
      }
    }

    if( !m_pcCfg->getPictureTimingSEIEnabled() && pictureTimingSEI )
    {
      delete pictureTimingSEI;
    }
  }
}

void EncGOP::xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData)
{
  if (duData.empty())
  {
    return;
  }
  // fix first
  uint32_t numNalUnits = (uint32_t)testAU.size();
  uint32_t numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = testAU.begin(); it != testAU.end(); it++)
  {
    numRBSPBytes += uint32_t((*it)->m_nalUnitData.str().size());
  }
  duData[0].accumBitsDU += ( numRBSPBytes << 3 );
  duData[0].accumNalsDU += numNalUnits;

  // adapt cumulative sums for all following DUs
  // and add one DU info SEI, if enabled
  for (int i=1; i<duData.size(); i++)
  {
    if (m_pcCfg->getDecodingUnitInfoSEIEnabled())
    {
      numNalUnits  += 1;
      numRBSPBytes += ( 5 << 3 );
    }
    duData[i].accumBitsDU += numRBSPBytes; // probably around 5 bytes
    duData[i].accumNalsDU += numNalUnits;
  }

  // The last DU may have a trailing SEI
  if (m_pcCfg->getDecodedPictureHashSEIType()!=HASHTYPE_NONE)
  {
    duData.back().accumBitsDU += ( 20 << 3 ); // probably around 20 bytes - should be further adjusted, e.g. by type
    duData.back().accumNalsDU += 1;
  }

}
void EncGOP::xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const SPS *sps)
{
  if (!pictureTimingSEI)
  {
    return;
  }
  const GeneralHrdParams *hrd = sps->getGeneralHrdParameters();
  if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
  {
    int i;
    uint64_t ui64Tmp;
    uint32_t uiPrev = 0;
    uint32_t numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
    std::vector<uint32_t> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
    uint32_t maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

    int maxNumSubLayers = sps->getMaxTLayers();
    for( int j = 0; j < maxNumSubLayers - 1; j++ )
      pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[j] = false;

    for( i = 0; i < numDU; i ++ )
    {
      pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
    }

    if( numDU == 1 )
    {
      rDuCpbRemovalDelayMinus1[ 0 + maxNumSubLayers - 1 ] = 0; /* don't care */
    }
    else
    {
      rDuCpbRemovalDelayMinus1[ (numDU - 1) * maxNumSubLayers + maxNumSubLayers - 1 ] = 0;/* by definition */
      uint32_t tmp = 0;
      uint32_t accum = 0;

      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        ui64Tmp = (((duData[numDU - 1].accumBitsDU - duData[i].accumBitsDU) * (sps->getGeneralHrdParameters()->getTimeScale() / sps->getGeneralHrdParameters()->getNumUnitsInTick()) * (hrd->getTickDivisorMinus2() + 2)) / (m_pcCfg->getTargetBitrate()));
        if( (uint32_t)ui64Tmp > maxDiff )
        {
          tmp ++;
        }
      }
      uiPrev = 0;

      uint32_t flag = 0;
      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        flag = 0;
        ui64Tmp = (((duData[numDU - 1].accumBitsDU - duData[i].accumBitsDU) * (sps->getGeneralHrdParameters()->getTimeScale() / sps->getGeneralHrdParameters()->getNumUnitsInTick()) * (hrd->getTickDivisorMinus2() + 2)) / (m_pcCfg->getTargetBitrate()));

        if( (uint32_t)ui64Tmp > maxDiff )
        {
          if(uiPrev >= maxDiff - tmp)
          {
            ui64Tmp = uiPrev + 1;
            flag = 1;
          }
          else                            ui64Tmp = maxDiff - tmp + 1;
        }
        rDuCpbRemovalDelayMinus1[ i * maxNumSubLayers + maxNumSubLayers - 1 ] = (uint32_t)ui64Tmp - uiPrev - 1;
        if( (int)rDuCpbRemovalDelayMinus1[ i * maxNumSubLayers + maxNumSubLayers - 1 ] < 0 )
        {
          rDuCpbRemovalDelayMinus1[ i * maxNumSubLayers + maxNumSubLayers - 1 ] = 0;
        }
        else if (tmp > 0 && flag == 1)
        {
          tmp --;
        }
        accum += rDuCpbRemovalDelayMinus1[ i * maxNumSubLayers + maxNumSubLayers - 1 ] + 1;
        uiPrev = accum;
      }
    }
  }
}
void EncGOP::xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI, int maxSubLayers)
{
  if (duInfoSeiMessages.empty() || (pictureTimingSEI == NULL))
  {
    return;
  }

  int i=0;

  for (SEIMessages::iterator du = duInfoSeiMessages.begin(); du!= duInfoSeiMessages.end(); du++)
  {
    SEIDecodingUnitInfo *duInfoSEI = (SEIDecodingUnitInfo*) (*du);
    duInfoSEI->m_decodingUnitIdx = i;
    for ( int j = 0; j < maxSubLayers; j++ )
    {
      duInfoSEI->m_duiSubLayerDelaysPresentFlag[j] = pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[j];
      duInfoSEI->m_duSptCpbRemovalDelayIncrement[j] = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i*maxSubLayers+j] + 1;
    }
    duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
    i++;
  }
}

static void
validateMinCrRequirements(const ProfileLevelTierFeatures &plt, std::size_t numBytesInVclNalUnits, const Picture *pPic, const EncCfg *pCfg)
{
  //  numBytesInVclNalUnits shall be less than or equal to
  //     FormatCapabilityFactor * MaxLumaSr * framePeriod / MinCr,
  //     ( = FormatCapabilityFactor * MaxLumaSr / (MinCr * frameRate),
  if (plt.getLevelTierFeatures() && plt.getProfileFeatures() && plt.getLevelTierFeatures()->level!=Level::LEVEL15_5)
  {
    const uint32_t formatCapabilityFactorx1000 = plt.getProfileFeatures()->formatCapabilityFactorx1000;
    const uint64_t maxLumaSr = plt.getLevelTierFeatures()->maxLumaSr;
    const uint32_t frameRate = pCfg->getFrameRate();
    const double   minCr = plt.getMinCr();
    const double   denominator = (minCr * frameRate * 1000);
    if (denominator!=0)
    {
      const double   threshold =(formatCapabilityFactorx1000 * maxLumaSr) / (denominator);

      if (numBytesInVclNalUnits > threshold)
      {
        msg(WARNING_, "WARNING_: Encoded stream does not meet MinCr requirements numBytesInVclNalUnits (%.0f) must be <= %.0f. Try increasing Qp, tier or level\n",
                      (double) numBytesInVclNalUnits, threshold );
      }
    }
  }
}
static std::size_t
cabac_zero_word_padding(const Slice *const pcSlice,
                        const Picture *const pcPic,
                        const std::size_t binCountsInNalUnits,
                        const std::size_t numBytesInVclNalUnits,
                        const std::size_t numZeroWordsAlreadyInserted,
                              std::ostringstream &nalUnitData,
                        const bool cabacZeroWordPaddingEnabled,
                        const ProfileLevelTierFeatures &plt)
{
  const SPS &sps=*(pcSlice->getSPS());
  const ChromaFormat format = sps.getChromaFormatIdc();
  const int log2subWidthCxsubHeightC = (::getComponentScaleX(COMPONENT_Cb, format)+::getComponentScaleY(COMPONENT_Cb, format));
  const int minCuWidth  = 1 << pcSlice->getSPS()->getLog2MinCodingBlockSize();
  const int minCuHeight = 1 << pcSlice->getSPS()->getLog2MinCodingBlockSize();
  const int paddedWidth = ( ( pcSlice->getPPS()->getPicWidthInLumaSamples() + minCuWidth - 1 ) / minCuWidth ) * minCuWidth;
  const int paddedHeight = ( ( pcSlice->getPPS()->getPicHeightInLumaSamples() + minCuHeight - 1 ) / minCuHeight ) * minCuHeight;
  const int rawBits = paddedWidth * paddedHeight *
                         (sps.getBitDepth(CHANNEL_TYPE_LUMA) + ((2*sps.getBitDepth(CHANNEL_TYPE_CHROMA))>>log2subWidthCxsubHeightC));
  const int vclByteScaleFactor_x3 = ( 32 + 4 * (plt.getTier()==Level::HIGH ? 1 : 0) );
  const std::size_t threshold = (vclByteScaleFactor_x3*numBytesInVclNalUnits/3) + (rawBits/32);
  // "The value of BinCountsInPicNalUnits shall be less than or equal to vclByteScaleFactor * NumBytesInPicVclNalUnits     + ( RawMinCuBits * PicSizeInMinCbsY ) / 32."
  //               binCountsInNalUnits                  <=               vclByteScaleFactor_x3 * numBytesInVclNalUnits / 3 +   rawBits / 32.
  // If it is currently not, then add cabac_zero_words to increase numBytesInVclNalUnits.
  if (binCountsInNalUnits >= threshold)
  {
    // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase numBytesInVclNalUnits
    const std::size_t targetNumBytesInVclNalUnits = ((binCountsInNalUnits - (rawBits/32))*3+vclByteScaleFactor_x3-1)/vclByteScaleFactor_x3;

    if (targetNumBytesInVclNalUnits>numBytesInVclNalUnits) // It should be!
    {
      const std::size_t numberOfAdditionalBytesNeeded= std::max<std::size_t>(0, targetNumBytesInVclNalUnits - numBytesInVclNalUnits - numZeroWordsAlreadyInserted * 3);
      const std::size_t numberOfAdditionalCabacZeroWords=(numberOfAdditionalBytesNeeded+2)/3;
      const std::size_t numberOfAdditionalCabacZeroBytes=numberOfAdditionalCabacZeroWords*3;
      if (cabacZeroWordPaddingEnabled)
      {
        std::vector<uint8_t> zeroBytesPadding(numberOfAdditionalCabacZeroBytes, uint8_t(0));
        for(std::size_t i=0; i<numberOfAdditionalCabacZeroWords; i++)
        {
          zeroBytesPadding[i*3+2]=3;  // 00 00 03
        }
        nalUnitData.write(reinterpret_cast<const char*>(&(zeroBytesPadding[0])), numberOfAdditionalCabacZeroBytes);
        msg(NOTICE_, "Adding %d bytes of padding\n", uint32_t( numberOfAdditionalCabacZeroWords * 3 ) );
      }
      else
      {
        msg(NOTICE_, "Standard would normally require adding %d bytes of padding\n", uint32_t( numberOfAdditionalCabacZeroWords * 3 ) );
      }
      return numberOfAdditionalCabacZeroWords;
    }
  }
      return 0;
}

class EfficientFieldIRAPMapping
{
  private:
    int  IRAPGOPid;
    bool IRAPtoReorder;
    bool swapIRAPForward;

  public:
    EfficientFieldIRAPMapping() :
      IRAPGOPid(-1),
      IRAPtoReorder(false),
      swapIRAPForward(false)
    { }

    void initialize(const bool isField, const int gopSize, const int POCLast, const int numPicRcvd, const int lastIDR, EncGOP *pEncGop, EncCfg *pCfg);

    int adjustGOPid(const int gopID);
    int restoreGOPid(const int gopID);
    int GetIRAPGOPid() const { return IRAPGOPid; }
};

void EfficientFieldIRAPMapping::initialize(const bool isField, const int gopSize, const int POCLast, const int numPicRcvd, const int lastIDR, EncGOP *pEncGop, EncCfg *pCfg )
{
  if(isField)
  {
    int pocCurr;
    for ( int iGOPid=0; iGOPid < gopSize; iGOPid++ )
    {
      // determine actual POC
      if(POCLast == 0) //case first frame or first top field
      {
        pocCurr=0;
      }
      else if(POCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = POCLast - numPicRcvd + pCfg->getGOPEntry(iGOPid).m_POC - isField;
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = pEncGop->getNalUnitType(pocCurr, lastIDR, isField);
      if (tmpUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
      {
        if(pocCurr%2 == 0 && iGOPid < gopSize-1 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid+1).m_POC-1)
        { // if top field and following picture in enc order is associated bottom field
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = true;
          break;
        }
        if(pocCurr%2 != 0 && iGOPid > 0 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid-1).m_POC+1)
        {
          // if picture is an IRAP remember to process it first
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = false;
          break;
        }
      }
    }
  }
}

int EfficientFieldIRAPMapping::adjustGOPid(const int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return IRAPGOPid;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid -1)
      {
        return IRAPGOPid;
      }
      else if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
    }
  }
  return GOPid;
}

int EfficientFieldIRAPMapping::restoreGOPid(const int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        IRAPtoReorder = false;
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return GOPid -1;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
      else if(GOPid == IRAPGOPid -1)
      {
        IRAPtoReorder = false;
        return IRAPGOPid;
      }
    }
  }
  return GOPid;
}


static void
printHash(const HashType hashType, const std::string &digestStr)
{
  const char *decodedPictureHashModeName;
  switch (hashType)
  {
    case HASHTYPE_MD5:
      decodedPictureHashModeName = "MD5";
      break;
    case HASHTYPE_CRC:
      decodedPictureHashModeName = "CRC";
      break;
    case HASHTYPE_CHECKSUM:
      decodedPictureHashModeName = "Checksum";
      break;
    default:
      decodedPictureHashModeName = NULL;
      break;
  }
  if (decodedPictureHashModeName != NULL)
  {
    if (digestStr.empty())
    {
      msg(NOTICE_, " [%s:%s]", decodedPictureHashModeName, "?");
    }
    else
    {
      msg(NOTICE_, " [%s:%s]", decodedPictureHashModeName, digestStr.c_str());
    }
  }
}

bool isPicEncoded( int targetPoc, int curPoc, int curTLayer, int gopSize, int intraPeriod )
{
  int  tarGop = targetPoc / gopSize;
  int  curGop = curPoc / gopSize;

  if( tarGop + 1 == curGop )
  {
    // part of next GOP only for tl0 pics
    return curTLayer == 0;
  }

  int  tarIFr = ( targetPoc / intraPeriod ) * intraPeriod;
  int  curIFr = ( curPoc / intraPeriod ) * intraPeriod;

  if( curIFr != tarIFr )
  {
    return false;
  }

  int  tarId = targetPoc - tarGop * gopSize;

  if( tarGop > curGop )
  {
    return ( tarId == 0 ) ? ( 0 == curTLayer ) : ( 1 >= curTLayer );
  }

  if( tarGop + 1 < curGop )
  {
    return false;
  }

  int  curId = curPoc - curGop * gopSize;
  int  tarTL = 0;

  while( tarId != 0 )
  {
    gopSize /= 2;
    if( tarId >= gopSize )
    {
      tarId -= gopSize;
      if( curId != 0 ) curId -= gopSize;
    }
    else if( curId == gopSize )
    {
      curId = 0;
    }
    tarTL++;
  }

  return curTLayer <= tarTL && curId == 0;
}

void trySkipOrDecodePicture( bool& decPic, bool& encPic, const EncCfg& cfg, Picture* pcPic, ParameterSetMap<APS> *apsMap )
{
  // check if we should decode a leading bitstream
  if( !cfg.getDecodeBitstream( 0 ).empty() )
  {
    static bool bDecode1stPart = true; /* TODO: MT */
    if( bDecode1stPart )
    {
      if( cfg.getForceDecodeBitstream1() )
      {
        if( ( bDecode1stPart = tryDecodePicture( pcPic, pcPic->getPOC(), cfg.getDecodeBitstream( 0 ), apsMap, false ) ) )
        {
          decPic = bDecode1stPart;
        }
      }
      else
      {
        // update decode decision
        bool dbgCTU = cfg.getDebugCTU() != -1 && cfg.getSwitchPOC() == pcPic->getPOC();

        if( ( bDecode1stPart = ( cfg.getSwitchPOC() != pcPic->getPOC() ) || dbgCTU ) && ( bDecode1stPart = tryDecodePicture( pcPic, pcPic->getPOC(), cfg.getDecodeBitstream( 0 ), apsMap, false, cfg.getDebugCTU(), cfg.getSwitchPOC() ) ) )
        {
          if( dbgCTU )
          {
            encPic = true;
            decPic = false;
            bDecode1stPart = false;

            return;
          }
          decPic = bDecode1stPart;
          return;
        }
        else if( pcPic->getPOC() )
        {
          // reset decoder if used and not required any further
          tryDecodePicture( NULL, 0, std::string( "" ) );
        }
      }
    }

    encPic |= cfg.getForceDecodeBitstream1() && !decPic;
    if( cfg.getForceDecodeBitstream1() ) { return; }
  }


  // check if we should decode a trailing bitstream
  if( ! cfg.getDecodeBitstream(1).empty() )
  {
    const int  iNextKeyPOC    = (1+cfg.getSwitchPOC()  / cfg.getGOPSize())     *cfg.getGOPSize();
    const int  iNextIntraPOC  = (1+(cfg.getSwitchPOC() / cfg.getIntraPeriod()))*cfg.getIntraPeriod();
    const int  iRestartIntraPOC   = iNextIntraPOC + (((iNextKeyPOC == iNextIntraPOC) && cfg.getSwitchDQP() ) ? cfg.getIntraPeriod() : 0);

    bool bDecode2ndPart = (pcPic->getPOC() >= iRestartIntraPOC);
    int expectedPoc = pcPic->getPOC();
    Slice slice0;
    if ( cfg.getBs2ModPOCAndType() )
    {
      expectedPoc = pcPic->getPOC() - iRestartIntraPOC;
      slice0.copySliceInfo( pcPic->slices[ 0 ], false );
    }
    if( bDecode2ndPart && (bDecode2ndPart = tryDecodePicture( pcPic, expectedPoc, cfg.getDecodeBitstream(1), apsMap, true )) )
    {
      decPic = bDecode2ndPart;
      if ( cfg.getBs2ModPOCAndType() )
      {
        for( int i = 0; i < pcPic->slices.size(); i++ )
        {
          pcPic->slices[ i ]->setPOC              ( slice0.getPOC()            );
          if ( pcPic->slices[ i ]->getNalUnitType() != slice0.getNalUnitType()
              && pcPic->slices[ i ]->getIdrPicFlag()
              && slice0.getRapPicFlag()
              && slice0.isIntra() )
          {
            // patch IDR-slice to CRA-Intra-slice
            pcPic->slices[ i ]->setNalUnitType    ( slice0.getNalUnitType()    );
            pcPic->slices[ i ]->setLastIDR        ( slice0.getLastIDR()        );
            if ( pcPic->cs->picHeader->getEnableTMVPFlag() )
            {
              pcPic->slices[ i ]->setColFromL0Flag( slice0.getColFromL0Flag()  );
              pcPic->slices[ i ]->setColRefIdx    ( slice0.getColRefIdx()      );
            }
          }
        }
      }
      return;
    }
  }

  // leave here if we do not use forward to poc
  if( ! cfg.useFastForwardToPOC() )
  {
    // let's encode
    encPic   = true;
    return;
  }

  // this is the forward to poc section
  static bool bHitFastForwardPOC = false; /* TODO: MT */
  if( bHitFastForwardPOC || isPicEncoded( cfg.getFastForwardToPOC(), pcPic->getPOC(), pcPic->temporalId, cfg.getGOPSize(), cfg.getIntraPeriod() ) )
  {
    bHitFastForwardPOC |= cfg.getFastForwardToPOC() == pcPic->getPOC(); // once we hit the poc we continue encoding

    if( bHitFastForwardPOC && cfg.getStopAfterFFtoPOC() && cfg.getFastForwardToPOC() != pcPic->getPOC() )
    {
      return;
    }

    //except if FastForwardtoPOC is meant to be a SwitchPOC in thist case drop all preceding pictures
    if( bHitFastForwardPOC && ( cfg.getSwitchPOC() == cfg.getFastForwardToPOC() ) && ( cfg.getFastForwardToPOC() > pcPic->getPOC() ) )
    {
      return;
    }
    // let's encode
    encPic   = true;
  }
}

void EncGOP::xPicInitHashME( Picture *pic, const PPS *pps, PicList &rcListPic )
{
  if (! m_pcCfg->getUseHashME())
  {
    return;
  }

  PicList::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    Picture* refPic = *(iterPic++);

    if (refPic->poc != pic->poc && refPic->referenced)
    {
      if (!refPic->getHashMap()->isInitial())
      {
        if (refPic->getPOC() == 0)
        {
          Pel* picSrc = refPic->getOrigBuf().get(COMPONENT_Y).buf;
          int stridePic = refPic->getOrigBuf().get(COMPONENT_Y).stride;
          int picWidth = pps->getPicWidthInLumaSamples();
          int picHeight = pps->getPicHeightInLumaSamples();
          int blockSize = 4;
          int allNum = 0;
          int simpleNum = 0;
          for (int j = 0; j <= picHeight - blockSize; j += blockSize)
          {
            for (int i = 0; i <= picWidth - blockSize; i += blockSize)
            {
              Pel* curBlock = picSrc + j * stridePic + i;
              bool isHorSame = true;
              for (int m = 0; m < blockSize&&isHorSame; m++)
              {
                for (int n = 1; n < blockSize&&isHorSame; n++)
                {
                  if (curBlock[m*stridePic] != curBlock[m*stridePic + n])
                  {
                    isHorSame = false;
                  }
                }
              }
              bool isVerSame = true;
              for (int m = 1; m < blockSize&&isVerSame; m++)
              {
                for (int n = 0; n < blockSize&&isVerSame; n++)
                {
                  if (curBlock[n] != curBlock[m*stridePic + n])
                  {
                    isVerSame = false;
                  }
                }
              }
              allNum++;
              if (isHorSame || isVerSame)
              {
                simpleNum++;
              }
            }
          }

          if (simpleNum < 0.3*allNum)
          {
            m_pcCfg->setUseHashME(false);
            break;
          }
        }
        refPic->addPictureToHashMapForInter();
      }
    }
  }
}

void EncGOP::xPicInitRateControl(int &estimatedBits, int gopId, double &lambda, Picture *pic, Slice *slice)
{
  if ( !m_pcCfg->getUseRateCtrl() ) // TODO: does this work with multiple slices and slice-segments?
  {
    return;
  }
  int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( gopId );
  if ( pic->slices[0]->isIRAP() )
  {
    frameLevel = 0;
  }
  m_pcRateCtrl->initRCPic( frameLevel );
  estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();

#if U0132_TARGET_BITS_SATURATION
  if (m_pcRateCtrl->getCpbSaturationEnabled() && frameLevel != 0)
  {
    int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

    // prevent overflow
    if (estimatedCpbFullness - estimatedBits > (int)(m_pcRateCtrl->getCpbSize()*0.9f))
    {
      estimatedBits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.9f);
    }

    estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
    // prevent underflow
#if V0078_ADAPTIVE_LOWER_BOUND
    if (estimatedCpbFullness - estimatedBits < m_pcRateCtrl->getRCPic()->getLowerBound())
    {
      estimatedBits = std::max(200, estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound());
    }
#else
    if (estimatedCpbFullness - estimatedBits < (int)(m_pcRateCtrl->getCpbSize()*0.1f))
    {
      estimatedBits = std::max(200, estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.1f));
    }
#endif

    m_pcRateCtrl->getRCPic()->setTargetBits(estimatedBits);
  }
#endif

  int sliceQP = m_pcCfg->getInitialQP();
  if ( ( slice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
  {
    int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
    double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)NumberBFrames );
    double dQPFactor     = 0.57*dLambda_scale;
    int    SHIFT_QP      = 12;
    int bitdepth_luma_qp_scale = 6 * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                                - DISTORTION_PRECISION_ADJUSTMENT(slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
    double qp_temp = (double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
    lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
  }
  else if ( frameLevel == 0 )   // intra case, but use the model
  {
    m_pcSliceEncoder->calCostPictureI(pic);
    if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
    {
      int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
      bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );

#if U0132_TARGET_BITS_SATURATION
      if (m_pcRateCtrl->getCpbSaturationEnabled() )
      {
        int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

        // prevent overflow
        if (estimatedCpbFullness - bits > (int)(m_pcRateCtrl->getCpbSize()*0.9f))
        {
          bits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.9f);
        }

        estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
        // prevent underflow
#if V0078_ADAPTIVE_LOWER_BOUND
        if (estimatedCpbFullness - bits < m_pcRateCtrl->getRCPic()->getLowerBound())
        {
          bits = estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound();
        }
#else
        if (estimatedCpbFullness - bits < (int)(m_pcRateCtrl->getCpbSize()*0.1f))
        {
          bits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.1f);
        }
#endif
      }
#endif

      if ( bits < 200 )
      {
        bits = 200;
      }
      m_pcRateCtrl->getRCPic()->setTargetBits( bits );
    }

    list<EncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
    m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
    lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, slice->isIRAP());
    sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
  }
  else    // normal case
  {
    list<EncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
    lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, slice->isIRAP());
    sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
  }

  sliceQP = Clip3( -slice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, sliceQP );
  m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );

  m_pcSliceEncoder->resetQP( pic, sliceQP, lambda );
}

void EncGOP::xPicInitLMCS(Picture *pic, PicHeader *picHeader, Slice *slice)
{
  if (slice->getSPS()->getUseLmcs())
  {
    const SliceType realSliceType = slice->getSliceType();
    SliceType condSliceType = realSliceType;

    if (condSliceType != I_SLICE && slice->getNalUnitLayerId() > 0 && (slice->getNalUnitType()>= NAL_UNIT_CODED_SLICE_IDR_W_RADL && slice->getNalUnitType()<= NAL_UNIT_CODED_SLICE_CRA))
    {
      condSliceType = I_SLICE;
    }
    m_pcReshaper->getReshapeCW()->rspTid = slice->getTLayer() + (slice->isIntra() ? 0 : 1);
    m_pcReshaper->getReshapeCW()->rspSliceQP = slice->getSliceQp();

    m_pcReshaper->setSrcReshaped(false);
    m_pcReshaper->setRecReshaped(true);

    m_pcReshaper->getSliceReshaperInfo().chrResScalingOffset = m_pcCfg->getReshapeCSoffset();

    if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ)
    {
      m_pcReshaper->preAnalyzerHDR(pic, condSliceType, m_pcCfg->getReshapeCW(), m_pcCfg->getDualITree());
    }
    else if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_SDR || m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_HLG)
    {
      m_pcReshaper->preAnalyzerLMCS(pic, m_pcCfg->getReshapeSignalType(), condSliceType, m_pcCfg->getReshapeCW());
    }
    else
    {
      THROW("Reshaper for other signal currently not defined!");
    }

    if (condSliceType == I_SLICE )
    {
      if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ)
      {
        m_pcReshaper->initLUTfromdQPModel();
        m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTableChromaMD(m_pcReshaper->getInvLUT());
      }
      else if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_SDR || m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_HLG)
      {
        if (m_pcReshaper->getReshapeFlag())
        {
          m_pcReshaper->constructReshaperLMCS();
          m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTable(m_pcReshaper->getSliceReshaperInfo(), m_pcReshaper->getWeightTable(), m_pcReshaper->getCWeight());
        }
      }
      else
      {
        THROW("Reshaper for other signal currently not defined!");
      }

      m_pcReshaper->setCTUFlag(false);
      if (realSliceType != condSliceType)
      {
        m_pcReshaper->setCTUFlag(true);
      }
    }
    else
    {
      if (!m_pcReshaper->getReshapeFlag())
      {
        m_pcReshaper->setCTUFlag(false);
      }
      else
        m_pcReshaper->setCTUFlag(true);

      m_pcReshaper->getSliceReshaperInfo().setSliceReshapeModelPresentFlag(false);

      if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ)
      {
        m_pcEncLib->getRdCost()->restoreReshapeLumaLevelToWeightTable();
      }
      else if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_SDR || m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_HLG)
      {
        int modIP = pic->getPOC() - pic->getPOC() / m_pcCfg->getReshapeCW().rspFpsToIp * m_pcCfg->getReshapeCW().rspFpsToIp;
        if (m_pcReshaper->getReshapeFlag() && m_pcCfg->getReshapeCW().updateCtrl == 2 && modIP == 0)
        {
          m_pcReshaper->getSliceReshaperInfo().setSliceReshapeModelPresentFlag(true);
          m_pcReshaper->constructReshaperLMCS();
          m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTable(m_pcReshaper->getSliceReshaperInfo(), m_pcReshaper->getWeightTable(), m_pcReshaper->getCWeight());
        }
      }
      else
      {
        THROW("Reshaper for other signal currently not defined!");
      }
    }

    //set all necessary information in LMCS APS and picture header
    picHeader->setLmcsEnabledFlag(m_pcReshaper->getSliceReshaperInfo().getUseSliceReshaper());
    slice->setLmcsEnabledFlag(m_pcReshaper->getSliceReshaperInfo().getUseSliceReshaper());
    picHeader->setLmcsChromaResidualScaleFlag(m_pcReshaper->getSliceReshaperInfo().getSliceReshapeChromaAdj() == 1);
    if (m_pcReshaper->getSliceReshaperInfo().getSliceReshapeModelPresentFlag())
    {
      int apsId = std::min<int>( 3, m_pcEncLib->getVPS() == nullptr ? 0 : m_pcEncLib->getVPS()->getGeneralLayerIdx( m_pcEncLib->getLayerId() ) );
      picHeader->setLmcsAPSId(apsId);
      APS* lmcsAPS = picHeader->getLmcsAPS();
      if (lmcsAPS == nullptr)
      {
        ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap();
        lmcsAPS = apsMap->getPS((apsId << NUM_APS_TYPE_LEN) + LMCS_APS);
        if (lmcsAPS == NULL)
        {
          lmcsAPS = apsMap->allocatePS((apsId << NUM_APS_TYPE_LEN) + LMCS_APS);
          lmcsAPS->setAPSId(apsId);
          lmcsAPS->setAPSType(LMCS_APS);
        }
        picHeader->setLmcsAPS(lmcsAPS);
      }
      //m_pcReshaper->copySliceReshaperInfo(lmcsAPS->getReshaperAPSInfo(), m_pcReshaper->getSliceReshaperInfo());
      SliceReshapeInfo& tInfo = lmcsAPS->getReshaperAPSInfo();
      SliceReshapeInfo& sInfo = m_pcReshaper->getSliceReshaperInfo();
      tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
      tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
      memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int)*(PIC_CODE_CW_BINS));
      tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
      tInfo.chrResScalingOffset = sInfo.chrResScalingOffset;
      m_pcEncLib->getApsMap()->setChangedFlag((lmcsAPS->getAPSId() << NUM_APS_TYPE_LEN) + LMCS_APS);
    }


    if (picHeader->getLmcsEnabledFlag())
    {
      int apsId = std::min<int>( 3, m_pcEncLib->getVPS() == nullptr ? 0 : m_pcEncLib->getVPS()->getGeneralLayerIdx( m_pcEncLib->getLayerId() ) );
      picHeader->setLmcsAPSId(apsId);
    }
  }
  else
  {
    m_pcReshaper->setCTUFlag(false);
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void EncGOP::compressGOP( int iPOCLast, int iNumPicRcvd, PicList& rcListPic,
                          std::list<PelUnitBuf*>& rcListPicYuvRecOut,
                          bool isField, bool isTff, const InputColourSpaceConversion snr_conversion, 
                          const bool printFrameMSE, const bool printMSSSIM, bool isEncodeLtRef, const int picIdInGOP)
{
  // TODO: Split this function up.

  Picture*        pcPic = NULL;
  PicHeader*      picHeader = NULL;
  Slice*      pcSlice;
  OutputBitstream  *pcBitstreamRedirect;
  pcBitstreamRedirect = new OutputBitstream;
  AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted
  Picture* scaledRefPic[MAX_NUM_REF] = {};

  xInitGOP( iPOCLast, iNumPicRcvd, isField, isEncodeLtRef );

  m_iNumPicCoded = 0;
  SEIMessages leadingSeiMessages;
  SEIMessages nestedSeiMessages;
  SEIMessages duInfoSeiMessages;
  SEIMessages trailingSeiMessages;
  std::deque<DUData> duData;

  EfficientFieldIRAPMapping effFieldIRAPMap;
  if (m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    effFieldIRAPMap.initialize(isField, m_iGopSize, iPOCLast, iNumPicRcvd, m_iLastIDR, this, m_pcCfg);
  }

  if( isField && picIdInGOP == 0 )
  {
    for( int iGOPid = 0; iGOPid < max(2, m_iGopSize); iGOPid++ )
    {
      m_pcCfg->setEncodedFlag( iGOPid, false );
    }
  }
  for( int iGOPid = picIdInGOP; iGOPid <= picIdInGOP; iGOPid++ )
  {
    // reset flag indicating whether pictures have been encoded
    m_pcCfg->setEncodedFlag( iGOPid, false );
    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.adjustGOPid(iGOPid);
    }

    //-- For time output for each slice
    auto beforeTime = std::chrono::steady_clock::now();

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
    uint32_t uiColDir = calculateCollocatedFromL1Flag(m_pcCfg, iGOPid, m_iGopSize);
#endif

    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
    int iTimeOffset;
    int pocCurr;
    int multipleFactor = m_pcCfg->getUseCompositeRef() ? 2 : 1;

    if(iPOCLast == 0) //case first frame or first top field
    {
      pocCurr=0;
      iTimeOffset = isField ? (1 - multipleFactor) : multipleFactor;
    }
    else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    {
      pocCurr = 1;
      iTimeOffset = multipleFactor + 1;
    }
    else
    {
      pocCurr = iPOCLast - iNumPicRcvd * multipleFactor + m_pcCfg->getGOPEntry(iGOPid).m_POC - ((isField && m_iGopSize>1) ? 1 : 0);
      iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    }

    if (m_pcCfg->getUseCompositeRef() && isEncodeLtRef)
    {
      pocCurr++;
      iTimeOffset--;
    }
    if (pocCurr / multipleFactor >= m_pcCfg->getFramesToBeEncoded())
    {
      if (m_pcCfg->getEfficientFieldIRAPEnabled())
      {
        iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
      }
      continue;
    }

    if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_iLastIDR = pocCurr;
    }

    // start a new access unit: create an entry in the list of output access units
    AccessUnit accessUnit;
    accessUnit.temporalId = m_pcCfg->getGOPEntry( iGOPid ).m_temporalId;
    xGetBuffer( rcListPic, rcListPicYuvRecOut,
                iNumPicRcvd, iTimeOffset, pcPic, pocCurr, isField );
    picHeader = pcPic->cs->picHeader;
    picHeader->setSPSId( pcPic->cs->pps->getSPSId() );
    picHeader->setPPSId( pcPic->cs->pps->getPPSId() );
    picHeader->setSplitConsOverrideFlag(false);
    // initial two flags to be false
    picHeader->setPicInterSliceAllowedFlag(false);
    picHeader->setPicIntraSliceAllowedFlag(false);
#if ER_CHROMA_QP_WCG_PPS
    // th this is a hot fix for the choma qp control
    if( m_pcEncLib->getWCGChromaQPControl().isEnabled() && m_pcEncLib->getSwitchPOC() != -1 )
    {
      static int usePPS = 0; /* TODO: MT */
      if( pocCurr == m_pcEncLib->getSwitchPOC() )
      {
        usePPS = 1;
      }
      const PPS *pPPS = m_pcEncLib->getPPS(usePPS);
      // replace the pps with a more appropriated one
      pcPic->cs->pps = pPPS;
    }
#endif

    // create objects based on the picture size
    const int picWidth = pcPic->cs->pps->getPicWidthInLumaSamples();
    const int picHeight = pcPic->cs->pps->getPicHeightInLumaSamples();
    const int maxCUWidth = pcPic->cs->sps->getMaxCUWidth();
    const int maxCUHeight = pcPic->cs->sps->getMaxCUHeight();
    const ChromaFormat chromaFormatIDC = pcPic->cs->sps->getChromaFormatIdc();
    const int maxTotalCUDepth = floorLog2(maxCUWidth) - pcPic->cs->sps->getLog2MinCodingBlockSize();

    m_pcSliceEncoder->create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxTotalCUDepth );

#if ENABLE_SPLIT_PARALLELISM
    pcPic->scheduler.init( pcPic->cs->pcv->heightInCtus, pcPic->cs->pcv->widthInCtus, 1                          , 0                             , m_pcCfg->getNumSplitThreads() );
#endif
    pcPic->createTempBuffers( pcPic->cs->pps->pcv->maxCUWidth );
    pcPic->cs->createCoeffs((bool)pcPic->cs->sps->getPLTMode());

    //  Slice data initialization
    pcPic->clearSliceBuffer();
    pcPic->allocateNewSlice();
    m_pcSliceEncoder->setSliceSegmentIdx(0);

    m_pcSliceEncoder->initEncSlice(pcPic, iPOCLast, pocCurr, iGOPid, pcSlice, isField, isEncodeLtRef, m_pcEncLib->getLayerId() );

    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", pocCurr ) ) );
    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 0 ) ) );

#if !SHARP_LUMA_DELTA_QP
    //Set Frame/Field coding
    pcPic->fieldPic = isField;
#endif

    pcSlice->setLastIDR(m_iLastIDR);
    pcSlice->setIndependentSliceIdx(0);

    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
    {
      pcSlice->setSliceType(P_SLICE);
    }
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
    {
      pcSlice->setSliceType(I_SLICE);
    }
    pcSlice->setTLayer(m_pcCfg->getGOPEntry(iGOPid).m_temporalId);

    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
    // set two flags according to slice type presented in the picture
    if (pcSlice->getSliceType() != I_SLICE)
    {
      picHeader->setPicInterSliceAllowedFlag(true);
    }
    if (pcSlice->getSliceType() == I_SLICE)
    {
      picHeader->setPicIntraSliceAllowedFlag(true);
    }
    picHeader->setGdrOrIrapPicFlag(picHeader->getGdrPicFlag() || pcSlice->isIRAP());

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)  // IRAP picture
      {
        m_associatedIRAPType[pcPic->layerId] = pcSlice->getNalUnitType();
        m_associatedIRAPPOC[pcPic->layerId] = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType[pcPic->layerId]);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC[pcPic->layerId]);
    }

    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic, m_pcCfg->getEfficientFieldIRAPEnabled());
    if (m_pcCfg->getUseCompositeRef() && isEncodeLtRef)
    {
      setUseLTRef(true);
      setPrepareLTRef(false);
      setNewestBgPOC(pocCurr);
      setLastLTRefPoc(pocCurr);
    }
    else if (m_pcCfg->getUseCompositeRef() && getLastLTRefPoc() >= 0 && getEncodedLTRef()==false && !getPicBg()->getSpliceFull() && (pocCurr - getLastLTRefPoc()) > (m_pcCfg->getFrameRate() * 2))
    {
      setUseLTRef(false);
      setPrepareLTRef(false);
      setEncodedLTRef(true);
      setNewestBgPOC(-1);
      setLastLTRefPoc(-1);
    }

    if (m_pcCfg->getUseCompositeRef() && m_picBg->getSpliceFull() && getUseLTRef())
    {
      m_pcEncLib->selectReferencePictureList(pcSlice, pocCurr, iGOPid, m_bgPOC);
    }
    else
    {
      m_pcEncLib->selectReferencePictureList(pcSlice, pocCurr, iGOPid, -1);
    }
    if (!m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)  // IRAP picture
      {
        m_associatedIRAPType[pcPic->layerId] = pcSlice->getNalUnitType();
        m_associatedIRAPPOC[pcPic->layerId] = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType[pcPic->layerId]);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC[pcPic->layerId]);
    }

    pcSlice->setEnableDRAPSEI(m_pcEncLib->getDependentRAPIndicationSEIEnabled());
    if (m_pcEncLib->getDependentRAPIndicationSEIEnabled())
    {
      // Only mark the picture as DRAP if all of the following applies:
      //  1) DRAP indication SEI messages are enabled
      //  2) The current picture is not an intra picture
      //  3) The current picture is in the DRAP period
      //  4) The current picture is a trailing picture
      pcSlice->setDRAP(m_pcEncLib->getDependentRAPIndicationSEIEnabled() && m_pcEncLib->getDrapPeriod() > 0 && !pcSlice->isIntra() &&
              pocCurr % m_pcEncLib->getDrapPeriod() == 0 && pocCurr > pcSlice->getAssociatedIRAPPOC());

      if (pcSlice->isDRAP())
      {
        int pocCycle = 1 << (pcSlice->getSPS()->getBitsForPOC());
        int deltaPOC = pocCurr > pcSlice->getAssociatedIRAPPOC() ? pocCurr - pcSlice->getAssociatedIRAPPOC() : pocCurr - ( pcSlice->getAssociatedIRAPPOC() & (pocCycle -1) );
        CHECK_(deltaPOC > (pocCycle >> 1), "Use a greater value for POC wraparound to enable a POC distance between IRAP and DRAP of " << deltaPOC << ".");
        m_latestDRAPPOC = pocCurr;
        pcSlice->setTLayer(0); // Force DRAP picture to have temporal layer 0
      }
      pcSlice->setLatestDRAPPOC(m_latestDRAPPOC);
      pcSlice->setUseLTforDRAP(false); // When set, sets the associated IRAP as long-term in RPL0 at slice level, unless the associated IRAP is already included in RPL0 or RPL1 defined in SPS

      PicList::iterator iterPic = rcListPic.begin();
      Picture *rpcPic;
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);
        if ( pcSlice->isDRAP() && rpcPic->getPOC() != pocCurr )
        {
            rpcPic->precedingDRAP = true;
        }
        else if ( !pcSlice->isDRAP() && rpcPic->getPOC() == pocCurr )
        {
          rpcPic->precedingDRAP = false;
        }
      }
    }

    if (pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPL0(), 0, false) != 0 || pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPL1(), 1, false) != 0 ||
        (m_pcEncLib->getDependentRAPIndicationSEIEnabled() && !pcSlice->isIRAP() && ( pcSlice->isDRAP() || !pcSlice->isPOCInRefPicList(pcSlice->getRPL0(), pcSlice->getAssociatedIRAPPOC())) )
      || ((m_pcEncLib->getAvoidIntraInDepLayer() || !pcSlice->isIRAP()) && pcSlice->getPic()->cs->vps && m_pcEncLib->getNumRefLayers(pcSlice->getPic()->cs->vps->getGeneralLayerIdx(m_pcEncLib->getLayerId())))
      )
    {
      xCreateExplicitReferencePictureSetFromReference( pcSlice, rcListPic, pcSlice->getRPL0(), pcSlice->getRPL1() );
    }

    pcSlice->applyReferencePictureListBasedMarking( rcListPic, pcSlice->getRPL0(), pcSlice->getRPL1(), pcSlice->getPic()->layerId, *(pcSlice->getPPS()));

    if(pcSlice->getTLayer() > 0
      && !(pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL     // Check if not a leading picture
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL)
        )
    {
    if (pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
      {
        bool isSTSA=true;


        for(int ii=0;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
        {
          int lTid = m_pcCfg->getRPLEntry(0, ii).m_temporalId;

          if (lTid == pcSlice->getTLayer())
          {
            const ReferencePictureList* rpl0 = pcSlice->getSPS()->getRPLList0()->getReferencePictureList(ii);
            for (int jj = 0; jj < pcSlice->getRPL0()->getNumberOfActivePictures(); jj++)
            {
              int tPoc = pcSlice->getPOC() + rpl0->getRefPicIdentifier(jj);
              int kk = 0;
              for (kk = 0; kk<m_pcCfg->getGOPSize(); kk++)
              {
                if (m_pcCfg->getRPLEntry(0, kk).m_POC == tPoc)
                {
                  break;
                }
              }
              int tTid = m_pcCfg->getRPLEntry(0, kk).m_temporalId;
              if (tTid >= pcSlice->getTLayer())
              {
                isSTSA = false;
                break;
              }
            }
            const ReferencePictureList* rpl1 = pcSlice->getSPS()->getRPLList1()->getReferencePictureList(ii);
            for (int jj = 0; jj < pcSlice->getRPL1()->getNumberOfActivePictures(); jj++)
            {
              int tPoc = pcSlice->getPOC() + rpl1->getRefPicIdentifier(jj);
              int kk = 0;
              for (kk = 0; kk<m_pcCfg->getGOPSize(); kk++)
              {
                if (m_pcCfg->getRPLEntry(1, kk).m_POC == tPoc)
                {
                  break;
                }
              }
              int tTid = m_pcCfg->getRPLEntry(1, kk).m_temporalId;
              if (tTid >= pcSlice->getTLayer())
              {
                isSTSA = false;
                break;
              }
            }
          }
        }
        if(isSTSA==true)
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA);
        }
      }
    }

    if (m_pcCfg->getUseCompositeRef() && getUseLTRef() && (pocCurr > getLastLTRefPoc()))
    {
      pcSlice->setNumRefIdx(REF_PIC_LIST_0, (pcSlice->isIntra()) ? 0 : min(m_pcCfg->getRPLEntry(0, iGOPid).m_numRefPicsActive + 1, pcSlice->getRPL0()->getNumberOfActivePictures()));
      pcSlice->setNumRefIdx(REF_PIC_LIST_1, (!pcSlice->isInterB()) ? 0 : min(m_pcCfg->getRPLEntry(1, iGOPid).m_numRefPicsActive + 1, pcSlice->getRPL1()->getNumberOfActivePictures()));
    }
    else
    {
      pcSlice->setNumRefIdx(REF_PIC_LIST_0, (pcSlice->isIntra()) ? 0 : pcSlice->getRPL0()->getNumberOfActivePictures());
      pcSlice->setNumRefIdx(REF_PIC_LIST_1, (!pcSlice->isInterB()) ? 0 : pcSlice->getRPL1()->getNumberOfActivePictures());
    }
    if (m_pcCfg->getUseCompositeRef() && getPrepareLTRef()) {
      arrangeCompositeReference(pcSlice, rcListPic, pocCurr);
    }
    //  Set reference list
    pcSlice->constructRefPicList(rcListPic);

    // store sub-picture numbers, sizes, and locations with a picture
    pcSlice->getPic()->subPictures.clear();

    for( int subPicIdx = 0; subPicIdx < pcPic->cs->pps->getNumSubPics(); subPicIdx++ )
    {
      pcSlice->getPic()->subPictures.push_back( pcPic->cs->pps->getSubPic( subPicIdx ) );
    }

    const VPS* vps = pcPic->cs->vps;
    int layerIdx = vps == nullptr ? 0 : vps->getGeneralLayerIdx(pcPic->layerId);
    if (vps && !vps->getIndependentLayerFlag(layerIdx) && pcPic->cs->pps->getNumSubPics() > 1)
    {
      CU::checkConformanceILRP(pcSlice);
    }

    xPicInitHashME( pcPic, pcSlice->getPPS(), rcListPic );

    if( m_pcCfg->getUseAMaxBT() )
    {
      if (!pcSlice->isIRAP())
      {
        int refLayer = pcSlice->getDepth();
        if( refLayer > 9 ) refLayer = 9; // Max layer is 10

        if( m_bInitAMaxBT && pcSlice->getPOC() > m_uiPrevISlicePOC )
        {
          ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
          ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
          m_bInitAMaxBT = false;
        }

        if( refLayer >= 0 && m_uiNumBlk[refLayer] != 0 )
        {
          picHeader->setSplitConsOverrideFlag(true);
          double dBlkSize = sqrt( ( double ) m_uiBlkSize[refLayer] / m_uiNumBlk[refLayer] );
          unsigned int newMaxBtSize = picHeader->getMaxBTSize(pcSlice->getSliceType(), CHANNEL_TYPE_LUMA);
          if( dBlkSize < AMAXBT_TH32 )
          {
            newMaxBtSize = 32;
          }
          else if( dBlkSize < AMAXBT_TH64 )
          {
            newMaxBtSize = 64;
          }
          else
          {
            newMaxBtSize = 128;
          }
          newMaxBtSize = Clip3(picHeader->getMinQTSize(pcSlice->getSliceType()), pcPic->cs->sps->getCTUSize(), newMaxBtSize);
          picHeader->setMaxBTSize(1, newMaxBtSize);

          m_uiBlkSize[refLayer] = 0;
          m_uiNumBlk [refLayer] = 0;
        }
      }
      else
      {
        if( m_bInitAMaxBT )
        {
          ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
          ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
        }

        m_uiPrevISlicePOC = pcSlice->getPOC();
        m_bInitAMaxBT = true;
      }
      bool identicalToSPS=true;
      const SPS* sps =pcSlice->getSPS();

      if (picHeader->getPicInterSliceAllowedFlag())
      {
        if (picHeader->getMinQTSize(pcSlice->getSliceType()) != pcSlice->getSPS()->getMinQTSize(pcSlice->getSliceType()) ||
            picHeader->getMaxMTTHierarchyDepth(pcSlice->getSliceType()) != pcSlice->getSPS()->getMaxMTTHierarchyDepth() ||
            picHeader->getMaxBTSize(pcSlice->getSliceType()) != pcSlice->getSPS()->getMaxBTSize() ||
            picHeader->getMaxTTSize(pcSlice->getSliceType()) != pcSlice->getSPS()->getMaxTTSize()
          )
        {
          identicalToSPS=false;
        }
      }

      if (identicalToSPS && picHeader->getPicIntraSliceAllowedFlag())
      {
        if (picHeader->getMinQTSize(I_SLICE) != sps->getMinQTSize(I_SLICE) ||
            picHeader->getMaxMTTHierarchyDepth(I_SLICE) != sps->getMaxMTTHierarchyDepthI() ||
            picHeader->getMaxBTSize(I_SLICE) != sps->getMaxBTSizeI() ||
            picHeader->getMaxTTSize(I_SLICE) != sps->getMaxTTSizeI()
        )
        {
          identicalToSPS=false;
        }

        if (identicalToSPS && sps->getUseDualITree())
        {
          if (picHeader->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA) != sps->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA) ||
              picHeader->getMaxMTTHierarchyDepth(I_SLICE, CHANNEL_TYPE_CHROMA) != sps->getMaxMTTHierarchyDepthIChroma() ||
              picHeader->getMaxBTSize(I_SLICE, CHANNEL_TYPE_CHROMA) != sps->getMaxBTSizeIChroma() ||
              picHeader->getMaxTTSize(I_SLICE, CHANNEL_TYPE_CHROMA) != sps->getMaxTTSizeIChroma()
           )
          {
            identicalToSPS=false;
          }
        }
      }

      if (identicalToSPS)
      {
        picHeader->setSplitConsOverrideFlag(false);
      }
    }

    //  Slice info. refinement
    if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )
    {
      pcSlice->setSliceType ( P_SLICE );
    }

    xUpdateRasInit( pcSlice );

    if (pcSlice->getPendingRasInit() || pcSlice->isIRAP())
    {
      // this ensures that independently encoded bitstream chunks can be combined to bit-equal
      pcSlice->setEncCABACTableIdx( pcSlice->getSliceType() );
    }
    else
    {
      pcSlice->setEncCABACTableIdx( m_pcSliceEncoder->getEncCABACTableIdx() );
    }

    if (pcSlice->getSliceType() == B_SLICE)
    {

      bool bLowDelay = true;
      int  iCurrPOC  = pcSlice->getPOC();
      int iRefIdx = 0;

      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }
    else
    {
      pcSlice->setCheckLDC(true);
    }


    //-------------------------------------------------------------
    pcSlice->setRefPOCList();


    pcSlice->setList1IdxToList0Idx();

    if (m_pcEncLib->getTMVPModeId() == 2)
    {
      if (iGOPid == 0) // first picture in SOP (i.e. forward B)
      {
        picHeader->setEnableTMVPFlag(0);
      }
      else
      {
        // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
        picHeader->setEnableTMVPFlag(1);
      }
    }
    else if (m_pcEncLib->getTMVPModeId() == 1)
    {
      picHeader->setEnableTMVPFlag(1);
    }
    else
    {
      picHeader->setEnableTMVPFlag(0);
    }

    // disable TMVP when current picture is the only ref picture
    if (pcSlice->isIRAP() && pcSlice->getSPS()->getIBCFlag())
    {
      picHeader->setEnableTMVPFlag(0);
    }

    if( pcSlice->getSliceType() != I_SLICE && picHeader->getEnableTMVPFlag() )
    {
      int colRefIdxL0 = -1, colRefIdxL1 = -1;

      for( int refIdx = 0; refIdx < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); refIdx++ )
      {
        CHECK_( pcSlice->getRefPic( REF_PIC_LIST_0, refIdx )->unscaledPic == nullptr, "unscaledPic is not set for L0 reference picture" );

        if( pcSlice->getRefPic( REF_PIC_LIST_0, refIdx )->isRefScaled( pcSlice->getPPS() ) == false )
        {
          colRefIdxL0 = refIdx;
          break;
        }
      }

      if( pcSlice->getSliceType() == B_SLICE )
      {
        for( int refIdx = 0; refIdx < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); refIdx++ )
        {
          CHECK_( pcSlice->getRefPic( REF_PIC_LIST_1, refIdx )->unscaledPic == nullptr, "unscaledPic is not set for L1 reference picture" );

          if( pcSlice->getRefPic( REF_PIC_LIST_1, refIdx )->isRefScaled( pcSlice->getPPS() ) == false )
          {
            colRefIdxL1 = refIdx;
            break;
          }
        }
      }

      if( colRefIdxL0 >= 0 && colRefIdxL1 >= 0 )
      {
        const Picture *refPicL0 = pcSlice->getRefPic( REF_PIC_LIST_0, colRefIdxL0 );
        if( !refPicL0->slices.size() )
        {
          refPicL0 = refPicL0->unscaledPic;
        }

        const Picture *refPicL1 = pcSlice->getRefPic( REF_PIC_LIST_1, colRefIdxL1 );
        if( !refPicL1->slices.size() )
        {
          refPicL1 = refPicL1->unscaledPic;
        }

        CHECK_( !refPicL0->slices.size(), "Wrong L0 reference picture" );
        CHECK_( !refPicL1->slices.size(), "Wrong L1 reference picture" );

        const uint32_t uiColFromL0 = refPicL0->slices[0]->getSliceQp() > refPicL1->slices[0]->getSliceQp();
        picHeader->setPicColFromL0Flag( uiColFromL0 );
        pcSlice->setColFromL0Flag( uiColFromL0 );
        pcSlice->setColRefIdx( uiColFromL0 ? colRefIdxL0 : colRefIdxL1 );
        picHeader->setColRefIdx( uiColFromL0 ? colRefIdxL0 : colRefIdxL1 );
      }
      else if( colRefIdxL0 < 0 && colRefIdxL1 >= 0 )
      {
        picHeader->setPicColFromL0Flag( false );
        pcSlice->setColFromL0Flag( false );
        pcSlice->setColRefIdx( colRefIdxL1 );
        picHeader->setColRefIdx( colRefIdxL1 );
      }
      else if( colRefIdxL0 >= 0 && colRefIdxL1 < 0 )
      {
        picHeader->setPicColFromL0Flag( true );
        pcSlice->setColFromL0Flag( true );
        pcSlice->setColRefIdx( colRefIdxL0 );
        picHeader->setColRefIdx( colRefIdxL0 );
      }
      else
      {
        picHeader->setEnableTMVPFlag( 0 );
      }
    }

    pcSlice->scaleRefPicList( scaledRefPic, pcPic->cs->picHeader, m_pcEncLib->getApss(), picHeader->getLmcsAPS(), picHeader->getScalingListAPS(), false );

    // set adaptive search range for non-intra-slices
    if (m_pcCfg->getUseASR() && !pcSlice->isIntra())
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);
    }

    bool bGPBcheck=false;
    if ( pcSlice->getSliceType() == B_SLICE)
    {
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        bGPBcheck=true;
        int i;
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
          {
            bGPBcheck=false;
            break;
          }
        }
      }
    }
    if(bGPBcheck)
    {
      picHeader->setMvdL1ZeroFlag(true);
    }
    else
    {
      picHeader->setMvdL1ZeroFlag(false);
    }

    if ( pcSlice->getSPS()->getUseSMVD() && pcSlice->getCheckLDC() == false
      && picHeader->getMvdL1ZeroFlag() == false
      )
    {
      int currPOC = pcSlice->getPOC();

      int forwardPOC = currPOC;
      int backwardPOC = currPOC;
      int ref = 0, refIdx0 = -1, refIdx1 = -1;

      // search nearest forward POC in List 0
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_0, ref)->longTerm;
        if ( poc < currPOC && (poc > forwardPOC || refIdx0 == -1) && !isRefLongTerm )
        {
          forwardPOC = poc;
          refIdx0 = ref;
        }
      }

      // search nearest backward POC in List 1
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_1, ref)->longTerm;
        if ( poc > currPOC && (poc < backwardPOC || refIdx1 == -1) && !isRefLongTerm )
        {
          backwardPOC = poc;
          refIdx1 = ref;
        }
      }

      if ( !(forwardPOC < currPOC && backwardPOC > currPOC) )
      {
        forwardPOC = currPOC;
        backwardPOC = currPOC;
        refIdx0 = -1;
        refIdx1 = -1;

        // search nearest backward POC in List 0
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_0, ref)->longTerm;
          if ( poc > currPOC && (poc < backwardPOC || refIdx0 == -1) && !isRefLongTerm )
          {
            backwardPOC = poc;
            refIdx0 = ref;
          }
        }

        // search nearest forward POC in List 1
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_1, ref)->longTerm;
          if ( poc < currPOC && (poc > forwardPOC || refIdx1 == -1) && !isRefLongTerm )
          {
            forwardPOC = poc;
            refIdx1 = ref;
          }
        }
      }

      if ( forwardPOC < currPOC && backwardPOC > currPOC )
      {
        pcSlice->setBiDirPred( true, refIdx0, refIdx1 );
      }
      else
      {
        pcSlice->setBiDirPred( false, -1, -1 );
      }
    }
    else
    {
      pcSlice->setBiDirPred( false, -1, -1 );
    }

    double lambda            = 0.0;
    int actualHeadBits       = 0;
    int actualTotalBits      = 0;
    int estimatedBits        = 0;
    int tmpBitsBeforeWriting = 0;

    xPicInitRateControl(estimatedBits, iGOPid, lambda, pcPic, pcSlice);

    uint32_t uiNumSliceSegments = 1;

    {
      pcSlice->setDefaultClpRng( *pcSlice->getSPS() );
    }

    // Allocate some coders, now the number of tiles are known.
    const uint32_t numberOfCtusInFrame = pcPic->cs->pcv->sizeInCtus;
    const int numSubstreamsColumns = pcSlice->getPPS()->getNumTileColumns();
    const int numSubstreamRows     = pcSlice->getSPS()->getEntropyCodingSyncEnabledFlag() ? pcPic->cs->pcv->heightInCtus : (pcSlice->getPPS()->getNumTileRows());
    const int numSubstreams        = std::max<int> (numSubstreamRows * numSubstreamsColumns, (int) pcPic->cs->pps->getNumSlicesInPic());
    std::vector<OutputBitstream> substreamsOut(numSubstreams);

#if ENABLE_QPA
    pcPic->m_uEnerHpCtu.resize (numberOfCtusInFrame);
    pcPic->m_iOffsetCtu.resize (numberOfCtusInFrame);
#if ENABLE_QPA_SUB_CTU
    if (pcSlice->getPPS()->getUseDQP() && pcSlice->getCuQpDeltaSubdiv() > 0)
    {
      const PreCalcValues &pcv = *pcPic->cs->pcv;
      const unsigned   mtsLog2 = (unsigned)floorLog2(std::min (pcPic->cs->sps->getMaxTbSize(), pcv.maxCUWidth));
      pcPic->m_subCtuQP.resize ((pcv.maxCUWidth >> mtsLog2) * (pcv.maxCUHeight >> mtsLog2));
    }
#endif
#endif
    if (pcSlice->getSPS()->getSAOEnabledFlag())
    {
      pcPic->resizeSAO( numberOfCtusInFrame, 0 );
      pcPic->resizeSAO( numberOfCtusInFrame, 1 );
    }

    // it is used for signalling during CTU mode decision, i.e. before ALF processing
    if( pcSlice->getSPS()->getALFEnabledFlag() )
    {
      pcPic->resizeAlfCtuEnableFlag( numberOfCtusInFrame );
      pcPic->resizeAlfCtuAlternative( numberOfCtusInFrame );
      pcPic->resizeAlfCtbFilterIndex(numberOfCtusInFrame);
    }

    bool decPic = false;
    bool encPic = false;
    // test if we can skip the picture entirely or decode instead of encoding
    trySkipOrDecodePicture( decPic, encPic, *m_pcCfg, pcPic, m_pcEncLib->getApsMap() );

    pcPic->cs->slice = pcSlice; // please keep this
#if ENABLE_QPA
    if (pcSlice->getPPS()->getSliceChromaQpFlag() && CS::isDualITree (*pcSlice->getPic()->cs) && !m_pcCfg->getUsePerceptQPA() && (m_pcCfg->getSliceChromaOffsetQpPeriodicity() == 0))
#else
    if (pcSlice->getPPS()->getSliceChromaQpFlag() && CS::isDualITree (*pcSlice->getPic()->cs))
#endif
    {
      // overwrite chroma qp offset for dual tree
      pcSlice->setSliceChromaQpDelta(COMPONENT_Cb, m_pcCfg->getChromaCbQpOffsetDualTree());
      pcSlice->setSliceChromaQpDelta(COMPONENT_Cr, m_pcCfg->getChromaCrQpOffsetDualTree());
      if (pcSlice->getSPS()->getJointCbCrEnabledFlag())
      {
        pcSlice->setSliceChromaQpDelta(JOINT_CbCr, m_pcCfg->getChromaCbCrQpOffsetDualTree());
      }
      m_pcSliceEncoder->setUpLambda(pcSlice, pcSlice->getLambdas()[0], pcSlice->getSliceQp());
    }

    xPicInitLMCS(pcPic, picHeader, pcSlice);

    if( pcSlice->getSPS()->getScalingListFlag() && m_pcCfg->getUseScalingListId() == SCALING_LIST_FILE_READ )
    {
      picHeader->setExplicitScalingListEnabledFlag( true );
      pcSlice->setExplicitScalingListUsed( true );

      int apsId = std::min<int>( 7, m_pcEncLib->getVPS() == nullptr ? 0 : m_pcEncLib->getVPS()->getGeneralLayerIdx( m_pcEncLib->getLayerId() ) );
      picHeader->setScalingListAPSId( apsId );

      ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap();
      APS*  scalingListAPS = apsMap->getPS( ( apsId << NUM_APS_TYPE_LEN ) + SCALING_LIST_APS );
      assert( scalingListAPS != NULL );
      picHeader->setScalingListAPS( scalingListAPS );
    }

    pcPic->cs->picHeader->setPic(pcPic);
    pcPic->cs->picHeader->setValid();
    if(pcPic->cs->sps->getFpelMmvdEnabledFlag())
    {
      // cannot set ph_fpel_mmvd_enabled_flag at slice level - need new picture-level version of checkDisFracMmvd algorithm?
      // m_pcSliceEncoder->checkDisFracMmvd( pcPic, 0, numberOfCtusInFrame );
      bool useIntegerMVD = (pcPic->lwidth()*pcPic->lheight() > 1920 * 1080);
      pcPic->cs->picHeader->setDisFracMMVD( useIntegerMVD );
    }
    if (pcSlice->getSPS()->getJointCbCrEnabledFlag())
    {
      m_pcSliceEncoder->setJointCbCrModes(*pcPic->cs, Position(0, 0), pcPic->cs->area.lumaSize());
    }
    if( encPic )
    // now compress (trial encode) the various slice segments (slices, and dependent slices)
    {
      DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", pocCurr ) ) );
      const std::vector<uint16_t> sliceLosslessArray = *(m_pcCfg->getSliceLosslessArray());
      bool mixedLossyLossless = m_pcCfg->getMixedLossyLossless();
      if (m_pcCfg->getCostMode() == COST_LOSSLESS_CODING)
      {
        pcPic->fillSliceLossyLosslessArray(sliceLosslessArray, mixedLossyLossless);
      }

      for(uint32_t sliceIdx = 0; sliceIdx < pcPic->cs->pps->getNumSlicesInPic(); sliceIdx++ )
      {
        pcSlice->setSliceMap( pcPic->cs->pps->getSliceMap( sliceIdx ) );
        if( pcPic->cs->pps->getRectSliceFlag() )
        {
          Position firstCtu;
          firstCtu.x = pcSlice->getFirstCtuRsAddrInSlice() % pcPic->cs->pps->getPicWidthInCtu();
          firstCtu.y = pcSlice->getFirstCtuRsAddrInSlice() / pcPic->cs->pps->getPicWidthInCtu();
          int subPicIdx = NOT_VALID;
          for( int sp = 0; sp < pcPic->cs->pps->getNumSubPics(); sp++ )
          {
            if( pcPic->cs->pps->getSubPic( sp ).containsCtu( firstCtu ) )
            {
              subPicIdx = sp;
              break;
            }
          }
          CHECK_( subPicIdx == NOT_VALID, "Sub-picture was not found" );

          pcSlice->setSliceSubPicId( pcPic->cs->pps->getSubPic( subPicIdx ).getSubPicID() );
        }
        if (pcPic->cs->sps->getUseLmcs())
        {
          pcSlice->setLmcsEnabledFlag(picHeader->getLmcsEnabledFlag());
          if (pcSlice->getSliceType() == I_SLICE)
          {
            //reshape original signal
            if(m_pcCfg->getGopBasedTemporalFilterEnabled())
            {
              pcPic->getOrigBuf().copyFrom(pcPic->getFilteredOrigBuf());
            }
            else
            {
              pcPic->getOrigBuf().copyFrom(pcPic->getTrueOrigBuf());
            } 

            if (pcSlice->getLmcsEnabledFlag())
            {
              pcPic->getOrigBuf(COMPONENT_Y).rspSignal(m_pcReshaper->getFwdLUT());
              m_pcReshaper->setSrcReshaped(true);
              m_pcReshaper->setRecReshaped(true);
            }
            else
            {
              m_pcReshaper->setSrcReshaped(false);
              m_pcReshaper->setRecReshaped(false);
            }
          }
        }

        bool isLossless = false;
        if (m_pcCfg->getCostMode() == COST_LOSSLESS_CODING)
        {
          isLossless = pcPic->losslessSlice(sliceIdx);
        }
        m_pcSliceEncoder->setLosslessSlice(pcPic, isLossless);

        if( pcSlice->getSliceType() != I_SLICE && pcSlice->getRefPic( REF_PIC_LIST_0, 0 )->subPictures.size() > 1 )
        {
          clipMv = clipMvInSubpic;
          m_pcEncLib->getInterSearch()->setClipMvInSubPic(true);
        }
        else
        {
          clipMv = clipMvInPic;
          m_pcEncLib->getInterSearch()->setClipMvInSubPic(false);
        }

        m_pcSliceEncoder->precompressSlice( pcPic );
        m_pcSliceEncoder->compressSlice   ( pcPic, false, false );

        if(sliceIdx < pcPic->cs->pps->getNumSlicesInPic() - 1)
        {
          uint32_t independentSliceIdx = pcSlice->getIndependentSliceIdx();
          pcPic->allocateNewSlice();
          m_pcSliceEncoder->setSliceSegmentIdx      (uiNumSliceSegments);
          // prepare for next slice
          pcSlice = pcPic->slices[uiNumSliceSegments];
          CHECK_(!(pcSlice->getPPS() != 0), "Unspecified error");
          pcSlice->copySliceInfo(pcPic->slices[uiNumSliceSegments - 1]);
          pcSlice->setSliceBits(0);
          independentSliceIdx++;
          pcSlice->setIndependentSliceIdx(independentSliceIdx);
          uiNumSliceSegments++;
        }
      }

      duData.clear();

      CodingStructure& cs = *pcPic->cs;
      pcSlice = pcPic->slices[0];

      if (cs.sps->getUseLmcs() && m_pcReshaper->getSliceReshaperInfo().getUseSliceReshaper())
      {
        picHeader->setLmcsEnabledFlag(true);
        int apsId = std::min<int>(3, m_pcEncLib->getVPS() == nullptr ? 0 : m_pcEncLib->getVPS()->getGeneralLayerIdx(m_pcEncLib->getLayerId()));
        picHeader->setLmcsAPSId(apsId);

        const PreCalcValues& pcv = *cs.pcv;
        for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
        {
          for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
          {
            const CodingUnit* cu = cs.getCU(Position(xPos, yPos), CHANNEL_TYPE_LUMA);
            if (cu->slice->getLmcsEnabledFlag())
            {
              const uint32_t width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
              const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
              const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
              cs.getRecoBuf(area).get(COMPONENT_Y).rspSignal(m_pcReshaper->getInvLUT());
            }
          }
        }
        m_pcReshaper->setRecReshaped(false);

        if(m_pcCfg->getGopBasedTemporalFilterEnabled())
        {
          pcPic->getOrigBuf().copyFrom(pcPic->getFilteredOrigBuf());
        }
        else
        {
          pcPic->getOrigBuf().copyFrom(pcPic->getTrueOrigBuf());
        } 
      }

      // create SAO object based on the picture size
      if( pcSlice->getSPS()->getSAOEnabledFlag() )
      {
        const uint32_t widthInCtus = ( picWidth + maxCUWidth - 1 ) / maxCUWidth;
        const uint32_t heightInCtus = ( picHeight + maxCUHeight - 1 ) / maxCUHeight;
        const uint32_t numCtuInFrame = widthInCtus * heightInCtus;
        const uint32_t  log2SaoOffsetScaleLuma   = (uint32_t) std::max(0, pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA  ) - MAX_SAO_TRUNCATED_BITDEPTH);
        const uint32_t  log2SaoOffsetScaleChroma = (uint32_t) std::max(0, pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - MAX_SAO_TRUNCATED_BITDEPTH);

        m_pcSAO->create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxTotalCUDepth, log2SaoOffsetScaleLuma, log2SaoOffsetScaleChroma );
        m_pcSAO->destroyEncData();
        m_pcSAO->createEncData( m_pcCfg->getSaoCtuBoundary(), numCtuInFrame );
        m_pcSAO->setReshaper( m_pcReshaper );
      }

      if( pcSlice->getSPS()->getScalingListFlag() && m_pcCfg->getUseScalingListId() == SCALING_LIST_FILE_READ )
      {
        picHeader->setExplicitScalingListEnabledFlag(true);
        pcSlice->setExplicitScalingListUsed(true);
        int apsId = 0;
        picHeader->setScalingListAPSId( apsId );
      }

      // SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
      if( pcSlice->getSPS()->getSAOEnabledFlag() && m_pcCfg->getSaoCtuBoundary() )
      {
        m_pcSAO->getPreDBFStatistics( cs );
      }

      //-- Loop filter
      if ( m_pcCfg->getDeblockingFilterMetric() )
      {
  #if W0038_DB_OPT
        if ( m_pcCfg->getDeblockingFilterMetric()==2 )
        {
          applyDeblockingFilterParameterSelection(pcPic, uiNumSliceSegments, iGOPid);
        }
        else
        {
  #endif
          applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);
  #if W0038_DB_OPT
        }
  #endif
      }
      if (m_pcCfg->getCostMode() == COST_LOSSLESS_CODING) 
      {
        for (int s = 0; s < uiNumSliceSegments; s++)
        {
          if (pcPic->slices[s]->isLossless())
          {
            pcPic->slices[s]->setDeblockingFilterDisable(true);
          }
        }
      }
      m_pcLoopFilter->loopFilterPic( cs );

      CS::setRefinedMotionField(cs);

      if( pcSlice->getSPS()->getSAOEnabledFlag() )
      {
        bool sliceEnabled[MAX_NUM_COMPONENT];
        m_pcSAO->initCABACEstimator( m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice );

        m_pcSAO->SAOProcess( cs, sliceEnabled, pcSlice->getLambdas(),
#if ENABLE_QPA
                             (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && pcSlice->getPPS()->getUseDQP() ? m_pcEncLib->getRdCost (PARL_PARAM0 (0))->getChromaWeight() : 0.0),
#endif
                             m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary(), m_pcCfg->getSaoGreedyMergeEnc() );
        //assign SAO slice header
        for (int s = 0; s < uiNumSliceSegments; s++)
        {

          if (pcPic->slices[s]->isLossless() && m_pcCfg->getCostMode() == COST_LOSSLESS_CODING)
          {
            pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, false);
            pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, false);
          }
          else
          {
            pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
            CHECK_(!(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]), "Unspecified error");
            pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);

        }

        }
      }

      if( pcSlice->getSPS()->getALFEnabledFlag() )
      {
        m_pcALF->destroy();
        m_pcALF->create( m_pcCfg, picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxTotalCUDepth, m_pcCfg->getBitDepth(), m_pcCfg->getInputBitDepth() );

        for (int s = 0; s < uiNumSliceSegments; s++)
        {
          pcPic->slices[s]->setTileGroupAlfEnabledFlag(COMPONENT_Y, false);
        }
        m_pcALF->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice, m_pcEncLib->getApsMap());
        m_pcALF->ALFProcess(cs, pcSlice->getLambdas()
#if ENABLE_QPA
          , (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && pcSlice->getPPS()->getUseDQP() ? m_pcEncLib->getRdCost(PARL_PARAM0(0))->getChromaWeight() : 0.0)
#endif
          , pcPic, uiNumSliceSegments
        );

        //assign ALF slice header
        for (int s = 0; s < uiNumSliceSegments; s++)
        {
           //For the first slice, even if it is lossless, slice level ALF is not disabled and ALF-APS is signaled so that the later lossy slices can use APS of the first slice. 
           //However, if the first slice is lossless, the ALF process is disabled for all of the CTUs ( m_ctuEnableFlag == 0) of that slice which is implemented in the function void EncAdaptiveLoopFilter::ALFProcess.         
      
          if (pcPic->slices[s]->isLossless() && s && m_pcCfg->getCostMode() == COST_LOSSLESS_CODING)
          {
            pcPic->slices[s]->setTileGroupAlfEnabledFlag(COMPONENT_Y, false);
            pcPic->slices[s]->setTileGroupAlfEnabledFlag(COMPONENT_Cb, false);
            pcPic->slices[s]->setTileGroupAlfEnabledFlag(COMPONENT_Cr, false);
          }
          else
          {
            pcPic->slices[s]->setTileGroupAlfEnabledFlag(COMPONENT_Y, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y));
            pcPic->slices[s]->setTileGroupAlfEnabledFlag(COMPONENT_Cb, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb));
            pcPic->slices[s]->setTileGroupAlfEnabledFlag(COMPONENT_Cr, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr));

          }
          if (pcPic->slices[s]->getTileGroupAlfEnabledFlag(COMPONENT_Y))
          {
            pcPic->slices[s]->setTileGroupNumAps(cs.slice->getTileGroupNumAps());
            pcPic->slices[s]->setAlfAPSs(cs.slice->getTileGroupApsIdLuma());
          }
          else
          {
            pcPic->slices[s]->setTileGroupNumAps(0);
          }
          pcPic->slices[s]->setAlfAPSs(cs.slice->getAlfAPSs());
          pcPic->slices[s]->setTileGroupApsIdChroma(cs.slice->getTileGroupApsIdChroma());
          pcPic->slices[s]->setTileGroupCcAlfCbApsId(cs.slice->getTileGroupCcAlfCbApsId());
          pcPic->slices[s]->setTileGroupCcAlfCrApsId(cs.slice->getTileGroupCcAlfCrApsId());
          pcPic->slices[s]->m_ccAlfFilterParam      = m_pcALF->getCcAlfFilterParam();
          pcPic->slices[s]->m_ccAlfFilterControl[0] = m_pcALF->getCcAlfControlIdc(COMPONENT_Cb);
          pcPic->slices[s]->m_ccAlfFilterControl[1] = m_pcALF->getCcAlfControlIdc(COMPONENT_Cr);
        }
      }
      DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 1 ) ) );
      if (m_pcCfg->getUseCompositeRef() && getPrepareLTRef())
      {
        updateCompositeReference(pcSlice, rcListPic, pocCurr);
      }
    }
    else // skip enc picture
    {
      pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

#if ENABLE_QPA
      if (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && pcSlice->getPPS()->getUseDQP())
      {
        const double picLambda = pcSlice->getLambdas()[0];

        for (uint32_t ctuRsAddr = 0; ctuRsAddr < numberOfCtusInFrame; ctuRsAddr++)
        {
          pcPic->m_uEnerHpCtu[ctuRsAddr] = picLambda;  // initialize to slice lambda (just for safety)
        }
      }
#endif
      if( pcSlice->getSPS()->getSAOEnabledFlag() )
      {
        m_pcSAO->disabledRate( *pcPic->cs, pcPic->getSAO(1), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma());
      }
      if (pcSlice->getSPS()->getALFEnabledFlag() && (pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y) || pcSlice->getTileGroupCcAlfCbEnabledFlag() || pcSlice->getTileGroupCcAlfCrEnabledFlag()))
      {
        // IRAP AU: reset APS map
        {
          int layerIdx = pcSlice->getVPS() == nullptr ? 0 : pcSlice->getVPS()->getGeneralLayerIdx( pcSlice->getPic()->layerId );
          if( !layerIdx && ( pcSlice->getPendingRasInit() || pcSlice->isIDRorBLA() ) )
          {
            // We have to reset all APS on IRAP, but in not encoding case we have to keep the parsed APS of current slice
            // Get active ALF APSs from picture/slice header
            const std::vector<int> sliceApsIdsLuma = pcSlice->getTileGroupApsIdLuma();

            m_pcALF->setApsIdStart( ALF_CTB_MAX_NUM_APS );

            ParameterSetMap<APS>* apsMap = m_pcEncLib->getApsMap();
            apsMap->clearActive();

            for( int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++ )
            {
              int psId = ( apsId << NUM_APS_TYPE_LEN ) + ALF_APS;
              APS* aps = apsMap->getPS( psId );
              if( aps )
              {
                // Check if this APS is currently the active one (used in current slice)
                bool activeAps = false;
                bool activeApsCcAlf = false;
                // Luma
                for( int i = 0; i < sliceApsIdsLuma.size(); i++ )
                {
                  if( aps->getAPSId() == sliceApsIdsLuma[i] )
                  {
                    activeAps = true;
                    break;
                  }
                }
                // Chroma
                activeAps |= aps->getAPSId() == pcSlice->getTileGroupApsIdChroma();
                // CC-ALF
                activeApsCcAlf |= pcSlice->getTileGroupCcAlfCbEnabledFlag() && aps->getAPSId() == pcSlice->getTileGroupCcAlfCbApsId();
                activeApsCcAlf |= pcSlice->getTileGroupCcAlfCrEnabledFlag() && aps->getAPSId() == pcSlice->getTileGroupCcAlfCrApsId();
                if( !activeAps && !activeApsCcAlf )
                {
                  apsMap->clearChangedFlag( psId );
                }
                if( !activeAps  )
                {
                  aps->getAlfAPSParam().reset();
                }
                if( !activeApsCcAlf )
                {
                  aps->getCcAlfAPSParam().reset();
                }
              }
            }
          }
        }

        // Assign tne correct APS to slice and emulate the setting of ALF start APS ID
        int changedApsId = -1;
        for( int apsId = ALF_CTB_MAX_NUM_APS - 1; apsId >= 0; apsId-- )
        {
          ParameterSetMap<APS>* apsMap = m_pcEncLib->getApsMap();
          int psId = ( apsId << NUM_APS_TYPE_LEN ) + ALF_APS;
          APS* aps = apsMap->getPS( psId );
          if( aps )
          {
            // In slice, replace the old APS (from decoder map) with the APS from encoder map due to later checks while bitstream writing
            if( pcSlice->getAlfAPSs() && pcSlice->getAlfAPSs()[apsId] )
            {
              pcSlice->getAlfAPSs()[apsId] = aps;
            }
            if( apsMap->getChangedFlag( psId ) )
              changedApsId = apsId;
          }
        }
        if( changedApsId >= 0 )
          m_pcALF->setApsIdStart( changedApsId );
      }
    }

    pcSlice->freeScaledRefPicList( scaledRefPic );

    if( m_pcCfg->getUseAMaxBT() )
    {
      for( const CodingUnit *cu : pcPic->cs->cus )
      {
        if( !pcSlice->isIntra() )
        {
          m_uiBlkSize[pcSlice->getDepth()] += cu->Y().area();
          m_uiNumBlk [pcSlice->getDepth()]++;
        }
      }
    }

    if( encPic || decPic )
    {
      pcSlice = pcPic->slices[0];

      /////////////////////////////////////////////////////////////////////////////////////////////////// File writing

      // write various parameter sets
      bool writePS = m_bSeqFirst || (m_pcCfg->getReWriteParamSets() && (pcSlice->isIRAP()));
      if (writePS)
      {
        m_pcEncLib->setParamSetChanged(pcSlice->getSPS()->getSPSId(), pcSlice->getPPS()->getPPSId());
      }

      int layerIdx = m_pcEncLib->getVPS() == nullptr ? 0 : m_pcEncLib->getVPS()->getGeneralLayerIdx( m_pcEncLib->getLayerId() );

      // it is assumed that layerIdx equal to 0 is always present
      m_audIrapOrGdrAuFlag = pcSlice->getPicHeader()->getGdrPicFlag() || (pcSlice->isIRAP() && !pcSlice->getPPS()->getMixedNaluTypesInPicFlag());
      if ((( m_pcEncLib->getVPS()->getMaxLayers() > 1 && m_audIrapOrGdrAuFlag) || m_pcCfg->getAccessUnitDelimiter()) && !layerIdx )
      {
        xWriteAccessUnitDelimiter(accessUnit, pcSlice);
      }

      // it is assumed that layerIdx equal to 0 is always present
      actualTotalBits += xWriteParameterSets(accessUnit, pcSlice, writePS, layerIdx);
      if (writePS)
      {
        // create prefix SEI messages at the beginning of the sequence
        CHECK_(!(leadingSeiMessages.empty()), "Unspecified error");
        xCreateIRAPLeadingSEIMessages(leadingSeiMessages, pcSlice->getSPS(), pcSlice->getPPS());

        m_bSeqFirst = false;
      }


      //send LMCS APS when LMCSModel is updated. It can be updated even current slice does not enable reshaper.
      //For example, in RA, update is on intra slice, but intra slice may not use reshaper
      if (pcSlice->getSPS()->getUseLmcs())
      {
        //only 1 LMCS data for 1 picture
        int apsId = picHeader->getLmcsAPSId();
        ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap();
        APS* aps = apsMap->getPS((apsId << NUM_APS_TYPE_LEN) + LMCS_APS);
        bool writeAPS = aps && apsMap->getChangedFlag((apsId << NUM_APS_TYPE_LEN) + LMCS_APS);
        if (writeAPS)
        {
          aps->chromaPresentFlag = pcSlice->getSPS()->getChromaFormatIdc() != CHROMA_400;
          actualTotalBits += xWriteAPS( accessUnit, aps, m_pcEncLib->getLayerId(), true );
          apsMap->clearChangedFlag((apsId << NUM_APS_TYPE_LEN) + LMCS_APS);
          CHECK_(aps != picHeader->getLmcsAPS(), "Wrong LMCS APS pointer in compressGOP");
        }
      }

      // only 1 SCALING LIST data for 1 picture
      if( pcSlice->getSPS()->getScalingListFlag() && ( m_pcCfg->getUseScalingListId() == SCALING_LIST_FILE_READ ) )
      {
        int apsId = picHeader->getScalingListAPSId();
        ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap();
        APS* aps = apsMap->getPS( ( apsId << NUM_APS_TYPE_LEN ) + SCALING_LIST_APS );
        bool writeAPS = aps && apsMap->getChangedFlag( ( apsId << NUM_APS_TYPE_LEN ) + SCALING_LIST_APS );
        if( writeAPS )
        {
          aps->chromaPresentFlag = pcSlice->getSPS()->getChromaFormatIdc() != CHROMA_400;
          actualTotalBits += xWriteAPS( accessUnit, aps, m_pcEncLib->getLayerId(), true );
          apsMap->clearChangedFlag( ( apsId << NUM_APS_TYPE_LEN ) + SCALING_LIST_APS );
          CHECK_( aps != picHeader->getScalingListAPS(), "Wrong SCALING LIST APS pointer in compressGOP" );
        }
      }

      if (pcSlice->getSPS()->getALFEnabledFlag() && (pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y) || pcSlice->getTileGroupCcAlfCbEnabledFlag() || pcSlice->getTileGroupCcAlfCrEnabledFlag()))
      {
        for (int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++)
        {
          ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap();

          APS* aps = apsMap->getPS((apsId << NUM_APS_TYPE_LEN) + ALF_APS);
          bool writeAPS = aps && apsMap->getChangedFlag((apsId << NUM_APS_TYPE_LEN) + ALF_APS);
          if (!aps && pcSlice->getAlfAPSs() && pcSlice->getAlfAPSs()[apsId])
          {
            writeAPS = true;
            aps = pcSlice->getAlfAPSs()[apsId]; // use asp from slice header
            *apsMap->allocatePS((apsId << NUM_APS_TYPE_LEN) + ALF_APS) = *aps; //allocate and cpy
            m_pcALF->setApsIdStart( apsId );
          }
          else if (pcSlice->getTileGroupCcAlfCbEnabledFlag() && !aps && apsId == pcSlice->getTileGroupCcAlfCbApsId())
          {
            writeAPS = true;
            aps = apsMap->getPS((pcSlice->getTileGroupCcAlfCbApsId() << NUM_APS_TYPE_LEN) + ALF_APS);
          }
          else if (pcSlice->getTileGroupCcAlfCrEnabledFlag() && !aps && apsId == pcSlice->getTileGroupCcAlfCrApsId())
          {
            writeAPS = true;
            aps = apsMap->getPS((pcSlice->getTileGroupCcAlfCrApsId() << NUM_APS_TYPE_LEN) + ALF_APS);
          }

          if (writeAPS )
          {
            aps->chromaPresentFlag = pcSlice->getSPS()->getChromaFormatIdc() != CHROMA_400;
            actualTotalBits += xWriteAPS( accessUnit, aps, m_pcEncLib->getLayerId(), true );
            apsMap->clearChangedFlag((apsId << NUM_APS_TYPE_LEN) + ALF_APS);
            CHECK_(aps != pcSlice->getAlfAPSs()[apsId] && apsId != pcSlice->getTileGroupCcAlfCbApsId() && apsId != pcSlice->getTileGroupCcAlfCrApsId(), "Wrong APS pointer in compressGOP");
          }
        }
      }

      // reset presence of BP SEI indication
      m_bufferingPeriodSEIPresentInAU = false;
      // create prefix SEI associated with a picture
      xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);

      // pcSlice is currently slice 0.
      std::size_t binCountsInNalUnits   = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
      std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
      std::size_t sumZeroWords          = 0; // sum of cabac_zero_word inserted per sub-picture
      std::vector<EncBitstreamParams> subPicStats (pcPic->cs->pps->getNumSubPics());

      for(uint32_t sliceSegmentIdxCount = 0; sliceSegmentIdxCount < pcPic->cs->pps->getNumSlicesInPic(); sliceSegmentIdxCount++ )
      {
        pcSlice = pcPic->slices[sliceSegmentIdxCount];
        if(sliceSegmentIdxCount > 0 && pcSlice->getSliceType()!= I_SLICE)
        {
          pcSlice->checkColRefIdx(sliceSegmentIdxCount, pcPic);
        }
        m_pcSliceEncoder->setSliceSegmentIdx(sliceSegmentIdxCount);

        *pcSlice->getRPL0() = *pcPic->slices[0]->getRPL0();
        *pcSlice->getRPL1() = *pcPic->slices[0]->getRPL1();
        pcSlice->setRPL0idx(pcPic->slices[0]->getRPL0idx());
        pcSlice->setRPL1idx(pcPic->slices[0]->getRPL1idx());

        picHeader->setNoOutputBeforeRecoveryFlag( false );
        if (pcSlice->isIRAP())
        {
          if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
          {
            picHeader->setNoOutputBeforeRecoveryFlag( true );
          }
          //the inference for NoOutputPriorPicsFlag
          // KJS: This cannot happen at the encoder
          if( !m_bFirst && ( pcSlice->isIRAP() || pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_GDR ) && picHeader->getNoOutputBeforeRecoveryFlag() )
          {
            if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_GDR)
            {
              pcSlice->setNoOutputOfPriorPicsFlag(true);
            }
          }
        }

        // code picture header before first slice
        if(sliceSegmentIdxCount == 0)
        {
          // code RPL in picture header or slice headers
          if( !m_pcCfg->getSliceLevelRpl() && (!pcSlice->getIdrPicFlag() || pcSlice->getSPS()->getIDRRefParamListPresent()) )
          {
            picHeader->setRPL0idx(pcSlice->getRPL0idx());
            picHeader->setRPL1idx(pcSlice->getRPL1idx());
            *picHeader->getRPL0() = *pcSlice->getRPL0();
            *picHeader->getRPL1() = *pcSlice->getRPL1();
          }

          // code DBLK in picture header or slice headers
          if( !m_pcCfg->getSliceLevelDblk() )
          {
            picHeader->setDeblockingFilterOverrideFlag   ( pcSlice->getDeblockingFilterOverrideFlag()   );
            picHeader->setDeblockingFilterDisable        ( pcSlice->getDeblockingFilterDisable()        );
            picHeader->setDeblockingFilterBetaOffsetDiv2 ( pcSlice->getDeblockingFilterBetaOffsetDiv2() );
            picHeader->setDeblockingFilterTcOffsetDiv2   ( pcSlice->getDeblockingFilterTcOffsetDiv2()   );
            picHeader->setDeblockingFilterCbBetaOffsetDiv2( pcSlice->getDeblockingFilterCbBetaOffsetDiv2() );
            picHeader->setDeblockingFilterCbTcOffsetDiv2  ( pcSlice->getDeblockingFilterCbTcOffsetDiv2() );
            picHeader->setDeblockingFilterCrBetaOffsetDiv2( pcSlice->getDeblockingFilterCrBetaOffsetDiv2() );
            picHeader->setDeblockingFilterCrTcOffsetDiv2  ( pcSlice->getDeblockingFilterCrTcOffsetDiv2() );
          }

          if (!m_pcCfg->getSliceLevelDeltaQp())
          {
            picHeader->setQpDelta(pcSlice->getSliceQp() - (pcSlice->getPPS()->getPicInitQPMinus26() + 26));
          }

          // code SAO parameters in picture header or slice headers
          if( !m_pcCfg->getSliceLevelSao() )
          {
            picHeader->setSaoEnabledFlag(CHANNEL_TYPE_LUMA,   pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_LUMA  ));
            picHeader->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA));
          }

          // code ALF parameters in picture header or slice headers
          if( !m_pcCfg->getSliceLevelAlf() )
          {
            picHeader->setAlfEnabledFlag(COMPONENT_Y,  pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y ) );
            picHeader->setAlfEnabledFlag(COMPONENT_Cb, pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) );
            picHeader->setAlfEnabledFlag(COMPONENT_Cr, pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) );
            picHeader->setNumAlfAps(pcSlice->getTileGroupNumAps());
            picHeader->setAlfAPSs(pcSlice->getTileGroupApsIdLuma());
            picHeader->setAlfApsIdChroma(pcSlice->getTileGroupApsIdChroma());
            picHeader->setCcAlfEnabledFlag(COMPONENT_Cb, pcSlice->getTileGroupCcAlfCbEnabledFlag());
            picHeader->setCcAlfEnabledFlag(COMPONENT_Cr, pcSlice->getTileGroupCcAlfCrEnabledFlag());
            picHeader->setCcAlfCbApsId(pcSlice->getTileGroupCcAlfCbApsId());
            picHeader->setCcAlfCrApsId(pcSlice->getTileGroupCcAlfCrApsId());
          }

          // code WP parameters in picture header or slice headers
          if (!m_pcCfg->getSliceLevelWp())
          {
            picHeader->setWpScaling(pcSlice->getWpScalingAll());
            picHeader->setNumL0Weights(pcSlice->getNumRefIdx(REF_PIC_LIST_0));
            picHeader->setNumL0Weights(pcSlice->getNumRefIdx(REF_PIC_LIST_1));
          }

          pcPic->cs->picHeader->setPic(pcPic);
          pcPic->cs->picHeader->setValid();
          if (pcPic->cs->pps->getNumSlicesInPic() > 1 || !m_pcCfg->getEnablePictureHeaderInSliceHeader())
          {
            pcSlice->setPictureHeaderInSliceHeader(false);
            actualTotalBits += xWritePicHeader(accessUnit, pcPic->cs->picHeader);
          }
          else
          {
            pcSlice->setPictureHeaderInSliceHeader(true);
          }
          if (pcSlice->getSPS()->getProfileTierLevel()->getConstraintInfo()->getPicHeaderInSliceHeaderConstraintFlag())
          {
            CHECK_(pcSlice->getPictureHeaderInSliceHeader() == false, "PH shall be present in SH, when pic_header_in_slice_header_constraint_flag is equal to 1");
          }
        }
        pcSlice->setPicHeader( pcPic->cs->picHeader );
        pcSlice->setNalUnitLayerId( m_pcEncLib->getLayerId() );

        for ( uint32_t ui = 0 ; ui < numSubstreams; ui++ )
        {
          substreamsOut[ui].clear();
        }

        /* start slice NALunit */
        OutputNALUnit nalu( pcSlice->getNalUnitType(), m_pcEncLib->getLayerId(), pcSlice->getTLayer() );
        m_HLSWriter->setBitstream( &nalu.m_Bitstream );


        tmpBitsBeforeWriting = m_HLSWriter->getNumberOfWrittenBits();
        m_HLSWriter->codeSliceHeader( pcSlice );
        actualHeadBits += ( m_HLSWriter->getNumberOfWrittenBits() - tmpBitsBeforeWriting );

        pcSlice->setFinalized(true);

        pcSlice->resetNumberOfSubstream( );
        pcSlice->setNumSubstream( pcSlice->getSPS(), pcSlice->getPPS() );
        pcSlice->clearSubstreamSizes(  );
        const int subpicIdx = pcPic->cs->pps->getSubPicIdxFromSubPicId(pcSlice->getSliceSubPicId());
        {
          uint32_t numBinsCoded = 0;
          m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
          binCountsInNalUnits+=numBinsCoded;
          subPicStats[subpicIdx].numBinsWritten += numBinsCoded;
        }
        {
          // Construct the final bitstream by concatenating substreams.
          // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
          // Complete the slice header info.
          m_HLSWriter->setBitstream( &nalu.m_Bitstream );
          m_HLSWriter->codeTilesWPPEntryPoint( pcSlice );

          // Append substreams...
          OutputBitstream *pcOut = pcBitstreamRedirect;
          const int numSubstreamsToCode = pcSlice->getNumberOfSubstream() + 1;

          for ( uint32_t ui = 0 ; ui < numSubstreamsToCode; ui++ )
          {
            pcOut->addSubstream(&(substreamsOut[ui]));
          }
        }

        // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
        // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
        bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
        xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
        accessUnit.push_back(new NALUnitEBSP(nalu));
        actualTotalBits += uint32_t(accessUnit.back()->m_nalUnitData.str().size()) * 8;
        numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
        subPicStats[subpicIdx].numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
        bNALUAlignedWrittenToList = true;

        if (!bNALUAlignedWrittenToList)
        {
          nalu.m_Bitstream.writeAlignZero();
          accessUnit.push_back(new NALUnitEBSP(nalu));
        }

        if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
        ((pcSlice->getSPS()->getGeneralHrdParameters()->getGeneralNalHrdParametersPresentFlag())
          || (pcSlice->getSPS()->getGeneralHrdParameters()->getGeneralVclHrdParametersPresentFlag())) &&
          (pcSlice->getSPS()->getGeneralHrdParameters()->getGeneralDecodingUnitHrdParamsPresentFlag()))
        {
          uint32_t numNalus = 0;
          uint32_t numRBSPBytes = 0;
          for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
          {
            numRBSPBytes += uint32_t((*it)->m_nalUnitData.str().size());
            numNalus ++;
          }
          duData.push_back(DUData());
          duData.back().accumBitsDU = ( numRBSPBytes << 3 );
          duData.back().accumNalsDU = numNalus;
        }
        if (pcSlice->isLastSliceInSubpic())
        {
          // Check picture level encoding constraints/requirements
          ProfileLevelTierFeatures profileLevelTierFeatures;
          profileLevelTierFeatures.extractPTLInformation(*(pcSlice->getSPS()));
          sumZeroWords += cabac_zero_word_padding(pcSlice, pcPic, subPicStats[subpicIdx].numBinsWritten, subPicStats[subpicIdx].numBytesInVclNalUnits, 0,
                                                  accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled(), profileLevelTierFeatures);
        }
      } // end iteration over slices

      {
        // Check picture level encoding constraints/requirements
        ProfileLevelTierFeatures profileLevelTierFeatures;
        profileLevelTierFeatures.extractPTLInformation(*(pcSlice->getSPS()));
        validateMinCrRequirements(profileLevelTierFeatures, numBytesInVclNalUnits, pcPic, m_pcCfg);
        // cabac_zero_words processing
        cabac_zero_word_padding(pcSlice, pcPic, binCountsInNalUnits, numBytesInVclNalUnits, sumZeroWords, accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled(), profileLevelTierFeatures);
      }

      //-- For time output for each slice
      auto elapsed = std::chrono::steady_clock::now() - beforeTime;
      auto encTime = std::chrono::duration_cast<std::chrono::seconds>( elapsed ).count();

      std::string digestStr;
      if (m_pcCfg->getDecodedPictureHashSEIType()!=HASHTYPE_NONE)
      {
        SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
        PelUnitBuf recoBuf = pcPic->cs->getRecoBuf();
        m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, recoBuf, digestStr, pcSlice->getSPS()->getBitDepths());
        trailingSeiMessages.push_back(decodedPictureHashSei);
      }
      // create per-subpicture decoded picture hash SEI messages, if more than one subpicture is enabled
      const PPS* pps = pcPic->cs->pps;
      const int numSubpics = pps->getNumSubPics();
      std::string subPicDigest;
      if (numSubpics > 1 && m_pcCfg->getSubpicDecodedPictureHashType() != HASHTYPE_NONE )
      {
        for (int subPicIdx = 0; subPicIdx < numSubpics; subPicIdx++)
        {
          const SubPic& subpic = pps->getSubPic(subPicIdx);
          const UnitArea area = UnitArea(pcSlice->getSPS()->getChromaFormatIdc(), Area(subpic.getSubPicLeft(), subpic.getSubPicTop(), subpic.getSubPicWidthInLumaSample(), subpic.getSubPicHeightInLumaSample()));
          PelUnitBuf recoBuf = pcPic->cs->getRecoBuf(area);
          SEIDecodedPictureHash *decodedPictureHashSEI = new SEIDecodedPictureHash();
          m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSEI, recoBuf, subPicDigest, pcSlice->getSPS()->getBitDepths());
          SEIMessages nestedSEI;
          nestedSEI.push_back(decodedPictureHashSEI);
          const std::vector<uint16_t> subPicIds = { (uint16_t)subpic.getSubPicID() };
          std::vector<int> targetOLS;
          std::vector<int> targetLayers = {pcPic->layerId};
          xCreateScalableNestingSEI(trailingSeiMessages, nestedSEI, targetOLS, targetLayers, subPicIds);
        }
      }

      m_pcCfg->setEncodedFlag(iGOPid, true);

      double PSNR_Y;
      xCalculateAddPSNRs(isField, isTff, iGOPid, pcPic, accessUnit, rcListPic, encTime, snr_conversion, 
        printFrameMSE, printMSSSIM, &PSNR_Y, isEncodeLtRef );


      xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->getTLayer());

      printHash(m_pcCfg->getDecodedPictureHashSEIType(), digestStr);

      if ( m_pcCfg->getUseRateCtrl() )
      {
        double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
        double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
        if ( avgLambda < 0.0 )
        {
          avgLambda = lambda;
        }

        m_pcRateCtrl->getRCPic()->updateAfterPicture( actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->isIRAP());
        m_pcRateCtrl->getRCPic()->addToPictureLsit( m_pcRateCtrl->getPicList() );

        m_pcRateCtrl->getRCSeq()->updateAfterPic( actualTotalBits );
        if ( !pcSlice->isIRAP() )
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture( actualTotalBits );
        }
        else    // for intra picture, the estimated bits are used to update the current status in the GOP
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture( estimatedBits );
        }
  #if U0132_TARGET_BITS_SATURATION
        if (m_pcRateCtrl->getCpbSaturationEnabled())
        {
          m_pcRateCtrl->updateCpbState(actualTotalBits);
          msg(NOTICE_, " [CPB %6d bits]", m_pcRateCtrl->getCpbState() );
        }
  #endif
      }
      xCreateFrameFieldInfoSEI( leadingSeiMessages, pcSlice, isField );
      xCreatePictureTimingSEI( m_pcCfg->getEfficientFieldIRAPEnabled() ? effFieldIRAPMap.GetIRAPGOPid() : 0, leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData );

      if (m_pcCfg->getScalableNestingSEIEnabled())
      {
        const SPS* sps = pcSlice->getSPS();
        const PPS* pps = pcSlice->getPPS();

        std::vector<uint16_t> subpicIDs;
        if (sps->getSubPicInfoPresentFlag())
        {
          if(sps->getSubPicIdMappingExplicitlySignalledFlag())
          {
            if(sps->getSubPicIdMappingPresentFlag())
            {
              subpicIDs = sps->getSubPicIds();
            }
            else
            {
              subpicIDs = pps->getSubPicIds();
            }
          }
          else
          {
            const int numSubPics = sps->getNumSubPics();
            subpicIDs.resize(numSubPics);
            for (int i = 0 ; i < numSubPics; i++)
            {
              subpicIDs[i] = (uint16_t) i;
            }
          }
        }
        // Note (KJS): Using targetOLS = 0, 1 is as random as encapsulating the same SEIs in scalable nesting.
        //             This can just be seen as example regarding how to write scalable nesting, not what to write.
        std::vector<int> targetOLS = {0, 1};
        std::vector<int> targetLayers;
        xCreateScalableNestingSEI(leadingSeiMessages, nestedSeiMessages, targetOLS, targetLayers, subpicIDs);
      }

      xWriteLeadingSEIMessages( leadingSeiMessages, duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData );
      xWriteDuSEIMessages( duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), duData );

      m_AUWriterIf->outputAU( accessUnit );

      msg(NOTICE_, "\n" );
      fflush( stdout );
    }


    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 0 ) ) );

    pcPic->reconstructed = true;
    m_bFirst = false;
    m_iNumPicCoded++;
    if (!(m_pcCfg->getUseCompositeRef() && isEncodeLtRef))
    {
      for( int i = pcSlice->getTLayer() ; i < pcSlice->getSPS()->getMaxTLayers() ; i ++ )
      {
        m_totalCoded[i]++;
      }
    }
    /* logging: insert a newline at end of picture period */

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
    }

    pcPic->destroyTempBuffers();
    pcPic->cs->destroyCoeffs();
    pcPic->cs->releaseIntermediateData();
  } // iGOPid-loop

  delete pcBitstreamRedirect;

  CHECK_( m_iNumPicCoded > 1, "Unspecified error" );
}

void EncGOP::printOutSummary( uint32_t uiNumAllPicCoded, bool isField, const bool printMSEBasedSNR, 
  const bool printSequenceMSE, const bool printMSSSIM, const bool printHexPsnr, const bool printRprPSNR, 
  const BitDepths &bitDepths )
{
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
#if WCG_WPSNR
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcCfg->getLmcs() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ);
#endif

  if( m_pcCfg->getDecodeBitstream(0).empty() && m_pcCfg->getDecodeBitstream(1).empty() && !m_pcCfg->useFastForwardToPOC() )
  {
    CHECK_( !( uiNumAllPicCoded == m_gcAnalyzeAll.getNumPic() ), "Unspecified error" );
  }

  //--CFG_KDY
  const int rateMultiplier=(isField?2:1);
  m_gcAnalyzeAll.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeI.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeP.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeB.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.setFrmRate(m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  }
#endif

  const ChromaFormat chFmt = m_pcCfg->getChromaFormatIdc();

  //-- all
  msg(INFO_, "\n" );
  msg(DETAILS_,"\nSUMMARY --------------------------------------------------------\n" );
#if JVET_O0756_CALCULATE_HDRMETRICS
  const bool calculateHdrMetrics = m_pcEncLib->getCalcluateHdrMetrics();
#endif
#if ENABLE_QPA
  m_gcAnalyzeAll.printOut( 'a', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, printHexPsnr, 
    printRprPSNR, bitDepths, useWPSNR
#if JVET_O0756_CALCULATE_HDRMETRICS
                          , calculateHdrMetrics
#endif
                          );
#else
  m_gcAnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, printHexPsnr, bitDepths
#if JVET_O0756_CALCULATE_HDRMETRICS
                          , calculateHdrMetrics
#endif
                          );
#endif
  msg(DETAILS_, "\n\nI Slices--------------------------------------------------------\n" );
  m_gcAnalyzeI.printOut( 'i', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, 
    printHexPsnr, printRprPSNR, bitDepths );

  msg(DETAILS_, "\n\nP Slices--------------------------------------------------------\n" );
  m_gcAnalyzeP.printOut( 'p', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM,
    printHexPsnr, printRprPSNR, bitDepths );

  msg(DETAILS_, "\n\nB Slices--------------------------------------------------------\n" );
  m_gcAnalyzeB.printOut( 'b', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, 
    printHexPsnr, printRprPSNR, bitDepths );

#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    msg(DETAILS_, "\nWPSNR SUMMARY --------------------------------------------------------\n");
    m_gcAnalyzeWPSNR.printOut( 'w', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM,
      printHexPsnr, printRprPSNR, bitDepths, useLumaWPSNR );
  }
#endif
  if (!m_pcCfg->getSummaryOutFilename().empty())
  {
    m_gcAnalyzeAll.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
  }

  if (!m_pcCfg->getSummaryPicFilenameBase().empty())
  {
    m_gcAnalyzeI.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"I.txt");
    m_gcAnalyzeP.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"P.txt");
    m_gcAnalyzeB.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"B.txt");
  }

#if WCG_WPSNR
  if (!m_pcCfg->getSummaryOutFilename().empty() && useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
  }
#endif
  if(isField)
  {
    //-- interlaced summary
    m_gcAnalyzeAll_in.setFrmRate( m_pcCfg->getFrameRate() / (double)m_pcCfg->getTemporalSubsampleRatio());
    m_gcAnalyzeAll_in.setBits(m_gcAnalyzeAll.getBits());
    // prior to the above statement, the interlace analyser does not contain the correct total number of bits.

    msg(INFO_,"\n\nSUMMARY INTERLACED ---------------------------------------------\n" );
#if ENABLE_QPA
    m_gcAnalyzeAll_in.printOut( 'a', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM,
      printHexPsnr, printRprPSNR, bitDepths, useWPSNR );
#else
    m_gcAnalyzeAll_in.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM,
      printHexPsnr, bitDepths);
#endif
    if (!m_pcCfg->getSummaryOutFilename().empty())
    {
      m_gcAnalyzeAll_in.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
#if WCG_WPSNR
      if (useLumaWPSNR)
      {
        m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
      }
#endif
    }
  }

  msg(DETAILS_,"\nRVM: %.3lf\n", xCalculateRVM() );
}

#if W0038_DB_OPT
uint64_t EncGOP::preLoopFilterPicAndCalcDist( Picture* pcPic )
{
  CodingStructure& cs = *pcPic->cs;
  m_pcLoopFilter->loopFilterPic( cs );

  const CPelUnitBuf picOrg = pcPic->getRecoBuf();
  const CPelUnitBuf picRec = cs.getRecoBuf();

  uint64_t uiDist = 0;
  for( uint32_t comp = 0; comp < (uint32_t)picRec.bufs.size(); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const uint32_t rshift = 2 * DISTORTION_PRECISION_ADJUSTMENT(cs.sps->getBitDepth(toChannelType(compID)));
#if ENABLE_QPA
    CHECK_( rshift >= 8, "shifts greater than 7 are not supported." );
#endif
    uiDist += xFindDistortionPlane( picOrg.get(compID), picRec.get(compID), rshift );
  }
  return uiDist;
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
void EncGOP::xInitGOP( int iPOCLast, int iNumPicRcvd, bool isField
  , bool isEncodeLtRef
)
{
  CHECK_(!( iNumPicRcvd > 0 ), "Unspecified error");
  //  Exception for the first frames
  if ((isField && (iPOCLast == 0 || iPOCLast == 1)) || (!isField && (iPOCLast == 0)) || isEncodeLtRef)
  {
    m_iGopSize    = 1;
  }
  else
  {
    m_iGopSize    = m_pcCfg->getGOPSize();
  }
  CHECK_(!(m_iGopSize > 0), "Unspecified error");

  return;
}


void EncGOP::xGetBuffer( PicList&                  rcListPic,
                         std::list<PelUnitBuf*>&   rcListPicYuvRecOut,
                         int                       iNumPicRcvd,
                         int                       iTimeOffset,
                         Picture*&                 rpcPic,
                         int                       pocCurr,
                         bool                      isField )
{
  int i;
  //  Rec. output
  std::list<PelUnitBuf*>::iterator     iterPicYuvRec = rcListPicYuvRecOut.end();

  if (isField && pocCurr > 1 && m_iGopSize!=1)
  {
    iTimeOffset--;
  }

  int multipleFactor = m_pcCfg->getUseCompositeRef() ? 2 : 1;
  for (i = 0; i < (iNumPicRcvd * multipleFactor - iTimeOffset + 1); i += multipleFactor)
  {
    iterPicYuvRec--;
  }

  //  Current pic.
  PicList::iterator        iterPic       = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic);
    if( rpcPic->getPOC() == pocCurr && rpcPic->layerId == m_pcEncLib->getLayerId() )
    {
      break;
    }
    iterPic++;
  }

  CHECK_(!(rpcPic != NULL), "Unspecified error");
  CHECK_(!(rpcPic->getPOC() == pocCurr), "Unspecified error");

  (**iterPicYuvRec) = rpcPic->getRecoBuf();
  return;
}

#if ENABLE_QPA

#ifndef BETA
  #define BETA 0.5 // value between 0.0 and 1; use 0.0 to obtain traditional PSNR
#endif

static inline double calcWeightedSquaredError(const CPelBuf& org,        const CPelBuf& rec,
                                              double &sumAct,            const uint32_t bitDepth,
                                              const uint32_t imageWidth, const uint32_t imageHeight,
                                              const uint32_t offsetX,    const uint32_t offsetY,
                                              int blockWidth,            int blockHeight)
{
  const int    O = org.stride;
  const int    R = rec.stride;
  const Pel   *o = org.bufAt(offsetX, offsetY);
  const Pel   *r = rec.bufAt(offsetX, offsetY);
  const int yAct = offsetY > 0 ? 0 : 1;
  const int xAct = offsetX > 0 ? 0 : 1;

  if (offsetY + (uint32_t)blockHeight > imageHeight) blockHeight = imageHeight - offsetY;
  if (offsetX + (uint32_t)blockWidth  > imageWidth ) blockWidth  = imageWidth  - offsetX;

  const int hAct = offsetY + (uint32_t)blockHeight < imageHeight ? blockHeight : blockHeight - 1;
  const int wAct = offsetX + (uint32_t)blockWidth  < imageWidth  ? blockWidth  : blockWidth  - 1;
  uint64_t ssErr = 0; // sum of squared diffs
  uint64_t saAct = 0; // sum of abs. activity
  double msAct;
  int x, y;

  // calculate image differences and activity
  for (y = 0; y < blockHeight; y++)  // error
  {
    for (x = 0; x < blockWidth; x++)
    {
      const     int64_t iDiff = (int64_t)o[y*O + x] - (int64_t)r[y*R + x];
      ssErr += uint64_t(iDiff * iDiff);
    }
  }
  if (wAct <= xAct || hAct <= yAct) return (double)ssErr;

  for (y = yAct; y < hAct; y++)   // activity
  {
    for (x = xAct; x < wAct; x++)
    {
      const int f = 12 * (int)o[y*O + x] - 2 * ((int)o[y*O + x-1] + (int)o[y*O + x+1] + (int)o[(y-1)*O + x] + (int)o[(y+1)*O + x])
                       - (int)o[(y-1)*O + x-1] - (int)o[(y-1)*O + x+1] - (int)o[(y+1)*O + x-1] - (int)o[(y+1)*O + x+1];
      saAct += abs(f);
    }
  }

  // calculate weight (mean squared activity)
  msAct = (double)saAct / (double(wAct - xAct) * double(hAct - yAct));

  // lower limit, accounts for high-pass gain
  if (msAct < double(1 << (bitDepth - 4))) msAct = double(1 << (bitDepth - 4));

  msAct *= msAct; // because ssErr is squared

  sumAct += msAct; // includes high-pass gain

  // calculate activity weighted error square
  return (double)ssErr * pow(msAct, -1.0 * BETA);
}
#endif // ENABLE_QPA

uint64_t EncGOP::xFindDistortionPlane(const CPelBuf& pic0, const CPelBuf& pic1, const uint32_t rshift
#if ENABLE_QPA
                                    , const uint32_t chromaShiftHor /*= 0*/, const uint32_t chromaShiftVer /*= 0*/
#endif
                                      )
{
  uint64_t uiTotalDiff;
  const  Pel*  pSrc0 = pic0.bufAt(0, 0);
  const  Pel*  pSrc1 = pic1.bufAt(0, 0);

  CHECK_(pic0.width  != pic1.width , "Unspecified error");
  CHECK_(pic0.height != pic1.height, "Unspecified error");

  if( rshift > 0 )
  {
#if ENABLE_QPA
    const   uint32_t  BD = rshift;      // image bit-depth
    if (BD >= 8)
    {
      const uint32_t   W = pic0.width;  // image width
      const uint32_t   H = pic0.height; // image height
      const double     R = double(W * H) / (1920.0 * 1080.0);
      const uint32_t   B = Clip3<uint32_t>(0, 128 >> chromaShiftVer, 4 * uint32_t(16.0 * sqrt(R) + 0.5)); // WPSNR block size in integer multiple of 4 (for SIMD, = 64 at full-HD)

      uint32_t x, y;

      if (B < 4) // image is too small to use WPSNR, resort to traditional PSNR
      {
        uiTotalDiff = 0;
        for (y = 0; y < H; y++)
        {
          for (x = 0; x < W; x++)
          {
            const           int64_t iDiff = (int64_t)pSrc0[x] - (int64_t)pSrc1[x];
            uiTotalDiff += uint64_t(iDiff * iDiff);
          }
          pSrc0 += pic0.stride;
          pSrc1 += pic1.stride;
        }
        return uiTotalDiff;
      }

      double wmse = 0.0, sumAct = 0.0; // compute activity normalized SNR value

      for (y = 0; y < H; y += B)
      {
        for (x = 0; x < W; x += B)
        {
          wmse += calcWeightedSquaredError(pic1,   pic0,
                                           sumAct, BD,
                                           W,      H,
                                           x,      y,
                                           B,      B);
        }
      }

      // integer weighted distortion
      sumAct = 16.0 * sqrt ((3840.0 * 2160.0) / double((W << chromaShiftHor) * (H << chromaShiftVer))) * double(1 << BD);

      return (wmse <= 0.0) ? 0 : uint64_t(wmse * pow(sumAct, BETA) + 0.5);
    }
#endif // ENABLE_QPA
    uiTotalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += uint64_t((iTemp * iTemp) >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }
  else
  {
    uiTotalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += uint64_t(iTemp * iTemp);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }

  return uiTotalDiff;
}
#if WCG_WPSNR
double EncGOP::xFindDistortionPlaneWPSNR(const CPelBuf& pic0, const CPelBuf& pic1, const uint32_t rshift, const CPelBuf& picLuma0,
  ComponentID compID, const ChromaFormat chfmt    )
{
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcCfg->getLmcs() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ);
  if (!useLumaWPSNR)
  {
    return 0;
  }

  double uiTotalDiffWPSNR;
  const  Pel*  pSrc0 = pic0.bufAt(0, 0);
  const  Pel*  pSrc1 = pic1.bufAt(0, 0);
  const  Pel*  pSrcLuma = picLuma0.bufAt(0, 0);
  CHECK_(pic0.width  != pic1.width , "Unspecified error");
  CHECK_(pic0.height != pic1.height, "Unspecified error");

  if( rshift > 0 )
  {
    uiTotalDiffWPSNR = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        double dW = m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[(x << getComponentScaleX(compID, chfmt))]);
        uiTotalDiffWPSNR += ((dW * (double)iTemp * (double)iTemp)) * (double)(1 >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }
  else
  {
    uiTotalDiffWPSNR = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        double dW = m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[x << getComponentScaleX(compID, chfmt)]);
        uiTotalDiffWPSNR += dW * (double)iTemp * (double)iTemp;
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }

  return uiTotalDiffWPSNR;
}
#endif

void EncGOP::xCalculateAddPSNRs( const bool isField, const bool isFieldTopFieldFirst, 
  const int iGOPid, Picture* pcPic, const AccessUnit&accessUnit, PicList &rcListPic, 
  const int64_t dEncTime, const InputColourSpaceConversion snr_conversion, 
  const bool printFrameMSE, const bool printMSSSIM, double* PSNR_Y, bool isEncodeLtRef)
{
  xCalculateAddPSNR(pcPic, pcPic->getRecoBuf(), accessUnit, (double)dEncTime, snr_conversion, 
    printFrameMSE, printMSSSIM, PSNR_Y, isEncodeLtRef);

  //In case of field coding, compute the interlaced PSNR for both fields
  if(isField)
  {
    bool bothFieldsAreEncoded = false;
    int correspondingFieldPOC = pcPic->getPOC();
    int currentPicGOPPoc = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    if(pcPic->getPOC() == 0)
    {
      // particular case for POC 0 and 1.
      // If they are not encoded first and separately from other pictures, we need to change this
      // POC 0 is always encoded first then POC 1 is encoded
      bothFieldsAreEncoded = false;
    }
    else if(pcPic->getPOC() == 1)
    {
      // if we are at POC 1, POC 0 has been encoded for sure
      correspondingFieldPOC = 0;
      bothFieldsAreEncoded = true;
    }
    else
    {
      if(pcPic->getPOC()%2 == 1)
      {
        correspondingFieldPOC -= 1; // all odd POC are associated with the preceding even POC (e.g poc 1 is associated to poc 0)
        currentPicGOPPoc      -= 1;
      }
      else
      {
        correspondingFieldPOC += 1; // all even POC are associated with the following odd POC (e.g poc 0 is associated to poc 1)
        currentPicGOPPoc      += 1;
      }
      for(int i = 0; i < m_iGopSize; i ++)
      {
        if(m_pcCfg->getGOPEntry(i).m_POC == currentPicGOPPoc)
        {
          bothFieldsAreEncoded = m_pcCfg->getGOPEntry(i).m_isEncoded;
          break;
        }
      }
    }

    if(bothFieldsAreEncoded)
    {
      //get complementary top field
      PicList::iterator   iterPic = rcListPic.begin();
      while ((*iterPic)->getPOC() != correspondingFieldPOC)
      {
        iterPic ++;
      }
      Picture* correspondingFieldPic = *(iterPic);

      if ((pcPic->topField && isFieldTopFieldFirst) || (!pcPic->topField && !isFieldTopFieldFirst))
      {
        xCalculateInterlacedAddPSNR(pcPic, correspondingFieldPic, pcPic->getRecoBuf(), 
          correspondingFieldPic->getRecoBuf(), snr_conversion, printFrameMSE, printMSSSIM, 
          PSNR_Y, isEncodeLtRef);
      }
      else
      {
        xCalculateInterlacedAddPSNR(correspondingFieldPic, pcPic, correspondingFieldPic->getRecoBuf(), 
          pcPic->getRecoBuf(), snr_conversion, printFrameMSE, printMSSSIM, PSNR_Y, isEncodeLtRef);
      }
    }
  }
}

void EncGOP::xCalculateAddPSNR(Picture* pcPic, PelUnitBuf cPicD, const AccessUnit& accessUnit, 
  double dEncTime, const InputColourSpaceConversion conversion, const bool printFrameMSE, const bool printMSSSIM,
  double* PSNR_Y, bool isEncodeLtRef)
{
  const SPS&         sps = *pcPic->cs->sps;
  const CPelUnitBuf& pic = cPicD;
  CHECK_(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
//  const CPelUnitBuf& org = (conversion != IPCOLOURSPACE_UNCHANGED) ? pcPic->getPicYuvTrueOrg()->getBuf() : pcPic->getPicYuvOrg()->getBuf();
  const CPelUnitBuf& org = (sps.getUseLmcs() || m_pcCfg->getGopBasedTemporalFilterEnabled()) ? pcPic->getTrueOrigBuf() : pcPic->getOrigBuf();
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
  double  dPSNR[MAX_NUM_COMPONENT];
  double msssim[MAX_NUM_COMPONENT] = {0.0};
#if WCG_WPSNR
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcCfg->getLmcs() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ);
  double  dPSNRWeighted[MAX_NUM_COMPONENT];
  double  MSEyuvframeWeighted[MAX_NUM_COMPONENT];
#endif
  double  upscaledPSNR[MAX_NUM_COMPONENT];
  for(int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
#if WCG_WPSNR
    dPSNRWeighted[i]=0.0;
    MSEyuvframeWeighted[i] = 0.0;
#endif
    upscaledPSNR[i] = 0.0;
  }
#if JVET_O0756_CALCULATE_HDRMETRICS
  double deltaE[hdrtoolslib::NB_REF_WHITE];
  double psnrL[hdrtoolslib::NB_REF_WHITE];
  for (int i=0; i<hdrtoolslib::NB_REF_WHITE; i++)
  {
    deltaE[i] = 0.0;
    psnrL[i] = 0.0;
  }
#endif

  PelStorage interm;

  if (conversion != IPCOLOURSPACE_UNCHANGED)
  {
    interm.create(pic.chromaFormat, Area(Position(), pic.Y()));
    VideoIOYuv::ColourSpaceConvert(pic, interm, conversion, false);
  }

  const CPelUnitBuf& picC = (conversion == IPCOLOURSPACE_UNCHANGED) ? pic : interm;

  //===== calculate PSNR =====
  double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};
  const ChromaFormat formatD = pic.chromaFormat;
  const ChromaFormat format  = sps.getChromaFormatIdc();

  const bool bPicIsField     = pcPic->fieldPic;
  const Slice*  pcSlice      = pcPic->slices[0];

  PelStorage upscaledRec;

  if (m_pcEncLib->isResChangeInClvsEnabled())
  {
    const CPelBuf& upscaledOrg = (sps.getUseLmcs() || m_pcCfg->getGopBasedTemporalFilterEnabled()) ? pcPic->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT).get( COMPONENT_Y ) : pcPic->M_BUFS( 0, PIC_ORIGINAL_INPUT).get( COMPONENT_Y );
    upscaledRec.create( pic.chromaFormat, Area( Position(), upscaledOrg ) );

    int xScale, yScale;
    // it is assumed that full resolution picture PPS has ppsId 0
    const PPS* pps = m_pcEncLib->getPPS(0);
    CU::getRprScaling( &sps, pps, pcPic, xScale, yScale );
    std::pair<int, int> scalingRatio = std::pair<int, int>( xScale, yScale );

    Picture::rescalePicture( scalingRatio, picC, pcPic->getScalingWindow(), upscaledRec, pps->getScalingWindow(), format, sps.getBitDepths(), false, false, sps.getHorCollocatedChromaFlag(), sps.getVerCollocatedChromaFlag() );
  }

  for (int comp = 0; comp < ::getNumberValidComponents(formatD); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const CPelBuf&    p = picC.get(compID);
    const CPelBuf&    o = org.get(compID);

    CHECK_(!( p.width  == o.width), "Unspecified error");
    CHECK_(!( p.height == o.height), "Unspecified error");

    int padX = m_pcEncLib->getPad( 0 );
    int padY = m_pcEncLib->getPad( 1 );

    // when RPR is enabled, picture padding is picture specific due to possible different picture resoluitons, however only full resolution padding is stored in EncLib
    // get per picture padding from the conformance window, in this case if conformance window is set not equal to the padding then PSNR results may be inaccurate
    if (m_pcEncLib->isResChangeInClvsEnabled())
    {
      Window& conf = pcPic->getConformanceWindow();
      padX = conf.getWindowRightOffset() * SPS::getWinUnitX( format );
      padY = conf.getWindowBottomOffset() * SPS::getWinUnitY( format );
    }

    const uint32_t width = p.width - ( padX >> ::getComponentScaleX( compID, format ) );
    const uint32_t height = p.height - ( padY >> ( !!bPicIsField + ::getComponentScaleY( compID, format ) ) );

    // create new buffers with correct dimensions
    const CPelBuf recPB(p.bufAt(0, 0), p.stride, width, height);
    const CPelBuf orgPB(o.bufAt(0, 0), o.stride, width, height);
    const uint32_t    bitDepth = sps.getBitDepth(toChannelType(compID));
#if ENABLE_QPA
    const uint64_t uiSSDtemp = xFindDistortionPlane(recPB, orgPB, useWPSNR ? bitDepth : 0, ::getComponentScaleX(compID, format), ::getComponentScaleY(compID, format));
#else
    const uint64_t uiSSDtemp = xFindDistortionPlane(recPB, orgPB, 0);
#endif
    const uint32_t maxval = 255 << (bitDepth - 8);
    const uint32_t size   = width * height;
    const double fRefValue = (double)maxval * maxval * size;
    dPSNR[comp]       = uiSSDtemp ? 10.0 * log10(fRefValue / (double)uiSSDtemp) : 999.99;
    MSEyuvframe[comp] = (double)uiSSDtemp / size;
    if(printMSSSIM)
    {
      msssim[comp] = xCalculateMSSSIM (o.bufAt(0, 0), o.stride, p.bufAt(0, 0), p.stride, width, height, bitDepth);
    }
#if WCG_WPSNR
    const double uiSSDtempWeighted = xFindDistortionPlaneWPSNR(recPB, orgPB, 0, org.get(COMPONENT_Y), compID, format);
    if (useLumaWPSNR)
    {
      dPSNRWeighted[comp] = uiSSDtempWeighted ? 10.0 * log10(fRefValue / (double)uiSSDtempWeighted) : 999.99;
      MSEyuvframeWeighted[comp] = (double)uiSSDtempWeighted / size;
    }
#endif


    if (m_pcEncLib->isResChangeInClvsEnabled())
    {
      const CPelBuf& upscaledOrg = (sps.getUseLmcs() || m_pcCfg->getGopBasedTemporalFilterEnabled()) ? pcPic->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT).get( compID ) : pcPic->M_BUFS( 0, PIC_ORIGINAL_INPUT).get( compID );

      const uint32_t upscaledWidth = upscaledOrg.width - ( m_pcEncLib->getPad( 0 ) >> ::getComponentScaleX( compID, format ) );
      const uint32_t upscaledHeight = upscaledOrg.height - ( m_pcEncLib->getPad( 1 ) >> ( !!bPicIsField + ::getComponentScaleY( compID, format ) ) );

      // create new buffers with correct dimensions
      const CPelBuf upscaledRecPB( upscaledRec.get( compID ).bufAt( 0, 0 ), upscaledRec.get( compID ).stride, upscaledWidth, upscaledHeight );
      const CPelBuf upscaledOrgPB( upscaledOrg.bufAt( 0, 0 ), upscaledOrg.stride, upscaledWidth, upscaledHeight );

#if ENABLE_QPA
      const uint64_t upscaledSSD = xFindDistortionPlane( upscaledRecPB, upscaledOrgPB, useWPSNR ? bitDepth : 0, ::getComponentScaleX( compID, format ) );
#else
      const uint64_t scaledSSD = xFindDistortionPlane( upsacledRecPB, upsacledOrgPB, 0 );
#endif

      upscaledPSNR[comp] = upscaledSSD ? 10.0 * log10( (double)maxval * maxval * upscaledWidth * upscaledHeight / (double)upscaledSSD ) : 999.99;
    }
  }

#if EXTENSION_360_VIDEO
  m_ext360.calculatePSNRs(pcPic);
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  const bool calculateHdrMetrics = m_pcEncLib->getCalcluateHdrMetrics();
  if (calculateHdrMetrics)
  {
    auto beforeTime = std::chrono::steady_clock::now();
    xCalculateHDRMetrics(pcPic, deltaE, psnrL);
    auto elapsed = std::chrono::steady_clock::now() - beforeTime;
    m_metricTime += elapsed;
  }
#endif

  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  uint32_t numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    uint32_t numRBSPBytes_nal = uint32_t((*it)->m_nalUnitData.str().size());
    if (m_pcCfg->getSummaryVerboseness() > 0)
    {
      msg(NOTICE_, "*** %6s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);
    }
    if( ( *it )->m_nalUnitType != NAL_UNIT_PREFIX_SEI && ( *it )->m_nalUnitType != NAL_UNIT_SUFFIX_SEI )
    {
      numRBSPBytes += numRBSPBytes_nal;
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
      if (it == accessUnit.begin() || (*it)->m_nalUnitType == NAL_UNIT_OPI || (*it)->m_nalUnitType == NAL_UNIT_VPS || (*it)->m_nalUnitType == NAL_UNIT_DCI || (*it)->m_nalUnitType == NAL_UNIT_SPS || (*it)->m_nalUnitType == NAL_UNIT_PPS || (*it)->m_nalUnitType == NAL_UNIT_PREFIX_APS || (*it)->m_nalUnitType == NAL_UNIT_SUFFIX_APS)
#else
      if (it == accessUnit.begin() || (*it)->m_nalUnitType == NAL_UNIT_VPS || (*it)->m_nalUnitType == NAL_UNIT_DCI || (*it)->m_nalUnitType == NAL_UNIT_SPS || (*it)->m_nalUnitType == NAL_UNIT_PPS || (*it)->m_nalUnitType == NAL_UNIT_PREFIX_APS || (*it)->m_nalUnitType == NAL_UNIT_SUFFIX_APS)
#endif
      {
        numRBSPBytes += 4;
      }
      else
      {
        numRBSPBytes += 3;
      }
    }
  }

  uint32_t uibits = numRBSPBytes * 8;
  m_vRVM_RP.push_back( uibits );

  //===== add PSNR =====
  m_gcAnalyzeAll.addResult(dPSNR, (double)uibits, MSEyuvframe, upscaledPSNR, msssim, isEncodeLtRef);
#if EXTENSION_360_VIDEO
  m_ext360.addResult(m_gcAnalyzeAll);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
  if (calculateHdrMetrics)
  {
    m_gcAnalyzeAll.addHDRMetricsResult(deltaE, psnrL);
  }
#endif
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI.addResult(dPSNR, (double)uibits, MSEyuvframe, upscaledPSNR, msssim, isEncodeLtRef);
    *PSNR_Y = dPSNR[COMPONENT_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeI);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if (calculateHdrMetrics)
    {
      m_gcAnalyzeI.addHDRMetricsResult(deltaE, psnrL);
    }
#endif
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP.addResult(dPSNR, (double)uibits, MSEyuvframe, upscaledPSNR, msssim, isEncodeLtRef);
    *PSNR_Y = dPSNR[COMPONENT_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeP);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if (calculateHdrMetrics)
    {
      m_gcAnalyzeP.addHDRMetricsResult(deltaE, psnrL);
    }
#endif
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB.addResult(dPSNR, (double)uibits, MSEyuvframe, upscaledPSNR, msssim, isEncodeLtRef);
    *PSNR_Y = dPSNR[COMPONENT_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeB);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if (calculateHdrMetrics)
    {
      m_gcAnalyzeB.addHDRMetricsResult(deltaE, psnrL);
    }
#endif
  }
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.addResult( dPSNRWeighted, (double)uibits, MSEyuvframeWeighted, upscaledPSNR, msssim, isEncodeLtRef );
  }
#endif

  char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (! pcPic->referenced)
  {
    c += 32;
  }
  if (m_pcCfg->getDependentRAPIndicationSEIEnabled() && pcSlice->isDRAP()) c = 'D';

  if( g_verbosity >= NOTICE_)
  {
    msg(NOTICE_, "POC %4d LId: %2d TId: %1d ( %s, %c-SLICE, QP %d ) %10d bits",
         pcSlice->getPOC(),
         pcSlice->getPic()->layerId,
         pcSlice->getTLayer(),
         nalUnitTypeToString(pcSlice->getNalUnitType()),
         c,
         pcSlice->getSliceQp(),
         uibits );

    msg(NOTICE_, " [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );

#if EXTENSION_360_VIDEO
    m_ext360.printPerPOCInfo(NOTICE_);
#endif

    if (m_pcEncLib->getPrintHexPsnr())
    {
      uint64_t xPsnr[MAX_NUM_COMPONENT];
      for (int i = 0; i < MAX_NUM_COMPONENT; i++)
      {
        copy(reinterpret_cast<uint8_t *>(&dPSNR[i]),
             reinterpret_cast<uint8_t *>(&dPSNR[i]) + sizeof(dPSNR[i]),
             reinterpret_cast<uint8_t *>(&xPsnr[i]));
      }
      msg(NOTICE_, " [xY %16" PRIx64 " xU %16" PRIx64 " xV %16" PRIx64 "]", xPsnr[COMPONENT_Y], xPsnr[COMPONENT_Cb], xPsnr[COMPONENT_Cr]);

#if EXTENSION_360_VIDEO
      m_ext360.printPerPOCInfo(NOTICE_, true);
#endif
    }
    if (printMSSSIM)
    {
      msg(NOTICE_, " [MS-SSIM Y %1.6lf    U %1.6lf    V %1.6lf]", msssim[COMPONENT_Y], msssim[COMPONENT_Cb], msssim[COMPONENT_Cr] );
    }  

    if( printFrameMSE )
    {
      msg(NOTICE_, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
    }
#if WCG_WPSNR
    if (useLumaWPSNR)
    {
      msg(NOTICE_, " [WY %6.4lf dB    WU %6.4lf dB    WV %6.4lf dB]", dPSNRWeighted[COMPONENT_Y], dPSNRWeighted[COMPONENT_Cb], dPSNRWeighted[COMPONENT_Cr]);

      if (m_pcEncLib->getPrintHexPsnr())
      {
        uint64_t xPsnrWeighted[MAX_NUM_COMPONENT];
        for (int i = 0; i < MAX_NUM_COMPONENT; i++)
        {
          copy(reinterpret_cast<uint8_t *>(&dPSNRWeighted[i]),
               reinterpret_cast<uint8_t *>(&dPSNRWeighted[i]) + sizeof(dPSNRWeighted[i]),
               reinterpret_cast<uint8_t *>(&xPsnrWeighted[i]));
        }
        msg(NOTICE_, " [xWY %16" PRIx64 " xWU %16" PRIx64 " xWV %16" PRIx64 "]", xPsnrWeighted[COMPONENT_Y], xPsnrWeighted[COMPONENT_Cb], xPsnrWeighted[COMPONENT_Cr]);
      }
    }
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if(calculateHdrMetrics)
    {
      for (int i=0; i<1; i++)
      {
        msg(NOTICE_, " [DeltaE%d %6.4lf dB]", (int)m_pcCfg->getWhitePointDeltaE(i), deltaE[i]);
        if (m_pcEncLib->getPrintHexPsnr())
        {
          int64_t xdeltaE[MAX_NUM_COMPONENT];
          for (int i = 0; i < 1; i++)
          {
            copy(reinterpret_cast<uint8_t *>(&deltaE[i]),
                 reinterpret_cast<uint8_t *>(&deltaE[i]) + sizeof(deltaE[i]),
                 reinterpret_cast<uint8_t *>(&xdeltaE[i]));
          }
          msg(NOTICE_, " [xDeltaE%d %16" PRIx64 "]", (int)m_pcCfg->getWhitePointDeltaE(i), xdeltaE[0]);
        }
      }
      for (int i=0; i<1; i++)
      {
        msg(NOTICE_, " [PSNRL%d %6.4lf dB]", (int)m_pcCfg->getWhitePointDeltaE(i), psnrL[i]);

        if (m_pcEncLib->getPrintHexPsnr())
        {
          int64_t xpsnrL[MAX_NUM_COMPONENT];
          for (int i = 0; i < 1; i++)
          {
            copy(reinterpret_cast<uint8_t *>(&psnrL[i]),
                 reinterpret_cast<uint8_t *>(&psnrL[i]) + sizeof(psnrL[i]),
                 reinterpret_cast<uint8_t *>(&xpsnrL[i]));
          }
          msg(NOTICE_, " [xPSNRL%d %16" PRIx64 "]", (int)m_pcCfg->getWhitePointDeltaE(i), xpsnrL[0]);

        }
      }
    }
#endif
    msg(NOTICE_, " [ET %5.0f ]", dEncTime );

    // msg( SOME, " [WP %d]", pcSlice->getUseWeightedPrediction());

    for( int iRefList = 0; iRefList < 2; iRefList++ )
    {
      msg(NOTICE_, " [L%d", iRefList );
      for( int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx( RefPicList( iRefList ) ); iRefIndex++ )
      {
        const std::pair<int, int>& scaleRatio = pcSlice->getScalingRatio( RefPicList( iRefList ), iRefIndex );

        if( pcPic->cs->picHeader->getEnableTMVPFlag() && pcSlice->getColFromL0Flag() == bool(1 - iRefList) && pcSlice->getColRefIdx() == iRefIndex )
        {
          if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
          {
            msg(NOTICE_, " %dc(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
          }
          else
          {
            msg(NOTICE_, " %dc", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) );
          }
        }
        else
        {
          if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
          {
            msg(NOTICE_, " %d(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
          }
          else
          {
            msg(NOTICE_, " %d", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) );
          }
        }

        if( pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) == pcSlice->getPOC() )
        {
          msg(NOTICE_, ".%d", pcSlice->getRefPic( RefPicList( iRefList ), iRefIndex )->layerId );
        }
      }
      msg(NOTICE_, "]" );
    }
    if (m_pcEncLib->isResChangeInClvsEnabled())
    {
      msg(NOTICE_, "\nPSNR2: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", upscaledPSNR[COMPONENT_Y], upscaledPSNR[COMPONENT_Cb], upscaledPSNR[COMPONENT_Cr] );
    }
  }
  else if( g_verbosity >= INFO_)
  {
    std::cout << "\r\t" << pcSlice->getPOC();
    std::cout.flush();
  }
}

double EncGOP::xCalculateMSSSIM (const Pel* org, const int orgStride, const Pel* rec, const int recStride, const int width, const int height, const uint32_t bitDepth)
{
  const int MAX_MSSSIM_SCALE  = 5;
  const int WEIGHTING_MID_TAP = 5;
  const int WEIGHTING_SIZE    = WEIGHTING_MID_TAP*2+1;

  uint32_t maxScale;

  // For low resolution videos determine number of scales 
  if (width < 22 || height < 22)
  {
    maxScale = 1; 
  }
  else if (width < 44 || height < 44)
  {
    maxScale = 2; 
  }
  else if (width < 88 || height < 88)
  {
    maxScale = 3; 
  }
  else if (width < 176 || height < 176)
  {
    maxScale = 4; 
  }
  else
  {
    maxScale = 5;
  }

  assert(maxScale>0 && maxScale<=MAX_MSSSIM_SCALE);

  //Normalized Gaussian mask design, 11*11, s.d. 1.5
  double weights[WEIGHTING_SIZE][WEIGHTING_SIZE];
  double coeffSum=0.0;
  for(int y=0; y<WEIGHTING_SIZE; y++)
  {
    for(int x=0; x<WEIGHTING_SIZE; x++)
    {
      weights[y][x]=exp(-((y-WEIGHTING_MID_TAP)*(y-WEIGHTING_MID_TAP)+(x-WEIGHTING_MID_TAP)*(x-WEIGHTING_MID_TAP))/(WEIGHTING_MID_TAP-0.5));
      coeffSum +=weights[y][x];
    }
  }

  for(int y=0; y<WEIGHTING_SIZE; y++)
  {
    for(int x=0; x<WEIGHTING_SIZE; x++)
    {
      weights[y][x] /=coeffSum;
    }
  }

  //Resolution based weights
  const double exponentWeights[MAX_MSSSIM_SCALE][MAX_MSSSIM_SCALE] = {{1.0,    0,      0,      0,      0     },
                                                                      {0.1356, 0.8644, 0,      0,      0     },
                                                                      {0.0711, 0.4530, 0.4760, 0,      0     },
                                                                      {0.0517, 0.3295, 0.3462, 0.2726, 0     },
                                                                      {0.0448, 0.2856, 0.3001, 0.2363, 0.1333}};

  //Downsampling of data:
  std::vector<double> original[MAX_MSSSIM_SCALE];
  std::vector<double> recon[MAX_MSSSIM_SCALE];

  for(uint32_t scale=0; scale<maxScale; scale++)
  {
    const int scaledHeight = height >> scale;
    const int scaledWidth  = width  >> scale;
    original[scale].resize(scaledHeight*scaledWidth, double(0));
    recon[scale].resize(scaledHeight*scaledWidth, double(0));
  }

  // Initial [0] arrays to be a copy of the source data (but stored in array "double", not Pel array).
  for(int y=0; y<height; y++)
  {
    for(int x=0; x<width; x++)
    {
      original[0][y*width+x] = org[y*orgStride+x];
      recon[0][   y*width+x] = rec[y*recStride+x];
    }
  }

  // Set up other arrays to be average value of each 2x2 sample.
  for(uint32_t scale=1; scale<maxScale; scale++)
  {
    const int scaledHeight = height >> scale;
    const int scaledWidth  = width  >> scale;
    for(int y=0; y<scaledHeight; y++)
    {
      for(int x=0; x<scaledWidth; x++)
      {
        original[scale][y*scaledWidth+x]= (original[scale-1][ 2*y   *(2*scaledWidth)+2*x  ] +
                                           original[scale-1][ 2*y   *(2*scaledWidth)+2*x+1] +
                                           original[scale-1][(2*y+1)*(2*scaledWidth)+2*x  ] +
                                           original[scale-1][(2*y+1)*(2*scaledWidth)+2*x+1]) / 4.0;
        recon[scale][y*scaledWidth+x]=    (   recon[scale-1][ 2*y   *(2*scaledWidth)+2*x  ] +
                                              recon[scale-1][ 2*y   *(2*scaledWidth)+2*x+1] +
                                              recon[scale-1][(2*y+1)*(2*scaledWidth)+2*x  ] +
                                              recon[scale-1][(2*y+1)*(2*scaledWidth)+2*x+1]) / 4.0;
      }
    }
  }
  
  // Calculate MS-SSIM:
  const uint32_t   maxValue  = (1<<bitDepth)-1;
  const double c1        = (0.01*maxValue)*(0.01*maxValue);
  const double c2        = (0.03*maxValue)*(0.03*maxValue);
  
  double finalMSSSIM = 1.0;

  for(uint32_t scale=0; scale<maxScale; scale++)
  {
    const int scaledHeight    = height >> scale;
    const int scaledWidth     = width  >> scale;
    const int blocksPerRow    = scaledWidth-WEIGHTING_SIZE+1;
    const int blocksPerColumn = scaledHeight-WEIGHTING_SIZE+1;
    const int totalBlocks     = blocksPerRow*blocksPerColumn;

    double meanSSIM= 0.0;

    for(int blockIndexY=0; blockIndexY<blocksPerColumn; blockIndexY++)
    {
      for(int blockIndexX=0; blockIndexX<blocksPerRow; blockIndexX++)
      {
        double muOrg          =0.0;
        double muRec          =0.0;
        double muOrigSqr      =0.0;
        double muRecSqr       =0.0;
        double muOrigMultRec  =0.0;

        for(int y=0; y<WEIGHTING_SIZE; y++)
        {
          for(int x=0;x<WEIGHTING_SIZE; x++)
          {
            const double gaussianWeight=weights[y][x];
            const int    sampleOffset=(blockIndexY+y)*scaledWidth+(blockIndexX+x);
            const double orgPel=original[scale][sampleOffset];
            const double recPel=   recon[scale][sampleOffset];

            muOrg        +=orgPel*       gaussianWeight;
            muRec        +=recPel*       gaussianWeight;
            muOrigSqr    +=orgPel*orgPel*gaussianWeight;
            muRecSqr     +=recPel*recPel*gaussianWeight;
            muOrigMultRec+=orgPel*recPel*gaussianWeight;
          }
        }

        const double sigmaSqrOrig = muOrigSqr    -(muOrg*muOrg);
        const double sigmaSqrRec  = muRecSqr     -(muRec*muRec);
        const double sigmaOrigRec = muOrigMultRec-(muOrg*muRec);

        double blockSSIMVal = ((2.0*sigmaOrigRec + c2)/(sigmaSqrOrig+sigmaSqrRec + c2));
        if(scale == maxScale-1)
        {
          blockSSIMVal*=(2.0*muOrg*muRec + c1)/(muOrg*muOrg+muRec*muRec + c1);
        }

        meanSSIM += blockSSIMVal;
      }
    }

    meanSSIM /=totalBlocks;

    finalMSSSIM *= pow(meanSSIM, exponentWeights[maxScale-1][scale]);
  }

  return finalMSSSIM;
}

#if JVET_O0756_CALCULATE_HDRMETRICS
void EncGOP::xCalculateHDRMetrics( Picture* pcPic, double deltaE[hdrtoolslib::NB_REF_WHITE], double psnrL[hdrtoolslib::NB_REF_WHITE])
{
  copyBuftoFrame(pcPic);

  ChromaFormat chFmt =  pcPic->chromaFormat;

  if (chFmt != CHROMA_444)
  {
    m_pcConvertFormat->process(m_ppcFrameOrg[1], m_ppcFrameOrg[0]);
    m_pcConvertFormat->process(m_ppcFrameRec[1], m_ppcFrameRec[0]);
  }

  m_pcConvertIQuantize->process(m_ppcFrameOrg[2], m_ppcFrameOrg[1]);
  m_pcConvertIQuantize->process(m_ppcFrameRec[2], m_ppcFrameRec[1]);

  m_pcColorTransform->process(m_ppcFrameOrg[3], m_ppcFrameOrg[2]);
  m_pcColorTransform->process(m_ppcFrameRec[3], m_ppcFrameRec[2]);

  m_pcTransferFct->forward(m_ppcFrameOrg[4], m_ppcFrameOrg[3]);
  m_pcTransferFct->forward(m_ppcFrameRec[4], m_ppcFrameRec[3]);

  // Calculate the Metrics
  m_pcDistortionDeltaE->computeMetric(m_ppcFrameOrg[4], m_ppcFrameRec[4]);

  *deltaE = m_pcDistortionDeltaE->getDeltaE();
  *psnrL  = m_pcDistortionDeltaE->getPsnrL();

}

void EncGOP::copyBuftoFrame( Picture* pcPic )
{
  int cropOffsetLeft   = m_pcCfg->getCropOffsetLeft();
  int cropOffsetTop    = m_pcCfg->getCropOffsetTop();
  int cropOffsetRight  = m_pcCfg->getCropOffsetRight();
  int cropOffsetBottom = m_pcCfg->getCropOffsetBottom();

  int height = pcPic->getOrigBuf(COMPONENT_Y).height - cropOffsetLeft + cropOffsetRight;
  int width = pcPic->getOrigBuf(COMPONENT_Y).width - cropOffsetTop + cropOffsetBottom;

  ChromaFormat chFmt =  pcPic->chromaFormat;

  Pel* pOrg = pcPic->getOrigBuf(COMPONENT_Y).buf;
  Pel* pRec = pcPic->getRecoBuf(COMPONENT_Y).buf;

  uint16_t* yOrg = m_ppcFrameOrg[0]->m_ui16Comp[hdrtoolslib::Y_COMP];
  uint16_t* yRec = m_ppcFrameRec[0]->m_ui16Comp[hdrtoolslib::Y_COMP];
  uint16_t* uOrg = m_ppcFrameOrg[0]->m_ui16Comp[hdrtoolslib::Cb_COMP];
  uint16_t* uRec = m_ppcFrameRec[0]->m_ui16Comp[hdrtoolslib::Cb_COMP];
  uint16_t* vOrg = m_ppcFrameOrg[0]->m_ui16Comp[hdrtoolslib::Cr_COMP];
  uint16_t* vRec = m_ppcFrameRec[0]->m_ui16Comp[hdrtoolslib::Cr_COMP];

  if(chFmt == CHROMA_444){
    yOrg = m_ppcFrameOrg[1]->m_ui16Comp[hdrtoolslib::Y_COMP];
    yRec = m_ppcFrameRec[1]->m_ui16Comp[hdrtoolslib::Y_COMP];
    uOrg = m_ppcFrameOrg[1]->m_ui16Comp[hdrtoolslib::Cb_COMP];
    uRec = m_ppcFrameRec[1]->m_ui16Comp[hdrtoolslib::Cb_COMP];
    vOrg = m_ppcFrameOrg[1]->m_ui16Comp[hdrtoolslib::Cr_COMP];
    vRec = m_ppcFrameRec[1]->m_ui16Comp[hdrtoolslib::Cr_COMP];
  }

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      yOrg[i*width + j] = static_cast<uint16_t>(pOrg[(i + cropOffsetTop) * pcPic->getOrigBuf(COMPONENT_Y).stride + j + cropOffsetLeft]);
      yRec[i*width + j] = static_cast<uint16_t>(pRec[(i + cropOffsetTop) * pcPic->getRecoBuf(COMPONENT_Y).stride + j + cropOffsetLeft]);
    }
  }

  if (chFmt != CHROMA_444) {
    height >>= 1;
    width  >>= 1;
    cropOffsetLeft >>= 1;
    cropOffsetTop >>= 1;
  }

  pOrg = pcPic->getOrigBuf(COMPONENT_Cb).buf;
  pRec = pcPic->getRecoBuf(COMPONENT_Cb).buf;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      uOrg[i*width + j] = static_cast<uint16_t>(pOrg[(i + cropOffsetTop) * pcPic->getOrigBuf(COMPONENT_Cb).stride + j + cropOffsetLeft]);
      uRec[i*width + j] = static_cast<uint16_t>(pRec[(i + cropOffsetTop) * pcPic->getRecoBuf(COMPONENT_Cb).stride + j + cropOffsetLeft]);
    }
  }

  pOrg = pcPic->getOrigBuf(COMPONENT_Cr).buf;
  pRec = pcPic->getRecoBuf(COMPONENT_Cr).buf;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      vOrg[i*width + j] = static_cast<uint16_t>(pOrg[(i + cropOffsetTop) * pcPic->getOrigBuf(COMPONENT_Cr).stride + j + cropOffsetLeft]);
      vRec[i*width + j] = static_cast<uint16_t>(pRec[(i + cropOffsetTop) * pcPic->getRecoBuf(COMPONENT_Cr).stride + j + cropOffsetLeft]);
    }
  }
}
#endif

void EncGOP::xCalculateInterlacedAddPSNR( Picture* pcPicOrgFirstField, Picture* pcPicOrgSecondField,
                                          PelUnitBuf cPicRecFirstField, PelUnitBuf cPicRecSecondField,
                                          const InputColourSpaceConversion conversion, const bool printFrameMSE, 
                                          const bool printMSSSIM, double* PSNR_Y, bool isEncodeLtRef)
{
  const SPS &sps = *pcPicOrgFirstField->cs->sps;
  const ChromaFormat format = sps.getChromaFormatIdc();
  double  dPSNR[MAX_NUM_COMPONENT];
  Picture    *apcPicOrgFields[2] = {pcPicOrgFirstField, pcPicOrgSecondField};
  PelUnitBuf acPicRecFields[2]   = {cPicRecFirstField, cPicRecSecondField};
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
  for(int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
  }

  PelStorage cscd[2 /* first/second field */];
  if (conversion!=IPCOLOURSPACE_UNCHANGED)
  {
    for(uint32_t fieldNum=0; fieldNum<2; fieldNum++)
    {
      PelUnitBuf& reconField= (acPicRecFields[fieldNum]);
      cscd[fieldNum].create( reconField.chromaFormat, Area( Position(), reconField.Y()) );
      VideoIOYuv::ColourSpaceConvert(reconField, cscd[fieldNum], conversion, false);
      acPicRecFields[fieldNum]=cscd[fieldNum];
    }
  }

  //===== calculate PSNR =====
  double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};
  double msssim[MAX_NUM_COMPONENT] = {0.0};

  CHECK_(!(acPicRecFields[0].chromaFormat==acPicRecFields[1].chromaFormat), "Unspecified error");
  const uint32_t numValidComponents = ::getNumberValidComponents( acPicRecFields[0].chromaFormat );

  for (int chan = 0; chan < numValidComponents; chan++)
  {
    const ComponentID ch=ComponentID(chan);
    CHECK_(!(acPicRecFields[0].get(ch).width==acPicRecFields[1].get(ch).width), "Unspecified error");
    CHECK_(!(acPicRecFields[0].get(ch).height==acPicRecFields[0].get(ch).height), "Unspecified error");

    uint64_t uiSSDtemp=0;
    const uint32_t width    = acPicRecFields[0].get(ch).width - (m_pcEncLib->getPad(0) >> ::getComponentScaleX(ch, format));
    const uint32_t height   = acPicRecFields[0].get(ch).height - ((m_pcEncLib->getPad(1) >> 1) >> ::getComponentScaleY(ch, format));
    const uint32_t bitDepth = sps.getBitDepth(toChannelType(ch));

    double sumOverFieldsMSSSIM = 0;
    for(uint32_t fieldNum=0; fieldNum<2; fieldNum++)
    {
      CHECK_(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
#if ENABLE_QPA
      uiSSDtemp += xFindDistortionPlane( acPicRecFields[fieldNum].get(ch), apcPicOrgFields[fieldNum]->getOrigBuf().get(ch), useWPSNR ? bitDepth : 0, ::getComponentScaleX(ch, format), ::getComponentScaleY(ch, format) );
#else
      uiSSDtemp += xFindDistortionPlane( acPicRecFields[fieldNum].get(ch), apcPicOrgFields[fieldNum]->getOrigBuf().get(ch), 0 );
#endif
      if(printMSSSIM)
      {
        CPelBuf o = apcPicOrgFields[fieldNum]->getOrigBuf().get(ch);
        CPelBuf p = acPicRecFields[fieldNum].get(ch);
        sumOverFieldsMSSSIM += xCalculateMSSSIM(o.bufAt(0, 0), o.stride, p.bufAt(0, 0), p.stride, width, height, bitDepth);
      }
    }
    if (printMSSSIM)
    {
      msssim[ch] = sumOverFieldsMSSSIM / 2;
    }
    const uint32_t maxval = 255 << (bitDepth - 8);
    const uint32_t size   = width * height * 2;
    const double fRefValue = (double)maxval * maxval * size;
    dPSNR[ch]         = uiSSDtemp ? 10.0 * log10(fRefValue / (double)uiSSDtemp) : 999.99;
    MSEyuvframe[ch]   = (double)uiSSDtemp / size;
  }

  uint32_t uibits = 0; // the number of bits for the pair is not calculated here - instead the overall total is used elsewhere.

  //===== add PSNR =====
  m_gcAnalyzeAll_in.addResult (dPSNR, (double)uibits, MSEyuvframe, MSEyuvframe, msssim, isEncodeLtRef);

  *PSNR_Y = dPSNR[COMPONENT_Y];

  msg(INFO_, "\n                                      Interlaced frame %d: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", pcPicOrgSecondField->getPOC()/2, dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );
  if (printMSSSIM)
  {
    printf(" [MS-SSIM Y %1.6lf    U %1.6lf    V %1.6lf]", msssim[COMPONENT_Y], msssim[COMPONENT_Cb], msssim[COMPONENT_Cr] );
  }
  if (printFrameMSE)
  {
    msg(DETAILS_, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
  }

  for(uint32_t fieldNum=0; fieldNum<2; fieldNum++)
  {
    cscd[fieldNum].destroy();
  }
}

/** Function for deciding the nal_unit_type.
 * \param pocCurr POC of the current picture
 * \param lastIDR  POC of the last IDR picture
 * \param isField  true to indicate field coding
 * \returns the NAL unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
NalUnitType EncGOP::getNalUnitType(int pocCurr, int lastIDR, bool isField)
{
  if (pocCurr == 0)
  {
    return NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }

  if (m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pocCurr == (m_pcCfg->getUseCompositeRef() ? 2: 1))
  {
    // to avoid the picture becoming an IRAP
    return NAL_UNIT_CODED_SLICE_TRAIL;
  }

  if (m_pcCfg->getDecodingRefreshType() != 3 && (pocCurr - isField) % (m_pcCfg->getIntraPeriod() * (m_pcCfg->getUseCompositeRef() ? 2 : 1)) == 0)
  {
    if (m_pcCfg->getDecodingRefreshType() == 1)
    {
      return NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_pcCfg->getDecodingRefreshType() == 2)
    {
      SPS *sps = m_pcEncLib->getSPS(m_pcEncLib->getLayerId());
      if (sps != nullptr)
      {
        int maxTLayer = sps->getMaxTLayers() - 1;
        return sps->getMaxNumReorderPics(maxTLayer) > 0 ? NAL_UNIT_CODED_SLICE_IDR_W_RADL : NAL_UNIT_CODED_SLICE_IDR_N_LP;
      }
      else
      {
        return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
      }
    }
  }
  if(m_pocCRA>0)
  {
    if(pocCurr<m_pocCRA)
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
      // controlling the reference pictures used for encoding that leading picture. Such a leading
      // picture need not be marked as a TFD picture.
      return NAL_UNIT_CODED_SLICE_RASL;
    }
  }
  if (lastIDR>0)
  {
    if (pocCurr < lastIDR)
    {
      return NAL_UNIT_CODED_SLICE_RADL;
    }
  }
  return NAL_UNIT_CODED_SLICE_TRAIL;
}

void EncGOP::xUpdateRasInit(Slice* slice)
{
  slice->setPendingRasInit( false );
  if ( slice->getPOC() > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->setPendingRasInit( true );
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->getPOC();
  }
}

double EncGOP::xCalculateRVM()
{
  double dRVM = 0;

  if( m_pcCfg->getGOPSize() == 1 && m_pcCfg->getIntraPeriod() != 1 && m_pcCfg->getFramesToBeEncoded() > RVM_VCEGAM10_M * 2 )
  {
    // calculate RVM only for lowdelay configurations
    std::vector<double> vRL , vB;
    size_t N = m_vRVM_RP.size();
    vRL.resize( N );
    vB.resize( N );

    int i;
    double dRavg = 0 , dBavg = 0;
    vB[RVM_VCEGAM10_M] = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      vRL[i] = 0;
      for( int j = i - RVM_VCEGAM10_M ; j <= i + RVM_VCEGAM10_M - 1 ; j++ )
      {
        vRL[i] += m_vRVM_RP[j];
      }
      vRL[i] /= ( 2 * RVM_VCEGAM10_M );
      vB[i] = vB[i-1] + m_vRVM_RP[i] - vRL[i];
      dRavg += m_vRVM_RP[i];
      dBavg += vB[i];
    }

    dRavg /= ( N - 2 * RVM_VCEGAM10_M );
    dBavg /= ( N - 2 * RVM_VCEGAM10_M );

    double dSigamB = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      double tmp = vB[i] - dBavg;
      dSigamB += tmp * tmp;
    }
    dSigamB = sqrt( dSigamB / ( N - 2 * RVM_VCEGAM10_M ) );

    double f = sqrt( 12.0 * ( RVM_VCEGAM10_M - 1 ) / ( RVM_VCEGAM10_M + 1 ) );

    dRVM = dSigamB / dRavg * f;
  }

  return( dRVM );
}

/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
void EncGOP::xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, OutputBitstream* codedSliceData)
{
  // Byte-align
  rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)
  {
    rNalu.m_Bitstream.addSubstream(codedSliceData);
  }
  codedSliceData->clear();
}


void EncGOP::arrangeCompositeReference(Slice* pcSlice, PicList& rcListPic, int pocCurr)
{
  Picture* curPic = NULL;
  PicList::iterator  iterPic = rcListPic.begin();
  const PreCalcValues *pcv = pcSlice->getPPS()->pcv;
  m_bgPOC = pocCurr + 1;
  if (m_picBg->getSpliceFull())
  {
    return;
  }
  while (iterPic != rcListPic.end())
  {
    curPic = *(iterPic++);
    if (curPic->getPOC() == pocCurr)
    {
      break;
    }
  }
  if (pcSlice->isIRAP())
  {
    return;
  }

  int width = pcv->lumaWidth;
  int height = pcv->lumaHeight;
  int stride = curPic->getOrigBuf().get(COMPONENT_Y).stride;
  int cStride = curPic->getOrigBuf().get(COMPONENT_Cb).stride;
  Pel* curLumaAddr = curPic->getOrigBuf().get(COMPONENT_Y).buf;
  Pel* curCbAddr = curPic->getOrigBuf().get(COMPONENT_Cb).buf;
  Pel* curCrAddr = curPic->getOrigBuf().get(COMPONENT_Cr).buf;
  Pel* bgOrgLumaAddr = m_picOrig->getOrigBuf().get(COMPONENT_Y).buf;
  Pel* bgOrgCbAddr = m_picOrig->getOrigBuf().get(COMPONENT_Cb).buf;
  Pel* bgOrgCrAddr = m_picOrig->getOrigBuf().get(COMPONENT_Cr).buf;
  int cuMaxWidth = pcv->maxCUWidth;
  int cuMaxHeight = pcv->maxCUHeight;
  int maxReplace = (pcv->sizeInCtus) / 2;
  maxReplace = maxReplace < 1 ? 1 : maxReplace;
  typedef struct tagCostStr
  {
    double cost;
    int ctuIdx;
  }CostStr;
  CostStr* minCtuCost = new CostStr[maxReplace];
  for (int i = 0; i < maxReplace; i++)
  {
    minCtuCost[i].cost = 1e10;
    minCtuCost[i].ctuIdx = -1;
  }
  int bitIncrementY = pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8;
  int bitIncrementUV = pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 8;
  for (int y = 0; y < height; y += cuMaxHeight)
  {
    for (int x = 0; x < width; x += cuMaxWidth)
    {
      double lcuDist = 0.0;
      double lcuDistCb = 0.0;
      double lcuDistCr = 0.0;
      int    realPixelCnt = 0;
      double lcuCost = 1e10;
      int largeDist = 0;

      for (int tmpy = 0; tmpy < cuMaxHeight; tmpy++)
      {
        if (y + tmpy >= height)
        {
          break;
        }
        for (int tmpx = 0; tmpx < cuMaxWidth; tmpx++)
        {
          if (x + tmpx >= width)
          {
            break;
          }

          realPixelCnt++;
          lcuDist += abs(curLumaAddr[(y + tmpy)*stride + x + tmpx] - bgOrgLumaAddr[(y + tmpy)*stride + x + tmpx]);
          if (abs(curLumaAddr[(y + tmpy)*stride + x + tmpx] - bgOrgLumaAddr[(y + tmpy)*stride + x + tmpx]) >(20 << bitIncrementY))
          {
            largeDist++;
          }

          if (tmpy % 2 == 0 && tmpx % 2 == 0)
          {
            lcuDistCb += abs(curCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] - bgOrgCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2]);
            lcuDistCr += abs(curCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] - bgOrgCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2]);
          }
        }
      }

      //Test the vertical or horizontal edge for background patches candidates
      int yInLCU = y / cuMaxHeight;
      int xInLCU = x / cuMaxWidth;
      int iLCUIdx = yInLCU * pcv->widthInCtus + xInLCU;
      if ((largeDist / (double)realPixelCnt < 0.01 &&lcuDist / realPixelCnt < (3.5 * (1 << bitIncrementY)) && lcuDistCb / realPixelCnt < (0.5 * (1 << bitIncrementUV)) && lcuDistCr / realPixelCnt < (0.5 * (1 << bitIncrementUV)) && m_picBg->getSpliceIdx(iLCUIdx) == 0))
      {
        lcuCost = lcuDist / realPixelCnt + lcuDistCb / realPixelCnt + lcuDistCr / realPixelCnt;
        //obtain the maxReplace smallest cost
        //1) find the largest cost in the maxReplace candidates
        for (int i = 0; i < maxReplace - 1; i++)
        {
          if (minCtuCost[i].cost > minCtuCost[i + 1].cost)
          {
            swap(minCtuCost[i].cost, minCtuCost[i + 1].cost);
            swap(minCtuCost[i].ctuIdx, minCtuCost[i + 1].ctuIdx);
          }
        }
        // 2) compare the current cost with the largest cost
        if (lcuCost < minCtuCost[maxReplace - 1].cost)
        {
          minCtuCost[maxReplace - 1].cost = lcuCost;
          minCtuCost[maxReplace - 1].ctuIdx = iLCUIdx;
        }
      }
    }
  }

  // modify QP for background CTU
  {
    for (int i = 0; i < maxReplace; i++)
    {
      if (minCtuCost[i].ctuIdx != -1)
      {
        m_picBg->setSpliceIdx(minCtuCost[i].ctuIdx, pocCurr);
      }
    }
  }
  delete[]minCtuCost;
}

void EncGOP::updateCompositeReference(Slice* pcSlice, PicList& rcListPic, int pocCurr)
{
  Picture* curPic = NULL;
  const PreCalcValues *pcv = pcSlice->getPPS()->pcv;
  PicList::iterator  iterPic = rcListPic.begin();
  iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    curPic = *(iterPic++);
    if (curPic->getPOC() == pocCurr)
    {
      break;
    }
  }
  assert(curPic->getPOC() == pocCurr);

  int width = pcv->lumaWidth;
  int height = pcv->lumaHeight;
  int stride = curPic->getRecoBuf().get(COMPONENT_Y).stride;
  int cStride = curPic->getRecoBuf().get(COMPONENT_Cb).stride;

  Pel* bgLumaAddr = m_picBg->getRecoBuf().get(COMPONENT_Y).buf;
  Pel* bgCbAddr = m_picBg->getRecoBuf().get(COMPONENT_Cb).buf;
  Pel* bgCrAddr = m_picBg->getRecoBuf().get(COMPONENT_Cr).buf;
  Pel* curLumaAddr = curPic->getRecoBuf().get(COMPONENT_Y).buf;
  Pel* curCbAddr = curPic->getRecoBuf().get(COMPONENT_Cb).buf;
  Pel* curCrAddr = curPic->getRecoBuf().get(COMPONENT_Cr).buf;

  int maxCuWidth = pcv->maxCUWidth;
  int maxCuHeight = pcv->maxCUHeight;

  // Update background reference
  if (pcSlice->isIRAP())//(pocCurr == 0)
  {
    curPic->extendPicBorder( pcSlice->getPPS() );
    curPic->setBorderExtension(true);

    m_picBg->getRecoBuf().copyFrom(curPic->getRecoBuf());
    m_picOrig->getOrigBuf().copyFrom(curPic->getOrigBuf());
  }
  else
  {
    //cout << "update B" << pocCurr << endl;
    for (int y = 0; y < height; y += maxCuHeight)
    {
      for (int x = 0; x < width; x += maxCuWidth)
      {
        if (m_picBg->getSpliceIdx((y / maxCuHeight)*pcv->widthInCtus + x / maxCuWidth) == pocCurr)
        {
          for (int tmpy = 0; tmpy < maxCuHeight; tmpy++)
          {
            if (y + tmpy >= height)
            {
              break;
            }
            for (int tmpx = 0; tmpx < maxCuWidth; tmpx++)
            {
              if (x + tmpx >= width)
              {
                break;
              }
              bgLumaAddr[(y + tmpy)*stride + x + tmpx] = curLumaAddr[(y + tmpy)*stride + x + tmpx];
              if (tmpy % 2 == 0 && tmpx % 2 == 0)
              {
                bgCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] = curCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2];
                bgCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] = curCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2];
              }
            }
          }
        }
      }
    }
    m_picBg->setBorderExtension(false);
    m_picBg->extendPicBorder( pcSlice->getPPS() );
    m_picBg->setBorderExtension(true);

    curPic->extendPicBorder( pcSlice->getPPS() );
    curPic->setBorderExtension(true);
    m_picOrig->getOrigBuf().copyFrom(curPic->getOrigBuf());

    m_picBg->setBorderExtension(false);
    m_picBg->extendPicBorder( pcSlice->getPPS() );
    m_picBg->setBorderExtension(true);
  }
}

void EncGOP::applyDeblockingFilterMetric( Picture* pcPic, uint32_t uiNumSlices )
{
  PelBuf cPelBuf = pcPic->getRecoBuf().get( COMPONENT_Y );
  Pel* Rec    = cPelBuf.buf;
  const int  stride = cPelBuf.stride;
  const uint32_t picWidth = cPelBuf.width;
  const uint32_t picHeight = cPelBuf.height;

  Pel* tempRec = Rec;
  const Slice* pcSlice = pcPic->slices[0];
  const uint32_t log2maxTB = pcSlice->getSPS()->getLog2MaxTbSize();
  const uint32_t maxTBsize = (1<<log2maxTB);
  const uint32_t minBlockArtSize = 8;
  const uint32_t noCol = (picWidth>>log2maxTB);
  const uint32_t noRows = (picHeight>>log2maxTB);
  CHECK_(!(noCol > 1), "Unspecified error");
  CHECK_(!(noRows > 1), "Unspecified error");
  std::vector<uint64_t> colSAD(noCol,  uint64_t(0));
  std::vector<uint64_t> rowSAD(noRows, uint64_t(0));
  uint32_t colIdx = 0;
  uint32_t rowIdx = 0;
  Pel p0, p1, p2, q0, q1, q2;

  int qp = pcSlice->getSliceQp();
  const int bitDepthLuma=pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  int bitdepthScale = 1 << (bitDepthLuma-8);
  int beta = LoopFilter::getBeta( qp ) * bitdepthScale;
  const int thr2 = (beta>>2);
  const int thr1 = 2*bitdepthScale;
  uint32_t a = 0;

  if (maxTBsize > minBlockArtSize)
  {
    // Analyze vertical artifact edges
    for(int c = maxTBsize; c < picWidth; c += maxTBsize)
    {
      for(int r = 0; r < picHeight; r++)
      {
        p2 = Rec[c-3];
        p1 = Rec[c-2];
        p0 = Rec[c-1];
        q0 = Rec[c];
        q1 = Rec[c+1];
        q2 = Rec[c+2];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if ( thr1 < a && a < thr2)
        {
          colSAD[colIdx] += abs(p0 - q0);
        }
        Rec += stride;
      }
      colIdx++;
      Rec = tempRec;
    }

    // Analyze horizontal artifact edges
    for(int r = maxTBsize; r < picHeight; r += maxTBsize)
    {
      for(int c = 0; c < picWidth; c++)
      {
        p2 = Rec[c + (r-3)*stride];
        p1 = Rec[c + (r-2)*stride];
        p0 = Rec[c + (r-1)*stride];
        q0 = Rec[c + r*stride];
        q1 = Rec[c + (r+1)*stride];
        q2 = Rec[c + (r+2)*stride];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if (thr1 < a && a < thr2)
        {
          rowSAD[rowIdx] += abs(p0 - q0);
        }
      }
      rowIdx++;
    }
  }

  uint64_t colSADsum = 0;
  uint64_t rowSADsum = 0;
  for(int c = 0; c < noCol-1; c++)
  {
    colSADsum += colSAD[c];
  }
  for(int r = 0; r < noRows-1; r++)
  {
    rowSADsum += rowSAD[r];
  }

  colSADsum <<= 10;
  rowSADsum <<= 10;
  colSADsum /= (noCol-1);
  colSADsum /= picHeight;
  rowSADsum /= (noRows-1);
  rowSADsum /= picWidth;

  uint64_t avgSAD = ((colSADsum + rowSADsum)>>1);
  avgSAD >>= (bitDepthLuma-8);

  if ( avgSAD > 2048 )
  {
    avgSAD >>= 9;
    int offset = Clip3(2,6,(int)avgSAD);
    for (int i=0; i<uiNumSlices; i++)
    {
      Slice* pcLocalSlice = pcPic->slices[i];
      pcLocalSlice->setDeblockingFilterOverrideFlag   ( true);
      pcLocalSlice->setDeblockingFilterDisable        ( false);
      pcLocalSlice->setDeblockingFilterBetaOffsetDiv2 ( offset );
      pcLocalSlice->setDeblockingFilterTcOffsetDiv2   ( offset );
      pcLocalSlice->setDeblockingFilterCbBetaOffsetDiv2 ( offset );
      pcLocalSlice->setDeblockingFilterCbTcOffsetDiv2   ( offset );
      pcLocalSlice->setDeblockingFilterCrBetaOffsetDiv2 ( offset );
      pcLocalSlice->setDeblockingFilterCrTcOffsetDiv2   ( offset );
    }
  }
  else
  {
    for (int i=0; i<uiNumSlices; i++)
    {
      Slice* pcLocalSlice = pcPic->slices[i];
      const PPS* pcPPS = pcSlice->getPPS();
      pcLocalSlice->setDeblockingFilterOverrideFlag  ( false);
      pcLocalSlice->setDeblockingFilterDisable       ( pcPPS->getPPSDeblockingFilterDisabledFlag() );
      pcLocalSlice->setDeblockingFilterBetaOffsetDiv2( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcLocalSlice->setDeblockingFilterTcOffsetDiv2  ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
      pcLocalSlice->setDeblockingFilterCbBetaOffsetDiv2 ( pcPPS->getDeblockingFilterCbBetaOffsetDiv2() );
      pcLocalSlice->setDeblockingFilterCbTcOffsetDiv2   ( pcPPS->getDeblockingFilterCbTcOffsetDiv2() );
      pcLocalSlice->setDeblockingFilterCrBetaOffsetDiv2 ( pcPPS->getDeblockingFilterCrBetaOffsetDiv2() );
      pcLocalSlice->setDeblockingFilterCrTcOffsetDiv2   ( pcPPS->getDeblockingFilterCrTcOffsetDiv2() );
    }
  }
}

#if W0038_DB_OPT
void EncGOP::applyDeblockingFilterParameterSelection( Picture* pcPic, const uint32_t numSlices, const int gopID )
{
  enum DBFltParam
  {
    DBFLT_PARAM_AVAILABLE = 0,
    DBFLT_DISABLE_FLAG,
    DBFLT_BETA_OFFSETD2,
    DBFLT_TC_OFFSETD2,
    //NUM_DBFLT_PARAMS
  };
  const int MAX_BETA_OFFSET = 3;
  const int MIN_BETA_OFFSET = -3;
  const int MAX_TC_OFFSET = 3;
  const int MIN_TC_OFFSET = -3;

  PelUnitBuf reco = pcPic->getRecoBuf();

  const int currQualityLayer = (!pcPic->slices[0]->isIRAP()) ? m_pcCfg->getGOPEntry(gopID).m_temporalId+1 : 0;
  CHECK_(!(currQualityLayer <MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS), "Unspecified error");

  CodingStructure& cs = *pcPic->cs;

  if(!m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv = new PelStorage;
    m_pcDeblockingTempPicYuv->create( cs.area );
    memset(m_DBParam, 0, sizeof(m_DBParam));
  }

  //preserve current reconstruction
  m_pcDeblockingTempPicYuv->copyFrom ( reco );

  const bool bNoFiltering      = m_DBParam[currQualityLayer][DBFLT_PARAM_AVAILABLE] && m_DBParam[currQualityLayer][DBFLT_DISABLE_FLAG]==false /*&& pcPic->getTLayer()==0*/;
  const int  maxBetaOffsetDiv2 = bNoFiltering? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]+1) : MAX_BETA_OFFSET;
  const int  minBetaOffsetDiv2 = bNoFiltering? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]-1) : MIN_BETA_OFFSET;
  const int  maxTcOffsetDiv2   = bNoFiltering? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]+2)       : MAX_TC_OFFSET;
  const int  minTcOffsetDiv2   = bNoFiltering? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]-2)       : MIN_TC_OFFSET;

  uint64_t distBetaPrevious      = std::numeric_limits<uint64_t>::max();
  uint64_t distMin               = std::numeric_limits<uint64_t>::max();
  bool   bDBFilterDisabledBest = true;
  int    betaOffsetDiv2Best    = 0;
  int    tcOffsetDiv2Best      = 0;

  for(int betaOffsetDiv2=maxBetaOffsetDiv2; betaOffsetDiv2>=minBetaOffsetDiv2; betaOffsetDiv2--)
  {
    uint64_t distTcMin = std::numeric_limits<uint64_t>::max();
    for(int tcOffsetDiv2=maxTcOffsetDiv2; tcOffsetDiv2 >= minTcOffsetDiv2; tcOffsetDiv2--)
    {
      for (int i=0; i<numSlices; i++)
      {
        Slice* pcSlice = pcPic->slices[i];
        pcSlice->setDeblockingFilterOverrideFlag  ( true);
        pcSlice->setDeblockingFilterDisable       ( false);
        pcSlice->setDeblockingFilterBetaOffsetDiv2( betaOffsetDiv2 );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( tcOffsetDiv2 );
        pcSlice->setDeblockingFilterCbBetaOffsetDiv2( betaOffsetDiv2 );
        pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( tcOffsetDiv2 );
        pcSlice->setDeblockingFilterCrBetaOffsetDiv2( betaOffsetDiv2 );
        pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( tcOffsetDiv2 );
      }

      // restore reconstruction
      reco.copyFrom( *m_pcDeblockingTempPicYuv );

      const uint64_t dist = preLoopFilterPicAndCalcDist( pcPic );

      if(dist < distMin)
      {
        distMin = dist;
        bDBFilterDisabledBest = false;
        betaOffsetDiv2Best  = betaOffsetDiv2;
        tcOffsetDiv2Best = tcOffsetDiv2;
      }
      if(dist < distTcMin)
      {
        distTcMin = dist;
      }
      else if(tcOffsetDiv2 <-2)
      {
        break;
      }
    }
    if(betaOffsetDiv2<-1 && distTcMin >= distBetaPrevious)
    {
      break;
    }
    distBetaPrevious = distTcMin;
  }

  //update:
  m_DBParam[currQualityLayer][DBFLT_PARAM_AVAILABLE] = 1;
  m_DBParam[currQualityLayer][DBFLT_DISABLE_FLAG]    = bDBFilterDisabledBest;
  m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]   = betaOffsetDiv2Best;
  m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]     = tcOffsetDiv2Best;

  // restore reconstruction
  reco.copyFrom( *m_pcDeblockingTempPicYuv );

  const PPS* pcPPS = pcPic->slices[0]->getPPS();
  if(bDBFilterDisabledBest)
  {
    for (int i=0; i<numSlices; i++)
    {
      Slice* pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag( true);
      pcSlice->setDeblockingFilterDisable     ( true);
    }
  }
  else if(betaOffsetDiv2Best == pcPPS->getDeblockingFilterBetaOffsetDiv2() &&  tcOffsetDiv2Best == pcPPS->getDeblockingFilterTcOffsetDiv2())
  {
    for (int i=0; i<numSlices; i++)
    {
      Slice*      pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag   ( false);
      pcSlice->setDeblockingFilterDisable        ( pcPPS->getPPSDeblockingFilterDisabledFlag() );
      pcSlice->setDeblockingFilterBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
      pcSlice->setDeblockingFilterCbBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCbTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
      pcSlice->setDeblockingFilterCrBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCrTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
    }
  }
  else
  {
    for (int i=0; i<numSlices; i++)
    {
      Slice* pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag   ( true);
      pcSlice->setDeblockingFilterDisable        ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2 ( betaOffsetDiv2Best);
      pcSlice->setDeblockingFilterTcOffsetDiv2   ( tcOffsetDiv2Best);
      pcSlice->setDeblockingFilterCbBetaOffsetDiv2 ( betaOffsetDiv2Best);
      pcSlice->setDeblockingFilterCbTcOffsetDiv2   ( tcOffsetDiv2Best);
      pcSlice->setDeblockingFilterCrBetaOffsetDiv2 ( betaOffsetDiv2Best);
      pcSlice->setDeblockingFilterCrTcOffsetDiv2   ( tcOffsetDiv2Best);
    }
  }
}
#endif

#if JVET_R0193
bool EncGOP::xCheckMaxTidILRefPics(int layerIdx, Picture* refPic, bool currentPicIsIRAP)
#else
bool EncGOP::xCheckMaxTidILRefPics(Picture* refPic, bool currentPicIsIRAP)
#endif
{
#if JVET_R0193
  const VPS* vps = refPic->cs->vps;
  int refLayerIdx = vps == nullptr ? 0 : vps->getGeneralLayerIdx(refPic->layerId);
  const int maxTidILRefPicsPlus1 = vps->getMaxTidIlRefPicsPlus1(layerIdx, refLayerIdx);
#else
  const int maxTidILRefPicsPlus1 = m_pcCfg->getVPSParameters().m_maxTidILRefPicsPlus1;
#endif

  // -1 means not set
  if (maxTidILRefPicsPlus1 < 0)
  {
    return true;
  }

  // 0 allows only IRAP pictures to use inter-layer prediction
  if (maxTidILRefPicsPlus1 == 0)
  {
    return currentPicIsIRAP;
  }

  // all other cases filter by temporalID
  return ( refPic->temporalId < maxTidILRefPicsPlus1 );
}

void EncGOP::xCreateExplicitReferencePictureSetFromReference( Slice* slice, PicList& rcListPic, const ReferencePictureList *rpl0, const ReferencePictureList *rpl1 )
{
  Picture* rpcPic;
  int pocCycle = 0;

  Picture* pic = slice->getPic();
  const VPS* vps = slice->getPic()->cs->vps;
  int layerIdx = vps == nullptr ? 0 : vps->getGeneralLayerIdx( pic->layerId );
  bool isIntraLayerPredAllowed = (vps->getIndependentLayerFlag(layerIdx) || (vps->getPredDirection(slice->getTLayer()) != 1)) && !slice->isIRAP();
  bool isInterLayerPredAllowed = !vps->getIndependentLayerFlag(layerIdx) && (vps->getPredDirection(slice->getTLayer()) != 2);

  auto localRPL0 = ReferencePictureList( slice->getSPS()->getInterLayerPresentFlag() );
  auto const pLocalRPL0 = &localRPL0;

  uint32_t numOfSTRPL0 = 0;
  uint32_t numOfLTRPL0 = 0;
  uint32_t numOfILRPL0 = 0;
  uint32_t numOfRefPic = rpl0->getNumberOfShorttermPictures() + rpl0->getNumberOfLongtermPictures();
  uint32_t refPicIdxL0 = 0;

  static_vector<int, MAX_NUM_REF_PICS> higherTLayerRefs;

  higherTLayerRefs.resize(0);
  if (isIntraLayerPredAllowed)
  {
    for (int ii = 0; ii < numOfRefPic; ii++)
    {
      // loop through all pictures in the reference picture buffer
      PicList::iterator iterPic = rcListPic.begin();

      bool isAvailable  = false;
      bool hasHigherTId = false;

      pocCycle = 1 << (slice->getSPS()->getBitsForPOC());
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);

        if (rpcPic->layerId == pic->layerId)
        {
          hasHigherTId = rpcPic->temporalId > pic->temporalId;
          if (!rpl0->isRefPicLongterm(ii) && rpcPic->referenced
              && rpcPic->getPOC() == slice->getPOC() + rpl0->getRefPicIdentifier(ii)
              && !slice->isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
          {
            isAvailable = true;
            break;
          }
          else if (rpl0->isRefPicLongterm(ii) && rpcPic->referenced && (rpcPic->getPOC() & (pocCycle - 1)) == rpl0->getRefPicIdentifier(ii) && !slice->isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
          {
            isAvailable = true;
            break;
          }
        }
      }

      if (isAvailable)
      {
        if (hasHigherTId)
        {
          higherTLayerRefs.push_back(ii);
        }
        else
        {
          pLocalRPL0->setRefPicIdentifier(refPicIdxL0, rpl0->getRefPicIdentifier(ii), rpl0->isRefPicLongterm(ii), false,
                                          NOT_VALID);
          refPicIdxL0++;
          numOfSTRPL0 = numOfSTRPL0 + ((rpl0->isRefPicLongterm(ii)) ? 0 : 1);
          numOfLTRPL0 += (rpl0->isRefPicLongterm(ii) && !rpl0->isInterLayerRefPic(ii)) ? 1 : 0;
        }
      }
    }
  }

  // inter-layer reference pictures are added to the end of the reference picture list
  if (layerIdx && vps && !vps->getAllIndependentLayersFlag() && isInterLayerPredAllowed)
  {
    numOfRefPic = rpl0->getNumberOfInterLayerPictures() ? rpl0->getNumberOfInterLayerPictures() : m_pcEncLib->getNumRefLayers( layerIdx );

    for( int ii = 0; ii < numOfRefPic; ii++ )
    {
      // loop through all pictures in the reference picture buffer
      PicList::iterator iterPic = rcListPic.begin();

      while( iterPic != rcListPic.end() && ii < numOfRefPic )
      {
        rpcPic = *( iterPic++ );
        int refLayerIdx = vps->getGeneralLayerIdx( rpcPic->layerId );
        if (rpcPic->referenced && rpcPic->getPOC() == pic->getPOC() && vps->getDirectRefLayerFlag(layerIdx, refLayerIdx)
#if JVET_R0193
            && xCheckMaxTidILRefPics(layerIdx, rpcPic, slice->isIRAP()))
#else
            && xCheckMaxTidILRefPics(rpcPic, slice->isIRAP()) )
#endif
        {
          pLocalRPL0->setRefPicIdentifier( refPicIdxL0, 0, true, true, vps->getInterLayerRefIdc( layerIdx, refLayerIdx ) );
          refPicIdxL0++;
          numOfILRPL0++;
          ii++;
        }
      }
    }
  }

  if( slice->getEnableDRAPSEI() )
  {
    pLocalRPL0->setNumberOfShorttermPictures( numOfSTRPL0 );
    pLocalRPL0->setNumberOfLongtermPictures( numOfLTRPL0 );
    pLocalRPL0->setNumberOfInterLayerPictures( numOfILRPL0 );

    if( !slice->isIRAP() && !slice->isPOCInRefPicList( pLocalRPL0, slice->getAssociatedIRAPPOC() ) )
    {
      if( slice->getUseLTforDRAP() && !slice->isPOCInRefPicList( rpl1, slice->getAssociatedIRAPPOC() ) )
      {
        // Adding associated IRAP as longterm picture
        pLocalRPL0->setRefPicIdentifier( refPicIdxL0, slice->getAssociatedIRAPPOC(), true, false, 0 );
        refPicIdxL0++;
        numOfLTRPL0++;
      }
      else
      {
        // Adding associated IRAP as shortterm picture
        pLocalRPL0->setRefPicIdentifier(refPicIdxL0, slice->getAssociatedIRAPPOC() - slice->getPOC(), false, false, 0);
        refPicIdxL0++;
        numOfSTRPL0++;
      }
    }
  }

  // now add higher TId refs
  for (int i = 0; i < higherTLayerRefs.size(); i++)
  {
    const int ii = higherTLayerRefs[i];
    pLocalRPL0->setRefPicIdentifier(refPicIdxL0, rpl0->getRefPicIdentifier(ii), rpl0->isRefPicLongterm(ii), false,
                                    NOT_VALID);
    refPicIdxL0++;
    numOfSTRPL0 = numOfSTRPL0 + ((rpl0->isRefPicLongterm(ii)) ? 0 : 1);
    numOfLTRPL0 += (rpl0->isRefPicLongterm(ii) && !rpl0->isInterLayerRefPic(ii)) ? 1 : 0;
  }

  auto localRPL1 = ReferencePictureList( slice->getSPS()->getInterLayerPresentFlag() );
  auto const pLocalRPL1 = &localRPL1;

  uint32_t numOfSTRPL1 = 0;
  uint32_t numOfLTRPL1 = 0;
  uint32_t numOfILRPL1 = 0;
  numOfRefPic = rpl1->getNumberOfShorttermPictures() + rpl1->getNumberOfLongtermPictures();
  uint32_t refPicIdxL1 = 0;

  higherTLayerRefs.resize(0);
  if (isIntraLayerPredAllowed)
  {
    for (int ii = 0; ii < numOfRefPic; ii++)
    {
      // loop through all pictures in the reference picture buffer
      PicList::iterator iterPic = rcListPic.begin();

      bool isAvailable  = false;
      bool hasHigherTId = false;
      pocCycle = 1 << (slice->getSPS()->getBitsForPOC());
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);
        if (rpcPic->layerId == pic->layerId)
        {
          hasHigherTId = rpcPic->temporalId > pic->temporalId;
          if (!rpl1->isRefPicLongterm(ii) && rpcPic->referenced
              && rpcPic->getPOC() == slice->getPOC() + rpl1->getRefPicIdentifier(ii)
              && !slice->isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
          {
            isAvailable = true;
            break;
          }
          else if (rpl1->isRefPicLongterm(ii) && rpcPic->referenced && (rpcPic->getPOC() & (pocCycle - 1)) == rpl1->getRefPicIdentifier(ii) && !slice->isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
          {
            isAvailable = true;
            break;
          }
        }
      }

      if (isAvailable)
      {
        if (hasHigherTId)
        {
          higherTLayerRefs.push_back(ii);
        }
        else
        {
          pLocalRPL1->setRefPicIdentifier(refPicIdxL1, rpl1->getRefPicIdentifier(ii), rpl1->isRefPicLongterm(ii), false,
                                          NOT_VALID);
          refPicIdxL1++;
          numOfSTRPL1 = numOfSTRPL1 + ((rpl1->isRefPicLongterm(ii)) ? 0 : 1);
          numOfLTRPL1 += (rpl1->isRefPicLongterm(ii) && !rpl1->isInterLayerRefPic(ii)) ? 1 : 0;
        }
      }
    }
  }


  // inter-layer reference pictures are added to the end of the reference picture list
  if (layerIdx && vps && !vps->getAllIndependentLayersFlag() && isInterLayerPredAllowed)
  {
    numOfRefPic = rpl1->getNumberOfInterLayerPictures() ? rpl1->getNumberOfInterLayerPictures() : m_pcEncLib->getNumRefLayers( layerIdx );

    for( int ii = 0; ii < numOfRefPic; ii++ )
    {
      // loop through all pictures in the reference picture buffer
      PicList::iterator iterPic = rcListPic.begin();

      while( iterPic != rcListPic.end() && ii < numOfRefPic )
      {
        rpcPic = *( iterPic++ );
        int refLayerIdx = vps->getGeneralLayerIdx( rpcPic->layerId );
        if (rpcPic->referenced && rpcPic->getPOC() == pic->getPOC() && vps->getDirectRefLayerFlag(layerIdx, refLayerIdx)
#if JVET_R0193
            && xCheckMaxTidILRefPics( layerIdx, rpcPic, slice->isIRAP()))
#else
            && xCheckMaxTidILRefPics( rpcPic, slice->isIRAP() ) )
#endif
        {
          pLocalRPL1->setRefPicIdentifier( refPicIdxL1, 0, true, true, vps->getInterLayerRefIdc( layerIdx, refLayerIdx ) );
          refPicIdxL1++;
          numOfILRPL1++;
          ii++;
        }
      }
    }
  }

  // now add higher TId refs
  for (int i = 0; i < higherTLayerRefs.size(); i++)
  {
    const int ii = higherTLayerRefs[i];
    pLocalRPL1->setRefPicIdentifier(refPicIdxL1, rpl1->getRefPicIdentifier(ii), rpl1->isRefPicLongterm(ii), false,
                                    NOT_VALID);
    refPicIdxL1++;
    numOfSTRPL1 = numOfSTRPL1 + ((rpl1->isRefPicLongterm(ii)) ? 0 : 1);
    numOfLTRPL1 += (rpl1->isRefPicLongterm(ii) && !rpl1->isInterLayerRefPic(ii)) ? 1 : 0;
  }

  //Copy from L1 if we have less than active ref pic
  int numOfNeedToFill = rpl0->getNumberOfActivePictures() - (numOfLTRPL0 + numOfSTRPL0);
  bool isDisallowMixedRefPic = ( slice->getSPS()->getAllActiveRplEntriesHasSameSignFlag() ) ? true : false;
  int originalL0StrpNum = numOfSTRPL0;
  int originalL0LtrpNum = numOfLTRPL0;
  int originalL0IlrpNum = numOfILRPL0;

  for( int ii = 0; numOfNeedToFill > 0 && ii < ( numOfLTRPL1 + numOfSTRPL1 + numOfILRPL1 ); ii++ )
  {
    if( ii <= ( numOfLTRPL1 + numOfSTRPL1 + numOfILRPL1 - 1 ) )
    {
      //Make sure this copy is not already in L0
      bool canIncludeThis = true;
      for( int jj = 0; jj < refPicIdxL0; jj++ )
      {
        if( ( pLocalRPL1->getRefPicIdentifier( ii ) == pLocalRPL0->getRefPicIdentifier( jj ) ) && ( pLocalRPL1->isRefPicLongterm( ii ) == pLocalRPL0->isRefPicLongterm( jj ) ) && pLocalRPL1->getInterLayerRefPicIdx( ii ) == pLocalRPL0->getInterLayerRefPicIdx( jj ) )
        {
          canIncludeThis = false;
        }

        bool sameSign = ( pLocalRPL1->getRefPicIdentifier( ii ) > 0 ) == ( pLocalRPL0->getRefPicIdentifier( 0 ) > 0 );

        if( isDisallowMixedRefPic && canIncludeThis && !pLocalRPL1->isRefPicLongterm( ii ) && !sameSign )
        {
          canIncludeThis = false;
        }
      }
      if( canIncludeThis )
      {
        pLocalRPL0->setRefPicIdentifier( refPicIdxL0, pLocalRPL1->getRefPicIdentifier( ii ), pLocalRPL1->isRefPicLongterm( ii ), pLocalRPL1->isInterLayerRefPic( ii ), pLocalRPL1->getInterLayerRefPicIdx( ii ) );
        refPicIdxL0++;
        numOfSTRPL0 = numOfSTRPL0 + ( ( pLocalRPL1->isRefPicLongterm( ii ) ) ? 0 : 1 );
        numOfLTRPL0 += ( pLocalRPL1->isRefPicLongterm( ii ) && !pLocalRPL1->isInterLayerRefPic( ii ) ) ? 1 : 0;
        numOfILRPL0 += pLocalRPL1->isInterLayerRefPic( ii ) ? 1 : 0;
        numOfNeedToFill--;
      }
    }
  }
  pLocalRPL0->setNumberOfLongtermPictures( numOfLTRPL0 );
  pLocalRPL0->setNumberOfShorttermPictures( numOfSTRPL0 );
  pLocalRPL0->setNumberOfInterLayerPictures( numOfILRPL0 );
  int numPics = numOfLTRPL0 + numOfSTRPL0;

  pLocalRPL0->setNumberOfActivePictures( ( numPics < rpl0->getNumberOfActivePictures() ? numPics : rpl0->getNumberOfActivePictures() ) + numOfILRPL0 );
  pLocalRPL0->setLtrpInSliceHeaderFlag( 1 );
  slice->setRPL0idx( -1 );
  *slice->getRPL0() = localRPL0;

  //Copy from L0 if we have less than active ref pic
  numOfNeedToFill = pLocalRPL0->getNumberOfActivePictures() - ( numOfLTRPL1 + numOfSTRPL1 );

  for( int ii = 0; numOfNeedToFill > 0 && ii < ( pLocalRPL0->getNumberOfLongtermPictures() + pLocalRPL0->getNumberOfShorttermPictures() + pLocalRPL0->getNumberOfInterLayerPictures() ); ii++ )
  {
    if( ii <= ( originalL0StrpNum + originalL0LtrpNum + originalL0IlrpNum - 1 ) )
    {
      //Make sure this copy is not already in L0
      bool canIncludeThis = true;
      for( int jj = 0; jj < refPicIdxL1; jj++ )
      {
        if( ( pLocalRPL0->getRefPicIdentifier( ii ) == pLocalRPL1->getRefPicIdentifier( jj ) ) && ( pLocalRPL0->isRefPicLongterm( ii ) == pLocalRPL1->isRefPicLongterm( jj ) ) && pLocalRPL0->getInterLayerRefPicIdx( ii ) == pLocalRPL1->getInterLayerRefPicIdx( jj ) )
        {
          canIncludeThis = false;
        }

        bool sameSign = ( pLocalRPL0->getRefPicIdentifier( ii ) > 0 ) == ( pLocalRPL1->getRefPicIdentifier( 0 ) > 0 );

        if( isDisallowMixedRefPic && canIncludeThis && !pLocalRPL0->isRefPicLongterm( ii ) && !sameSign )
        {
          canIncludeThis = false;
        }
      }
      if( canIncludeThis )
      {
        pLocalRPL1->setRefPicIdentifier( refPicIdxL1, pLocalRPL0->getRefPicIdentifier( ii ), pLocalRPL0->isRefPicLongterm( ii ), pLocalRPL0->isInterLayerRefPic( ii ), pLocalRPL0->getInterLayerRefPicIdx( ii ) );
        refPicIdxL1++;
        numOfSTRPL1 = numOfSTRPL1 + ( ( pLocalRPL0->isRefPicLongterm( ii ) ) ? 0 : 1 );
        numOfLTRPL1 += ( pLocalRPL0->isRefPicLongterm( ii ) && !pLocalRPL0->isInterLayerRefPic( ii ) ) ? 1 : 0;
        numOfLTRPL1 += pLocalRPL0->isInterLayerRefPic( ii ) ? 1 : 0;
        numOfNeedToFill--;
      }
    }
  }
  pLocalRPL1->setNumberOfLongtermPictures( numOfLTRPL1 );
  pLocalRPL1->setNumberOfShorttermPictures( numOfSTRPL1 );
  pLocalRPL1->setNumberOfInterLayerPictures( numOfILRPL1 );
  numPics = numOfLTRPL1 + numOfSTRPL1;

  pLocalRPL1->setNumberOfActivePictures( ( isDisallowMixedRefPic ? numPics : ( numPics < rpl1->getNumberOfActivePictures() ? numPics : rpl1->getNumberOfActivePictures() ) ) + numOfILRPL1 );
  pLocalRPL1->setLtrpInSliceHeaderFlag( 1 );
  slice->setRPL1idx( -1 );
  *slice->getRPL1() = localRPL1;
}
//! \}

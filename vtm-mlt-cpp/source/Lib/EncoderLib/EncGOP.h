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

/** \file     EncGOP.h
    \brief    GOP encoder class (header)
*/

#ifndef __ENCGOP__
#define __ENCGOP__

#include <list>

#include <stdlib.h>

#include "CommonLib/Picture.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/NAL.h"
#include "EncSampleAdaptiveOffset.h"
#include "EncAdaptiveLoopFilter.h"
#include "EncReshape.h"
#include "EncSlice.h"
#include "VLCWriter.h"
#include "CABACWriter.h"
#include "SEIwrite.h"
#include "SEIEncoder.h"
#if EXTENSION_360_VIDEO
#include "AppEncHelper360/TExt360EncGop.h"
#endif

#include "Analyze.h"
#include "RateCtrl.h"
#include <vector>
#include "EncHRD.h"

#if JVET_O0756_CALCULATE_HDRMETRICS
#include "HDRLib/inc/ConvertColorFormat.H"
#include "HDRLib/inc/Convert.H"
#include "HDRLib/inc/ColorTransform.H"
#include "HDRLib/inc/TransferFunction.H"
#include "HDRLib/inc/DistortionMetricDeltaE.H"
#include <chrono>
#endif

//! \ingroup EncoderLib
//! \{

class EncLib;

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class AUWriterIf
{
public:
  virtual void outputAU( const AccessUnit& ) = 0;
};


class EncGOP
{
  class DUData
  {
  public:
    DUData()
    :accumBitsDU(0)
    ,accumNalsDU(0) {};

    int accumBitsDU;
    int accumNalsDU;
  };

private:

  Analyze                 m_gcAnalyzeAll;
  Analyze                 m_gcAnalyzeI;
  Analyze                 m_gcAnalyzeP;
  Analyze                 m_gcAnalyzeB;
#if WCG_WPSNR
  Analyze                 m_gcAnalyzeWPSNR;
#endif
  Analyze                 m_gcAnalyzeAll_in;
#if EXTENSION_360_VIDEO
  TExt360EncGop           m_ext360;
public:
  TExt360EncGop &getExt360Data() { return m_ext360; }
private:
#endif

  //  Data
  bool                    m_bLongtermTestPictureHasBeenCoded;
  bool                    m_bLongtermTestPictureHasBeenCoded2;
  uint32_t                    m_numLongTermRefPicSPS;
  uint32_t                    m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  bool                    m_ltRefPicUsedByCurrPicFlag[MAX_NUM_LONG_TERM_REF_PICS];
  int                     m_iLastIDR;
  int                     m_iGopSize;
  int                     m_iNumPicCoded;
  bool                    m_bFirst;
  int                     m_iLastRecoveryPicPOC;
  int                     m_latestDRAPPOC;
  int                     m_lastRasPoc;

  //  Access channel
  EncLib*                 m_pcEncLib;
  EncCfg*                 m_pcCfg;
  EncSlice*               m_pcSliceEncoder;
  PicList*                m_pcListPic;

  HLSWriter*              m_HLSWriter;
  LoopFilter*             m_pcLoopFilter;

  SEIWriter               m_seiWriter;

  Picture *               m_picBg;
  Picture *               m_picOrig;
  int                     m_bgPOC;
  bool                    m_isEncodedLTRef;
  bool                    m_isPrepareLTRef;
  bool                    m_isUseLTRef;
  int                     m_lastLTRefPoc;
  //--Adaptive Loop filter
  EncSampleAdaptiveOffset*  m_pcSAO;
  EncAdaptiveLoopFilter*    m_pcALF;
  EncReshape*               m_pcReshaper;
  RateCtrl*                 m_pcRateCtrl;
  // indicate sequence first
  bool                    m_bSeqFirst;
  bool                    m_audIrapOrGdrAuFlag;

  EncHRD*                 m_HRD;

  // clean decoding refresh
  bool                    m_bRefreshPending;
  int                     m_pocCRA;
  NalUnitType             m_associatedIRAPType[MAX_VPS_LAYERS];
  int                     m_associatedIRAPPOC[MAX_VPS_LAYERS];

  std::vector<int>        m_vRVM_RP;
  uint32_t                    m_lastBPSEI[MAX_TLAYER];
  uint32_t                    m_totalCoded[MAX_TLAYER];
  bool                        m_rapWithLeading;
  bool                    m_bufferingPeriodSEIPresentInAU;
  SEIEncoder              m_seiEncoder;
#if W0038_DB_OPT
  PelStorage*             m_pcDeblockingTempPicYuv;
  int                     m_DBParam[MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS][4];   //[layer_id][0: available; 1: bDBDisabled; 2: Beta Offset Div2; 3: Tc Offset Div2;]
#endif

  // members needed for adaptive max BT size
  uint32_t                    m_uiBlkSize[10];
  uint32_t                    m_uiNumBlk[10];
  uint32_t                    m_uiPrevISlicePOC;
  bool                    m_bInitAMaxBT;

  AUWriterIf*             m_AUWriterIf;

#if JVET_O0756_CALCULATE_HDRMETRICS

  hdrtoolslib::Frame **m_ppcFrameOrg;
  hdrtoolslib::Frame **m_ppcFrameRec;

  hdrtoolslib::ConvertColorFormat     *m_pcConvertFormat;
  hdrtoolslib::Convert                *m_pcConvertIQuantize;
  hdrtoolslib::ColorTransform         *m_pcColorTransform;
  hdrtoolslib::DistortionMetricDeltaE *m_pcDistortionDeltaE;
  hdrtoolslib::TransferFunction       *m_pcTransferFct;

  hdrtoolslib::ColorTransformParams   *m_pcColorTransformParams;
  hdrtoolslib::FrameFormat            *m_pcFrameFormat;

  std::chrono::duration<long long, ratio<1, 1000000000>> m_metricTime;
#endif

public:
  EncGOP();
  virtual ~EncGOP();

  void  create      ();
  void  destroy     ();

  void  init        ( EncLib* pcEncLib );

  void  compressGOP ( int iPOCLast, int iNumPicRcvd, PicList& rcListPic, std::list<PelUnitBuf*>& rcListPicYuvRec,
                      bool isField, bool isTff, const InputColourSpaceConversion snr_conversion, const bool printFrameMSE,
                      bool printMSSSIM, bool isEncodeLtRef, const int picIdInGOP);
  void  xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, OutputBitstream* pcBitstreamRedirect);


  int   getGOPSize()          { return  m_iGopSize;  }

  PicList*   getListPic()      { return m_pcListPic; }
  void      setPicBg(Picture* tmpPicBg) { m_picBg = tmpPicBg; }
  Picture*  getPicBg() const { return m_picBg; }
  void      setPicOrig(Picture* tmpPicBg) { m_picOrig = tmpPicBg; }
  Picture*  getPicOrig() { return m_picOrig; }
  void      setNewestBgPOC(int poc) { m_bgPOC = poc; }
  int       getNewestBgPOC() const { return m_bgPOC; }
  void      setEncodedLTRef(bool isEncodedLTRef) { m_isEncodedLTRef = isEncodedLTRef; }
  bool      getEncodedLTRef() { return m_isEncodedLTRef; }
  void      setUseLTRef(bool isUseLTRef) { m_isUseLTRef = isUseLTRef; }
  bool      getUseLTRef() { return m_isUseLTRef; }
  void      setPrepareLTRef(bool isPrepareLTRef) { m_isPrepareLTRef = isPrepareLTRef; }
  bool      getPrepareLTRef() { return m_isPrepareLTRef; }
  void      setLastLTRefPoc(int iLastLTRefPoc) { m_lastLTRefPoc = iLastLTRefPoc; }
  int       getLastLTRefPoc() const { return m_lastLTRefPoc; }

  void  printOutSummary( uint32_t uiNumAllPicCoded, bool isField, const bool printMSEBasedSNR, const bool printSequenceMSE, 
    const bool printMSSSIM, const bool printHexPsnr, const bool printRprPSNR, const BitDepths &bitDepths );
#if W0038_DB_OPT
  uint64_t  preLoopFilterPicAndCalcDist( Picture* pcPic );
#endif
  EncSlice*  getSliceEncoder()   { return m_pcSliceEncoder; }
  NalUnitType getNalUnitType( int pocCurr, int lastIdr, bool isField );
  void arrangeCompositeReference(Slice* pcSlice, PicList& rcListPic, int pocCurr);
  void updateCompositeReference(Slice* pcSlice, PicList& rcListPic, int pocCurr);

#if EXTENSION_360_VIDEO
  Analyze& getAnalyzeAllData() { return m_gcAnalyzeAll; }
  Analyze& getAnalyzeIData() { return m_gcAnalyzeI; }
  Analyze& getAnalyzePData() { return m_gcAnalyzeP; }
  Analyze& getAnalyzeBData() { return m_gcAnalyzeB; }
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
  std::chrono::duration<long long, ratio<1, 1000000000>> getMetricTime()    const { return m_metricTime; };
#endif

protected:
  RateCtrl* getRateCtrl()       { return m_pcRateCtrl;  }

protected:

  void  xInitGOP          ( int iPOCLast, int iNumPicRcvd, bool isField
    , bool isEncodeLtRef
  );
  void  xPicInitHashME( Picture *pic, const PPS *pps, PicList &rcListPic );
  void  xPicInitRateControl(int &estimatedBits, int gopId, double &lambda, Picture *pic, Slice *slice);
  void  xPicInitLMCS       (Picture *pic, PicHeader *picHeader, Slice *slice);
  void  xGetBuffer        ( PicList& rcListPic, std::list<PelUnitBuf*>& rcListPicYuvRecOut,
                            int iNumPicRcvd, int iTimeOffset, Picture*& rpcPic, int pocCurr, bool isField );

#if JVET_O0756_CALCULATE_HDRMETRICS
  void xCalculateHDRMetrics ( Picture* pcPic, double deltaE[hdrtoolslib::NB_REF_WHITE], double psnrL[hdrtoolslib::NB_REF_WHITE]);
  void copyBuftoFrame       ( Picture* pcPic );
#endif

  void  xCalculateAddPSNRs(const bool isField, const bool isFieldTopFieldFirst, const int iGOPid, Picture* pcPic, 
    const AccessUnit&accessUnit, PicList &rcListPic, int64_t dEncTime, const InputColourSpaceConversion snr_conversion, 
    const bool printFrameMSE, const bool printMSSSIM, double* PSNR_Y, bool isEncodeLtRef);
  void  xCalculateAddPSNR(Picture* pcPic, PelUnitBuf cPicD, const AccessUnit&, double dEncTime, const InputColourSpaceConversion snr_conversion, 
    const bool printFrameMSE, const bool printMSSSIM, double* PSNR_Y, bool isEncodeLtRef);
  void  xCalculateInterlacedAddPSNR( Picture* pcPicOrgFirstField, Picture* pcPicOrgSecondField,
                                     PelUnitBuf cPicRecFirstField, PelUnitBuf cPicRecSecondField,
                                     const InputColourSpaceConversion snr_conversion, const bool printFrameMSE, 
                                     const bool printMSSSIM, double* PSNR_Y, bool isEncodeLtRef);
  double xCalculateMSSSIM (const Pel* org, const int orgStride, const Pel* rec, const int recStride, 
    const int width, const int height, const uint32_t bitDepth);
  uint64_t xFindDistortionPlane(const CPelBuf& pic0, const CPelBuf& pic1, const uint32_t rshift
#if ENABLE_QPA
                            , const uint32_t chromaShiftHor = 0, const uint32_t chromaShiftVer = 0
#endif
                             );
#if WCG_WPSNR
  double xFindDistortionPlaneWPSNR(const CPelBuf& pic0, const CPelBuf& pic1, const uint32_t rshift, const CPelBuf& picLuma0, ComponentID compID, const ChromaFormat chfmt );
#endif
  double xCalculateRVM();

  void xUpdateRasInit(Slice* slice);

  void xWriteAccessUnitDelimiter (AccessUnit &accessUnit, Slice *slice);

  void xWriteFillerData (AccessUnit &accessUnit, Slice *slice, uint32_t &fdSize);

  void xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const SPS *sps, const PPS *pps);
  void xCreatePerPictureSEIMessages (int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, Slice *slice);
  void xCreateFrameFieldInfoSEI (SEIMessages& seiMessages, Slice *slice, bool isField);
  void xCreatePictureTimingSEI  (int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, Slice *slice, bool isField, std::deque<DUData> &duData);
  void xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData);
  void xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const SPS *sps);
  void xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI, int maxSubLayers);
  void xCreateScalableNestingSEI(SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, const std::vector<int> &targetOLSs, const std::vector<int> &targetLayers, const std::vector<uint16_t>& subpicIDs);
  void xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId);
  void xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId);
  void xClearSEIs(SEIMessages& seiMessages, bool deleteMessages);
  void xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, bool testWrite);
  void xWriteLeadingSEIMessages  (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, const SPS *sps, std::deque<DUData> &duData);
  void xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, int temporalId);
  void xWriteDuSEIMessages       (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, std::deque<DUData> &duData);

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  int xWriteOPI (AccessUnit &accessUnit, const OPI *opi);
#endif
  int xWriteVPS (AccessUnit &accessUnit, const VPS *vps);
  int xWriteDCI (AccessUnit &accessUnit, const DCI *dci);
  int xWriteSPS( AccessUnit &accessUnit, const SPS *sps, const int layerId = 0 );
  int xWritePPS( AccessUnit &accessUnit, const PPS *pps, const int layerId = 0 );
  int xWriteAPS( AccessUnit &accessUnit, APS *aps, const int layerId, const bool isPrefixNUT );
  int xWriteParameterSets(AccessUnit &accessUnit, Slice *slice, const bool bSeqFirst, const int layerIdx);
  int xWritePicHeader( AccessUnit &accessUnit, PicHeader *picHeader );

  void applyDeblockingFilterMetric( Picture* pcPic, uint32_t uiNumSlices );
#if W0038_DB_OPT
  void applyDeblockingFilterParameterSelection( Picture* pcPic, const uint32_t numSlices, const int gopID );
#endif
  void xCreateExplicitReferencePictureSetFromReference( Slice* slice, PicList& rcListPic, const ReferencePictureList *rpl0, const ReferencePictureList *rpl1 );
#if JVET_R0193
  bool xCheckMaxTidILRefPics(int layerIdx, Picture* refPic, bool currentPicIsIRAP);
#else
  bool xCheckMaxTidILRefPics(Picture* refPic, bool currentPicIsIRAP);
#endif
};// END CLASS DEFINITION EncGOP

//! \}

class EncBitstreamParams
{
public:
  EncBitstreamParams()
  : numBinsWritten(0)
  , numBytesInVclNalUnits(0)
  {};

  std::size_t numBinsWritten;
  std::size_t numBytesInVclNalUnits;
};

#endif // __ENCGOP__


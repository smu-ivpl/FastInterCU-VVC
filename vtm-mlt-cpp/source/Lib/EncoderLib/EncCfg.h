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

/** \file     EncCfg.h
    \brief    encoder configuration class (header)
*/

#ifndef __ENCCFG__
#define __ENCCFG__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CommonLib/CommonDef.h"
#include "CommonLib/Slice.h"

#include "CommonLib/Unit.h"

#include "EncCfgParam.h"

using namespace EncCfgParam;

#if JVET_O0756_CALCULATE_HDRMETRICS
#include "HDRLib/inc/DistortionMetric.H"
#endif

struct GOPEntry
{
  int m_POC;
  int m_QPOffset;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  double m_QPOffsetModelOffset;
  double m_QPOffsetModelScale;
#endif
#if W0038_CQP_ADJ
  int m_CbQPoffset;
  int m_CrQPoffset;
#endif
  double m_QPFactor;
  int m_tcOffsetDiv2;
  int m_betaOffsetDiv2;
  int m_CbTcOffsetDiv2;
  int m_CbBetaOffsetDiv2;
  int m_CrTcOffsetDiv2;
  int m_CrBetaOffsetDiv2;
  int m_temporalId;
  bool m_refPic;
  int8_t m_sliceType;
  int m_numRefPicsActive0;
  int m_numRefPics0;
  int m_deltaRefPics0[MAX_NUM_REF_PICS];
  int m_numRefPicsActive1;
  int m_numRefPics1;
  int m_deltaRefPics1[MAX_NUM_REF_PICS];
  bool m_isEncoded;
  bool m_ltrp_in_slice_header_flag;
  GOPEntry()
  : m_POC(-1)
  , m_QPOffset(0)
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  , m_QPOffsetModelOffset(0)
  , m_QPOffsetModelScale(0)
#endif
#if W0038_CQP_ADJ
  , m_CbQPoffset(0)
  , m_CrQPoffset(0)
#endif
  , m_QPFactor(0)
  , m_tcOffsetDiv2(0)
  , m_betaOffsetDiv2(0)
  , m_CbTcOffsetDiv2(0)
  , m_CbBetaOffsetDiv2(0)
  , m_CrTcOffsetDiv2(0)
  , m_CrBetaOffsetDiv2(0)
  , m_temporalId(0)
  , m_refPic(false)
  , m_sliceType('P')
    , m_numRefPicsActive0(0)
    , m_numRefPics0(0)
    , m_numRefPicsActive1(0)
    , m_numRefPics1(0)
    , m_isEncoded(false)
    , m_ltrp_in_slice_header_flag(false)
  {
    ::memset(m_deltaRefPics0, 0, sizeof(m_deltaRefPics0));
    ::memset(m_deltaRefPics1, 0, sizeof(m_deltaRefPics1));
  }
};

struct RPLEntry
{
  int m_POC;
  int m_temporalId;
  bool m_refPic;
  int m_numRefPicsActive;
  int8_t m_sliceType;
  int m_numRefPics;
  int m_deltaRefPics[MAX_NUM_REF_PICS];
  bool m_isEncoded;
  bool m_ltrp_in_slice_header_flag;
  RPLEntry()
    : m_POC(-1)
    , m_temporalId(0)
    , m_refPic(false)
    , m_numRefPicsActive(0)
    , m_sliceType('P')
    , m_numRefPics(0)
    , m_isEncoded(false)
    , m_ltrp_in_slice_header_flag(false)
  {
    ::memset(m_deltaRefPics, 0, sizeof(m_deltaRefPics));
  }
};

std::istringstream &operator>>(std::istringstream &in, GOPEntry &entry);     //input


//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class EncCfg
{
protected:
  //==== File I/O ========
  int       m_iFrameRate;
  int       m_FrameSkip;
  uint32_t      m_temporalSubsampleRatio;
  int       m_iSourceWidth;
  int       m_iSourceHeight;
  Window    m_conformanceWindow;
  int       m_framesToBeEncoded;
  double    m_adLambdaModifier[ MAX_TLAYER ];
  std::vector<double> m_adIntraLambdaModifier;
  double    m_dIntraQpFactor;                                 ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  bool      m_printMSEBasedSequencePSNR;
  bool      m_printHexPsnr;
  bool      m_printFrameMSE;
  bool      m_printSequenceMSE;
  bool      m_printMSSSIM;
  bool      m_cabacZeroWordPaddingEnabled;

  bool      m_gciPresentFlag;
  bool      m_onePictureOnlyConstraintFlag;
  bool      m_bIntraOnlyConstraintFlag;
  uint32_t  m_maxBitDepthConstraintIdc;
  int  m_maxChromaFormatConstraintIdc;
  bool      m_allLayersIndependentConstraintFlag;
  bool      m_noMrlConstraintFlag;
  bool      m_noIspConstraintFlag;
  bool      m_noMipConstraintFlag;
  bool      m_noLfnstConstraintFlag;
  bool      m_noMmvdConstraintFlag;
  bool      m_noSmvdConstraintFlag;
  bool      m_noProfConstraintFlag;
  bool      m_noPaletteConstraintFlag;
  bool      m_noActConstraintFlag;
  bool      m_noLmcsConstraintFlag;
  bool      m_noExplicitScaleListConstraintFlag;
  bool      m_noVirtualBoundaryConstraintFlag;
  bool      m_noMttConstraintFlag;
  bool      m_noChromaQpOffsetConstraintFlag;
  bool      m_noQtbttDualTreeIntraConstraintFlag;
  int       m_maxLog2CtuSizeConstraintIdc;
  bool      m_noPartitionConstraintsOverrideConstraintFlag;
  bool      m_noSaoConstraintFlag;
  bool      m_noAlfConstraintFlag;
  bool      m_noCCAlfConstraintFlag;
  bool      m_noWeightedPredictionConstraintFlag;
  bool      m_noRefWraparoundConstraintFlag;
  bool      m_noTemporalMvpConstraintFlag;
  bool      m_noSbtmvpConstraintFlag;
  bool      m_noAmvrConstraintFlag;
  bool      m_noBdofConstraintFlag;
  bool      m_noDmvrConstraintFlag;
  bool      m_noCclmConstraintFlag;
  bool      m_noMtsConstraintFlag;
  bool      m_noSbtConstraintFlag;
  bool      m_noAffineMotionConstraintFlag;
  bool      m_noBcwConstraintFlag;
  bool      m_noIbcConstraintFlag;
  bool      m_noCiipConstraintFlag;
  bool      m_noGeoConstraintFlag;
  bool      m_noLadfConstraintFlag;
  bool      m_noTransformSkipConstraintFlag;
  bool      m_noLumaTransformSize64ConstraintFlag;
  bool      m_noBDPCMConstraintFlag;
  bool      m_noJointCbCrConstraintFlag;
  bool      m_noCuQpDeltaConstraintFlag;
  bool      m_noDepQuantConstraintFlag;
  bool      m_noSignDataHidingConstraintFlag;
  bool      m_noTrailConstraintFlag;
  bool      m_noStsaConstraintFlag;
  bool      m_noRaslConstraintFlag;
  bool      m_noRadlConstraintFlag;
  bool      m_noIdrConstraintFlag;
  bool      m_noCraConstraintFlag;
  bool      m_noGdrConstraintFlag;
  bool      m_noApsConstraintFlag;

  /* profile & level */
  Profile::Name m_profile;
  Level::Tier   m_levelTier;
  Level::Name   m_level;
  bool m_frameOnlyConstraintFlag;
  bool m_multiLayerEnabledFlag;
  std::vector<uint32_t>      m_subProfile;
  uint8_t       m_numSubProfile;
  bool m_nonPackedConstraintFlag;
  bool m_nonProjectedConstraintFlag;
  bool m_noRprConstraintFlag;
  bool m_noResChangeInClvsConstraintFlag;
  bool m_oneTilePerPicConstraintFlag;
  bool m_picHeaderInSliceHeaderConstraintFlag;
  bool m_oneSlicePerPicConstraintFlag;
  bool m_noIdrRplConstraintFlag;
  bool m_noRectSliceConstraintFlag;
  bool m_oneSlicePerSubpicConstraintFlag;
  bool m_noSubpicInfoConstraintFlag;
  bool m_intraOnlyConstraintFlag;

  //====== Coding Structure ========
  int       m_intraPeriod;                        // needs to be signed to allow '-1' for no intra period
  uint32_t  m_decodingRefreshType;            ///< the type of decoding refresh employed for the random access.
  bool      m_rewriteParamSets;
  bool      m_idrRefParamList;
  int       m_iGOPSize;
  RPLEntry  m_RPLList0[MAX_GOP];
  RPLEntry  m_RPLList1[MAX_GOP];
  int       m_numRPLList0;
  int       m_numRPLList1;
  GOPEntry  m_GOPList[MAX_GOP];
  int       m_maxDecPicBuffering[MAX_TLAYER];
  int       m_maxNumReorderPics[MAX_TLAYER];
  int       m_drapPeriod;

  int       m_iQP;                              //  if (AdaptiveQP == OFF)
  ChromaQpMappingTableParams m_chromaQpMappingTableParams;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  int       m_intraQPOffset;                    ///< QP offset for intra slice (integer)
  int       m_lambdaFromQPEnable;               ///< enable lambda derivation from QP
#endif
  int       m_aiPad[2];

  bool      m_AccessUnitDelimiter;               ///< add Access Unit Delimiter NAL units
  bool      m_enablePictureHeaderInSliceHeader;  ///< Enable Picture Header in Slice Header

  int       m_iMaxRefPicNum;                     ///< this is used to mimic the sliding mechanism used by the decoder
                                                 // TODO: We need to have a common sliding mechanism used by both the encoder and decoder

  int       m_maxTempLayer;                      ///< Max temporal layer
  unsigned  m_CTUSize;
  bool                  m_subPicInfoPresentFlag;
  uint32_t              m_numSubPics;
  bool                  m_subPicSameSizeFlag;
  std::vector<uint32_t> m_subPicCtuTopLeftX;
  std::vector<uint32_t> m_subPicCtuTopLeftY;
  std::vector<uint32_t> m_subPicWidth;
  std::vector<uint32_t> m_subPicHeight;
  std::vector<bool>     m_subPicTreatedAsPicFlag;
  std::vector<bool>     m_loopFilterAcrossSubpicEnabledFlag;
  bool                  m_subPicIdMappingExplicitlySignalledFlag;
  bool                  m_subPicIdMappingInSpsFlag;
  unsigned              m_subPicIdLen;
  std::vector<uint16_t> m_subPicId;
  bool      m_useSplitConsOverride;
  unsigned  m_uiMinQT[3]; //0: I slice; 1: P/B slice, 2: I slice chroma
  unsigned  m_uiMaxBT[3]; //0: I slice; 1: P/B slice, 2: I slice chroma
  unsigned  m_uiMaxTT[3]; //0: I slice; 1: P/B slice, 2: I slice chroma
  unsigned  m_uiMaxMTTHierarchyDepth;
  unsigned  m_uiMaxMTTHierarchyDepthI;
  unsigned  m_uiMaxMTTHierarchyDepthIChroma;
  bool      m_dualITree;
  unsigned  m_maxCUWidth;
  unsigned  m_maxCUHeight;
  unsigned m_log2MinCUSize;

  int       m_LMChroma;
  bool      m_horCollocatedChromaFlag;
  bool      m_verCollocatedChromaFlag;
  int       m_IntraMTS;
  int       m_InterMTS;
  int       m_MTSIntraMaxCand;
  int       m_MTSInterMaxCand;
  int       m_ImplicitMTS;
  bool      m_SBT;                                ///< Sub-Block Transform for inter blocks
  int       m_SBTFast64WidthTh;                   ///< Enable size-64 SBT in encoder RDO check for HD and above sequences

  bool      m_LFNST;
  bool      m_useFastLFNST;
  bool      m_sbTmvpEnableFlag;
  bool      m_Affine;
  bool      m_AffineType;
  bool      m_PROF;
  bool      m_BIO;

  bool      m_SMVD;
  bool      m_compositeRefEnabled;        //composite reference
  bool      m_bcw;
  bool      m_BcwFast;
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool      m_LadfEnabled;
  int       m_LadfNumIntervals;
  int       m_LadfQpOffset[MAX_LADF_INTERVALS];
  int       m_LadfIntervalLowerBound[MAX_LADF_INTERVALS];
#endif

  bool      m_ciip;
  bool      m_Geo;
  bool      m_allowDisFracMMVD;
  bool      m_AffineAmvr;
  bool      m_HashME;
  bool      m_AffineAmvrEncOpt;
  bool      m_DMVR;
  bool      m_MMVD;
  int       m_MmvdDisNum;
  bool      m_rgbFormat;
  bool      m_useColorTrans;
  unsigned  m_PLTMode;
  bool      m_JointCbCrMode;
  unsigned  m_IBCMode;
  unsigned  m_IBCLocalSearchRangeX;
  unsigned  m_IBCLocalSearchRangeY;
  unsigned  m_IBCHashSearch;
  unsigned  m_IBCHashSearchMaxCand;
  unsigned  m_IBCHashSearchRange4SmallBlk;
  unsigned  m_IBCFastMethod;

  bool      m_wrapAround;
  unsigned  m_wrapAroundOffset;

  // ADD_NEW_TOOL : (encoder lib) add tool enabling flags and associated parameters here
  bool      m_virtualBoundariesEnabledFlag;
  bool      m_virtualBoundariesPresentFlag;
  unsigned  m_numVerVirtualBoundaries;
  unsigned  m_numHorVirtualBoundaries;
  unsigned  m_virtualBoundariesPosX[3];
  unsigned  m_virtualBoundariesPosY[3];
  bool      m_lmcsEnabled;
  unsigned  m_reshapeSignalType;
  unsigned  m_intraCMD;
  ReshapeCW m_reshapeCW;
  int       m_CSoffset;
  bool      m_encDbOpt;
  bool      m_useFastLCTU;
  bool      m_useFastMrg;
  bool      m_usePbIntraFast;
  bool      m_useAMaxBT;
  bool      m_e0023FastEnc;
  bool      m_contentBasedFastQtbt;
  bool      m_useNonLinearAlfLuma;
  bool      m_useNonLinearAlfChroma;
  unsigned  m_maxNumAlfAlternativesChroma;
  bool      m_MRL;
  bool      m_MIP;
  bool      m_useFastMIP;
  int       m_fastLocalDualTreeMode;
  uint32_t  m_log2MaxTbSize;

  //====== Loop/Deblock Filter ========
  bool      m_bLoopFilterDisable;
  bool      m_loopFilterOffsetInPPS;
  int       m_loopFilterBetaOffsetDiv2;
  int       m_loopFilterTcOffsetDiv2;
  int       m_loopFilterCbBetaOffsetDiv2;
  int       m_loopFilterCbTcOffsetDiv2;
  int       m_loopFilterCrBetaOffsetDiv2;
  int       m_loopFilterCrTcOffsetDiv2;
#if W0038_DB_OPT
  int       m_deblockingFilterMetric;
#else
  bool      m_DeblockingFilterMetric;
#endif
  bool      m_bUseSAO;
  bool      m_bTestSAODisableAtPictureLevel;
  double    m_saoEncodingRate;       // When non-0 SAO early picture termination is enabled for luma and chroma
  double    m_saoEncodingRateChroma; // The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  int       m_maxNumOffsetsPerPic;
  bool      m_saoCtuBoundary;

  bool      m_saoGreedyMergeEnc;
  //====== Motion search ========
  bool      m_bDisableIntraPUsInInterSlices;
  MESearchMethod m_motionEstimationSearchMethod;
  int       m_iSearchRange;                     //  0:Full frame
  int       m_bipredSearchRange;
  bool      m_bClipForBiPredMeEnabled;
  bool      m_bFastMEAssumingSmootherMVEnabled;
  int       m_minSearchWindow;
  bool      m_bRestrictMESampling;

  //====== Quality control ========
  int       m_iMaxDeltaQP;                      //  Max. absolute delta QP (1:default)
  int       m_cuQpDeltaSubdiv;                  //  Max. subdivision level for a CuDQP (0:default)
  int       m_cuChromaQpOffsetSubdiv;           ///< If negative, then do not apply chroma qp offsets.

  int       m_chromaCbQpOffset;                 //  Chroma Cb QP Offset (0:default)
  int       m_chromaCrQpOffset;                 //  Chroma Cr Qp Offset (0:default)
  int       m_chromaCbQpOffsetDualTree;         //  Chroma Cb QP Offset for dual tree
  int       m_chromaCrQpOffsetDualTree;         //  Chroma Cr Qp Offset for dual tree
  int       m_chromaCbCrQpOffset;               //  QP Offset for the joint Cb-Cr mode
  int       m_chromaCbCrQpOffsetDualTree;       //  QP Offset for the joint Cb-Cr mode in dual tree
#if ER_CHROMA_QP_WCG_PPS
  WCGChromaQPControl m_wcgChromaQpControl;                    ///< Wide-colour-gamut chroma QP control.
#endif
#if W0038_CQP_ADJ
  uint32_t      m_sliceChromaQpOffsetPeriodicity;                 ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  int       m_sliceChromaQpOffsetIntraOrPeriodic[2/*Cb,Cr*/]; ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
#endif

  ChromaFormat m_chromaFormatIDC;

  bool      m_extendedPrecisionProcessingFlag;
  bool      m_highPrecisionOffsetsEnabledFlag;
  bool      m_bUseAdaptiveQP;
  int       m_iQPAdaptationRange;
#if ENABLE_QPA
  bool      m_bUsePerceptQPA;
  bool      m_bUseWPSNR;
#endif

  //====== Tool list ========
  int       m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of input file
  int       m_bitDepth[MAX_NUM_CHANNEL_TYPE];
  bool      m_bUseASR;
  bool      m_bUseHADME;
  bool      m_useRDOQ;
  bool      m_useRDOQTS;
#if T0196_SELECTIVE_RDOQ
  bool      m_useSelectiveRDOQ;
#endif
  uint32_t      m_rdPenalty;
  FastInterSearchMode m_fastInterSearchMode;
  bool      m_bUseEarlyCU;
  bool      m_useFastDecisionForMerge;
  bool      m_useEarlySkipDetection;
  bool      m_reconBasedCrossCPredictionEstimate;
  bool      m_useTransformSkip;
  bool      m_useTransformSkipFast;
  bool      m_useChromaTS;
  bool      m_useBDPCM;
  uint32_t      m_log2MaxTransformSkipBlockSize;
  bool      m_transformSkipRotationEnabledFlag;
  bool      m_transformSkipContextEnabledFlag;
  bool      m_persistentRiceAdaptationEnabledFlag;
  bool      m_cabacBypassAlignmentEnabledFlag;
#if SHARP_LUMA_DELTA_QP
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping; ///< mapping from luma level to delta QP.
#endif
  int*      m_aidQP;
  uint32_t      m_uiDeltaQpRD;
  bool      m_bFastDeltaQP;
  bool      m_ISP;
  bool      m_useFastISP;

  bool      m_bFastUDIUseMPMEnabled;
  bool      m_bFastMEForGenBLowDelayEnabled;
  bool      m_bUseBLambdaForNonKeyLowDelayPictures;
  bool      m_gopBasedTemporalFilterEnabled;
  bool      m_noPicPartitionFlag;                             ///< no picture partitioning flag (single tile, single slice)
  bool      m_mixedLossyLossless;                             ///< enable mixed lossy/lossless coding
  std::vector<uint16_t> m_sliceLosslessArray;                      ///< Slice lossless array
  std::vector<uint32_t> m_tileColumnWidth;                    ///< tile column widths in units of CTUs (last column width will be repeated uniformly to cover any remaining picture width)
  std::vector<uint32_t> m_tileRowHeight;                      ///< tile row heights in units of CTUs (last row height will be repeated uniformly to cover any remaining picture height)
  bool      m_rectSliceFlag;                                  ///< indicates if using rectangular or raster-scan slices
  uint32_t  m_numSlicesInPic;                                 ///< number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool      m_tileIdxDeltaPresentFlag;                        ///< rectangular slice tile index delta present flag
  std::vector<RectSlice> m_rectSlices;                        ///< list of rectanglar slice syntax parameters
  std::vector<uint32_t> m_rasterSliceSize;                    ///< raster-scan slice sizes in units of tiles
  bool      m_bLFCrossTileBoundaryFlag;                       ///< 1: filter across tile boundaries  0: do not filter across tile boundaries
  bool      m_bLFCrossSliceBoundaryFlag;                      ///< 1: filter across slice boundaries 0: do not filter across slice boundaries

  bool      m_intraSmoothingDisabledFlag;
  //====== Sub-picture and Slices ========
  bool      m_singleSlicePerSubPicFlag;
  bool      m_entropyCodingSyncEnabledFlag;
  bool      m_entryPointPresentFlag;                           ///< flag for the presence of entry points

  HashType  m_decodedPictureHashSEIType;
  HashType  m_subpicDecodedPictureHashType;
  bool      m_bufferingPeriodSEIEnabled;
  bool      m_pictureTimingSEIEnabled;
  bool      m_frameFieldInfoSEIEnabled;
  bool      m_dependentRAPIndicationSEIEnabled;
  bool      m_framePackingSEIEnabled;
  int       m_framePackingSEIType;
  int       m_framePackingSEIId;
  int       m_framePackingSEIQuincunx;
  int       m_framePackingSEIInterpretation;
  bool      m_parameterSetsInclusionIndicationSEIEnabled;
  bool      m_selfContainedClvsFlag;
  bool      m_bpDeltasGOPStructure;
  bool      m_decodingUnitInfoSEIEnabled;

  bool      m_scalableNestingSEIEnabled;

  bool      m_erpSEIEnabled;
  bool      m_erpSEICancelFlag;
  bool      m_erpSEIPersistenceFlag;
  bool      m_erpSEIGuardBandFlag;
  uint32_t  m_erpSEIGuardBandType;
  uint32_t  m_erpSEILeftGuardBandWidth;
  uint32_t  m_erpSEIRightGuardBandWidth;
  bool      m_sphereRotationSEIEnabled;
  bool      m_sphereRotationSEICancelFlag;
  bool      m_sphereRotationSEIPersistenceFlag;
  int       m_sphereRotationSEIYaw;
  int       m_sphereRotationSEIPitch;
  int       m_sphereRotationSEIRoll;
  bool      m_omniViewportSEIEnabled;
  uint32_t  m_omniViewportSEIId;
  bool      m_omniViewportSEICancelFlag;
  bool      m_omniViewportSEIPersistenceFlag;
  uint32_t  m_omniViewportSEICntMinus1;
  std::vector<int>      m_omniViewportSEIAzimuthCentre;
  std::vector<int>      m_omniViewportSEIElevationCentre;
  std::vector<int>      m_omniViewportSEITiltCentre;
  std::vector<uint32_t> m_omniViewportSEIHorRange;
  std::vector<uint32_t> m_omniViewportSEIVerRange;
  bool                  m_rwpSEIEnabled;
  bool                  m_rwpSEIRwpCancelFlag;
  bool                  m_rwpSEIRwpPersistenceFlag;
  bool                  m_rwpSEIConstituentPictureMatchingFlag;
  int                   m_rwpSEINumPackedRegions;
  int                   m_rwpSEIProjPictureWidth;
  int                   m_rwpSEIProjPictureHeight;
  int                   m_rwpSEIPackedPictureWidth;
  int                   m_rwpSEIPackedPictureHeight;
  std::vector<uint8_t>  m_rwpSEIRwpTransformType;
  std::vector<bool>     m_rwpSEIRwpGuardBandFlag;
  std::vector<uint32_t> m_rwpSEIProjRegionWidth;
  std::vector<uint32_t> m_rwpSEIProjRegionHeight;
  std::vector<uint32_t> m_rwpSEIRwpSEIProjRegionTop;
  std::vector<uint32_t> m_rwpSEIProjRegionLeft;
  std::vector<uint16_t> m_rwpSEIPackedRegionWidth;
  std::vector<uint16_t> m_rwpSEIPackedRegionHeight;
  std::vector<uint16_t> m_rwpSEIPackedRegionTop;
  std::vector<uint16_t> m_rwpSEIPackedRegionLeft;
  std::vector<uint8_t>  m_rwpSEIRwpLeftGuardBandWidth;
  std::vector<uint8_t>  m_rwpSEIRwpRightGuardBandWidth;
  std::vector<uint8_t>  m_rwpSEIRwpTopGuardBandHeight;
  std::vector<uint8_t>  m_rwpSEIRwpBottomGuardBandHeight;
  std::vector<bool>     m_rwpSEIRwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  m_rwpSEIRwpGuardBandType;
  bool                 m_gcmpSEIEnabled;
  bool                 m_gcmpSEICancelFlag;
  bool                 m_gcmpSEIPersistenceFlag;
  uint8_t              m_gcmpSEIPackingType;
  uint8_t              m_gcmpSEIMappingFunctionType;
  std::vector<uint8_t> m_gcmpSEIFaceIndex;
  std::vector<uint8_t> m_gcmpSEIFaceRotation;
  std::vector<double>  m_gcmpSEIFunctionCoeffU;
  std::vector<bool>    m_gcmpSEIFunctionUAffectedByVFlag;
  std::vector<double>  m_gcmpSEIFunctionCoeffV;
  std::vector<bool>    m_gcmpSEIFunctionVAffectedByUFlag;
  bool                 m_gcmpSEIGuardBandFlag;
  uint8_t              m_gcmpSEIGuardBandType;
  bool                 m_gcmpSEIGuardBandBoundaryExteriorFlag;
  uint8_t              m_gcmpSEIGuardBandSamplesMinus1;
  CfgSEISubpictureLevel m_cfgSubpictureLevelInfoSEI;
  bool                  m_sampleAspectRatioInfoSEIEnabled;
  bool                  m_sariCancelFlag;
  bool                  m_sariPersistenceFlag;
  int                   m_sariAspectRatioIdc;
  int                   m_sariSarWidth;
  int                   m_sariSarHeight;
  bool      m_MCTSEncConstraint;
  SEIMasteringDisplay m_masteringDisplay;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  bool      m_alternativeTransferCharacteristicsSEIEnabled;
  uint8_t     m_preferredTransferCharacteristics;
#endif
  // film grain characterstics sei
  bool      m_fgcSEIEnabled;
  bool      m_fgcSEICancelFlag;
  bool      m_fgcSEIPersistenceFlag;
  uint8_t   m_fgcSEIModelID;
  bool      m_fgcSEISepColourDescPresentFlag;
  uint8_t   m_fgcSEIBlendingModeID;
  uint8_t   m_fgcSEILog2ScaleFactor;
  bool      m_fgcSEICompModelPresent[MAX_NUM_COMPONENT];
// cll SEI
  bool      m_cllSEIEnabled;
  uint16_t  m_cllSEIMaxContentLevel;
  uint16_t  m_cllSEIMaxPicAvgLevel;
// ave sei
  bool      m_aveSEIEnabled;
  uint32_t  m_aveSEIAmbientIlluminance;
  uint16_t  m_aveSEIAmbientLightX;
  uint16_t  m_aveSEIAmbientLightY;
// ccv sei
  bool      m_ccvSEIEnabled;
  bool      m_ccvSEICancelFlag;
  bool      m_ccvSEIPersistenceFlag;
  bool      m_ccvSEIPrimariesPresentFlag;
  bool      m_ccvSEIMinLuminanceValuePresentFlag;
  bool      m_ccvSEIMaxLuminanceValuePresentFlag;
  bool      m_ccvSEIAvgLuminanceValuePresentFlag;
  double    m_ccvSEIPrimariesX[MAX_NUM_COMPONENT];
  double    m_ccvSEIPrimariesY[MAX_NUM_COMPONENT];
  double    m_ccvSEIMinLuminanceValue;
  double    m_ccvSEIMaxLuminanceValue;
  double    m_ccvSEIAvgLuminanceValue;
  //====== Weighted Prediction ========
  bool      m_useWeightedPred;       //< Use of Weighting Prediction (P_SLICE)
  bool      m_useWeightedBiPred;    //< Use of Bi-directional Weighting Prediction (B_SLICE)
  WeightedPredictionMethod m_weightedPredictionMethod;
  uint32_t      m_log2ParallelMergeLevelMinus2;       ///< Parallel merge estimation region
  uint32_t      m_maxNumMergeCand;                    ///< Maximum number of merge candidates
  uint32_t      m_maxNumAffineMergeCand;              ///< Maximum number of affine merge candidates
  uint32_t      m_maxNumGeoCand;
  uint32_t      m_maxNumIBCMergeCand;                 ///< Max number of IBC merge candidates
  ScalingListMode m_useScalingListId;             ///< Using quantization matrix i.e. 0=off, 1=default, 2=file.
  std::string m_scalingListFileName;              ///< quantization matrix file name
  bool      m_disableScalingMatrixForAlternativeColourSpace;
  bool      m_scalingMatrixDesignatedColourSpace;
  bool      m_sliceLevelRpl;                      ///< code reference picture lists in slice headers rather than picture header
  bool      m_sliceLevelDblk;                     ///< code deblocking filter parameters in slice headers rather than picture header
  bool      m_sliceLevelSao;                      ///< code SAO parameters in slice headers rather than picture header
  bool      m_sliceLevelAlf;                      ///< code ALF parameters in slice headers rather than picture header
  bool      m_sliceLevelWp;                       ///< code weighted prediction parameters in slice headers rather than picture header
  bool      m_sliceLevelDeltaQp;                  ///< code delta in slice headers rather than picture header
  bool      m_disableScalingMatrixForLfnstBlks;
  int       m_TMVPModeId;
  bool      m_constantSliceHeaderParamsEnabledFlag;
  int       m_PPSDepQuantEnabledIdc;
  int       m_PPSRefPicListSPSIdc0;
  int       m_PPSRefPicListSPSIdc1;
  int       m_PPSMvdL1ZeroIdc;
  int       m_PPSCollocatedFromL0Idc;
  uint32_t  m_PPSSixMinusMaxNumMergeCandPlus1;
  uint32_t  m_PPSMaxNumMergeCandMinusMaxNumGeoCandPlus1;
  bool      m_DepQuantEnabledFlag;
  bool      m_SignDataHidingEnabledFlag;
  bool      m_RCEnableRateControl;
  int       m_RCTargetBitrate;
  int       m_RCKeepHierarchicalBit;
  bool      m_RCLCULevelRC;
  bool      m_RCUseLCUSeparateModel;
  int       m_RCInitialQP;
  bool      m_RCForceIntraQP;
#if U0132_TARGET_BITS_SATURATION
  bool      m_RCCpbSaturationEnabled;
  uint32_t      m_RCCpbSize;
  double    m_RCInitialCpbFullness;
#endif
  CostMode  m_costMode;                                       ///< The cost function to use, primarily when considering lossless coding.
  bool      m_TSRCdisableLL;                                  ///< Disable TSRC for lossless

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  OPI       m_opi;
  bool      m_OPIEnabled;                                     ///< enable Operating Point Information (OPI)
#endif

  DCI       m_dci;
  bool      m_DCIEnabled;                                     ///< enable Decoding Capability Information (DCI)

  bool      m_recalculateQPAccordingToLambda;                 ///< recalculate QP value according to the lambda value
  bool      m_hrdParametersPresentFlag;                       ///< enable generation of HRD parameters
  bool      m_vuiParametersPresentFlag;                       ///< enable generation of VUI parameters
  bool      m_samePicTimingInAllOLS;                          ///< same picture timing SEI message is used in all OLS
  bool      m_aspectRatioInfoPresentFlag;                     ///< Signals whether aspect_ratio_idc is present
  int       m_aspectRatioIdc;                                 ///< aspect_ratio_idc
  int       m_sarWidth;                                       ///< horizontal size of the sample aspect ratio
  int       m_sarHeight;                                      ///< vertical size of the sample aspect ratio
  bool      m_colourDescriptionPresentFlag;                   ///< Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  int       m_colourPrimaries;                                ///< Indicates chromaticity coordinates of the source primaries
  int       m_transferCharacteristics;                        ///< Indicates the opto-electronic transfer characteristics of the source
  int       m_matrixCoefficients;                             ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool      m_progressiveSourceFlag;                          ///< Indicates if the content is progressive
  bool      m_interlacedSourceFlag;                           ///< Indicates if the content is interlaced
  bool      m_chromaLocInfoPresentFlag;                       ///< Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  int       m_chromaSampleLocTypeTopField;                    ///< Specifies the location of chroma samples for top field
  int       m_chromaSampleLocTypeBottomField;                 ///< Specifies the location of chroma samples for bottom field
  int       m_chromaSampleLocType;                            ///< Specifies the location of chroma samples for progressive content
  bool      m_overscanInfoPresentFlag;                        ///< Signals whether overscan_appropriate_flag is present
  bool      m_overscanAppropriateFlag;                        ///< Indicates whether conformant decoded pictures are suitable for display using overscan
  bool      m_videoFullRangeFlag;                             ///< Indicates the black level and range of luma and chroma signals

  bool      m_bEfficientFieldIRAPEnabled;                     ///< enable to code fields in a specific, potentially more efficient, order.
  bool      m_bHarmonizeGopFirstFieldCoupleEnabled;

  std::string m_summaryOutFilename;                           ///< filename to use for producing summary output file.
  std::string m_summaryPicFilenameBase;                       ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  uint32_t        m_summaryVerboseness;                           ///< Specifies the level of the verboseness of the text output.
  int       m_ImvMode;
  int       m_Imv4PelFast;
  std::string m_decodeBitstreams[2];                          ///< filename for decode bitstreams.
  bool        m_forceDecodeBitstream1;                        ///< guess what it means
  int         m_switchPOC;                                    ///< dbg poc.
  int         m_switchDQP;                                    ///< dqp applied to  switchPOC and subsequent pictures.
  int         m_fastForwardToPOC;                             ///<
  bool        m_stopAfterFFtoPOC;                             ///<
  int         m_debugCTU;                                     ///< dbg ctu
  bool        m_bs2ModPOCAndType;

  CfgVPSParameters m_cfgVPSParameters;

#if ENABLE_SPLIT_PARALLELISM
  int         m_numSplitThreads;
  bool        m_forceSingleSplitThread;
#endif

  bool        m_alf;                                          ///< Adaptive Loop Filter
#if JVET_T0064
  double      m_alfStrength;
  bool        m_alfAllowPredefinedFilters;
  double      m_ccalfStrength;
#endif
  bool        m_ccalf;
  int         m_ccalfQpThreshold;
#if JVET_O0756_CALCULATE_HDRMETRICS
  double                       m_whitePointDeltaE[hdrtoolslib::NB_REF_WHITE];
  double                       m_maxSampleValue;
  hdrtoolslib::SampleRange     m_sampleRange;
  hdrtoolslib::ColorPrimaries  m_colorPrimaries;
  bool                         m_enableTFunctionLUT;
  hdrtoolslib::ChromaLocation  m_chromaLocation[2];
  int                          m_chromaUPFilter;
  int                          m_cropOffsetLeft;
  int                          m_cropOffsetTop;
  int                          m_cropOffsetRight;
  int                          m_cropOffsetBottom;
  bool                         m_calculateHdrMetrics;
#endif
  double      m_scalingRatioHor;
  double      m_scalingRatioVer;
  bool        m_rprEnabledFlag;
  bool        m_resChangeInClvsEnabled;
  int         m_switchPocPeriod;
  int         m_upscaledOutput;
  int         m_numRefLayers[MAX_VPS_LAYERS];
  bool        m_avoidIntraInDepLayer;

public:
  EncCfg()
  {
  }

  virtual ~EncCfg()
  {}

  void setProfile(Profile::Name profile) { m_profile = profile; }
  void setLevel(Level::Tier tier, Level::Name level) { m_levelTier = tier; m_level = level; }
  bool      getFrameOnlyConstraintFlag() const { return m_frameOnlyConstraintFlag; }
  void      setFrameOnlyConstraintFlag(bool b) { m_frameOnlyConstraintFlag = b;    }
  bool      getMultiLayerEnabledFlag() const   { return m_multiLayerEnabledFlag;   }
  void      setMultiLayerEnabledFlag(bool b)   { m_multiLayerEnabledFlag = b;      }
  void setNumSubProfile( uint8_t numSubProfile) { m_numSubProfile = numSubProfile; m_subProfile.resize(m_numSubProfile); }
  void setSubProfile( int i, uint32_t subProfile) { m_subProfile[i] = subProfile; }

  bool      getOnePictureOnlyConstraintFlag() const                        { return m_onePictureOnlyConstraintFlag; }
  void      setOnePictureOnlyConstraintFlag(bool b)                        { m_onePictureOnlyConstraintFlag=b; }

  bool      getIntraOnlyConstraintFlag() const { return m_bIntraOnlyConstraintFlag; }
  void      setIntraOnlyConstraintFlag(bool val) { m_bIntraOnlyConstraintFlag = val; }
  uint32_t  getMaxBitDepthConstraintIdc() const { return m_maxBitDepthConstraintIdc; }
  void      setMaxBitDepthConstraintIdc(uint32_t u) { m_maxBitDepthConstraintIdc = u; }
  int       getMaxChromaFormatConstraintIdc() const { return m_maxChromaFormatConstraintIdc; }
  void      setMaxChromaFormatConstraintIdc(int u) { m_maxChromaFormatConstraintIdc = u; }
  bool          getGciPresentFlag() const { return m_gciPresentFlag; }
  void          setGciPresentFlag(bool b) { m_gciPresentFlag = b; }
  bool          getAllLayersIndependentConstraintFlag() const { return m_allLayersIndependentConstraintFlag; }
  void          setAllLayersIndependentConstraintFlag(bool val) { m_allLayersIndependentConstraintFlag = val; }
  bool          getNoMrlConstraintFlag() const { return m_noMrlConstraintFlag; }
  void          setNoMrlConstraintFlag(bool val) { m_noMrlConstraintFlag = val; }
  bool          getNoIspConstraintFlag() const { return m_noIspConstraintFlag; }
  void          setNoIspConstraintFlag(bool val) { m_noIspConstraintFlag = val; }
  bool          getNoMipConstraintFlag() const { return m_noMipConstraintFlag; }
  void          setNoMipConstraintFlag(bool val) { m_noMipConstraintFlag = val; }
  bool          getNoLfnstConstraintFlag() const { return m_noLfnstConstraintFlag; }
  void          setNoLfnstConstraintFlag(bool val) { m_noLfnstConstraintFlag = val; }
  bool          getNoMmvdConstraintFlag() const { return m_noMmvdConstraintFlag; }
  void          setNoMmvdConstraintFlag(bool val) { m_noMmvdConstraintFlag = val; }
  bool          getNoSmvdConstraintFlag() const { return m_noSmvdConstraintFlag; }
  void          setNoSmvdConstraintFlag(bool val) { m_noSmvdConstraintFlag = val; }
  bool          getNoProfConstraintFlag() const { return m_noProfConstraintFlag; }
  void          setNoProfConstraintFlag(bool val) { m_noProfConstraintFlag = val; }
  bool          getNoPaletteConstraintFlag() const { return m_noPaletteConstraintFlag; }
  void          setNoPaletteConstraintFlag(bool val) { m_noPaletteConstraintFlag = val; }
  bool          getNoActConstraintFlag() const { return m_noActConstraintFlag; }
  void          setNoActConstraintFlag(bool val) { m_noActConstraintFlag = val; }
  bool          getNoLmcsConstraintFlag() const { return m_noLmcsConstraintFlag; }
  void          setNoLmcsConstraintFlag(bool val) { m_noLmcsConstraintFlag = val; }
  bool          getNoExplicitScaleListConstraintFlag() const { return m_noExplicitScaleListConstraintFlag; }
  void          setNoExplicitScaleListConstraintFlag(bool val) { m_noExplicitScaleListConstraintFlag = val; }
  bool          getNoVirtualBoundaryConstraintFlag() const { return m_noVirtualBoundaryConstraintFlag; }
  void          setNoVirtualBoundaryConstraintFlag(bool val) { m_noVirtualBoundaryConstraintFlag = val; }
  bool          getNoMttConstraintFlag() const { return m_noMttConstraintFlag; }
  void          setNoMttConstraintFlag(bool val) { m_noMttConstraintFlag = val; }
  bool      getNoChromaQpOffsetConstraintFlag() const { return m_noChromaQpOffsetConstraintFlag; }
  void      setNoChromaQpOffsetConstraintFlag(bool bVal) { m_noChromaQpOffsetConstraintFlag = bVal; }
  bool      getNoQtbttDualTreeIntraConstraintFlag() const { return m_noQtbttDualTreeIntraConstraintFlag; }
  void      setNoQtbttDualTreeIntraConstraintFlag(bool val) { m_noQtbttDualTreeIntraConstraintFlag = val; }
  int       getMaxLog2CtuSizeConstraintIdc() const { return m_maxLog2CtuSizeConstraintIdc; }
  void      setMaxLog2CtuSizeConstraintIdc(int u) { m_maxLog2CtuSizeConstraintIdc = u; }
  bool      getNoPartitionConstraintsOverrideConstraintFlag() const { return m_noPartitionConstraintsOverrideConstraintFlag; }
  void      setNoPartitionConstraintsOverrideConstraintFlag(bool val) { m_noPartitionConstraintsOverrideConstraintFlag = val; }
  bool      getNoSaoConstraintFlag() const { return m_noSaoConstraintFlag; }
  void      setNoSaoConstraintFlag(bool val) { m_noSaoConstraintFlag = val; }
  bool      getNoAlfConstraintFlag() const { return m_noAlfConstraintFlag; }
  void      setNoAlfConstraintFlag(bool val) { m_noAlfConstraintFlag = val; }
  bool      getNoCCAlfConstraintFlag() const { return m_noCCAlfConstraintFlag; }
  void      setNoCCAlfConstraintFlag(bool val) { m_noCCAlfConstraintFlag = val; }
  bool      getWeightedPredictionConstraintFlag() const { return m_noWeightedPredictionConstraintFlag; }
  void      setNoWeightedPredictionConstraintFlag(bool val) { m_noWeightedPredictionConstraintFlag = val; }
  bool      getNoRefWraparoundConstraintFlag() const { return m_noRefWraparoundConstraintFlag; }
  void      setNoRefWraparoundConstraintFlag(bool val) { m_noRefWraparoundConstraintFlag = val; }
  bool      getNoTemporalMvpConstraintFlag() const { return m_noTemporalMvpConstraintFlag; }
  void      setNoTemporalMvpConstraintFlag(bool val) { m_noTemporalMvpConstraintFlag = val; }
  bool      getNoSbtmvpConstraintFlag() const { return m_noSbtmvpConstraintFlag; }
  void      setNoSbtmvpConstraintFlag(bool val) { m_noSbtmvpConstraintFlag = val; }
  bool      getNoAmvrConstraintFlag() const { return m_noAmvrConstraintFlag; }
  void      setNoAmvrConstraintFlag(bool val) { m_noAmvrConstraintFlag = val; }
  bool      getNoBdofConstraintFlag() const { return m_noBdofConstraintFlag; }
  void      setNoBdofConstraintFlag(bool val) { m_noBdofConstraintFlag = val; }
  bool      getNoDmvrConstraintFlag() const { return m_noDmvrConstraintFlag; }
  void      setNoDmvrConstraintFlag(bool val) { m_noDmvrConstraintFlag = val; }
  bool      getNoCclmConstraintFlag() const { return m_noCclmConstraintFlag; }
  void      setNoCclmConstraintFlag(bool val) { m_noCclmConstraintFlag = val; }
  bool      getNoMtsConstraintFlag() const { return m_noMtsConstraintFlag; }
  void      setNoMtsConstraintFlag(bool val) { m_noMtsConstraintFlag = val; }
  bool      getNoSbtConstraintFlag() const { return m_noSbtConstraintFlag; }
  void      setNoSbtConstraintFlag(bool val) { m_noSbtConstraintFlag = val; }
  bool      getNoAffineMotionConstraintFlag() const { return m_noAffineMotionConstraintFlag; }
  void      setNoAffineMotionConstraintFlag(bool val) { m_noAffineMotionConstraintFlag = val; }
  bool      getNoBcwConstraintFlag() const { return m_noBcwConstraintFlag; }
  void      setNoBcwConstraintFlag(bool val) { m_noBcwConstraintFlag = val; }
  bool      getNoIbcConstraintFlag() const { return m_noIbcConstraintFlag; }
  void      setNoIbcConstraintFlag(bool val) { m_noIbcConstraintFlag = val; }
  bool      getNoCiipConstraintFlag() const { return m_noCiipConstraintFlag; }
  void      setNoCiipConstraintFlag(bool val) { m_noCiipConstraintFlag = val; }
  bool      getNoGeoConstraintFlag() const { return m_noGeoConstraintFlag; }
  void      setNoGeoConstraintFlag(bool val) { m_noGeoConstraintFlag = val; }
  bool      getNoLadfConstraintFlag() const { return m_noLadfConstraintFlag; }
  void      setNoLadfConstraintFlag(bool val) { m_noLadfConstraintFlag = val; }
  bool      getNoTransformSkipConstraintFlag() const { return m_noTransformSkipConstraintFlag; }
  void      setNoTransformSkipConstraintFlag(bool val) { m_noTransformSkipConstraintFlag = val; }
  bool      getNoLumaTransformSize64ConstraintFlag() const { return m_noLumaTransformSize64ConstraintFlag; }
  void      setNoLumaTransformSize64ConstraintFlag(bool val) { m_noLumaTransformSize64ConstraintFlag = val; }
  bool      getNoBDPCMConstraintFlag() const { return m_noBDPCMConstraintFlag; }
  void      setNoBDPCMConstraintFlag(bool val) { m_noBDPCMConstraintFlag = val; }
  bool      getNoJointCbCrConstraintFlag() const { return m_noJointCbCrConstraintFlag; }
  void      setNoJointCbCrConstraintFlag(bool val) { m_noJointCbCrConstraintFlag = val; }
  bool      getNoCuQpDeltaConstraintFlag() const { return m_noCuQpDeltaConstraintFlag; }
  void      setNoCuQpDeltaConstraintFlag(bool val) { m_noCuQpDeltaConstraintFlag = val; }
  bool      getNoDepQuantConstraintFlag() const { return m_noDepQuantConstraintFlag; }
  void      setNoDepQuantConstraintFlag(bool val) { m_noDepQuantConstraintFlag = val; }
  bool      getNoSignDataHidingConstraintFlag() const { return m_noSignDataHidingConstraintFlag; }
  void      setNoSignDataHidingConstraintFlag(bool val) { m_noSignDataHidingConstraintFlag = val; }
  bool      getNoTrailConstraintFlag() const { return m_noTrailConstraintFlag; }
  void      setNoTrailConstraintFlag(bool val) { m_noTrailConstraintFlag = val; }
  bool      getNoStsaConstraintFlag() const { return m_noStsaConstraintFlag; }
  void      setNoStsaConstraintFlag(bool val) { m_noStsaConstraintFlag = val; }
  bool      getNoRaslConstraintFlag() const { return m_noRaslConstraintFlag; }
  void      setNoRaslConstraintFlag(bool val) { m_noRaslConstraintFlag = val; }
  bool      getNoRadlConstraintFlag() const { return m_noRadlConstraintFlag; }
  void      setNoRadlConstraintFlag(bool val) { m_noRadlConstraintFlag = val; }
  bool      getNoIdrConstraintFlag() const { return m_noIdrConstraintFlag; }
  void      setNoIdrConstraintFlag(bool val) { m_noIdrConstraintFlag = val; }
  bool      getNoCraConstraintFlag() const { return m_noCraConstraintFlag; }
  void      setNoCraConstraintFlag(bool val) { m_noCraConstraintFlag = val; }
  bool      getNoGdrConstraintFlag() const { return m_noGdrConstraintFlag; }
  void      setNoGdrConstraintFlag(bool val) { m_noGdrConstraintFlag = val; }
  bool      getNoApsConstraintFlag() const { return m_noApsConstraintFlag; }
  void      setNoApsConstraintFlag(bool val) { m_noApsConstraintFlag = val; }


  void      setFrameRate                    ( int   i )      { m_iFrameRate = i; }
  void      setFrameSkip                    ( uint32_t  i )      { m_FrameSkip = i; }
  void      setTemporalSubsampleRatio       ( uint32_t  i )      { m_temporalSubsampleRatio = i; }
  void      setSourceWidth                  ( int   i )      { m_iSourceWidth = i; }
  void      setSourceHeight                 ( int   i )      { m_iSourceHeight = i; }

  Window   &getConformanceWindow()                           { return m_conformanceWindow; }
  void      setConformanceWindow (int confLeft, int confRight, int confTop, int confBottom ) { m_conformanceWindow.setWindow (confLeft, confRight, confTop, confBottom); }

  void      setFramesToBeEncoded            ( int   i )      { m_framesToBeEncoded = i; }

  bool      getPrintMSEBasedSequencePSNR    ()         const { return m_printMSEBasedSequencePSNR;  }
  void      setPrintMSEBasedSequencePSNR    (bool value)     { m_printMSEBasedSequencePSNR = value; }

  bool      getPrintHexPsnr                 ()         const { return m_printHexPsnr;               }
  void      setPrintHexPsnr                 (bool value)     { m_printHexPsnr = value;              }

  bool      getPrintFrameMSE                ()         const { return m_printFrameMSE;              }
  void      setPrintFrameMSE                (bool value)     { m_printFrameMSE = value;             }

  bool      getPrintSequenceMSE             ()         const { return m_printSequenceMSE;           }
  void      setPrintSequenceMSE             (bool value)     { m_printSequenceMSE = value;          }

  bool      getPrintMSSSIM                  ()         const { return m_printMSSSIM;               }
  void      setPrintMSSSIM                  (bool value)     { m_printMSSSIM = value;              }

  bool      getCabacZeroWordPaddingEnabled()           const { return m_cabacZeroWordPaddingEnabled;  }
  void      setCabacZeroWordPaddingEnabled(bool value)       { m_cabacZeroWordPaddingEnabled = value; }

  //====== Coding Structure ========
  void      setIntraPeriod                  (int   i)        { m_intraPeriod = i;                   }
  void      setDecodingRefreshType          ( int   i )      { m_decodingRefreshType = (uint32_t)i; }
  void      setReWriteParamSets             ( bool  b )      { m_rewriteParamSets = b; }
  void      setIDRRefParamListPresent       ( bool  b )      { m_idrRefParamList  = b; }
  bool      getIDRRefParamListPresent       ()        const  { return m_idrRefParamList; }
  void      setGOPSize                      ( int   i )      { m_iGOPSize = i; }
  void      setGopList(const GOPEntry GOPList[MAX_GOP]) { for (int i = 0; i < MAX_GOP; i++) m_GOPList[i] = GOPList[i]; }
  const GOPEntry &getGOPEntry               ( int   i ) const { return m_GOPList[i]; }
  void      setRPLList0(const RPLEntry RPLList[MAX_GOP])
  {
    m_numRPLList0 = 0;
    for (int i = 0; i < MAX_GOP; i++)
    {
      m_RPLList0[i] = RPLList[i];
      if (m_RPLList0[i].m_POC != -1) m_numRPLList0++;
    }
  }
  void      setRPLList1(const RPLEntry RPLList[MAX_GOP])
  {
    m_numRPLList1 = 0;
    for (int i = 0; i < MAX_GOP; i++)
    {
      m_RPLList1[i] = RPLList[i];
      if (m_RPLList1[i].m_POC != -1) m_numRPLList1++;
    }
  }
  const RPLEntry &getRPLEntry(int L01, int idx) const { return (L01 == 0) ? m_RPLList0[idx] : m_RPLList1[idx]; }
  int       getRPLCandidateSize(int L01) const { return  (L01 == 0) ? m_numRPLList0 : m_numRPLList1; }
  void      setEncodedFlag(uint32_t  i, bool value) { m_RPLList0[i].m_isEncoded = value; m_RPLList1[i].m_isEncoded = value; m_GOPList[i].m_isEncoded = value; }
  void      setMaxDecPicBuffering           ( uint32_t u, uint32_t tlayer ) { m_maxDecPicBuffering[tlayer] = u;    }
  void      setMaxNumReorderPics            ( int  i, uint32_t tlayer ) { m_maxNumReorderPics[tlayer] = i;    }
  void      setDrapPeriod                   (int drapPeriod) { m_drapPeriod = drapPeriod; }

  void      setBaseQP                       ( int   i )      { m_iQP = i; }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  void      setIntraQPOffset                ( int   i )         { m_intraQPOffset = i; }
  void      setLambdaFromQPEnable           ( bool  b )         { m_lambdaFromQPEnable = b; }
#endif
  void      setChromaQpMappingTableParams   (const ChromaQpMappingTableParams &params) { m_chromaQpMappingTableParams = params; }

  void      setPad                          ( int*  iPad                   )      { for ( int i = 0; i < 2; i++ ) m_aiPad[i] = iPad[i]; }

  int       getMaxRefPicNum                 ()                              { return m_iMaxRefPicNum;           }
  void      setMaxRefPicNum                 ( int iMaxRefPicNum )           { m_iMaxRefPicNum = iMaxRefPicNum;  }

  int       getMaxTempLayer                 ()                              { return m_maxTempLayer;              }
  void      setMaxTempLayer                 ( int maxTempLayer )            { m_maxTempLayer = maxTempLayer;      }

  void      setCTUSize                      ( unsigned  u )      { m_CTUSize  = u; }
  void      setMinQTSizes                   ( unsigned* minQT)   { m_uiMinQT[0] = minQT[0]; m_uiMinQT[1] = minQT[1]; m_uiMinQT[2] = minQT[2]; }
  void      setMaxBTSizes                   ( unsigned* maxBT)   { m_uiMaxBT[0] = maxBT[0]; m_uiMaxBT[1] = maxBT[1]; m_uiMaxBT[2] = maxBT[2]; }
  void      setMaxTTSizes                   ( unsigned* maxTT)   { m_uiMaxTT[0] = maxTT[0]; m_uiMaxTT[1] = maxTT[1]; m_uiMaxTT[2] = maxTT[2]; }
  void      setMaxMTTHierarchyDepth         ( unsigned uiMaxMTTHierarchyDepth, unsigned uiMaxMTTHierarchyDepthI, unsigned uiMaxMTTHierarchyDepthIChroma )
                                                             { m_uiMaxMTTHierarchyDepth = uiMaxMTTHierarchyDepth; m_uiMaxMTTHierarchyDepthI = uiMaxMTTHierarchyDepthI; m_uiMaxMTTHierarchyDepthIChroma = uiMaxMTTHierarchyDepthIChroma; }
  unsigned  getMaxMTTHierarchyDepth         ()         const { return m_uiMaxMTTHierarchyDepth; }
  unsigned  getMaxMTTHierarchyDepthI        ()         const { return m_uiMaxMTTHierarchyDepthI; }
  unsigned  getMaxMTTHierarchyDepthIChroma  ()         const { return m_uiMaxMTTHierarchyDepthIChroma; }
  int       getCTUSize                      ()         const { return m_CTUSize; }
  void      setUseSplitConsOverride         (bool  n)        { m_useSplitConsOverride = n; }
  bool      getUseSplitConsOverride         ()         const { return m_useSplitConsOverride; }
  void      setDualITree                    ( bool b )       { m_dualITree = b; }
  bool      getDualITree                    ()         const { return m_dualITree; }
  void      setSubPicInfoPresentFlag                        (bool b)                    { m_subPicInfoPresentFlag = b; }
  void      setNumSubPics                               ( uint32_t u )              {
    CHECK_( u >= MAX_NUM_SUB_PICS, "Maximum number of subpictures exceeded" );
                                                                                      m_numSubPics = u;
                                                                                      m_subPicCtuTopLeftX.resize(m_numSubPics);
                                                                                      m_subPicCtuTopLeftY.resize(m_numSubPics);
                                                                                      m_subPicWidth.resize(m_numSubPics);
                                                                                      m_subPicHeight.resize(m_numSubPics);
                                                                                      m_subPicTreatedAsPicFlag.resize(m_numSubPics);
                                                                                      m_loopFilterAcrossSubpicEnabledFlag.resize(m_numSubPics);
                                                                                      m_subPicId.resize(m_numSubPics);
                                                                                    }
  void      setSubPicSameSizeFlag                       (bool b)                    { m_subPicSameSizeFlag = b; }
  void      setSubPicCtuTopLeftX                        (uint32_t u, int i)         { m_subPicCtuTopLeftX[i] = u; }
  void      setSubPicCtuTopLeftY                        (uint32_t u, int i)         { m_subPicCtuTopLeftY[i] = u; }
  void      setSubPicWidth                              (uint32_t u, int i)         { m_subPicWidth[i] = u; }
  void      setSubPicHeight                             (uint32_t u, int i)         { m_subPicHeight[i] = u; }
  void      setSubPicTreatedAsPicFlag                   (bool b, int i)             { m_subPicTreatedAsPicFlag[i] = b; }
  void      setLoopFilterAcrossSubpicEnabledFlag        (bool b, int i)             { m_loopFilterAcrossSubpicEnabledFlag[i] = b; }
  void      setSubPicCtuTopLeftX                        (const std::vector<uint32_t> &v)   {
    CHECK_(v.size() != (m_subPicSameSizeFlag ? 0 : m_numSubPics), "number of vector entries must be equal to numSubPics(subPicSameSize=0) or 0(subPicSameSize=1)"); m_subPicCtuTopLeftX = v; }
  void      setSubPicCtuTopLeftY                        (const std::vector<uint32_t> &v)   {
    CHECK_(v.size() != (m_subPicSameSizeFlag ? 0 : m_numSubPics), "number of vector entries must be equal to numSubPics(subPicSameSize=0) or 0(subPicSameSize=1)"); m_subPicCtuTopLeftY = v; }
  void      setSubPicWidth                              (const std::vector<uint32_t> &v)   {
    CHECK_(v.size() != (m_subPicSameSizeFlag ? 1 : m_numSubPics), "number of vector entries must be equal to numSubPics(subPicSameSize=0) or 1(subPicSameSize=1)"); m_subPicWidth = v; }
  void      setSubPicHeight                             (const std::vector<uint32_t> &v)   {
    CHECK_(v.size() != (m_subPicSameSizeFlag ? 1 : m_numSubPics), "number of vector entries must be equal to numSubPics(subPicSameSize=0) or 1(subPicSameSize=1)"); m_subPicHeight = v; }
  void      setSubPicTreatedAsPicFlag                   (const std::vector<bool> &v)       {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicTreatedAsPicFlag = v; }
  void      setLoopFilterAcrossSubpicEnabledFlag        (const std::vector<bool> &v)       {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_loopFilterAcrossSubpicEnabledFlag = v; }

  void      setSubPicIdMappingExplicitlySignalledFlag   (bool b)                    { m_subPicIdMappingExplicitlySignalledFlag = b; }
  void      setSubPicIdMappingInSpsFlag                 (bool b)                    { m_subPicIdMappingInSpsFlag = b; }
  void      setSubPicIdLen                              (uint32_t u)                { m_subPicIdLen = u; }
  void      setSubPicId                                 (uint32_t b, int i)         { m_subPicId[i] = b; }
  void      setSubPicId                                 (const std::vector<uint16_t> &v)   {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics"); m_subPicId = v; }

  bool      getSubPicInfoPresentFlag                    ()                          { return m_subPicInfoPresentFlag; }
  bool      getSubPicSameSizeFlag                       ()                          { return m_subPicSameSizeFlag; }
  uint32_t  getNumSubPics                               ()                          { return m_numSubPics; }
  uint32_t  getSubPicCtuTopLeftX                        (int i)                     { return m_subPicCtuTopLeftX[i]; }
  uint32_t  getSubPicCtuTopLeftY                        (int i)                     { return m_subPicCtuTopLeftY[i]; }
  uint32_t  getSubPicWidth                              (int i)                     { return m_subPicWidth[i]; }
  uint32_t  getSubPicHeight                             (int i)                     { return m_subPicHeight[i]; }
  bool      getSubPicTreatedAsPicFlag                   (int i)                     { return m_subPicTreatedAsPicFlag[i]; }
  uint32_t  getLoopFilterAcrossSubpicEnabledFlag        (int i)                     { return m_loopFilterAcrossSubpicEnabledFlag[i]; }
  bool      getSubPicIdMappingExplicitlySignalledFlag   ()                          { return m_subPicIdMappingExplicitlySignalledFlag; }
  bool      getSubPicIdMappingInSpsFlag                 ()                          { return m_subPicIdMappingInSpsFlag; }
  uint32_t  getSubPicIdLen                              ()                          { return m_subPicIdLen; }
  uint32_t  getSubPicId                                 (int i)                     { return m_subPicId[i]; }
  void      setLFNST                        ( bool b )       { m_LFNST = b; }
  bool      getLFNST()                                 const { return m_LFNST; }
  void      setUseFastLFNST                 ( bool b )       { m_useFastLFNST = b; }
  bool      getUseFastLFNST()                          const { return m_useFastLFNST; }

  void      setUseLMChroma                  ( int n )        { m_LMChroma = n; }
  int       getUseLMChroma()                           const { return m_LMChroma; }
  void      setHorCollocatedChromaFlag( bool b )             { m_horCollocatedChromaFlag = b; }
  bool      getHorCollocatedChromaFlag()               const { return m_horCollocatedChromaFlag; }
  void      setVerCollocatedChromaFlag( bool b )             { m_verCollocatedChromaFlag = b; }
  bool      getVerCollocatedChromaFlag()               const { return m_verCollocatedChromaFlag; }

  void setSbTmvpEnabledFlag(bool val) { m_sbTmvpEnableFlag = val; }

  void      setAffine                       ( bool b )       { m_Affine = b; }
  bool      getAffine                       ()         const { return m_Affine; }
  void      setAffineType( bool b )                          { m_AffineType = b; }
  bool      getAffineType()                            const { return m_AffineType; }
  void      setPROF                         (bool b)         { m_PROF = b; }
  bool      getPROF                         ()         const { return m_PROF; }
  void      setBIO(bool b)                                   { m_BIO = b; }
  bool      getBIO()                                   const { return m_BIO; }

  void      setMTSIntraMaxCand              ( unsigned u )   { m_MTSIntraMaxCand = u; }
  unsigned  getMTSIntraMaxCand              ()         const { return m_MTSIntraMaxCand; }
  void      setMTSInterMaxCand              ( unsigned u )   { m_MTSInterMaxCand = u; }
  unsigned  getMTSInterMaxCand              ()         const { return m_MTSInterMaxCand; }
  void      setIntraMTS                     ( bool b )       { m_IntraMTS = b; }
  bool      getIntraMTS                     ()         const { return m_IntraMTS; }
  void      setInterMTS                     ( bool b )       { m_InterMTS = b; }
  bool      getInterMTS                     ()         const { return m_InterMTS; }
  void      setImplicitMTS                  ( bool b )       { m_ImplicitMTS = b; }
  bool      getImplicitMTS                  ()         const { return m_ImplicitMTS; }
  void      setUseSBT                       ( bool b )       { m_SBT = b; }
  bool      getUseSBT                       ()         const { return m_SBT; }

  void      setSBTFast64WidthTh             ( int  b )       { m_SBTFast64WidthTh = b; }
  int       getSBTFast64WidthTh             ()         const { return m_SBTFast64WidthTh; }

  void      setUseCompositeRef              (bool b)         { m_compositeRefEnabled = b; }
  bool      getUseCompositeRef              ()         const { return m_compositeRefEnabled; }
  void      setUseSMVD                      ( bool b )       { m_SMVD = b; }
  bool      getUseSMVD                      ()         const { return m_SMVD; }
  void      setUseBcw                       ( bool b )       { m_bcw = b; }
  bool      getUseBcw                       ()         const { return m_bcw; }
  void      setUseBcwFast                   ( uint32_t b )   { m_BcwFast = b; }
  bool      getUseBcwFast                   ()         const { return m_BcwFast; }

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  void      setUseLadf                      ( bool b )       { m_LadfEnabled = b; }
  bool      getUseLadf                      ()         const { return m_LadfEnabled; }
  void      setLadfNumIntervals             ( int i )        { m_LadfNumIntervals = i; }
  int       getLadfNumIntervals             ()         const { return m_LadfNumIntervals; }
  void      setLadfQpOffset                 ( int value, int idx ){ m_LadfQpOffset[ idx ] = value; }
  int       getLadfQpOffset                 ( int idx ) const { return m_LadfQpOffset[ idx ]; }
  void      setLadfIntervalLowerBound       ( int value, int idx ){ m_LadfIntervalLowerBound[ idx ] = value; }
  int       getLadfIntervalLowerBound       ( int idx ) const { return m_LadfIntervalLowerBound[ idx ]; }

#endif

  void      setUseCiip                   ( bool b )       { m_ciip = b; }
  bool      getUseCiip                   ()         const { return m_ciip; }
  void      setUseGeo                       ( bool b )       { m_Geo = b; }
  bool      getUseGeo                       ()         const { return m_Geo; }
  void      setAllowDisFracMMVD             ( bool b )       { m_allowDisFracMMVD = b;    }
  bool      getAllowDisFracMMVD             ()         const { return m_allowDisFracMMVD; }
  void      setUseHashME                    ( bool b )       { m_HashME = b; }
  bool      getUseHashME                    ()         const { return m_HashME; }
  void      setUseAffineAmvr                ( bool b )       { m_AffineAmvr = b;    }
  bool      getUseAffineAmvr                ()         const { return m_AffineAmvr; }
  void      setUseAffineAmvrEncOpt          ( bool b )       { m_AffineAmvrEncOpt = b;    }
  bool      getUseAffineAmvrEncOpt          ()         const { return m_AffineAmvrEncOpt; }
  void      setDMVR                      ( bool b )       { m_DMVR = b; }
  bool      getDMVR                      ()         const { return m_DMVR; }
  void      setMMVD                         (bool b)         { m_MMVD = b;    }
  bool      getMMVD                         ()         const { return m_MMVD; }
  void      setMmvdDisNum                   ( int b )        { m_MmvdDisNum = b; }
  int       getMmvdDisNum                   ()         const { return m_MmvdDisNum; }
  void      setRGBFormatFlag(bool value) { m_rgbFormat = value; }
  bool      getRGBFormatFlag()                         const { return m_rgbFormat; }
  void      setUseColorTrans(bool value) { m_useColorTrans = value; }
  bool      getUseColorTrans()                         const { return m_useColorTrans; }
  void      setPLTMode                   ( unsigned n)    { m_PLTMode = n; }
  unsigned  getPLTMode                   ()         const { return m_PLTMode; }
  void      setJointCbCr                    ( bool b )       { m_JointCbCrMode = b; }
  bool      getJointCbCr                    ()         const { return m_JointCbCrMode; }
  void      setIBCMode                      (unsigned n)     { m_IBCMode = n; }
  unsigned  getIBCMode                      ()         const { return m_IBCMode; }
  void      setIBCLocalSearchRangeX         (unsigned n)     { m_IBCLocalSearchRangeX = n; }
  unsigned  getIBCLocalSearchRangeX         ()         const { return m_IBCLocalSearchRangeX; }
  void      setIBCLocalSearchRangeY         (unsigned n)     { m_IBCLocalSearchRangeY = n; }
  unsigned  getIBCLocalSearchRangeY         ()         const { return m_IBCLocalSearchRangeY; }
  void      setIBCHashSearch                (unsigned n)     { m_IBCHashSearch = n; }
  unsigned  getIBCHashSearch                ()         const { return m_IBCHashSearch; }
  void      setIBCHashSearchMaxCand         (unsigned n)     { m_IBCHashSearchMaxCand = n; }
  unsigned  getIBCHashSearchMaxCand         ()         const { return m_IBCHashSearchMaxCand; }
  void      setIBCHashSearchRange4SmallBlk  (unsigned n)     { m_IBCHashSearchRange4SmallBlk = n; }
  unsigned  getIBCHashSearchRange4SmallBlk  ()         const { return m_IBCHashSearchRange4SmallBlk; }
  void      setIBCFastMethod                (unsigned n)     { m_IBCFastMethod = n; }
  unsigned  getIBCFastMethod                ()         const { return m_IBCFastMethod; }

  void      setUseWrapAround                ( bool b )       { m_wrapAround = b; }
  bool      getUseWrapAround                ()         const { return m_wrapAround; }
  void      setWrapAroundOffset             ( unsigned u )   { m_wrapAroundOffset = u; }
  unsigned  getWrapAroundOffset             ()         const { return m_wrapAroundOffset; }

  // ADD_NEW_TOOL : (encoder lib) add access functions here
  void      setVirtualBoundariesEnabledFlag( bool b ) { m_virtualBoundariesEnabledFlag = b; }
  bool      getVirtualBoundariesEnabledFlag() const { return m_virtualBoundariesEnabledFlag; }
  void      setVirtualBoundariesPresentFlag( bool b ) { m_virtualBoundariesPresentFlag = b; }
  bool      getVirtualBoundariesPresentFlag() const { return m_virtualBoundariesPresentFlag; }
  void      setNumVerVirtualBoundaries      ( unsigned u )   { m_numVerVirtualBoundaries = u; }
  unsigned  getNumVerVirtualBoundaries      ()         const { return m_numVerVirtualBoundaries; }
  void      setNumHorVirtualBoundaries      ( unsigned u )   { m_numHorVirtualBoundaries = u; }
  unsigned  getNumHorVirtualBoundaries      ()         const { return m_numHorVirtualBoundaries; }
  void      setVirtualBoundariesPosX        ( unsigned u, unsigned idx ) { m_virtualBoundariesPosX[idx] = u; }
  unsigned  getVirtualBoundariesPosX        ( unsigned idx ) const { return m_virtualBoundariesPosX[idx]; }
  void      setVirtualBoundariesPosY        ( unsigned u, unsigned idx ) { m_virtualBoundariesPosY[idx] = u; }
  unsigned  getVirtualBoundariesPosY        ( unsigned idx ) const { return m_virtualBoundariesPosY[idx]; }
  void      setUseISP                       ( bool b )       { m_ISP = b; }
  bool      getUseISP                       ()         const { return m_ISP; }
  void      setLmcs                         ( bool b )                   { m_lmcsEnabled = b; }
  bool      getLmcs                         () const                     { return m_lmcsEnabled; }
  void      setReshapeSignalType            ( uint32_t signalType )      { m_reshapeSignalType = signalType; }
  uint32_t  getReshapeSignalType            () const                     { return m_reshapeSignalType; }
  void      setReshapeIntraCMD              (uint32_t intraCMD)          { m_intraCMD = intraCMD; }
  uint32_t  getReshapeIntraCMD              ()                           { return m_intraCMD; }
  void      setReshapeCW                    (const ReshapeCW &reshapeCW) { m_reshapeCW = reshapeCW; }
  const ReshapeCW& getReshapeCW             ()                           { return m_reshapeCW; }
  void      setReshapeCSoffset              (int CSoffset)          { m_CSoffset = CSoffset; }
  int       getReshapeCSoffset              ()                      { return m_CSoffset; }
  void      setMaxCUWidth                   ( uint32_t  u )      { m_maxCUWidth  = u; }
  uint32_t      getMaxCUWidth                   () const         { return m_maxCUWidth; }
  void      setMaxCUHeight                  ( uint32_t  u )      { m_maxCUHeight = u; }
  uint32_t      getMaxCUHeight                  () const         { return m_maxCUHeight; }
  void      setLog2MinCodingBlockSize       ( int n )        { m_log2MinCUSize = n;   }
  int       getLog2MinCodingBlockSize       () const         { return m_log2MinCUSize;}
  void      setUseEncDbOpt                  ( bool  n )          { m_encDbOpt = n; }
  bool      getUseEncDbOpt                  () const             { return m_encDbOpt; }

  void      setUseFastLCTU                  ( bool  n )      { m_useFastLCTU = n; }
  bool      getUseFastLCTU                  () const         { return m_useFastLCTU; }
  void      setUseFastMerge                 ( bool  n )      { m_useFastMrg = n; }
  bool      getUseFastMerge                 () const         { return m_useFastMrg; }
  void      setUsePbIntraFast               ( bool  n )      { m_usePbIntraFast = n; }
  bool      getUsePbIntraFast               () const         { return m_usePbIntraFast; }
  void      setUseAMaxBT                    ( bool  n )      { m_useAMaxBT = n; }
  bool      getUseAMaxBT                    () const         { return m_useAMaxBT; }

  void      setUseE0023FastEnc              ( bool b )       { m_e0023FastEnc = b; }
  bool      getUseE0023FastEnc              () const         { return m_e0023FastEnc; }
  void      setUseContentBasedFastQtbt      ( bool b )       { m_contentBasedFastQtbt = b; }
  bool      getUseContentBasedFastQtbt      () const         { return m_contentBasedFastQtbt; }
  void      setUseNonLinearAlfLuma          ( bool b )       { m_useNonLinearAlfLuma = b; }
  bool      getUseNonLinearAlfLuma          () const         { return m_useNonLinearAlfLuma; }
  void      setUseNonLinearAlfChroma        ( bool b )       { m_useNonLinearAlfChroma = b; }
  bool      getUseNonLinearAlfChroma        () const         { return m_useNonLinearAlfChroma; }
  void      setMaxNumAlfAlternativesChroma  ( uint32_t u )   { m_maxNumAlfAlternativesChroma = u; }
  uint32_t  getMaxNumAlfAlternativesChroma  () const         { return m_maxNumAlfAlternativesChroma; }
  void      setUseMRL                       ( bool b )       { m_MRL = b; }
  bool      getUseMRL                       () const         { return m_MRL; }
  void      setUseMIP                       ( bool b )       { m_MIP = b; }
  bool      getUseMIP                       () const         { return m_MIP; }
  void      setUseFastMIP                   ( bool b )       { m_useFastMIP = b; }
  bool      getUseFastMIP                   () const         { return m_useFastMIP; }
  void     setFastLocalDualTreeMode         ( int i )        { m_fastLocalDualTreeMode = i; }
  int      getFastLocalDualTreeMode         () const         { return m_fastLocalDualTreeMode; }

  void      setLog2MaxTbSize                ( uint32_t  u )   { m_log2MaxTbSize = u; }

  //====== Loop/Deblock Filter ========
  void      setLoopFilterDisable            ( bool  b )      { m_bLoopFilterDisable       = b; }
  void      setLoopFilterOffsetInPPS        ( bool  b )      { m_loopFilterOffsetInPPS      = b; }
  void      setLoopFilterBetaOffset         ( int   i )      { m_loopFilterBetaOffsetDiv2  = i; }
  void      setLoopFilterTcOffset           ( int   i )      { m_loopFilterTcOffsetDiv2    = i; }
  void      setLoopFilterCbBetaOffset       ( int   i )      { m_loopFilterCbBetaOffsetDiv2  = i; }
  void      setLoopFilterCbTcOffset         ( int   i )      { m_loopFilterCbTcOffsetDiv2    = i; }
  void      setLoopFilterCrBetaOffset       ( int   i )      { m_loopFilterCrBetaOffsetDiv2  = i; }
  void      setLoopFilterCrTcOffset         ( int   i )      { m_loopFilterCrTcOffsetDiv2    = i; }
#if W0038_DB_OPT
  void      setDeblockingFilterMetric       ( int   i )      { m_deblockingFilterMetric = i; }
#else
  void      setDeblockingFilterMetric       ( bool  b )      { m_DeblockingFilterMetric = b; }
#endif
  //====== Motion search ========
  void      setDisableIntraPUsInInterSlices ( bool  b )      { m_bDisableIntraPUsInInterSlices = b; }
  void      setMotionEstimationSearchMethod ( MESearchMethod e ) { m_motionEstimationSearchMethod = e; }
  void      setSearchRange                  ( int   i )      { m_iSearchRange = i; }
  void      setBipredSearchRange            ( int   i )      { m_bipredSearchRange = i; }
  void      setClipForBiPredMeEnabled       ( bool  b )      { m_bClipForBiPredMeEnabled = b; }
  void      setFastMEAssumingSmootherMVEnabled ( bool b )    { m_bFastMEAssumingSmootherMVEnabled = b; }
  void      setMinSearchWindow              ( int   i )      { m_minSearchWindow = i; }
  void      setRestrictMESampling           ( bool  b )      { m_bRestrictMESampling = b; }

  //====== Quality control ========
  void      setMaxDeltaQP                   ( int   i )      { m_iMaxDeltaQP = i; }
  void      setCuQpDeltaSubdiv              ( int   i )      { m_cuQpDeltaSubdiv = i; }
  int       getCuChromaQpOffsetSubdiv       ()         const { return m_cuChromaQpOffsetSubdiv;  }
  void      setCuChromaQpOffsetSubdiv       (int value)      { m_cuChromaQpOffsetSubdiv = value; }

  void      setChromaCbQpOffset             ( int   i )      { m_chromaCbQpOffset = i; }
  void      setChromaCrQpOffset             ( int   i )      { m_chromaCrQpOffset = i; }
  void      setChromaCbQpOffsetDualTree     ( int   i )      { m_chromaCbQpOffsetDualTree = i; }
  void      setChromaCrQpOffsetDualTree     ( int   i )      { m_chromaCrQpOffsetDualTree = i; }
  int       getChromaCbQpOffsetDualTree     ()         const { return m_chromaCbQpOffsetDualTree; }
  int       getChromaCrQpOffsetDualTree     ()         const { return m_chromaCrQpOffsetDualTree; }
  void      setChromaCbCrQpOffset           ( int   i )      { m_chromaCbCrQpOffset = i; }
  void      setChromaCbCrQpOffsetDualTree   ( int   i )      { m_chromaCbCrQpOffsetDualTree = i; }
  int       getChromaCbCrQpOffsetDualTree   ()         const { return m_chromaCbCrQpOffsetDualTree; }
#if ER_CHROMA_QP_WCG_PPS
  void      setWCGChromaQpControl           ( const WCGChromaQPControl &ctrl )     { m_wcgChromaQpControl = ctrl; }
  const WCGChromaQPControl &getWCGChromaQPControl () const { return m_wcgChromaQpControl; }
#endif
#if W0038_CQP_ADJ
  void      setSliceChromaOffsetQpIntraOrPeriodic( uint32_t periodicity, int sliceChromaQpOffsetIntraOrPeriodic[2]) { m_sliceChromaQpOffsetPeriodicity = periodicity; memcpy(m_sliceChromaQpOffsetIntraOrPeriodic, sliceChromaQpOffsetIntraOrPeriodic, sizeof(m_sliceChromaQpOffsetIntraOrPeriodic)); }
  int       getSliceChromaOffsetQpIntraOrPeriodic( bool bIsCr) const                                            { return m_sliceChromaQpOffsetIntraOrPeriodic[bIsCr?1:0]; }
  uint32_t      getSliceChromaOffsetQpPeriodicity() const                                                           { return m_sliceChromaQpOffsetPeriodicity; }
#endif

  void      setChromaFormatIdc              ( ChromaFormat cf ) { m_chromaFormatIDC = cf; }
#if REUSE_CU_RESULTS
  ChromaFormat  getChromaFormatIdc          ( ) const        { return m_chromaFormatIDC; }
#else
  ChromaFormat  getChromaFormatIdc          ( )              { return m_chromaFormatIDC; }
#endif

#if SHARP_LUMA_DELTA_QP
  void      setLumaLevelToDeltaQPControls( const LumaLevelToDeltaQPMapping &lumaLevelToDeltaQPMapping ) { m_lumaLevelToDeltaQPMapping=lumaLevelToDeltaQPMapping; }
  const LumaLevelToDeltaQPMapping& getLumaLevelToDeltaQPMapping() const { return m_lumaLevelToDeltaQPMapping; }
#endif

  bool      getExtendedPrecisionProcessingFlag         ()         const { return m_extendedPrecisionProcessingFlag;  }
  void      setExtendedPrecisionProcessingFlag         (bool value)     { m_extendedPrecisionProcessingFlag = value; }

  bool      getHighPrecisionOffsetsEnabledFlag() const { return m_highPrecisionOffsetsEnabledFlag; }
  void      setHighPrecisionOffsetsEnabledFlag(bool value) { m_highPrecisionOffsetsEnabledFlag = value; }

  void      setUseAdaptiveQP                ( bool  b )      { m_bUseAdaptiveQP = b; }
  void      setQPAdaptationRange            ( int   i )      { m_iQPAdaptationRange = i; }
#if ENABLE_QPA
  void      setUsePerceptQPA                ( const bool b ) { m_bUsePerceptQPA = b; }
  void      setUseWPSNR                     ( const bool b ) { m_bUseWPSNR = b; }
#endif

  //====== Sequence ========
  int       getFrameRate                    () const     { return  m_iFrameRate; }
  uint32_t      getFrameSkip                    () const     { return  m_FrameSkip; }
  uint32_t      getTemporalSubsampleRatio       () const     { return  m_temporalSubsampleRatio; }
  int       getSourceWidth                  () const     { return  m_iSourceWidth; }
  int       getSourceHeight                 () const     { return  m_iSourceHeight; }
  int       getFramesToBeEncoded            () const     { return  m_framesToBeEncoded; }

  //====== Lambda Modifiers ========
  void      setLambdaModifier               ( uint32_t uiIndex, double dValue ) { m_adLambdaModifier[ uiIndex ] = dValue; }
  double    getLambdaModifier               ( uint32_t uiIndex )          const { return m_adLambdaModifier[ uiIndex ]; }
  void      setIntraLambdaModifier          ( const std::vector<double> &dValue )               { m_adIntraLambdaModifier = dValue;       }
  const std::vector<double>& getIntraLambdaModifier()                        const { return m_adIntraLambdaModifier;         }
  void      setIntraQpFactor                ( double dValue )               { m_dIntraQpFactor = dValue;              }
  double    getIntraQpFactor                ()                        const { return m_dIntraQpFactor;                }

  //==== Coding Structure ========
  int       getIntraPeriod                  () const     { return  m_intraPeriod; }
  uint32_t  getDecodingRefreshType          () const     { return  m_decodingRefreshType; }
  bool      getReWriteParamSets             ()  const    { return m_rewriteParamSets; }
  int       getGOPSize                      () const     { return  m_iGOPSize; }
  int       getMaxDecPicBuffering           (uint32_t tlayer) { return m_maxDecPicBuffering[tlayer]; }
  int       getMaxNumReorderPics            (uint32_t tlayer) { return m_maxNumReorderPics[tlayer]; }
  int       getDrapPeriod                   ()     { return m_drapPeriod; }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  int       getIntraQPOffset                () const    { return  m_intraQPOffset; }
  int       getLambdaFromQPEnable           () const    { return  m_lambdaFromQPEnable; }
public:
  int       getBaseQP                       () const { return  m_iQP; } // public should use getQPForPicture.
  int       getQPForPicture                 (const uint32_t gopIndex, const Slice *pSlice) const; // Function actually defined in EncLib.cpp
#else
  int       getBaseQP                       ()       { return  m_iQP; }
#endif
  int       getPad                          ( int i )      {
    CHECK_(i >= 2, "Invalid index");                      return  m_aiPad[i]; }

  bool      getAccessUnitDelimiter() const  { return m_AccessUnitDelimiter; }
  void      setAccessUnitDelimiter(bool val){ m_AccessUnitDelimiter = val; }
  bool      getEnablePictureHeaderInSliceHeader() const { return m_enablePictureHeaderInSliceHeader; }
  void      setEnablePictureHeaderInSliceHeader(bool val) { m_enablePictureHeaderInSliceHeader = val; }

  //==== Loop/Deblock Filter ========
  bool      getLoopFilterDisable            ()      { return  m_bLoopFilterDisable;       }
  bool      getLoopFilterOffsetInPPS        ()      { return m_loopFilterOffsetInPPS; }
  int       getLoopFilterBetaOffset         ()      { return m_loopFilterBetaOffsetDiv2; }
  int       getLoopFilterTcOffset           ()      { return m_loopFilterTcOffsetDiv2; }
  int       getLoopFilterCbBetaOffset       ()      { return m_loopFilterCbBetaOffsetDiv2; }
  int       getLoopFilterCbTcOffset         ()      { return m_loopFilterCbTcOffsetDiv2;   }
  int       getLoopFilterCrBetaOffset       ()      { return m_loopFilterCrBetaOffsetDiv2; }
  int       getLoopFilterCrTcOffset         ()      { return m_loopFilterCrTcOffsetDiv2;   }
#if W0038_DB_OPT
  int       getDeblockingFilterMetric       ()      { return m_deblockingFilterMetric; }
#else
  bool      getDeblockingFilterMetric       ()      { return m_DeblockingFilterMetric; }
#endif

  //==== Motion search ========
  bool      getDisableIntraPUsInInterSlices    () const { return m_bDisableIntraPUsInInterSlices; }
  MESearchMethod getMotionEstimationSearchMethod ( ) const { return m_motionEstimationSearchMethod; }
  int       getSearchRange                     () const { return m_iSearchRange; }
  bool      getClipForBiPredMeEnabled          () const { return m_bClipForBiPredMeEnabled; }
  bool      getFastMEAssumingSmootherMVEnabled () const { return m_bFastMEAssumingSmootherMVEnabled; }
  int       getMinSearchWindow                 () const { return m_minSearchWindow; }
  bool      getRestrictMESampling              () const { return m_bRestrictMESampling; }

  //==== Quality control ========
  int       getMaxDeltaQP                   () const { return m_iMaxDeltaQP; }
  int       getCuQpDeltaSubdiv              () const { return m_cuQpDeltaSubdiv; }
  bool      getUseAdaptiveQP                () const { return m_bUseAdaptiveQP; }
  int       getQPAdaptationRange            () const { return m_iQPAdaptationRange; }
#if ENABLE_QPA
  bool      getUsePerceptQPA                () const { return m_bUsePerceptQPA; }
  bool      getUseWPSNR                     () const { return m_bUseWPSNR; }
#endif

  //==== Tool list ========
  void      setBitDepth( const ChannelType chType, int internalBitDepthForChannel ) { m_bitDepth[chType] = internalBitDepthForChannel; }
  void      setInputBitDepth( const ChannelType chType, int internalBitDepthForChannel ) { m_inputBitDepth[chType] = internalBitDepthForChannel; }
  int*      getInputBitDepth()                              { return m_inputBitDepth; }
  void      setUseASR                       ( bool  b )     { m_bUseASR     = b; }
  void      setUseHADME                     ( bool  b )     { m_bUseHADME   = b; }
  void      setUseRDOQ                      ( bool  b )     { m_useRDOQ    = b; }
  void      setUseRDOQTS                    ( bool  b )     { m_useRDOQTS  = b; }
#if T0196_SELECTIVE_RDOQ
  void      setUseSelectiveRDOQ             ( bool b )      { m_useSelectiveRDOQ = b; }
#endif
  void      setRDpenalty                    ( uint32_t  u )     { m_rdPenalty  = u; }
  void      setFastInterSearchMode          ( FastInterSearchMode m ) { m_fastInterSearchMode = m; }
  void      setUseEarlyCU                   ( bool  b )     { m_bUseEarlyCU = b; }
  void      setUseFastDecisionForMerge      ( bool  b )     { m_useFastDecisionForMerge = b; }
  void      setUseEarlySkipDetection        ( bool  b )     { m_useEarlySkipDetection = b; }
  void      setFastUDIUseMPMEnabled         ( bool  b )     { m_bFastUDIUseMPMEnabled = b; }
  void      setFastMEForGenBLowDelayEnabled ( bool  b )     { m_bFastMEForGenBLowDelayEnabled = b; }
  void      setUseBLambdaForNonKeyLowDelayPictures ( bool b ) { m_bUseBLambdaForNonKeyLowDelayPictures = b; }

  void      setdQPs                         ( int*  p )     { m_aidQP       = p; }
  void      setDeltaQpRD                    ( uint32_t  u )     {m_uiDeltaQpRD  = u; }
  void      setFastDeltaQp                  ( bool  b )     {m_bFastDeltaQP = b; }
  int       getBitDepth                     (const ChannelType chType) const { return m_bitDepth[chType]; }
  int*      getBitDepth                     ()      { return m_bitDepth; }
  bool      getUseASR                       ()      { return m_bUseASR;     }
  bool      getUseHADME                     ()      { return m_bUseHADME;   }
  bool      getUseRDOQ                      ()      { return m_useRDOQ;    }
  bool      getUseRDOQTS                    ()      { return m_useRDOQTS;  }
#if T0196_SELECTIVE_RDOQ
  bool      getUseSelectiveRDOQ             ()      { return m_useSelectiveRDOQ; }
#endif
  int       getRDpenalty                    ()      { return m_rdPenalty;  }
  FastInterSearchMode getFastInterSearchMode() const{ return m_fastInterSearchMode;  }
  bool      getUseEarlyCU                   () const{ return m_bUseEarlyCU; }
  bool      getUseFastDecisionForMerge      () const{ return m_useFastDecisionForMerge; }
  bool      getUseEarlySkipDetection        () const{ return m_useEarlySkipDetection; }
  bool      getFastUDIUseMPMEnabled         ()      { return m_bFastUDIUseMPMEnabled; }
  bool      getFastMEForGenBLowDelayEnabled ()      { return m_bFastMEForGenBLowDelayEnabled; }
  bool      getUseBLambdaForNonKeyLowDelayPictures () { return m_bUseBLambdaForNonKeyLowDelayPictures; }
  void  setGopBasedTemporalFilterEnabled(bool flag) { m_gopBasedTemporalFilterEnabled = flag; }
  bool  getGopBasedTemporalFilterEnabled()          { return m_gopBasedTemporalFilterEnabled; }

  bool      getUseReconBasedCrossCPredictionEstimate ()                const { return m_reconBasedCrossCPredictionEstimate;  }
  void      setUseReconBasedCrossCPredictionEstimate (const bool value)      { m_reconBasedCrossCPredictionEstimate = value; }

  bool getUseTransformSkip                             ()      { return m_useTransformSkip;        }
  void setUseTransformSkip                             ( bool b ) { m_useTransformSkip  = b;       }
  bool getTransformSkipRotationEnabledFlag             ()            const { return m_transformSkipRotationEnabledFlag;  }
  void setTransformSkipRotationEnabledFlag             (const bool value)  { m_transformSkipRotationEnabledFlag = value; }
  bool getTransformSkipContextEnabledFlag              ()            const { return m_transformSkipContextEnabledFlag;  }
  void setTransformSkipContextEnabledFlag              (const bool value)  { m_transformSkipContextEnabledFlag = value; }
  bool getUseChromaTS                                  ()       { return m_useChromaTS; }
  void setUseChromaTS                                  (bool b) { m_useChromaTS = b; }
  bool getUseBDPCM                                     ()         { return m_useBDPCM; }
  void setUseBDPCM                                     ( bool b ) { m_useBDPCM  = b;   }
  bool getUseJointCbCr                                 ()         { return m_JointCbCrMode; }
  void setUseJointCbCr                                 (bool b)   { m_JointCbCrMode = b; }
  bool getPersistentRiceAdaptationEnabledFlag          ()                 const { return m_persistentRiceAdaptationEnabledFlag;  }
  void setPersistentRiceAdaptationEnabledFlag          (const bool value)       { m_persistentRiceAdaptationEnabledFlag = value; }
  bool getCabacBypassAlignmentEnabledFlag              ()       const      { return m_cabacBypassAlignmentEnabledFlag;  }
  void setCabacBypassAlignmentEnabledFlag              (const bool value)  { m_cabacBypassAlignmentEnabledFlag = value; }
  bool getUseTransformSkipFast                         ()      { return m_useTransformSkipFast;    }
  void setUseTransformSkipFast                         ( bool b ) { m_useTransformSkipFast  = b;   }
  uint32_t getLog2MaxTransformSkipBlockSize                () const      { return m_log2MaxTransformSkipBlockSize;     }
  void setLog2MaxTransformSkipBlockSize                ( uint32_t u )    { m_log2MaxTransformSkipBlockSize  = u;       }
  bool getIntraSmoothingDisabledFlag               ()      const { return m_intraSmoothingDisabledFlag; }
  void setIntraSmoothingDisabledFlag               (bool bValue) { m_intraSmoothingDisabledFlag=bValue; }
  bool getUseFastISP                                   () const   { return m_useFastISP;    }
  void setUseFastISP                                   ( bool b ) { m_useFastISP  = b;   }

  const int* getdQPs                        () const { return m_aidQP;       }
  uint32_t      getDeltaQpRD                    () const { return m_uiDeltaQpRD; }
  bool      getFastDeltaQp                  () const { return m_bFastDeltaQP; }
  void      setMixedLossyLossless(bool b) { m_mixedLossyLossless = b; }
  bool      getMixedLossyLossless()       { return m_mixedLossyLossless; }
  void      setSliceLosslessArray(std::vector<uint16_t> sliceLosslessArray) { m_sliceLosslessArray = sliceLosslessArray; }
  const     std::vector<uint16_t>*   getSliceLosslessArray() const { return &m_sliceLosslessArray; }
  //====== Tiles and Slices ========
  void      setNoPicPartitionFlag( bool b )                                { m_noPicPartitionFlag = b;              }
  bool      getNoPicPartitionFlag()                                        { return m_noPicPartitionFlag;           }
  void      setTileColWidths( std::vector<uint32_t> tileColWidths )        { m_tileColumnWidth = tileColWidths;     }
  const     std::vector<uint32_t>*   getTileColWidths() const              { return &m_tileColumnWidth;             }
  void      setTileRowHeights( std::vector<uint32_t> tileRowHeights )      { m_tileRowHeight = tileRowHeights;      }
  const     std::vector<uint32_t>*   getTileRowHeights() const             { return &m_tileRowHeight;               }
  void      setRectSliceFlag( bool b )                                     { m_rectSliceFlag = b;                   }
  bool      getRectSliceFlag()                                             { return m_rectSliceFlag;                }
  void      setNumSlicesInPic( uint32_t u )                                { m_numSlicesInPic = u;                  }
  uint32_t  getNumSlicesInPic()                                            { return m_numSlicesInPic;               }
  void      setTileIdxDeltaPresentFlag( bool b )                           { m_tileIdxDeltaPresentFlag = b;         }
  bool      getTileIdxDeltaPresentFlag()                                   { return m_tileIdxDeltaPresentFlag;      }
  void      setRectSlices( std::vector<RectSlice> rectSlices )             { m_rectSlices = rectSlices;             }
  const     std::vector<RectSlice>*   getRectSlices() const                { return &m_rectSlices;                  }
  void      setRasterSliceSizes( std::vector<uint32_t> rasterSliceSizes )  { m_rasterSliceSize = rasterSliceSizes;  }
  const     std::vector<uint32_t>*   getRasterSliceSizes() const           { return &m_rasterSliceSize;             }
  void      setLFCrossTileBoundaryFlag( bool b )                           { m_bLFCrossTileBoundaryFlag = b;        }
  bool      getLFCrossTileBoundaryFlag()                                   { return m_bLFCrossTileBoundaryFlag;     }
  void      setLFCrossSliceBoundaryFlag( bool b )                          { m_bLFCrossSliceBoundaryFlag = b;       }
  bool      getLFCrossSliceBoundaryFlag()                                  { return m_bLFCrossSliceBoundaryFlag;    }
  //====== Sub-picture and Slices ========
  void      setSingleSlicePerSubPicFlagFlag( bool b )                { m_singleSlicePerSubPicFlag = b;    }
  bool      getSingleSlicePerSubPicFlagFlag( )                       { return m_singleSlicePerSubPicFlag;    }
  void      setUseSAO                  (bool bVal)                   { m_bUseSAO = bVal; }
  bool      getUseSAO                  ()                            { return m_bUseSAO; }
  void  setTestSAODisableAtPictureLevel (bool bVal)                  { m_bTestSAODisableAtPictureLevel = bVal; }
  bool  getTestSAODisableAtPictureLevel ( ) const                    { return m_bTestSAODisableAtPictureLevel; }

  void   setSaoEncodingRate(double v)                                { m_saoEncodingRate = v; }
  double getSaoEncodingRate() const                                  { return m_saoEncodingRate; }
  void   setSaoEncodingRateChroma(double v)                          { m_saoEncodingRateChroma = v; }
  double getSaoEncodingRateChroma() const                            { return m_saoEncodingRateChroma; }
  void  setMaxNumOffsetsPerPic                   (int iVal)          { m_maxNumOffsetsPerPic = iVal; }
  int   getMaxNumOffsetsPerPic                   ()                  { return m_maxNumOffsetsPerPic; }
  void  setSaoCtuBoundary              (bool val)                    { m_saoCtuBoundary = val; }
  bool  getSaoCtuBoundary              ()                            { return m_saoCtuBoundary; }

  void  setSaoGreedyMergeEnc           (bool val)                    { m_saoGreedyMergeEnc = val; }
  bool  getSaoGreedyMergeEnc           ()                            { return m_saoGreedyMergeEnc; }
  void  setEntropyCodingSyncEnabledFlag(bool b)                      { m_entropyCodingSyncEnabledFlag = b; }
  bool  getEntropyCodingSyncEnabledFlag() const                      { return m_entropyCodingSyncEnabledFlag; }
  void  setEntryPointPresentFlag(bool b)                             { m_entryPointPresentFlag = b; }
  void  setDecodedPictureHashSEIType(HashType m)                     { m_decodedPictureHashSEIType = m; }
  HashType getDecodedPictureHashSEIType() const                      { return m_decodedPictureHashSEIType; }
  void  setSubpicDecodedPictureHashType(HashType m)                  { m_subpicDecodedPictureHashType = m; }
  HashType getSubpicDecodedPictureHashType() const                   { return m_subpicDecodedPictureHashType; }

  void  setBufferingPeriodSEIEnabled(bool b)                         { m_bufferingPeriodSEIEnabled = b; }
  bool  getBufferingPeriodSEIEnabled() const                         { return m_bufferingPeriodSEIEnabled; }
  void  setPictureTimingSEIEnabled(bool b)                           { m_pictureTimingSEIEnabled = b; }
  bool  getPictureTimingSEIEnabled() const                           { return m_pictureTimingSEIEnabled; }
  void  setFrameFieldInfoSEIEnabled(bool b)                           { m_frameFieldInfoSEIEnabled = b; }
  bool  getFrameFieldInfoSEIEnabled() const                           { return m_frameFieldInfoSEIEnabled; }
  void  setDependentRAPIndicationSEIEnabled(bool b)                  { m_dependentRAPIndicationSEIEnabled = b; }
  int   getDependentRAPIndicationSEIEnabled() const                  { return m_dependentRAPIndicationSEIEnabled; }
  void  setFramePackingArrangementSEIEnabled(bool b)                 { m_framePackingSEIEnabled = b; }
  bool  getFramePackingArrangementSEIEnabled() const                 { return m_framePackingSEIEnabled; }
  void  setFramePackingArrangementSEIType(int b)                     { m_framePackingSEIType = b; }
  int   getFramePackingArrangementSEIType()                          { return m_framePackingSEIType; }
  void  setFramePackingArrangementSEIId(int b)                       { m_framePackingSEIId = b; }
  int   getFramePackingArrangementSEIId()                            { return m_framePackingSEIId; }
  void  setFramePackingArrangementSEIQuincunx(int b)                 { m_framePackingSEIQuincunx = b; }
  int   getFramePackingArrangementSEIQuincunx()                      { return m_framePackingSEIQuincunx; }
  void  setFramePackingArrangementSEIInterpretation(int b)           { m_framePackingSEIInterpretation = b; }
  int   getFramePackingArrangementSEIInterpretation()                { return m_framePackingSEIInterpretation; }
  void  setParameterSetsInclusionIndicationSEIEnabled(bool b)        { m_parameterSetsInclusionIndicationSEIEnabled = b; }
  bool  getParameterSetsInclusionIndicationSEIEnabled() const        { return m_parameterSetsInclusionIndicationSEIEnabled; }
  void  setSelfContainedClvsFlag(bool b)                             { m_selfContainedClvsFlag = b; }
  int   getSelfContainedClvsFlag()                                   { return m_selfContainedClvsFlag; }
  void  setBpDeltasGOPStructure(bool b)                              { m_bpDeltasGOPStructure = b;    }
  bool  getBpDeltasGOPStructure() const                              { return m_bpDeltasGOPStructure; }
  void  setDecodingUnitInfoSEIEnabled(bool b)                        { m_decodingUnitInfoSEIEnabled = b;    }
  bool  getDecodingUnitInfoSEIEnabled() const                        { return m_decodingUnitInfoSEIEnabled; }
  void  setScalableNestingSEIEnabled(bool b)                         { m_scalableNestingSEIEnabled = b; }
  bool  getScalableNestingSEIEnabled() const                         { return m_scalableNestingSEIEnabled; }

  void  setErpSEIEnabled(bool b)                                     { m_erpSEIEnabled = b; }
  bool  getErpSEIEnabled()                                           { return m_erpSEIEnabled; }
  void  setErpSEICancelFlag(bool b)                                  { m_erpSEICancelFlag = b; }
  bool  getErpSEICancelFlag()                                        { return m_erpSEICancelFlag; }
  void  setErpSEIPersistenceFlag(bool b)                             { m_erpSEIPersistenceFlag = b; }
  bool  getErpSEIPersistenceFlag()                                   { return m_erpSEIPersistenceFlag; }
  void  setErpSEIGuardBandFlag(bool b)                               { m_erpSEIGuardBandFlag = b; }
  bool  getErpSEIGuardBandFlag()                                     { return m_erpSEIGuardBandFlag; }
  void  setErpSEIGuardBandType(uint32_t b)                           { m_erpSEIGuardBandType = b; }
  uint32_t  getErpSEIGuardBandType()                                 { return m_erpSEIGuardBandType; }
  void  setErpSEILeftGuardBandWidth(uint32_t b)                      { m_erpSEILeftGuardBandWidth = b; }
  uint32_t  getErpSEILeftGuardBandWidth()                            { return m_erpSEILeftGuardBandWidth; }
  void  setErpSEIRightGuardBandWidth(uint32_t b)                     { m_erpSEIRightGuardBandWidth = b; }
  uint32_t  getErpSEIRightGuardBandWidth()                           { return m_erpSEIRightGuardBandWidth; }
  void  setSphereRotationSEIEnabled(bool b)                          { m_sphereRotationSEIEnabled = b; }
  bool  getSphereRotationSEIEnabled()                                { return m_sphereRotationSEIEnabled; }
  void  setSphereRotationSEICancelFlag(bool b)                       { m_sphereRotationSEICancelFlag = b; }
  bool  getSphereRotationSEICancelFlag()                             { return m_sphereRotationSEICancelFlag; }
  void  setSphereRotationSEIPersistenceFlag(bool b)                  { m_sphereRotationSEIPersistenceFlag = b; }
  bool  getSphereRotationSEIPersistenceFlag()                        { return m_sphereRotationSEIPersistenceFlag; }
  void  setSphereRotationSEIYaw(int b)                               { m_sphereRotationSEIYaw = b; }
  int   getSphereRotationSEIYaw()                                    { return m_sphereRotationSEIYaw; }
  void  setSphereRotationSEIPitch(int b)                             { m_sphereRotationSEIPitch = b; }
  int   getSphereRotationSEIPitch()                                  { return m_sphereRotationSEIPitch; }
  void  setSphereRotationSEIRoll(int b)                              { m_sphereRotationSEIRoll = b; }
  int   getSphereRotationSEIRoll()                                   { return m_sphereRotationSEIRoll; }
  void  setOmniViewportSEIEnabled(bool b)                            { m_omniViewportSEIEnabled = b; }
  bool  getOmniViewportSEIEnabled()                                  { return m_omniViewportSEIEnabled; }
  void  setOmniViewportSEIId(uint32_t b)                             { m_omniViewportSEIId = b; }
  uint32_t  getOmniViewportSEIId()                                   { return m_omniViewportSEIId; }
  void  setOmniViewportSEICancelFlag(bool b)                         { m_omniViewportSEICancelFlag = b; }
  bool  getOmniViewportSEICancelFlag()                               { return m_omniViewportSEICancelFlag; }
  void  setOmniViewportSEIPersistenceFlag(bool b)                    { m_omniViewportSEIPersistenceFlag = b; }
  bool  getOmniViewportSEIPersistenceFlag()                          { return m_omniViewportSEIPersistenceFlag; }
  void  setOmniViewportSEICntMinus1(uint32_t b)                      { m_omniViewportSEICntMinus1 = b; }
  uint32_t  getOmniViewportSEICntMinus1()                            { return m_omniViewportSEICntMinus1; }
  void  setOmniViewportSEIAzimuthCentre(const std::vector<int>& vi)  { m_omniViewportSEIAzimuthCentre = vi; }
  int   getOmniViewportSEIAzimuthCentre(int idx)                     { return m_omniViewportSEIAzimuthCentre[idx]; }
  void  setOmniViewportSEIElevationCentre(const std::vector<int>& vi){ m_omniViewportSEIElevationCentre = vi; }
  int   getOmniViewportSEIElevationCentre(int idx)                   { return m_omniViewportSEIElevationCentre[idx]; }
  void  setOmniViewportSEITiltCentre(const std::vector<int>& vi)     { m_omniViewportSEITiltCentre = vi; }
  int   getOmniViewportSEITiltCentre(int idx)                        { return m_omniViewportSEITiltCentre[idx]; }
  void  setOmniViewportSEIHorRange(const std::vector<uint32_t>& vi)  { m_omniViewportSEIHorRange = vi; }
  uint32_t  getOmniViewportSEIHorRange(int idx)                      { return m_omniViewportSEIHorRange[idx]; }
  void  setOmniViewportSEIVerRange(const std::vector<uint32_t>& vi)  { m_omniViewportSEIVerRange = vi; }
  uint32_t  getOmniViewportSEIVerRange(int idx)                      { return m_omniViewportSEIVerRange[idx]; }
  void     setRwpSEIEnabled(bool b)                                                                     { m_rwpSEIEnabled = b; }
  bool     getRwpSEIEnabled()                                                                           { return m_rwpSEIEnabled; }
  void     setRwpSEIRwpCancelFlag(bool b)                                                               { m_rwpSEIRwpCancelFlag = b; }
  bool     getRwpSEIRwpCancelFlag()                                                                     { return m_rwpSEIRwpCancelFlag; }
  void     setRwpSEIRwpPersistenceFlag (bool b)                                                         { m_rwpSEIRwpPersistenceFlag = b; }
  bool     getRwpSEIRwpPersistenceFlag ()                                                               { return m_rwpSEIRwpPersistenceFlag; }
  void     setRwpSEIConstituentPictureMatchingFlag (bool b)                                             { m_rwpSEIConstituentPictureMatchingFlag = b; }
  bool     getRwpSEIConstituentPictureMatchingFlag ()                                                   { return m_rwpSEIConstituentPictureMatchingFlag; }
  void     setRwpSEINumPackedRegions (int value)                                                        { m_rwpSEINumPackedRegions = value; }
  int      getRwpSEINumPackedRegions ()                                                                 { return m_rwpSEINumPackedRegions; }
  void     setRwpSEIProjPictureWidth (int value)                                                        { m_rwpSEIProjPictureWidth = value; }
  int      getRwpSEIProjPictureWidth ()                                                                 { return m_rwpSEIProjPictureWidth; }
  void     setRwpSEIProjPictureHeight (int value)                                                       { m_rwpSEIProjPictureHeight = value; }
  int      getRwpSEIProjPictureHeight ()                                                                { return m_rwpSEIProjPictureHeight; }
  void     setRwpSEIPackedPictureWidth (int value)                                                      { m_rwpSEIPackedPictureWidth = value; }
  int      getRwpSEIPackedPictureWidth ()                                                               { return m_rwpSEIPackedPictureWidth; }
  void     setRwpSEIPackedPictureHeight (int value)                                                     { m_rwpSEIPackedPictureHeight = value; }
  int      getRwpSEIPackedPictureHeight ()                                                              { return m_rwpSEIPackedPictureHeight; }
  void     setRwpSEIRwpTransformType(const std::vector<uint8_t>& rwpTransformType)                          { m_rwpSEIRwpTransformType =rwpTransformType; }
  uint8_t  getRwpSEIRwpTransformType(uint32_t idx) const                                                    { return m_rwpSEIRwpTransformType[idx]; }
  void     setRwpSEIRwpGuardBandFlag(const std::vector<bool>& rwpGuardBandFlag)                             { m_rwpSEIRwpGuardBandFlag = rwpGuardBandFlag; }
  bool     getRwpSEIRwpGuardBandFlag(uint32_t idx) const                                                    { return m_rwpSEIRwpGuardBandFlag[idx]; }
  void     setRwpSEIProjRegionWidth(const std::vector<uint32_t>& projRegionWidth)                           { m_rwpSEIProjRegionWidth = projRegionWidth; }
  uint32_t getRwpSEIProjRegionWidth(uint32_t idx) const                                                     { return m_rwpSEIProjRegionWidth[idx]; }
  void     setRwpSEIProjRegionHeight(const std::vector<uint32_t>& projRegionHeight)                         { m_rwpSEIProjRegionHeight = projRegionHeight; }
  uint32_t getRwpSEIProjRegionHeight(uint32_t idx) const                                                    { return m_rwpSEIProjRegionHeight[idx]; }
  void     setRwpSEIRwpSEIProjRegionTop(const std::vector<uint32_t>& projRegionTop)                         { m_rwpSEIRwpSEIProjRegionTop = projRegionTop; }
  uint32_t getRwpSEIRwpSEIProjRegionTop(uint32_t idx) const                                                 { return m_rwpSEIRwpSEIProjRegionTop[idx]; }
  void     setRwpSEIProjRegionLeft(const std::vector<uint32_t>& projRegionLeft)                             { m_rwpSEIProjRegionLeft = projRegionLeft; }
  uint32_t getRwpSEIProjRegionLeft(uint32_t idx) const                                                      { return m_rwpSEIProjRegionLeft[idx]; }
  void     setRwpSEIPackedRegionWidth(const std::vector<uint16_t>& packedRegionWidth)                       { m_rwpSEIPackedRegionWidth  = packedRegionWidth; }
  uint16_t getRwpSEIPackedRegionWidth(uint32_t idx) const                                                   { return m_rwpSEIPackedRegionWidth[idx]; }
  void     setRwpSEIPackedRegionHeight(const std::vector<uint16_t>& packedRegionHeight)                     { m_rwpSEIPackedRegionHeight = packedRegionHeight; }
  uint16_t getRwpSEIPackedRegionHeight(uint32_t idx) const                                                  { return m_rwpSEIPackedRegionHeight[idx]; }
  void     setRwpSEIPackedRegionTop(const std::vector<uint16_t>& packedRegionTop)                           { m_rwpSEIPackedRegionTop = packedRegionTop; }
  uint16_t getRwpSEIPackedRegionTop(uint32_t idx) const                                                     { return m_rwpSEIPackedRegionTop[idx]; }
  void     setRwpSEIPackedRegionLeft(const std::vector<uint16_t>& packedRegionLeft)                         { m_rwpSEIPackedRegionLeft = packedRegionLeft; }
  uint16_t getRwpSEIPackedRegionLeft(uint32_t idx) const                                                    { return m_rwpSEIPackedRegionLeft[idx]; }
  void     setRwpSEIRwpLeftGuardBandWidth(const std::vector<uint8_t>& rwpLeftGuardBandWidth)                { m_rwpSEIRwpLeftGuardBandWidth = rwpLeftGuardBandWidth; }
  uint8_t  getRwpSEIRwpLeftGuardBandWidth(uint32_t idx) const                                               { return m_rwpSEIRwpLeftGuardBandWidth[idx]; }
  void     setRwpSEIRwpRightGuardBandWidth(const std::vector<uint8_t>& rwpRightGuardBandWidth)              { m_rwpSEIRwpRightGuardBandWidth = rwpRightGuardBandWidth; }
  uint8_t  getRwpSEIRwpRightGuardBandWidth(uint32_t idx) const                                              { return m_rwpSEIRwpRightGuardBandWidth[idx]; }
  void     setRwpSEIRwpTopGuardBandHeight(const std::vector<uint8_t>& rwpTopGuardBandHeight)                { m_rwpSEIRwpTopGuardBandHeight = rwpTopGuardBandHeight; }
  uint8_t  getRwpSEIRwpTopGuardBandHeight(uint32_t idx) const                                               { return m_rwpSEIRwpTopGuardBandHeight[idx]; }
  void     setRwpSEIRwpBottomGuardBandHeight(const std::vector<uint8_t>& rwpBottomGuardBandHeight)          { m_rwpSEIRwpBottomGuardBandHeight = rwpBottomGuardBandHeight; }
  uint8_t  getRwpSEIRwpBottomGuardBandHeight(uint32_t idx) const                                            { return m_rwpSEIRwpBottomGuardBandHeight[idx]; }
  void     setRwpSEIRwpGuardBandNotUsedForPredFlag(const std::vector<bool>& rwpGuardBandNotUsedForPredFlag) { m_rwpSEIRwpGuardBandNotUsedForPredFlag = rwpGuardBandNotUsedForPredFlag; }
  bool     getRwpSEIRwpGuardBandNotUsedForPredFlag(uint32_t idx) const                                      { return m_rwpSEIRwpGuardBandNotUsedForPredFlag[idx]; }
  void     setRwpSEIRwpGuardBandType(const std::vector<uint8_t>& rwpGuardBandType)                          { m_rwpSEIRwpGuardBandType = rwpGuardBandType; }
  uint8_t  getRwpSEIRwpGuardBandType(uint32_t idx) const                                                    { return m_rwpSEIRwpGuardBandType[idx]; }
  void    setGcmpSEIEnabled(bool b)                                                                 { m_gcmpSEIEnabled = b; }
  bool    getGcmpSEIEnabled()                                                                       { return m_gcmpSEIEnabled; }
  void    setGcmpSEICancelFlag(bool b)                                                              { m_gcmpSEICancelFlag = b; }
  bool    getGcmpSEICancelFlag()                                                                    { return m_gcmpSEICancelFlag; }
  void    setGcmpSEIPersistenceFlag(bool b)                                                         { m_gcmpSEIPersistenceFlag = b; }
  bool    getGcmpSEIPersistenceFlag()                                                               { return m_gcmpSEIPersistenceFlag; }
  void    setGcmpSEIPackingType(uint8_t u)                                                          { m_gcmpSEIPackingType = u; }
  uint8_t getGcmpSEIPackingType()                                                                   { return m_gcmpSEIPackingType; }
  void    setGcmpSEIMappingFunctionType(uint8_t u)                                                  { m_gcmpSEIMappingFunctionType = u; }
  uint8_t getGcmpSEIMappingFunctionType()                                                           { return m_gcmpSEIMappingFunctionType; }
  void    setGcmpSEIFaceIndex(const std::vector<uint8_t>& gcmpFaceIndex)                            { m_gcmpSEIFaceIndex = gcmpFaceIndex; }
  uint8_t getGcmpSEIFaceIndex(int idx) const                                                        { return m_gcmpSEIFaceIndex[idx]; }
  void    setGcmpSEIFaceRotation(const std::vector<uint8_t>& gcmpFaceRotation)                      { m_gcmpSEIFaceRotation = gcmpFaceRotation; }
  uint8_t getGcmpSEIFaceRotation(int idx) const                                                     { return m_gcmpSEIFaceRotation[idx]; }
  void    setGcmpSEIFunctionCoeffU(const std::vector<double>& gcmpFunctionCoeffU)                   { m_gcmpSEIFunctionCoeffU = gcmpFunctionCoeffU; }
  double  getGcmpSEIFunctionCoeffU(int idx) const                                                   { return m_gcmpSEIFunctionCoeffU[idx]; }
  void    setGcmpSEIFunctionUAffectedByVFlag(const std::vector<bool>& gcmpFunctionUAffectedByVFlag) { m_gcmpSEIFunctionUAffectedByVFlag = gcmpFunctionUAffectedByVFlag; }
  bool    getGcmpSEIFunctionUAffectedByVFlag(int idx) const                                         { return m_gcmpSEIFunctionUAffectedByVFlag[idx]; }
  void    setGcmpSEIFunctionCoeffV(const std::vector<double>& gcmpFunctionCoeffV)                   { m_gcmpSEIFunctionCoeffV = gcmpFunctionCoeffV; }
  double  getGcmpSEIFunctionCoeffV(int idx) const                                                   { return m_gcmpSEIFunctionCoeffV[idx]; }
  void    setGcmpSEIFunctionVAffectedByUFlag(const std::vector<bool>& gcmpFunctionVAffectedByUFlag) { m_gcmpSEIFunctionVAffectedByUFlag = gcmpFunctionVAffectedByUFlag; }
  bool    getGcmpSEIFunctionVAffectedByUFlag(int idx) const                                         { return m_gcmpSEIFunctionVAffectedByUFlag[idx]; }
  void    setGcmpSEIGuardBandFlag(bool b)                                                           { m_gcmpSEIGuardBandFlag = b; }
  bool    getGcmpSEIGuardBandFlag()                                                                 { return m_gcmpSEIGuardBandFlag; }
  void    setGcmpSEIGuardBandType(uint8_t u)                                                        { m_gcmpSEIGuardBandType = u; }
  uint8_t getGcmpSEIGuardBandType()                                                                 { return m_gcmpSEIGuardBandType; }
  void    setGcmpSEIGuardBandBoundaryExteriorFlag(bool b)                                           { m_gcmpSEIGuardBandBoundaryExteriorFlag = b; }
  bool    getGcmpSEIGuardBandBoundaryExteriorFlag()                                                 { return m_gcmpSEIGuardBandBoundaryExteriorFlag; }
  void    setGcmpSEIGuardBandSamplesMinus1( uint8_t u )                                             { m_gcmpSEIGuardBandSamplesMinus1 = u; }
  uint8_t getGcmpSEIGuardBandSamplesMinus1()                                                        { return m_gcmpSEIGuardBandSamplesMinus1; }
  const CfgSEISubpictureLevel& getSubpicureLevelInfoSEICfg() const                                  { return m_cfgSubpictureLevelInfoSEI; }
  void     setSubpicureLevelInfoSEICfg(const CfgSEISubpictureLevel& cfg)                            { m_cfgSubpictureLevelInfoSEI = cfg; }
  bool     getSampleAspectRatioInfoSEIEnabled() const                                                       { return m_sampleAspectRatioInfoSEIEnabled; }
  void     setSampleAspectRatioInfoSEIEnabled(const bool val)                                               { m_sampleAspectRatioInfoSEIEnabled = val; }
  bool     getSariCancelFlag() const                                                                        { return m_sariCancelFlag; }
  void     setSariCancelFlag(const bool val)                                                                { m_sariCancelFlag = val; }
  bool     getSariPersistenceFlag() const                                                                   { return m_sariPersistenceFlag; }
  void     setSariPersistenceFlag(const bool val)                                                           { m_sariPersistenceFlag = val; }
  int      getSariAspectRatioIdc() const                                                                    { return m_sariAspectRatioIdc; }
  void     setSariAspectRatioIdc(const int val)                                                             { m_sariAspectRatioIdc = val; }
  int      getSariSarWidth() const                                                                          { return m_sariSarWidth; }
  void     setSariSarWidth(const int val)                                                                   { m_sariSarWidth = val; }
  int      getSariSarHeight() const                                                                         { return m_sariSarHeight; }
  void     setSariSarHeight(const int val)                                                                  { m_sariSarHeight = val; }
  void  setMCTSEncConstraint(bool b)                                 { m_MCTSEncConstraint = b; }
  bool  getMCTSEncConstraint()                                       { return m_MCTSEncConstraint; }
  void  setMasteringDisplaySEI(const SEIMasteringDisplay &src)       { m_masteringDisplay = src; }
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void  setSEIAlternativeTransferCharacteristicsSEIEnable( bool b)   { m_alternativeTransferCharacteristicsSEIEnabled = b;    }
  bool  getSEIAlternativeTransferCharacteristicsSEIEnable( ) const   { return m_alternativeTransferCharacteristicsSEIEnabled; }
  void  setSEIPreferredTransferCharacteristics(uint8_t v)              { m_preferredTransferCharacteristics = v;    }
  uint8_t getSEIPreferredTransferCharacteristics() const               { return m_preferredTransferCharacteristics; }
#endif
  const SEIMasteringDisplay &getMasteringDisplaySEI() const          { return m_masteringDisplay; }
  // film grain SEI
  void  setFilmGrainCharactersticsSEIEnabled (bool b)                { m_fgcSEIEnabled = b; }
  bool  getFilmGrainCharactersticsSEIEnabled()                       { return m_fgcSEIEnabled; }
  void  setFilmGrainCharactersticsSEICancelFlag(bool b)              { m_fgcSEICancelFlag = b; }
  bool  getFilmGrainCharactersticsSEICancelFlag()                    { return m_fgcSEICancelFlag; }
  void  setFilmGrainCharactersticsSEIPersistenceFlag(bool b)         { m_fgcSEIPersistenceFlag = b; }
  bool  getFilmGrainCharactersticsSEIPersistenceFlag()               { return m_fgcSEIPersistenceFlag; }
  void  setFilmGrainCharactersticsSEIModelID(uint8_t v )             { m_fgcSEIModelID = v; }
  uint8_t getFilmGrainCharactersticsSEIModelID()                     { return m_fgcSEIModelID; }
  void  setFilmGrainCharactersticsSEISepColourDescPresent(bool b)    { m_fgcSEISepColourDescPresentFlag = b; }
  bool  getFilmGrainCharactersticsSEISepColourDescPresent()          { return m_fgcSEISepColourDescPresentFlag; }
  void  setFilmGrainCharactersticsSEIBlendingModeID(uint8_t v )      { m_fgcSEIBlendingModeID = v; }
  uint8_t getFilmGrainCharactersticsSEIBlendingModeID()              { return m_fgcSEIBlendingModeID; }
  void  setFilmGrainCharactersticsSEILog2ScaleFactor(uint8_t v )     { m_fgcSEILog2ScaleFactor = v; }
  uint8_t getFilmGrainCharactersticsSEILog2ScaleFactor()             { return m_fgcSEILog2ScaleFactor; }
  void  setFGCSEICompModelPresent(bool b, int index)                 { m_fgcSEICompModelPresent[index] = b; }
  bool  getFGCSEICompModelPresent(int index)                         { return m_fgcSEICompModelPresent[index]; }
  // cll SEI
  void  setCLLSEIEnabled(bool b)                                     { m_cllSEIEnabled = b; }
  bool  getCLLSEIEnabled()                                           { return m_cllSEIEnabled; }
  void  setCLLSEIMaxContentLightLevel (uint16_t v)                   { m_cllSEIMaxContentLevel = v; }
  uint16_t getCLLSEIMaxContentLightLevel()                           { return m_cllSEIMaxContentLevel; }
  void  setCLLSEIMaxPicAvgLightLevel(uint16_t v)                     { m_cllSEIMaxPicAvgLevel = v; }
  uint16_t getCLLSEIMaxPicAvgLightLevel()                            { return m_cllSEIMaxPicAvgLevel; }
  // ave SEI
  void  setAmbientViewingEnvironmentSEIEnabled (bool b)              { m_aveSEIEnabled = b; }
  bool  getAmbientViewingEnvironmentSEIEnabled ()                    { return m_aveSEIEnabled; }
  void  setAmbientViewingEnvironmentSEIIlluminance( uint32_t v )     { m_aveSEIAmbientIlluminance = v; }
  uint32_t getAmbientViewingEnvironmentSEIIlluminance()              { return m_aveSEIAmbientIlluminance; }
  void  setAmbientViewingEnvironmentSEIAmbientLightX( uint16_t v )   { m_aveSEIAmbientLightX = v; }
  uint16_t getAmbientViewingEnvironmentSEIAmbientLightX()            { return m_aveSEIAmbientLightX; }
  void  setAmbientViewingEnvironmentSEIAmbientLightY( uint16_t v )   { m_aveSEIAmbientLightY = v; }
  uint16_t getAmbientViewingEnvironmentSEIAmbientLightY()            { return m_aveSEIAmbientLightY; }
  // ccv SEI
  void     setCcvSEIEnabled(bool b)                                  { m_ccvSEIEnabled = b; }
  bool     getCcvSEIEnabled()                                        { return m_ccvSEIEnabled; }
  void     setCcvSEICancelFlag(bool b)                               { m_ccvSEICancelFlag = b; }
  bool     getCcvSEICancelFlag()                                     { return m_ccvSEICancelFlag; }
  void     setCcvSEIPersistenceFlag(bool b)                          { m_ccvSEIPersistenceFlag = b; }
  bool     getCcvSEIPersistenceFlag()                                { return m_ccvSEIPersistenceFlag; }
  void     setCcvSEIPrimariesPresentFlag(bool b)                     { m_ccvSEIPrimariesPresentFlag = b; }
  bool     getCcvSEIPrimariesPresentFlag()                           { return m_ccvSEIPrimariesPresentFlag; }
  void     setCcvSEIMinLuminanceValuePresentFlag(bool b)             { m_ccvSEIMinLuminanceValuePresentFlag = b; }
  bool     getCcvSEIMinLuminanceValuePresentFlag()                   { return m_ccvSEIMinLuminanceValuePresentFlag; }
  void     setCcvSEIMaxLuminanceValuePresentFlag(bool b)             { m_ccvSEIMaxLuminanceValuePresentFlag = b; }
  bool     getCcvSEIMaxLuminanceValuePresentFlag()                   { return m_ccvSEIMaxLuminanceValuePresentFlag; }
  void     setCcvSEIAvgLuminanceValuePresentFlag(bool b)             { m_ccvSEIAvgLuminanceValuePresentFlag = b; }
  bool     getCcvSEIAvgLuminanceValuePresentFlag()                   { return m_ccvSEIAvgLuminanceValuePresentFlag; }
  void     setCcvSEIPrimariesX(double dValue, int index)             { m_ccvSEIPrimariesX[index] = dValue; }
  double   getCcvSEIPrimariesX(int index)                            { return m_ccvSEIPrimariesX[index]; }
  void     setCcvSEIPrimariesY(double dValue, int index)             { m_ccvSEIPrimariesY[index] = dValue; }
  double   getCcvSEIPrimariesY(int index)                            { return m_ccvSEIPrimariesY[index]; }
  void     setCcvSEIMinLuminanceValue  (double dValue)               { m_ccvSEIMinLuminanceValue = dValue; }
  double   getCcvSEIMinLuminanceValue  ()                            { return m_ccvSEIMinLuminanceValue;  }
  void     setCcvSEIMaxLuminanceValue  (double dValue)               { m_ccvSEIMaxLuminanceValue = dValue; }
  double   getCcvSEIMaxLuminanceValue  ()                            { return m_ccvSEIMaxLuminanceValue;  }
  void     setCcvSEIAvgLuminanceValue  (double dValue)               { m_ccvSEIAvgLuminanceValue = dValue; }
  double   getCcvSEIAvgLuminanceValue  ()                            { return m_ccvSEIAvgLuminanceValue;  }
  void         setUseWP               ( bool b )                     { m_useWeightedPred   = b;    }
  void         setWPBiPred            ( bool b )                     { m_useWeightedBiPred = b;    }
  bool         getUseWP               ()                             { return m_useWeightedPred;   }
  bool         getWPBiPred            ()                             { return m_useWeightedBiPred; }
  void         setLog2ParallelMergeLevelMinus2(uint32_t u)           { m_log2ParallelMergeLevelMinus2 = u; }
  uint32_t     getLog2ParallelMergeLevelMinus2()                     { return m_log2ParallelMergeLevelMinus2; }
  void         setMaxNumMergeCand                ( uint32_t u )          { m_maxNumMergeCand = u;      }
  uint32_t         getMaxNumMergeCand                ()                  { return m_maxNumMergeCand;   }
  void         setMaxNumAffineMergeCand          ( uint32_t u )      { m_maxNumAffineMergeCand = u;    }
  uint32_t     getMaxNumAffineMergeCand          ()                  { return m_maxNumAffineMergeCand; }
  void         setMaxNumGeoCand                  ( uint32_t u )      { m_maxNumGeoCand = u;    }
  uint32_t     getMaxNumGeoCand                  ()                  { return m_maxNumGeoCand; }
  void         setMaxNumIBCMergeCand             ( uint32_t u )      { m_maxNumIBCMergeCand = u; }
  uint32_t     getMaxNumIBCMergeCand             ()                  { return m_maxNumIBCMergeCand; }
  void         setUseScalingListId    ( ScalingListMode u )          { m_useScalingListId       = u;   }
  ScalingListMode getUseScalingListId    ()                          { return m_useScalingListId;      }
  void         setScalingListFileName       ( const std::string &s ) { m_scalingListFileName = s;      }
  const std::string& getScalingListFileName () const                 { return m_scalingListFileName;   }
  void         setDisableScalingMatrixForAlternativeColourSpace(bool b) { m_disableScalingMatrixForAlternativeColourSpace = b; }
  bool         getDisableScalingMatrixForAlternativeColourSpace()    { return m_disableScalingMatrixForAlternativeColourSpace; }
  void         setScalingMatrixDesignatedColourSpace (bool b)        { m_scalingMatrixDesignatedColourSpace = b; }
  bool         getScalingMatrixDesignatedColourSpace ()              { return m_scalingMatrixDesignatedColourSpace; }
  void         setSliceLevelRpl  ( bool b )                          { m_sliceLevelRpl = b;     }
  bool         getSliceLevelRpl  ()                                  { return m_sliceLevelRpl;  }
  void         setSliceLevelDblk ( bool b )                          { m_sliceLevelDblk = b;    }
  bool         getSliceLevelDblk ()                                  { return m_sliceLevelDblk; }
  void         setSliceLevelSao  ( bool b )                          { m_sliceLevelSao = b;     }
  bool         getSliceLevelSao  ()                                  { return m_sliceLevelSao;  }
  void         setSliceLevelAlf  ( bool b )                          { m_sliceLevelAlf = b;     }
  bool         getSliceLevelAlf  ()                                  { return m_sliceLevelAlf;  }
  void         setSliceLevelWp(bool b)                               { m_sliceLevelWp = b;      }
  bool         getSliceLevelWp()                                     { return m_sliceLevelWp;   }
  void         setSliceLevelDeltaQp(bool b)                          { m_sliceLevelDeltaQp = b; }
  bool         getSliceLevelDeltaQp()                                { return m_sliceLevelDeltaQp; }
  void         setDisableScalingMatrixForLfnstBlks(bool u) { m_disableScalingMatrixForLfnstBlks = u; }
  bool         getDisableScalingMatrixForLfnstBlks() const          { return m_disableScalingMatrixForLfnstBlks; }
  void         setTMVPModeId ( int  u )                              { m_TMVPModeId = u;    }
  int          getTMVPModeId ()                                      { return m_TMVPModeId; }
  WeightedPredictionMethod getWeightedPredictionMethod() const       { return m_weightedPredictionMethod; }
  void         setWeightedPredictionMethod( WeightedPredictionMethod m ) { m_weightedPredictionMethod = m; }
  void         setDepQuantEnabledFlag( bool b )                      { m_DepQuantEnabledFlag = b;    }
  bool         getDepQuantEnabledFlag()                              { return m_DepQuantEnabledFlag; }
  void         setSignDataHidingEnabledFlag( bool b )                { m_SignDataHidingEnabledFlag = b;    }
  bool         getSignDataHidingEnabledFlag()                        { return m_SignDataHidingEnabledFlag; }
  bool         getUseRateCtrl         () const                       { return m_RCEnableRateControl;   }
  void         setUseRateCtrl         ( bool b )                     { m_RCEnableRateControl = b;      }
  int          getTargetBitrate       ()                             { return m_RCTargetBitrate;       }
  void         setTargetBitrate       ( int bitrate )                { m_RCTargetBitrate  = bitrate;   }
  int          getKeepHierBit         ()                             { return m_RCKeepHierarchicalBit; }
  void         setKeepHierBit         ( int i )                      { m_RCKeepHierarchicalBit = i;    }
  bool         getLCULevelRC          ()                             { return m_RCLCULevelRC; }
  void         setLCULevelRC          ( bool b )                     { m_RCLCULevelRC = b; }
  bool         getUseLCUSeparateModel ()                             { return m_RCUseLCUSeparateModel; }
  void         setUseLCUSeparateModel ( bool b )                     { m_RCUseLCUSeparateModel = b;    }
  int          getInitialQP           ()                             { return m_RCInitialQP;           }
  void         setInitialQP           ( int QP )                     { m_RCInitialQP = QP;             }
  bool         getForceIntraQP        ()                             { return m_RCForceIntraQP;        }
  void         setForceIntraQP        ( bool b )                     { m_RCForceIntraQP = b;           }
#if U0132_TARGET_BITS_SATURATION
  bool         getCpbSaturationEnabled()                             { return m_RCCpbSaturationEnabled;}
  void         setCpbSaturationEnabled( bool b )                     { m_RCCpbSaturationEnabled = b;   }
  uint32_t         getCpbSize             ()                             { return m_RCCpbSize;}
  void         setCpbSize             ( uint32_t ui )                    { m_RCCpbSize = ui;   }
  double       getInitialCpbFullness  ()                             { return m_RCInitialCpbFullness;  }
  void         setInitialCpbFullness  (double f)                     { m_RCInitialCpbFullness = f;     }
#endif
  CostMode     getCostMode( ) const                                  { return m_costMode; }
  void         setCostMode(CostMode m )                              { m_costMode = m; }
  bool         getTSRCdisableLL       ()                             { return m_TSRCdisableLL;         }
  void         setTSRCdisableLL       ( bool b )                     { m_TSRCdisableLL = b;            }

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  void         setOPI(OPI *p)                                        { m_opi = *p; }
  OPI*         getOPI()                                              { return &m_opi; }
#endif

  void         setDCI(DCI *p)                                        { m_dci = *p; }
  DCI*         getDCI()                                              { return &m_dci; }
  void         setUseRecalculateQPAccordingToLambda (bool b)         { m_recalculateQPAccordingToLambda = b;    }
  bool         getUseRecalculateQPAccordingToLambda ()               { return m_recalculateQPAccordingToLambda; }

  void         setEfficientFieldIRAPEnabled( bool b )                { m_bEfficientFieldIRAPEnabled = b; }
  bool         getEfficientFieldIRAPEnabled( ) const                 { return m_bEfficientFieldIRAPEnabled; }

  void         setHarmonizeGopFirstFieldCoupleEnabled( bool b )      { m_bHarmonizeGopFirstFieldCoupleEnabled = b; }
  bool         getHarmonizeGopFirstFieldCoupleEnabled( ) const       { return m_bHarmonizeGopFirstFieldCoupleEnabled; }

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  bool         getOPIEnabled()                      { return m_OPIEnabled; }
  void         setOPIEnabled(bool i)                { m_OPIEnabled = i; }
  void         setHtidPlus1(int HTid)               { m_opi.setHtidInfoPresentFlag(true); m_opi.setOpiHtidPlus1(HTid); }
  void         setTargetOlsIdx(int TOlsIdx)         { m_opi.setOlsInfoPresentFlag(true); m_opi.setOpiOlsIdx(TOlsIdx); }
#endif

  bool         getDCIEnabled()                      { return m_DCIEnabled; }
  void         setDCIEnabled(bool i)                { m_DCIEnabled = i; }
  bool         getHrdParametersPresentFlag()                         { return m_hrdParametersPresentFlag; }
  void         setHrdParametersPresentFlag(bool i)                   { m_hrdParametersPresentFlag = i; }
  bool         getVuiParametersPresentFlag()                         { return m_vuiParametersPresentFlag; }
  void         setVuiParametersPresentFlag(bool i)                   { m_vuiParametersPresentFlag = i; }
  bool         getSamePicTimingInAllOLS() const                      { return m_samePicTimingInAllOLS; }
  void         setSamePicTimingInAllOLS(bool b)                      { m_samePicTimingInAllOLS = b; }
  bool         getAspectRatioInfoPresentFlag()                       { return m_aspectRatioInfoPresentFlag; }
  void         setAspectRatioInfoPresentFlag(bool i)                 { m_aspectRatioInfoPresentFlag = i; }
  int          getAspectRatioIdc()                                   { return m_aspectRatioIdc; }
  void         setAspectRatioIdc(int i)                              { m_aspectRatioIdc = i; }
  int          getSarWidth()                                         { return m_sarWidth; }
  void         setSarWidth(int i)                                    { m_sarWidth = i; }
  int          getSarHeight()                                        { return m_sarHeight; }
  void         setSarHeight(int i)                                   { m_sarHeight = i; }
  bool         getColourDescriptionPresentFlag()                     { return m_colourDescriptionPresentFlag; }
  void         setColourDescriptionPresentFlag(bool i)               { m_colourDescriptionPresentFlag = i; }
  int          getColourPrimaries()                                  { return m_colourPrimaries; }
  void         setColourPrimaries(int i)                             { m_colourPrimaries = i; }
  int          getTransferCharacteristics()                          { return m_transferCharacteristics; }
  void         setTransferCharacteristics(int i)                     { m_transferCharacteristics = i; }
  int          getMatrixCoefficients()                               { return m_matrixCoefficients; }
  void         setMatrixCoefficients(int i)                          { m_matrixCoefficients = i; }
  bool         getChromaLocInfoPresentFlag()                         { return m_chromaLocInfoPresentFlag; }
  void         setChromaLocInfoPresentFlag(bool i)                   { m_chromaLocInfoPresentFlag = i; }
  int          getChromaSampleLocTypeTopField()                      { return m_chromaSampleLocTypeTopField; }
  void         setChromaSampleLocTypeTopField(int i)                 { m_chromaSampleLocTypeTopField = i; }
  int          getChromaSampleLocTypeBottomField()                   { return m_chromaSampleLocTypeBottomField; }
  void         setChromaSampleLocTypeBottomField(int i)              { m_chromaSampleLocTypeBottomField = i; }
  int          getChromaSampleLocType()                              { return m_chromaSampleLocType; }
  void         setChromaSampleLocType(int i)                         { m_chromaSampleLocType = i; }
  bool         getOverscanInfoPresentFlag()                          { return m_overscanInfoPresentFlag; }
  void         setOverscanInfoPresentFlag(bool i)                    { m_overscanInfoPresentFlag = i; }
  bool         getOverscanAppropriateFlag()                          { return m_overscanAppropriateFlag; }
  void         setOverscanAppropriateFlag(bool i)                    { m_overscanAppropriateFlag = i; }
  bool         getVideoFullRangeFlag()                               { return m_videoFullRangeFlag; }
  void         setVideoFullRangeFlag(bool i)                         { m_videoFullRangeFlag = i; }

  bool         getProgressiveSourceFlag() const                      { return m_progressiveSourceFlag; }
  void         setProgressiveSourceFlag(bool b)                      { m_progressiveSourceFlag = b; }

  bool         getInterlacedSourceFlag() const                       { return m_interlacedSourceFlag; }
  void         setInterlacedSourceFlag(bool b)                       { m_interlacedSourceFlag = b; }

  bool         getNonPackedConstraintFlag() const                    { return m_nonPackedConstraintFlag; }
  void         setNonPackedConstraintFlag(bool b)                    { m_nonPackedConstraintFlag = b; }

  bool         getNonProjectedConstraintFlag() const                 { return m_nonProjectedConstraintFlag; }
  void         setNonProjectedConstraintFlag(bool b)                 { m_nonProjectedConstraintFlag = b; }

  bool         getNoRprConstraintFlag() const                        { return m_noRprConstraintFlag; }
  void         setNoRprConstraintFlag(bool b)                        { m_noRprConstraintFlag = b; }

  bool         getNoResChangeInClvsConstraintFlag() const            { return m_noResChangeInClvsConstraintFlag; }
  void         setNoResChangeInClvsConstraintFlag(bool b)            { m_noResChangeInClvsConstraintFlag = b; }

  bool         getOneTilePerPicConstraintFlag() const                { return m_oneTilePerPicConstraintFlag; }
  void         setOneTilePerPicConstraintFlag(bool b)                { m_oneTilePerPicConstraintFlag = b; }

  bool         getPicHeaderInSliceHeaderConstraintFlag() const { return m_picHeaderInSliceHeaderConstraintFlag; }
  void         setPicHeaderInSliceHeaderConstraintFlag(bool b) { m_picHeaderInSliceHeaderConstraintFlag = b; }

  bool         getOneSlicePerPicConstraintFlag() const               { return m_oneSlicePerPicConstraintFlag; }
  void         setOneSlicePerPicConstraintFlag(bool b)               { m_oneSlicePerPicConstraintFlag = b; }

  bool         getNoIdrRplConstraintFlag() const                     { return m_noIdrRplConstraintFlag; }
  void         setNoIdrRplConstraintFlag(bool b)                     { m_noIdrRplConstraintFlag = b; }

  bool         getNoRectSliceConstraintFlag() const                  { return m_noRectSliceConstraintFlag; }
  void         setNoRectSliceConstraintFlag(bool b)                  { m_noRectSliceConstraintFlag = b; }

  bool         getOneSlicePerSubpicConstraintFlag() const            { return m_oneSlicePerSubpicConstraintFlag; }
  void         setOneSlicePerSubpicConstraintFlag(bool b)            { m_oneSlicePerSubpicConstraintFlag = b; }

  bool         getNoSubpicInfoConstraintFlag() const                 { return m_noSubpicInfoConstraintFlag; }
  void         setNoSubpicInfoConstraintFlag(bool b)                 { m_noSubpicInfoConstraintFlag = b; }

  void         setSummaryOutFilename(const std::string &s)           { m_summaryOutFilename = s; }
  const std::string& getSummaryOutFilename() const                   { return m_summaryOutFilename; }
  void         setSummaryPicFilenameBase(const std::string &s)       { m_summaryPicFilenameBase = s; }
  const std::string& getSummaryPicFilenameBase() const               { return m_summaryPicFilenameBase; }

  void         setSummaryVerboseness(uint32_t v)                         { m_summaryVerboseness = v; }
  uint32_t         getSummaryVerboseness( ) const                        { return m_summaryVerboseness; }
  void         setIMV(int n)                                         { m_ImvMode = n; }
  int          getIMV() const                                        { return m_ImvMode; }
  void         setIMV4PelFast(int n)                                 { m_Imv4PelFast = n; }
  int          getIMV4PelFast() const                                { return m_Imv4PelFast; }
  void         setDecodeBitstream( int i, const std::string& s )     { m_decodeBitstreams[i] = s; }
  const std::string& getDecodeBitstream( int i )               const { return m_decodeBitstreams[i]; }
  bool         getForceDecodeBitstream1()                      const { return m_forceDecodeBitstream1; }
  void         setForceDecodeBitstream1( bool b )                    { m_forceDecodeBitstream1 = b; }
  void         setSwitchPOC( int i )                                 { m_switchPOC = i; }
  int          getSwitchPOC()                                  const { return m_switchPOC; }
  void         setSwitchDQP( int i )                                 { m_switchDQP = i; }
  int          getSwitchDQP()                                  const { return m_switchDQP; }
  void         setFastForwardToPOC( int i )                          { m_fastForwardToPOC = i; }
  int          getFastForwardToPOC()                           const { return m_fastForwardToPOC; }
  bool         useFastForwardToPOC()                           const { return m_fastForwardToPOC >= 0; }
  void         setStopAfterFFtoPOC( bool b )                         { m_stopAfterFFtoPOC = b; }
  bool         getStopAfterFFtoPOC()                           const { return m_stopAfterFFtoPOC; }
  void         setBs2ModPOCAndType( bool b )                         { m_bs2ModPOCAndType = b; }
  bool         getBs2ModPOCAndType()                           const { return m_bs2ModPOCAndType; }
  void         setDebugCTU( int i )                                  { m_debugCTU = i; }
  int          getDebugCTU()                                   const { return m_debugCTU; }

#if ENABLE_SPLIT_PARALLELISM
  void         setNumSplitThreads( int n )                           { m_numSplitThreads = n; }
  int          getNumSplitThreads()                            const { return m_numSplitThreads; }
  void         setForceSingleSplitThread( bool b )                   { m_forceSingleSplitThread = b; }
  int          getForceSingleSplitThread()                     const { return m_forceSingleSplitThread; }
#endif
  void         setUseALF( bool b ) { m_alf = b; }
  bool         getUseALF()                                      const { return m_alf; }
#if JVET_T0064
  void         setALFStrength( double s)                        { m_alfStrength = s; }
  double       getALFStrength()                                 const { return m_alfStrength; }
  void         setALFAllowPredefinedFilters(bool b)             { m_alfAllowPredefinedFilters = b; }
  bool         getALFAllowPredefinedFilters()                   const { return m_alfAllowPredefinedFilters; }
  void         setCCALFStrength(double s)                       { m_ccalfStrength = s; }
  double       getCCALFStrength()                               const { return m_ccalfStrength; }
#endif
  void         setUseCCALF( bool b )                                  { m_ccalf = b; }
  bool         getUseCCALF()                                    const { return m_ccalf; }
  void         setCCALFQpThreshold( int b )                           { m_ccalfQpThreshold = b; }
  int          getCCALFQpThreshold()                            const { return m_ccalfQpThreshold; }
#if JVET_O0756_CALCULATE_HDRMETRICS
  void        setWhitePointDeltaE( uint32_t index, double value )     { m_whitePointDeltaE[ index ] = value; }
  double      getWhitePointDeltaE( uint32_t index )             const { return m_whitePointDeltaE[ index ]; }
  void        setMaxSampleValue(double value)                         { m_maxSampleValue = value;}
  double      getMaxSampleValue()                               const { return m_maxSampleValue;}
  void        setSampleRange(int value)                               { m_sampleRange = static_cast<hdrtoolslib::SampleRange>(value);}
  hdrtoolslib::SampleRange getSampleRange()                     const { return m_sampleRange;}
  void        setColorPrimaries(int value)                            { m_colorPrimaries = static_cast<hdrtoolslib::ColorPrimaries>(value);}
  hdrtoolslib::ColorPrimaries getColorPrimaries()               const { return m_colorPrimaries;}
  void        setEnableTFunctionLUT(bool value)                       { m_enableTFunctionLUT = value;}
  bool        getEnableTFunctionLUT()                           const { return m_enableTFunctionLUT;}
  void        setChromaLocation(uint32_t index, int value)            { m_chromaLocation[ index ] = static_cast<hdrtoolslib::ChromaLocation>(value);}
  hdrtoolslib::ChromaLocation getChromaLocation(uint32_t index) const { return m_chromaLocation[index];}
  void        setChromaUPFilter(int value)                            { m_chromaUPFilter = value;}
  int         getChromaUPFilter()                               const { return m_chromaUPFilter;}
  void        setCropOffsetLeft(int value)                            { m_cropOffsetLeft = value;}
  int         getCropOffsetLeft()                               const { return m_cropOffsetLeft;}
  void        setCropOffsetTop(int value)                             { m_cropOffsetTop = value;}
  int         getCropOffsetTop()                                const { return m_cropOffsetTop;}
  void        setCropOffsetRight(int value)                           { m_cropOffsetRight = value;}
  int         getCropOffsetRight()                              const { return m_cropOffsetRight;}
  void        setCropOffsetBottom(int value)                          { m_cropOffsetBottom = value;}
  int         getCropOffsetBottom()                             const { return m_cropOffsetBottom;}
  void        setCalculateHdrMetrics(bool value)                      { m_calculateHdrMetrics = value;}
  bool        getCalcluateHdrMetrics()                          const { return m_calculateHdrMetrics;}
#endif
  void        setRprEnabled(bool b)                                   { m_rprEnabledFlag = b; }
  bool        isRprEnabled()                                    const { return m_rprEnabledFlag; }
  void        setScalingRatio( double hor, double ver )              { m_scalingRatioHor = hor, m_scalingRatioVer = ver;  }
  void        setResChangeInClvsEnabled(bool b)                      { m_resChangeInClvsEnabled = b; }
  bool        isResChangeInClvsEnabled()                        const { return m_resChangeInClvsEnabled; }
  void        setSwitchPocPeriod( int p )                            { m_switchPocPeriod = p;}
  void        setUpscaledOutput( int b )                             { m_upscaledOutput = b; }
  int         getUpscaledOutput()                              const { return m_upscaledOutput; }

  void        setNumRefLayers( int* numRefLayers )                   { std::memcpy( m_numRefLayers, numRefLayers, sizeof( m_numRefLayers ) ); }
  int         getNumRefLayers( int layerIdx )                  const { return m_numRefLayers[layerIdx];  }

  void        setAvoidIntraInDepLayer(bool b)                        { m_avoidIntraInDepLayer = b; }
  bool        getAvoidIntraInDepLayer()                        const { return m_avoidIntraInDepLayer; }

  const CfgVPSParameters& getVPSParameters() const                                  { return m_cfgVPSParameters; }
  void                    setVPSParameters(const CfgVPSParameters& cfg)             { m_cfgVPSParameters = cfg; }
};

//! \}

#endif // !defined(AFX_TENCCFG_H__6B99B797_F4DA_4E46_8E78_7656339A6C41__INCLUDED_)

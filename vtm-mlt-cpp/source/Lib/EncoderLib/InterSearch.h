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

/** \file     InterSearch.h
    \brief    inter search class (header)
 */

#ifndef __INTERSEARCH__
#define __INTERSEARCH__

// Include files
#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/MotionInfo.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/RdCost.h"

#include "CommonLib/AffineGradientSearch.h"
#include "CommonLib/IbcHashMap.h"
#include "CommonLib/Hash.h"
#include <unordered_map>
#include <vector>
#include "EncReshape.h"
//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static const uint32_t MAX_NUM_REF_LIST_ADAPT_SR = 2;
static const uint32_t MAX_IDX_ADAPT_SR          = 33;
static const uint32_t NUM_MV_PREDICTORS         = 3;
struct BlkRecord
{
  std::unordered_map<Mv, Distortion> bvRecord;
};
class EncModeCtrl;

struct AffineMVInfo
{
  Mv  affMVs[2][33][3];
  int x, y, w, h;
};

struct BlkUniMvInfo
{
  Mv uniMvs[2][33];
  int x, y, w, h;
};

typedef struct
{
  Mv acMvAffine4Para[2][3];
  Mv acMvAffine6Para[2][3];
  int16_t affine4ParaRefIdx[2];
  int16_t affine6ParaRefIdx[2];
  Distortion hevcCost[3];
  Distortion affineCost[3];
  bool affine4ParaAvail;
  bool affine6ParaAvail;
} EncAffineMotion;

/// encoder search class
class InterSearch : public InterPrediction, AffineGradientSearch
{
private:
  EncModeCtrl     *m_modeCtrl;

  PelStorage      m_tmpPredStorage              [NUM_REF_PIC_LIST_01];
  PelStorage      m_tmpStorageLCU;
  PelStorage      m_tmpAffiStorage;
  Pel*            m_tmpAffiError;
  int*            m_tmpAffiDeri[2];

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure **m_pSaveCS;

  ClpRng          m_lumaClpRng;
  uint32_t        m_estWeightIdxBits[BCW_NUM];
  BcwMotionParam  m_uniMotions;
  bool            m_affineModeSelected;
  std::unordered_map< Position, std::unordered_map< Size, BlkRecord> > m_ctuRecord;
  AffineMVInfo       *m_affMVList;
  int             m_affMVListIdx;
  int             m_affMVListSize;
  int             m_affMVListMaxSize;
  BlkUniMvInfo*   m_uniMvList;
  int             m_uniMvListIdx;
  int             m_uniMvListSize;
  int             m_uniMvListMaxSize;
  Distortion      m_hevcCost;
  EncAffineMotion m_affineMotion;
  PatentBvCand    m_defaultCachedBvs;
protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;
  EncReshape*     m_pcReshape;

  // ME parameters
  int             m_iSearchRange;
  int             m_bipredSearchRange; // Search range for bi-prediction
  MESearchMethod  m_motionEstimationSearchMethod;
  int             m_aaiAdaptSR                  [MAX_NUM_REF_LIST_ADAPT_SR][MAX_IDX_ADAPT_SR];

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;
  DistParam       m_cDistParam;

  RefPicList      m_currRefPicList;
  int             m_currRefPicIndex;
  bool            m_skipFracME;
  int             m_numHashMVStoreds[NUM_REF_PIC_LIST_01][MAX_NUM_REF];
  Mv              m_hashMVStoreds[NUM_REF_PIC_LIST_01][MAX_NUM_REF][5];

  // Misc.
  Pel            *m_pTempPel;

  // AMVP cost computation
  uint32_t            m_auiMVPIdxCost               [AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS+1]; //th array bounds

  Mv              m_integerMv2Nx2N              [NUM_REF_PIC_LIST_01][MAX_NUM_REF];

  bool            m_isInitialized;

  Mv              m_acBVs[2 * IBC_NUM_CANDIDATES];
  unsigned int    m_numBVs;
  bool            m_useCompositeRef;
  Distortion      m_estMinDistSbt[NUMBER_SBT_MODE + 1]; // estimated minimum SSE value of the PU if using a SBT mode
  uint8_t         m_sbtRdoOrder[NUMBER_SBT_MODE];       // order of SBT mode in RDO
  bool            m_skipSbtAll;                         // to skip all SBT modes for the current PU
  uint8_t         m_histBestSbt;                        // historical best SBT mode for PU of certain SSE values
  uint8_t         m_histBestMtsIdx;                     // historical best MTS idx  for PU of certain SSE values
  bool            m_clipMvInSubPic;

public:
  InterSearch();
  virtual ~InterSearch();

  void init                         ( EncCfg*        pcEncCfg,
                                      TrQuant*       pcTrQuant,
                                      int            iSearchRange,
                                      int            bipredSearchRange,
                                      MESearchMethod motionEstimationSearchMethod,
                                      bool           useCompositeRef,
                                      const uint32_t     maxCUWidth,
                                      const uint32_t     maxCUHeight,
                                      const uint32_t     maxTotalCUDepth,
                                      RdCost*        pcRdCost,
                                      CABACWriter*   CABACEstimator,
                                      CtxCache*      ctxCache
                                     , EncReshape*   m_pcReshape
                                    );

  void destroy                      ();

  void       calcMinDistSbt         ( CodingStructure &cs, const CodingUnit& cu, const uint8_t sbtAllowed );
  uint8_t    skipSbtByRDCost        ( int width, int height, int mtDepth, uint8_t sbtIdx, uint8_t sbtPos, double bestCost, Distortion distSbtOff, double costSbtOff, bool rootCbfSbtOff );
  bool       getSkipSbtAll          ()                 { return m_skipSbtAll; }
  void       setSkipSbtAll          ( bool skipAll )   { m_skipSbtAll = skipAll; }
  uint8_t    getSbtRdoOrder         ( uint8_t idx )    { assert( m_sbtRdoOrder[idx] < NUMBER_SBT_MODE ); assert( (uint32_t)( m_estMinDistSbt[m_sbtRdoOrder[idx]] >> 2 ) < ( MAX_UINT >> 1 ) ); return m_sbtRdoOrder[idx]; }
  Distortion getEstDistSbt          ( uint8_t sbtMode) { return m_estMinDistSbt[sbtMode]; }
  void       initTuAnalyzer         ()                 { m_estMinDistSbt[NUMBER_SBT_MODE] = std::numeric_limits<uint64_t>::max(); m_skipSbtAll = false; }
  void       setHistBestTrs         ( uint8_t sbtInfo, uint8_t mtsIdx ) { m_histBestSbt = sbtInfo; m_histBestMtsIdx = mtsIdx; }
  void       initSbtRdoOrder        ( uint8_t sbtMode ) { m_sbtRdoOrder[0] = sbtMode; m_estMinDistSbt[0] = m_estMinDistSbt[sbtMode]; }

  void setTempBuffers               (CodingStructure ****pSlitCS, CodingStructure ****pFullCS, CodingStructure **pSaveCS );
  void resetCtuRecord               ()             { m_ctuRecord.clear(); }
#if ENABLE_SPLIT_PARALLELISM
  void copyState                    ( const InterSearch& other );
#endif
  void setAffineModeSelected        ( bool flag) { m_affineModeSelected = flag; }
  void resetAffineMVList() { m_affMVListIdx = 0; m_affMVListSize = 0; }
  void savePrevAffMVInfo(int idx, AffineMVInfo &tmpMVInfo, bool& isSaved)
  {
    if (m_affMVListSize > idx)
    {
      tmpMVInfo = m_affMVList[(m_affMVListIdx - 1 - idx + m_affMVListMaxSize) % m_affMVListMaxSize];
      isSaved = true;
    }
    else
      isSaved = false;
  }
  void addAffMVInfo(AffineMVInfo &tmpMVInfo)
  {
    int j = 0;
    AffineMVInfo *prevInfo = nullptr;
    for (; j < m_affMVListSize; j++)
    {
      prevInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
      if ((tmpMVInfo.x == prevInfo->x) && (tmpMVInfo.y == prevInfo->y) && (tmpMVInfo.w == prevInfo->w) && (tmpMVInfo.h == prevInfo->h))
      {
        break;
      }
    }
    if (j < m_affMVListSize)
      *prevInfo = tmpMVInfo;
    else
    {
      m_affMVList[m_affMVListIdx] = tmpMVInfo;
      m_affMVListIdx = (m_affMVListIdx + 1) % m_affMVListMaxSize;
      m_affMVListSize = std::min(m_affMVListSize + 1, m_affMVListMaxSize);
    }
  }
  void resetUniMvList() { m_uniMvListIdx = 0; m_uniMvListSize = 0; }
  void insertUniMvCands(CompArea blkArea, Mv cMvTemp[2][33])
  {
    BlkUniMvInfo* curMvInfo = m_uniMvList + m_uniMvListIdx;
    int j = 0;
    for (; j < m_uniMvListSize; j++)
    {
      BlkUniMvInfo* prevMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
      if ((blkArea.x == prevMvInfo->x) && (blkArea.y == prevMvInfo->y) && (blkArea.width == prevMvInfo->w) && (blkArea.height == prevMvInfo->h))
      {
        break;
      }
    }

    if (j < m_uniMvListSize)
    {
      curMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
    }

    ::memcpy(curMvInfo->uniMvs, cMvTemp, 2 * 33 * sizeof(Mv));
    if (j == m_uniMvListSize)  // new element
    {
      curMvInfo->x = blkArea.x;
      curMvInfo->y = blkArea.y;
      curMvInfo->w = blkArea.width;
      curMvInfo->h = blkArea.height;
      m_uniMvListSize = std::min(m_uniMvListSize + 1, m_uniMvListMaxSize);
      m_uniMvListIdx = (m_uniMvListIdx + 1) % (m_uniMvListMaxSize);
    }
  }
  void savePrevUniMvInfo(CompArea blkArea, BlkUniMvInfo &tmpUniMvInfo, bool& isUniMvInfoSaved)
  {
    int j = 0;
    BlkUniMvInfo* curUniMvInfo = nullptr;
    for (; j < m_uniMvListSize; j++)
    {
      curUniMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
      if ((blkArea.x == curUniMvInfo->x) && (blkArea.y == curUniMvInfo->y) && (blkArea.width == curUniMvInfo->w) && (blkArea.height == curUniMvInfo->h))
      {
        break;
      }
    }

    if (j < m_uniMvListSize)
    {
      isUniMvInfoSaved = true;
      tmpUniMvInfo = *curUniMvInfo;
    }
  }
  void addUniMvInfo(BlkUniMvInfo &tmpUniMVInfo)
  {
    int j = 0;
    BlkUniMvInfo* prevUniMvInfo = nullptr;
    for (; j < m_uniMvListSize; j++)
    {
      prevUniMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
      if ((tmpUniMVInfo.x == prevUniMvInfo->x) && (tmpUniMVInfo.y == prevUniMvInfo->y) && (tmpUniMVInfo.w == prevUniMvInfo->w) && (tmpUniMVInfo.h == prevUniMvInfo->h))
      {
        break;
      }
    }
    if (j < m_uniMvListSize)
    {
      *prevUniMvInfo = tmpUniMVInfo;
    }
    else
    {
      m_uniMvList[m_uniMvListIdx] = tmpUniMVInfo;
      m_uniMvListIdx = (m_uniMvListIdx + 1) % m_uniMvListMaxSize;
      m_uniMvListSize = std::min(m_uniMvListSize + 1, m_uniMvListMaxSize);
    }
  }
  void resetSavedAffineMotion();
  void storeAffineMotion( Mv acAffineMv[2][3], int16_t affineRefIdx[2], EAffineModel affineType, int bcwIdx );
  bool searchBv(PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xBv, int yBv, int ctuSize);
  void setClipMvInSubPic(bool flag) { m_clipMvInSubPic = flag; }
protected:

  /// sub-function for motion vector refinement used in fractional-pel accuracy
  Distortion  xPatternRefinement    ( const CPelBuf* pcPatternKey, Mv baseRefMv, int iFrac, Mv& rcMvFrac, bool bAllowUseOfHadamard );

   typedef struct
   {
     int left;
     int right;
     int top;
     int bottom;
   }SearchRange;

  typedef struct
  {
    SearchRange searchRange;
    const CPelBuf* pcPatternKey;
    const Pel*  piRefY;
    int         iRefStride;
    int         iBestX;
    int         iBestY;
    uint32_t        uiBestRound;
    uint32_t        uiBestDistance;
    Distortion  uiBestSad;
    uint8_t       ucPointNr;
    int         subShiftMode;
    unsigned    imvShift;
    bool        useAltHpelIf;
    bool        inCtuSearch;
    bool        zeroMV;
  } IntTZSearchStruct;

  // sub-functions for ME
  inline void xTZSearchHelp         ( IntTZSearchStruct& rcStruct, const int iSearchX, const int iSearchY, const uint8_t ucPointNr, const uint32_t uiDistance );
  inline void xTZ2PointSearch       ( IntTZSearchStruct& rcStruct );
  inline void xTZ8PointSquareSearch ( IntTZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist );
  inline void xTZ8PointDiamondSearch( IntTZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist, const bool bCheckCornersAtDist1 );

  Distortion xGetInterPredictionError( PredictionUnit& pu, PelUnitBuf& origBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X );

public:
  /// encoder estimation - inter prediction (non-skip)

  void setModeCtrl( EncModeCtrl *modeCtrl ) { m_modeCtrl = modeCtrl;}

  void predInterSearch(CodingUnit& cu, Partitioner& partitioner );

  /// set ME search range
  void setAdaptiveSearchRange       ( int iDir, int iRefIdx, int iSearchRange) {
    CHECK_(iDir >= MAX_NUM_REF_LIST_ADAPT_SR || iRefIdx>=int(MAX_IDX_ADAPT_SR), "Invalid index"); m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange; }
  bool  predIBCSearch           ( CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap);
  void  xIntraPatternSearch         ( PredictionUnit& pu, IntTZSearchStruct&  cStruct, Mv& rcMv, Distortion&  ruiCost, Mv* cMvSrchRngLT, Mv* cMvSrchRngRB, Mv* pcMvPred);
  void  xSetIntraSearchRange        ( PredictionUnit& pu, int iRoiWidth, int iRoiHeight, const int localSearchRangeX, const int localSearchRangeY, Mv& rcMvSrchRngLT, Mv& rcMvSrchRngRB);
  void  resetIbcSearch()
  {
    for (int i = 0; i < IBC_NUM_CANDIDATES; i++)
    {
      m_defaultCachedBvs.m_bvCands[i].setZero();
    }
    m_defaultCachedBvs.currCnt = 0;
  }
  void  xIBCEstimation   ( PredictionUnit& pu, PelUnitBuf& origBuf, Mv     *pcMvPred, Mv     &rcMv, Distortion &ruiCost, const int localSearchRangeX, const int localSearchRangeY);
  void  xIBCSearchMVCandUpdate  ( Distortion  uiSad, int x, int y, Distortion* uiSadBestCand, Mv* cMVCand);
  int   xIBCSearchMVChromaRefine( PredictionUnit& pu, int iRoiWidth, int iRoiHeight, int cuPelX, int cuPelY, Distortion* uiSadBestCand, Mv*     cMVCand);
  void addToSortList(std::list<BlockHash>& listBlockHash, std::list<int>& listCost, int cost, const BlockHash& blockHash);
  bool predInterHashSearch(CodingUnit& cu, Partitioner& partitioner, bool& isPerfectMatch);
  bool xHashInterEstimation(PredictionUnit& pu, RefPicList& bestRefPicList, int& bestRefIndex, Mv& bestMv, Mv& bestMvd, int& bestMVPIndex, bool& isPerfectMatch);
  bool xRectHashInterEstimation(PredictionUnit& pu, RefPicList& bestRefPicList, int& bestRefIndex, Mv& bestMv, Mv& bestMvd, int& bestMVPIndex, bool& isPerfectMatch);
  void selectRectangleMatchesInter(const MapIterator& itBegin, int count, std::list<BlockHash>& listBlockHash, const BlockHash& currBlockHash, int width, int height, int idxNonSimple, unsigned int* &hashValues, int baseNum, int picWidth, int picHeight, bool isHorizontal, uint16_t* curHashPic);
  void selectMatchesInter(const MapIterator& itBegin, int count, std::list<BlockHash>& vecBlockHash, const BlockHash& currBlockHash);
protected:

  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------

  void xEstimateMvPredAMVP        ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdx,
                                    Mv&                   rcMvPred,
                                    AMVPInfo&             amvpInfo,
                                    bool                  bFilled = false,
                                    Distortion*           puiDistBiP = NULL
                                  );

  void xCheckBestMVP              ( RefPicList  eRefPicList,
                                    Mv          cMv,
                                    Mv&         rcMvPred,
                                    int&        riMVPIdx,
                                    AMVPInfo&   amvpInfo,
                                    uint32_t&       ruiBits,
                                    Distortion& ruiCost
                                    ,
                                    const uint8_t  imv
                                  );

  Distortion xGetTemplateCost     ( const PredictionUnit& pu,
                                    PelUnitBuf&           origBuf,
                                    PelUnitBuf&           predBuf,
                                    Mv                    cMvCand,
                                    int                   iMVPIdx,
                                    int                   iMVPNum,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdx
                                  );
  uint32_t xCalcAffineMVBits      ( PredictionUnit& pu, Mv mvCand[3], Mv mvPred[3] );

  void xCopyAMVPInfo              ( AMVPInfo*   pSrc, AMVPInfo* pDst );
  uint32_t xGetMvpIdxBits             ( int iIdx, int iNum );
  void xGetBlkBits                ( bool bPSlice, int iPartIdx,  uint32_t uiLastMode, uint32_t uiBlkBit[3]);



  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------

  void xMotionEstimation          ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    RefPicList            eRefPicList,
                                    Mv&                   rcMvPred,
                                    int                   iRefIdxPred,
                                    Mv&                   rcMv,
                                    int&                  riMVPIdx,
                                    uint32_t&                 ruiBits,
                                    Distortion&           ruiCost,
                                    const AMVPInfo&       amvpInfo,
                                    bool                  bBi = false
                                  );

  void xTZSearch                  ( const PredictionUnit& pu,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdxPred,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred,
                                    const bool            bExtendedSettings,
                                    const bool            bFastSettings = false
                                  );

  void xTZSearchSelective         ( const PredictionUnit& pu,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdxPred,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  void xSetSearchRange            ( const PredictionUnit& pu,
                                    const Mv&             cMvPred,
                                    const int             iSrchRng,
                                    SearchRange&          sr
                                  , IntTZSearchStruct &  cStruct
                                  );

  void xPatternSearchFast         ( const PredictionUnit& pu,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdxPred,
                                    IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  void xPatternSearch             ( IntTZSearchStruct&    cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD
                                  );

  void xPatternSearchIntRefine    ( PredictionUnit&     pu,
                                    IntTZSearchStruct&  cStruct,
                                    Mv&                 rcMv,
                                    Mv&                 rcMvPred,
                                    int&                riMVPIdx,
                                    uint32_t&               ruiBits,
                                    Distortion&         ruiCost,
                                    const AMVPInfo&     amvpInfo,
                                    double              fWeight
                                  );

  void xPatternSearchFracDIF      ( const PredictionUnit& pu,
                                    RefPicList            eRefPicList,
                                    int                   iRefIdx,
                                    IntTZSearchStruct&    cStruct,
                                    const Mv&             rcMvInt,
                                    Mv&                   rcMvHalf,
                                    Mv&                   rcMvQter,
                                    Distortion&           ruiCost
                                  );

  void xPredAffineInterSearch     ( PredictionUnit&       pu,
                                    PelUnitBuf&           origBuf,
                                    int                   puIdx,
                                    uint32_t&                 lastMode,
                                    Distortion&           affineCost,
                                    Mv                    hevcMv[2][33]
                                  , Mv                    mvAffine4Para[2][33][3]
                                  , int                   refIdx4Para[2]
                                  , uint8_t               bcwIdx = BCW_DEFAULT
                                  , bool                  enforceBcwPred = false
                                  , uint32_t              bcwIdxBits = 0
                                  );

  void xAffineMotionEstimation    ( PredictionUnit& pu,
                                    PelUnitBuf&     origBuf,
                                    RefPicList      eRefPicList,
                                    Mv              acMvPred[3],
                                    int             iRefIdxPred,
                                    Mv              acMv[3],
                                    uint32_t&           ruiBits,
                                    Distortion&     ruiCost,
                                    int&            mvpIdx,
                                    const AffineAMVPInfo& aamvpi,
                                    bool            bBi = false
                                  );

  void xEstimateAffineAMVP        ( PredictionUnit&  pu,
                                    AffineAMVPInfo&  affineAMVPInfo,
                                    PelUnitBuf&      origBuf,
                                    RefPicList       eRefPicList,
                                    int              iRefIdx,
                                    Mv               acMvPred[3],
                                    Distortion*      puiDistBiP
                                  );

  Distortion xGetAffineTemplateCost( PredictionUnit& pu, PelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv acMvCand[3], int iMVPIdx, int iMVPNum, RefPicList eRefPicList, int iRefIdx );

  void xCopyAffineAMVPInfo        ( AffineAMVPInfo& src, AffineAMVPInfo& dst );
  void xCheckBestAffineMVP        ( PredictionUnit &pu, AffineAMVPInfo &affineAMVPInfo, RefPicList eRefPicList, Mv acMv[3], Mv acMvPred[3], int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost );

  Distortion xGetSymmetricCost( PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eCurRefPicList, const MvField& cCurMvField, MvField& cTarMvField , int bcwIdx );

  Distortion xSymmeticRefineMvSearch( PredictionUnit& pu, PelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred
    , RefPicList eRefPicList, MvField& rCurMvField, MvField& rTarMvField, Distortion uiMinCost, int searchPattern, int nSearchStepShift, uint32_t uiMaxSearchRounds , int bcwIdx );

  void xSymmetricMotionEstimation( PredictionUnit& pu, PelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred, RefPicList eRefPicList, MvField& rCurMvField, MvField& rTarMvField, Distortion& ruiCost, int bcwIdx );

  bool xReadBufferedAffineUniMv   ( PredictionUnit& pu, RefPicList eRefPicList, int32_t iRefIdx, Mv acMvPred[3], Mv acMv[3], uint32_t& ruiBits, Distortion& ruiCost
                                    , int& mvpIdx, const AffineAMVPInfo& aamvpi
  );
  double xGetMEDistortionWeight   ( uint8_t bcwIdx, RefPicList eRefPicList);
  bool xReadBufferedUniMv         ( PredictionUnit& pu, RefPicList eRefPicList, int32_t iRefIdx, Mv& pcMvPred, Mv& rcMv, uint32_t& ruiBits, Distortion& ruiCost);

  void xClipMv                    ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps );

public:
  void resetBufferedUniMotions    () { m_uniMotions.reset(); }
  uint32_t getWeightIdxBits       ( uint8_t bcwIdx ) { return m_estWeightIdxBits[bcwIdx]; }
  void initWeightIdxBits          ();
  void symmvdCheckBestMvp(
    PredictionUnit& pu,
    PelUnitBuf& origBuf,
    Mv curMv,
    RefPicList curRefList,
    AMVPInfo amvpInfo[2][33],
    int32_t bcwIdx,
    Mv cMvPredSym[2],
    int32_t mvpIdxSym[2],
    Distortion& bestCost,
    bool skip = false
    );
protected:

  void xExtDIFUpSamplingH(CPelBuf* pcPattern, bool useAltHpelIf);
  void xExtDIFUpSamplingQ         ( CPelBuf* pcPatternKey, Mv halfPelRef );
  uint32_t xDetermineBestMvp      ( PredictionUnit& pu, Mv acMvTemp[3], int& mvpIdx, const AffineAMVPInfo& aamvpi );
  // -------------------------------------------------------------------------------------------------------------------
  // compute symbol bits
  // -------------------------------------------------------------------------------------------------------------------

  void  setWpScalingDistParam     ( int iRefIdx, RefPicList eRefPicListCur, Slice *slice );
private:
  void  xxIBCHashSearch(PredictionUnit& pu, Mv* mvPred, int numMvPred, Mv &mv, int& idxMvPred, IbcHashMap& ibcHashMap);
public:

  void encodeResAndCalcRdInterCU  (CodingStructure &cs, Partitioner &partitioner, const bool &skipResidual
    , const bool luma = true, const bool chroma = true
  );
  void xEncodeInterResidualQT     (CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID);
  void xEstimateInterResidualQT   (CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist = NULL
    , const bool luma = true, const bool chroma = true
    , PelUnitBuf* orgResi = NULL
  );
  uint64_t xGetSymbolFracBitsInter  (CodingStructure &cs, Partitioner &partitioner);
  uint64_t xCalcPuMeBits            (PredictionUnit& pu);

};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__

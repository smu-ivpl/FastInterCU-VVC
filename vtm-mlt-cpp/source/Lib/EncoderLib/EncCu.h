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

/** \file     EncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __ENCCU__
#define __ENCCU__

// Include files
#include "CommonLib/CommonDef.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/IbcHashMap.h"
#include "CommonLib/LoopFilter.h"

#include "DecoderLib/DecCu.h"

#include "CABACWriter.h"
#include "IntraSearch.h"
#include "InterSearch.h"
#include "RateCtrl.h"
#include "EncModeCtrl.h"
//! \ingroup EncoderLib
//! \{

class EncLib;
class HLSWriter;
class EncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
struct GeoMergeCombo
{
  int splitDir;
  int mergeIdx0;
  int mergeIdx1;
  double cost;
  GeoMergeCombo() : splitDir(), mergeIdx0(-1), mergeIdx1(-1), cost(0.0) {};
  GeoMergeCombo(int _splitDir, int _mergeIdx0, int _mergeIdx1, double _cost) : splitDir(_splitDir), mergeIdx0(_mergeIdx0), mergeIdx1(_mergeIdx1), cost(_cost) {};
};
struct GeoMotionInfo
{
  uint8_t   m_candIdx0;
  uint8_t   m_candIdx1;

  GeoMotionInfo(uint8_t candIdx0, uint8_t candIdx1) : m_candIdx0(candIdx0), m_candIdx1(candIdx1) { }
  GeoMotionInfo() { m_candIdx0 = m_candIdx1 = 0; }
};
struct SmallerThanComboCost
{
  inline bool operator() (const GeoMergeCombo& first, const GeoMergeCombo& second)
  {
      return (first.cost < second.cost);
  }
};
class GeoComboCostList
{
public:
  GeoComboCostList() {};
  ~GeoComboCostList() {};
  std::vector<GeoMergeCombo> list;
  void sortByCost() { std::sort(list.begin(), list.end(), SmallerThanComboCost()); };
};
struct SingleGeoMergeEntry
{
  int mergeIdx;
  double cost;
  SingleGeoMergeEntry() : mergeIdx(0), cost(MAX_DOUBLE) {};
  SingleGeoMergeEntry(int _mergeIdx, double _cost) : mergeIdx(_mergeIdx), cost(_cost) {};
};
class FastGeoCostList
{
public:
  FastGeoCostList() { numGeoTemplatesInitialized = 0; };
  ~FastGeoCostList()
  {
    for (int partIdx = 0; partIdx < 2; partIdx++)
    {
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
        delete[] singleDistList[partIdx][splitDir];
      }
      delete[] singleDistList[partIdx];
      singleDistList[partIdx] = nullptr;
    }
  };
  SingleGeoMergeEntry** singleDistList[2];
  void init(int numTemplates, int maxNumGeoCand)
  {
    if (numGeoTemplatesInitialized == 0 || numGeoTemplatesInitialized < numTemplates)
    {
      for (int partIdx = 0; partIdx < 2; partIdx++)
      {
        singleDistList[partIdx] = new SingleGeoMergeEntry*[numTemplates];
        for (int splitDir = 0; splitDir < numTemplates; splitDir++)
        {
          singleDistList[partIdx][splitDir] = new SingleGeoMergeEntry[maxNumGeoCand];
        }
      }
      numGeoTemplatesInitialized = numTemplates;
    }
  }
  void insert(int geoIdx, int partIdx, int mergeIdx, double cost)
  {
    assert(geoIdx < numGeoTemplatesInitialized);
    singleDistList[partIdx][geoIdx][mergeIdx] = SingleGeoMergeEntry(mergeIdx, cost);
  }
  int numGeoTemplatesInitialized;
};

class EncCu
  : DecCu
{
private:
  bool m_bestModeUpdated;
  struct CtxPair
  {
    Ctx start;
    Ctx best;
  };

  std::vector<CtxPair>  m_CtxBuffer;
  CtxPair*              m_CurrCtx;
  CtxCache*             m_CtxCache;

#if ENABLE_SPLIT_PARALLELISM
  int                   m_dataId;
#endif

  //  Data : encoder control
  int                   m_cuChromaQpOffsetIdxPlus1; // if 0, then cu_chroma_qp_offset_flag will be 0, otherwise cu_chroma_qp_offset_flag will be 1.

  XUCache               m_unitCache;

  CodingStructure    ***m_pTempCS;
  CodingStructure    ***m_pBestCS;
  CodingStructure    ***m_pTempCS2;
  CodingStructure    ***m_pBestCS2;
  //  Access channel
  EncCfg*               m_pcEncCfg;
  IntraSearch*          m_pcIntraSearch;
  InterSearch*          m_pcInterSearch;
  TrQuant*              m_pcTrQuant;
  RdCost*               m_pcRdCost;
  EncSlice*             m_pcSliceEncoder;
  LoopFilter*           m_pcLoopFilter;

  CABACWriter*          m_CABACEstimator;
  RateCtrl*             m_pcRateCtrl;
  IbcHashMap            m_ibcHashMap;
  EncModeCtrl          *m_modeCtrl;

  PelStorage            m_acMergeBuffer[MMVD_MRG_MAX_RD_BUF_NUM];
  PelStorage            m_acRealMergeBuffer[MRG_MAX_NUM_CANDS];
  PelStorage            m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS];
  PelStorage            m_acGeoWeightedBuffer[GEO_MAX_TRY_WEIGHTED_SAD]; // to store weighted prediction pixles
  FastGeoCostList       m_GeoCostList;
  double                m_AFFBestSATDCost;
  double                m_mergeBestSATDCost;
  MotionInfo            m_SubPuMiBuf      [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];

  int                   m_ctuIbcSearchRangeX;
  int                   m_ctuIbcSearchRangeY;
#if ENABLE_SPLIT_PARALLELISM
  EncLib*               m_pcEncLib;
#endif
  int                   m_bestBcwIdx[2];
  double                m_bestBcwCost[2];
  GeoMotionInfo         m_GeoModeTest[GEO_MAX_NUM_CANDS];
#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
  void    updateLambda      ( Slice* slice, const int dQP,
 #if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                              const bool useWCGChromaControl,
 #endif
                              const bool updateRdCostLambda );
#endif
  double                m_sbtCostSave[2];
public:
  /// copy parameters from encoder class
  void  init                ( EncLib* pcEncLib, const SPS& sps PARL_PARAM( const int jId = 0 ) );

  void setDecCuReshaperInEncCU(EncReshape* pcReshape, ChromaFormat chromaFormatIDC) { initDecCuReshaper((Reshape*) pcReshape, chromaFormatIDC); }
  /// create internal buffers
  void  create              ( EncCfg* encCfg );

  /// destroy internal buffers
  void  destroy             ();

  /// CTU analysis function
  void  compressCtu         ( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] );
  /// CTU encoding function
  int   updateCtuDataISlice ( const CPelBuf buf );

  EncModeCtrl* getModeCtrl  () { return m_modeCtrl; }


  void   setMergeBestSATDCost(double cost) { m_mergeBestSATDCost = cost; }
  double getMergeBestSATDCost()            { return m_mergeBestSATDCost; }
  void   setAFFBestSATDCost(double cost)   { m_AFFBestSATDCost = cost; }
  double getAFFBestSATDCost()              { return m_AFFBestSATDCost; }
  IbcHashMap& getIbcHashMap()              { return m_ibcHashMap;        }
  EncCfg*     getEncCfg()            const { return m_pcEncCfg;          }

  EncCu();
  ~EncCu();

protected:

  void xCalDebCost            ( CodingStructure &cs, Partitioner &partitioner, bool calDist = false );
  Distortion getDistortionDb  ( CodingStructure &cs, CPelBuf org, CPelBuf reco, ComponentID compID, const CompArea& compArea, bool afterDb );

  void xCompressCU            ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, double maxCostAllowed = MAX_DOUBLE );
#if ENABLE_SPLIT_PARALLELISM
  void xCompressCUParallel    ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm );
  void copyState              ( EncCu* other, Partitioner& pm, const UnitArea& currArea, const bool isDist );
#endif

  bool
    xCheckBestMode         ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestmode );

  void xCheckModeSplit        ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool &skipInterPass );

  bool xCheckRDCostIntra(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, bool adaptiveColorTrans);

  void xCheckDQP              ( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx = false);
  void xCheckChromaQPOffset   ( CodingStructure& cs, Partitioner& partitioner);
  void xFillPCMBuffer         ( CodingUnit &cu);

  void xCheckRDCostHashInter  ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
  void xCheckRDCostAffineMerge2Nx2N
                              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
  void xCheckRDCostInter      ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
  bool xCheckRDCostInterIMV(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, double &bestIntPelCost);
  void xEncodeDontSplit       ( CodingStructure &cs, Partitioner &partitioner);

  void xCheckRDCostMerge2Nx2N ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );

  void xCheckRDCostMergeGeo2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode);

  void xEncodeInterResidual(   CodingStructure *&tempCS
                             , CodingStructure *&bestCS
                             , Partitioner &partitioner
                             , const EncTestMode& encTestMode
                             , int residualPass       = 0
                             , bool* bestHasNonResi   = NULL
                             , double* equBcwCost     = NULL
                           );
#if REUSE_CU_RESULTS
  void xReuseCachedResult     ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &Partitioner );
#endif
  bool xIsBcwSkip(const CodingUnit& cu)
  {
    if (cu.slice->getSliceType() != B_SLICE)
    {
      return true;
    }
    return((m_pcEncCfg->getBaseQP() > 32) && ((cu.slice->getTLayer() >= 4)
       || ((cu.refIdxBi[0] >= 0 && cu.refIdxBi[1] >= 0)
       && (abs(cu.slice->getPOC() - cu.slice->getRefPOC(REF_PIC_LIST_0, cu.refIdxBi[0])) == 1
       ||  abs(cu.slice->getPOC() - cu.slice->getRefPOC(REF_PIC_LIST_1, cu.refIdxBi[1])) == 1))));
  }
  void xCheckRDCostIBCMode    ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
  void xCheckRDCostIBCModeMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );

  void xCheckPLT              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
};

//! \}

#endif // __ENCMB__

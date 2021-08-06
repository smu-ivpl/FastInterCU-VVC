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

/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#ifndef __INTRASEARCH__
#define __INTRASEARCH__

// Include files

#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/IntraPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"
#include "EncReshape.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class EncModeCtrl;

enum PLTScanMode
{
  PLT_SCAN_HORTRAV = 0,
  PLT_SCAN_VERTRAV = 1,
  NUM_PLT_SCAN = 2
};
class SortingElement
{
public:
  inline bool operator<(const SortingElement &other) const
  {
    return cnt > other.cnt;
  }
  SortingElement() {
    cnt[0] = cnt[1] = cnt[2] = cnt[3] = 0;
    shift[0] = shift[1] = shift[2] = 0;
    lastCnt[0] = lastCnt[1] = lastCnt[2] = 0;
    data[0] = data[1] = data[2] = 0;
    sumData[0] = sumData[1] = sumData[2] = 0;
  }
  uint32_t  getCnt(int idx) const         { return cnt[idx]; }
  void      setCnt(uint32_t val, int idx) { cnt[idx] = val; }
  int       getSumData (int id) const   { return sumData[id]; }

  void resetAll(ComponentID compBegin, uint32_t numComp)
  {
    shift[0] = shift[1] = shift[2] = 0;
    lastCnt[0] = lastCnt[1] = lastCnt[2] = 0;
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch] = 0;
      sumData[ch] = 0;
    }
  }
  void setAll(uint32_t* ui, ComponentID compBegin, uint32_t numComp)
  {
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch] = ui[ch];
    }
  }
  bool almostEqualData(SortingElement element, int errorLimit, const BitDepths& bitDepths, ComponentID compBegin, uint32_t numComp, bool lossless)
  {
    bool almostEqual = true;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      if (lossless)
      {
        if ((std::abs(data[comp] - element.data[comp])) > errorLimit)
        {
          almostEqual = false;
          break;
        }
      }
      else
      {
      uint32_t absError = 0;
      if (isChroma((ComponentID) comp))
      {
        absError += int(double(std::abs(data[comp] - element.data[comp])) * PLT_CHROMA_WEIGHTING) >> (bitDepths.recon[CHANNEL_TYPE_CHROMA] - PLT_ENCBITDEPTH);
      }
      else
      {
        absError += (std::abs(data[comp] - element.data[comp]))>> (bitDepths.recon[CHANNEL_TYPE_LUMA] - PLT_ENCBITDEPTH);
      }
      if (absError > errorLimit)
      {
        almostEqual = false;
        break;
      }
      }
    }
    return almostEqual;
  }
  uint32_t getSAD(SortingElement element, const BitDepths &bitDepths, ComponentID compBegin, uint32_t numComp, bool lossless)
  {
    uint32_t sumAd = 0;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      ChannelType chType = (comp > 0) ? CHANNEL_TYPE_CHROMA : CHANNEL_TYPE_LUMA;
      if (lossless)
      {
        sumAd += (std::abs(data[comp] - element.data[comp]));
      }
      else
      {
      sumAd += (std::abs(data[comp] - element.data[comp]) >> (bitDepths.recon[chType] - PLT_ENCBITDEPTH));
      }
    }
    return sumAd;
  }
  void copyDataFrom(SortingElement element, ComponentID compBegin, uint32_t numComp)
  {
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      data[comp] = element.data[comp];
      sumData[comp] = data[comp];
      shift[comp] = 0;
      lastCnt[comp] = 1;
    }
  }
  void copyAllFrom(SortingElement element, ComponentID compBegin, uint32_t numComp)
  {
    copyDataFrom(element, compBegin, numComp);
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      sumData[comp] = element.sumData[comp];
      cnt[comp]     = element.cnt[comp];
      shift[comp]   = element.shift[comp];
      lastCnt[comp] = element.lastCnt[comp];
    }
    cnt[MAX_NUM_COMPONENT] = element.cnt[MAX_NUM_COMPONENT];
  }
  void addElement(const SortingElement& element, ComponentID compBegin, uint32_t numComp)
  {
    for (int i = compBegin; i<(compBegin + numComp); i++)
    {
      sumData[i] += element.data[i];
      cnt[i]++;
      if( cnt[i] > 1 && cnt[i] == 2 * lastCnt[i] )
      {
        uint32_t rnd = 1 << shift[i];
        shift[i]++;
        data[i] = (sumData[i] + rnd) >> shift[i];
        lastCnt[i] = cnt[i];
      }
    }
  }
private:
  uint32_t cnt[MAX_NUM_COMPONENT+1];
  int shift[3], lastCnt[3], data[3], sumData[3];
};
/// encoder search class
class IntraSearch : public IntraPrediction
{
private:
  EncModeCtrl    *m_modeCtrl;
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_TBLOCKS];

  XUCache         m_unitCache;

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure ***m_pTempCS;
  CodingStructure ***m_pBestCS;

  CodingStructure **m_pSaveCS;

  bool            m_saveCuCostInSCIPU;
  uint8_t         m_numCuInSCIPU;
  Area            m_cuAreaInSCIPU[NUM_INTER_CU_INFO_SAVE];
  double          m_cuCostInSCIPU[NUM_INTER_CU_INFO_SAVE];

  struct ModeInfo
  {
    bool     mipFlg; // CU::mipFlag
    bool     mipTrFlg; // PU::mipTransposedFlag
    int      mRefId; // PU::multiRefIdx
    uint8_t  ispMod; // CU::ispMode
    uint32_t modeId; // PU::intraDir[CHANNEL_TYPE_LUMA]

    ModeInfo() : mipFlg(false), mipTrFlg(false), mRefId(0), ispMod(NOT_INTRA_SUBPARTITIONS), modeId(0) {}
    ModeInfo(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode) : mipFlg(mipf), mipTrFlg(miptf), mRefId(mrid), ispMod(ispm), modeId(mode) {}
    bool operator==(const ModeInfo cmp) const { return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId); }
  };
  struct ModeInfoWithCost : public ModeInfo
  {
    double rdCost;
    ModeInfoWithCost() : ModeInfo(), rdCost(MAX_DOUBLE) {}
    ModeInfoWithCost(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode, double cost) : ModeInfo(mipf, miptf, mrid, ispm, mode), rdCost(cost) {}
    bool operator==(const ModeInfoWithCost cmp) const { return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId && rdCost == cmp.rdCost); }
    static bool compareModeInfoWithCost(ModeInfoWithCost a, ModeInfoWithCost b) { return a.rdCost < b.rdCost; }
  };

  struct ISPTestedModeInfo
  {
    int    numCompSubParts;
    double rdCost;

    ISPTestedModeInfo() {}

    void setMode(int numParts, double cost)
    {
      numCompSubParts = numParts;
      rdCost = cost;
    }
    void clear()
    {
      numCompSubParts = -1;
      rdCost = MAX_DOUBLE;
    }
  };
  struct ISPTestedModesInfo
  {
    ISPTestedModeInfo                           intraMode[NUM_LUMA_MODE][2];
    bool                                        modeHasBeenTested[NUM_LUMA_MODE][2];
    int                                         numTotalParts[2];
    static_vector<int, FAST_UDI_MAX_RDMODE_NUM> testedModes[2];
    int                                         bestModeSoFar;
    ISPType                                     bestSplitSoFar;
    int                                         bestMode[2];
    double                                      bestCost[2];
    int                                         numTestedModes[2];
    int                                         candIndexInList[2];
    bool                                        splitIsFinished[2];
    int                                         numOrigModesToTest;

    // set a tested mode results
    void setModeResults(ISPType splitType, int iModeIdx, int numCompletedParts, double rdCost, double currentBestCost)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      const int maxNumParts = numTotalParts[st];
      intraMode[iModeIdx][st].setMode(numCompletedParts, numCompletedParts == maxNumParts ? rdCost : MAX_DOUBLE);
      testedModes[st].push_back(iModeIdx);
      numTestedModes[st]++;
      modeHasBeenTested[iModeIdx][st] = true;
      if (numCompletedParts == maxNumParts && rdCost < bestCost[st])   // best mode update
      {
        bestMode[st] = iModeIdx;
        bestCost[st] = rdCost;
      }
      if (numCompletedParts == maxNumParts && rdCost < currentBestCost)   // best mode update
      {
        bestModeSoFar = iModeIdx;
        bestSplitSoFar = splitType;
      }
    }

    int getNumCompletedSubParts(ISPType splitType, int iModeIdx)
    {
      const unsigned st = splitType - 1;
      CHECK_(st < 0 || st > 1, "The split type is invalid!");
      CHECK_(iModeIdx < 0 || iModeIdx >(NUM_LUMA_MODE - 1), "The modeIdx is invalid");
      return modeHasBeenTested[iModeIdx][st] ? intraMode[iModeIdx][st].numCompSubParts : -1;
    }

    double getRDCost(ISPType splitType, int iModeIdx)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      return modeHasBeenTested[iModeIdx][st] ? intraMode[iModeIdx][st].rdCost : MAX_DOUBLE;
    }

    // get a tested intra mode index
    int getTestedIntraMode(ISPType splitType, int pos)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      return pos < testedModes[st].size() ? testedModes[st].at(pos) : -1;
    }

    // set everything to default values
    void clear()
    {
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        numTestedModes [splitIdx] = 0;
        candIndexInList[splitIdx] = 0;
        numTotalParts  [splitIdx] = 0;
        splitIsFinished[splitIdx] = false;
        testedModes    [splitIdx].clear();
        bestCost       [splitIdx] = MAX_DOUBLE;
        bestMode       [splitIdx] = -1;
      }
      bestModeSoFar = -1;
      bestSplitSoFar = NOT_INTRA_SUBPARTITIONS;
      numOrigModesToTest = -1;
      memset(modeHasBeenTested, 0, sizeof(modeHasBeenTested));
    }
    void clearISPModeInfo(int idx)
    {
      intraMode[idx][0].clear();
      intraMode[idx][1].clear();
    }
    void init(const int numTotalPartsHor, const int numTotalPartsVer)
    {
      clear();
      const int horSplit = HOR_INTRA_SUBPARTITIONS - 1, verSplit = VER_INTRA_SUBPARTITIONS - 1;
      numTotalParts  [horSplit] = numTotalPartsHor;
      numTotalParts  [verSplit] = numTotalPartsVer;
      splitIsFinished[horSplit] = (numTotalParts[horSplit] == 0);
      splitIsFinished[verSplit] = (numTotalParts[verSplit] == 0);
    }
  };

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_ispCandListHor, m_ispCandListVer;
  static_vector<ModeInfoWithCost, FAST_UDI_MAX_RDMODE_NUM> m_regIntraRDListWithCosts;

  ISPTestedModesInfo m_ispTestedModes[NUM_LFNST_NUM_PER_SET];
  int m_curIspLfnstIdx;

  //cost variables for the EMT algorithm and new modes list
  double     m_bestModeCostStore[ NUM_LFNST_NUM_PER_SET ];                                    // RD cost of the best mode for each PU using DCT2
  bool       m_bestModeCostValid[ NUM_LFNST_NUM_PER_SET ];
  double     m_modeCostStore[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];                   // RD cost of each mode for each PU using DCT2
  ModeInfo   m_savedRdModeList[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];
  int32_t    m_savedNumRdModes[ NUM_LFNST_NUM_PER_SET ];

  ModeInfo                                           m_savedRdModeFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2][FAST_UDI_MAX_RDMODE_NUM];
  char                                               m_savedBDPCMModeFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2][FAST_UDI_MAX_RDMODE_NUM];
  double                                             m_savedRdCostFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2][FAST_UDI_MAX_RDMODE_NUM];
  int                                                m_numSavedRdModeFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2];
  int                                                m_savedRdModeIdx;

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedRdModeListLFNST;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedHadModeListLFNST;
  uint32_t                                         m_uiSavedNumRdModesLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedModeCostLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedHadListLFNST;

  PelStorage      m_tmpStorageLCU;
  PelStorage      m_colorTransResiBuf;
protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;
  RdCost*         m_pcRdCost;
  EncReshape*     m_pcReshape;

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;

  bool            m_isInitialized;
  uint32_t        m_symbolSize;
  uint16_t**      m_truncBinBits;
  uint16_t*       m_escapeNumBins;
  bool            m_bestEscape;
  double*         m_indexError[MAXPLTSIZE + 1];
  uint8_t*        m_minErrorIndexMap; // store the best index in terms of distortion for each pixel
  uint8_t         m_indexMapRDOQ   [2][NUM_TRELLIS_STATE][2 * MAX_CU_BLKSIZE_PLT];
  bool            m_runMapRDOQ     [2][NUM_TRELLIS_STATE][2 * MAX_CU_BLKSIZE_PLT];
  uint8_t*        m_statePtRDOQ    [NUM_TRELLIS_STATE];
  bool            m_prevRunTypeRDOQ[2][NUM_TRELLIS_STATE];
  int             m_prevRunPosRDOQ [2][NUM_TRELLIS_STATE];
  double          m_stateCostRDOQ  [2][NUM_TRELLIS_STATE];
public:

  IntraSearch();
  ~IntraSearch();

  void init                       ( EncCfg*        pcEncCfg,
                                    TrQuant*       pcTrQuant,
                                    RdCost*        pcRdCost,
                                    CABACWriter*   CABACEstimator,
                                    CtxCache*      ctxCache,
                                    const uint32_t     maxCUWidth,
                                    const uint32_t     maxCUHeight,
                                    const uint32_t     maxTotalCUDepth
                                  , EncReshape*   m_pcReshape
                                  , const unsigned bitDepthY
                                  );

  void destroy                    ();

  CodingStructure****getSplitCSBuf() { return m_pSplitCS; }
  CodingStructure****getFullCSBuf () { return m_pFullCS; }
  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

  void setModeCtrl                ( EncModeCtrl *modeCtrl ) { m_modeCtrl = modeCtrl; }

  bool getSaveCuCostInSCIPU       ()               { return m_saveCuCostInSCIPU; }
  void setSaveCuCostInSCIPU       ( bool b )       { m_saveCuCostInSCIPU = b;  }
  void setNumCuInSCIPU            ( uint8_t i )    { m_numCuInSCIPU = i; }
  void saveCuAreaCostInSCIPU      ( Area area, double cost );
  void initCuAreaCostInSCIPU      ();
  double findInterCUCost          ( CodingUnit &cu );

public:
  bool estIntraPredLumaQT(CodingUnit &cu, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false, CodingStructure* bestCS = NULL);
  void estIntraPredChromaQT       ( CodingUnit &cu, Partitioner& pm, const double maxCostAllowed = MAX_DOUBLE );
  void PLTSearch                  ( CodingStructure &cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  uint64_t xFracModeBitsIntra     (PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &compID);
  void invalidateBestModeCost     () { for( int i = 0; i < NUM_LFNST_NUM_PER_SET; i++ ) m_bestModeCostValid[ i ] = false; };

  void sortRdModeListFirstColorSpace(ModeInfo mode, double cost, char bdpcmMode, ModeInfo* rdModeList, double* rdCostList, char* bdpcmModeList, int& candNum);
  void invalidateBestRdModeFirstColorSpace();
  void setSavedRdModeIdx(int idx) { m_savedRdModeIdx = idx; }

protected:

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------


  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  void     xEncIntraHeader                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1 );
  void     xEncSubdivCbfQT                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
  uint64_t xGetIntraFracBitsQT                     ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, CUCtx * cuCtx = nullptr  );
  uint64_t xGetIntraFracBitsQTSingleChromaComponent( CodingStructure &cs, Partitioner& pm, const ComponentID compID );

  uint64_t xGetIntraFracBitsQTChroma(TransformUnit& tu, const ComponentID &compID);
  void xEncCoeffQT                                 ( CodingStructure &cs, Partitioner& pm, const ComponentID compID, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, CUCtx * cuCtx = nullptr );

  void xIntraCodingTUBlock        (TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, const int &default0Save1Load2 = 0, uint32_t* numSig = nullptr, std::vector<TrMode>* trModes=nullptr, const bool loadTr=false );
  void xIntraCodingACTTUBlock(TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, std::vector<TrMode>* trModes = nullptr, const bool loadTr = false);

  ChromaCbfs xRecurIntraChromaCodingQT( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE,                          const PartSplit ispType = TU_NO_ISP );
  bool       xRecurIntraCodingLumaQT  ( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, const bool ispIsCurrentWinner = false, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false );
  bool       xRecurIntraCodingACTQT(CodingStructure &cs, Partitioner& pm, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false);
  bool       xIntraCodingLumaISP      ( CodingStructure& cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE );

  template<typename T, size_t N>
  void reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const PredictionUnit &pu, const bool fastMip);
  void   derivePLTLossy  (      CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  void   calcPixelPred   (      CodingStructure& cs, Partitioner& partitioner, uint32_t    yPos,      uint32_t xPos,             ComponentID compBegin, uint32_t  numComp);
  void     preCalcPLTIndexRD      (CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  void     calcPixelPredRD        (CodingStructure& cs, Partitioner& partitioner, Pel* orgBuf, Pel* pixelValue, Pel* recoValue, ComponentID compBegin, uint32_t numComp);
  void     deriveIndexMap         (CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp, PLTScanMode pltScanMode, double& dCost, bool* idxExist);
  bool     deriveSubblockIndexMap(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, PLTScanMode pltScanMode, int minSubPos, int maxSubPos, const BinFracBits& fracBitsPltRunType, const BinFracBits* fracBitsPltIndexINDEX, const BinFracBits* fracBitsPltIndexCOPY, const double minCost, bool useRotate);
  double   rateDistOptPLT         (bool RunType, uint8_t RunIndex, bool prevRunType, uint8_t prevRunIndex, uint8_t aboveRunIndex, bool& prevCodedRunType, int& prevCodedRunPos, int scanPos, uint32_t width, int dist, int indexMaxValue, const BinFracBits* IndexfracBits, const BinFracBits& TypefracBits);
  void     initTBCTable           (int bitDepth);
  uint32_t getTruncBinBits        (uint32_t symbol, uint32_t maxSymbol);
  uint32_t getEpExGolombNumBins   (uint32_t symbol, uint32_t count);
  void xGetNextISPMode                    ( ModeInfo& modeInfo, const ModeInfo* lastMode, const Size cuSize );
  bool xSortISPCandList                   ( double bestCostSoFar, double bestNonISPCost, ModeInfo bestNonISPMode );
  void xSortISPCandListLFNST              ( );
  void xFindAlreadyTestedNearbyIntraModes ( int currentLfnstIdx, int currentIntraMode, int* refLfnstIdx, int* leftIntraMode, int* rightIntraMode, ISPType ispOption, int windowSize );
  bool updateISPStatusFromRelCU           ( double bestNonISPCostCurrCu, ModeInfo bestNonISPModeCurrCu, int& bestISPModeInRelCU );
  void xFinishISPModes                    ( );
};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__

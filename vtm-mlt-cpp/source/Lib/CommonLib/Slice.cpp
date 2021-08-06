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

/** \file     Slice.cpp
    \brief    slice header and SPS class
*/

#include "CommonDef.h"
#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "dtrace_next.h"

#include "UnitTools.h"

//! \ingroup CommonLib
//! \{

Slice::Slice()
: m_iPOC                          ( 0 )
, m_iLastIDR                      ( 0 )
, m_prevGDRInSameLayerPOC         ( -MAX_INT )
, m_iAssociatedIRAP               ( 0 )
, m_iAssociatedIRAPType           ( NAL_UNIT_INVALID )
, m_prevGDRSubpicPOC              ( -MAX_INT )
, m_prevIRAPSubpicPOC             ( -MAX_INT )
, m_prevIRAPSubpicType            ( NAL_UNIT_INVALID )
, m_rpl0Idx                       ( -1 )
, m_rpl1Idx                       ( -1 )
, m_eNalUnitType                  ( NAL_UNIT_CODED_SLICE_IDR_W_RADL )
, m_pictureHeaderInSliceHeader   ( false )
, m_eSliceType                    ( I_SLICE )
, m_noOutputOfPriorPicsFlag       ( 0 )
, m_iSliceQp                      ( 0 )
, m_ChromaQpAdjEnabled            ( false )
, m_lmcsEnabledFlag               ( 0 )
, m_explicitScalingListUsed       ( 0 )
, m_deblockingFilterDisable       ( false )
, m_deblockingFilterOverrideFlag  ( false )
, m_deblockingFilterBetaOffsetDiv2( 0 )
, m_deblockingFilterTcOffsetDiv2  ( 0 )
, m_deblockingFilterCbBetaOffsetDiv2( 0 )
, m_deblockingFilterCbTcOffsetDiv2  ( 0 )
, m_deblockingFilterCrBetaOffsetDiv2( 0 )
, m_deblockingFilterCrTcOffsetDiv2  ( 0 )
, m_depQuantEnabledFlag             ( false )
, m_signDataHidingEnabledFlag       ( false )
, m_tsResidualCodingDisabledFlag  ( false )
, m_pendingRasInit                ( false )
, m_bCheckLDC                     ( false )
, m_biDirPred                    ( false )
, m_iSliceQpDelta                 ( 0 )
, m_iDepth                        ( 0 )
, m_pcSPS                         ( NULL )
, m_pcPPS                         ( NULL )
, m_pcPic                         ( NULL )
, m_pcPicHeader                   ( NULL )
, m_colFromL0Flag                 ( true )
, m_colRefIdx                     ( 0 )
, m_uiTLayer                      ( 0 )
, m_bTLayerSwitchingFlag          ( false )
, m_independentSliceIdx           ( 0 )
, m_nextSlice                     ( false )
, m_sliceBits                     ( 0 )
, m_bFinalized                    ( false )
, m_bTestWeightPred               ( false )
, m_bTestWeightBiPred             ( false )
, m_substreamSizes                ( )
, m_numEntryPoints                ( 0 )
, m_cabacInitFlag                 ( false )
 , m_sliceSubPicId               ( 0 )
, m_encCABACTableIdx              (I_SLICE)
, m_iProcessingStartTime          ( 0 )
, m_dProcessingTime               ( 0 )
{
  for(uint32_t i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i] = 0;
  }

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_lambdas            [component] = 0.0;
    m_iSliceChromaQpDelta[component] = 0;
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = 0;

  initEqualRef();

  for ( int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    m_list1IdxToList0Idx[idx] = -1;
  }

  for(int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
  {
    for(uint32_t i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_apcRefPicList [i][iNumCount] = NULL;
      m_aiRefPOCList  [i][iNumCount] = 0;
    }
  }

  resetWpScaling();
  initWpAcDcParam();

  for(int ch=0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = false;
  }

  memset(m_alfApss, 0, sizeof(m_alfApss));
  m_ccAlfFilterParam.reset();
  resetTileGroupAlfEnabledFlag();
  resetTileGroupCcAlCbfEnabledFlag();
  resetTileGroupCcAlCrfEnabledFlag();

  m_sliceMap.initSliceMap();
}

Slice::~Slice()
{
  m_sliceMap.initSliceMap();
}


void Slice::initSlice()
{
  for(uint32_t i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]      = 0;
  }
  m_colFromL0Flag = true;
  m_colRefIdx = 0;
  m_lmcsEnabledFlag = 0;
  m_explicitScalingListUsed = 0;
  initEqualRef();

  m_noOutputOfPriorPicsFlag = 0;

  m_bCheckLDC = false;

  m_biDirPred = false;
  m_symRefIdx[0] = -1;
  m_symRefIdx[1] = -1;

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = 0;
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = 0;


  m_bFinalized=false;

  m_substreamSizes.clear();
  m_cabacInitFlag        = false;
  m_enableDRAPSEI        = false;
  m_useLTforDRAP         = false;
  m_isDRAP               = false;
  m_latestDRAPPOC        = MAX_INT;
  resetTileGroupAlfEnabledFlag();
  m_ccAlfFilterParam.reset();
  m_tileGroupCcAlfCbEnabledFlag = 0;
  m_tileGroupCcAlfCrEnabledFlag = 0;
  m_tileGroupCcAlfCbApsId = -1;
  m_tileGroupCcAlfCrApsId = -1;
  m_nuhLayerId = 0;
}

void Slice::inheritFromPicHeader( PicHeader *picHeader, const PPS *pps, const SPS *sps )
{
  if (pps->getRplInfoInPhFlag())
  {
    setRPL0idx( picHeader->getRPL0idx() );
    m_RPL0 = *picHeader->getRPL0();
    if(getRPL0idx() != -1)
    {
      m_RPL0 = *sps->getRPLList0()->getReferencePictureList(getRPL0idx());
    }

    setRPL1idx( picHeader->getRPL1idx() );
    m_RPL1 = *picHeader->getRPL1();
    if(getRPL1idx() != -1)
    {
      m_RPL1 = *sps->getRPLList1()->getReferencePictureList(getRPL1idx());
    }
  }

  setDeblockingFilterDisable( picHeader->getDeblockingFilterDisable() );
  setDeblockingFilterBetaOffsetDiv2( picHeader->getDeblockingFilterBetaOffsetDiv2() );
  setDeblockingFilterTcOffsetDiv2( picHeader->getDeblockingFilterTcOffsetDiv2() );
  if (pps->getPPSChromaToolFlag())
  {
    setDeblockingFilterCbBetaOffsetDiv2 ( picHeader->getDeblockingFilterCbBetaOffsetDiv2() );
    setDeblockingFilterCbTcOffsetDiv2   ( picHeader->getDeblockingFilterCbTcOffsetDiv2()   );
    setDeblockingFilterCrBetaOffsetDiv2 ( picHeader->getDeblockingFilterCrBetaOffsetDiv2() );
    setDeblockingFilterCrTcOffsetDiv2   ( picHeader->getDeblockingFilterCrTcOffsetDiv2()   );
  }
  else
  {
    setDeblockingFilterCbBetaOffsetDiv2 ( getDeblockingFilterBetaOffsetDiv2() );
    setDeblockingFilterCbTcOffsetDiv2   ( getDeblockingFilterTcOffsetDiv2()   );
    setDeblockingFilterCrBetaOffsetDiv2 ( getDeblockingFilterBetaOffsetDiv2() );
    setDeblockingFilterCrTcOffsetDiv2   ( getDeblockingFilterTcOffsetDiv2()   );
  }

  setSaoEnabledFlag(CHANNEL_TYPE_LUMA,     picHeader->getSaoEnabledFlag(CHANNEL_TYPE_LUMA));
  setSaoEnabledFlag(CHANNEL_TYPE_CHROMA,   picHeader->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA));

  setTileGroupAlfEnabledFlag(COMPONENT_Y,  picHeader->getAlfEnabledFlag(COMPONENT_Y));
  setTileGroupAlfEnabledFlag(COMPONENT_Cb, picHeader->getAlfEnabledFlag(COMPONENT_Cb));
  setTileGroupAlfEnabledFlag(COMPONENT_Cr, picHeader->getAlfEnabledFlag(COMPONENT_Cr));
  setTileGroupNumAps(picHeader->getNumAlfAps());
  setAlfAPSs(picHeader->getAlfAPSs());
  setTileGroupApsIdChroma(picHeader->getAlfApsIdChroma());
  setTileGroupCcAlfCbEnabledFlag(picHeader->getCcAlfEnabledFlag(COMPONENT_Cb));
  setTileGroupCcAlfCrEnabledFlag(picHeader->getCcAlfEnabledFlag(COMPONENT_Cr));
  setTileGroupCcAlfCbApsId(picHeader->getCcAlfCbApsId());
  setTileGroupCcAlfCrApsId(picHeader->getCcAlfCrApsId());
  m_ccAlfFilterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1] = picHeader->getCcAlfEnabledFlag(COMPONENT_Cb);
  m_ccAlfFilterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1] = picHeader->getCcAlfEnabledFlag(COMPONENT_Cr);
}

void Slice::setNumSubstream(const SPS* sps, const PPS* pps)
{
  uint32_t ctuAddr, ctuX, ctuY;
  m_numSubstream = 0;

  // count the number of CTUs that align with either the start of a tile, or with an entropy coding sync point
  // ignore the first CTU since it doesn't count as an entry point
  for (uint32_t i = 1; i < m_sliceMap.getNumCtuInSlice(); i++)
  {
    ctuAddr = m_sliceMap.getCtuAddrInSlice(i);
    ctuX    = (ctuAddr % pps->getPicWidthInCtu());
    ctuY    = (ctuAddr / pps->getPicWidthInCtu());

    if (pps->ctuIsTileColBd(ctuX) && (pps->ctuIsTileRowBd(ctuY) || sps->getEntropyCodingSyncEnabledFlag()))
    {
      m_numSubstream++;
    }
  }
}

void Slice::setNumEntryPoints(const SPS *sps, const PPS *pps)
{
  uint32_t ctuAddr, ctuX, ctuY;
  uint32_t prevCtuAddr, prevCtuX, prevCtuY;
  m_numEntryPoints = 0;

  if (!sps->getEntryPointsPresentFlag())
  {
    return;
  }

  // count the number of CTUs that align with either the start of a tile, or with an entropy coding sync point
  // ignore the first CTU since it doesn't count as an entry point
  for( uint32_t i = 1; i < m_sliceMap.getNumCtuInSlice(); i++ )
  {
    ctuAddr = m_sliceMap.getCtuAddrInSlice( i );
    ctuX = ( ctuAddr % pps->getPicWidthInCtu() );
    ctuY = ( ctuAddr / pps->getPicWidthInCtu() );
    prevCtuAddr = m_sliceMap.getCtuAddrInSlice(i - 1);
    prevCtuX    = (prevCtuAddr % pps->getPicWidthInCtu());
    prevCtuY    = (prevCtuAddr / pps->getPicWidthInCtu());

    if (pps->ctuToTileRowBd(ctuY) != pps->ctuToTileRowBd(prevCtuY) || pps->ctuToTileColBd(ctuX) != pps->ctuToTileColBd(prevCtuX) || (ctuY != prevCtuY && sps->getEntropyCodingSyncEnabledFlag()))
    {
      m_numEntryPoints++;
    }
  }
}

void Slice::setDefaultClpRng( const SPS& sps )
{
  m_clpRngs.comp[COMPONENT_Y].min = m_clpRngs.comp[COMPONENT_Cb].min  = m_clpRngs.comp[COMPONENT_Cr].min = 0;
  m_clpRngs.comp[COMPONENT_Y].max                                     = (1<< sps.getBitDepth(CHANNEL_TYPE_LUMA))-1;
  m_clpRngs.comp[COMPONENT_Y].bd  = sps.getBitDepth(CHANNEL_TYPE_LUMA);
  m_clpRngs.comp[COMPONENT_Y].n   = 0;
  m_clpRngs.comp[COMPONENT_Cb].max = m_clpRngs.comp[COMPONENT_Cr].max = (1<< sps.getBitDepth(CHANNEL_TYPE_CHROMA))-1;
  m_clpRngs.comp[COMPONENT_Cb].bd  = m_clpRngs.comp[COMPONENT_Cr].bd  = sps.getBitDepth(CHANNEL_TYPE_CHROMA);
  m_clpRngs.comp[COMPONENT_Cb].n   = m_clpRngs.comp[COMPONENT_Cr].n   = 0;
  m_clpRngs.used = m_clpRngs.chroma = false;
}


bool Slice::getRapPicFlag() const
{
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}

void Slice::sortPicList(PicList &picList)
{
  picList.sort([](Picture *const &a, Picture *const &b) {
    return a->getPOC() < b->getPOC() || (a->getPOC() == b->getPOC() && a->layerId < b->layerId);
  });
}

Picture* Slice::xGetRefPic( PicList& rcListPic, const int poc, const int layerId )
{
  // return a nullptr, if picture is not found
  Picture* refPic = nullptr;

  for ( auto &currPic : rcListPic )
  {
    if( currPic->getPOC() == poc && currPic->layerId == layerId )
    {
      refPic = currPic;
      break;
    }
  }
  return  refPic;
}

Picture* Slice::xGetLongTermRefPic( PicList& rcListPic, const int poc, const bool pocHasMsb, const int layerId )
{
  // return a nullptr, if picture is not found or the found picture is not long-term
  Picture*  refPic = nullptr;
  const int pocCycle = 1 << getSPS()->getBitsForPOC();

  const int refPoc = pocHasMsb ? poc : (poc & (pocCycle - 1));

  for ( auto &currPic : rcListPic )
  {
    if( currPic->getPOC() != this->getPOC() && currPic->referenced && currPic->layerId == layerId )
    {
      int currPicPoc = pocHasMsb ? currPic->getPOC() : (currPic->getPOC() & (pocCycle - 1));
      if (refPoc == currPicPoc)
      {
        if(currPic->longTerm)
        {
          refPic = currPic;
        }
        break;
      }
    }
  }

  return refPic;
}

Picture* Slice::xGetLongTermRefPicCandidate( PicList& rcListPic, const int poc, const bool pocHasMsb, const int layerId )
{
  // return a nullptr, if picture is not found (might be a short-term or a long-term)
  Picture*  refPic = nullptr;
  const int pocCycle = 1 << getSPS()->getBitsForPOC();

  const int refPoc = pocHasMsb ? poc : (poc & (pocCycle - 1));

  for ( auto &currPic : rcListPic )
  {
    if( currPic->getPOC() != this->getPOC() && currPic->referenced && currPic->layerId == layerId )
    {
      int currPicPoc = pocHasMsb ? currPic->getPOC() : (currPic->getPOC() & (pocCycle - 1));
      if (refPoc == currPicPoc)
      {
        refPic = currPic;
        break;
      }
    }
  }

  return refPic;
}

void Slice::setRefPOCList       ()
{
  for (int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (int iNumRefIdx = 0; iNumRefIdx < m_aiNumRefIdx[iDir]; iNumRefIdx++)
    {
      m_aiRefPOCList[iDir][iNumRefIdx] = m_apcRefPicList[iDir][iNumRefIdx]->getPOC();
    }
  }

}

void Slice::setList1IdxToList0Idx()
{
  int idxL0, idxL1;
  for ( idxL1 = 0; idxL1 < getNumRefIdx( REF_PIC_LIST_1 ); idxL1++ )
  {
    m_list1IdxToList0Idx[idxL1] = -1;
    for ( idxL0 = 0; idxL0 < getNumRefIdx( REF_PIC_LIST_0 ); idxL0++ )
    {
      if ( m_apcRefPicList[REF_PIC_LIST_0][idxL0]->getPOC() == m_apcRefPicList[REF_PIC_LIST_1][idxL1]->getPOC() )
      {
        m_list1IdxToList0Idx[idxL1] = idxL0;
        break;
      }
    }
  }
}

void Slice::constructRefPicList(PicList& rcListPic)
{
  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));
  if (m_eSliceType == I_SLICE)
  {
    ::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
    ::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));
    return;
  }

  Picture*  pcRefPic = NULL;
  uint32_t numOfActiveRef = 0;
  //construct L0
  numOfActiveRef = getNumRefIdx(REF_PIC_LIST_0);
  int layerIdx = m_pcPic->cs->vps == nullptr ? 0 : m_pcPic->cs->vps->getGeneralLayerIdx( m_pcPic->layerId );

  for (int ii = 0; ii < m_RPL0.getNumRefEntries(); ii++)
  {
    if( m_RPL0.isInterLayerRefPic( ii ) )
    {
      CHECK_( m_RPL0.getInterLayerRefPicIdx( ii ) == NOT_VALID, "Wrong ILRP index" );

      int refLayerId = m_pcPic->cs->vps->getLayerId( m_pcPic->cs->vps->getDirectRefLayerIdx( layerIdx, m_RPL0.getInterLayerRefPicIdx( ii ) ) );

      pcRefPic = xGetRefPic( rcListPic, getPOC(), refLayerId );
      pcRefPic->longTerm = true;
    }
    else
    if (!m_RPL0.isRefPicLongterm(ii))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC() + m_RPL0.getRefPicIdentifier(ii), m_pcPic->layerId);
      pcRefPic->longTerm = false;
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = m_RPL0.getRefPicIdentifier(ii) & pocMask;
      if(m_RPL0.getDeltaPocMSBPresentFlag(ii))
      {
        ltrpPoc += getPOC() - m_RPL0.getDeltaPocMSBCycleLT(ii) * (pocMask + 1) - (getPOC() & pocMask);
      }
      pcRefPic = xGetLongTermRefPicCandidate( rcListPic, ltrpPoc, m_RPL0.getDeltaPocMSBPresentFlag( ii ), m_pcPic->layerId );
      pcRefPic->longTerm = true;
    }
    if(ii < numOfActiveRef)
    {
      pcRefPic->extendPicBorder( getPPS() );
      m_apcRefPicList[REF_PIC_LIST_0][ii] = pcRefPic;
      m_bIsUsedAsLongTerm[REF_PIC_LIST_0][ii] = pcRefPic->longTerm;
    }
  }

  //construct L1
  numOfActiveRef = getNumRefIdx(REF_PIC_LIST_1);
  for (int ii = 0; ii < m_RPL1.getNumRefEntries(); ii++)
  {
    if( m_RPL1.isInterLayerRefPic( ii ) )
    {
      CHECK_( m_RPL1.getInterLayerRefPicIdx( ii ) == NOT_VALID, "Wrong ILRP index" );

      int refLayerId = m_pcPic->cs->vps->getLayerId( m_pcPic->cs->vps->getDirectRefLayerIdx( layerIdx, m_RPL1.getInterLayerRefPicIdx( ii ) ) );

      pcRefPic = xGetRefPic( rcListPic, getPOC(), refLayerId );
      pcRefPic->longTerm = true;
    }
    else
    if (!m_RPL1.isRefPicLongterm(ii))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC() + m_RPL1.getRefPicIdentifier(ii), m_pcPic->layerId);
      pcRefPic->longTerm = false;
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = m_RPL1.getRefPicIdentifier(ii) & pocMask;
      if(m_RPL1.getDeltaPocMSBPresentFlag(ii))
      {
        ltrpPoc += getPOC() - m_RPL1.getDeltaPocMSBCycleLT(ii) * (pocMask + 1) - (getPOC() & pocMask);
      }
      pcRefPic = xGetLongTermRefPicCandidate( rcListPic, ltrpPoc, m_RPL1.getDeltaPocMSBPresentFlag( ii ), m_pcPic->layerId );
      pcRefPic->longTerm = true;
    }
    if(ii < numOfActiveRef)
    {
      pcRefPic->extendPicBorder( getPPS() );
      m_apcRefPicList[REF_PIC_LIST_1][ii] = pcRefPic;
      m_bIsUsedAsLongTerm[REF_PIC_LIST_1][ii] = pcRefPic->longTerm;
    }
  }
}

void Slice::initEqualRef()
{
  for (int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (int iRefIdx1 = 0; iRefIdx1 < MAX_NUM_REF; iRefIdx1++)
    {
      for (int iRefIdx2 = iRefIdx1; iRefIdx2 < MAX_NUM_REF; iRefIdx2++)
      {
        m_abEqualRef[iDir][iRefIdx1][iRefIdx2] = m_abEqualRef[iDir][iRefIdx2][iRefIdx1] = (iRefIdx1 == iRefIdx2? true : false);
      }
    }
  }
}

void Slice::checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic)
{
  int i;
  Slice* curSlice = pic->slices[curSliceSegmentIdx];
  int currColRefPOC =  curSlice->getRefPOC( RefPicList(1 - curSlice->getColFromL0Flag()), curSlice->getColRefIdx());

  for(i=curSliceSegmentIdx-1; i>=0; i--)
  {
    const Slice* preSlice = pic->slices[i];
    if(preSlice->getSliceType() != I_SLICE)
    {
      const int preColRefPOC  = preSlice->getRefPOC( RefPicList(1 - preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
      if(currColRefPOC != preColRefPOC)
      {
        THROW("sh_collocated_ref_idx shall always be the same for all slices of a coded picture!");
      }
      else
      {
        break;
      }
    }
  }
}

void Slice::checkCRA(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int pocCRA, PicList& rcListPic)
{
  if (pocCRA < MAX_UINT && getPOC() > pocCRA)
  {
    uint32_t numRefPic = pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures();
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL0->isRefPicLongterm(i))
      {
        CHECK_(getPOC() + pRPL0->getRefPicIdentifier(i) < pocCRA, "Invalid state");
      }
      else if (!pRPL0->isInterLayerRefPic(i))
      {
        int pocBits = getSPS()->getBitsForPOC();
        int pocMask = (1 << pocBits) - 1;
        int ltrpPoc = pRPL0->getRefPicIdentifier(i) & pocMask;
        if(pRPL0->getDeltaPocMSBPresentFlag(i))
        {
          ltrpPoc += getPOC() - pRPL0->getDeltaPocMSBCycleLT(i) * (pocMask + 1) - (getPOC() & pocMask);
        }
        const Picture *ltrp =
          xGetLongTermRefPic(rcListPic, ltrpPoc, pRPL0->getDeltaPocMSBPresentFlag(i), m_pcPic->layerId);
        CHECK_(ltrp == nullptr, "Long-term pic not found");
        CHECK_(ltrp->getPOC() < pocCRA, "Invalid state");
      }
    }
    numRefPic = pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures();
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL1->isRefPicLongterm(i))
      {
        CHECK_(getPOC() + pRPL1->getRefPicIdentifier(i) < pocCRA, "Invalid state");
      }
      else if( !pRPL1->isInterLayerRefPic( i ) )
      {
        int pocBits = getSPS()->getBitsForPOC();
        int pocMask = (1 << pocBits) - 1;
        int ltrpPoc = pRPL1->getRefPicIdentifier(i) & pocMask;
        if(pRPL1->getDeltaPocMSBPresentFlag(i))
        {
          ltrpPoc += getPOC() - pRPL1->getDeltaPocMSBCycleLT(i) * (pocMask + 1) - (getPOC() & pocMask);
        }
        const Picture *ltrp =
          xGetLongTermRefPic(rcListPic, ltrpPoc, pRPL1->getDeltaPocMSBPresentFlag(i), m_pcPic->layerId);
        CHECK_(ltrp == nullptr, "Long-term pic not found");
        CHECK_(ltrp->getPOC() < pocCRA, "Invalid state");
      }
    }
  }
}

void Slice::checkRPL(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int associatedIRAPDecodingOrderNumber, PicList& rcListPic)
{
  Picture* pcRefPic;
  int refPicPOC;
  int refPicDecodingOrderNumber;

  int irapPOC = getAssociatedIRAPPOC();

  const int numEntries[] = { pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures() + pRPL0->getNumberOfInterLayerPictures(), pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures() + pRPL1->getNumberOfInterLayerPictures() };
  const int numActiveEntries[] = { getNumRefIdx( REF_PIC_LIST_0 ), getNumRefIdx( REF_PIC_LIST_1 ) };
  const ReferencePictureList* rpl[] = { pRPL0, pRPL1 };
  const bool fieldSeqFlag = getSPS()->getFieldSeqFlag();
  const int layerIdx = m_pcPic->cs->vps == nullptr ? 0 : m_pcPic->cs->vps->getGeneralLayerIdx( m_pcPic->layerId );

  for( int refPicList = 0; refPicList < 2; refPicList++ )
  {
    for( int i = 0; i < numEntries[refPicList]; i++ )
    {
      if( rpl[refPicList]->isInterLayerRefPic( i ) )
      {
        int refLayerId = m_pcPic->cs->vps->getLayerId( m_pcPic->cs->vps->getDirectRefLayerIdx( layerIdx, rpl[refPicList]->getInterLayerRefPicIdx( i ) ) );
        pcRefPic = xGetRefPic( rcListPic, getPOC(), refLayerId );
        refPicPOC = pcRefPic->getPOC();
      }
      else if( !rpl[refPicList]->isRefPicLongterm( i ) )
      {
        refPicPOC = getPOC() + rpl[refPicList]->getRefPicIdentifier(i);
        pcRefPic = xGetRefPic( rcListPic, refPicPOC, m_pcPic->layerId );
      }
      else
      {
        int pocBits = getSPS()->getBitsForPOC();
        int pocMask = ( 1 << pocBits ) - 1;
        int ltrpPoc = rpl[refPicList]->getRefPicIdentifier( i ) & pocMask;
        if( rpl[refPicList]->getDeltaPocMSBPresentFlag( i ) )
        {
          ltrpPoc += getPOC() - rpl[refPicList]->getDeltaPocMSBCycleLT( i ) * ( pocMask + 1 ) - ( getPOC() & pocMask );
        }
        pcRefPic = xGetLongTermRefPic( rcListPic, ltrpPoc, rpl[refPicList]->getDeltaPocMSBPresentFlag( i ), m_pcPic->layerId );
        refPicPOC = pcRefPic->getPOC();
      }
      if (pcRefPic) // the checks are for all reference picture, but we may not have an inactive reference picture, if starting with a CRA
      {
        refPicDecodingOrderNumber = pcRefPic->getDecodingOrderNumber();

        if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP )
        {
          CHECK_( refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "When the current picture, with nuh_layer_id equal to a particular value layerId, "
            "is an IRAP picture, there shall be no picture referred to by an entry in RefPicList[ 0 ] that precedes, in output order or decoding order, any preceding IRAP picture "
            "with nuh_layer_id equal to layerId in decoding order (when present)." );
        }

        if( irapPOC < getPOC() && !fieldSeqFlag )
        {
          CHECK_( refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "When the current picture follows an IRAP picture having the same value "
            "of nuh_layer_id and the leading pictures, if any, associated with that IRAP picture, in both decoding order and output order, there shall be no picture referred "
            "to by an entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes that IRAP picture in output order or decoding order." );
        }

        // Generated reference picture does not have picture header
        const bool isGeneratedRefPic = pcRefPic->slices[0]->getPicHeader() ? false : true;

        const bool nonReferencePictureFlag = isGeneratedRefPic ? pcRefPic->slices[0]->getPicHeader()->getNonReferencePictureFlag() : pcRefPic->nonReferencePictureFlag;
        CHECK_( pcRefPic == m_pcPic || nonReferencePictureFlag, "The picture referred to by each entry in RefPicList[ 0 ] or RefPicList[ 1 ] shall not be the current picture and shall have ph_non_ref_pic_flag equal to 0" );

        if( i < numActiveEntries[refPicList] )
        {
          if( irapPOC < getPOC() )
          {
            CHECK_( refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "When the current picture follows an IRAP picture having the same value "
              "of nuh_layer_id in both decoding order and output order, there shall be no picture referred to by an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that "
              "precedes that IRAP picture in output order or decoding order." );
          }

          // Checking this: "When the current picture is a RADL picture, there shall be no active entry in RefPicList[ 0 ] or
          // RefPicList[ 1 ] that is any of the following: A picture that precedes the associated IRAP picture in decoding order"
          if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL )
          {
            CHECK_( refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "RADL picture detected that violate the rule that no active entry in RefPicList[] shall precede the associated IRAP picture in decoding order" );
#if JVET_S0084_S0110_RADL
            // Checking this: "When the current picture is a RADL picture, there shall be no active entry in RefPicList[ 0 ] or 
            // RefPicList[ 1 ] that is any of the following: A RASL picture with pps_mixed_nalu_types_in_pic_flag is equal to 0
            for (int i = 0; i < pcRefPic->numSlices; i++)
            {
              if (pcRefPic->slices[i]->getPPS()->getMixedNaluTypesInPicFlag() == 0)
              {
                CHECK_(pcRefPic->slices[i]->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL, "When the current picture is a RADL picture, there shall be no active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that is a RASL picture with pps_mixed_nalu_types_in_pic_flag is equal to 0");
              }
            }
#endif

          }

          CHECK_( pcRefPic->temporalId > m_pcPic->temporalId, "The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] shall be present in the DPB and shall have TemporalId less than or equal to that of the current picture." );
        }
#if JVET_R0046_IRAP_ASPECT2
        // Add a constraint on an ILRP being either an IRAP picture or having TemporalId less than or equal to
        // Max (0, vps_max_tid_il_ref_pics_plus1[ refPicVpsLayerId ] - 1 ), with refPicVpsLayerId equal to the value of
        // the nuh_layer_id of the referenced picture.
        if (rpl[refPicList]->isInterLayerRefPic(i))
        {
          bool cond1      = (pcRefPic->getPictureType() == NAL_UNIT_CODED_SLICE_GDR);
          bool cond2      = (pcRefPic->slices[0]->getPicHeader()->getRecoveryPocCnt() == 0);
          bool cond3      = (pcRefPic->cs->slice->isIRAP());
          
          const VPS *vps                  = pcRefPic->cs->vps;
          const int  maxTidILRefPicsPlus1 = vps->getMaxTidIlRefPicsPlus1(layerIdx, pcRefPic->layerId);
          bool cond4 = (pcRefPic->temporalId < maxTidILRefPicsPlus1);

          CHECK_((cond1 && cond2) || cond3 || cond4,
                "Either of the following conditions shall apply for the picture referred to by each ILRP entry, when "
                "present, in RefPicList[ 0 ] or RefPicList[ 1 ] of a slice of the current picture:-The picture is a "
                "GDR picture with "
                "ph_recovery_poc_cnt equal to 0 or an IRAP picture."
                "-The picture has TemporalId less than vps_max_tid_il_ref_pics_plus1[ currLayerIdx ][ refLayerIdx ], "
                "where currLayerIdx and refLayerIdx are equal to "
                "GeneralLayerIdx[ nuh_layer_id ] and GeneralLayerIdx[ refpicLayerId ], respectively. ");
        }
#endif
      }
    }
  }
}

void Slice::checkSTSA(PicList& rcListPic)
{
  int ii;
  Picture* pcRefPic = NULL;
  int numOfActiveRef = getNumRefIdx(REF_PIC_LIST_0);

  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    pcRefPic = m_apcRefPicList[REF_PIC_LIST_0][ii];

    if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA && pcRefPic->layerId == m_pcPic->layerId )
    {
      CHECK_( pcRefPic->temporalId == m_uiTLayer, "When the current picture is an STSA picture and nuh_layer_id equal to that of the current picture, there shall be no active entry in the RPL that has TemporalId equal to that of the current picture" );
    }

    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has TemporalId equal to that of the current picture, there shall be no
    // picture that has TemporalId equal to that of the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA picture in decoding order."
    CHECK_(pcRefPic->subLayerNonReferencePictureDueToSTSA, "The RPL of the current picture contains a picture that is not allowed in this temporal layer due to an earlier STSA picture");
  }

  numOfActiveRef = getNumRefIdx(REF_PIC_LIST_1);
  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    pcRefPic = m_apcRefPicList[REF_PIC_LIST_1][ii];

    if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA && pcRefPic->layerId == m_pcPic->layerId )
    {
      CHECK_( pcRefPic->temporalId == m_uiTLayer, "When the current picture is an STSA picture and nuh_layer_id equal to that of the current picture, there shall be no active entry in the RPL that has TemporalId equal to that of the current picture" );
    }

    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has TemporalId equal to that of the current picture, there shall be no
    // picture that has TemporalId equal to that of the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA picture in decoding order."
    CHECK_(pcRefPic->subLayerNonReferencePictureDueToSTSA, "The active RPL part of the current picture contains a picture that is not allowed in this temporal layer due to an earlier STSA picture");
  }

  // If the current picture is an STSA picture, make all reference pictures in the DPB with temporal
  // id equal to the temproal id of the current picture sub-layer non-reference pictures. The flag
  // subLayerNonReferencePictureDueToSTSA equal to true means that the picture may not be used for
  // reference by a picture that follows the current STSA picture in decoding order
  if (getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA)
  {
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      pcRefPic = *(iterPic++);
      if (!pcRefPic->referenced || pcRefPic->getPOC() == m_iPOC)
      {
        continue;
      }

      if (pcRefPic->temporalId == m_uiTLayer)
      {
        pcRefPic->subLayerNonReferencePictureDueToSTSA = true;
      }
    }
  }
}


/** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
 * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
 * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
 * \param rcListPic reference to the reference picture list
 * This function marks the reference pictures as "unused for reference" in the following conditions.
 * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
 * are marked as "unused for reference"
 *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
 * Otherwise
 *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
 *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
 *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
 *    the bRefreshPending flag to false.
 *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
 *    reference of the current picture.
 * Note that the current picture is already placed in the reference list and its marking is not changed.
 * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
 */
void Slice::decodingRefreshMarking(int& pocCRA, bool& bRefreshPending, PicList& rcListPic, const bool bEfficientFieldIRAPEnabled)
{
  Picture* rpcPic;
  int      pocCurr = getPOC();

  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP)  // IDR picture
  {
    // mark all pictures as not used for reference
    PicList::iterator        iterPic       = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic);
      if (rpcPic->getPOC() != pocCurr)
      {
        rpcPic->referenced = false;
        rpcPic->getHashMap()->clearAll();
      }
      iterPic++;
    }
    if (bEfficientFieldIRAPEnabled)
    {
      bRefreshPending = true;
    }
  }
  else // CRA or No DR
  {
    if(bEfficientFieldIRAPEnabled && (getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
    {
      if (bRefreshPending==true && pocCurr > m_iLastIDR) // IDR reference marking pending
      {
        PicList::iterator        iterPic       = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != m_iLastIDR)
          {
            rpcPic->referenced = false;
            rpcPic->getHashMap()->clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    else
    {
      if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending
      {
        PicList::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
          {
            rpcPic->referenced = false;
            rpcPic->getHashMap()->clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
    {
      bRefreshPending = true;
      pocCRA = pocCurr;
    }
  }
}

void Slice::copySliceInfo(Slice *pSrc, bool cpyAlmostAll)
{
  CHECK_(!pSrc, "Source is NULL");

  int i, j, k;

  m_iPOC                 = pSrc->m_iPOC;
  m_eNalUnitType         = pSrc->m_eNalUnitType;
  m_eSliceType           = pSrc->m_eSliceType;
  m_iSliceQp             = pSrc->m_iSliceQp;
  m_iSliceQpBase         = pSrc->m_iSliceQpBase;
  m_ChromaQpAdjEnabled              = pSrc->m_ChromaQpAdjEnabled;
  m_deblockingFilterDisable         = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag    = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2  = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2    = pSrc->m_deblockingFilterTcOffsetDiv2;
  m_deblockingFilterCbBetaOffsetDiv2  = pSrc->m_deblockingFilterCbBetaOffsetDiv2;
  m_deblockingFilterCbTcOffsetDiv2    = pSrc->m_deblockingFilterCbTcOffsetDiv2;
  m_deblockingFilterCrBetaOffsetDiv2  = pSrc->m_deblockingFilterCrBetaOffsetDiv2;
  m_deblockingFilterCrTcOffsetDiv2    = pSrc->m_deblockingFilterCrTcOffsetDiv2;
  m_depQuantEnabledFlag               = pSrc->m_depQuantEnabledFlag;
  m_signDataHidingEnabledFlag         = pSrc->m_signDataHidingEnabledFlag;
  m_tsResidualCodingDisabledFlag      = pSrc->m_tsResidualCodingDisabledFlag;

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]     = pSrc->m_aiNumRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  }

  m_bCheckLDC             = pSrc->m_bCheckLDC;
  m_iSliceQpDelta        = pSrc->m_iSliceQpDelta;

  m_biDirPred = pSrc->m_biDirPred;
  m_symRefIdx[0] = pSrc->m_symRefIdx[0];
  m_symRefIdx[1] = pSrc->m_symRefIdx[1];

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = pSrc->m_iSliceChromaQpDelta[component];
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = pSrc->m_iSliceChromaQpDelta[JOINT_CbCr];

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_apcRefPicList[i][j]  = pSrc->m_apcRefPicList[i][j];
      m_aiRefPOCList[i][j]   = pSrc->m_aiRefPOCList[i][j];
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];
    }
    m_bIsUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_bIsUsedAsLongTerm[i][MAX_NUM_REF];
  }
  if( cpyAlmostAll ) m_iDepth = pSrc->m_iDepth;

  // access channel
  if (cpyAlmostAll) m_RPL0 = pSrc->m_RPL0;
  if (cpyAlmostAll) m_RPL1 = pSrc->m_RPL1;
  m_iLastIDR             = pSrc->m_iLastIDR;

  if( cpyAlmostAll ) m_pcPic  = pSrc->m_pcPic;

  m_pcPicHeader          = pSrc->m_pcPicHeader;
  m_colFromL0Flag        = pSrc->m_colFromL0Flag;
  m_colRefIdx            = pSrc->m_colRefIdx;

  if( cpyAlmostAll ) setLambdas(pSrc->getLambdas());

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      for (k =0; k < MAX_NUM_REF; k++)
      {
        m_abEqualRef[i][j][k] = pSrc->m_abEqualRef[i][j][k];
      }
    }
  }

  m_uiTLayer                      = pSrc->m_uiTLayer;
  m_bTLayerSwitchingFlag          = pSrc->m_bTLayerSwitchingFlag;

  m_sliceMap                      = pSrc->m_sliceMap;
  m_independentSliceIdx           = pSrc->m_independentSliceIdx;
  m_nextSlice                     = pSrc->m_nextSlice;
  m_clpRngs                       = pSrc->m_clpRngs;
  m_lmcsEnabledFlag               = pSrc->m_lmcsEnabledFlag;
  m_explicitScalingListUsed       = pSrc->m_explicitScalingListUsed;

  m_pendingRasInit                = pSrc->m_pendingRasInit;

  for ( uint32_t e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( uint32_t n=0 ; n<MAX_NUM_REF ; n++ )
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(WPScalingParam)*MAX_NUM_COMPONENT );
    }
  }

  for( uint32_t ch = 0 ; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = pSrc->m_saoEnabledFlag[ch];
  }

  m_cabacInitFlag                 = pSrc->m_cabacInitFlag;
  memcpy(m_alfApss, pSrc->m_alfApss, sizeof(m_alfApss)); // this might be quite unsafe
  memcpy( m_tileGroupAlfEnabledFlag, pSrc->m_tileGroupAlfEnabledFlag, sizeof(m_tileGroupAlfEnabledFlag));
  m_tileGroupNumAps               = pSrc->m_tileGroupNumAps;
  m_tileGroupLumaApsId            = pSrc->m_tileGroupLumaApsId;
  m_tileGroupChromaApsId          = pSrc->m_tileGroupChromaApsId;
  m_disableSATDForRd              = pSrc->m_disableSATDForRd;
  m_isLossless = pSrc->m_isLossless;

  if( cpyAlmostAll ) m_encCABACTableIdx  = pSrc->m_encCABACTableIdx;
  for( int i = 0; i < NUM_REF_PIC_LIST_01; i ++ )
  {
    for (int j = 0; j < MAX_NUM_REF_PICS; j ++ )
    {
      m_scalingRatio[i][j]          = pSrc->m_scalingRatio[i][j];
    }
  }
  m_ccAlfFilterParam                        = pSrc->m_ccAlfFilterParam;
  m_ccAlfFilterControl[0]                   = pSrc->m_ccAlfFilterControl[0];
  m_ccAlfFilterControl[1]                   = pSrc->m_ccAlfFilterControl[1];
  m_tileGroupCcAlfCbEnabledFlag             = pSrc->m_tileGroupCcAlfCbEnabledFlag;
  m_tileGroupCcAlfCrEnabledFlag             = pSrc->m_tileGroupCcAlfCrEnabledFlag;
  m_tileGroupCcAlfCbApsId                   = pSrc->m_tileGroupCcAlfCbApsId;
  m_tileGroupCcAlfCrApsId                   = pSrc->m_tileGroupCcAlfCrApsId;
}


/** Function for checking if this is a switching-point
*/
bool Slice::isTemporalLayerSwitchingPoint(PicList& rcListPic) const
{
  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    const Picture* pcPic = *(iterPic++);
    if( pcPic->referenced && pcPic->poc != getPOC())
    {
      if( pcPic->temporalId >= getTLayer())
      {
        return false;
      }
    }
  }
  return true;
}

/** Function for checking if this is a STSA candidate
 */
bool Slice::isStepwiseTemporalLayerSwitchingPointCandidate(PicList& rcListPic) const
{
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    const Picture* pcPic = *(iterPic++);
    if( pcPic->referenced && pcPic->poc != getPOC())
    {
      if( pcPic->temporalId >= getTLayer())
      {
        return false;
      }
    }
  }
  return true;
}


void Slice::checkLeadingPictureRestrictions(PicList& rcListPic, const PPS& pps) const
{
  int nalUnitType = this->getNalUnitType();

  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() > this->getPOC())
  {
    //check this only when pps_mixed_nalu_types_in_pic_flag is equal to 0
    if (pps.getMixedNaluTypesInPicFlag() == 0)
    {
      // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
      if (nalUnitType < NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
          nalUnitType > NAL_UNIT_CODED_SLICE_CRA)
      {
        CHECK_(nalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
              nalUnitType != NAL_UNIT_CODED_SLICE_RADL, "Invalid NAL unit type");
      }
    }
  }

  if (this->getAssociatedIRAPPOC() <= this->getPOC())
  {
    if (pps.getMixedNaluTypesInPicFlag() == 0)
    {
      CHECK_(nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL, "When a picture is not a leading picture, it shall not be a RADL or RASL picture.");
    }
  }

  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && !pps.getMixedNaluTypesInPicFlag())
  {
    CHECK_( this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL, "Invalid NAL unit type");
  }

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL && !pps.getMixedNaluTypesInPicFlag())
  {
    CHECK_(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP, "Invalid NAL unit type");
  }

  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  int numNonLPFound = 0;
  while ( iterPic != rcListPic.end())
  {
    Picture* pcPic = *(iterPic++);
    if( ! pcPic->reconstructed)
    {
      continue;
    }
    if( pcPic->poc == this->getPOC())
    {
      continue;
    }
    const Slice* pcSlice = pcPic->slices[0];

    if (pcSlice->getPicHeader()->getPicOutputFlag() == 1 && !this->getNoOutputOfPriorPicsFlag() && pcPic->layerId == this->m_nuhLayerId)
    {
      if ((nalUnitType == NAL_UNIT_CODED_SLICE_CRA || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) && !pps.getMixedNaluTypesInPicFlag())
      {
        CHECK_(pcPic->poc >= this->getPOC(), "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes an IRAP picture with nuh_layer_id "
              "equal to layerId in decoding order shall precede the IRAP picture in output order.");
      }
    }

    if (pcSlice->getPicHeader()->getPicOutputFlag() == 1 && pcPic->layerId == this->m_nuhLayerId)
    {
      if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
      {
        if (this->getAssociatedIRAPPOC() > pcSlice->getAssociatedIRAPPOC() && !pps.getMixedNaluTypesInPicFlag())
        {
          if (this->getAssociatedIRAPPOC() != pcPic->poc)
          {
            CHECK_(pcPic->poc >= this->getPOC(), "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes an IRAP picture with nuh_layer_id "
                  "equal to layerId in decoding order shall precede any RADL picture associated with the IRAP picture in output order.");
          }
        }
      }
    }

    if (pcSlice->getPicHeader()->getPicOutputFlag() == 1 && !this->getPicHeader()->getNoOutputBeforeRecoveryFlag() && pcPic->layerId == this->m_nuhLayerId
        && nalUnitType != NAL_UNIT_CODED_SLICE_GDR && this->getPicHeader()->getRecoveryPocCnt() != -1)
    {
      if (this->getPOC() == this->getPicHeader()->getRecoveryPocCnt() + this->getPrevGDRInSameLayerPOC())
      {
        CHECK_(pcPic->poc >= this->getPOC(), "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes a recovery point picture with "
              "nuh_layer_id equal to layerId in decoding order shall precede the recovery point picture in output order.");
      }
    }

    if ((nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL) && 
      (pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL && pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL) && !pps.getMixedNaluTypesInPicFlag())
    {
      if (pcSlice->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC() && pcPic->layerId == this->m_nuhLayerId)
      {
        numNonLPFound++;
        int limitNonLP = 0;
        if (pcSlice->getSPS()->getFieldSeqFlag())
        {
          limitNonLP = 1;
        }
        CHECK_(pcPic->poc > this->getAssociatedIRAPPOC() && numNonLPFound > limitNonLP, "If sps_field_seq_flag is equal to 0 and the current picture, with nuh_layer_id "
              "equal to a particular value layerId, is a leading picture associated with an IRAP picture, it shall precede, in decoding order, all non-leading "
              "pictures that are associated with the same IRAP picture.Otherwise, let picA and picB be the first and the last leading pictures, in decoding order, "
              "associated with an IRAP picture, respectively, there shall be at most one non-leading picture with nuh_layer_id equal to layerId preceding picA in "
              "decoding order, and there shall be no non-leading picture with nuh_layer_id equal to layerId between picA and picB in decoding order.");
      }
    }

    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && !pps.getMixedNaluTypesInPicFlag())
    {
      if ((this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA) &&
          this->getAssociatedIRAPPOC() == pcSlice->getAssociatedIRAPPOC())
      {
        if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL)
        {
          CHECK_(pcPic->poc <= this->getPOC(), "Any RASL picture associated with a CRA picture shall precede any RADL picture associated with the CRA picture in output order.");
        }
      }
    }

    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && !pps.getMixedNaluTypesInPicFlag())
    {
      if(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)
      {
        if(pcSlice->getPOC() < this->getAssociatedIRAPPOC() &&
          (
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) &&
            pcPic->layerId == this->m_nuhLayerId)
        {
          CHECK_(this->getPOC() <= pcSlice->getPOC(), "Any RASL picture, with nuh_layer_id equal to a particular value layerId, associated with a CRA picture shall follow, "
               "in output order, any IRAP or GDR picture with nuh_layer_id equal to layerId that precedes the CRA picture in decoding order.");
        }
      }
    }
  }
}

void Slice::checkSubpicTypeConstraints(PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int prevIRAPSubpicDecOrderNo)
{
  int curSubpicIdx = getPPS()->getSubPicIdxFromSubPicId(getSliceSubPicId());

  if (getPPS()->getMixedNaluTypesInPicFlag() && getSliceType() != I_SLICE)
  {
    CHECK_(!getSPS()->getSubPicTreatedAsPicFlag(curSubpicIdx), "When pps_mixed_nalu_types_in_pic_flag is equal 1, the value of sps_subpic_treated_as_pic_flag shall be equal to 1 "
          "for all the subpictures that are in the picture and contain at least one P or B slice");
  }

  int nalUnitType = getNalUnitType();
  int prevIRAPSubpicPOC = getPrevIRAPSubpicPOC();

  if (getCtuAddrInSlice(0) == getPPS()->getSubPic(curSubpicIdx).getFirstCTUInSubPic())
  {
    // subpicture type related constraints invoked only if the current slice is the first slice of a subpicture
    int prevGDRSubpicPOC = getPrevGDRSubpicPOC();
    int prevIRAPSubpicType = getPrevIRAPSubpicType();

    if (prevIRAPSubpicPOC > getPOC() && (nalUnitType < NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType > NAL_UNIT_CODED_SLICE_CRA))
    {
      CHECK_(nalUnitType != NAL_UNIT_CODED_SLICE_RASL && nalUnitType != NAL_UNIT_CODED_SLICE_RADL,
        "When a subpicture is a leading subpicture of an IRAP subpicture, it shall be a RADL or RASL subpicture");
    }

    if (prevIRAPSubpicPOC <= getPOC())
    {
      CHECK_(nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL,
        "When a subpicture is not a leading subpicture of an IRAP subpicture, it shall not be a RADL or RASL subpicture");
    }

    CHECK_(nalUnitType == NAL_UNIT_CODED_SLICE_RASL && (prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP || prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_IDR_W_RADL),
      "No RASL subpictures shall be present in the bitstream that are associated with an IDR subpicture");

    CHECK_(nalUnitType == NAL_UNIT_CODED_SLICE_RADL && prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP,
      "No RADL subpictures shall be present in the bitstream that are associated with an IDR subpicture having nal_unit_type equal to IDR_N_LP");

    //constraints related to current subpicture type and its preceding subpicture types
    PicList::iterator iterPic = rcListPic.begin();
    int numNonLeadingPic = 0;
    while (iterPic != rcListPic.end())
    {
      Picture* bufPic = *(iterPic++);
      if (!bufPic->reconstructed)
      {
        continue;
      }
      if (bufPic->poc == getPOC())
      {
        continue;
      }

      //identify the subpicture in the reference picture buffer that with nuh_layer_id equal to current subpicture layerId and subpicture index equal to current subpicIdx
      bool isBufPicOutput = false;
      int bufSubpicType = NAL_UNIT_INVALID;
      int bufSubpicPrevIRAPSubpicPOC = 0;
      for (int i = 0; i < bufPic->numSlices; i++)
      {
        if (bufPic->sliceSubpicIdx[i] == curSubpicIdx)
        {
          isBufPicOutput = bufPic->slices[i]->getPicHeader()->getPicOutputFlag();
          bufSubpicType = bufPic->slices[i]->getNalUnitType();
          bufSubpicPrevIRAPSubpicPOC = bufPic->slices[i]->getPrevIRAPSubpicPOC();
          break;
        }
      }

      if ((nalUnitType == NAL_UNIT_CODED_SLICE_CRA || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) &&
        !this->getNoOutputOfPriorPicsFlag() && isBufPicOutput == 1 && bufPic->layerId == m_nuhLayerId)
      {
        CHECK_(bufPic->poc >= getPOC(), "Any subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal to a particular value subpicIdx, that "
          "precedes, in decoding order, an IRAP subpicture with nuh_layer_id equal to layerId and subpicture index equal to subpicIdx shall precede, in output order, the "
          "IRAP subpicture");
      }

      if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL && isBufPicOutput == 1 && bufPic->layerId == m_nuhLayerId &&
        prevIRAPSubpicPOC > bufSubpicPrevIRAPSubpicPOC && prevIRAPSubpicPOC != bufPic->poc)
      {
        CHECK_(bufPic->poc >= getPOC(), "Any subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal to a particular value subpicIdx, that "
          "precedes, in decoding order, an IRAP subpicture with nuh_layer_id equal to layerId and subpicture index equal to subpicIdx shall precede, in output order, all "
          "its associated RADL subpictures");
      }

      if ((getPOC() == getPicHeader()->getRecoveryPocCnt() + prevGDRSubpicPOC) && !this->getNoOutputOfPriorPicsFlag() && isBufPicOutput == 1 &&
        bufPic->layerId == m_nuhLayerId && nalUnitType != NAL_UNIT_CODED_SLICE_GDR && getPicHeader()->getRecoveryPocCnt() != -1)
      {
        CHECK_(bufPic->poc >= getPOC(), "Any subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal to a particular value subpicIdx, that "
          "precedes, in decoding order, a subpicture with nuh_layer_id equal to layerId and subpicture index equal to subpicIdx in a recovery point picture shall precede "
          "that subpicture in the recovery point picture in output order");
      }

      if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_CRA && bufSubpicType == NAL_UNIT_CODED_SLICE_RADL &&
        prevIRAPSubpicPOC == bufSubpicPrevIRAPSubpicPOC)
      {
        CHECK_(bufPic->poc <= getPOC(), "Any RASL subpicture associated with a CRA subpicture shall precede any RADL subpicture associated with the CRA subpicture in output order");
      }

      if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_CRA && bufPic->layerId == m_nuhLayerId && bufPic->poc < prevIRAPSubpicPOC)
      {
        if (bufSubpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP || bufSubpicType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
          bufSubpicType == NAL_UNIT_CODED_SLICE_CRA || bufSubpicType == NAL_UNIT_CODED_SLICE_GDR)
        {
          CHECK_(bufPic->poc >= getPOC(), "Any RASL subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal to a particular value subpicIdx, "
            "associated with a CRA subpicture shall follow, in output order, any IRAP or GDR subpicture , with nuh_layer_id equal to layerId and subpicture index equal to "
            "subpicIdx, that precedes the CRA subpicture in decoding order");
        }
      }

      if ((nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL) &&
        bufSubpicType != NAL_UNIT_CODED_SLICE_RASL && bufSubpicType != NAL_UNIT_CODED_SLICE_RADL &&
        bufSubpicPrevIRAPSubpicPOC == prevIRAPSubpicPOC && bufPic->layerId == m_nuhLayerId)
      {
        numNonLeadingPic++;
        int th = bufPic->cs->sps->getFieldSeqFlag() ? 1 : 0;
        CHECK_(bufPic->poc > prevIRAPSubpicPOC && numNonLeadingPic > th, "If sps_field_seq_flag is equal to 0 and the current subpicture, with nuh_layer_id equal to a particular value "
          "layerId and subpicture index equal to a particular value subpicIdx, is a leading subpicture associated with an IRAP subpicture, it shall precede, in decoding order, "
          "all non-leading subpictures that are associated with the same IRAP subpicture. Otherwise, let subpicA and subpicB be the first and the last leading subpictures, in "
          "decoding order, associated with an IRAP subpicture, respectively, there shall be at most one non-leading subpicture with nuh_layer_id equal to layerId and subpicture "
          "index equal to subpicIdx preceding subpicA in decoding order, and there shall be no non-leading picture with nuh_layer_id equal to layerId and subpicture index equal "
          "to subpicIdx between picA and picB in decoding order");
      }
    }
  }

  //subpic RPL related constraints
  Picture* pcRefPic;
  int refPicPOC;
  int refPicDecodingOrderNumber;

  int numEntriesL0 = pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures() + pRPL0->getNumberOfInterLayerPictures();
  int numEntriesL1 = pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures() + pRPL1->getNumberOfInterLayerPictures();

  int numActiveEntriesL0 = getNumRefIdx(REF_PIC_LIST_0);
  int numActiveEntriesL1 = getNumRefIdx(REF_PIC_LIST_1);

  int layerIdx = m_pcPic->cs->vps == nullptr ? 0 : m_pcPic->cs->vps->getGeneralLayerIdx(m_pcPic->layerId);
  for (int i = 0; i < numEntriesL0; i++)
  {
    if (pRPL0->isInterLayerRefPic(i))
    {
      int refLayerId = m_pcPic->cs->vps->getLayerId(m_pcPic->cs->vps->getDirectRefLayerIdx(layerIdx, pRPL0->getInterLayerRefPicIdx(i)));
      pcRefPic = xGetRefPic(rcListPic, getPOC(), refLayerId);
      refPicPOC = pcRefPic->getPOC();
    }
    else if (!pRPL0->isRefPicLongterm(i))
    {
      refPicPOC = getPOC() + pRPL0->getRefPicIdentifier(i);
      pcRefPic = xGetRefPic(rcListPic, refPicPOC, m_pcPic->layerId);
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = pRPL0->getRefPicIdentifier(i) & pocMask;
      if(pRPL0->getDeltaPocMSBPresentFlag(i))
      {
        ltrpPoc += getPOC() - pRPL0->getDeltaPocMSBCycleLT(i) * (pocMask + 1) - (getPOC() & pocMask);
      }
      pcRefPic = xGetLongTermRefPic(rcListPic, ltrpPoc, pRPL0->getDeltaPocMSBPresentFlag(i), m_pcPic->layerId);
      refPicPOC = pcRefPic->getPOC();
    }

    if (pcRefPic) // the checks are for all reference picture, but we may not have an inactive reference picture, if starting with a CRA
    {
      refPicDecodingOrderNumber = pcRefPic->getDecodingOrderNumber();

      if (nalUnitType == NAL_UNIT_CODED_SLICE_CRA || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)
      {
        CHECK_(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture, with nuh_layer_id equal to a particular value "
              "layerId and subpicture index equal to a particular value subpicIdx, is an IRAP subpicture, there shall be no picture referred to by an entry in RefPicList[0] that "
              "precedes, in output order or decoding order,any preceding picture, in decoding order (when present), containing an IRAP subpicture with nuh_layer_id equal to "
              "layerId and subpicture index equal to subpicIdx");
      }

      if (prevIRAPSubpicPOC < getPOC() && !getSPS()->getFieldSeqFlag())
      {
        CHECK_(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture follows an IRAP subpicture having the same value "
              "of nuh_layer_id and the same value of subpicture index in both decoding and output order, there shall be no picture referred to by an active entry in RefPicList[ 0 ] "
              "that precedes the picture containing that IRAP subpicture in output order or decoding order");
      }

      if (i < numActiveEntriesL0)
      {
        if (prevIRAPSubpicPOC < getPOC())
        {
          CHECK_(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture follows an IRAP subpicture having the same value "
                "of nuh_layer_id and the same value of subpicture index and the leading subpictures, if any, associated with that IRAP subpicture in both decoding and output order, "
                "there shall be no picture referred to by an entry in RefPicList[ 0 ] that precedes the picture containing that IRAP subpicture in output order or decoding order");
        }

        if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
        {
          CHECK_(refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal "
                "to a particular value subpicIdx, is a RADL subpicture, there shall be no active entry in RefPicList[ 0 ] that is a picture that precedes the picture containing the"
                "associated IRAP subpicture in decoding order");

          if (pcRefPic->layerId == m_nuhLayerId)
          {
            for (int i = 0; i < pcRefPic->numSlices; i++)
            {
              if (pcRefPic->sliceSubpicIdx[i] == curSubpicIdx)
              {
                CHECK_(pcRefPic->slices[i]->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL, "When the current subpicture, with nuh_layer_id equal to a particular value layerId and "
                      "subpicture index equal to a particular value subpicIdx, is a RADL subpicture, there shall be no active entry in RefPicList[ 0 ] that is a picture with "
                      "nuh_layer_id equal to layerId containing a RASL subpicture with subpicture index equal to subpicIdx");
              }
            }
          }
        }
      }
    }
  }

  for (int i = 0; i < numEntriesL1; i++)
  {
    if (pRPL1->isInterLayerRefPic(i))
    {
      int refLayerId = m_pcPic->cs->vps->getLayerId(m_pcPic->cs->vps->getDirectRefLayerIdx(layerIdx, pRPL1->getInterLayerRefPicIdx(i)));
      pcRefPic = xGetRefPic(rcListPic, getPOC(), refLayerId);
      refPicPOC = pcRefPic->getPOC();
    }
    else if (!pRPL1->isRefPicLongterm(i))
    {
      refPicPOC = getPOC() + pRPL1->getRefPicIdentifier(i);
      pcRefPic = xGetRefPic(rcListPic, refPicPOC, m_pcPic->layerId);
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = pRPL1->getRefPicIdentifier(i) & pocMask;
      if (pRPL1->getDeltaPocMSBPresentFlag(i))
      {
        ltrpPoc += getPOC() - pRPL1->getDeltaPocMSBCycleLT(i) * (pocMask + 1) - (getPOC() & pocMask);
      }
      pcRefPic = xGetLongTermRefPic(rcListPic, ltrpPoc, pRPL1->getDeltaPocMSBPresentFlag(i), m_pcPic->layerId);
      refPicPOC = pcRefPic->getPOC();
    }
    if (pcRefPic) // the checks are for all reference picture, but we may not have an inactive reference picture, if starting with a CRA
    {
      refPicDecodingOrderNumber = pcRefPic->getDecodingOrderNumber();

      if (nalUnitType == NAL_UNIT_CODED_SLICE_CRA || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)
      {
        CHECK_(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture, with nuh_layer_id equal to a particular value"
              "layerId and subpicture index equal to a particular value subpicIdx, is an IRAP subpicture, there shall be no picture referred to by an entry in RefPicList[1] that"
              "precedes, in output order or decoding order,any preceding picture, in decoding order (when present), containing an IRAP subpicture with nuh_layer_id equal to "
              "layerId and subpicture index equal to subpicIdx");
      }

      if (prevIRAPSubpicPOC < getPOC() && !getSPS()->getFieldSeqFlag())
      {
        CHECK_(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture follows an IRAP subpicture having the same value "
              "of nuh_layer_id and the same value of subpicture index in both decoding and output order, there shall be no picture referred to by an active entry in RefPicList[ 1 ] "
              "that precedes the picture containing that IRAP subpicture in output order or decoding order");
      }
      if (i < numActiveEntriesL1)
      {
        if (prevIRAPSubpicPOC < getPOC())
        {
          CHECK_(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture follows an IRAP subpicture having the same value "
                "of nuh_layer_id and the same value of subpicture index and the leading subpictures, if any, associated with that IRAP subpicture in both decoding and output order, "
                "there shall be no picture referred to by an entry in RefPicList[ 1 ] that precedes the picture containing that IRAP subpicture in output order or decoding order");
        }

        if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
        {
          CHECK_(refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo, "When the current subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal "
                "to a particular value subpicIdx, is a RADL subpicture, there shall be no active entry in RefPicList[ 1 ] that is a picture that precedes the picture containing the"
                "associated IRAP subpicture in decoding order");

          if (pcRefPic->layerId == m_nuhLayerId)
          {
            for (int i = 0; i < pcRefPic->numSlices; i++)
            {
              if (pcRefPic->sliceSubpicIdx[i] == curSubpicIdx)
              {
                CHECK_(pcRefPic->slices[i]->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL, "When the current subpicture, with nuh_layer_id equal to a particular value layerId and "
                      "subpicture index equal to a particular value subpicIdx, is a RADL subpicture, there shall be no active entry in RefPicList[ 1 ] that is a picture with "
                      "nuh_layer_id equal to layerId containing a RASL subpicture with subpicture index equal to subpicIdx");
              }
            }
          }
        }
      }
    }
  }
}


//Function for applying picture marking based on the Reference Picture List
void Slice::applyReferencePictureListBasedMarking( PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, const int layerId, const PPS& pps ) const
{
  int i, isReference;
  checkLeadingPictureRestrictions(rcListPic, pps);

  bool isNeedToCheck = (this->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL) ? false : true;

  // mark long-term reference pictures in List0
  for( i = 0; i < pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures() + pRPL0->getNumberOfInterLayerPictures(); i++ )
  {
    if( !pRPL0->isRefPicLongterm( i ) || pRPL0->isInterLayerRefPic( i ) )
    {
      continue;
    }

    int isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      Picture* rpcPic = *(iterPic++);
      if (!rpcPic->referenced)
      {
        continue;
      }
      int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
      int curPoc = rpcPic->getPOC();
      int refPoc = pRPL0->getRefPicIdentifier(i) & (pocCycle - 1);
      if(pRPL0->getDeltaPocMSBPresentFlag(i))
      {
        refPoc += getPOC() - pRPL0->getDeltaPocMSBCycleLT(i) * pocCycle - (getPOC() & (pocCycle - 1));
      }
      else
      {
        curPoc = curPoc & (pocCycle - 1);
      }
      if (rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        Picture* rpcPic = *(iterPic++);
        if (!rpcPic->referenced)
        {
          continue;
        }
        int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
        int curPoc = rpcPic->getPOC();
        int refPoc = pRPL0->getRefPicIdentifier(i) & (pocCycle - 1);
        if(pRPL0->getDeltaPocMSBPresentFlag(i))
        {
          refPoc += getPOC() - pRPL0->getDeltaPocMSBCycleLT(i) * pocCycle - (getPOC() & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (!rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
        {
          isAvailable = 1;
          rpcPic->longTerm = true;
          break;
        }
      }
    }
  }

  // mark long-term reference pictures in List1
  for( i = 0; i < pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures() + pRPL1->getNumberOfInterLayerPictures(); i++ )
  {
    if( !pRPL1->isRefPicLongterm( i ) || pRPL1->isInterLayerRefPic( i ) )
    {
      continue;
    }

    int isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      Picture* rpcPic = *(iterPic++);
      if (!rpcPic->referenced)
      {
        continue;
      }
      int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
      int curPoc = rpcPic->getPOC();
      int refPoc = pRPL1->getRefPicIdentifier(i) & (pocCycle - 1);
      if(pRPL1->getDeltaPocMSBPresentFlag(i))
      {
        refPoc += getPOC() - pRPL1->getDeltaPocMSBCycleLT(i) * pocCycle - (getPOC() & (pocCycle - 1));
      }
      else
      {
        curPoc = curPoc & (pocCycle - 1);
      }
      if (rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        Picture* rpcPic = *(iterPic++);
        if (!rpcPic->referenced)
        {
          continue;
        }
        int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
        int curPoc = rpcPic->getPOC();
        int refPoc = pRPL1->getRefPicIdentifier(i) & (pocCycle - 1);
        if(pRPL1->getDeltaPocMSBPresentFlag(i))
        {
          refPoc += getPOC() - pRPL1->getDeltaPocMSBCycleLT(i) * pocCycle - (getPOC() & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (!rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
        {
          isAvailable = 1;
          rpcPic->longTerm = true;
          break;
        }
      }
    }
  }

  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    Picture* pcPic = *(iterPic++);

    if (!pcPic->referenced)
      continue;

    isReference = 0;
    // loop through all pictures in the Reference Picture Set
    // to see if the picture should be kept as reference picture
    for( i = 0; isNeedToCheck && !isReference && i < pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures() + pRPL0->getNumberOfInterLayerPictures(); i++ )
    {
      if( pRPL0->isInterLayerRefPic( i ) )
      {
        // Diagonal inter-layer prediction is not allowed
        CHECK_( pRPL0->getRefPicIdentifier( i ), "ILRP identifier should be 0" );

        if( pcPic->poc == m_iPOC )
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
      else if (pcPic->layerId == layerId)
      {
      if (!(pRPL0->isRefPicLongterm(i)))
      {
        if (pcPic->poc == this->getPOC() + pRPL0->getRefPicIdentifier(i))
        {
          isReference = 1;
          pcPic->longTerm = false;
        }
      }
      else
      {
        int pocCycle = 1 << (pcPic->cs->sps->getBitsForPOC());
        int curPoc = pcPic->poc;
        int refPoc = pRPL0->getRefPicIdentifier(i) & (pocCycle - 1);
        if(pRPL0->getDeltaPocMSBPresentFlag(i))
        {
          refPoc += getPOC() - pRPL0->getDeltaPocMSBCycleLT(i) * pocCycle - (getPOC() & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (pcPic->longTerm && curPoc == refPoc)
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
      }
    }

    for( i = 0; isNeedToCheck && !isReference && i < pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures() + pRPL1->getNumberOfInterLayerPictures(); i++ )
    {
      if( pRPL1->isInterLayerRefPic( i ) )
      {
        // Diagonal inter-layer prediction is not allowed
        CHECK_( pRPL1->getRefPicIdentifier( i ), "ILRP identifier should be 0" );

        if( pcPic->poc == m_iPOC )
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
      else if( pcPic->layerId == layerId )
      {
      if (!(pRPL1->isRefPicLongterm(i)))
      {
        if (pcPic->poc == this->getPOC() + pRPL1->getRefPicIdentifier(i))
        {
          isReference = 1;
          pcPic->longTerm = false;
        }
      }
      else
      {
        int pocCycle = 1 << (pcPic->cs->sps->getBitsForPOC());
        int curPoc = pcPic->poc;
        int refPoc = pRPL1->getRefPicIdentifier(i) & (pocCycle - 1);
        if(pRPL1->getDeltaPocMSBPresentFlag(i))
        {
          refPoc += getPOC() - pRPL1->getDeltaPocMSBCycleLT(i) * pocCycle - (getPOC() & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (pcPic->longTerm && curPoc == refPoc)
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
      }
    }
    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture List
    if( pcPic->layerId == layerId && pcPic->poc != m_iPOC && isReference == 0 )
    {
      pcPic->referenced = false;
      pcPic->longTerm = false;
    }

    // sanity checks
    if (pcPic->referenced)
    {
      //check that pictures of higher temporal layers are not used
      CHECK_(pcPic->usedByCurr && !(pcPic->temporalId <= this->getTLayer()), "Invalid state");
    }
  }
}

int Slice::checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx, bool printErrors) const
{
  Picture* rpcPic;
  int isAvailable = 0;
  int notPresentPoc = 0;

  if (this->isIDRorBLA()) return 0; //Assume that all pic in the DPB will be flushed anyway so no need to check.

  int numberOfPictures = pRPL->getNumberOfLongtermPictures() + pRPL->getNumberOfShorttermPictures() + pRPL->getNumberOfInterLayerPictures();
  //Check long term ref pics
  for (int ii = 0; pRPL->getNumberOfLongtermPictures() > 0 && ii < numberOfPictures; ii++)
  {
    if( !pRPL->isRefPicLongterm( ii ) || pRPL->isInterLayerRefPic( ii ) )
    {
      continue;
    }

    notPresentPoc = pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
      int curPoc = rpcPic->getPOC();
      int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
      if(pRPL->getDeltaPocMSBPresentFlag(ii))
      {
        refPoc += getPOC() - pRPL->getDeltaPocMSBCycleLT(ii) * pocCycle - (getPOC() & (pocCycle - 1));
      }
      else
      {
        curPoc = curPoc & (pocCycle - 1);
      }
      if (rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);
        int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
        int curPoc = rpcPic->getPOC();
        int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
        if(pRPL->getDeltaPocMSBPresentFlag(ii))
        {
          refPoc += getPOC() - pRPL->getDeltaPocMSBCycleLT(ii) * pocCycle - (getPOC() & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (!rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
        {
          isAvailable = 1;
          rpcPic->longTerm = true;
          break;
        }
      }
    }
    if (!isAvailable)
    {
      if (printErrors)
      {
        msg(ERROR_, "\nCurrent picture: %d Long-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      return notPresentPoc;
    }
  }
  //report that a picture is lost if it is in the Reference Picture List but not in the DPB

  isAvailable = 0;
  //Check short term ref pics
  for (int ii = 0; ii < numberOfPictures; ii++)
  {
    if (pRPL->isRefPicLongterm(ii))
      continue;

    notPresentPoc = this->getPOC() + pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if (!rpcPic->longTerm && rpcPic->getPOC() == this->getPOC() + pRPL->getRefPicIdentifier(ii) && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    //report that a picture is lost if it is in the Reference Picture List but not in the DPB
    if (isAvailable == 0 && pRPL->getNumberOfShorttermPictures() > 0)
    {
      if (printErrors)
      {
        msg(ERROR_, "\nCurrent picture: %d Short-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      return notPresentPoc;
    }
  }
  return 0;
}

int Slice::checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx, bool printErrors, int *refPicIndex, int numActiveRefPics) const
{
  Picture* rpcPic;
  int isAvailable = 0;
  int notPresentPoc = 0;
  *refPicIndex = 0;

  if (this->isIDRorBLA()) return 0; //Assume that all pic in the DPB will be flushed anyway so no need to check.

  int numberOfPictures = numActiveRefPics;
  //Check long term ref pics
  for (int ii = 0; pRPL->getNumberOfLongtermPictures() > 0 && ii < numberOfPictures; ii++)
  {
    if( !pRPL->isRefPicLongterm( ii ) || pRPL->isInterLayerRefPic( ii ) )
    {
      continue;
    }

    notPresentPoc = pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
      int curPoc = rpcPic->getPOC();
      int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
      if(pRPL->getDeltaPocMSBPresentFlag(ii))
      {
        refPoc += getPOC() - pRPL->getDeltaPocMSBCycleLT(ii) * pocCycle - (getPOC() & (pocCycle - 1));
      }
      else
      {
        curPoc = curPoc & (pocCycle - 1);
      }
      if (rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);
        int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
        int curPoc = rpcPic->getPOC();
        int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
        if(pRPL->getDeltaPocMSBPresentFlag(ii))
        {
          refPoc += getPOC() - pRPL->getDeltaPocMSBCycleLT(ii) * pocCycle - (getPOC() & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (!rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
        {
          isAvailable = 1;
          rpcPic->longTerm = true;
          break;
        }
      }
    }
    if (!isAvailable)
    {
      if (printErrors)
      {
        msg(ERROR_, "\nCurrent picture: %d Long-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      *refPicIndex = ii;
      return notPresentPoc;
    }
  }
  //report that a picture is lost if it is in the Reference Picture List but not in the DPB

  isAvailable = 0;
  //Check short term ref pics
  for (int ii = 0; ii < numberOfPictures; ii++)
  {
    if (pRPL->isRefPicLongterm(ii))
      continue;

    notPresentPoc = this->getPOC() + pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if (!rpcPic->longTerm && rpcPic->getPOC() == this->getPOC() + pRPL->getRefPicIdentifier(ii) && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    //report that a picture is lost if it is in the Reference Picture List but not in the DPB
    if (isAvailable == 0 && pRPL->getNumberOfShorttermPictures() > 0)
    {
      if (printErrors)
      {
        msg(ERROR_, "\nCurrent picture: %d Short-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      *refPicIndex = ii;
      return notPresentPoc;
    }
  }
  return 0;
}

bool Slice::isPOCInRefPicList(const ReferencePictureList *rpl, int poc )
{
  for( int i = 0; i < rpl->getNumberOfLongtermPictures() + rpl->getNumberOfShorttermPictures() + rpl->getNumberOfInterLayerPictures(); i++ )
  {
    if( rpl->isInterLayerRefPic( i ) )
    {
      // Diagonal inter-layer prediction is not allowed
      CHECK_( rpl->getRefPicIdentifier( i ), "ILRP identifier should be 0" );

      if( poc == m_iPOC )
      {
        return true;
      }
    }
    else
    if (rpl->isRefPicLongterm(i))
    {
      if (poc == rpl->getRefPicIdentifier(i))
      {
        return true;
      }
    }
    else
    {
      if (poc == getPOC() + rpl->getRefPicIdentifier(i))
      {
        return true;
      }
    }
  }
  return false;
}

bool Slice::isPocRestrictedByDRAP( int poc, bool precedingDRAPInDecodingOrder )
{
  if (!getEnableDRAPSEI())
  {
    return false;
  }
  return ( isDRAP() && poc != getAssociatedIRAPPOC() ) ||
         ( cvsHasPreviousDRAP() && getPOC() > getLatestDRAPPOC() && (precedingDRAPInDecodingOrder || poc < getLatestDRAPPOC()) );
}

void Slice::checkConformanceForDRAP( uint32_t temporalId )
{
  if (!(isDRAP() || cvsHasPreviousDRAP()))
  {
    return;
  }

  if (isDRAP())
  {
    if (!(getNalUnitType() == NalUnitType::NAL_UNIT_CODED_SLICE_TRAIL ||
          getNalUnitType() == NalUnitType::NAL_UNIT_CODED_SLICE_STSA))
    {
      msg(WARNING_, "Warning, non-conforming bitstream. The DRAP picture should be a trailing picture.\n");
    }
    if ( temporalId != 0)
    {
      msg(
        WARNING_, "Warning, non-conforming bitstream. The DRAP picture shall have a temporal sublayer identifier equal to 0.\n");
    }
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_0); i++)
    {
      if (getRefPic(REF_PIC_LIST_0,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg(WARNING_, "Warning, non-conforming bitstream. The DRAP picture shall not include any pictures in the active "
                      "entries of its reference picture lists except the preceding IRAP picture in decoding order.\n");
      }
    }
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_1); i++)
    {
      if (getRefPic(REF_PIC_LIST_1,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg(WARNING_, "Warning, non-conforming bitstream. The DRAP picture shall not include any pictures in the active "
                      "entries of its reference picture lists except the preceding IRAP picture in decoding order.\n");
      }
    }
  }

  if (cvsHasPreviousDRAP() && getPOC() > getLatestDRAPPOC())
  {
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_0); i++)
    {
      if (getRefPic(REF_PIC_LIST_0,i)->getPOC() < getLatestDRAPPOC() && getRefPic(REF_PIC_LIST_0,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg(WARNING_, "Warning, non-conforming bitstream. Any picture that follows the DRAP picture in both decoding order "
                    "and output order shall not include, in the active entries of its reference picture lists, any picture "
                    "that precedes the DRAP picture in decoding order or output order, with the exception of the preceding "
                    "IRAP picture in decoding order. Problem is POC %d in RPL0.\n", getRefPic(REF_PIC_LIST_0,i)->getPOC());
      }
    }
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_1); i++)
    {
      if (getRefPic(REF_PIC_LIST_1,i)->getPOC() < getLatestDRAPPOC() && getRefPic(REF_PIC_LIST_1,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg(WARNING_, "Warning, non-conforming bitstream. Any picture that follows the DRAP picture in both decoding order "
                    "and output order shall not include, in the active entries of its reference picture lists, any picture "
                    "that precedes the DRAP picture in decoding order or output order, with the exception of the preceding "
                    "IRAP picture in decoding order. Problem is POC %d in RPL1", getRefPic(REF_PIC_LIST_1,i)->getPOC());
      }
    }
  }
}


//! get AC and DC values for weighted pred
void  Slice::getWpAcDcParam(const WPACDCParam *&wp) const
{
  wp = m_weightACDCParam;
}

//! init AC and DC values for weighted pred
void  Slice::initWpAcDcParam()
{
  for(int iComp = 0; iComp < MAX_NUM_COMPONENT; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}

//! get tables for weighted prediction
const WPScalingParam *Slice::getWpScaling(const RefPicList refPicList, const int refIdx) const
{
  CHECK_(refPicList >= NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  if (refIdx < 0)
  {
    return nullptr;
  }
  else
  {
    return m_weightPredTable[refPicList][refIdx];
  }
}

WPScalingParam *Slice::getWpScaling(const RefPicList refPicList, const int refIdx)
{
  CHECK_(refPicList >= NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  if (refIdx < 0)
  {
    return nullptr;
  }
  else
  {
    return m_weightPredTable[refPicList][refIdx];
  }
}

//! reset Default WP tables settings : no weight.
void  Slice::resetWpScaling()
{
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->presentFlag     = false;
        pwp->log2WeightDenom = 0;
        pwp->log2WeightDenom = 0;
        pwp->codedWeight     = 1;
        pwp->codedOffset     = 0;
      }
    }
  }
}

//! init WP table
void  Slice::initWpScaling(const SPS *sps)
{
  const bool bUseHighPrecisionPredictionWeighting = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        if (!pwp->presentFlag)
        {
          // Inferring values not present :
          pwp->codedWeight = (1 << pwp->log2WeightDenom);
          pwp->codedOffset = 0;
        }

        const int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (sps->getBitDepth(toChannelType(ComponentID(yuv)))-8));

        pwp->w = pwp->codedWeight;
        pwp->o = pwp->codedOffset * offsetScalingFactor;   // NOTE: This value of the ".o" variable is never used - .o
                                                           // is set immediately before it gets used
        pwp->shift = pwp->log2WeightDenom;
        pwp->round = (pwp->log2WeightDenom >= 1) ? (1 << (pwp->log2WeightDenom - 1)) : (0);
      }
    }
  }
}


void Slice::startProcessingTimer()
{
  m_iProcessingStartTime = clock();
}

void Slice::stopProcessingTimer()
{
  m_dProcessingTime += (double)(clock()-m_iProcessingStartTime) / CLOCKS_PER_SEC;
  m_iProcessingStartTime = 0;
}

unsigned Slice::getMinPictureDistance() const
{
  int minPicDist = MAX_INT;
  if (getSPS()->getIBCFlag())
  {
    minPicDist = 0;
  }
  else
  if( ! isIntra() )
  {
    const int currPOC  = getPOC();
    for (int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_0); refIdx++)
    {
      minPicDist = std::min( minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_0, refIdx)->getPOC()));
    }
    if( getSliceType() == B_SLICE )
    {
      for (int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_1); refIdx++)
      {
        minPicDist = std::min(minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_1, refIdx)->getPOC()));
      }
    }
  }
  return (unsigned) minPicDist;
}

// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
VPS::VPS()
  : m_VPSId(0)
  , m_maxLayers(1)
  , m_vpsMaxSubLayers(7)
  , m_vpsDefaultPtlDpbHrdMaxTidFlag (true)
  , m_vpsAllIndependentLayersFlag(true)
  , m_vpsEachLayerIsAnOlsFlag (1)
  , m_vpsOlsModeIdc (0)
  , m_vpsNumOutputLayerSets (1)
  , m_vpsNumPtls (1)
  , m_vpsExtensionFlag(false)
  , m_vpsGeneralHrdParamsPresentFlag(false)
  , m_vpsSublayerCpbParamsPresentFlag(false)
  , m_numOlsTimingHrdParamsMinus1(0)
  , m_totalNumOLSs( 1 )
  , m_numMultiLayeredOlss( 0 )
  , m_numDpbParams( 0 )
  , m_sublayerDpbParamsPresentFlag( false )
  , m_targetOlsIdx( 0 )
{
  for (int i = 0; i < MAX_VPS_SUBLAYERS; i++)
  {
    m_vpsCfgPredDirection[i] = 0;
  }
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_vpsLayerId[i] = 0;
    m_vpsIndependentLayerFlag[i] = true;
#if !JVET_R0193
    m_vpsMaxTidIlRefPicsPlus1[i] = 7;
#endif
    m_generalLayerIdx[i] = 0;
    for (int j = 0; j < MAX_VPS_LAYERS; j++)
    {
      m_vpsDirectRefLayerFlag[i][j] = 0;
      m_directRefLayerIdx[i][j] = MAX_VPS_LAYERS;
      m_interLayerRefIdx[i][i] = NOT_VALID;
    }
  }
  for (int i = 0; i < MAX_NUM_OLSS; i++)
  {
    for (int j = 0; j < MAX_VPS_LAYERS; j++)
    {
      m_vpsOlsOutputLayerFlag[i][j] = 0;
    }
    if (i == 0)
    {
      m_ptPresentFlag[i] = 1;
    }
    else
    {
      m_ptPresentFlag[i] = 0;
    }
    m_ptlMaxTemporalId[i] = m_vpsMaxSubLayers - 1;
    m_olsPtlIdx[i] = 0;
    m_hrdMaxTid[i] = m_vpsMaxSubLayers - 1;
    m_olsTimingHrdIdx[i] = 0;
  }
}

VPS::~VPS()
{
}

void VPS::deriveOutputLayerSets()
{
  if( m_vpsEachLayerIsAnOlsFlag || m_vpsOlsModeIdc < 2 )
  {
    m_totalNumOLSs = m_maxLayers;
  }
  else if( m_vpsOlsModeIdc == 2 )
  {
    m_totalNumOLSs = m_vpsNumOutputLayerSets;
  }

  m_olsDpbParamsIdx.resize( m_totalNumOLSs );
  m_olsDpbPicSize.resize( m_totalNumOLSs, Size(0, 0) );
  m_numOutputLayersInOls.resize( m_totalNumOLSs );
  m_numLayersInOls.resize( m_totalNumOLSs );
  m_outputLayerIdInOls.resize( m_totalNumOLSs, std::vector<int>( m_maxLayers, NOT_VALID ) );
  m_numSubLayersInLayerInOLS.resize( m_totalNumOLSs, std::vector<int>( m_maxLayers, NOT_VALID ) );
  m_layerIdInOls.resize( m_totalNumOLSs, std::vector<int>( m_maxLayers, NOT_VALID ) );
  m_olsDpbChromaFormatIdc.resize(m_totalNumOLSs);
  m_olsDpbBitDepthMinus8.resize(m_totalNumOLSs);

  std::vector<int> numRefLayers( m_maxLayers );
  std::vector<std::vector<int>> outputLayerIdx( m_totalNumOLSs, std::vector<int>( m_maxLayers, NOT_VALID ) );
  std::vector<std::vector<int>> layerIncludedInOlsFlag( m_totalNumOLSs, std::vector<int>( m_maxLayers, 0 ) );
  std::vector<std::vector<int>> dependencyFlag( m_maxLayers, std::vector<int>( m_maxLayers, NOT_VALID ) );
  std::vector<std::vector<int>> refLayerIdx( m_maxLayers, std::vector<int>( m_maxLayers, NOT_VALID ) );
  std::vector<int> layerUsedAsRefLayerFlag( m_maxLayers, 0 );
  std::vector<int> layerUsedAsOutputLayerFlag( m_maxLayers, NOT_VALID );

  for( int i = 0; i < m_maxLayers; i++ )
  {
    int r = 0;

    for( int j = 0; j < m_maxLayers; j++ )
    {
      dependencyFlag[i][j] = m_vpsDirectRefLayerFlag[i][j];

      for( int k = 0; k < i; k++ )
      {
        if( m_vpsDirectRefLayerFlag[i][k] && dependencyFlag[k][j] )
        {
          dependencyFlag[i][j] = 1;
        }
      }
      if (m_vpsDirectRefLayerFlag[i][j])
      {
        layerUsedAsRefLayerFlag[j] = 1;
      }

      if( dependencyFlag[i][j] )
      {
        refLayerIdx[i][r++] = j;
      }
    }

    numRefLayers[i] = r;
  }

  m_numOutputLayersInOls[0] = 1;
  m_outputLayerIdInOls[0][0] = m_vpsLayerId[0];
#if JVET_R0193_S0141
  m_numSubLayersInLayerInOLS[0][0] = m_ptlMaxTemporalId[m_olsPtlIdx[0]] + 1;
#else
  m_numSubLayersInLayerInOLS[0][0] = m_vpsMaxSubLayers;
#endif
  layerUsedAsOutputLayerFlag[0] = 1;
  for (int i = 1; i < m_maxLayers; i++)
  {
    if (m_vpsEachLayerIsAnOlsFlag || m_vpsOlsModeIdc < 2)
    {
      layerUsedAsOutputLayerFlag[i] = 1;
    }
    else
    {
      layerUsedAsOutputLayerFlag[i] = 0;
    }
  }
  for( int i = 1; i < m_totalNumOLSs; i++ )
  {
    if( m_vpsEachLayerIsAnOlsFlag || m_vpsOlsModeIdc == 0 )
    {
      m_numOutputLayersInOls[i] = 1;
      m_outputLayerIdInOls[i][0] = m_vpsLayerId[i];
#if JVET_R0193_S0141
      if (m_vpsEachLayerIsAnOlsFlag)
      {
        m_numSubLayersInLayerInOLS[i][0] = m_ptlMaxTemporalId[m_olsPtlIdx[i]] + 1;
      }
      else
      {
        m_numSubLayersInLayerInOLS[i][i] = m_ptlMaxTemporalId[m_olsPtlIdx[i]] + 1;
        for (int k = i - 1; k >= 0; k--)
        {
          m_numSubLayersInLayerInOLS[i][k] = 0;
          for (int m = k + 1; m <= i; m++)
          {
            uint32_t maxSublayerNeeded = std::min((uint32_t)m_numSubLayersInLayerInOLS[i][m], m_vpsMaxTidIlRefPicsPlus1[m][k]);
            if (m_vpsDirectRefLayerFlag[m][k] && m_numSubLayersInLayerInOLS[i][k] < maxSublayerNeeded)
            {
              m_numSubLayersInLayerInOLS[i][k] = maxSublayerNeeded;
            }
          }
      }
      }
#else
      for(int  j = 0; j < i  &&  ( m_vpsOlsModeIdc  ==  0 ); j++ )
      {
        m_numSubLayersInLayerInOLS[i][j] = m_vpsMaxTidIlRefPicsPlus1[i];
      }
      m_numSubLayersInLayerInOLS[i][i] = m_vpsMaxSubLayers;
#endif
    }
    else if( m_vpsOlsModeIdc == 1 )
    {
      m_numOutputLayersInOls[i] = i + 1;

      for( int j = 0; j < m_numOutputLayersInOls[i]; j++ )
      {
        m_outputLayerIdInOls[i][j] = m_vpsLayerId[j];
#if JVET_R0193_S0141
        m_numSubLayersInLayerInOLS[i][j] = m_ptlMaxTemporalId[m_olsPtlIdx[i]] + 1;
#else
        m_numSubLayersInLayerInOLS[i][j] = m_vpsMaxSubLayers;
#endif
      }
    }
    else if( m_vpsOlsModeIdc == 2 )
    {
      int j = 0;
#if JVET_R0193
      int highestIncludedLayer = 0;
#endif
      for( j = 0; j  <  m_maxLayers; j++ )
      {
        m_numSubLayersInLayerInOLS[i][j] = 0;
      }
      j = 0;
      for( int k = 0; k < m_maxLayers; k++ )
      {
        if( m_vpsOlsOutputLayerFlag[i][k] )
        {
          layerIncludedInOlsFlag[i][k] = 1;
#if JVET_R0193
          highestIncludedLayer = k;
#endif
          layerUsedAsOutputLayerFlag[k] = 1;
          outputLayerIdx[i][j] = k;
          m_outputLayerIdInOls[i][j++] = m_vpsLayerId[k];
#if JVET_R0193_S0141
          m_numSubLayersInLayerInOLS[i][k] = m_ptlMaxTemporalId[m_olsPtlIdx[i]] + 1;
#else
          m_numSubLayersInLayerInOLS[i][k] = m_vpsMaxSubLayers;
#endif
        }
      }
      m_numOutputLayersInOls[i] = j;

      for( j = 0; j < m_numOutputLayersInOls[i]; j++ )
      {
        int idx = outputLayerIdx[i][j];
        for( int k = 0; k < numRefLayers[idx]; k++ )
        {
          layerIncludedInOlsFlag[i][refLayerIdx[idx][k]] = 1;
#if !JVET_R0193
          if( m_numSubLayersInLayerInOLS[i][ refLayerIdx[idx][k] ] < m_vpsMaxTidIlRefPicsPlus1[ m_outputLayerIdInOls[i][j] ] )
          {
            m_numSubLayersInLayerInOLS[i][ refLayerIdx[idx][k] ] =  m_vpsMaxTidIlRefPicsPlus1[ m_outputLayerIdInOls[i][j] ];
          }
#endif
        }
      }
#if JVET_R0193
      for (int k = highestIncludedLayer - 1; k >= 0; k--)
      {
        if (layerIncludedInOlsFlag[i][k] && !m_vpsOlsOutputLayerFlag[i][k])
        {
          for (int m = k + 1; m <= highestIncludedLayer; m++)
          {
            uint32_t maxSublayerNeeded = std::min((uint32_t)m_numSubLayersInLayerInOLS[i][m], m_vpsMaxTidIlRefPicsPlus1[m][k]);
            if (m_vpsDirectRefLayerFlag[m][k] && layerIncludedInOlsFlag[i][m] && m_numSubLayersInLayerInOLS[i][k] < maxSublayerNeeded)
            {
              m_numSubLayersInLayerInOLS[i][k] = maxSublayerNeeded;
            }
          }
        }
      }
#endif
    }
  }
  for (int i = 0; i < m_maxLayers; i++)
  {
    CHECK_(layerUsedAsRefLayerFlag[i] == 0 && layerUsedAsOutputLayerFlag[i] == 0, "There shall be no layer that is neither an output layer nor a direct reference layer");
  }

  m_numLayersInOls[0] = 1;
  m_layerIdInOls[0][0] = m_vpsLayerId[0];
  m_numMultiLayeredOlss = 0;
  for( int i = 1; i < m_totalNumOLSs; i++ )
  {
    if( m_vpsEachLayerIsAnOlsFlag )
    {
      m_numLayersInOls[i] = 1;
      m_layerIdInOls[i][0] = m_vpsLayerId[i];
    }
    else if( m_vpsOlsModeIdc == 0 || m_vpsOlsModeIdc == 1 )
    {
      m_numLayersInOls[i] = i + 1;
      for( int j = 0; j < m_numLayersInOls[i]; j++ )
      {
        m_layerIdInOls[i][j] = m_vpsLayerId[j];
      }
    }
    else if( m_vpsOlsModeIdc == 2 )
    {
      int j = 0;
      for( int k = 0; k < m_maxLayers; k++ )
      {
        if( layerIncludedInOlsFlag[i][k] )
        {
          m_layerIdInOls[i][j++] = m_vpsLayerId[k];
        }
      }

      m_numLayersInOls[i] = j;
    }
    if( m_numLayersInOls[i] > 1 )
    {
      m_multiLayerOlsIdx[i] = m_numMultiLayeredOlss;
      m_numMultiLayeredOlss++;
    }
  }
  m_multiLayerOlsIdxToOlsIdx.resize(m_numMultiLayeredOlss);

  for (int i=0, j=0; i<m_totalNumOLSs; i++)
  {
    if (m_numLayersInOls[i] > 1)
    {
      m_multiLayerOlsIdxToOlsIdx[j] = i;
    }
  }
}

void VPS::checkVPS()
{
  for (int multiLayerOlsIdx=0; multiLayerOlsIdx < m_numMultiLayeredOlss; multiLayerOlsIdx++)
  {
    const int olsIdx = m_multiLayerOlsIdxToOlsIdx[multiLayerOlsIdx];
    const int olsTimingHrdIdx = getOlsTimingHrdIdx(multiLayerOlsIdx);
    const int olsPtlIdx = getOlsPtlIdx(olsIdx);
    CHECK_(getHrdMaxTid(olsTimingHrdIdx) < getPtlMaxTemporalId(olsPtlIdx), "The value of vps_hrd_max_tid[vps_ols_timing_hrd_idx[m]] shall be greater than or equal to "
                                                                     "vps_ptl_max_tid[ vps_ols_ptl_idx[n]] for each m-th multi-layer OLS for m from 0 to "
                                                                     "NumMultiLayerOlss - 1, inclusive, and n being the OLS index of the m-th multi-layer OLS among all OLSs.");
    const int olsDpbParamsIdx = getOlsDpbParamsIdx(multiLayerOlsIdx);
    CHECK_(m_dpbMaxTemporalId[olsDpbParamsIdx] < getPtlMaxTemporalId(olsPtlIdx), "The value of vps_dpb_max_tid[vps_ols_dpb_params_idx[m]] shall be greater than or equal to "
                                                                     "vps_ptl_max_tid[ vps_ols_ptl_idx[n]] for each m-th multi-layer OLS for m from 0 to "
                                                                     "NumMultiLayerOlss - 1, inclusive, and n being the OLS index of the m-th multi-layer OLS among all OLSs.");
  }
}


void VPS::deriveTargetOutputLayerSet( int targetOlsIdx )
{
  m_targetOlsIdx = targetOlsIdx < 0 ? m_maxLayers - 1 : targetOlsIdx;
  m_targetOutputLayerIdSet.clear();
  m_targetLayerIdSet.clear();

  for( int i = 0; i < m_numOutputLayersInOls[m_targetOlsIdx]; i++ )
  {
    m_targetOutputLayerIdSet.push_back( m_outputLayerIdInOls[m_targetOlsIdx][i] );
  }

  for( int i = 0; i < m_numLayersInOls[m_targetOlsIdx]; i++ )
  {
    m_targetLayerIdSet.push_back( m_layerIdInOls[m_targetOlsIdx][i] );
  }
}

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
int VPS::deriveTargetOLSIdx(void)
{
  int lowestIdx = 0;
  int highestNumLayers = m_numLayersInOls[lowestIdx];

  if ((m_numLayersInOls.size() > 1 ))
  {
    for (int idx = 1; idx < m_numLayersInOls.size(); idx++)
    {
      if(highestNumLayers == m_numLayersInOls[idx])
      {
        if (m_numOutputLayersInOls[lowestIdx] < m_numOutputLayersInOls[idx])
        {
          lowestIdx       = idx;
        }
      }
      else if(highestNumLayers < m_numLayersInOls[idx])
      {
        highestNumLayers = m_numLayersInOls[idx];
        lowestIdx       = idx;
      }
    }
  }
  return lowestIdx;
}

uint32_t VPS::getMaxTidinTOls(int m_targetOlsIdx)
{
  return getPtlMaxTemporalId(getOlsPtlIdx(m_targetOlsIdx));
}

#endif

// ------------------------------------------------------------------------------------------------
// Picture Header
// ------------------------------------------------------------------------------------------------

PicHeader::PicHeader()
: m_valid                                         ( 0 )
, m_nonReferencePictureFlag                       ( 0 )
, m_gdrPicFlag                                    ( 0 )
, m_recoveryPocCnt                                ( -1 )
, m_noOutputBeforeRecoveryFlag                    ( false )
, m_handleCraAsCvsStartFlag                       ( false )
, m_handleGdrAsCvsStartFlag                       ( false )
, m_spsId                                         ( -1 )
, m_ppsId                                         ( -1 )
, m_pocMsbPresentFlag                             ( 0 )
, m_pocMsbVal                                     ( 0 )
, m_virtualBoundariesEnabledFlag                  ( 0 )
, m_virtualBoundariesPresentFlag                  ( 0 )
, m_numVerVirtualBoundaries                       ( 0 )
, m_numHorVirtualBoundaries                       ( 0 )
, m_picOutputFlag                                 ( true )
, m_rpl0Idx                                       ( 0 )
, m_rpl1Idx                                       ( 0 )
, m_splitConsOverrideFlag                         ( 0 )
, m_cuQpDeltaSubdivIntra                          ( 0 )
, m_cuQpDeltaSubdivInter                          ( 0 )
, m_cuChromaQpOffsetSubdivIntra                   ( 0 )
, m_cuChromaQpOffsetSubdivInter                   ( 0 )
, m_enableTMVPFlag                                ( true )
, m_picColFromL0Flag                              ( true )
, m_mvdL1ZeroFlag                                 ( 0 )
, m_maxNumAffineMergeCand                         ( AFFINE_MRG_MAX_NUM_CANDS )
, m_disFracMMVD                                   ( 0 )
, m_bdofDisabledFlag                              ( 0 )
, m_dmvrDisabledFlag                              ( 0 )
, m_profDisabledFlag                              ( 0 )
, m_jointCbCrSignFlag                             ( 0 )
, m_qpDelta                                       ( 0 )
, m_numAlfAps                                     ( 0 )
, m_alfApsId                                      ( 0 )
, m_alfChromaApsId                                ( 0 )
, m_deblockingFilterOverrideFlag                  ( 0 )
, m_deblockingFilterDisable                       ( 0 )
, m_deblockingFilterBetaOffsetDiv2                ( 0 )
, m_deblockingFilterTcOffsetDiv2                  ( 0 )
, m_deblockingFilterCbBetaOffsetDiv2              ( 0 )
, m_deblockingFilterCbTcOffsetDiv2                ( 0 )
, m_deblockingFilterCrBetaOffsetDiv2              ( 0 )
, m_deblockingFilterCrTcOffsetDiv2                ( 0 )
, m_lmcsEnabledFlag                               ( 0 )
, m_lmcsApsId                                     ( -1 )
, m_lmcsAps                                       ( nullptr )
, m_lmcsChromaResidualScaleFlag                   ( 0 )
, m_explicitScalingListEnabledFlag                ( 0 )
, m_scalingListApsId                              ( -1 )
, m_scalingListAps                                ( nullptr )
, m_numL0Weights                                  ( 0 )
, m_numL1Weights                                  ( 0 )
{
  memset(m_virtualBoundariesPosX,                   0,    sizeof(m_virtualBoundariesPosX));
  memset(m_virtualBoundariesPosY,                   0,    sizeof(m_virtualBoundariesPosY));
  memset(m_saoEnabledFlag,                          0,    sizeof(m_saoEnabledFlag));
  memset(m_alfEnabledFlag,                          0,    sizeof(m_alfEnabledFlag));
  memset(m_minQT,                                   0,    sizeof(m_minQT));
  memset(m_maxMTTHierarchyDepth,                    0,    sizeof(m_maxMTTHierarchyDepth));
  memset(m_maxBTSize,                               0,    sizeof(m_maxBTSize));
  memset(m_maxTTSize,                               0,    sizeof(m_maxTTSize));

  m_RPL0.setNumberOfActivePictures(0);
  m_RPL0.setNumberOfShorttermPictures(0);
  m_RPL0.setNumberOfLongtermPictures(0);
  m_RPL0.setLtrpInSliceHeaderFlag(0);
  m_RPL0.setNumberOfInterLayerPictures( 0 );

  m_RPL1.setNumberOfActivePictures(0);
  m_RPL1.setNumberOfShorttermPictures(0);
  m_RPL1.setNumberOfLongtermPictures(0);
  m_RPL1.setLtrpInSliceHeaderFlag(0);
  m_RPL1.setNumberOfInterLayerPictures( 0 );

  m_alfApsId.resize(0);

  resetWpScaling();
}

PicHeader::~PicHeader()
{
  m_alfApsId.resize(0);
}

/**
 - initialize picture header to defaut state
 */
void PicHeader::initPicHeader()
{
  m_valid                                         = 0;
  m_nonReferencePictureFlag                       = 0;
  m_gdrPicFlag                                    = 0;
  m_recoveryPocCnt                                = -1;
  m_spsId                                         = -1;
  m_ppsId                                         = -1;
  m_pocMsbPresentFlag                             = 0;
  m_pocMsbVal                                     = 0;
  m_virtualBoundariesEnabledFlag                  = 0;
  m_virtualBoundariesPresentFlag                  = 0;
  m_numVerVirtualBoundaries                       = 0;
  m_numHorVirtualBoundaries                       = 0;
  m_picOutputFlag                                 = true;
  m_rpl0Idx                                       = 0;
  m_rpl1Idx                                       = 0;
  m_splitConsOverrideFlag                         = 0;
  m_cuQpDeltaSubdivIntra                          = 0;
  m_cuQpDeltaSubdivInter                          = 0;
  m_cuChromaQpOffsetSubdivIntra                   = 0;
  m_cuChromaQpOffsetSubdivInter                   = 0;
  m_enableTMVPFlag                                = true;
  m_picColFromL0Flag                              = true;
  m_mvdL1ZeroFlag                                 = 0;
  m_maxNumAffineMergeCand                         = AFFINE_MRG_MAX_NUM_CANDS;
  m_disFracMMVD                                   = 0;
  m_bdofDisabledFlag                              = 0;
  m_dmvrDisabledFlag                              = 0;
  m_profDisabledFlag                              = 0;
  m_jointCbCrSignFlag                             = 0;
  m_qpDelta                                       = 0;
  m_numAlfAps                                     = 0;
  m_alfChromaApsId                                = 0;
  m_deblockingFilterOverrideFlag                  = 0;
  m_deblockingFilterDisable                       = 0;
  m_deblockingFilterBetaOffsetDiv2                = 0;
  m_deblockingFilterTcOffsetDiv2                  = 0;
  m_deblockingFilterCbBetaOffsetDiv2              = 0;
  m_deblockingFilterCbTcOffsetDiv2                = 0;
  m_deblockingFilterCrBetaOffsetDiv2              = 0;
  m_deblockingFilterCrTcOffsetDiv2                = 0;
  m_lmcsEnabledFlag                               = 0;
  m_lmcsApsId                                     = -1;
  m_lmcsAps                                       = nullptr;
  m_lmcsChromaResidualScaleFlag                   = 0;
  m_explicitScalingListEnabledFlag                = 0;
  m_scalingListApsId                              = -1;
  m_scalingListAps                                = nullptr;
  m_numL0Weights                                  = 0;
  m_numL1Weights                                  = 0;
  memset(m_virtualBoundariesPosX,                   0,    sizeof(m_virtualBoundariesPosX));
  memset(m_virtualBoundariesPosY,                   0,    sizeof(m_virtualBoundariesPosY));
  memset(m_saoEnabledFlag,                          0,    sizeof(m_saoEnabledFlag));
  memset(m_alfEnabledFlag,                          0,    sizeof(m_alfEnabledFlag));
  memset(m_minQT,                                   0,    sizeof(m_minQT));
  memset(m_maxMTTHierarchyDepth,                    0,    sizeof(m_maxMTTHierarchyDepth));
  memset(m_maxBTSize,                               0,    sizeof(m_maxBTSize));
  memset(m_maxTTSize,                               0,    sizeof(m_maxTTSize));

  m_RPL0.setNumberOfActivePictures(0);
  m_RPL0.setNumberOfShorttermPictures(0);
  m_RPL0.setNumberOfLongtermPictures(0);
  m_RPL0.setLtrpInSliceHeaderFlag(0);

  m_RPL1.setNumberOfActivePictures(0);
  m_RPL1.setNumberOfShorttermPictures(0);
  m_RPL1.setNumberOfLongtermPictures(0);
  m_RPL1.setLtrpInSliceHeaderFlag(0);

  m_alfApsId.resize(0);
}

const WPScalingParam *PicHeader::getWpScaling(const RefPicList refPicList, const int refIdx) const
{
  CHECK_(refPicList >= NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  if (refIdx < 0)
  {
    return nullptr;
  }
  else
  {
    return m_weightPredTable[refPicList][refIdx];
  }
}

WPScalingParam *PicHeader::getWpScaling(const RefPicList refPicList, const int refIdx)
{
  CHECK_(refPicList >= NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  if (refIdx < 0)
  {
    return nullptr;
  }
  else
  {
    return m_weightPredTable[refPicList][refIdx];
  }
}

void PicHeader::resetWpScaling()
{
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->presentFlag     = false;
        pwp->log2WeightDenom = 0;
        pwp->codedWeight     = 1;
        pwp->codedOffset     = 0;
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------
SPSRExt::SPSRExt()
 : m_transformSkipRotationEnabledFlag   (false)
 , m_transformSkipContextEnabledFlag    (false)
 , m_extendedPrecisionProcessingFlag    (false)
 , m_intraSmoothingDisabledFlag         (false)
 , m_highPrecisionOffsetsEnabledFlag    (false)
 , m_persistentRiceAdaptationEnabledFlag(false)
 , m_cabacBypassAlignmentEnabledFlag    (false)
{
}


SPS::SPS()
: m_SPSId                     (  0)
, m_VPSId                     ( 0 )
, m_layerId                   ( 0 )
, m_affineAmvrEnabledFlag     ( false )
, m_DMVR                      ( false )
, m_MMVD                      ( false )
, m_SBT                       ( false )
, m_ISP                       ( false )
, m_chromaFormatIdc           (CHROMA_420)
, m_uiMaxTLayers              (  1)
, m_ptlDpbHrdParamsPresentFlag (1)
, m_SubLayerDpbParamsFlag      (0)
// Structure
, m_maxWidthInLumaSamples     (352)
, m_maxHeightInLumaSamples    (288)
, m_subPicInfoPresentFlag     (false)
, m_numSubPics(1)
, m_independentSubPicsFlag     (false)
, m_subPicSameSizeFlag        (false)
, m_subPicIdMappingExplicitlySignalledFlag ( false )
, m_subPicIdMappingPresentFlag ( false )
, m_subPicIdLen(16)
, m_log2MinCodingBlockSize    (  2)
, m_CTUSize(0)
, m_minQT{ 0, 0, 0 }
, m_maxMTTHierarchyDepth{ MAX_BT_DEPTH, MAX_BT_DEPTH_INTER, MAX_BT_DEPTH_C }
, m_maxBTSize{ 0, 0, 0 }
, m_maxTTSize{ 0, 0, 0 }
, m_uiMaxCUWidth              ( 32)
, m_uiMaxCUHeight             ( 32)
, m_numRPL0                   ( 0 )
, m_numRPL1                   ( 0 )
, m_rpl1CopyFromRpl0Flag      ( false )
, m_rpl1IdxPresentFlag        ( false )
, m_allRplEntriesHasSameSignFlag ( true )
, m_bLongTermRefsPresent      (false)
// Tool list
, m_transformSkipEnabledFlag  (false)
, m_log2MaxTransformSkipBlockSize (2)
, m_BDPCMEnabledFlag          (false)
, m_JointCbCrEnabledFlag      (false)
, m_entropyCodingSyncEnabledFlag(false)
, m_entryPointPresentFlag(false)
, m_internalMinusInputBitDepth{ 0, 0 }
, m_sbtmvpEnabledFlag         (false)
, m_bdofEnabledFlag           (false)
, m_fpelMmvdEnabledFlag       ( false )
, m_BdofControlPresentInPhFlag( false )
, m_DmvrControlPresentInPhFlag( false )
, m_ProfControlPresentInPhFlag( false )
, m_uiBitsForPOC              (  8)
, m_pocMsbCycleFlag           ( false )
, m_pocMsbCycleLen            ( 1 )
, m_numExtraPHBytes           ( 0 )
, m_numExtraSHBytes           ( 0 )
, m_numLongTermRefPicSPS      (  0)

, m_log2MaxTbSize             (  6)

, m_useWeightPred             (false)
, m_useWeightedBiPred         (false)
, m_saoEnabledFlag            (false)
, m_bTemporalIdNestingFlag    (false)
, m_scalingListEnabledFlag    (false)
, m_virtualBoundariesEnabledFlag( 0 )
, m_virtualBoundariesPresentFlag( 0 )
, m_numVerVirtualBoundaries(0)
, m_numHorVirtualBoundaries(0)
, m_generalHrdParametersPresentFlag(false)
, m_fieldSeqFlag              (false)
, m_vuiParametersPresentFlag  (false)
, m_vuiParameters             ()
, m_wrapAroundEnabledFlag     (false)
, m_IBCFlag                   (  0)
, m_PLTMode                   (  0)
, m_lmcsEnabled               (false)
, m_AMVREnabledFlag                       ( false )
, m_LMChroma                  ( false )
, m_horCollocatedChromaFlag   ( true )
, m_verCollocatedChromaFlag   ( false )
, m_IntraMTS                  ( false )
, m_InterMTS                  ( false )
, m_LFNST                     ( false )
, m_Affine                    ( false )
, m_AffineType                ( false )
, m_PROF                      ( false )
, m_ciip                   ( false )
, m_Geo                       ( false )
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
, m_LadfEnabled               ( false )
, m_LadfNumIntervals          ( 0 )
, m_LadfQpOffset              { 0 }
, m_LadfIntervalLowerBound    { 0 }
#endif
, m_MRL                       ( false )
, m_MIP                       ( false )
, m_GDREnabledFlag            ( true )
, m_SubLayerCbpParametersPresentFlag ( true )
, m_rprEnabledFlag            ( false )
, m_resChangeInClvsEnabledFlag ( false )
, m_maxNumMergeCand(MRG_MAX_NUM_CANDS)
, m_maxNumAffineMergeCand(AFFINE_MRG_MAX_NUM_CANDS)
, m_maxNumIBCMergeCand(IBC_MRG_MAX_NUM_CANDS)
, m_maxNumGeoCand(0)
, m_scalingMatrixAlternativeColourSpaceDisabledFlag( false )
, m_scalingMatrixDesignatedColourSpaceFlag( true )
, m_disableScalingMatrixForLfnstBlks( true)
{
  for(int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_bitDepths.recon[ch] = 8;
    m_qpBDOffset   [ch] = 0;
  }

  for ( int i = 0; i < MAX_TLAYER; i++ )
  {
    m_uiMaxLatencyIncreasePlus1[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_maxNumReorderPics[i]    = 0;
  }

  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_usedByCurrPicLtSPSFlag, 0, sizeof(m_usedByCurrPicLtSPSFlag));
  ::memset(m_virtualBoundariesPosX, 0, sizeof(m_virtualBoundariesPosX));
  ::memset(m_virtualBoundariesPosY, 0, sizeof(m_virtualBoundariesPosY));
  ::memset(m_ppsValidFlag, 0, sizeof(m_ppsValidFlag));
}

SPS::~SPS()
{
}

void  SPS::createRPLList0(int numRPL)
{
  m_RPLList0.destroy();
  m_RPLList0.create(numRPL);
  m_numRPL0 = numRPL;
  m_rpl1IdxPresentFlag = (m_numRPL0 != m_numRPL1) ? true : false;
}
void  SPS::createRPLList1(int numRPL)
{
  m_RPLList1.destroy();
  m_RPLList1.create(numRPL);
  m_numRPL1 = numRPL;

  m_rpl1IdxPresentFlag = (m_numRPL0 != m_numRPL1) ? true : false;
}


const int SPS::m_winUnitX[]={1,2,2,1};
const int SPS::m_winUnitY[]={1,2,1,1};

void ChromaQpMappingTable::setParams(const ChromaQpMappingTableParams &params, const int qpBdOffset)
{
  m_qpBdOffset = qpBdOffset;
  m_sameCQPTableForAllChromaFlag = params.m_sameCQPTableForAllChromaFlag;
  m_numQpTables = params.m_numQpTables;

  for (int i = 0; i < MAX_NUM_CQP_MAPPING_TABLES; i++)
  {
    m_numPtsInCQPTableMinus1[i] = params.m_numPtsInCQPTableMinus1[i];
    m_deltaQpInValMinus1[i] = params.m_deltaQpInValMinus1[i];
    m_qpTableStartMinus26[i] = params.m_qpTableStartMinus26[i];
    m_deltaQpOutVal[i] = params.m_deltaQpOutVal[i];
  }
}
void ChromaQpMappingTable::derivedChromaQPMappingTables()
{
  for (int i = 0; i < getNumQpTables(); i++)
  {
    const int qpBdOffsetC = m_qpBdOffset;
    const int numPtsInCQPTableMinus1 = getNumPtsInCQPTableMinus1(i);
    std::vector<int> qpInVal(numPtsInCQPTableMinus1 + 2), qpOutVal(numPtsInCQPTableMinus1 + 2);

    qpInVal[0] = getQpTableStartMinus26(i) + 26;
    qpOutVal[0] = qpInVal[0];
    for (int j = 0; j <= getNumPtsInCQPTableMinus1(i); j++)
    {
      qpInVal[j + 1] = qpInVal[j] + getDeltaQpInValMinus1(i, j) + 1;
      qpOutVal[j + 1] = qpOutVal[j] + getDeltaQpOutVal(i, j);
    }

    for (int j = 0; j <= getNumPtsInCQPTableMinus1(i); j++)
    {
      CHECK_(qpInVal[j]  < -qpBdOffsetC || qpInVal[j]  > MAX_QP, "qpInVal out of range");
      CHECK_(qpOutVal[j] < -qpBdOffsetC || qpOutVal[j] > MAX_QP, "qpOutVal out of range");
    }

    m_chromaQpMappingTables[i][qpInVal[0]] = qpOutVal[0];
    for (int k = qpInVal[0] - 1; k >= -qpBdOffsetC; k--)
    {
      m_chromaQpMappingTables[i][k] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k + 1] - 1);
    }
    for (int j = 0; j <= numPtsInCQPTableMinus1; j++)
    {
      int sh = (getDeltaQpInValMinus1(i, j) + 1) >> 1;
      for (int k = qpInVal[j] + 1, m = 1; k <= qpInVal[j + 1]; k++, m++)
      {
        m_chromaQpMappingTables[i][k] = m_chromaQpMappingTables[i][qpInVal[j]]
          + ((qpOutVal[j + 1] - qpOutVal[j]) * m + sh) / (getDeltaQpInValMinus1(i, j) + 1);
      }
    }
    for (int k = qpInVal[numPtsInCQPTableMinus1 + 1] + 1; k <= MAX_QP; k++)
    {
      m_chromaQpMappingTables[i][k] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k - 1] + 1);
    }
  }
}

SliceMap::SliceMap()
: m_sliceID              (0)
, m_numTilesInSlice      (0)
, m_numCtuInSlice        (0)
{
  m_ctuAddrInSlice.clear();
}

SliceMap::~SliceMap()
{
  m_numCtuInSlice = 0;
  m_ctuAddrInSlice.clear();
}

RectSlice::RectSlice()
: m_tileIdx            (0)
, m_sliceWidthInTiles  (0)
, m_sliceHeightInTiles (0)
, m_numSlicesInTile    (0)
, m_sliceHeightInCtu   (0)
{
}

RectSlice::~RectSlice()
{
}

SubPic::SubPic()
: m_subPicID              (0)
, m_numCTUsInSubPic       (0)
, m_subPicCtuTopLeftX     (0)
, m_subPicCtuTopLeftY     (0)
, m_subPicWidth           (0)
, m_subPicHeight          (0)
, m_firstCtuInSubPic      (0)
, m_lastCtuInSubPic       (0)
, m_subPicLeft            (0)
, m_subPicRight           (0)
, m_subPicTop             (0)
, m_subPicBottom          (0)
, m_treatedAsPicFlag                  (false)
, m_loopFilterAcrossSubPicEnabledFlag (false)
{
  m_ctuAddrInSubPic.clear();
}

SubPic::~SubPic()
{
  m_ctuAddrInSubPic.clear();
}


PPS::PPS()
: m_PPSId                            (0)
, m_SPSId                            (0)
, m_picInitQPMinus26                 (0)
, m_useDQP                           (false)
, m_usePPSChromaTool                 (false)
, m_bSliceChromaQpFlag               (false)
, m_chromaCbQpOffset                 (0)
, m_chromaCrQpOffset                 (0)
, m_chromaCbCrQpOffset               (0)
, m_chromaQpOffsetListLen              (0)
, m_numRefIdxL0DefaultActive         (1)
, m_numRefIdxL1DefaultActive         (1)
, m_rpl1IdxPresentFlag               (false)
, m_numSubPics                       (1)
, m_subPicIdMappingInPpsFlag         (0)
, m_subPicIdLen                      (16)
, m_noPicPartitionFlag               (1)
, m_log2CtuSize                      (0)
, m_ctuSize                          (0)
, m_picWidthInCtu                    (0)
, m_picHeightInCtu                   (0)
, m_numTileCols                      (1)
, m_numTileRows                      (1)
, m_rectSliceFlag                    (1)
, m_singleSlicePerSubPicFlag         (0)
, m_numSlicesInPic                   (1)
, m_tileIdxDeltaPresentFlag          (0)
, m_loopFilterAcrossTilesEnabledFlag (1)
, m_loopFilterAcrossSlicesEnabledFlag(0)
, m_cabacInitPresentFlag             (false)
, m_pictureHeaderExtensionPresentFlag(0)
, m_sliceHeaderExtensionPresentFlag  (false)
, m_listsModificationPresentFlag     (0)
, m_rplInfoInPhFlag                  (0)
, m_dbfInfoInPhFlag                  (0)
, m_saoInfoInPhFlag                  (0)
, m_alfInfoInPhFlag                  (0)
, m_wpInfoInPhFlag                   (0)
, m_qpDeltaInfoInPhFlag              (0)
, m_mixedNaluTypesInPicFlag          ( false )
, m_conformanceWindowFlag            (false)
, m_picWidthInLumaSamples(352)
, m_picHeightInLumaSamples( 288 )
, m_wrapAroundEnabledFlag            (false)
, m_picWidthMinusWrapAroundOffset    (0)
, m_wrapAroundOffset                 (0)
, pcv                                (NULL)
{
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset = 0; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here and never subsequently changed.
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.JointCbCrOffset = 0;
  m_tileColWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_ctuToSubPicIdx.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();
  m_subPics.clear();
}

PPS::~PPS()
{
  m_tileColWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_ctuToSubPicIdx.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();

  m_subPics.clear();
  delete pcv;
}

/**
 - reset tile and slice parameters and lists
 */
void PPS::resetTileSliceInfo()
{
  m_numExpTileCols = 0;
  m_numExpTileRows = 0;
  m_numTileCols    = 0;
  m_numTileRows    = 0;
  m_numSlicesInPic = 0;
  m_tileColWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_ctuToSubPicIdx.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();
}

/**
 - initialize tile row/column sizes and boundaries
 */
void PPS::initTiles()
{
  int       colIdx, rowIdx;
  int       ctuX, ctuY;

  // check explicit tile column sizes
  uint32_t  remainingWidthInCtu  = m_picWidthInCtu;

  for( colIdx = 0; colIdx < m_numExpTileCols; colIdx++ )
  {
    CHECK_(m_tileColWidth[colIdx] > remainingWidthInCtu,    "Tile column width exceeds picture width");
    remainingWidthInCtu -= m_tileColWidth[colIdx];
  }

  // divide remaining picture width into uniform tile columns
  uint32_t  uniformTileColWidth = m_tileColWidth[colIdx-1];
  while( remainingWidthInCtu > 0 )
  {
    CHECK_(colIdx >= MAX_TILE_COLS, "Number of tile columns exceeds valid range");
    uniformTileColWidth = std::min(remainingWidthInCtu, uniformTileColWidth);
    m_tileColWidth.push_back( uniformTileColWidth );
    remainingWidthInCtu -= uniformTileColWidth;
    colIdx++;
  }
  m_numTileCols = colIdx;

  // check explicit tile row sizes
  uint32_t  remainingHeightInCtu  = m_picHeightInCtu;

  for( rowIdx = 0; rowIdx < m_numExpTileRows; rowIdx++ )
  {
    CHECK_(m_tileRowHeight[rowIdx] > remainingHeightInCtu,     "Tile row height exceeds picture height");
    remainingHeightInCtu -= m_tileRowHeight[rowIdx];
  }

  // divide remaining picture height into uniform tile rows
  uint32_t  uniformTileRowHeight = m_tileRowHeight[rowIdx - 1];
  while( remainingHeightInCtu > 0 )
  {
    uniformTileRowHeight = std::min(remainingHeightInCtu, uniformTileRowHeight);
    m_tileRowHeight.push_back( uniformTileRowHeight );
    remainingHeightInCtu -= uniformTileRowHeight;
    rowIdx++;
  }
  m_numTileRows = rowIdx;

  // set left column bounaries
  m_tileColBd.push_back( 0 );
  for( colIdx = 0; colIdx < m_numTileCols; colIdx++ )
  {
    m_tileColBd.push_back( m_tileColBd[ colIdx ] + m_tileColWidth[ colIdx ] );
  }

  // set top row bounaries
  m_tileRowBd.push_back( 0 );
  for( rowIdx = 0; rowIdx < m_numTileRows; rowIdx++ )
  {
    m_tileRowBd.push_back( m_tileRowBd[ rowIdx ] + m_tileRowHeight[ rowIdx ] );
  }

  // set mapping between horizontal CTU address and tile column index
  colIdx = 0;
  for( ctuX = 0; ctuX <= m_picWidthInCtu; ctuX++ )
  {
    if( ctuX == m_tileColBd[ colIdx + 1 ] )
    {
      colIdx++;
    }
    m_ctuToTileCol.push_back( colIdx );
  }

  // set mapping between vertical CTU address and tile row index
  rowIdx = 0;
  for( ctuY = 0; ctuY <= m_picHeightInCtu; ctuY++ )
  {
    if( ctuY == m_tileRowBd[ rowIdx + 1 ] )
    {
      rowIdx++;
    }
    m_ctuToTileRow.push_back( rowIdx );
  }
}

/**
 - initialize memory for rectangular slice parameters
 */
void PPS::initRectSlices()
{
  CHECK_(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_rectSlices.resize(m_numSlicesInPic);
}

/**
 - initialize mapping between rectangular slices and CTUs
 */
void PPS::initRectSliceMap(const SPS  *sps)
{
  uint32_t  ctuY;
  uint32_t  tileX, tileY;

  if (sps)
  {
    m_ctuToSubPicIdx.resize(getPicWidthInCtu() * getPicHeightInCtu());
    if (sps->getNumSubPics() > 1)
    {
      for (int i = 0; i <= sps->getNumSubPics() - 1; i++)
      {
        for (int y = sps->getSubPicCtuTopLeftY(i); y < sps->getSubPicCtuTopLeftY(i) + sps->getSubPicHeight(i); y++)
        {
          for (int x = sps->getSubPicCtuTopLeftX(i); x < sps->getSubPicCtuTopLeftX(i) + sps->getSubPicWidth(i); x++)
          {
            m_ctuToSubPicIdx[ x+ y * getPicWidthInCtu()] = i;
          }
        }
      }
    }
    else
    {
      for (int i = 0; i < getPicWidthInCtu() * getPicHeightInCtu(); i++)
      {
        m_ctuToSubPicIdx[i] = 0;
      }
    }
  }

  if( getSingleSlicePerSubPicFlag() )
  {
    CHECK_(sps==nullptr, "RectSliceMap can only be initialized for slice_per_sub_pic_flag with a valid SPS");
    m_numSlicesInPic = sps->getNumSubPics();

    // allocate new memory for slice list
    CHECK_(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
    m_sliceMap.resize( m_numSlicesInPic );

    if (sps->getNumSubPics() > 1)
    {
      // Q2001 v15 equation 29
      std::vector<uint32_t> subpicWidthInTiles;
      std::vector<uint32_t> subpicHeightInTiles;
      std::vector<uint32_t> subpicHeightLessThanOneTileFlag;
      subpicWidthInTiles.resize(sps->getNumSubPics());
      subpicHeightInTiles.resize(sps->getNumSubPics());
      subpicHeightLessThanOneTileFlag.resize(sps->getNumSubPics());
      for (uint32_t i = 0; i <sps->getNumSubPics(); i++)
      {
        uint32_t leftX = sps->getSubPicCtuTopLeftX(i);
        uint32_t rightX = leftX + sps->getSubPicWidth(i) - 1;
        subpicWidthInTiles[i] = m_ctuToTileCol[rightX] + 1 - m_ctuToTileCol[leftX];

        uint32_t topY = sps->getSubPicCtuTopLeftY(i);
        uint32_t bottomY = topY + sps->getSubPicHeight(i) - 1;
        subpicHeightInTiles[i] = m_ctuToTileRow[bottomY] + 1 - m_ctuToTileRow[topY];

        if (subpicHeightInTiles[i] == 1 && sps->getSubPicHeight(i) < m_tileRowHeight[m_ctuToTileRow[topY]] )
        {
          subpicHeightLessThanOneTileFlag[i] = 1;
        }
        else
        {
          subpicHeightLessThanOneTileFlag[i] = 0;
        }
      }

      for( int i = 0; i < m_numSlicesInPic; i++ )
      {
        CHECK_(m_numSlicesInPic != sps->getNumSubPics(), "in single slice per subpic mode, number of slice and subpic shall be equal");
        m_sliceMap[ i ].initSliceMap();
        if (subpicHeightLessThanOneTileFlag[i])
        {
          m_sliceMap[i].addCtusToSlice(sps->getSubPicCtuTopLeftX(i), sps->getSubPicCtuTopLeftX(i) + sps->getSubPicWidth(i),
                                       sps->getSubPicCtuTopLeftY(i), sps->getSubPicCtuTopLeftY(i) + sps->getSubPicHeight(i), m_picWidthInCtu);
        }
        else
        {
          tileX = m_ctuToTileCol[sps->getSubPicCtuTopLeftX(i)];
          tileY = m_ctuToTileRow[sps->getSubPicCtuTopLeftY(i)];
          for (uint32_t j = 0; j< subpicHeightInTiles[i]; j++)
          {
            for (uint32_t k = 0; k < subpicWidthInTiles[i]; k++)
            {
              m_sliceMap[i].addCtusToSlice(getTileColumnBd(tileX + k), getTileColumnBd(tileX + k + 1), getTileRowBd(tileY + j), getTileRowBd(tileY + j + 1), m_picWidthInCtu);
            }
          }
        }
      }
      subpicWidthInTiles.clear();
      subpicHeightInTiles.clear();
      subpicHeightLessThanOneTileFlag.clear();
    }
    else
    {
      m_sliceMap[0].initSliceMap();
      for (int tileY=0; tileY<m_numTileRows; tileY++)
      {
        for (int tileX=0; tileX<m_numTileCols; tileX++)
        {
          m_sliceMap[0].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1),
                                       getTileRowBd(tileY), getTileRowBd(tileY + 1), m_picWidthInCtu);
        }
      }
      m_sliceMap[0].setSliceID(0);
    }
  }
  else
  {
    // allocate new memory for slice list
    CHECK_(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
    m_sliceMap.resize( m_numSlicesInPic );
    // generate CTU maps for all rectangular slices in picture
    for( uint32_t i = 0; i < m_numSlicesInPic; i++ )
    {
      m_sliceMap[ i ].initSliceMap();

      // get position of first tile in slice
      tileX =  m_rectSlices[ i ].getTileIdx() % m_numTileCols;
      tileY =  m_rectSlices[ i ].getTileIdx() / m_numTileCols;

      // infer slice size for last slice in picture
      if( i == m_numSlicesInPic-1 )
      {
        m_rectSlices[ i ].setSliceWidthInTiles ( m_numTileCols - tileX );
        m_rectSlices[ i ].setSliceHeightInTiles( m_numTileRows - tileY );
        m_rectSlices[ i ].setNumSlicesInTile( 1 );
      }

      // set slice index
      m_sliceMap[ i ].setSliceID(i);

      // complete tiles within a single slice case
      if( m_rectSlices[ i ].getSliceWidthInTiles( ) > 1 || m_rectSlices[ i ].getSliceHeightInTiles( ) > 1)
      {
        for( uint32_t j = 0; j < m_rectSlices[ i ].getSliceHeightInTiles( ); j++ )
        {
          for( uint32_t k = 0; k < m_rectSlices[ i ].getSliceWidthInTiles( ); k++ )
          {
            m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX + k), getTileColumnBd(tileX + k +1),
                                            getTileRowBd(tileY + j), getTileRowBd(tileY + j +1), m_picWidthInCtu);
          }
        }
      }
      // multiple slices within a single tile case
      else
      {
        uint32_t  numSlicesInTile = m_rectSlices[ i ].getNumSlicesInTile( );

        ctuY = getTileRowBd( tileY );
        for( uint32_t j = 0; j < numSlicesInTile-1; j++ )
        {
          m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX), getTileColumnBd(tileX+1),
                                          ctuY, ctuY + m_rectSlices[ i ].getSliceHeightInCtu(), m_picWidthInCtu);
          ctuY += m_rectSlices[ i ].getSliceHeightInCtu();
          i++;
          m_sliceMap[ i ].initSliceMap();
          m_sliceMap[ i ].setSliceID(i);
        }

        // infer slice height for last slice in tile
        CHECK_( ctuY >= getTileRowBd( tileY + 1 ), "Invalid rectangular slice signalling");
        m_rectSlices[ i ].setSliceHeightInCtu( getTileRowBd( tileY + 1 ) - ctuY );
        m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX), getTileColumnBd(tileX+1),
                                        ctuY, getTileRowBd( tileY + 1 ), m_picWidthInCtu);
      }
    }
  }
  // check for valid rectangular slice map
  checkSliceMap();
}

/**
- initialize mapping between subpicture and CTUs
*/
void PPS::initSubPic(const SPS &sps)
{
  if (getSubPicIdMappingInPpsFlag())
  {
    // When signalled, the number of subpictures has to match in PPS and SPS
    CHECK_(getNumSubPics() != sps.getNumSubPics(), "pps_num_subpics_minus1 shall be equal to sps_num_subpics_minus1");
  }
  else
  {
    // When not signalled  set the numer equal for convenient access
    setNumSubPics(sps.getNumSubPics());
  }

  CHECK_(getNumSubPics() > MAX_NUM_SUB_PICS, "Number of sub-pictures in picture exceeds valid range");
  m_subPics.resize(getNumSubPics());

  // Check that no subpicture is specified outside of the conformance cropping window
  for(int i = 0; i < sps.getNumSubPics(); i++)
  {
    CHECK_( (sps.getSubPicCtuTopLeftX(i) * sps.getCTUSize()) >=
          (sps.getMaxPicWidthInLumaSamples() - sps.getConformanceWindow().getWindowRightOffset() * SPS::getWinUnitX(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window");
    CHECK_( ((sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i)) * sps.getCTUSize()) <= (sps.getConformanceWindow().getWindowLeftOffset() * SPS::getWinUnitX(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window" );
    CHECK_( (sps.getSubPicCtuTopLeftY(i) * sps.getCTUSize()) >=
          (sps.getMaxPicHeightInLumaSamples()  - sps.getConformanceWindow().getWindowBottomOffset() * SPS::getWinUnitY(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window");
    CHECK_( ((sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i)) * sps.getCTUSize()) <= (sps.getConformanceWindow().getWindowTopOffset() * SPS::getWinUnitY(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window");
  }

  // m_ctuSize,  m_picWidthInCtu, and m_picHeightInCtu might not be initialized yet.
  if (m_ctuSize == 0 || m_picWidthInCtu == 0 || m_picHeightInCtu == 0)
  {
    m_ctuSize = sps.getCTUSize();
    m_picWidthInCtu = (m_picWidthInLumaSamples + m_ctuSize - 1) / m_ctuSize;
    m_picHeightInCtu = (m_picHeightInLumaSamples + m_ctuSize - 1) / m_ctuSize;
  }
  for (int i=0; i< getNumSubPics(); i++)
  {
    m_subPics[i].setSubPicIdx(i);
    if(sps.getSubPicIdMappingExplicitlySignalledFlag())
    {
      if(m_subPicIdMappingInPpsFlag)
      {
        m_subPics[i].setSubPicID(m_subPicId[i]);
      }
      else
      {
        m_subPics[i].setSubPicID(sps.getSubPicId(i));
      }
    }
    else
    {
      m_subPics[i].setSubPicID(i);
    }
    m_subPics[i].setSubPicCtuTopLeftX(sps.getSubPicCtuTopLeftX(i));
    m_subPics[i].setSubPicCtuTopLeftY(sps.getSubPicCtuTopLeftY(i));
    m_subPics[i].setSubPicWidthInCTUs(sps.getSubPicWidth(i));
    m_subPics[i].setSubPicHeightInCTUs(sps.getSubPicHeight(i));

    uint32_t firstCTU = sps.getSubPicCtuTopLeftY(i) * m_picWidthInCtu + sps.getSubPicCtuTopLeftX(i); 	
    m_subPics[i].setFirstCTUInSubPic(firstCTU);
    uint32_t lastCTU = (sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i) - 1) * m_picWidthInCtu + sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i) - 1;
    m_subPics[i].setLastCTUInSubPic(lastCTU);

    uint32_t left = sps.getSubPicCtuTopLeftX(i) * m_ctuSize;
    m_subPics[i].setSubPicLeft(left);

    uint32_t right = std::min(m_picWidthInLumaSamples - 1, (sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i)) * m_ctuSize - 1);
    m_subPics[i].setSubPicRight(right);

    m_subPics[i].setSubPicWidthInLumaSample(right - left + 1);

    uint32_t top = sps.getSubPicCtuTopLeftY(i) * m_ctuSize;
    m_subPics[i].setSubPicTop(top);

    uint32_t bottom = std::min(m_picHeightInLumaSamples - 1, (sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i)) * m_ctuSize - 1);

    m_subPics[i].setSubPicHeightInLumaSample(bottom - top + 1);

    m_subPics[i].setSubPicBottom(bottom);

    m_subPics[i].clearCTUAddrList();

    if (m_numSlicesInPic == 1)
    {
      CHECK_(getNumSubPics() != 1, "only one slice in picture, but number of subpic is not one");
      m_subPics[i].addAllCtusInPicToSubPic(0, getPicWidthInCtu(), 0, getPicHeightInCtu(), getPicWidthInCtu());
      m_subPics[i].setNumSlicesInSubPic(1);
    }
    else
    {
      int numSlicesInSubPic = 0;
      int idxLastSliceInSubpic = -1;
      int idxFirstSliceAfterSubpic = m_numSlicesInPic;
      for (int j = 0; j < m_numSlicesInPic; j++)
      {
        uint32_t ctu = m_sliceMap[j].getCtuAddrInSlice(0);
        uint32_t ctu_x = ctu % m_picWidthInCtu;
        uint32_t ctu_y = ctu / m_picWidthInCtu;
        if (ctu_x >= sps.getSubPicCtuTopLeftX(i) &&
          ctu_x < (sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i)) &&
          ctu_y >= sps.getSubPicCtuTopLeftY(i) &&
          ctu_y < (sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i)))
        {
          // add ctus in a slice to the subpicture it belongs to
          m_subPics[i].addCTUsToSubPic(m_sliceMap[j].getCtuAddrList());
	  numSlicesInSubPic++;
          idxLastSliceInSubpic = j;
        }
        else if (idxFirstSliceAfterSubpic == m_numSlicesInPic && idxLastSliceInSubpic != -1)
        {
          idxFirstSliceAfterSubpic = j;
        }
      }
      CHECK_( idxFirstSliceAfterSubpic < idxLastSliceInSubpic, "The signalling order of slices shall follow the coding order" );
      m_subPics[i].setNumSlicesInSubPic(numSlicesInSubPic);
    }
    m_subPics[i].setTreatedAsPicFlag(sps.getSubPicTreatedAsPicFlag(i));
    m_subPics[i].setloopFilterAcrossEnabledFlag(sps.getLoopFilterAcrossSubpicEnabledFlag(i));
  }
}

const SubPic& PPS::getSubPicFromPos(const Position& pos)  const
{
  for (int i = 0; i< m_numSubPics; i++)
  {
    if (m_subPics[i].isContainingPos(pos))
    {
      return m_subPics[i];
    }
  }
  return m_subPics[0];
}

const SubPic&  PPS::getSubPicFromCU(const CodingUnit& cu) const
{
  const Position lumaPos = cu.Y().valid() ? cu.Y().pos() : recalcPosition(cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos());
  return getSubPicFromPos(lumaPos);
}

uint32_t PPS::getSubPicIdxFromSubPicId( uint32_t subPicId ) const
{
  for (int i = 0; i < m_numSubPics; i++)
  {
    if(m_subPics[i].getSubPicID() == subPicId)
    {
      return i;
    }
  }
  return 0;
}

void PPS::initRasterSliceMap( std::vector<uint32_t> numTilesInSlice )
{
  uint32_t tileIdx = 0;
  setNumSlicesInPic( (uint32_t) numTilesInSlice.size() );

  // allocate new memory for slice list
  CHECK_(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_sliceMap.resize( m_numSlicesInPic );

  for( uint32_t sliceIdx = 0; sliceIdx < numTilesInSlice.size(); sliceIdx++ )
  {
    m_sliceMap[sliceIdx].initSliceMap();
    m_sliceMap[sliceIdx].setSliceID( tileIdx );
    m_sliceMap[sliceIdx].setNumTilesInSlice( numTilesInSlice[sliceIdx] );
    for( uint32_t idx = 0; idx < numTilesInSlice[sliceIdx]; idx++ )
    {
      uint32_t tileX = tileIdx % getNumTileColumns();
      uint32_t tileY = tileIdx / getNumTileColumns();
      CHECK_(tileY >= getNumTileRows(), "Number of tiles in slice exceeds the remaining number of tiles in picture");

      m_sliceMap[sliceIdx].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1),
                                          getTileRowBd(tileY), getTileRowBd(tileY + 1),
                                          getPicWidthInCtu());
      tileIdx++;
    }
  }

  // check for valid raster-scan slice map
  checkSliceMap();
}

/**
 - check if slice map covers the entire picture without skipping or duplicating any CTU positions
 */
void PPS::checkSliceMap()
{
  uint32_t i;
  std::vector<uint32_t>  ctuList, sliceList;
  uint32_t picSizeInCtu = getPicWidthInCtu() * getPicHeightInCtu();
  for( i = 0; i < m_numSlicesInPic; i++ )
  {
    sliceList = m_sliceMap[ i ].getCtuAddrList();
    ctuList.insert( ctuList.end(), sliceList.begin(), sliceList.end() );
  }
  CHECK_( ctuList.size() < picSizeInCtu, "Slice map contains too few CTUs");
  CHECK_( ctuList.size() > picSizeInCtu, "Slice map contains too many CTUs");
  std::sort( ctuList.begin(), ctuList.end() );
  for( i = 1; i < ctuList.size(); i++ )
  {
    CHECK_( ctuList[i] > ctuList[i-1]+1, "CTU missing in slice map");
    CHECK_( ctuList[i] == ctuList[i-1],  "CTU duplicated in slice map");
  }
}

APS::APS()
: m_APSId(0)
, m_temporalId( 0 )
, m_layerId( 0 )
{
}

APS::~APS()
{
}

ReferencePictureList::ReferencePictureList( const bool interLayerPicPresentFlag )
  : m_numberOfShorttermPictures(0)
  , m_numberOfLongtermPictures(0)
  , m_numberOfActivePictures(MAX_INT)
  , m_ltrp_in_slice_header_flag(0)
  , m_interLayerPresentFlag( interLayerPicPresentFlag )
  , m_numberOfInterLayerPictures( 0 )
{
  ::memset(m_isLongtermRefPic, 0, sizeof(m_isLongtermRefPic));
  ::memset(m_refPicIdentifier, 0, sizeof(m_refPicIdentifier));
  ::memset(m_POC, 0, sizeof(m_POC));
  ::memset( m_isInterLayerRefPic, 0, sizeof( m_isInterLayerRefPic ) );
  ::memset( m_interLayerRefPicIdx, 0, sizeof( m_interLayerRefPicIdx ) );

  ::memset(m_deltaPOCMSBCycleLT, 0, sizeof(m_deltaPOCMSBCycleLT));
  ::memset(m_deltaPocMSBPresentFlag, 0, sizeof(m_deltaPocMSBPresentFlag));
}

ReferencePictureList::~ReferencePictureList()
{
}

void ReferencePictureList::setRefPicIdentifier( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx )
{
  m_refPicIdentifier[idx] = identifier;
  m_isLongtermRefPic[idx] = isLongterm;

  m_deltaPocMSBPresentFlag[idx] = false;
  m_deltaPOCMSBCycleLT[idx] = 0;

  m_isInterLayerRefPic[idx] = isInterLayerRefPic;
  m_interLayerRefPicIdx[idx] = interLayerIdx;
}

int ReferencePictureList::getRefPicIdentifier(int idx) const
{
  return m_refPicIdentifier[idx];
}


bool ReferencePictureList::isRefPicLongterm(int idx) const
{
  return m_isLongtermRefPic[idx];
}

void ReferencePictureList::setNumberOfShorttermPictures(int numberOfStrp)
{
  m_numberOfShorttermPictures = numberOfStrp;
}

int ReferencePictureList::getNumberOfShorttermPictures() const
{
  return m_numberOfShorttermPictures;
}

void ReferencePictureList::setNumberOfLongtermPictures(int numberOfLtrp)
{
  m_numberOfLongtermPictures = numberOfLtrp;
}

int ReferencePictureList::getNumberOfLongtermPictures() const
{
  return m_numberOfLongtermPictures;
}

void ReferencePictureList::setPOC(int idx, int POC)
{
  m_POC[idx] = POC;
}

int ReferencePictureList::getPOC(int idx) const
{
  return m_POC[idx];
}

void ReferencePictureList::setNumberOfActivePictures(int numberActive)
{
  m_numberOfActivePictures = numberActive;
}

int ReferencePictureList::getNumberOfActivePictures() const
{
  return m_numberOfActivePictures;
}

void ReferencePictureList::printRefPicInfo() const
{
  //DTRACE(g_trace_ctx, D_RPSINFO, "RefPics = { ");
  printf("RefPics = { ");
  int numRefPic = getNumberOfShorttermPictures() + getNumberOfLongtermPictures();
  for (int ii = 0; ii < numRefPic; ii++)
  {
    //DTRACE(g_trace_ctx, D_RPSINFO, "%d%s ", m_refPicIdentifier[ii], (m_isLongtermRefPic[ii] == 1) ? "[LT]" : "[ST]");
    printf("%d%s ", m_refPicIdentifier[ii], (m_isLongtermRefPic[ii] == 1) ? "[LT]" : "[ST]");
  }
  //DTRACE(g_trace_ctx, D_RPSINFO, "}\n");
  printf("}\n");
}

ScalingList::ScalingList()
{
  m_chromaScalingListPresentFlag = true;
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    m_scalingListCoef[scalingListId].resize(matrixSize*matrixSize);
  }
}

/** set default quantization matrix to array
*/
void ScalingList::setDefaultScalingList()
{
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    processDefaultMatrix(scalingListId);
  }
}
/** check if use default quantization matrix
 * \returns true if the scaling list is not equal to the default quantization matrix
*/
bool ScalingList::isNotDefaultScalingList()
{
  bool isAllDefault = true;
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    if (scalingListId < SCALING_LIST_1D_START_16x16)
    {
      if (::memcmp(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId), sizeof(int) * matrixSize * matrixSize))
      {
        isAllDefault = false;
        break;
      }
    }
    else
    {
      if ((::memcmp(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId), sizeof(int) * MAX_MATRIX_COEF_NUM)) || (getScalingListDC(scalingListId) != 16))
      {
        isAllDefault = false;
        break;
      }
    }
    if (!isAllDefault) break;
  }

  return !isAllDefault;
}

/** get scaling matrix from RefMatrixID
 * \param sizeId    size index
 * \param listId    index of input matrix
 * \param refListId index of reference matrix
 */
int ScalingList::lengthUvlc(int uiCode)
{
  if (uiCode < 0) printf("Error UVLC! \n");

  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK_(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}
int ScalingList::lengthSvlc(int uiCode)
{
  uint32_t uiCode2 = uint32_t(uiCode <= 0 ? (-uiCode) << 1 : (uiCode << 1) - 1);
  int uiLength = 1;
  int uiTemp = ++uiCode2;

  CHECK_(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}
void ScalingList::codePredScalingList(int* scalingList, const int* scalingListPred, int scalingListDC, int scalingListPredDC, int scalingListId, int& bitsCost) //sizeId, listId is current to-be-coded matrix idx
{
  int deltaValue = 0;
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  int coefNum = matrixSize*matrixSize;
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)][gp_sizeIdxInfo->idxFrom(matrixSize)];
  int nextCoef = 0;

  int8_t data;
  const int *src = scalingList;
  const int *srcPred = scalingListPred;
  if (scalingListDC!=-1 && scalingListPredDC!=-1)
  {
    bitsCost += lengthSvlc((int8_t)(scalingListDC - scalingListPredDC - nextCoef));
    nextCoef =  scalingListDC - scalingListPredDC;
  }
  else if ((scalingListDC != -1 && scalingListPredDC == -1))
  {
    bitsCost += lengthSvlc((int8_t)(scalingListDC - srcPred[scan[0].idx] - nextCoef));
    nextCoef =  scalingListDC - srcPred[scan[0].idx];
  }
  else if ((scalingListDC == -1 && scalingListPredDC == -1))
  {
  }
  else
  {
    printf("Predictor DC mismatch! \n");
  }
  for (int i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
      continue;
    deltaValue = (src[scan[i].idx] - srcPred[scan[i].idx]);
    data = (int8_t)(deltaValue - nextCoef);
    nextCoef = deltaValue;

    bitsCost += lengthSvlc(data);
  }
}
void ScalingList::codeScalingList(int* scalingList, int scalingListDC, int scalingListId, int& bitsCost) //sizeId, listId is current to-be-coded matrix idx
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  int coefNum = matrixSize * matrixSize;
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)][gp_sizeIdxInfo->idxFrom(matrixSize)];
  int nextCoef = SCALING_LIST_START_VALUE;
  int8_t data;
  const int *src = scalingList;

  if (scalingListId >= SCALING_LIST_1D_START_16x16)
  {
    bitsCost += lengthSvlc(int8_t(getScalingListDC(scalingListId) - nextCoef));
    nextCoef = getScalingListDC(scalingListId);
  }

  for (int i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
      continue;
    data = int8_t(src[scan[i].idx] - nextCoef);
    nextCoef = src[scan[i].idx];

    bitsCost += lengthSvlc(data);
  }
}
void ScalingList::CheckBestPredScalingList(int scalingListId, int predListId, int& BitsCount)
{
  //check previously coded matrix as a predictor, code "lengthUvlc" function
  int *scalingList = getScalingListAddress(scalingListId);
  const int *scalingListPred = (scalingListId == predListId) ? ((predListId < SCALING_LIST_1D_START_8x8) ? g_quantTSDefault4x4 : g_quantIntraDefault8x8) : getScalingListAddress(predListId);
  int scalingListDC = (scalingListId >= SCALING_LIST_1D_START_16x16) ? getScalingListDC(scalingListId) : -1;
  int scalingListPredDC = (predListId >= SCALING_LIST_1D_START_16x16) ? ((scalingListId == predListId) ? 16 : getScalingListDC(predListId)) : -1;

  int bitsCost = 0;
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  int predMatrixSize = (predListId < SCALING_LIST_1D_START_4x4) ? 2 : (predListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;

  if (matrixSize != predMatrixSize) printf("Predictor size mismatch! \n");

  bitsCost = 2 + lengthUvlc(scalingListId - predListId);
  //copy-flag + predictor-mode-flag + deltaListId
  codePredScalingList(scalingList, scalingListPred, scalingListDC, scalingListPredDC, scalingListId, bitsCost);
  BitsCount = bitsCost;
}
void ScalingList::processRefMatrix(uint32_t scalinListId, uint32_t refListId)
{
  int matrixSize = (scalinListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalinListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  ::memcpy(getScalingListAddress(scalinListId), ((scalinListId == refListId) ? getScalingListDefaultAddress(refListId) : getScalingListAddress(refListId)), sizeof(int)*matrixSize*matrixSize);
}
void ScalingList::checkPredMode(uint32_t scalingListId)
{
  int bestBitsCount = MAX_INT;
  int BitsCount = 2;
  setScalingListPreditorModeFlag(scalingListId, false);
  codeScalingList(getScalingListAddress(scalingListId), ((scalingListId >= SCALING_LIST_1D_START_16x16) ? getScalingListDC(scalingListId) : -1), scalingListId, BitsCount);
  bestBitsCount = BitsCount;

  for (int predListIdx = (int)scalingListId; predListIdx >= 0; predListIdx--)
  {

    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    int predMatrixSize = (predListIdx < SCALING_LIST_1D_START_4x4) ? 2 : (predListIdx < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    if (((scalingListId == SCALING_LIST_1D_START_2x2 || scalingListId == SCALING_LIST_1D_START_4x4 || scalingListId == SCALING_LIST_1D_START_8x8) && predListIdx != (int)scalingListId) || matrixSize != predMatrixSize)
      continue;
    const int* refScalingList = (scalingListId == predListIdx) ? getScalingListDefaultAddress(predListIdx) : getScalingListAddress(predListIdx);
    const int refDC = (predListIdx < SCALING_LIST_1D_START_16x16) ? refScalingList[0] : (scalingListId == predListIdx) ? 16 : getScalingListDC(predListIdx);
    if (!::memcmp(getScalingListAddress(scalingListId), refScalingList, sizeof(int)*matrixSize*matrixSize) // check value of matrix
      // check DC value
      && (scalingListId < SCALING_LIST_1D_START_16x16 || getScalingListDC(scalingListId) == refDC))
    {
      //copy mode
      setRefMatrixId(scalingListId, predListIdx);
      setScalingListCopyModeFlag(scalingListId, true);
      setScalingListPreditorModeFlag(scalingListId, false);
      return;
    }
    else
    {
      //predictor mode
      //use previously coded matrix as a predictor
      CheckBestPredScalingList(scalingListId, predListIdx, BitsCount);
      if (BitsCount < bestBitsCount)
      {
        bestBitsCount = BitsCount;
        setScalingListCopyModeFlag(scalingListId, false);
        setScalingListPreditorModeFlag(scalingListId, true);
        setRefMatrixId(scalingListId, predListIdx);
      }
    }
  }
  setScalingListCopyModeFlag(scalingListId, false);
}

static void outputScalingListHelp(std::ostream &os)
{
  os << "The scaling list file specifies all matrices and their DC values; none can be missing,\n"
         "but their order is arbitrary.\n\n"
         "The matrices are specified by:\n"
         "<matrix name><unchecked data>\n"
         "  <value>,<value>,<value>,....\n\n"
         "  Line-feeds can be added arbitrarily between values, and the number of values needs to be\n"
         "  at least the number of entries for the matrix (superfluous entries are ignored).\n"
         "  The <unchecked data> is text on the same line as the matrix that is not checked\n"
         "  except to ensure that the matrix name token is unique. It is recommended that it is ' ='\n"
         "  The values in the matrices are the absolute values (0-255), not the delta values as\n"
         "  exchanged between the encoder and decoder\n\n"
         "The DC values (for matrix sizes larger than 8x8) are specified by:\n"
         "<matrix name>_DC<unchecked data>\n"
         "  <value>\n";

  os << "The permitted matrix names are:\n";
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)
  {
    for (uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if (!(((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0)) || ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0))))
      {
        os << "  " << MatrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}

void ScalingList::outputScalingLists(std::ostream &os) const
{
  int scalingListId = 0;
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)
  {
    const uint32_t size = (sizeIdc == 1) ? 2 : ((sizeIdc == 2) ? 4 : 8);
    for(uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if (!((sizeIdc== SCALING_LIST_64x64 && listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0) || (sizeIdc == SCALING_LIST_2x2 && listIdc < 4)))
      {
        const int *src = getScalingListAddress(scalingListId);
        os << (MatrixType[sizeIdc][listIdc]) << " =\n  ";
        for(uint32_t y=0; y<size; y++)
        {
          for(uint32_t x=0; x<size; x++, src++)
          {
            os << std::setw(3) << (*src) << ", ";
          }
          os << (y+1<size?"\n  ":"\n");
        }
        if(sizeIdc > SCALING_LIST_8x8)
        {
          os << MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw(3) << getScalingListDC(scalingListId) << "\n";
        }
        os << "\n";
        scalingListId++;
      }
    }
  }
}

bool ScalingList::xParseScalingList(const std::string &fileName)
{
  static const int LINE_SIZE=1024;
  FILE *fp = NULL;
  char line[LINE_SIZE];

  if (fileName.empty())
  {
    msg(ERROR_, "Error: no scaling list file specified. Help on scaling lists being output\n");
    outputScalingListHelp(std::cout);
    std::cout << "\n\nExample scaling list file using default values:\n\n";
    outputScalingLists(std::cout);
    return true;
  }
  else if ((fp = fopen(fileName.c_str(),"r")) == (FILE*)NULL)
  {
    msg(ERROR_, "Error: cannot open scaling list file %s for reading\n", fileName.c_str());
    return true;
  }

  int scalingListId = 0;
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)//2x2-128x128
  {
    const uint32_t size = std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeIdc]);

    for(uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {

      if ((sizeIdc == SCALING_LIST_64x64 && listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0) || (sizeIdc == SCALING_LIST_2x2 && listIdc < 4))
      {
        continue;
      }
      else
      {
        int * const src = getScalingListAddress(scalingListId);
        {
          fseek(fp, 0, SEEK_SET);
          bool bFound=false;
          while ((!feof(fp)) && (!bFound))
          {
            char *ret = fgets(line, LINE_SIZE, fp);
            char *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType[sizeIdc][listIdc]);
            // This could be a match against the DC string as well, so verify it isn't
            if (findNamePosition!= NULL && (MatrixType_DC[sizeIdc][listIdc]==NULL || strstr(line, MatrixType_DC[sizeIdc][listIdc])==NULL))
            {
              bFound=true;
            }
          }
          if (!bFound)
          {
            msg(ERROR_, "Error: cannot find Matrix %s from scaling list file %s\n", MatrixType[sizeIdc][listIdc], fileName.c_str());
            return true;

          }
        }
        for (uint32_t i=0; i<size; i++)
        {
          int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            msg(ERROR_, "Error: cannot read value #%d for Matrix %s from scaling list file %s at file position %ld\n", i, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            msg(ERROR_, "Error: QMatrix entry #%d of value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", i, data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          src[i] = data;
        }

        //set DC value for default matrix check
        setScalingListDC(scalingListId, src[0]);

        if(sizeIdc > SCALING_LIST_8x8)
        {
          {
            fseek(fp, 0, SEEK_SET);
            bool bFound=false;
            while ((!feof(fp)) && (!bFound))
            {
              char *ret = fgets(line, LINE_SIZE, fp);
              char *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType_DC[sizeIdc][listIdc]);
              if (findNamePosition!= NULL)
              {
                // This won't be a match against the non-DC string.
                bFound=true;
              }
            }
            if (!bFound)
            {
              msg(ERROR_, "Error: cannot find DC Matrix %s from scaling list file %s\n", MatrixType_DC[sizeIdc][listIdc], fileName.c_str());
              return true;
            }
          }
          int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            msg(ERROR_, "Error: cannot read DC %s from scaling list file %s at file position %ld\n", MatrixType_DC[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            msg(ERROR_, "Error: DC value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          //overwrite DC value when size of matrix is larger than 16x16
          setScalingListDC(scalingListId, data);
        }
      }
      scalingListId++;
    }
  }
//  std::cout << "\n\nRead scaling lists of:\n\n";
//  outputScalingLists(std::cout);

  fclose(fp);
  return false;
}


/** get default address of quantization matrix
 * \param sizeId size index
 * \param listId list index
 * \returns pointer of quantization matrix
 */
const int* ScalingList::getScalingListDefaultAddress(uint32_t scalingListId)
{
  const int *src = 0;
  int sizeId = (scalingListId < SCALING_LIST_1D_START_8x8) ? 2 : 3;
  switch (sizeId)
  {
    case SCALING_LIST_1x1:
    case SCALING_LIST_2x2:
    case SCALING_LIST_4x4:
      src = g_quantTSDefault4x4;
      break;
    case SCALING_LIST_8x8:
    case SCALING_LIST_16x16:
    case SCALING_LIST_32x32:
    case SCALING_LIST_64x64:
    case SCALING_LIST_128x128:
      src = g_quantInterDefault8x8;
      break;
    default:
      THROW( "Invalid scaling list" );
      src = NULL;
      break;
  }
  return src;
}

/** process of default matrix
 * \param sizeId size index
 * \param listId index of input matrix
 */
void ScalingList::processDefaultMatrix(uint32_t scalingListId)
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  ::memcpy(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId), sizeof(int)*matrixSize*matrixSize);
  setScalingListDC(scalingListId, SCALING_LIST_DC);
}

/** check DC value of matrix for default matrix signaling
 */
void ScalingList::checkDcOfMatrix()
{
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    //check default matrix?
    if (getScalingListDC(scalingListId) == 0)
    {
      processDefaultMatrix(scalingListId);
    }
  }
}

bool ScalingList::isLumaScalingList( int scalingListId) const
{
  return (scalingListId % MAX_NUM_COMPONENT == SCALING_LIST_1D_START_4x4 || scalingListId == SCALING_LIST_1D_START_64x64 + 1);
}

uint32_t PreCalcValues::getValIdx( const Slice &slice, const ChannelType chType ) const
{
  return slice.isIntra() ? ( ISingleTree ? 0 : ( chType << 1 ) ) : 1;
}

uint32_t PreCalcValues::getMaxBtDepth( const Slice &slice, const ChannelType chType ) const
{
  if ( slice.getPicHeader()->getSplitConsOverrideFlag() )
    return slice.getPicHeader()->getMaxMTTHierarchyDepth( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
  else
  return maxBtDepth[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMinBtSize( const Slice &slice, const ChannelType chType ) const
{
  return minBtSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMaxBtSize( const Slice &slice, const ChannelType chType ) const
{
  if (slice.getPicHeader()->getSplitConsOverrideFlag())
    return slice.getPicHeader()->getMaxBTSize( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
  else
    return maxBtSize[getValIdx(slice, chType)];
}

uint32_t PreCalcValues::getMinTtSize( const Slice &slice, const ChannelType chType ) const
{
  return minTtSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMaxTtSize( const Slice &slice, const ChannelType chType ) const
{
  if (slice.getPicHeader()->getSplitConsOverrideFlag())
    return slice.getPicHeader()->getMaxTTSize( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
  else
  return maxTtSize[getValIdx( slice, chType )];
}
uint32_t PreCalcValues::getMinQtSize( const Slice &slice, const ChannelType chType ) const
{
  if (slice.getPicHeader()->getSplitConsOverrideFlag())
    return slice.getPicHeader()->getMinQTSize( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
  else
  return minQtSize[getValIdx( slice, chType )];
}

void Slice::scaleRefPicList( Picture *scaledRefPic[ ], PicHeader *picHeader, APS** apss, APS* lmcsAps, APS* scalingListAps, const bool isDecoder )
{
  int i;
  const SPS* sps = getSPS();
  const PPS* pps = getPPS();

  bool refPicIsSameRes = false;

  // this is needed for IBC
  m_pcPic->unscaledPic = m_pcPic;

  if( m_eSliceType == I_SLICE )
  {
    return;
  }

  freeScaledRefPicList( scaledRefPic );

  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      // if rescaling is needed, otherwise just reuse the original picture pointer; it is needed for motion field, otherwise motion field requires a copy as well
      // reference resampling for the whole picture is not applied at decoder

      int xScale, yScale;
      CU::getRprScaling( sps, pps, m_apcRefPicList[refList][rIdx], xScale, yScale );
      m_scalingRatio[refList][rIdx] = std::pair<int, int>( xScale, yScale );

      CHECK_( m_apcRefPicList[refList][rIdx]->unscaledPic == nullptr, "unscaledPic is not properly set" );

      if( m_apcRefPicList[refList][rIdx]->isRefScaled( pps ) == false )
      {
        refPicIsSameRes = true;
      }

      if( m_scalingRatio[refList][rIdx] == SCALE_1X || isDecoder )
      {
        m_scaledRefPicList[refList][rIdx] = m_apcRefPicList[refList][rIdx];
      }
      else
      {
        int poc = m_apcRefPicList[refList][rIdx]->getPOC();
        int layerId = m_apcRefPicList[refList][rIdx]->layerId;

        // check whether the reference picture has already been scaled
        for( i = 0; i < MAX_NUM_REF; i++ )
        {
          if( scaledRefPic[i] != nullptr && scaledRefPic[i]->poc == poc && scaledRefPic[i]->layerId == layerId )
          {
            break;
          }
        }

        if( i == MAX_NUM_REF )
        {
          int j;
          // search for unused Picture structure in scaledRefPic
          for( j = 0; j < MAX_NUM_REF; j++ )
          {
            if( scaledRefPic[j] == nullptr )
            {
              break;
            }
          }

          CHECK_( j >= MAX_NUM_REF, "scaledRefPic can not hold all reference pictures!" );

          if( j >= MAX_NUM_REF )
          {
            j = 0;
          }

          if( scaledRefPic[j] == nullptr )
          {
            scaledRefPic[j] = new Picture;

            scaledRefPic[j]->setBorderExtension( false );
            scaledRefPic[j]->reconstructed = false;
            scaledRefPic[j]->referenced = true;

            scaledRefPic[j]->finalInit( m_pcPic->cs->vps, *sps, *pps, picHeader, apss, lmcsAps, scalingListAps );

            scaledRefPic[j]->poc = NOT_VALID;

            scaledRefPic[j]->create( sps->getChromaFormatIdc(), Size( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples() ), sps->getMaxCUWidth(), sps->getMaxCUWidth() + 16, isDecoder, layerId );
          }

          scaledRefPic[j]->poc = poc;
          scaledRefPic[j]->longTerm = m_apcRefPicList[refList][rIdx]->longTerm;

          // rescale the reference picture
          const bool downsampling = m_apcRefPicList[refList][rIdx]->getRecoBuf().Y().width >= scaledRefPic[j]->getRecoBuf().Y().width && m_apcRefPicList[refList][rIdx]->getRecoBuf().Y().height >= scaledRefPic[j]->getRecoBuf().Y().height;
          Picture::rescalePicture( m_scalingRatio[refList][rIdx],
                                   m_apcRefPicList[refList][rIdx]->getRecoBuf(), m_apcRefPicList[refList][rIdx]->slices[0]->getPPS()->getScalingWindow(),
                                   scaledRefPic[j]->getRecoBuf(), pps->getScalingWindow(),
                                   sps->getChromaFormatIdc(), sps->getBitDepths(), true, downsampling,
                                   sps->getHorCollocatedChromaFlag(), sps->getVerCollocatedChromaFlag() );
          scaledRefPic[j]->unscaledPic = m_apcRefPicList[refList][rIdx];
          scaledRefPic[j]->extendPicBorder( getPPS() );

          m_scaledRefPicList[refList][rIdx] = scaledRefPic[j];
        }
        else
        {
          m_scaledRefPicList[refList][rIdx] = scaledRefPic[i];
        }
      }
    }
  }

  // make the scaled reference picture list as the default reference picture list
  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      m_savedRefPicList[refList][rIdx] = m_apcRefPicList[refList][rIdx];
      m_apcRefPicList[refList][rIdx] = m_scaledRefPicList[refList][rIdx];

      // allow the access of the unscaled version in xPredInterBlk()
      m_apcRefPicList[refList][rIdx]->unscaledPic = m_savedRefPicList[refList][rIdx];
    }
  }

  //Make sure that TMVP is disabled when there are no reference pictures with the same resolution
  if(!refPicIsSameRes)
  {
    CHECK_(getPicHeader()->getEnableTMVPFlag() != 0, "TMVP cannot be enabled in pictures that have no reference pictures with the same resolution")
  }
}

void Slice::freeScaledRefPicList( Picture *scaledRefPic[] )
{
  if( m_eSliceType == I_SLICE )
  {
    return;
  }
  for( int i = 0; i < MAX_NUM_REF; i++ )
  {
    if( scaledRefPic[i] != nullptr )
    {
      scaledRefPic[i]->destroy();
      scaledRefPic[i] = nullptr;
    }
  }
}

bool Slice::checkRPR()
{
  const PPS* pps = getPPS();

  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {

    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      if( m_scaledRefPicList[refList][rIdx]->cs->pcv->lumaWidth != pps->getPicWidthInLumaSamples() || m_scaledRefPicList[refList][rIdx]->cs->pcv->lumaHeight != pps->getPicHeightInLumaSamples() )
      {
        return true;
      }
    }
  }

  return false;
}

bool             operator == (const ConstraintInfo& op1, const ConstraintInfo& op2)
{
  if( op1.m_intraOnlyConstraintFlag                      != op2.m_intraOnlyConstraintFlag                        ) return false;
  if( op1.m_maxBitDepthConstraintIdc                     != op2.m_maxBitDepthConstraintIdc                       ) return false;
  if( op1.m_maxChromaFormatConstraintIdc                 != op2.m_maxChromaFormatConstraintIdc                   ) return false;
  if( op1.m_onePictureOnlyConstraintFlag                 != op2.m_onePictureOnlyConstraintFlag                   ) return false;
  if( op1.m_lowerBitRateConstraintFlag                   != op2.m_lowerBitRateConstraintFlag                     ) return false;
  if (op1.m_allLayersIndependentConstraintFlag           != op2.m_allLayersIndependentConstraintFlag             ) return false;
  if (op1.m_noMrlConstraintFlag                          != op2.m_noMrlConstraintFlag                            ) return false;
  if (op1.m_noIspConstraintFlag                          != op2.m_noIspConstraintFlag                            ) return false;
  if (op1.m_noMipConstraintFlag                          != op2.m_noMipConstraintFlag                            ) return false;
  if (op1.m_noLfnstConstraintFlag                        != op2.m_noLfnstConstraintFlag                          ) return false;
  if (op1.m_noMmvdConstraintFlag                         != op2.m_noMmvdConstraintFlag                           ) return false;
  if (op1.m_noSmvdConstraintFlag                         != op2.m_noSmvdConstraintFlag                           ) return false;
  if (op1.m_noProfConstraintFlag                         != op2.m_noProfConstraintFlag                           ) return false;
  if (op1.m_noPaletteConstraintFlag                      != op2.m_noPaletteConstraintFlag                        ) return false;
  if (op1.m_noActConstraintFlag                          != op2.m_noActConstraintFlag                            ) return false;
  if (op1.m_noLmcsConstraintFlag                         != op2.m_noLmcsConstraintFlag                           ) return false;
  if (op1.m_noExplicitScaleListConstraintFlag            != op2.m_noExplicitScaleListConstraintFlag              ) return false;
  if (op1.m_noVirtualBoundaryConstraintFlag              != op2.m_noVirtualBoundaryConstraintFlag                ) return false;
  if (op1.m_noChromaQpOffsetConstraintFlag               != op2.m_noChromaQpOffsetConstraintFlag                 ) return false;
  if (op1.m_noRprConstraintFlag                          != op2.m_noRprConstraintFlag                            ) return false;
  if (op1.m_noResChangeInClvsConstraintFlag              != op2.m_noResChangeInClvsConstraintFlag                ) return false;
  if (op1.m_noMttConstraintFlag                          != op2.m_noMttConstraintFlag                            ) return false;
  if( op1.m_noQtbttDualTreeIntraConstraintFlag           != op2.m_noQtbttDualTreeIntraConstraintFlag             ) return false;
  if( op1.m_noPartitionConstraintsOverrideConstraintFlag != op2.m_noPartitionConstraintsOverrideConstraintFlag   ) return false;
  if( op1.m_noSaoConstraintFlag                          != op2.m_noSaoConstraintFlag                            ) return false;
  if( op1.m_noAlfConstraintFlag                          != op2.m_noAlfConstraintFlag                            ) return false;
  if( op1.m_noCCAlfConstraintFlag                        != op2.m_noCCAlfConstraintFlag                          ) return false;
  if (op1.m_noWeightedPredictionConstraintFlag           != op2.m_noWeightedPredictionConstraintFlag             ) return false;
  if( op1.m_noRefWraparoundConstraintFlag                != op2.m_noRefWraparoundConstraintFlag                  ) return false;
  if( op1.m_noTemporalMvpConstraintFlag                  != op2.m_noTemporalMvpConstraintFlag                    ) return false;
  if( op1.m_noSbtmvpConstraintFlag                       != op2.m_noSbtmvpConstraintFlag                         ) return false;
  if( op1.m_noAmvrConstraintFlag                         != op2.m_noAmvrConstraintFlag                           ) return false;
  if( op1.m_noBdofConstraintFlag                         != op2.m_noBdofConstraintFlag                           ) return false;
  if( op1.m_noDmvrConstraintFlag                         != op2.m_noDmvrConstraintFlag                           ) return false;
  if( op1.m_noCclmConstraintFlag                         != op2.m_noCclmConstraintFlag                           ) return false;
  if( op1.m_noMtsConstraintFlag                          != op2.m_noMtsConstraintFlag                            ) return false;
  if( op1.m_noSbtConstraintFlag                          != op2.m_noSbtConstraintFlag                            ) return false;
  if( op1.m_noAffineMotionConstraintFlag                 != op2.m_noAffineMotionConstraintFlag                   ) return false;
  if( op1.m_noBcwConstraintFlag                          != op2.m_noBcwConstraintFlag                            ) return false;
  if( op1.m_noIbcConstraintFlag                          != op2.m_noIbcConstraintFlag                            ) return false;
  if( op1.m_noCiipConstraintFlag                         != op2.m_noCiipConstraintFlag                           ) return false;
  if( op1.m_noLadfConstraintFlag                         != op2.m_noLadfConstraintFlag                           ) return false;
  if( op1.m_noTransformSkipConstraintFlag                != op2.m_noTransformSkipConstraintFlag                  ) return false;
  if( op1.m_noBDPCMConstraintFlag                        != op2.m_noBDPCMConstraintFlag                          ) return false;
  if( op1.m_noJointCbCrConstraintFlag                    != op2.m_noJointCbCrConstraintFlag                      ) return false;
  if( op1.m_noCuQpDeltaConstraintFlag                    != op2.m_noCuQpDeltaConstraintFlag                      ) return false;
  if( op1.m_noDepQuantConstraintFlag                     != op2.m_noDepQuantConstraintFlag                       ) return false;
  if( op1.m_noSignDataHidingConstraintFlag               != op2.m_noSignDataHidingConstraintFlag                 ) return false;
  if( op1.m_noTrailConstraintFlag                        != op2.m_noTrailConstraintFlag                          ) return false;
  if( op1.m_noStsaConstraintFlag                         != op2.m_noStsaConstraintFlag                           ) return false;
  if( op1.m_noRaslConstraintFlag                         != op2.m_noRaslConstraintFlag                           ) return false;
  if( op1.m_noRadlConstraintFlag                         != op2.m_noRadlConstraintFlag                           ) return false;
  if( op1.m_noIdrConstraintFlag                          != op2.m_noIdrConstraintFlag                            ) return false;
  if( op1.m_noCraConstraintFlag                          != op2.m_noCraConstraintFlag                            ) return false;
  if( op1.m_noGdrConstraintFlag                          != op2.m_noGdrConstraintFlag                            ) return false;
  if( op1.m_noApsConstraintFlag                          != op2.m_noApsConstraintFlag                            ) return false;
  return true;
}
bool             operator != (const ConstraintInfo& op1, const ConstraintInfo& op2)
{
  return !(op1 == op2);
}

bool             operator == (const ProfileTierLevel& op1, const ProfileTierLevel& op2)
{
  if (op1.m_tierFlag        != op2.m_tierFlag) return false;
  if (op1.m_profileIdc      != op2.m_profileIdc) return false;
  if (op1.m_numSubProfile   != op2.m_numSubProfile) return false;
  if (op1.m_levelIdc        != op2.m_levelIdc) return false;
  if (op1.m_frameOnlyConstraintFlag != op2.m_frameOnlyConstraintFlag) return false;
  if (op1.m_multiLayerEnabledFlag   != op2.m_multiLayerEnabledFlag) return false;
  if (op1.m_constraintInfo  != op2.m_constraintInfo) return false;
  if (op1.m_subProfileIdc   != op2.m_subProfileIdc) return false;

  for (int i = 0; i < MAX_TLAYER - 1; i++)
  {
    if (op1.m_subLayerLevelPresentFlag[i] != op2.m_subLayerLevelPresentFlag[i])
    {
      return false;
    }
  }
  for (int i = 0; i < MAX_TLAYER; i++)
  {
    if (op1.m_subLayerLevelIdc[i] != op2.m_subLayerLevelIdc[i])
    {
      return false;
    }
  }
  return true;
}
bool             operator != (const ProfileTierLevel& op1, const ProfileTierLevel& op2)
{
  return !(op1 == op2);
}

bool Slice::isLastSliceInSubpic()
{
  CHECK_(m_pcPPS == NULL, "PPS pointer not initialized");

  int lastCTUAddrInSlice = m_sliceMap.getCtuAddrList().back();

  if (m_pcPPS->getNumSubPics() > 1)
  {
    const SubPic& subpic = m_pcPPS->getSubPic(m_pcPPS->getSubPicIdxFromSubPicId(getSliceSubPicId()));
    return subpic.isLastCTUinSubPic(lastCTUAddrInSlice);
  }
  else
  {
    const CodingStructure *cs = m_pcPic->cs;
    const PreCalcValues* pcv = cs->pcv;
    const uint32_t picSizeInCtus   = pcv->heightInCtus * pcv->widthInCtus;
    return lastCTUAddrInSlice == (picSizeInCtus-1);
  }
}


#if ENABLE_TRACING
void xTraceVPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Video Parameter Set     ===========\n" );
}

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
void xTraceOPIHeader()
{
  DTRACE(g_trace_ctx, D_HEADER, "=========== Operating Point Information     ===========\n");
}
#endif

void xTraceDCIHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Decoding Capability Information     ===========\n" );
}

void xTraceSPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Sequence Parameter Set  ===========\n" );
}

void xTracePPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Parameter Set  ===========\n" );
}

void xTraceAPSHeader()
{
  DTRACE(g_trace_ctx, D_HEADER, "=========== Adaptation Parameter Set  ===========\n");
}

void xTracePictureHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Header ===========\n" );
}

void xTraceSliceHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Slice ===========\n" );
}

void xTraceAccessUnitDelimiter()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n" );
}

void xTraceFillerData ()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Filler Data ===========\n" );
}
#endif

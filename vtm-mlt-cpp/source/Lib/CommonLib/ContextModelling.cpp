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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"


CoeffCodingContext::CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm )
  : m_compID                    (component)
  , m_chType                    (toChannelType(m_compID))
  , m_width                     (tu.block(m_compID).width)
  , m_height                    (tu.block(m_compID).height)
  , m_log2CGWidth               ( g_log2SbbSize[ floorLog2(m_width) ][ floorLog2(m_height) ][0] )
  , m_log2CGHeight              ( g_log2SbbSize[ floorLog2(m_width) ][ floorLog2(m_height) ][1] )
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups(std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) >> m_log2CGWidth)
  , m_heightInGroups(std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) >> m_log2CGHeight)
  , m_log2BlockWidth            ((unsigned)floorLog2(m_width))
  , m_log2BlockHeight           ((unsigned)floorLog2(m_height))
  , m_maxNumCoeff               (m_width * m_height)
  , m_signHiding                (signHide)
  , m_extendedPrecision         (tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  , m_maxLog2TrDynamicRange     (tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scanType                  (SCAN_DIAG)
  , m_scan                      (g_scanOrder     [SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )])
  , m_scanCG                    (g_scanOrder     [SCAN_UNGROUPED  ][m_scanType][gp_sizeIdxInfo->idxFrom(m_widthInGroups)][gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
  , m_CtxSetLastX               (Ctx::LastX[m_chType])
  , m_CtxSetLastY               (Ctx::LastY[m_chType])
  , m_maxLastPosX(g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) - 1])
  , m_maxLastPosY(g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) - 1])
  , m_lastOffsetX               (0)
  , m_lastOffsetY               (0)
  , m_lastShiftX                (0)
  , m_lastShiftY                (0)
  , m_TrafoBypass               (tu.cs->sps->getSpsRangeExtension().getTransformSkipContextEnabledFlag() && (tu.mtsIdx[m_compID] == MTS_SKIP))
  , m_minCoeff                  (-(1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)))
  , m_maxCoeff                  ((1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)) - 1)
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
  , m_tmplCpSum1                (-1)
  , m_tmplCpDiag                (-1)
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType+4] }
  , m_parFlagCtxSet             ( Ctx::ParFlag[m_chType] )
  , m_gtxFlagCtxSet             { Ctx::GtxFlag[m_chType], Ctx::GtxFlag[m_chType+2] }
  , m_sigGroupCtxIdTS           (-1)
  , m_tsSigFlagCtxSet           ( Ctx::TsSigFlag )
  , m_tsParFlagCtxSet           ( Ctx::TsParFlag )
  , m_tsGtxFlagCtxSet           ( Ctx::TsGtxFlag )
  , m_tsLrg1FlagCtxSet          (Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet          (Ctx::TsResidualSign)
  , m_sigCoeffGroupFlag         ()
  , m_bdpcm                     (bdpcm)
{
  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
  if (m_chType == CHANNEL_TYPE_CHROMA)
  {
    const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( m_width  >> 3) );
    const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( m_height >> 3) );
  }
  else
  {
    static const int prefix_ctx[8]  = { 0, 0, 0, 3, 6, 10, 15, 21 };
    const_cast<int&>(m_lastOffsetX) = prefix_ctx[ log2sizeX ];
    const_cast<int&>(m_lastOffsetY) = prefix_ctx[ log2sizeY ];;
    const_cast<int&>(m_lastShiftX)  = (log2sizeX + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (log2sizeY + 1) >> 2;
  }
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[m_subSetId].idx;
  m_subSetPosY              = m_subSetPos / m_widthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
  unsigned  sigLeft   = unsigned( CGPosX > 0 ? m_sigCoeffGroupFlag[m_subSetPos - 1              ] : false );
  unsigned  sigAbove  = unsigned( CGPosY > 0 ? m_sigCoeffGroupFlag[m_subSetPos - m_widthInGroups] : false );
  m_sigGroupCtxIdTS   = Ctx::TsSigCoeffGroup( sigLeft  + sigAbove );
}


unsigned DeriveCtx::CtxModeConsFlag( const CodingStructure& cs, Partitioner& partitioner )
{
  assert( partitioner.chType == CHANNEL_TYPE_LUMA );
  const Position pos = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  unsigned ctxId = ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;
  return ctxId;
}


void DeriveCtx::CtxSplit( const CodingStructure& cs, Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, bool* _canSplit /*= nullptr */ )
{
  const Position pos         = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx  = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  // get left depth
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  // get above depth
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  bool canSplit[6];

  if( _canSplit == nullptr )
  {
    partitioner.canSplit( cs, canSplit[0], canSplit[1], canSplit[2], canSplit[3], canSplit[4], canSplit[5] );
  }
  else
  {
    memcpy( canSplit, _canSplit, 6 * sizeof( bool ) );
  }

  ///////////////////////
  // CTX do split (0-8)
  ///////////////////////
  const unsigned widthCurr  = partitioner.currArea().blocks[partitioner.chType].width;
  const unsigned heightCurr = partitioner.currArea().blocks[partitioner.chType].height;

  ctxSpl = 0;

  if( cuLeft )
  {
    const unsigned heightLeft = cuLeft->blocks[partitioner.chType].height;
    ctxSpl += ( heightLeft < heightCurr ? 1 : 0 );
  }
  if( cuAbove )
  {
    const unsigned widthAbove = cuAbove->blocks[partitioner.chType].width;
    ctxSpl += ( widthAbove < widthCurr ? 1 : 0 );
  }

  unsigned numSplit = 0;
  if (canSplit[1])
  {
    numSplit += 2;
  }
  if (canSplit[2])
  {
    numSplit += 1;
  }
  if (canSplit[3])
  {
    numSplit += 1;
  }
  if (canSplit[4])
  {
    numSplit += 1;
  }
  if (canSplit[5])
  {
    numSplit += 1;
  }

  if (numSplit > 0)
  {
    numSplit--;
  }

  ctxSpl += 3 * ( numSplit >> 1 );

  //////////////////////////
  // CTX is qt split (0-5)
  //////////////////////////
  ctxQt =  ( cuLeft  && cuLeft->qtDepth  > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += partitioner.currQtDepth < 2 ? 0 : 3;

  ////////////////////////////
  // CTX is ver split (0-4)
  ////////////////////////////
  ctxHv = 0;

  const unsigned numHor = ( canSplit[2] ? 1 : 0 ) + ( canSplit[4] ? 1 : 0 );
  const unsigned numVer = ( canSplit[3] ? 1 : 0 ) + ( canSplit[5] ? 1 : 0 );

  if( numVer == numHor )
  {
    const Area& area = partitioner.currArea().blocks[partitioner.chType];

    const unsigned wAbove       = cuAbove ? cuAbove->blocks[partitioner.chType].width  : 1;
    const unsigned hLeft        = cuLeft  ? cuLeft ->blocks[partitioner.chType].height : 1;

    const unsigned depAbove     = area.width / wAbove;
    const unsigned depLeft      = area.height / hLeft;

    if (depAbove == depLeft || !cuLeft || !cuAbove)
    {
      ctxHv = 0;
    }
    else if (depAbove < depLeft)
    {
      ctxHv = 1;
    }
    else
    {
      ctxHv = 2;
    }
  }
  else if( numVer < numHor )
  {
    ctxHv = 3;
  }
  else
  {
    ctxHv = 4;
  }

  //////////////////////////
  // CTX is h/v bt (0-3)
  //////////////////////////
  ctxHorBt = ( partitioner.currMtDepth <= 1 ? 1 : 0 );
  ctxVerBt = ( partitioner.currMtDepth <= 1 ? 3 : 2 );
}

unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const bool prevCbf, const int ispIdx )
{
  if( ispIdx && isLuma( compID ) )
  {
    return 2 + (int)prevCbf;
  }
  if( compID == COMPONENT_Cr )
  {
    return ( prevCbf ? 1 : 0 );
  }
  return 0;
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
  return ( 7 - ((floorLog2(pu.lumaSize().width) + floorLog2(pu.lumaSize().height) + 1) >> 1) );
}

unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->affine ) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->affine ) ? 1 : 0;

  return ctxId;
}
unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->skip ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->skip ) ? 1 : 0;

  return ctxId;
}



unsigned DeriveCtx::CtxPredModeFlag( const CodingUnit& cu )
{
  const CodingUnit *cuLeft  = cu.cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  const CodingUnit *cuAbove = cu.cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);

  unsigned ctxId = ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxIBCFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;
  const Position pos = cu.chType == CHANNEL_TYPE_CHROMA ? cu.chromaPos() : cu.lumaPos();
  const CodingUnit *cuLeft = cs->getCURestricted(pos.offset(-1, 0), cu, cu.chType);
  ctxId += (cuLeft && CU::isIBC(*cuLeft)) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(pos.offset(0, -1), cu, cu.chType);
  ctxId += (cuAbove && CU::isIBC(*cuAbove)) ? 1 : 0;
  return ctxId;
}

void MergeCtx::setMergeInfo( PredictionUnit& pu, int candIdx )
{
  CHECK_( candIdx >= numValidMergeCand, "Merge candidate does not exist" );
  pu.regularMergeFlag        = !(pu.ciipFlag || pu.cu->geoFlag);
  pu.mergeFlag               = true;
  pu.mmvdMergeFlag = false;
  pu.interDir                = interDirNeighbours[candIdx];
  pu.cu->imv = (!pu.cu->geoFlag && useAltHpelIf[candIdx]) ? IMV_HPEL : 0;
  pu.mergeIdx                = candIdx;
  pu.mergeType               = mrgTypeNeighbours[candIdx];
  pu.mv     [REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  pu.mv     [REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
  pu.mvd    [REF_PIC_LIST_0] = Mv();
  pu.mvd    [REF_PIC_LIST_1] = Mv();
  pu.refIdx [REF_PIC_LIST_0] = mvFieldNeighbours[( candIdx << 1 ) + 0].refIdx;
  pu.refIdx [REF_PIC_LIST_1] = mvFieldNeighbours[( candIdx << 1 ) + 1].refIdx;
  pu.mvpIdx [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx [REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_1] = NOT_VALID;
  if (CU::isIBC(*pu.cu))
  {
    pu.bv = pu.mv[REF_PIC_LIST_0];
    pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // used for only integer resolution
    pu.cu->imv = pu.cu->imv == IMV_HPEL ? 0 : pu.cu->imv;
  }
  pu.cu->BcwIdx = ( interDirNeighbours[candIdx] == 3 ) ? BcwIdx[candIdx] : BCW_DEFAULT;

  PU::restrictBiPredMergeCandsOne(pu);
  pu.mmvdEncOptMode = 0;
}
void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx)
{
  const Slice &slice = *pu.cs->slice;
  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
  const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  int fPosGroup = 0;
  int fPosBaseIdx = 0;
  int fPosStep = 0;
  int tempIdx = 0;
  int fPosPosition = 0;
  Mv tempMv[2];

  tempIdx = candIdx;
  fPosGroup = tempIdx / (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  tempIdx = tempIdx - fPosGroup * (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  fPosBaseIdx = tempIdx / MMVD_MAX_REFINE_NUM;
  tempIdx = tempIdx - fPosBaseIdx * (MMVD_MAX_REFINE_NUM);
  fPosStep = tempIdx / 4;
  fPosPosition = tempIdx - fPosStep * (4);
  int offset = refMvdCands[fPosStep];
  if ( pu.cu->slice->getPicHeader()->getDisFracMMVD() )
  {
    offset <<= 2;
  }
  const int refList0 = mmvdBaseMv[fPosBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[fPosBaseIdx][1].refIdx;

  if ((refList0 != -1) && (refList1 != -1))
  {
    const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
    const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
    const int currPoc = slice.getPOC();
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      tempMv[1] = tempMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      tempMv[1] = tempMv[0];
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set(-1 * tempMv[1].getHor(), -1 * tempMv[1].getVer());
        }
      }
      else
      tempMv[0] = tempMv[1].scaleMv(scale);
    }
    else
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
        }
      }
      else
      tempMv[1] = tempMv[0].scaleMv(scale);
    }

    pu.interDir = 3;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }
  else if (refList0 != -1)
  {
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
    pu.interDir = 1;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
  }
  else if (refList1 != -1)
  {
    if (fPosPosition == 0)
    {
      tempMv[1] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[1] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[1] = Mv(0, offset);
    }
    else
    {
      tempMv[1] = Mv(0, -offset);
    }
    pu.interDir = 2;
    pu.mv[REF_PIC_LIST_0] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_0] = -1;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }

  pu.mmvdMergeFlag = true;
  pu.mmvdMergeIdx = candIdx;
  pu.mergeFlag = true;
  pu.regularMergeFlag = true;
  pu.mergeIdx = candIdx;
  pu.mergeType = MRG_TYPE_DEFAULT_N;
  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.cu->imv = mmvdUseAltHpelIf[fPosBaseIdx] ? IMV_HPEL : 0;

  pu.cu->BcwIdx = (interDirNeighbours[fPosBaseIdx] == 3) ? BcwIdx[fPosBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      pu.mv[refList].clipToStorageBitDepth();
    }
  }


  PU::restrictBiPredMergeCandsOne(pu);
}

unsigned DeriveCtx::CtxMipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = (cuLeft && cuLeft->mipFlag) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += (cuAbove && cuAbove->mipFlag) ? 1 : 0;

  ctxId  = (cu.lwidth() > 2*cu.lheight() || cu.lheight() > 2*cu.lwidth()) ? 3 : ctxId;

  return ctxId;
}

unsigned DeriveCtx::CtxPltCopyFlag( const unsigned prevRunType, const unsigned dist )
{
  uint8_t *ucCtxLut = (prevRunType == PLT_RUN_INDEX) ? g_paletteRunLeftLut : g_paletteRunTopLut;
  if ( dist <= RUN_IDX_THRE )
  {
     return ucCtxLut[dist];
  }
  else
  {
    return ucCtxLut[RUN_IDX_THRE];
  }
}

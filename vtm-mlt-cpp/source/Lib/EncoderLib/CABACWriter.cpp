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

/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CommonLib/Contexts.h"
#include "CABACWriter.h"

#include "EncLib.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"

#include <map>
#include <algorithm>
#include <limits>


//! \ingroup EncoderLib
//! \{

void CABACWriter::initCtxModels( const Slice& slice )
{
  int       qp                = slice.getSliceQp();
  SliceType sliceType         = slice.getSliceType();
  SliceType encCABACTableIdx  = slice.getEncCABACTableIdx();
  if( !slice.isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && slice.getPPS()->getCabacInitPresentFlag() )
  {
    sliceType = encCABACTableIdx;
  }
  m_BinEncoder.reset( qp, (int)sliceType );
}



template <class BinProbModel>
SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore<BinProbModel>& ctxStoreTest = static_cast<const CtxStore<BinProbModel>&>( ctxTest );
  const CtxStore<BinProbModel>& ctxStoreRef  = static_cast<const CtxStore<BinProbModel>&>( binEncoder.getCtx() );
  int qp = slice.getSliceQp();
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (uint32_t idx=0; idx<2; idx++)
    {
      uint64_t  curCost           = 0;
      SliceType curSliceType      = aSliceTypeChoices[idx];
      ctxTest.init( qp, (int)curSliceType );
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          curCost += uint64_t( binEncoder.getNumBins(k) ) * ctxStoreRef[k].estFracExcessBits( ctxStoreTest[k] );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}


SliceType CABACWriter::getCtxInitId( const Slice& slice )
{
  switch( m_TestCtx.getBPMType() )
  {
  case BPM_Std:   return  xGetCtxInitId<BinProbModel_Std>   ( slice, m_BinEncoder, m_TestCtx );
  default:        return  NUMBER_OF_SLICE_TYPES;
  }
}



unsigned estBits( BinEncIf& binEnc, const std::vector<bool>& bins, const Ctx& ctx, const int ctxId, const uint8_t winSize )
{
  binEnc.initCtxAndWinSize( ctxId, ctx, winSize );
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for( std::size_t binId = 0; binId < numBins; binId++ )
  {
    unsigned  bin = ( bins[binId] ? 1 : 0 );
    binEnc.encodeBin( bin, ctxId );
  }
  unsigned endBits    = binEnc.getNumWrittenBits();
  unsigned codedBits  = endBits - startBits;
  return   codedBits;
}





//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_BinEncoder.encodeBinTrm ( 1 );
  m_BinEncoder.finish       ();
}




//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao, skipAlf )
//================================================================================

void CABACWriter::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr, bool skipSao /* = false */, bool skipAlf /* = false */ )
{
  CUCtx cuCtx( qps[CH_L] );
  QTBTPartitioner partitioner;

  partitioner.initCtu(area, CH_L, *cs.slice);

  if( !skipSao )
  {
    sao( *cs.slice, ctuRsAddr );
  }

  if (!skipAlf)
  {
    for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      codeAlfCtuEnableFlag(cs, ctuRsAddr, compIdx, NULL);
      if (isLuma(ComponentID(compIdx)))
      {
        codeAlfCtuFilterIndex(cs, ctuRsAddr, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y));
      }
      if (isChroma(ComponentID(compIdx)))
      {
        uint8_t* ctbAlfFlag = cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx) ? cs.slice->getPic()->getAlfCtuEnableFlag( compIdx ) : nullptr;
        if( ctbAlfFlag && ctbAlfFlag[ctuRsAddr] )
        {
          codeAlfCtuAlternative( cs, ctuRsAddr, compIdx );
        }
      }
    }
  }

  if ( !skipAlf )
  {
    for ( int compIdx = 1; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
      if (cs.slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
      {
        const int filterCount   = cs.slice->m_ccAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        codeCcAlfFilterControlIdc(cs.slice->m_ccAlfFilterControl[compIdx - 1][ctuRsAddr], cs, ComponentID(compIdx),
                                  ctuRsAddr, cs.slice->m_ccAlfFilterControl[compIdx - 1], lumaPos, filterCount);
      }
    }
  }

  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
  {
    CUCtx chromaCuCtx(qps[CH_C]);
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, CH_C, *cs.slice);
    coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &chromaCuCtx);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = chromaCuCtx.qp;
  }
  else
  {
    coding_tree(cs, partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner.initCtu(area, CH_C, *cs.slice);
      coding_tree(cs, partitioner, cuCtxChroma);
      qps[CH_C] = cuCtxChroma.qp;
    }
  }
}





//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_pars  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_pars ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao( const Slice& slice, unsigned ctuRsAddr )
{
  const SPS& sps = *slice.getSPS();
  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  CodingStructure&     cs                     = *slice.getPic()->cs;
  const PreCalcValues& pcv                    = *cs.pcv;
  const SAOBlkParam&  sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool                slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool                slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  bool                sliceEnabled[3]         = { slice_sao_luma_flag, slice_sao_chroma_flag, slice_sao_chroma_flag };
  int                 frame_width_in_ctus     = pcv.widthInCtus;
  int                 ry                      = ctuRsAddr      / frame_width_in_ctus;
  int                 rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                     ( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned      curSliceIdx             = slice.getIndependentSliceIdx();
  const unsigned      curTileIdx              = cs.pps->getTileIdx( pos );
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0  ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  sao_block_pars( sao_ctu_pars, sps.getBitDepths(), sliceEnabled, leftMergeAvail, aboveMergeAvail, false );
}


void CABACWriter::sao_block_pars( const SAOBlkParam& saoPars, const BitDepths& bitDepths, bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo )
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if( leftMergeAvail )
  {
    // sao_merge_left_flag
    isLeftMerge   = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_LEFT );
    m_BinEncoder.encodeBin( (isLeftMerge), Ctx::SaoMergeFlag() );
  }
  if( aboveMergeAvail && !isLeftMerge )
  {
    // sao_merge_above_flag
    isAboveMerge  = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_ABOVE );
    m_BinEncoder.encodeBin( (isAboveMerge), Ctx::SaoMergeFlag() );
  }
  if( onlyEstMergeInfo )
  {
    return; //only for RDO
  }
  if( !isLeftMerge && !isAboveMerge )
  {
    // explicit parameters
    for( int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      sao_offset_pars( saoPars[compIdx], ComponentID(compIdx), sliceEnabled[compIdx], bitDepths.recon[ toChannelType(ComponentID(compIdx)) ] );
    }
  }
}


void CABACWriter::sao_offset_pars( const SAOOffset& ctbPars, ComponentID compID, bool sliceEnabled, int bitDepth )
{
  if( !sliceEnabled )
  {
    CHECK_( ctbPars.modeIdc != SAO_MODE_OFF, "Sao must be off, if it is disabled on slice level" );
    return;
  }
  const bool isFirstCompOfChType = ( getFirstComponentOfChannel( toChannelType(compID) ) == compID );

  if( isFirstCompOfChType )
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if( ctbPars.modeIdc == SAO_MODE_OFF )
    {
      m_BinEncoder.encodeBin  ( 0, Ctx::SaoTypeIdx() );
    }
    else if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 0 );
    }
    else
    {
      CHECK_(!( ctbPars.typeIdc < SAO_TYPE_START_BO ), "Unspecified error");
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 1 );
    }
  }

  if( ctbPars.modeIdc == SAO_MODE_NEW )
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
    int       numClasses    = ( ctbPars.typeIdc == SAO_TYPE_BO ? 4 : NUM_SAO_EO_CLASSES );
    int       k             = 0;
    int       offset[4];
    for( int i = 0; i < numClasses; i++ )
    {
      if( ctbPars.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN )
      {
        continue;
      }
      int classIdx = ( ctbPars.typeIdc == SAO_TYPE_BO ? ( ctbPars.typeAuxInfo + i ) % NUM_SAO_BO_CLASSES : i );
      offset[k++]  = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for( int i = 0; i < 4; i++ )
    {
      unsigned absOffset = ( offset[i] < 0 ? -offset[i] : offset[i] );
      unary_max_eqprob( absOffset, maxOffsetQVal );
    }

    // band offset mode
    if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      // sao_offset_sign
      for( int i = 0; i < 4; i++ )
      {
        if( offset[i] )
        {
          m_BinEncoder.encodeBinEP( (offset[i] < 0) );
        }
      }
      // sao_band_position
      m_BinEncoder.encodeBinsEP( ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2 );
    }
    // edge offset mode
    else
    {
      if( isFirstCompOfChType )
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        CHECK_( ctbPars.typeIdc - SAO_TYPE_START_EO < 0, "sao edge offset class is outside valid range" );
        m_BinEncoder.encodeBinsEP( ctbPars.typeIdc - SAO_TYPE_START_EO, NUM_SAO_EO_TYPES_LOG2 );
      }
    }
  }
}



//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

void CABACWriter::coding_tree(const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  const CodingUnit &cu        = *cs.getCU( currArea.blocks[partitioner.chType], partitioner.chType );

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  //Note: do not reset qg at chroma CU
  if( pps.getUseDQP() && partitioner.currQgEnable() && !isChroma( partitioner.chType ) )
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded          = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
  }
  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.getUseDQP() && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->getUseChromaQpAdj() && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
    }
  }

  const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

  split_cu_mode( splitMode, cs, partitioner );

  CHECK_( !partitioner.canSplit( splitMode, cs ), "The chosen split mode is invalid!" );

  if( splitMode != CU_DONT_SPLIT )
  {
      if (CS::isDualITree(cs) && pPartitionerChroma != nullptr && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
      {
        partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
        pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
        bool beContinue = true;
        bool lumaContinue = true;
        bool chromaContinue = true;

        while (beContinue)
        {
          if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
          {
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
            }
            lumaContinue = partitioner.nextPart(cs);
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK_(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
          else
          {
            //dual tree coding under 64x64 block
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx);
            }
            lumaContinue = partitioner.nextPart(cs);
            if (cs.picture->blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
            {
              coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
            }
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK_(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
        }
        partitioner.exitCurrSplit();
        pPartitionerChroma->exitCurrSplit();

      }
      else
      {
        const ModeType modeTypeParent = partitioner.modeType;
        const ModeType modeTypeChild = CU::getModeTypeAtDepth( cu, partitioner.currDepth );
        mode_constraint( splitMode, cs, partitioner, modeTypeChild );
        partitioner.modeType = modeTypeChild;

        bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA ? true : false;
        CHECK_( chromaNotSplit && partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma" );
        if( partitioner.treeType == TREE_D )
        {
          partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
        }
      partitioner.splitCurrArea( splitMode, cs );

      do
      {
        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      if( chromaNotSplit )
      {
        if (isChromaEnabled(cs.pcv->chrFormat))
        {
          CHECK_( partitioner.chType != CHANNEL_TYPE_LUMA, "must be luma status" );
        partitioner.chType = CHANNEL_TYPE_CHROMA;
        partitioner.treeType = TREE_C;

        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
        }

        //recover
        partitioner.chType = CHANNEL_TYPE_LUMA;
        partitioner.treeType = TREE_D;
      }
      partitioner.modeType = modeTypeParent;
      }
      return;
  }

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }
  CHECK_( cu.treeType != partitioner.treeType, "treeType mismatch" );


  // coding unit
  coding_unit( cu, partitioner, cuCtx );

  if( cu.chType == CHANNEL_TYPE_CHROMA )
  {
    DTRACE_COND( (isEncoding()), g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  }
  DTRACE_BLOCK_REC_COND( ( !isEncoding() ), cs.picture->getRecoBuf( cu ), cu, cu.predMode );
}

void CABACWriter::mode_constraint( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner, const ModeType modeType )
{
  CHECK_( split == CU_DONT_SPLIT, "splitMode shall not be no split" );
  int val = cs.signalModeCons( split, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    CHECK_( modeType == MODE_TYPE_ALL, "shall not be no constraint case" );
    bool flag = modeType == MODE_TYPE_INTRA;
    int ctxIdx = DeriveCtx::CtxModeConsFlag( cs, partitioner );
    m_BinEncoder.encodeBin( flag, Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    assert( modeType == MODE_TYPE_INTRA );
  }
  else
  {
    assert( modeType == partitioner.modeType );
  }
}

void CABACWriter::split_cu_mode( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner )
{
  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  const bool canSplit = canBh || canBv || canTh || canTv || canQt;
  const bool isNo     = split == CU_DONT_SPLIT;

  if( canNo && canSplit )
  {
    m_BinEncoder.encodeBin( !isNo, Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, !isNo );

  if( isNo )
  {
    return;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  const bool isQt   = split == CU_QUAD_SPLIT;

  if( canQt && canBtt )
  {
    m_BinEncoder.encodeBin( isQt, Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return;
  }

  const bool canHor = canBh || canTh;
  const bool canVer = canBv || canTv;
  const bool  isVer = split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT;

  if( canVer && canHor )
  {
    m_BinEncoder.encodeBin( isVer, Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  const bool can12 = isVer ? canBv : canBh;
  const bool  is12 = isVer ? ( split == CU_VERT_SPLIT ) : ( split == CU_HORZ_SPLIT );

  if( can12 && can14 )
  {
    m_BinEncoder.encodeBin( is12, Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, split );
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACWriter::coding_unit( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
  CodingStructure& cs = *cu.cs;

  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
  {
    cu_skip_flag( cu );
  }


  // skip data
  if( cu.skip )
  {
    CHECK_( !cu.firstPU->mergeFlag, "Merge flag has to be on!" );
    CHECK_(cu.colorTransform, "ACT should not be enabled for skip mode");
    PredictionUnit&   pu = *cu.firstPU;
    prediction_unit ( pu );
    end_of_ctu      ( cu, cuCtx );
    return;
  }


  // prediction mode and partitioning data
  pred_mode ( cu );
  if (CU::isIntra(cu))
  {
    adaptive_color_transform(cu);
  }
  if (CU::isPLT(cu))
  {
    CHECK_(cu.colorTransform, "ACT should not be enabled for PLT mode");
    if (cu.isSepTree())
    {
      if (isLuma(partitioner.chType))
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
      if (cu.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
      {
        cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
      }
    }
    else
    {
      if( cu.chromaFormat != CHROMA_400 )
      {
        cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
      }
      else
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
    }
    end_of_ctu(cu, cuCtx);
    return;
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // end of cu
  end_of_ctu( cu, cuCtx );
}


void CABACWriter::cu_skip_flag( const CodingUnit& cu )
{
  unsigned ctxId = DeriveCtx::CtxSkipFlag( cu );

  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getSPS()->getIBCFlag())
  {
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
    {
    m_BinEncoder.encodeBin((cu.skip), Ctx::SkipFlag(ctxId));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0);
    }
    return;
  }
  if ( !cu.cs->slice->getSPS()->getIBCFlag() && cu.lwidth() == 4 && cu.lheight() == 4 )
  {
    return;
  }
  if( !cu.cs->slice->getSPS()->getIBCFlag() && cu.isConsIntra() )
  {
    return;
  }
  m_BinEncoder.encodeBin( ( cu.skip ), Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0 );
  if (cu.skip && cu.cs->slice->getSPS()->getIBCFlag())
  {
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !cu.isConsInter()) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
    {
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        return;
      }
    unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
    m_BinEncoder.encodeBin(CU::isIBC(cu) ? 1 : 0, Ctx::IBCFlag(ctxidx));
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
  }
}


void CABACWriter::pred_mode( const CodingUnit& cu )
{
  if (cu.cs->slice->getSPS()->getIBCFlag() && cu.chType != CHANNEL_TYPE_CHROMA)
  {
    if( cu.isConsInter() )
    {
      assert( CU::isInter( cu ) );
      return;
    }

    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
    {
      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
      {
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
      }
      if (!CU::isIBC(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
      {
        m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
    }
    else
    {
      if( cu.isConsInter() )
      {
        return;
      }
      m_BinEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
      if (CU::isIntra(cu) || CU::isPLT(cu))
      {
        if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
          m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
      else
      {
        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
        {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
        }
      }
    }
  }
  else
  {
    if( cu.isConsInter() )
    {
      assert( CU::isInter( cu ) );
      return;
    }

    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
    {
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) )  ) && (!cu.isLocalSepTree() || isLuma(cu.chType)  ) )
        m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
      return;
    }
    m_BinEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
    if ((CU::isIntra(cu) || CU::isPLT(cu)) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) )  ) && (!cu.isLocalSepTree() || isLuma(cu.chType)  )  )
    {
      m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
    }
  }
}
void CABACWriter::bdpcm_mode( const CodingUnit& cu, const ComponentID compID )
{
  if( !cu.cs->sps->getBDPCMEnabledFlag() ) return;
  if( !CU::bdpcmAllowed( cu, compID ) ) return;

  int bdpcmMode = isLuma(compID) ? cu.bdpcmMode : cu.bdpcmModeChroma;

  unsigned ctxId = isLuma(compID) ? 0 : 2;
  m_BinEncoder.encodeBin(bdpcmMode > 0 ? 1 : 0, Ctx::BDPCMMode(ctxId));

  if (bdpcmMode)
  {
    m_BinEncoder.encodeBin(bdpcmMode > 1 ? 1 : 0, Ctx::BDPCMMode(ctxId+1));
  }
  if (isLuma(compID))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_LUMA, cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_CHROMA, cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmModeChroma);
  }
}


void CABACWriter::cu_pred_data( const CodingUnit& cu )
{
  if( CU::isIntra( cu ) )
  {
    if( cu.Y().valid() )
    {
      bdpcm_mode( cu, COMPONENT_Y );
    }

    intra_luma_pred_modes  ( cu );
    if( ( !cu.Y().valid() || ( !cu.isSepTree() && cu.Y().valid() ) ) && isChromaEnabled(cu.chromaFormat) )
    {
      bdpcm_mode( cu, ComponentID(CHANNEL_TYPE_CHROMA) );
    }
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu );
  }

  imv_mode   ( cu );
  affine_amvr_mode( cu );

  cu_bcw_flag( cu );

}

void CABACWriter::cu_bcw_flag(const CodingUnit& cu)
{
  if(!CU::isBcwIdxCoded(cu))
  {
    return;
  }

  CHECK_(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)), " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");
  const uint8_t bcwCodingIdx = (uint8_t)g_BcwCodingOrder[CU::getValidBcwIdx(cu)];

  const int32_t numBcw = (cu.slice->getCheckLDC()) ? 5 : 3;
  m_BinEncoder.encodeBin((bcwCodingIdx == 0 ? 0 : 1), Ctx::BcwIdx(0));
  if(numBcw > 2 && bcwCodingIdx != 0)
  {
    const uint32_t prefixNumBits = numBcw - 2;
    const uint32_t step = 1;

    uint8_t idx = 1;
    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (bcwCodingIdx == idx)
      {
        m_BinEncoder.encodeBinEP(0);
        break;
      }
      else
      {
        m_BinEncoder.encodeBinEP(1);
        idx += step;
      }
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.BcwIdx ? 1 : 0);
}

void CABACWriter::xWriteTruncBinCode(uint32_t symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  assert(val <= maxSymbol);
  assert((val << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  int b = maxSymbol - val;
  assert(b < val);
  if (symbol < val - b)
  {
    m_BinEncoder.encodeBinsEP(symbol, thresh);
  }
  else
  {
    symbol += val - b;
    assert(symbol < (val << 1));
    assert((symbol >> 1) >= val - b);
    m_BinEncoder.encodeBinsEP(symbol, thresh + 1);
  }
}

void CABACWriter::extend_ref_line(const PredictionUnit& pu)
{

  const CodingUnit& cu = *pu.cu;
  if( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma( cu.chType ) || cu.bdpcmMode )
  {
    return;
  }
  if( !cu.cs->sps->getUseMRL() )
  {
    return;
  }
  bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
  if (isFirstLineOfCtu)
  {
    return;
  }
  int multiRefIdx = pu.multiRefIdx;
  if (MRL_NUM_REF_LINES > 1)
  {
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
    }
  }
}

void CABACWriter::extend_ref_line(const CodingUnit& cu)
{

  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode )
  {
    return;
  }
  if( !cu.cs->sps->getUseMRL() )
  {
    return;
  }

  const int numBlocks = CU::getNumPUs(cu);
  const PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
    if (isFirstLineOfCtu)
    {
      return;
    }
    int multiRefIdx = pu->multiRefIdx;
    if (MRL_NUM_REF_LINES > 1)
    {
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
      }

    }
    pu = pu->next;
  }
}

void CABACWriter::intra_luma_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  if( cu.bdpcmMode )
  {
    cu.firstPU->intraDir[0] = cu.bdpcmMode == 2? VER_IDX : HOR_IDX;
    return;
  }

  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
  extend_ref_line( cu );

  isp_mode( cu );

  const int numMPMs   = NUM_MOST_PROBABLE_MODES;
  const int numBlocks = CU::getNumPUs( cu );
  unsigned  mpm_preds   [4][numMPMs];
  unsigned  mpm_idxs    [4];
  unsigned  ipred_modes [4];

  const PredictionUnit* pu = cu.firstPU;

  // prev_intra_luma_pred_flag
  for( int k = 0; k < numBlocks; k++ )
  {
    unsigned*  mpm_pred   = mpm_preds[k];
    unsigned&  mpm_idx    = mpm_idxs[k];
    unsigned&  ipred_mode = ipred_modes[k];

    PU::getIntraMPMs( *pu, mpm_pred );

    ipred_mode = pu->intraDir[0];
    mpm_idx    = numMPMs;
    for( unsigned idx = 0; idx < numMPMs; idx++ )
    {
      if( ipred_mode == mpm_pred[idx] )
      {
        mpm_idx = idx;
        break;
      }
    }
    if ( pu->multiRefIdx )
    {
      CHECK_(mpm_idx >= numMPMs, "use of non-MPM");
    }
    else
    {
      m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
    }

    pu = pu->next;
  }

  pu = cu.firstPU;

  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    const unsigned& mpm_idx = mpm_idxs[k];
    if( mpm_idx < numMPMs )
    {
      {
        unsigned ctx = (pu->cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
        if (pu->multiRefIdx == 0)
          m_BinEncoder.encodeBin(mpm_idx > 0, Ctx::IntraLumaPlanarFlag(ctx));
        if( mpm_idx )
        {
          m_BinEncoder.encodeBinEP( mpm_idx > 1 );
        }
        if (mpm_idx > 1)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 2);
        }
        if (mpm_idx > 2)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 3);
        }
        if (mpm_idx > 3)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 4);
        }
      }
    }
    else
    {
      unsigned* mpm_pred   = mpm_preds[k];
      unsigned  ipred_mode = ipred_modes[k];

      // sorting of MPMs
      std::sort( mpm_pred, mpm_pred + numMPMs );

      {
        for (int idx = numMPMs - 1; idx >= 0; idx--)
        {
          if (ipred_mode > mpm_pred[idx])
          {
            ipred_mode--;
          }
        }
        CHECK_(ipred_mode >= 64, "Incorrect mode");
        xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
      }
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}


void CABACWriter::intra_luma_pred_mode( const PredictionUnit& pu )
{

  if( pu.cu->bdpcmMode ) return;
  mip_flag(*pu.cu);
  if (pu.cu->mipFlag)
  {
    mip_pred_mode(pu);
    return;
  }
  extend_ref_line( pu );
  isp_mode( *pu.cu );

  // prev_intra_luma_pred_flag
  const int numMPMs  = NUM_MOST_PROBABLE_MODES;
  unsigned  mpm_pred[numMPMs];

  PU::getIntraMPMs( pu, mpm_pred );

  unsigned ipred_mode = pu.intraDir[0];
  unsigned mpm_idx = numMPMs;

  for( int idx = 0; idx < numMPMs; idx++ )
  {
    if( ipred_mode == mpm_pred[idx] )
    {
      mpm_idx = idx;
      break;
    }
  }
  if ( pu.multiRefIdx )
  {
    CHECK_(mpm_idx >= numMPMs, "use of non-MPM");
  }
  else
  {
    m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
  }

  // mpm_idx / rem_intra_luma_pred_mode
  if( mpm_idx < numMPMs )
  {
    {
      unsigned ctx = (pu.cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
      if (pu.multiRefIdx == 0)
        m_BinEncoder.encodeBin( mpm_idx > 0, Ctx::IntraLumaPlanarFlag(ctx) );
      if( mpm_idx )
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 1 );
      }
      if (mpm_idx > 1)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 2);
      }
      if (mpm_idx > 2)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 3);
      }
      if (mpm_idx > 3)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 4);
      }
    }
  }
  else
  {
    std::sort( mpm_pred, mpm_pred + numMPMs );
    {
      for (int idx = numMPMs - 1; idx >= 0; idx--)
      {
        if (ipred_mode > mpm_pred[idx])
        {
          ipred_mode--;
        }
      }
      xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
    }
  }
}


void CABACWriter::intra_chroma_pred_modes( const CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_LUMA ) )
  {
    return;
  }

  if( cu.bdpcmModeChroma )
  {
    cu.firstPU->intraDir[1] = cu.bdpcmModeChroma == 2 ? VER_IDX : HOR_IDX;
    return;
  }
  const PredictionUnit* pu = cu.firstPU;

  intra_chroma_pred_mode( *pu );
}
void CABACWriter::intra_chroma_lmc_mode(const PredictionUnit& pu)
{
  const unsigned intraDir = pu.intraDir[1];
  int lmModeList[10];
  PU::getLMSymbolList(pu, lmModeList);
  int symbol = -1;
  for (int k = 0; k < LM_SYMBOL_NUM; k++)
  {
    if (lmModeList[k] == intraDir)
    {
      symbol = k;
      break;
    }
  }
  CHECK_(symbol < 0, "invalid symbol found");

  m_BinEncoder.encodeBin(symbol == 0 ? 0 : 1, Ctx::CclmModeIdx(0));

  if (symbol > 0)
  {
    CHECK_(symbol > 2, "invalid symbol for MMLM");
    unsigned int symbol_minus_1 = symbol - 1;
    m_BinEncoder.encodeBinEP(symbol_minus_1);
  }
}


void CABACWriter::intra_chroma_pred_mode(const PredictionUnit& pu)
{

  const unsigned intraDir = pu.intraDir[1];
  if (pu.cu->colorTransform)
  {
    CHECK_(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM for adaptive color transform");
    return;
  }
  if (pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed())
  {
    m_BinEncoder.encodeBin(PU::isLMCMode(intraDir) ? 1 : 0, Ctx::CclmModeFlag(0));
    if (PU::isLMCMode(intraDir))
    {
      intra_chroma_lmc_mode(pu);
      return;
    }
  }

  const bool     isDerivedMode = intraDir == DM_CHROMA_IDX;
  m_BinEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));
  if (isDerivedMode)
  {
    return;
  }

  // chroma candidate index
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(pu, chromaCandModes);

  int candId = 0;
  for (; candId < NUM_CHROMA_MODE; candId++)
  {
    if (intraDir == chromaCandModes[candId])
    {
      break;
    }
  }

  CHECK_(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK_(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
  {
    m_BinEncoder.encodeBinsEP(candId, 2);
  }
}

void CABACWriter::cu_residual( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    PredictionUnit& pu = *cu.firstPU;
    if( !pu.mergeFlag )
    {
      rqt_root_cbf( cu );
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }

    if( !cu.rootCbf )
    {
      CHECK_(cu.colorTransform, "ACT should not be enabled for root_cbf = 0");
      return;
    }
  }

  if (CU::isInter(cu) || CU::isIBC(cu))
  {
    adaptive_color_transform(cu);
  }

  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA]   = false;
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
  cuCtx.lfnstLastScanPos                              = false;
  cuCtx.violatesMtsCoeffConstraint                    = false;
  cuCtx.mtsLastScanPos                                = false;

  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    transform_tree( *cu.cs, subTuPartitioner, cuCtx,             CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType)  ), 0 );
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx );
  }

  residual_lfnst_mode( cu, cuCtx );
  mts_idx            ( cu, &cuCtx );
}

void CABACWriter::rqt_root_cbf( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( cu.rootCbf, Ctx::QtRootCbf() );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACWriter::adaptive_color_transform(const CodingUnit& cu)
{
  if (!cu.slice->getSPS()->getUseColorTrans())
  {
    return;
  }

  if (cu.isSepTree())
  {
    CHECK_(cu.colorTransform, "adaptive color transform should be disabled when dualtree and localtree are enabled");
    return;
  }

  if (CU::isInter(cu) || CU::isIBC(cu) || CU::isIntra(cu))
  {
    m_BinEncoder.encodeBin(cu.colorTransform, Ctx::ACTFlag());
  }
}

void CABACWriter::sbt_mode( const CodingUnit& cu )
{
  uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();
  uint8_t sbtIdx = cu.getSbtIdx();
  uint8_t sbtPos = cu.getSbtPos();

  //bin - flag
  bool sbtFlag = cu.sbtInfo != 0;
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  m_BinEncoder.encodeBin( sbtFlag, Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  bool sbtQuadFlag = sbtIdx == SBT_HOR_QUAD || sbtIdx == SBT_VER_QUAD;
  bool sbtHorFlag = sbtIdx == SBT_HOR_HALF || sbtIdx == SBT_HOR_QUAD;
  bool sbtPosFlag = sbtPos == SBT_POS1;

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  //bin - type
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    m_BinEncoder.encodeBin( sbtQuadFlag, Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    assert( sbtQuadFlag == 0 );
  }

  //bin - dir
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    m_BinEncoder.encodeBin( sbtHorFlag, Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    assert( sbtHorFlag == ( ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow ) ) );
  }

  //bin - pos
  m_BinEncoder.encodeBin( sbtPosFlag, Ctx::SbtPosFlag( 0 ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}

void CABACWriter::end_of_ctu( const CodingUnit& cu, CUCtx& cuCtx )
{
  const bool    isLastSubCUOfCtu  = CU::isLastSubCUOfCtu( cu );

  if ( isLastSubCUOfCtu
    && ( !cu.isSepTree() || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

  }
}

void CABACWriter::cu_palette_info(const CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  const SPS&       sps = *(cu.cs->sps);
  TransformUnit&   tu = *cu.firstTU;
  uint32_t indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];

  int maxPltSize = cu.isSepTree() ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;

  if (cu.lastPLTSize[compBegin])
  {

    xEncodePLTPredIndicator(cu, maxPltSize, compBegin);
  }

  uint32_t reusedPLTnum = 0;
  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
      reusedPLTnum++;
  }
  if (reusedPLTnum < maxPltSize)
  {
    exp_golomb_eqprob(cu.curPLTSize[compBegin] - reusedPLTnum, 0);
  }

  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = cu.reusePLTSize[compBegin]; idx < cu.curPLTSize[compBegin]; idx++)
    {
      ComponentID compID = (ComponentID)comp;
      const int  channelBitDepth = sps.getBitDepth(toChannelType(compID));
      m_BinEncoder.encodeBinsEP(cu.curPLT[comp][idx], channelBitDepth);
    }
  }
  uint32_t signalEscape = (cu.useEscape[compBegin]) ? 1 : 0;
  if (cu.curPLTSize[compBegin] > 0)
  {
    m_BinEncoder.encodeBinEP(signalEscape);
  }
  //encode index map
  uint32_t   height = cu.block(compBegin).height;
  uint32_t   width = cu.block(compBegin).width;

  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t total = height * width;
  if (indexMaxSize > 1)
    codeScanRotationModeFlag(cu, compBegin);
  else
    assert(!cu.useRotation[compBegin]);

  if (cu.useEscape[compBegin] && cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
  {
    if (!cu.isSepTree() || isLuma(tu.chType))
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (cu.useEscape[compBegin] && cu.cs->slice->getUseChromaQpAdj() && !cuCtx.isChromaQpAdjCoded)
  {
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  uint32_t prevRunPos = 0;
  unsigned prevRunType = 0;
  for (int subSetId = 0; subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE; subSetId++)
  {
    cuPaletteSubblockInfo(cu, compBegin, numComp, subSetId, prevRunPos, prevRunType);
  }
  CHECK_(cu.curPLTSize[compBegin] > maxPltSize, " Current palette size is larger than maximum palette size");
}
void CABACWriter::cuPaletteSubblockInfo(const CodingUnit& cu, ComponentID compBegin, uint32_t numComp, int subSetId, uint32_t& prevRunPos, unsigned& prevRunType)
{
  const SPS&      sps = *(cu.cs->sps);
  TransformUnit&  tu  = *cu.firstTU;
  PLTtypeBuf      runType = tu.getrunType(compBegin);
  PelBuf          curPLTIdx = tu.getcurPLTIdx(compBegin);
  uint32_t        indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  uint32_t        totalPel = cu.block(compBegin).height*cu.block(compBegin).width;

  int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
  int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
  maxSubPos = (maxSubPos > totalPel) ? totalPel : maxSubPos; // if last position is out of the current CU size

  unsigned runCopyFlag[(1 << LOG2_PALETTE_CG_SIZE)];
  for (int i = 0; i < (1 << LOG2_PALETTE_CG_SIZE); i++)
    runCopyFlag[i] = MAX_INT;

  if (minSubPos == 0)
    runCopyFlag[0] = 0;

// PLT runCopy flag and runType - context coded
  int curPos = minSubPos;
  for (; curPos < maxSubPos && indexMaxSize > 1; curPos++)
  {
    uint32_t posy = m_scanOrder[curPos].y;
    uint32_t posx = m_scanOrder[curPos].x;
    uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
    uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
    // encode runCopyFlag
    bool identityFlag = !((runType.at(posx, posy) != runType.at(posxprev, posyprev))
      || ((runType.at(posx, posy) == PLT_RUN_INDEX) && (curPLTIdx.at(posx, posy) != curPLTIdx.at(posxprev, posyprev))));

    const CtxSet&   ctxSet = (prevRunType == PLT_RUN_INDEX)? Ctx::IdxRunModel: Ctx::CopyRunModel;
    if ( curPos > 0 )
    {
      int dist = curPos - prevRunPos - 1;
      const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(prevRunType, dist);
      runCopyFlag[curPos - minSubPos] = identityFlag;
      m_BinEncoder.encodeBin( identityFlag, ctxSet( ctxId ) );
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_copy_flag() bin=%d ctx=%d\n", identityFlag, ctxId);
    }
    // encode run_type
    if ( !identityFlag || curPos == 0 )
    {
      prevRunPos  = curPos;
      prevRunType = runType.at(posx, posy);
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else if (curPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else
      {
        m_BinEncoder.encodeBin(runType.at(posx, posy), Ctx::RunTypeFlag());
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_type_flag() bin=%d sp=%d\n", runType.at(posx, posy), curPos);
    }
  }

// PLT index values - bypass coded
  if (indexMaxSize > 1)
  {
    curPos = minSubPos;
    for (; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if ( runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        writePLTIndex(cu, curPos, curPLTIdx, runType, indexMaxSize, compBegin);
        DTRACE(g_trace_ctx, D_SYNTAX, "plt_idx_idc() value=%d sp=%d\n", curPLTIdx.at(posx, posy), curPos);
      }
    }
  }

// Quantized escape colors - bypass coded
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, sps.getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, sps.getChromaFormatIdc());
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    ComponentID compID = (ComponentID)comp;
    for (curPos = minSubPos; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if (curPLTIdx.at(posx, posy) == cu.curPLTSize[compBegin])
      {
          PLTescapeBuf    escapeValue = tu.getescapeValue((ComponentID)comp);
          if (compID == COMPONENT_Y || compBegin != COMPONENT_Y)
          {
            exp_golomb_eqprob((unsigned)escapeValue.at(posx, posy), 5);
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
          if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
          {
            uint32_t posxC = posx >> scaleX;
            uint32_t posyC = posy >> scaleY;
            exp_golomb_eqprob((unsigned)escapeValue.at(posxC, posyC), 5);
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
      }
    }
  }
}
void CABACWriter::codeScanRotationModeFlag(const CodingUnit& cu, ComponentID compBegin)
{
  m_BinEncoder.encodeBin((cu.useRotation[compBegin]), Ctx::RotationFlag());
}
void CABACWriter::xEncodePLTPredIndicator(const CodingUnit& cu, uint32_t maxPLTSize, ComponentID compBegin)
{
  int lastPredIdx = -1;
  uint32_t run = 0;
  uint32_t numPLTPredicted = 0;
  for (uint32_t idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      numPLTPredicted++;
      lastPredIdx = idx;
    }
  }

  int idx = 0;
  while (idx <= lastPredIdx)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      exp_golomb_eqprob(run ? run + 1 : run, 0);
      run = 0;
    }
    else
    {
      run++;
    }
    idx++;
  }
  if ((numPLTPredicted < maxPLTSize && lastPredIdx + 1 < cu.lastPLTSize[compBegin]) || !numPLTPredicted)
  {
    exp_golomb_eqprob(1, 0);
  }
}
Pel CABACWriter::writePLTIndex(const CodingUnit& cu, uint32_t idx, PelBuf& paletteIdx, PLTtypeBuf& paletteRunType, int maxSymbol, ComponentID compBegin)
{
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  Pel curLevel = (paletteIdx.at(posx, posy) == cu.curPLTSize[compBegin]) ? (maxSymbol - 1) : paletteIdx.at(posx, posy);
  if (idx) // R0348: remove index redundancy
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      Pel leftLevel = paletteIdx.at(prevposx, prevposy); // left index
      if (leftLevel == cu.curPLTSize[compBegin]) // escape mode
      {
        leftLevel = maxSymbol - 1;
      }
      assert(leftLevel != curLevel);
      if (curLevel > leftLevel)
      {
        curLevel--;
      }
    }
    else
    {
      Pel aboveLevel;
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        aboveLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin]) // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        aboveLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin]) // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      assert(curLevel != aboveLevel);
      if (curLevel > aboveLevel)
      {
        curLevel--;
      }
    }
    maxSymbol--;
  }
  assert(maxSymbol > 0);
  assert(curLevel >= 0);
  assert(maxSymbol > curLevel);
  if (maxSymbol > 1)
  {
    xWriteTruncBinCode(curLevel, maxSymbol);
  }
  return curLevel;
}


//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu );
//    void  merge_flag      ( pu );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACWriter::prediction_unit( const PredictionUnit& pu )
{
  CHECK_( pu.cu->treeType == TREE_C, "cannot be chroma CU" );
#if ENABLE_SPLIT_PARALLELISM
  CHECK_( pu.cacheUsed, "Processing a PU that should be in cache!" );
  CHECK_( pu.cu->cacheUsed, "Processing a CU that should be in cache!" );

#endif
  if( pu.cu->skip )
  {
    CHECK_( !pu.mergeFlag, "merge_flag must be true for skipped CUs" );
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
    merge_data(pu);
  }
  else if (CU::isIBC(*pu.cu))
  {
    ref_idx(pu, REF_PIC_LIST_0);
    Mv mvd = pu.mvd[REF_PIC_LIST_0];
    mvd.changeIbcPrecInternal2Amvr(pu.cu->imv);
    mvd_coding(mvd, 0); // already changed to signaling precision
    if (pu.cs->sps->getMaxNumIBCMergeCand() == 1)
    {
      CHECK_( pu.mvpIdx[REF_PIC_LIST_0], "mvpIdx for IBC mode should be 0" );
    }
    else
    mvp_flag(pu, REF_PIC_LIST_0);
  }
  else
  {
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
    smvd_mode( pu );
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_0 );
      if ( pu.cu->affine )
      {
        Mv mvd = pu.mvdAffi[REF_PIC_LIST_0][0];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
        mvd = pu.mvdAffi[REF_PIC_LIST_0][1];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd = pu.mvdAffi[REF_PIC_LIST_0][2];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
        }
      }
      else
      {
        Mv mvd = pu.mvd[REF_PIC_LIST_0];
        mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }
    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      if ( pu.cu->smvdMode != 1 )
      {
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( !pu.cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
      {
        if ( pu.cu->affine )
        {
          Mv mvd = pu.mvdAffi[REF_PIC_LIST_1][0];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
          mvd = pu.mvdAffi[REF_PIC_LIST_1][1];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
          if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
          {
            mvd = pu.mvdAffi[REF_PIC_LIST_1][2];
            mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
            mvd_coding(mvd, 0); // already changed to signaling precision
          }
        }
        else
        {
          Mv mvd = pu.mvd[REF_PIC_LIST_1];
          mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
        }
      }
      }
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
  }
}

void CABACWriter::smvd_mode( const PredictionUnit& pu )
{
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }

  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }

  m_BinEncoder.encodeBin( pu.cu->smvdMode ? 1 : 0, Ctx::SmvdFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

void CABACWriter::subblock_merge_flag( const CodingUnit& cu )
{

  if ( !cu.cs->slice->isIntra() && (cu.slice->getPicHeader()->getMaxNumAffineMergeCand() > 0) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    m_BinEncoder.encodeBin( cu.affine, Ctx::SubblockMergeFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACWriter::affine_flag( const CodingUnit& cu )
{
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      unsigned ctxId = 0;
      m_BinEncoder.encodeBin( cu.affineType, Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
  }
}

void CABACWriter::merge_flag( const PredictionUnit& pu )
{
  m_BinEncoder.encodeBin( pu.mergeFlag, Ctx::MergeFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

}

void CABACWriter::merge_data(const PredictionUnit& pu)
{
  if (CU::isIBC(*pu.cu))
  {
    merge_idx(pu);
    return;
  }
  subblock_merge_flag(*pu.cu);
  if (pu.cu->affine)
  {
    merge_idx(pu);
    return;
  }
  const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() < MAX_CU_SIZE && pu.cu->lheight() < MAX_CU_SIZE && pu.cu->lwidth() * pu.cu->lheight() >= 64;
  const bool geoAvailable = pu.cu->cs->slice->getSPS()->getUseGeo() && pu.cu->cs->slice->isInterB() &&
    pu.cs->sps->getMaxNumGeoCand() > 1
                                                                    && pu.cu->lwidth() >= GEO_MIN_CU_SIZE && pu.cu->lheight() >= GEO_MIN_CU_SIZE
                                                                    && pu.cu->lwidth() <= GEO_MAX_CU_SIZE && pu.cu->lheight() <= GEO_MAX_CU_SIZE
                                                                    && pu.cu->lwidth() < 8 * pu.cu->lheight() && pu.cu->lheight() < 8 * pu.cu->lwidth();
  if (geoAvailable || ciipAvailable)
  {
    m_BinEncoder.encodeBin(pu.regularMergeFlag, Ctx::RegularMergeFlag(pu.cu->skip ? 0 : 1));
  }
  if (pu.regularMergeFlag)
  {
    if (pu.cs->sps->getUseMMVD())
    {
      m_BinEncoder.encodeBin(pu.mmvdMergeFlag, Ctx::MmvdFlag(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
    }
    if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
    {
      mmvd_merge_idx(pu);
    }
    else
    {
      merge_idx(pu);
    }
  }
  else
  {
    if (geoAvailable && ciipAvailable)
    {
      Ciip_flag(pu);
    }
    merge_idx(pu);
  }
}

void CABACWriter::imv_mode( const CodingUnit& cu )
{
  const SPS *sps = cu.cs->sps;

  if( !sps->getAMVREnabledFlag() )
  {
    return;
  }
  if ( cu.affine )
  {
    return;
  }

  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  if (CU::isIBC(cu) == false)
    m_BinEncoder.encodeBin( (cu.imv > 0), Ctx::ImvFlag( 0 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 0), 0 );

  if( sps->getAMVREnabledFlag() && cu.imv > 0 )
  {
    if (!CU::isIBC(cu))
    {
      m_BinEncoder.encodeBin(cu.imv < IMV_HPEL, Ctx::ImvFlag(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", cu.imv < 3, 4);
    }
    if (cu.imv < IMV_HPEL)
    {
    m_BinEncoder.encodeBin( (cu.imv > 1), Ctx::ImvFlag( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 1), 1 );
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACWriter::affine_amvr_mode( const CodingUnit& cu )
{
  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  m_BinEncoder.encodeBin( (cu.imv > 0), Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 0), 2 );

  if( cu.imv > 0 )
  {
    m_BinEncoder.encodeBin( (cu.imv > 1), Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 1), 3 );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

void CABACWriter::merge_idx( const PredictionUnit& pu )
{

  if ( pu.cu->affine )
  {
    int numCandminus1 = int( pu.cs->picHeader->getMaxNumAffineMergeCand() ) - 1;
    if ( numCandminus1 > 0 )
    {
      if ( pu.mergeIdx == 0 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::AffMergeIdx() );
        DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
        return;
      }
      else
      {
        m_BinEncoder.encodeBin( 1, Ctx::AffMergeIdx() );
        for ( unsigned idx = 1; idx < numCandminus1; idx++ )
        {
            m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
          if ( pu.mergeIdx == idx )
          {
            break;
          }
        }
      }
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
  }
  else
  {
    if( pu.cu->geoFlag )
    {
      uint8_t splitDir = pu.geoSplitDir;
      uint8_t candIdx0 = pu.geoMergeIdx0;
      uint8_t candIdx1 = pu.geoMergeIdx1;
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", splitDir );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx0=%d\n", candIdx0 );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx1=%d\n", candIdx1 );
      xWriteTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
      candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;
      const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
      CHECK_(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      CHECK_(candIdx0 >= maxNumGeoCand, "Incorrect candIdx0");
      CHECK_(candIdx1 >= maxNumGeoCand, "Incorrect candIdx1");
      int numCandminus2 = maxNumGeoCand - 2;
      m_BinEncoder.encodeBin( candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx() );
      if( candIdx0 > 0 )
      {
        unary_max_eqprob(candIdx0 - 1, numCandminus2);
      }
      if (numCandminus2 > 0)
      {
        m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
        if (candIdx1 > 0)
        {
          unary_max_eqprob(candIdx1 - 1, numCandminus2 - 1);
        }
      }
      return;
    }
    int numCandminus1;
    if (pu.cu->predMode == MODE_IBC)
      numCandminus1 = int(pu.cs->sps->getMaxNumIBCMergeCand()) - 1;
    else
      numCandminus1 = int(pu.cs->sps->getMaxNumMergeCand()) - 1;
  if( numCandminus1 > 0 )
  {
    if( pu.mergeIdx == 0 )
    {
      m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
      for( unsigned idx = 1; idx < numCandminus1; idx++ )
      {
          m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
        if( pu.mergeIdx == idx )
        {
          break;
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
  }
}
void CABACWriter::mmvd_merge_idx(const PredictionUnit& pu)
{
  int var0, var1, var2;
  int mvpIdx = pu.mmvdMergeIdx;
  var0 = mvpIdx / MMVD_MAX_REFINE_NUM;
  var1 = (mvpIdx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
  var2 = mvpIdx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;
  if (pu.cs->sps->getMaxNumMergeCand() > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    assert(var0 < 2);
    m_BinEncoder.encodeBin(var0, Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);

  int numCandminus1_step = MMVD_REFINE_STEP - 1;
  if (numCandminus1_step > 0)
  {
    if (var1 == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::MmvdStepMvpIdx());
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::MmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numCandminus1_step; idx++)
      {
        m_BinEncoder.encodeBinEP(var1 == idx ? 0 : 1);
        if (var1 == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);

  m_BinEncoder.encodeBinsEP(var2, 2);

  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
}
void CABACWriter::inter_pred_idc( const PredictionUnit& pu )
{
  if( !pu.cs->slice->isInterB() )
  {
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( pu.interDir == 3 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::InterDir(ctxId) );
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 0, Ctx::InterDir(ctxId) );
    }
  }
  m_BinEncoder.encodeBin( ( pu.interDir == 2 ), Ctx::InterDir( 5 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
}


void CABACWriter::ref_idx( const PredictionUnit& pu, RefPicList eRefList )
{
  if ( pu.cu->smvdMode )
  {
    CHECK_( pu.refIdx[eRefList] != pu.cs->slice->getSymRefIdx( eRefList ), "Invalid reference index!\n" );
    return;
  }

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

  if (eRefList == REF_PIC_LIST_0 && pu.cs->sps->getIBCFlag())
  {
    if (CU::isIBC(*pu.cu))
      return;
  }

  if( numRef <= 1 )
  {
    return;
  }
  int refIdx  = pu.refIdx[eRefList];
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::RefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::RefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      break;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
}

void CABACWriter::mvp_flag( const PredictionUnit& pu, RefPicList eRefList )
{
  m_BinEncoder.encodeBin( pu.mvpIdx[eRefList], Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", pu.mvpIdx[eRefList], pu.lumaPos().x, pu.lumaPos().y );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, pu.mvpIdx[eRefList] );
}

void CABACWriter::Ciip_flag(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseCiip())
  {
    CHECK_(pu.ciipFlag == true, "invalid Ciip SPS");
    return;
  }
  if (pu.cu->skip)
  {
    CHECK_(pu.ciipFlag == true, "invalid Ciip and skip");
    return;
  }
  m_BinEncoder.encodeBin(pu.ciipFlag, Ctx::CiipFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "Ciip_flag() Ciip=%d pos=(%d,%d) size=%dx%d\n", pu.ciipFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
}





//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================
void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx,                         const PartSplit ispType, const int subTuIdx )
{
  const UnitArea&       area = partitioner.currArea();
  int             subTuCounter = subTuIdx;
  const TransformUnit&  tu = *cs.getTU(area.blocks[partitioner.chType].pos(), partitioner.chType, subTuIdx);
  const CodingUnit&     cu = *tu.cu;
  const unsigned        trDepth = partitioner.currTrDepth;
  const bool            split = (tu.depth > trDepth);

  // split_transform_flag
  if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
  {
    CHECK_( !split, "transform split implied" );
  }
  else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
  {
    CHECK_( !split, "transform split implied - sbt" );
  }
  else
    CHECK_( split && !cu.ispMode, "transform split not allowed with QTBT" );


  if( split )
  {

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n", partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( cu.ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
    {
      partitioner.splitCurrArea( PartSplit( cu.getSbtTuSplit() ), cs );
    }
    else
      THROW( "Implicit TU split not available" );

    do
    {
      transform_tree( cs, partitioner, cuCtx,                ispType, subTuCounter );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    transform_unit( tu, cuCtx, partitioner, subTuCounter);
  }
}

void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
  unsigned  ctxId = DeriveCtx::CtxQtCbf(area.compID, prevCbf, useISP && isLuma(area.compID));
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID ];

  if ((area.compID == COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmMode)
   || (area.compID != COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID)) != NULL && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmModeChroma))
  {
    if (area.compID == COMPONENT_Y)
      ctxId = 1;
    else if (area.compID == COMPONENT_Cb)
      ctxId = 1;
    else
      ctxId = 2;
    m_BinEncoder.encodeBin(cbf, ctxSet(ctxId));
  }
  else
  {
  m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================
void CABACWriter::mvd_coding( const Mv &rMvd, int8_t imv )
{
  int       horMvd = rMvd.getHor();
  int       verMvd = rMvd.getVer();
  if ( imv > 0 )
  {
    CHECK_((horMvd % 2) != 0 && (verMvd % 2) != 0, "IMV: MVD is not a multiple of 2");
    horMvd >>= 1;
    verMvd >>= 1;
    if (imv < IMV_HPEL)
    {
      CHECK_((horMvd % 2) != 0 && (verMvd % 2) != 0, "IMV: MVD is not a multiple of 4");
      horMvd >>= 1;
      verMvd >>= 1;
      if (imv == IMV_4PEL)//IMV_4PEL
      {
        CHECK_((horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 16");
        horMvd >>= 2;
        verMvd >>= 2;
      }
    }
  }
  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );


  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_BinEncoder.encodeBin( (horAbs > 0), Ctx::Mvd() );
  m_BinEncoder.encodeBin( (verAbs > 0), Ctx::Mvd() );

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
  }
  if( verAbs > 0 )
  {
    m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    if( horAbs > 1 )
    {
      m_BinEncoder.encodeRemAbsEP(horAbs - 2, 1, 0, MV_BITS - 1);
    }
    m_BinEncoder.encodeBinEP( (horMvd < 0) );
  }
  if( verAbs > 0 )
  {
    if( verAbs > 1 )
    {
      m_BinEncoder.encodeRemAbsEP(verAbs - 2, 1, 0, MV_BITS - 1);
    }
    m_BinEncoder.encodeBinEP( (verMvd < 0) );
  }
}




//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACWriter::transform_unit( const TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter)
{
  const CodingStructure&  cs = *tu.cs;
  const CodingUnit&       cu = *tu.cu;
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;
  ChromaCbfs              chromaCbfs;
  CHECK_(tu.depth != trDepth, " transform unit should be not be futher partitioned");

  // cbf_cb & cbf_cr
  if (area.chromaFormat != CHROMA_400)
  {
    const bool              chromaCbfISP = area.blocks[COMPONENT_Cb].valid() && cu.ispMode;
    if (area.blocks[COMPONENT_Cb].valid() && (!cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
  {
    {
      unsigned cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
      {
        chromaCbfs.Cb = TU::getCbfAtDepth(tu, COMPONENT_Cb, trDepth);
        //if (!(cu.sbtInfo && trDepth == 1))
        if (!(cu.sbtInfo && tu.noResidual))
          cbf_comp(cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], cbfDepth);
      }

      {
        chromaCbfs.Cr = TU::getCbfAtDepth(tu, COMPONENT_Cr, trDepth);
        //if (!(cu.sbtInfo && trDepth == 1))
        if (!(cu.sbtInfo && tu.noResidual))
          cbf_comp(cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb);
      }
    }
  }
  else if (cu.isSepTree())
  {
    chromaCbfs = ChromaCbfs(false);
  }
  }
  else if (cu.isSepTree())
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      CHECK_(!TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be true for inter units with no chroma coeffs");
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      CHECK_(TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be false for inter sbt no-residual tu");
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      CHECK_(!TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be true for inter sbt residual tu");
    }
    else
    {
      bool lumaCbfIsInferredACT = (cu.colorTransform && cu.predMode == MODE_INTRA && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat));
      CHECK_(lumaCbfIsInferredACT && !TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "adaptive color transform cannot have all zero coefficients");
      bool lastCbfIsInferred    = lumaCbfIsInferredACT; // ISP and ACT are mutually exclusive
      bool previousCbf          = false;
      bool rootCbfSoFar         = false;
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < subTuCounter; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, trDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMPONENT_Y, partitioner.currTrDepth);
        }
      }
      if (!lastCbfIsInferred)
      {
        cbf_comp(cs, TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), tu.Y(), trDepth, previousCbf, cu.ispMode);
      }
    }
  }
  bool        lumaOnly  = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbf[3]    = { TU::getCbf( tu, COMPONENT_Y ), chromaCbfs.Cb, chromaCbfs.Cr };
  bool        cbfLuma   = ( cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma = false;

  if( !lumaOnly )
  {
    if( tu.blocks[COMPONENT_Cb].valid() )
    {
      cbf   [ COMPONENT_Cb  ] = TU::getCbf( tu, COMPONENT_Cb );
      cbf   [ COMPONENT_Cr  ] = TU::getCbf( tu, COMPONENT_Cr );
    }
    cbfChroma = ( cbf[ COMPONENT_Cb ] || cbf[ COMPONENT_Cr ] );
  }

  if( ( cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma ) &&
    (!tu.cu->isSepTree() || isLuma(tu.chType)) )
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (!cu.isSepTree() || isChroma(tu.chType))   // !DUAL_TREE_LUMA
  {
    SizeType channelWidth = !cu.isSepTree() ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !cu.isSepTree() ? cu.lheight() : cu.chromaSize().height;

    if (cu.cs->slice->getUseChromaQpAdj() && (channelWidth > 64 || channelHeight > 64 || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  if( !lumaOnly )
  {
    joint_cb_cr( tu, ( cbf[COMPONENT_Cb] ? 2 : 0 ) + ( cbf[COMPONENT_Cr] ? 1 : 0 ) );
  }

    if( cbfLuma )
    {
      residual_coding( tu, COMPONENT_Y, &cuCtx );
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( cbf[ compID ] )
        {
          residual_coding( tu, compID, &cuCtx );
      }
    }
  }
}

void CABACWriter::cu_qp_delta( const CodingUnit& cu, int predQP, const int8_t qp )
{
  CHECK_(!( predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int       DQp         = qp - predQP;
  int       qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
  DQp                   = ( DQp + (MAX_QP + 1) + (MAX_QP + 1) / 2 + qpBdOffsetY + (qpBdOffsetY / 2)) % ((MAX_QP + 1) + qpBdOffsetY) - (MAX_QP + 1) / 2 - (qpBdOffsetY / 2);
  unsigned  absDQP      = unsigned( DQp < 0 ? -DQp : DQp );
  unsigned  unaryDQP    = std::min<unsigned>( absDQP, CU_DQP_TU_CMAX );

  unary_max_symbol( unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( absDQP >= CU_DQP_TU_CMAX )
  {
    exp_golomb_eqprob( absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }
  if( absDQP > 0 )
  {
    m_BinEncoder.encodeBinEP( DQp < 0 );
  }

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACWriter::cu_chroma_qp_offset( const CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if( qpAdj == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ChromaQpAdjFlag() );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ChromaQpAdjFlag() );
    int length = cu.cs->pps->getChromaQpOffsetListLen();
    if( length > 1 )
    {
      unary_max_symbol( qpAdj-1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
    }
  }
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACWriter::joint_cb_cr( const TransformUnit& tu, const int cbfMask )
{
  if ( !tu.cu->slice->getSPS()->getJointCbCrEnabledFlag() )
  {
    return;
  }

  CHECK_( tu.jointCbCr && tu.jointCbCr != cbfMask, "wrong value of jointCbCr (" << (int)tu.jointCbCr << " vs " << (int)cbfMask << ")" );
  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
  {
    m_BinEncoder.encodeBin( tu.jointCbCr ? 1 : 0, Ctx::JointCbCrFlag( cbfMask - 1 ) );
  }
}

void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID, CUCtx* cuCtx )
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

  if( compID == COMPONENT_Cr && tu.jointCbCr == 3 )
    return;

  ts_flag            ( tu, compID );

  if( tu.mtsIdx[compID] == MTS_SKIP && !tu.cs->slice->getTSResidualCodingDisabledFlag() )
  {
    residual_codingTS( tu, compID );
    return;
  }

  // determine sign hiding
  bool signHiding = cu.cs->slice->getSignDataHidingEnabledFlag();

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;

  // determine and set last coeff position and sig group flags
  int                      scanPosLast = -1;
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      scanPosLast = scanPos;
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }
  CHECK_( scanPosLast < 0, "Coefficient coding called for empty TU" );
  cctx.setScanPosLast(scanPosLast);

  if (cuCtx && tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
  {
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
    cuCtx->violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
  if (cuCtx && tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
  {
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx->lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
  if (cuCtx && isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP)
  {
    cuCtx->mtsLastScanPos |= cctx.scanPosLast() >= 1;
  }


  // code last coeff position
  last_sig_coeff( cctx, tu, compID );

  // code subblocks
  const int stateTab = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
  int       state     = 0;

  int ctxBinSampleRatio = (compID == COMPONENT_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock       ( subSetId, sigGroupFlags[subSetId] );

    if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 && compID == COMPONENT_Y )
    {
      if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) )
       || ( tu.blocks[ compID ].width  == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth()  ) ) )
      {
        continue;
      }
    }
    residual_coding_subblock( cctx, coeff, stateTab, state );

    if ( cuCtx && isLuma(compID) && cctx.isSigGroup() && ( cctx.cgPosY() > 3 || cctx.cgPosX() > 3 ) )
    {
      cuCtx->violatesMtsCoeffConstraint = true;
    }
  }
}

void CABACWriter::ts_flag( const TransformUnit& tu, ComponentID compID )
{
  int tsFlag = tu.mtsIdx[compID] == MTS_SKIP ? 1 : 0;
  int ctxIdx = isLuma(compID) ? 0 : 1;

  if( TU::isTSAllowed ( tu, compID ) )
  {
    m_BinEncoder.encodeBin( tsFlag, Ctx::TransformSkipFlag(ctxIdx));
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), tsFlag );
}

void CABACWriter::mts_idx( const CodingUnit& cu, CUCtx* cuCtx )
{
  TransformUnit &tu = *cu.firstTU;
  int        mtsIdx = tu.mtsIdx[COMPONENT_Y];

  if( CU::isMTSAllowed( cu, COMPONENT_Y ) && cuCtx && !cuCtx->violatesMtsCoeffConstraint &&
      cuCtx->mtsLastScanPos && cu.lfnstIdx == 0 && mtsIdx != MTS_SKIP)
  {
    int symbol = mtsIdx != MTS_DCT2_DCT2 ? 1 : 0;
    int ctxIdx = 0;

    m_BinEncoder.encodeBin( symbol, Ctx::MTSIdx(ctxIdx));

    if( symbol )
    {
      ctxIdx = 1;
      for( int i = 0; i < 3; i++, ctxIdx++ )
      {
        symbol = mtsIdx > i + MTS_DST7_DST7 ? 1 : 0;
        m_BinEncoder.encodeBin( symbol, Ctx::MTSIdx(ctxIdx));

        if( !symbol )
        {
          break;
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
}

void CABACWriter::isp_mode( const CodingUnit& cu )
{
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) || cu.colorTransform )
  {
    CHECK_( cu.ispMode != NOT_INTRA_SUBPARTITIONS, "cu.ispMode != 0" );
    return;
  }
  if ( cu.ispMode == NOT_INTRA_SUBPARTITIONS )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ISPMode( 0 ) );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ISPMode( 0 ) );
    m_BinEncoder.encodeBin( cu.ispMode - 1, Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACWriter::residual_lfnst_mode( const CodingUnit& cu, CUCtx& cuCtx )
{
  int chIdx = cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
  if( ( cu.ispMode && !CU::canUseLfnstWithISP( cu, cu.chType ) ) ||
      (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize())) ||
    ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
    || ( cu.blocks[ chIdx ].lumaSize().width > cu.cs->sps->getMaxTbSize() || cu.blocks[ chIdx ].lumaSize().height > cu.cs->sps->getMaxTbSize() )
    )
  {
    return;
  }

  if( cu.cs->sps->getUseLFNST() && CU::isIntra( cu ) )
  {
    const bool lumaFlag                   = cu.isSepTree() ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag                 = cu.isSepTree() ? ( isChroma( cu.chType ) ? true : false ) : true;
          bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] );
    bool isTrSkip = false;
    for (auto &currTU : CU::traverseTUs(cu))
    {
      const uint32_t numValidComp = getNumberValidComponents(cu.chromaFormat);
      for (uint32_t compID = COMPONENT_Y; compID < numValidComp; compID++)
      {
        if (currTU.blocks[compID].valid() && TU::getCbf(currTU, (ComponentID)compID) && currTU.mtsIdx[compID] == MTS_SKIP)
        {
          isTrSkip = true;
          break;
        }
      }
    }
    if( (!cuCtx.lfnstLastScanPos && !cu.ispMode) || nonZeroCoeffNonTsCorner8x8 || isTrSkip )
    {
      return;
    }
  }
  else
  {
    return;
  }


  unsigned cctx = 0;
  if ( cu.isSepTree() ) cctx++;

  const uint32_t idxLFNST = cu.lfnstIdx;
  assert( idxLFNST < 3 );
  m_BinEncoder.encodeBin( idxLFNST ? 1 : 0, Ctx::LFNSTIdx( cctx ) );

  if( idxLFNST )
  {
    m_BinEncoder.encodeBin( (idxLFNST - 1) ? 1 : 0, Ctx::LFNSTIdx(2));
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}

void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx, const TransformUnit& tu, ComponentID compID )
{
  unsigned blkPos = cctx.blockPos( cctx.scanPosLast() );
  unsigned posX, posY;
  {
    posY  = blkPos / cctx.width();
    posX  = blkPos - ( posY * cctx.width() );
  }

  unsigned CtxLast;
  unsigned GroupIdxX = g_uiGroupIdx[ posX ];
  unsigned GroupIdxY = g_uiGroupIdx[ posY ];

  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

  if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 && compID == COMPONENT_Y )
  {
    maxLastPosX = ( tu.blocks[compID].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[compID].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }

  for( CtxLast = 0; CtxLast < GroupIdxX; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastXCtxId( CtxLast ) );
  }
  if( GroupIdxX < maxLastPosX )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastXCtxId( CtxLast ) );
  }
  for( CtxLast = 0; CtxLast < GroupIdxY; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxY < maxLastPosY )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxX > 3 )
  {
    posX -= g_uiMinInGroup[ GroupIdxX ];
    for (int i = ( ( GroupIdxX - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posX >> i ) & 1 );
    }
  }
  if( GroupIdxY > 3 )
  {
    posY -= g_uiMinInGroup[ GroupIdxY ];
    for ( int i = ( ( GroupIdxY - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posY >> i ) & 1 );
    }
  }
}



void CABACWriter::residual_coding_subblock( CoeffCodingContext& cctx, const TCoeff* coeff, const int stateTransTable, int& state )
{
  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  uint8_t   ctxOffset[16];

  //===== encode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  TCoeff    remAbsLevel   = -1;
  int       numNonZero    =  0;
  unsigned  signPattern   =  0;
  int       remRegBins    = cctx.regBinLimit;
  int       firstPosMode2 = minSubPos - 1;

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      m_BinEncoder.encodeBin( sigFlag, sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
      cctx.sigCtxIdAbs( nextSigPos, coeff, state ); // required for setting variables that are needed for gtx/par context selection
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff  = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff            = cctx.ctxOffsetAbs();
      numNonZero++;
      firstNZPos  = nextSigPos;
      lastNZPos   = std::max<int>( lastNZPos, nextSigPos );
      remAbsLevel = abs( Coeff ) - 1;

      if( nextSigPos != cctx.scanPosLast() ) signPattern <<= 1;
      if( Coeff < 0 )                        signPattern++;

      unsigned gt1 = !!remAbsLevel;
      m_BinEncoder.encodeBin( gt1, cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      if( gt1 )
      {
        remAbsLevel  -= 1;
        m_BinEncoder.encodeBin( remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        remAbsLevel >>= 1;

        remRegBins--;
        unsigned gt2 = !!remAbsLevel;
        m_BinEncoder.encodeBin(gt2, cctx.greater2CtxIdAbs(ctxOff));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2, cctx.greater2CtxIdAbs(ctxOff));
        remRegBins--;
      }
    }

    state = ( stateTransTable >> ((state<<2)+((Coeff&1)<<1)) ) & 3;
  }
  firstPosMode2 = nextSigPos;
  cctx.regBinLimit = remRegBins;


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
    ricePar = g_auiGoRiceParsCoeff[sumAll];
    unsigned absLevel = (unsigned) abs( coeff[ cctx.blockPos( scanPos ) ] );
    if( absLevel >= 4 )
    {
      unsigned rem      = ( absLevel - 4 ) >> 1;
      m_BinEncoder.encodeRemAbsEP( rem, ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    TCoeff    Coeff     = coeff[ cctx.blockPos( scanPos ) ];
    unsigned    absLevel  = (unsigned) abs( Coeff );
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0(state, rice);
    unsigned  rem       = ( absLevel == 0 ? pos0 : absLevel <= pos0 ? absLevel-1 : absLevel );
    m_BinEncoder.encodeRemAbsEP( rem, rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    state = ( stateTransTable >> ((state<<2)+((absLevel&1)<<1)) ) & 3;
    if( absLevel )
    {
      numNonZero++;
      firstNZPos = scanPos;
      lastNZPos   = std::max<int>( lastNZPos, scanPos );
      signPattern <<= 1;
      if( Coeff < 0 ) signPattern++;
    }
  }

  //===== encode sign's =====
  unsigned numSigns = numNonZero;
  if( cctx.hideSign( firstNZPos, lastNZPos ) )
  {
    numSigns    --;
    signPattern >>= 1;
  }
  m_BinEncoder.encodeBinsEP( signPattern, numSigns );
}

void CABACWriter::residual_codingTS( const TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, isLuma(compID) ? tu.cu->bdpcmMode : tu.cu->bdpcmModeChroma);
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;
  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  // determine and set last coeff position and sig group flags
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }

  // code subblocks
  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId, sigGroupFlags[subSetId] );
    residual_coding_subblockTS( cctx, coeff );
  }
}

void CABACWriter::residual_coding_subblockTS( CoeffCodingContext& cctx, const TCoeff* coeff )
{
  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !cctx.isLastSubSet() || !cctx.only1stSigGroup() )
  {
    if( cctx.isSigGroup() )
    {
        m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 1, cctx.sigGroupCtxId() );
    }
    else
    {
        m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  //===== encode absolute values =====
  const int inferSigPos   = minSubPos;
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;

  int rightPixel, belowPixel, modAbsCoeff;

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.numCtxBins() >= 4; nextSigPos++)
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
        const unsigned sigCtxId = cctx.sigCtxIdAbsTS( nextSigPos, coeff );
        m_BinEncoder.encodeBin( sigFlag, sigCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
        cctx.decimateNumCtxBins(1);
    }

    if( sigFlag )
    {
      //===== encode sign's =====
      int sign = Coeff < 0;
        const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
        m_BinEncoder.encodeBin(sign, signCtxId);
        cctx.decimateNumCtxBins(1);
      numNonZero++;
      cctx.neighTS(rightPixel, belowPixel, nextSigPos, coeff);
      modAbsCoeff = cctx.deriveModCoeff(rightPixel, belowPixel, abs(Coeff), cctx.bdpcm());
      remAbsLevel = modAbsCoeff - 1;

      unsigned gt1 = !!remAbsLevel;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
        m_BinEncoder.encodeBin(gt1, gt1CtxId);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1, gt1CtxId);
        cctx.decimateNumCtxBins(1);

      if( gt1 )
      {
        remAbsLevel  -= 1;
          m_BinEncoder.encodeBin( remAbsLevel&1, cctx.parityCtxIdAbsTS() );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbsTS() );
          cctx.decimateNumCtxBins(1);
      }
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  int numGtBins = 4;
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.numCtxBins() >= 4; scanPos++)
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm());
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if (absLevel >= cutoffVal)
      {
        unsigned gt2 = (absLevel >= (cutoffVal + 2));
          m_BinEncoder.encodeBin(gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
          DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, min<int>(absLevel, cutoffVal + 2));
          cctx.decimateNumCtxBins(1);
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }

  //===== coeff bypass ====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm()||!cutoffVal);

    if( absLevel >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      unsigned  rem = scanPos <= lastScanPosPass1 ? (absLevel - cutoffVal) >> 1 : absLevel;
      m_BinEncoder.encodeRemAbsEP( rem, rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );

      if (absLevel && scanPos > lastScanPosPass1)
      {
        int sign = coeff[cctx.blockPos(scanPos)] < 0;
        m_BinEncoder.encodeBinEP(sign);
      }
    }
  }
}







//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol )
{
  CHECK_( symbol > maxSymbol, "symbol > maxSymbol" );
  const unsigned totalBinsToWrite = std::min( symbol + 1, maxSymbol );
  for( unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten )
  {
    const unsigned nextBin = symbol > binsWritten;
    m_BinEncoder.encodeBin( nextBin, binsWritten == 0 ? ctxId0 : ctxIdN );
  }
}


void CABACWriter::unary_max_eqprob( unsigned symbol, unsigned maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }
  bool     codeLast = ( maxSymbol > symbol );
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while( symbol-- )
  {
    bins   <<= 1;
    bins   ++;
    numBins++;
  }
  if( codeLast )
  {
    bins  <<= 1;
    numBins++;
  }
  CHECK_(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::exp_golomb_eqprob( unsigned symbol, unsigned count )
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while( symbol >= (unsigned)(1<<count) )
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  //CHECK_(!( numBins + count <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP(bins, numBins);
  m_BinEncoder.encodeBinsEP(symbol, count);
}

void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isLuma( channel ) )
  {
    if (alfParam->enabledFlag[COMPONENT_Y])
      codeAlfCtuEnableFlags( cs, COMPONENT_Y, alfParam );
  }
  else
  {
    if (alfParam->enabledFlag[COMPONENT_Cb])
      codeAlfCtuEnableFlags( cs, COMPONENT_Cb, alfParam );
    if (alfParam->enabledFlag[COMPONENT_Cr])
      codeAlfCtuEnableFlags( cs, COMPONENT_Cr, alfParam );
  }
}
void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  uint32_t numCTUs = cs.pcv->sizeInCtus;

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    codeAlfCtuEnableFlag( cs, ctuIdx, compID, alfParam );
  }
}

void CABACWriter::codeAlfCtuEnableFlag( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfParam* alfParam)
{
  const bool alfComponentEnabled = (alfParam != NULL) ? alfParam->enabledFlag[compIdx] : cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx);

  if( cs.sps->getALFEnabledFlag() && alfComponentEnabled )
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
    const uint32_t      curTileIdx = cs.pps->getTileIdx( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
    int ctx = 0;
    ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
    ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;
    m_BinEncoder.encodeBin( ctbAlfFlag[ctuRsAddr], Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );
  }
}

void CABACWriter::codeCcAlfFilterControlIdc(uint8_t idcVal, CodingStructure &cs, const ComponentID compID,
                                            const int curIdx, const uint8_t *filterControlIdc, Position lumaPos,
                                            const int filterCount)
{
  CHECK_(idcVal > filterCount, "Filter index is too large");

  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
    ctxt += ( filterControlIdc[curIdx - 1]) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus]) ? 1 : 0;
  }
  ctxt += ( compID == COMPONENT_Cr ) ? 3 : 0;

  m_BinEncoder.encodeBin( ( idcVal == 0 ) ? 0 : 1, Ctx::CcAlfFilterControlFlag( ctxt ) ); // ON/OFF flag is context coded
  if ( idcVal > 0 )
  {
    int val = (idcVal - 1);
    while ( val )
    {
      m_BinEncoder.encodeBinEP( 1 );
      val--;
    }
    if ( idcVal < filterCount )
    {
      m_BinEncoder.encodeBinEP( 0 );
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ccAlfFilterControlIdc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n", compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal );
}

void CABACWriter::code_unary_fixed( unsigned symbol, unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  bool unary = (symbol <= unary_max);
  m_BinEncoder.encodeBin( unary, ctxId );
  if( unary )
  {
    unary_max_eqprob( symbol, unary_max );
  }
  else
  {
    m_BinEncoder.encodeBinsEP( symbol - unary_max - 1, fixed );
  }
}

void CABACWriter::mip_flag( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->getUseMIP() )
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  m_BinEncoder.encodeBin( cu.mipFlag, Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACWriter::mip_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  for( const auto &pu : CU::traversePUs( cu ) )
  {
    mip_pred_mode( pu );
  }
}

void CABACWriter::mip_pred_mode( const PredictionUnit& pu )
{
  m_BinEncoder.encodeBinEP( (pu.mipTransposedFlag ? 1 : 0) );

  const int numModes = getNumModesMip( pu.Y() );
  CHECKD( pu.intraDir[CHANNEL_TYPE_LUMA] < 0 || pu.intraDir[CHANNEL_TYPE_LUMA] >= numModes, "Invalid MIP mode" );
  xWriteTruncBinCode( pu.intraDir[CHANNEL_TYPE_LUMA], numModes );

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA], pu.mipTransposedFlag ? 1 : 0 );
}

void CABACWriter::codeAlfCtuFilterIndex(CodingStructure& cs, uint32_t ctuRsAddr, bool alfEnableLuma)
{
  if ( (!cs.sps->getALFEnabledFlag()) || (!alfEnableLuma))
  {
    return;
  }

  uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag(COMPONENT_Y);
  if (!ctbAlfFlag[ctuRsAddr])
  {
    return;
  }

  short* alfCtbFilterIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  const unsigned filterSetIdx = alfCtbFilterIndex[ctuRsAddr];
  unsigned numAps = cs.slice->getTileGroupNumAps();
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    int useTemporalFilt = (filterSetIdx >= NUM_FIXED_FILTER_SETS) ? 1 : 0;
    m_BinEncoder.encodeBin(useTemporalFilt, Ctx::AlfUseTemporalFilt());
    if (useTemporalFilt)
    {
      CHECK_((filterSetIdx - NUM_FIXED_FILTER_SETS) >= (numAvailableFiltSets - NUM_FIXED_FILTER_SETS), "temporal non-latest set");
      if (numAps > 1)
      {
        xWriteTruncBinCode(filterSetIdx - NUM_FIXED_FILTER_SETS, numAvailableFiltSets - NUM_FIXED_FILTER_SETS);
      }
    }
    else
    {
      CHECK_(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set larger than temporal");
      xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
    }
  }
  else
  {
    CHECK_(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set numavail < num_fixed");
    xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
  }
}
void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isChroma( channel ) )
  {
    if (alfParam->enabledFlag[COMPONENT_Cb])
      codeAlfCtuAlternatives( cs, COMPONENT_Cb, alfParam );
    if (alfParam->enabledFlag[COMPONENT_Cr])
      codeAlfCtuAlternatives( cs, COMPONENT_Cr, alfParam );
  }
}
void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  if( compID == COMPONENT_Y )
    return;
  uint32_t numCTUs = cs.pcv->sizeInCtus;
  uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compID );

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    if( ctbAlfFlag[ctuIdx] )
    {
      codeAlfCtuAlternative( cs, ctuIdx, compID, alfParam );
    }
  }
}

void CABACWriter::codeAlfCtuAlternative( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, const AlfParam* alfParam)
{
  if( compIdx == COMPONENT_Y )
    return;
  int apsIdx = alfParam ? 0 : cs.slice->getTileGroupApsIdChroma();
  const AlfParam& alfParamRef = alfParam ? (*alfParam) : cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();

  if( alfParam || (cs.sps->getALFEnabledFlag() && cs.slice->getTileGroupAlfEnabledFlag( (ComponentID)compIdx )) )
  {
    uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );

    if( ctbAlfFlag[ctuRsAddr] )
    {
      const int numAlts = alfParamRef.numAlternativesChroma;
      uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData( compIdx );
      unsigned numOnes = ctbAlfAlternative[ctuRsAddr];
      assert( ctbAlfAlternative[ctuRsAddr] < numAlts );
      for( int i = 0; i < numOnes; ++i )
        m_BinEncoder.encodeBin( 1, Ctx::ctbAlfAlternative( compIdx-1 ) );
      if( numOnes < numAlts-1 )
        m_BinEncoder.encodeBin( 0, Ctx::ctbAlfAlternative( compIdx-1 ) );
    }
  }
}


//! \}

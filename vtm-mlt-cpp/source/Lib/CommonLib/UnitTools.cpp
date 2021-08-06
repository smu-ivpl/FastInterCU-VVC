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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools


uint64_t CS::getEstBits(const CodingStructure &cs)
{
  return cs.fracBits >> SCALE_BITS;
}

bool CS::isDualITree( const CodingStructure &cs )
{
  return cs.slice->isIntra() && !cs.pcv->ISingleTree;
}

UnitArea CS::getArea( const CodingStructure &cs, const UnitArea &area, const ChannelType chType )
{
  return isDualITree( cs ) || cs.treeType != TREE_D ? area.singleChan( chType ) : area;
}

void CS::setRefinedMotionField(CodingStructure &cs)
{
  for (CodingUnit *cu : cs.cus)
  {
    for (auto &pu : CU::traversePUs(*cu))
    {
      PredictionUnit subPu = pu;
      int dx, dy, x, y, num = 0;
      dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
      dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
      Position puPos = pu.lumaPos();
      if (PU::checkDMVRCondition(pu))
      {
        for (y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy)
        {
          for (x = puPos.x; x < (puPos.x + pu.lumaSize().width); x = x + dx)
          {
            subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
            subPu.mv[0] = pu.mv[0];
            subPu.mv[1] = pu.mv[1];
            subPu.mv[REF_PIC_LIST_0] += pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_1] -= pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_0].clipToStorageBitDepth();
            subPu.mv[REF_PIC_LIST_1].clipToStorageBitDepth();
            pu.mvdL0SubPu[num].setZero();
            num++;
            PU::spanMotionInfo(subPu);
          }
        }
      }
    }
  }
}
// CU tools

bool CU::getRprScaling( const SPS* sps, const PPS* curPPS, Picture* refPic, int& xScale, int& yScale )
{
  const Window& curScalingWindow = curPPS->getScalingWindow();
  int curPicWidth = curPPS->getPicWidthInLumaSamples()   - SPS::getWinUnitX( sps->getChromaFormatIdc() ) * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset());
  int curPicHeight = curPPS->getPicHeightInLumaSamples() - SPS::getWinUnitY( sps->getChromaFormatIdc() ) * (curScalingWindow.getWindowTopOffset()  + curScalingWindow.getWindowBottomOffset());

  const Window& refScalingWindow = refPic->getScalingWindow();
  int refPicWidth = refPic->getPicWidthInLumaSamples()   - SPS::getWinUnitX( sps->getChromaFormatIdc() ) * (refScalingWindow.getWindowLeftOffset() + refScalingWindow.getWindowRightOffset());
  int refPicHeight = refPic->getPicHeightInLumaSamples() - SPS::getWinUnitY( sps->getChromaFormatIdc() ) * (refScalingWindow.getWindowTopOffset()  + refScalingWindow.getWindowBottomOffset());

  xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
  yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;

  int curSeqMaxPicWidthY = sps->getMaxPicWidthInLumaSamples();                  // sps_pic_width_max_in_luma_samples
  int curSeqMaxPicHeightY = sps->getMaxPicHeightInLumaSamples();                // sps_pic_height_max_in_luma_samples
  int curPicWidthY = curPPS->getPicWidthInLumaSamples();                        // pps_pic_width_in_luma_samples
  int curPicHeightY = curPPS->getPicHeightInLumaSamples();                      // pps_pic_height_in_luma_samples
  int max8MinCbSizeY = std::max((int)8, (1<<sps->getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)

  CHECK_((curPicWidth * curSeqMaxPicWidthY) < refPicWidth * (curPicWidthY - max8MinCbSizeY), "(curPicWidth * curSeqMaxPicWidthY) should be greater than or equal to refPicWidth * (curPicWidthY - max8MinCbSizeY))");
  CHECK_((curPicHeight * curSeqMaxPicHeightY) < refPicHeight * (curPicHeightY - max8MinCbSizeY), "(curPicHeight * curSeqMaxPicHeightY) should be greater than or equal to refPicHeight * (curPicHeightY - max8MinCbSizeY))");

  CHECK_(curPicWidth * 2 < refPicWidth, "curPicWidth * 2 shall be greater than or equal to refPicWidth");
  CHECK_(curPicHeight * 2 < refPicHeight, "curPicHeight * 2 shall be greater than or equal to refPicHeight");
  CHECK_(curPicWidth > refPicWidth * 8, "curPicWidth shall be less than or equal to refPicWidth * 8");
  CHECK_(curPicHeight > refPicHeight * 8, "curPicHeight shall be less than or equal to refPicHeight * 8");

  int subWidthC = SPS::getWinUnitX(sps->getChromaFormatIdc());
  int subHeightC = SPS::getWinUnitY(sps->getChromaFormatIdc());

  CHECK_(subWidthC * curScalingWindow.getWindowLeftOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_left_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK_(subWidthC * curScalingWindow.getWindowLeftOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_left_offset shall be less than pps_pic_width_in_luma_samples");
  CHECK_(subWidthC * curScalingWindow.getWindowRightOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_right_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK_(subWidthC * curScalingWindow.getWindowRightOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_right_offset shall be less than pps_pic_width_in_luma_samples");

  CHECK_(subHeightC * curScalingWindow.getWindowTopOffset() < (-curPicHeightY) * 15, "The value of SubHeightC * pps_scaling_win_top_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK_(subHeightC * curScalingWindow.getWindowTopOffset() >= curPicHeightY, "The value of SubHeightC * pps_scaling_win_top_offset shall be less than pps_pic_height_in_luma_samples");
  CHECK_(subHeightC * curScalingWindow.getWindowBottomOffset() < (-curPicHeightY) * 15, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK_(subHeightC * curScalingWindow.getWindowBottomOffset() >= curPicHeightY, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be less than pps_pic_height_in_luma_samples");

  CHECK_(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) < (-curPicWidthY) * 15, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK_(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) >= curPicWidthY, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be less than pps_pic_width_in_luma_samples");
  CHECK_(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) < (-curPicHeightY) * 15, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK_(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) >= curPicHeightY, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be less than pps_pic_height_in_luma_samples");

  return refPic->isRefScaled( curPPS );
}

void CU::checkConformanceILRP(Slice *slice)
{
  const int numRefList = slice->isInterB() ? 2 : 1;

  int currentSubPicIdx = NOT_VALID;

  // derive sub-picture index for the current slice
  for( int subPicIdx = 0; subPicIdx < slice->getPic()->cs->sps->getNumSubPics(); subPicIdx++ )
  {
    if( slice->getPic()->cs->pps->getSubPic( subPicIdx ).getSubPicID() == slice->getSliceSubPicId() )
    {
      currentSubPicIdx = subPicIdx;
      break;
    }
  }

  CHECK_( currentSubPicIdx == NOT_VALID, "Sub-picture was not found" );

  if( !slice->getPic()->cs->sps->getSubPicTreatedAsPicFlag( currentSubPicIdx ) )
  {
    return;
  }

  //constraint 1: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] has the same subpicture layout as the current picture 
  bool isAllRefSameSubpicLayout = true;
  for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
  {
    RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
    {
      const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );

      if( refPic->subPictures.size() != slice->getPic()->cs->pps->getNumSubPics() )
      {
        isAllRefSameSubpicLayout = false;
        refList = numRefList;
        break;
      }
      else
      {
        for( int i = 0; i < refPic->subPictures.size(); i++ )
        {
          const SubPic& refSubPic = refPic->subPictures[i];
          const SubPic& curSubPic = slice->getPic()->cs->pps->getSubPic( i );

          if( refSubPic.getSubPicWidthInCTUs() != curSubPic.getSubPicWidthInCTUs()
            || refSubPic.getSubPicHeightInCTUs() != curSubPic.getSubPicHeightInCTUs()
            || refSubPic.getSubPicCtuTopLeftX() != curSubPic.getSubPicCtuTopLeftX()
            || refSubPic.getSubPicCtuTopLeftY() != curSubPic.getSubPicCtuTopLeftY()
            || ( refPic->layerId != slice->getPic()->layerId && refSubPic.getSubPicID() != curSubPic.getSubPicID() )
            || refSubPic.getTreatedAsPicFlag() != curSubPic.getTreatedAsPicFlag())
          {
            isAllRefSameSubpicLayout = false;
            refIdx = slice->getNumRefIdx(eRefPicList);
            refList = numRefList;
            break;
          }
        }

        // A picture with different sub-picture ID of the collocated sub-picture cannot be used as an active reference picture in the same layer
        if( refPic->layerId == slice->getPic()->layerId )
        {
          isAllRefSameSubpicLayout = isAllRefSameSubpicLayout && refPic->subPictures[currentSubPicIdx].getSubPicID() == slice->getSliceSubPicId();
        }
      }
    }
  }

  //constraint 2: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] is an ILRP for which the value of sps_num_subpics_minus1 is equal to 0
  if (!isAllRefSameSubpicLayout)
  {
    for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
    {
      RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
      for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
      {
        const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );
        CHECK_( refPic->layerId == slice->getPic()->layerId || refPic->subPictures.size() > 1, "The inter-layer reference shall contain a single subpicture or have same subpicture layout with the current picture" );
      }
    }
  }

  return;
}

bool CU::isIntra(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTRA;
}

bool CU::isInter(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTER;
}

bool CU::isIBC(const CodingUnit &cu)
{
  return cu.predMode == MODE_IBC;
}

bool CU::isPLT(const CodingUnit &cu)
{
  return cu.predMode == MODE_PLT;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return ( cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx() ) && ( cu.tileIdx == cu2.tileIdx );
}

bool CU::isSameSubPic(const CodingUnit& cu, const CodingUnit& cu2)
{
  return (cu.slice->getPPS()->getSubPicFromCU(cu).getSubPicIdx() == cu2.slice->getPPS()->getSubPicFromCU(cu2).getSubPicIdx()) ;
}

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  uint32_t ctuSizeBit = floorLog2(cu.cs->sps->getMaxCUWidth());

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

bool CU::isLastSubCUOfCtu( const CodingUnit &cu )
{
  const Area cuAreaY = cu.isSepTree() ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : (const Area&)cu.Y();


  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == cu.cs->pps->getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == cu.cs->pps->getPicHeightInLumaSamples() ) );
}

uint32_t CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType].lumaPos(), *cu.cs->pcv );
}

int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const CodingStructure &cs = *cu.cs;

  uint32_t  ctuRsAddr       = getCtuAddr( cu );
  uint32_t  ctuXPosInCtus   = ctuRsAddr % cs.pcv->widthInCtus;
  uint32_t  tileColIdx      = cu.slice->getPPS()->ctuToTileCol( ctuXPosInCtus );
  uint32_t  tileXPosInCtus  = cu.slice->getPPS()->getTileColumnBd( tileColIdx );
  if( ctuXPosInCtus == tileXPosInCtus &&
      !( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) &&
      !( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) &&
      ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType) != NULL ) &&
      CU::isSameSliceAndTile( *cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType), cu ) )
  {
    return ( ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType ) )->qp );
  }
  else
  {
    const int a = ( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType))->qp : prevQP;
    const int b = ( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( -1, 0 ), cu.chType))->qp : prevQP;

    return ( a + b + 1 ) >> 1;
  }
}


uint32_t CU::getNumPUs( const CodingUnit& cu )
{
  uint32_t cnt = 0;
  PredictionUnit *pu = cu.firstPU;

  do
  {
    cnt++;
  } while( ( pu != cu.lastPU ) && ( pu = pu->next ) );

  return cnt;
}

void CU::addPUs( CodingUnit& cu )
{
  cu.cs->addPU( CS::getArea( *cu.cs, cu, cu.chType ), cu.chType );
}

void CU::saveMotionInHMVP( const CodingUnit& cu, const bool isToBeDone )
{
  const PredictionUnit& pu = *cu.firstPU;

  if (!cu.geoFlag && !cu.affine && !isToBeDone)
  {
    MotionInfo mi = pu.getMotionInfo();

    mi.BcwIdx = (mi.interDir == 3) ? cu.BcwIdx : BCW_DEFAULT;

    const unsigned log2ParallelMergeLevel = (pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2);
    const unsigned xBr = pu.cu->Y().width + pu.cu->Y().x;
    const unsigned yBr = pu.cu->Y().height + pu.cu->Y().y;
    bool enableHmvp = ((xBr >> log2ParallelMergeLevel) > (pu.cu->Y().x >> log2ParallelMergeLevel)) && ((yBr >> log2ParallelMergeLevel) > (pu.cu->Y().y >> log2ParallelMergeLevel));
    bool enableInsertion = CU::isIBC(cu) || enableHmvp;
    if (enableInsertion)
    cu.cs->addMiToLut(CU::isIBC(cu) ? cu.cs->motionLut.lutIbc : cu.cs->motionLut.lut, mi);
  }
}

PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  if (depth >= cu.depth)
  {
    return CU_DONT_SPLIT;
  }

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if (cuSplitType == CU_QUAD_SPLIT)
  {
    return CU_QUAD_SPLIT;
  }

  else if (cuSplitType == CU_HORZ_SPLIT)
  {
    return CU_HORZ_SPLIT;
  }

  else if (cuSplitType == CU_VERT_SPLIT)
  {
    return CU_VERT_SPLIT;
  }

  else if (cuSplitType == CU_TRIH_SPLIT)
  {
    return CU_TRIH_SPLIT;
  }
  else if (cuSplitType == CU_TRIV_SPLIT)
  {
    return CU_TRIV_SPLIT;
  }
  else
  {
    THROW("Unknown split mode");
    return CU_QUAD_SPLIT;
  }
}

ModeType CU::getModeTypeAtDepth( const CodingUnit& cu, const unsigned depth )
{
  ModeType modeType = ModeType( (cu.modeTypeSeries >> (depth * 3)) & 0x07 );
  CHECK_( depth > cu.depth, " depth is wrong" );
  return modeType;
}



bool CU::divideTuInRows( const CodingUnit &cu )
{
  CHECK_( cu.ispMode != HOR_INTRA_SUBPARTITIONS && cu.ispMode != VER_INTRA_SUBPARTITIONS, "Intra Subpartitions type not recognized!" );
  return cu.ispMode == HOR_INTRA_SUBPARTITIONS ? true : false;
}


PartSplit CU::getISPType( const CodingUnit &cu, const ComponentID compID )
{
  if( cu.ispMode && isLuma( compID ) )
  {
    const bool tuIsDividedInRows = CU::divideTuInRows( cu );

    return tuIsDividedInRows ? TU_1D_HORZ_SPLIT : TU_1D_VERT_SPLIT;
  }
  return TU_NO_ISP;
}

bool CU::isISPLast( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  PartSplit partitionType = CU::getISPType( cu, compID );

  Area originalArea = cu.blocks[compID];
  switch( partitionType )
  {
    case TU_1D_HORZ_SPLIT:
      return tuArea.y + tuArea.height == originalArea.y + originalArea.height;
    case TU_1D_VERT_SPLIT:
      return tuArea.x + tuArea.width == originalArea.x + originalArea.width;
    default:
      THROW( "Unknown ISP processing order type!" );
      return false;
  }
}

bool CU::isISPFirst( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  return tuArea == cu.firstTU->blocks[compID];
}

bool CU::canUseISP( const CodingUnit &cu, const ComponentID compID )
{
  const int width     = cu.blocks[compID].width;
  const int height    = cu.blocks[compID].height;
  const int maxTrSize = cu.cs->sps->getMaxTbSize();
  return CU::canUseISP( width, height, maxTrSize );
}

bool CU::canUseISP( const int width, const int height, const int maxTrSize )
{
  bool  notEnoughSamplesToSplit = ( floorLog2(width) + floorLog2(height) <= ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  bool  cuSizeLargerThanMaxTrSize = width > maxTrSize || height > maxTrSize;
  if ( notEnoughSamplesToSplit || cuSizeLargerThanMaxTrSize )
  {
    return false;
  }
  return true;
}

bool CU::canUseLfnstWithISP( const CompArea& cuArea, const ISPType ispSplitType )
{
  if( ispSplitType == NOT_INTRA_SUBPARTITIONS )
  {
    return false;
  }
  Size tuSize = ( ispSplitType == HOR_INTRA_SUBPARTITIONS ) ? Size( cuArea.width, CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_HORZ_SPLIT ) ) :
    Size( CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_VERT_SPLIT ), cuArea.height );

  if( !( tuSize.width >= MIN_TB_SIZEY && tuSize.height >= MIN_TB_SIZEY ) )
  {
    return false;
  }
  return true;
}

bool CU::canUseLfnstWithISP( const CodingUnit& cu, const ChannelType chType )
{
  CHECK_( !isLuma( chType ), "Wrong ISP mode!" );
  return CU::canUseLfnstWithISP( cu.blocks[chType == CHANNEL_TYPE_LUMA ? 0 : 1], (ISPType)cu.ispMode );
}

uint32_t CU::getISPSplitDim( const int width, const int height, const PartSplit ispType )
{
  bool divideTuInRows = ispType == TU_1D_HORZ_SPLIT;
  uint32_t splitDimensionSize, nonSplitDimensionSize, partitionSize, divShift = 2;

  if( divideTuInRows )
  {
    splitDimensionSize    = height;
    nonSplitDimensionSize = width;
  }
  else
  {
    splitDimensionSize    = width;
    nonSplitDimensionSize = height;
  }

  const int minNumberOfSamplesPerCu = 1 << ( ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  const int factorToMinSamples = nonSplitDimensionSize < minNumberOfSamplesPerCu ? minNumberOfSamplesPerCu >> floorLog2(nonSplitDimensionSize) : 1;
  partitionSize = ( splitDimensionSize >> divShift ) < factorToMinSamples ? factorToMinSamples : ( splitDimensionSize >> divShift );

  CHECK_( floorLog2(partitionSize) + floorLog2(nonSplitDimensionSize) < floorLog2(minNumberOfSamplesPerCu), "A partition has less than the minimum amount of samples!" );
  return partitionSize;
}

bool CU::allLumaCBFsAreZero(const CodingUnit& cu)
{
  if (!cu.ispMode)
  {
    return TU::getCbf(*cu.firstTU, COMPONENT_Y) == false;
  }
  else
  {
    int numTotalTUs = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(cu.firstTU->lheight()) : cu.lwidth() >> floorLog2(cu.firstTU->lwidth());
    TransformUnit* tuPtr = cu.firstTU;
    for (int tuIdx = 0; tuIdx < numTotalTUs; tuIdx++)
    {
      if (TU::getCbf(*tuPtr, COMPONENT_Y) == true)
      {
        return false;
      }
      tuPtr = tuPtr->next;
    }
    return true;
  }
}


PUTraverser CU::traversePUs( CodingUnit& cu )
{
  return PUTraverser( cu.firstPU, cu.lastPU->next );
}

TUTraverser CU::traverseTUs( CodingUnit& cu )
{
  return TUTraverser( cu.firstTU, cu.lastTU->next );
}

cPUTraverser CU::traversePUs( const CodingUnit& cu )
{
  return cPUTraverser( cu.firstPU, cu.lastPU->next );
}

cTUTraverser CU::traverseTUs( const CodingUnit& cu )
{
  return cTUTraverser( cu.firstTU, cu.lastTU->next );
}

// PU tools

int PU::getIntraMPMs( const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/ )
{
  const int numMPMs = NUM_MOST_PROBABLE_MODES;
  {
    CHECK_(channelType != CHANNEL_TYPE_LUMA, "Not harmonized yet");
    int numCand      = -1;
    int leftIntraDir = PLANAR_IDX, aboveIntraDir = PLANAR_IDX;

    const CompArea &area = pu.block(getFirstComponentOfChannel(channelType));
    const Position posRT = area.topRight();
    const Position posLB = area.bottomLeft();

    // Get intra direction of left PU
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType);
    if (puLeft && CU::isIntra(*puLeft->cu))
    {
      leftIntraDir = PU::getIntraDirLuma( *puLeft );
    }

    // Get intra direction of above PU
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType);
    if (puAbove && CU::isIntra(*puAbove->cu) && CU::isSameCtu(*pu.cu, *puAbove->cu))
    {
      aboveIntraDir = PU::getIntraDirLuma( *puAbove );
    }

    CHECK_(2 >= numMPMs, "Invalid number of most probable modes");

    const int offset = (int)NUM_LUMA_MODE - 6;
    const int mod = offset + 3;

    mpm[0] = PLANAR_IDX;
    mpm[1] = DC_IDX;
    mpm[2] = VER_IDX;
    mpm[3] = HOR_IDX;
    mpm[4] = VER_IDX - 4;
    mpm[5] = VER_IDX + 4;

    if (leftIntraDir == aboveIntraDir)
    {
      numCand = 1;
      if (leftIntraDir > DC_IDX)
      {
        mpm[0] = PLANAR_IDX;
        mpm[1] = leftIntraDir;
        mpm[2] = ((leftIntraDir + offset) % mod) + 2;
        mpm[3] = ((leftIntraDir - 1) % mod) + 2;
        mpm[4] = ((leftIntraDir + offset - 1) % mod) + 2;
        mpm[5] = (leftIntraDir % mod) + 2;
      }
    }
    else   // L!=A
    {
      numCand            = 2;
      int maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;

      if ((leftIntraDir > DC_IDX) && (aboveIntraDir > DC_IDX))
      {
        mpm[0]             = PLANAR_IDX;
        mpm[1]             = leftIntraDir;
        mpm[2]             = aboveIntraDir;
        maxCandModeIdx     = mpm[1] > mpm[2] ? 1 : 2;
        int minCandModeIdx = mpm[1] > mpm[2] ? 2 : 1;
        if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 1)
        {
          mpm[3] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] - 1) % mod) + 2;
          mpm[5] = ((mpm[minCandModeIdx] + offset - 1) % mod) + 2;
        }
        else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] >= 62)
        {
          mpm[3] = ((mpm[minCandModeIdx] - 1) % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
          mpm[5] = (mpm[minCandModeIdx] % mod) + 2;
        }
        else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 2)
        {
          mpm[3] = ((mpm[minCandModeIdx] - 1) % mod) + 2;
          mpm[4] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
          mpm[5] = ((mpm[maxCandModeIdx] - 1) % mod) + 2;
        }
        else
        {
          mpm[3] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
          mpm[4] = ((mpm[minCandModeIdx] - 1) % mod) + 2;
          mpm[5] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
        }
      }
      else if (leftIntraDir + aboveIntraDir >= 2)
      {
        mpm[0]         = PLANAR_IDX;
        mpm[1]         = (leftIntraDir < aboveIntraDir) ? aboveIntraDir : leftIntraDir;
        maxCandModeIdx = 1;
        mpm[2]         = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
        mpm[3]         = ((mpm[maxCandModeIdx] - 1) % mod) + 2;
        mpm[4]         = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
        mpm[5]         = (mpm[maxCandModeIdx] % mod) + 2;
      }
    }
    for (int i = 0; i < numMPMs; i++)
    {
      CHECK_(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
    }
    CHECK_(numCand == 0, "No candidates found");
    return numCand;
  }
}

bool PU::isMIP(const PredictionUnit &pu, const ChannelType &chType)
{
  if (chType == CHANNEL_TYPE_LUMA)
  {
    // Default case if chType is omitted.
    return pu.cu->mipFlag;
  }
  else
  {
    return isDMChromaMIP(pu) && (pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX);
  }
}

bool PU::isDMChromaMIP(const PredictionUnit &pu)
{
  return !pu.cu->isSepTree() && (pu.chromaFormat == CHROMA_444) && getCoLocatedLumaPU(pu).cu->mipFlag;
}

uint32_t PU::getIntraDirLuma( const PredictionUnit &pu )
{
  if (isMIP(pu))
  {
    return PLANAR_IDX;
  }
  else
  {
    return pu.intraDir[CHANNEL_TYPE_LUMA];
  }
}


void PU::getIntraChromaCandModes( const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE] )
{
  modeList[0] = PLANAR_IDX;
  modeList[1] = VER_IDX;
  modeList[2] = HOR_IDX;
  modeList[3] = DC_IDX;
  modeList[4] = LM_CHROMA_IDX;
  modeList[5] = MDLM_L_IDX;
  modeList[6] = MDLM_T_IDX;
  modeList[7] = DM_CHROMA_IDX;

  // If Direct Mode is MIP, mode cannot be already in the list.
  if (isDMChromaMIP(pu))
  {
    return;
  }

  const uint32_t lumaMode = getCoLocatedIntraLumaMode(pu);
  for (int i = 0; i < 4; i++)
  {
    if (lumaMode == modeList[i])
    {
      modeList[i] = VDIA_IDX;
      break;
    }
  }
}

bool PU::isLMCMode(unsigned mode)
{
  return (mode >= LM_CHROMA_IDX && mode <= MDLM_T_IDX);
}

bool PU::isLMCModeEnabled(const PredictionUnit &pu, unsigned mode)
{
  if ( pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed() )
  {
    return true;
  }
  return false;
}

int PU::getLMSymbolList(const PredictionUnit &pu, int *modeList)
{
  int idx = 0;

  modeList[idx++] = LM_CHROMA_IDX;
  modeList[idx++] = MDLM_L_IDX;
  modeList[idx++] = MDLM_T_IDX;
  return idx;
}

bool PU::isChromaIntraModeCrossCheckMode( const PredictionUnit &pu )
{
  return !pu.cu->bdpcmModeChroma && pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX;
}

uint32_t PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
  uint32_t uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
    uiIntraMode = getCoLocatedIntraLumaMode(pu);
  }
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) && uiIntraMode < NUM_LUMA_MODE ) // map directional, planar and dc
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
  return uiIntraMode;
}

const PredictionUnit &PU::getCoLocatedLumaPU(const PredictionUnit &pu)
{
  Position              topLeftPos = pu.blocks[pu.chType].lumaPos();
  Position              refPos     = topLeftPos.offset(pu.blocks[pu.chType].lumaSize().width  >> 1,
                                                       pu.blocks[pu.chType].lumaSize().height >> 1);
  const PredictionUnit &lumaPU     = pu.cu->isSepTree() ? *pu.cs->picture->cs->getPU(refPos, CHANNEL_TYPE_LUMA)
                                                        : *pu.cs->getPU(topLeftPos, CHANNEL_TYPE_LUMA);

  return lumaPU;
}

uint32_t PU::getCoLocatedIntraLumaMode(const PredictionUnit &pu)
{
  return PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu));
}

int PU::getWideAngle( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID )
{
  //This function returns a wide angle index taking into account that the values 0 and 1 are reserved 
  //for Planar and DC respectively, as defined in the Spec. Text.
  if( dirMode < 2 )
  {
    return ( int ) dirMode;
  }

  const CompArea&  area         = tu.cu->ispMode && isLuma(compID) ? tu.cu->blocks[compID] : tu.blocks[ compID ];
  int              width        = area.width;
  int              height       = area.height;
  int              modeShift[ ] = { 0, 6, 10, 12, 14, 15 };
  int              deltaSize    = abs( floorLog2( width ) - floorLog2( height ) );
  int              predMode     = dirMode;

  if( width > height && dirMode < 2 + modeShift[ deltaSize ] )
  {
    predMode += ( VDIA_IDX - 1 );
  }
  else if( height > width && predMode > VDIA_IDX - modeShift[ deltaSize ] )
  {
    predMode -= ( VDIA_IDX + 1 );
  }

  return predMode;
}

bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                          const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                          const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove,
                          const bool ibcFlag, const bool isGt4x4)
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;

  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
  int num_avai_candInLUT = (int)lut.size();

  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];

    if ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
    {
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
      mrgCtx.useAltHpelIf      [cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
      mrgCtx.BcwIdx            [cnt] = (miNeighbor.interDir == 3) ? miNeighbor.BcwIdx : BCW_DEFAULT;

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
      }

      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt ++;

      if (cnt  == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }

  return false;
}

void PU::getIBCMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumIBCMergeCand();
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours[ui] = MRG_TYPE_IBC;
    mrgCtx.mvFieldNeighbours[ui * 2].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui * 2 + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  bool isGt4x4 = pu.lwidth() * pu.lheight() > 16;
  const bool isAvailableA1 = puLeft && pu.cu != puLeft->cu && CU::isIBC(*puLeft->cu);
  if (isGt4x4 && isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
    if (mrgCandIdx == cnt)
    {
      return;
    }
    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool isAvailableB1 = puAbove && pu.cu != puAbove->cu && CU::isIBC(*puAbove->cu);
  if (isGt4x4 && isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    if (!isAvailableA1 || (miAbove != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Above
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (cnt != maxNumMergeCand)
  {
    bool bFound = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCand, cnt, isAvailableA1, miLeft, isAvailableB1,
                                   miAbove, true, isGt4x4);

    if (bFound)
    {
      return;
    }
  }

    while (cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), MAX_NUM_REF);
      mrgCtx.interDirNeighbours[cnt] = 1;
      if (mrgCandIdx == cnt)
      {
        return;
      }
      cnt++;
    }

  mrgCtx.numValidMergeCand = cnt;
}

void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx,
                                 int mmvdList,
                                 const int& mrgCandIdx )
{
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
  CHECK_(maxNumMergeCand > MRG_MAX_NUM_CANDS, "selected maximum number of merge candidate exceeds global limit");
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours [ui] = MRG_TYPE_DEFAULT_N;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);

  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu);

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
    mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
    // get Mv from Above
    mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->BcwIdx : BCW_DEFAULT;
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);

    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);
    }
    if (mrgCandIdx == cnt)
    {
      return;
    }

    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);

  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    if (!isAvailableB1 || (miAbove != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
      mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->BcwIdx : BCW_DEFAULT;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
      }
      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );

  bool isAvailableB0 = puAboveRight && isDiffMER( pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter( *puAboveRight->cu );

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

    if( !isAvailableB1 || ( miAbove != miAboveRight ) )
    {

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
      // get Mv from Above-right
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->BcwIdx : BCW_DEFAULT;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
      }

      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );

  bool isAvailableA0 = puLeftBottom && isDiffMER( pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter( *puLeftBottom->cu );

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->BcwIdx : BCW_DEFAULT;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
      }

      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  // above left
  if ( cnt < 4 )
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

    bool isAvailableB2 = puAboveLeft && isDiffMER( pu.lumaPos(), posLT.offset(-1, -1), plevel ) && CU::isInter( *puAboveLeft->cu );

    if( isAvailableB2 )
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )
      {

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
        mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->BcwIdx : BCW_DEFAULT;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
        }

        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset( -3, -3 );
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool C0Avail = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx, false ) )
                              || getColocatedMVP( pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx, false );
    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx, false ) )
                   || getColocatedMVP( pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx, false );
      if (bExistMV)
      {
        dir     |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if( dir != 0 )
    {
      bool addTMvp = true;
      if( addTMvp )
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        mrgCtx.BcwIdx[uiArrayAddr] = BCW_DEFAULT;
        mrgCtx.useAltHpelIf[uiArrayAddr] = false;
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
  if (cnt != maxNumMergeCandMin1)
  {
    bool isGt4x4 = true;
    bool bFound  = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt, isAvailableA1, miLeft,
                                   isAvailableB1, miAbove, CU::isIBC(*pu.cu), isGt4x4);

    if (bFound)
    {
      return;
    }
  }

  // pairwise-average candidates
  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField( Mv( 0, 0 ), NOT_VALID );
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField( Mv( 0, 0 ), NOT_VALID );
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;

      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for( int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++ )
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

        // both MVs are invalid, skip
        if( (refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID) )
        {
          continue;
        }

        interDir += 1 << refListId;
        // both MVs are valid, average these two MVs
        if( (refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID) )
        {
          const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

          // average two MVs
          Mv avgMv = MvI;
          avgMv += MvJ;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);

          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( avgMv, refIdxI );
        }
        // only one MV is valid, take the only one MV
        else if( refIdxI != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxI );
        }
        else if( refIdxJ != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxJ );
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if( interDir > 0 )
      {
        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  uint32_t uiArrayAddr = cnt;

  int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);

  int r = 0;
  int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.BcwIdx             [uiArrayAddr     ] = BCW_DEFAULT;
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);
    mrgCtx.useAltHpelIf[uiArrayAddr] = false;

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    if ( mrgCtx.interDirNeighbours[uiArrayAddr] == 1 && pu.cs->slice->getRefPic(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[uiArrayAddr << 1].refIdx)->getPOC() == pu.cs->slice->getPOC())
    {
      mrgCtx.mrgTypeNeighbours[uiArrayAddr] = MRG_TYPE_IBC;
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  mrgCtx.numValidMergeCand = uiArrayAddr;
}

bool PU::checkDMVRCondition(const PredictionUnit& pu)
{
  if (pu.cs->sps->getUseDMVR() && !pu.cs->picHeader->getDmvrDisabledFlag())
  {
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cu->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
    const WPScalingParam *wp1 = pu.cu->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
                                ? false
                                : pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdx0)->isRefScaled(pu.cs->pps);
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
                                ? false
                                : pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdx1)->isRefScaled(pu.cs->pps);

    return pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
           && !pu.cu->mmvdSkip && PU::isBiPredFromDifferentDirEqDistPoc(pu) && (pu.lheight() >= 8) && (pu.lwidth() >= 8)
           && ((pu.lheight() * pu.lwidth()) >= 128) && (pu.cu->BcwIdx == BCW_DEFAULT)
           && !WPScalingParam::isWeighted(wp0) && !WPScalingParam::isWeighted(wp1) && !ref0IsScaled && !ref1IsScaled;
  }
  else
  {
    return false;
  }
}


static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  int iDiffPocD = iColPOC - iColRefPOC;
  int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    int iTDB = Clip3(-128, 127, iDiffPocB);
    int iTDD = Clip3(-128, 127, iDiffPocD);
    int iX = (0x4000 + abs(iTDD / 2)) / iTDD;
    int iScale = Clip3(-4096, 4095, (iTDB * iX + 32) >> 6);
    return iScale;
  }
}

int convertMvFixedToFloat(int32_t val)
{
  int sign  = val >> 31;
  int scale = floorLog2((val ^ sign) | MV_MANTISSA_UPPER_LIMIT) - (MV_MANTISSA_BITCOUNT - 1);

  int exponent;
  int mantissa;
  if (scale >= 0)
  {
    int round = (1 << scale) >> 1;
    int n     = (val + round) >> scale;
    exponent  = scale + ((n ^ sign) >> (MV_MANTISSA_BITCOUNT - 1));
    mantissa  = (n & MV_MANTISSA_UPPER_LIMIT) | (sign << (MV_MANTISSA_BITCOUNT - 1));
  }
  else
  {
    exponent = 0;
    mantissa = val;
  }

  return exponent | (mantissa << MV_EXPONENT_BITCOUNT);
}

int convertMvFloatToFixed(int val)
{
  int exponent = val & MV_EXPONENT_MASK;
  int mantissa = val >> MV_EXPONENT_BITCOUNT;
  return exponent == 0 ? mantissa : (mantissa ^ MV_MANTISSA_LIMIT) << (exponent - 1);
}

int roundMvComp(int x)
{
  return convertMvFloatToFixed(convertMvFixedToFloat(x));
}

int PU::getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC)
{
  return xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);
}

void PU::getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  int refIdxList0, refIdxList1;
  int k;
  int currBaseNum = 0;
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;

  for (k = 0; k < maxNumMergeCand; k++)
  {
    if (mrgCtx.mrgTypeNeighbours[k] == MRG_TYPE_DEFAULT_N)
    {
      refIdxList0 = mrgCtx.mvFieldNeighbours[(k << 1)].refIdx;
      refIdxList1 = mrgCtx.mvFieldNeighbours[(k << 1) + 1].refIdx;

      if ((refIdxList0 >= 0) && (refIdxList1 >= 0))
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      else if (refIdxList0 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = MvField(Mv(0, 0), -1);
      }
      else if (refIdxList1 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = MvField(Mv(0, 0), -1);
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      mrgCtx.mmvdUseAltHpelIf[currBaseNum] = mrgCtx.useAltHpelIf[k];

      currBaseNum++;

      if (currBaseNum == MMVD_BASE_MV_NUM)
      {
        break;
      }
    }
  }
}
bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv, const int &refIdx, bool sbFlag)
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~( scale - 1 );

  const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

  const Slice &slice = *pu.cs->slice;

  // use coldir.
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

  if( !pColPic )
  {
    return false;
  }

  // Check the position of colocated block is within a subpicture
  const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
  if (curSubPic.getTreatedAsPicFlag())
  {
    if (!curSubPic.isContainingPos(pos))
    {
      return false;
    }
  }
  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());

  const MotionInfo& mi = pColPic->cs->getMotionInfo( pos );

  if( !mi.isInter )
  {
    return false;
  }
  if (mi.isIBCmot)
  {
    return false;
  }
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (sbFlag && !slice.getCheckLDC())
  {
    eColRefPicList = eRefPicList;
    iColRefIdx = mi.refIdx[eColRefPicList];
    if (iColRefIdx < 0)
    {
      return false;
    }
  }
  else
  {
    if (iColRefIdx < 0)
    {
      eColRefPicList = RefPicList(1 - eColRefPicList);
      iColRefIdx = mi.refIdx[eColRefPicList];

      if (iColRefIdx < 0)
      {
        return false;
      }
    }
  }

  const Slice *pColSlice = nullptr;

  for( const auto s : pColPic->slices )
  {
    if( s->getIndependentSliceIdx() == mi.sliceIdx )
    {
      pColSlice = s;
      break;
    }
  }

  CHECK_( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;

  const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->longTerm;
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];
  cColMv.setHor(roundMvComp(cColMv.getHor()));
  cColMv.setVer(roundMvComp(cColMv.getVer()));

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
    rcMv.clipToStorageBitDepth();
  }
  else
  {
    const int currPOC    = slice.getPOC();
    const int colPOC     = colSlice.getPOC();
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->getPOC();
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
      rcMv.clipToStorageBitDepth();
    }
    else
    {
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER(const Position &pos1, const Position &pos2, const unsigned plevel)
{
  const unsigned xN = pos1.x;
  const unsigned yN = pos1.y;
  const unsigned xP = pos2.x;
  const unsigned yP = pos2.y;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

bool PU::isAddNeighborMv(const Mv& currMv, Mv* neighborMvs, int numNeighborMv)
{
  bool existed = false;
  for (uint32_t cand = 0; cand < numNeighborMv && !existed; cand++)
  {
    if (currMv == neighborMvs[cand])
    {
      existed = true;
    }
  }

  if (!existed)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PU::getIbcMVPsEncOnly(PredictionUnit &pu, Mv* mvPred, int& nbPred)
{
  const PreCalcValues   &pcv = *pu.cs->pcv;
  const int  cuWidth = pu.blocks[COMPONENT_Y].width;
  const int  cuHeight = pu.blocks[COMPONENT_Y].height;
  const int  log2UnitWidth = floorLog2(pcv.minCUWidth);
  const int  log2UnitHeight = floorLog2(pcv.minCUHeight);
  const int  totalAboveUnits = (cuWidth >> log2UnitWidth) + 1;
  const int  totalLeftUnits = (cuHeight >> log2UnitHeight) + 1;

  nbPred = 0;
  Position posLT = pu.Y().topLeft();

  // above-left
  const PredictionUnit *aboveLeftPU = pu.cs->getPURestricted(posLT.offset(-1, -1), pu, CHANNEL_TYPE_LUMA);
  if (aboveLeftPU && CU::isIBC(*aboveLeftPU->cu))
  {
    if (isAddNeighborMv(aboveLeftPU->bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = aboveLeftPU->bv;
    }
  }

  // above neighbors
  for (uint32_t dx = 0; dx < totalAboveUnits && nbPred < IBC_NUM_CANDIDATES; dx++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset((dx << log2UnitWidth), -1), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  // left neighbors
  for (uint32_t dy = 0; dy < totalLeftUnits && nbPred < IBC_NUM_CANDIDATES; dy++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset(-1, (dy << log2UnitHeight)), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  size_t numAvaiCandInLUT = pu.cs->motionLut.lutIbc.size();
  for (uint32_t cand = 0; cand < numAvaiCandInLUT && nbPred < IBC_NUM_CANDIDATES; cand++)
  {
    MotionInfo neibMi = pu.cs->motionLut.lutIbc[cand];
    if (isAddNeighborMv(neibMi.bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = neibMi.bv;
    }
  }

  bool isBvCandDerived[IBC_NUM_CANDIDATES];
  ::memset(isBvCandDerived, false, IBC_NUM_CANDIDATES);

  int curNbPred = nbPred;
  if (curNbPred < IBC_NUM_CANDIDATES)
  {
    do
    {
      curNbPred = nbPred;
      for (uint32_t idx = 0; idx < curNbPred && nbPred < IBC_NUM_CANDIDATES; idx++)
      {
        if (!isBvCandDerived[idx])
        {
          Mv derivedBv;
          if (getDerivedBV(pu, mvPred[idx], derivedBv))
          {
            if (isAddNeighborMv(derivedBv, mvPred, nbPred))
            {
              mvPred[nbPred++] = derivedBv;
            }
          }
          isBvCandDerived[idx] = true;
        }
      }
    } while (nbPred > curNbPred && nbPred < IBC_NUM_CANDIDATES);
  }
}

bool PU::getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv)
{
  int   cuPelX = pu.lumaPos().x;
  int   cuPelY = pu.lumaPos().y;
  int rX = cuPelX + currentMv.getHor();
  int rY = cuPelY + currentMv.getVer();
  int offsetX = currentMv.getHor();
  int offsetY = currentMv.getVer();

  if( rX < 0 || rY < 0 || rX >= pu.cs->slice->getPPS()->getPicWidthInLumaSamples() || rY >= pu.cs->slice->getPPS()->getPicHeightInLumaSamples() )
  {
    return false;
  }

  const PredictionUnit *neibRefPU = NULL;
  neibRefPU = pu.cs->getPURestricted(pu.lumaPos().offset(offsetX, offsetY), pu, CHANNEL_TYPE_LUMA);

  bool isIBC = (neibRefPU) ? CU::isIBC(*neibRefPU->cu) : 0;
  if (isIBC)
  {
    derivedMv = neibRefPU->bv;
    derivedMv += currentMv;
  }
  return isIBC;
}

/**
 * Constructs a list of candidates for IBC AMVP (See specification, section "Derivation process for motion vector predictor candidates")
 */
void PU::fillIBCMvpCand(PredictionUnit &pu, AMVPInfo &amvpInfo)
{
  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  MergeCtx mergeCtx;
  PU::getIBCMergeCandidates(pu, mergeCtx, AMVP_MAX_NUM_CANDS - 1);
  int candIdx = 0;
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;;
    pInfo->numCand++;
    candIdx++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundIbcPrecInternal2Amvr(pu.cu->imv);
  }
}


/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo)
{
  CodingStructure &cs = *pu.cs;

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );

    }
  }

  // Above predictor search
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && (pu.lumaSize().width + pu.lumaSize().height > 12))
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool C0Avail = false;
    Position posC1 = pu.Y().center();
    Mv cColMv;

    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }
    if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdx_Col, false ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdx_Col, false ) )
    {
      cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }

  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    const int currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
    addAMVPHMVPCand(pu, eRefPicList, currRefPOC, *pInfo);
  }

  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
    pInfo->numCand++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundTransPrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo )
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  switch ( dir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1, 0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset( 0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset( 1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1, 1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if (neibPU == NULL || !CU::isInter(*neibPU->cu) || !neibPU->cu->affine || neibPU->mergeType != MRG_TYPE_DEFAULT_N)
  {
    return false;
  }

  Mv outputAffineMv[3];
  const MotionInfo& neibMi = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC = cs.slice->getRefPic( refPicList, refIdx )->getPOC();
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for ( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

    if ( ((neibPU->interDir & (eRefPicListIndex + 1)) == 0) || pu.cu->slice->getRefPOC( eRefPicListIndex, neibRefIdx ) != currRefPOC )
    {
      continue;
    }

    xInheritedAffineMv( pu, neibPU, eRefPicListIndex, outputAffineMv );
    outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
    outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    }
    affiAMVPInfo.numCand++;
    return true;
  }

  return false;
}

void PU::xInheritedAffineMv( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;

  Mv mvLT, mvRT, mvLB;
  mvLT = puNeighbour->mvAffi[eRefPicList][0];
  mvRT = puNeighbour->mvAffi[eRefPicList][1];
  mvLB = puNeighbour->mvAffi[eRefPicList][2];

  bool isTopCtuBoundary = false;
  if ( (posNeiY + neiH) % pu.cs->sps->getCTUSize() == 0 && (posNeiY + neiH) == posCurY )
  {
    // use bottom-left and bottom-right sub-block MVs for inheritance
    const Position posRB = puNeighbour->Y().bottomRight();
    const Position posLB = puNeighbour->Y().bottomLeft();
    mvLT = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];
    mvRT = puNeighbour->getMotionInfo( posRB ).mv[eRefPicList];
    posNeiY += neiH;
    isTopCtuBoundary = true;
  }

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(neiW));
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(neiW));
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM && !isTopCtuBoundary )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - floorLog2(neiH));
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - floorLog2(neiH));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;
  rcMv[0].clipToStorageBitDepth();

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;
  rcMv[1].clipToStorageBitDepth();

  // v2
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2].hor = horTmp;
    rcMv[2].ver = verTmp;
    rcMv[2].clipToStorageBitDepth();
  }
}


void PU::fillAffineMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  // insert inherited affine candidates
  Mv outputAffineMv[3];
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  // check left neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo ) )
  {
    addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo );
  }

  // check above neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo ) )
  {
    if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo ) )
    {
      addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo );
    }
  }

  if ( affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS )
  {
    for (int i = 0; i < affiAMVPInfo.numCand; i++)
    {
      affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    }
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0 );
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0 );
  }
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0 );
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1 );
  if ( amvpInfo1.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2 );
  if ( amvpInfo2.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo2.numCand << 2);

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);

  if ( cornerMVPattern == 7 || (cornerMVPattern == 3 && pu.cu->affineType == AFFINEMODEL_4PARAM) )
  {
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    affiAMVPInfo.numCand++;
  }

  if ( affiAMVPInfo.numCand < 2 )
  {
    // check corner MVs
    for ( int i = 2; i >= 0 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; i-- )
    {
      if ( cornerMVPattern & (1 << i) ) // MV i exist
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.numCand++;
      }
    }

    // Get Temporal Motion Predictor
    if ( affiAMVPInfo.numCand < 2 && pu.cs->picHeader->getEnableTMVPFlag() )
    {
      const int refIdxCol = refIdx;

      Position posRB = pu.Y().bottomRight().offset( -3, -3 );

      const PreCalcValues& pcv = *pu.cs->pcv;

      Position posC0;
      bool C0Avail = false;
      Position posC1 = pu.Y().center();
      Mv cColMv;
      bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
      const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
      if (curSubPic.getTreatedAsPicFlag())
      {
        boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
          (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
      }
      if (boundaryCond)
      {
        int posYInCtu = posRB.y & pcv.maxCUHeightMask;
        if (posYInCtu + 4 < pcv.maxCUHeight)
        {
          posC0 = posRB.offset(4, 4);
          C0Avail = true;
        }
      }
      if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdxCol, false ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdxCol, false ) )
      {
        cColMv.roundAffinePrecInternal2Amvr(pu.cu->imv);
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.numCand++;
      }
    }

    if ( affiAMVPInfo.numCand < 2 )
    {
      // add zero MV
      for ( int i = affiAMVPInfo.numCand; i < AMVP_MAX_NUM_CANDS; i++ )
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.numCand++;
      }
    }
  }

  for (int i = 0; i < affiAMVPInfo.numCand; i++)
  {
    affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC     = cs.slice->getRefPic( eRefPicList, iRefIdx )->getPOC();
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if( neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
      info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
      return true;
    }
  }

  return false;
}


void PU::addAMVPHMVPCand(const PredictionUnit &pu, const RefPicList eRefPicList, const int currRefPOC, AMVPInfo &info)
{
  const Slice &slice = *(*pu.cs).slice;

  MotionInfo neibMi;
  auto &lut = CU::isIBC(*pu.cu) ? pu.cs->motionLut.lutIbc : pu.cs->motionLut.lut;
  int num_avai_candInLUT = (int) lut.size();
  int num_allowedCand = std::min(MAX_NUM_HMVP_AVMPCANDS, num_avai_candInLUT);
  const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for (int mrgIdx = 1; mrgIdx <= num_allowedCand; mrgIdx++)
  {
    if (info.numCand >= AMVP_MAX_NUM_CANDS)
    {
      return;
    }
    neibMi = lut[mrgIdx - 1];

    for (int predictorSource = 0; predictorSource < 2; predictorSource++)
    {
      const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
      const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

      if (neibRefIdx >= 0 && (CU::isIBC(*pu.cu) || (currRefPOC == slice.getRefPOC(eRefPicListIndex, neibRefIdx))))
      {
        Mv pmv = neibMi.mv[eRefPicListIndex];
        pmv.roundTransPrecInternal2Amvr(pu.cu->imv);

        info.mvCand[info.numCand++] = pmv;
        if (info.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return;
        }
      }
    }
  }
}

bool PU::isBipredRestriction(const PredictionUnit &pu)
{
  if(pu.cu->lumaSize().width == 4 && pu.cu->lumaSize().height ==4 )
  {
    return true;
  }
  /* disable bi-prediction for 4x8/8x4 */
  if ( pu.cu->lumaSize().width + pu.cu->lumaSize().height == 12 )
  {
    return true;
  }
  return false;
}

void PU::getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgType)
{
  int cuW = pu.Y().width;
  int cuH = pu.Y().height;
  int vx, vy;
  int shift = MAX_CU_DEPTH;
  int shiftHtoW = shift + floorLog2(cuW) - floorLog2(cuH);

  // motion info
  Mv cMv[2][4];
  int refIdx[2] = { -1, -1 };
  int dir = 0;
  EAffineModel curType = (verNum == 2) ? AFFINEMODEL_4PARAM : AFFINEMODEL_6PARAM;

  if ( verNum == 2 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1];
    if ( !isAvailable[idx0] || !isAvailable[idx1] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }

  }
  else if ( verNum == 3 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1], idx2 = verIdx[2];
    if ( !isAvailable[idx0] || !isAvailable[idx1] || !isAvailable[idx2] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 && mi[idx2].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l] && mi[idx0].refIdx[l] == mi[idx2].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }

  }

  if ( dir == 0 )
  {
    return;
  }

  for ( int l = 0; l < 2; l++ )
  {
    if ( dir & (l + 1) )
    {
      for ( int i = 0; i < verNum; i++ )
      {
        cMv[l][verIdx[i]] = mi[verIdx[i]].mv[l];
      }

      // convert to LT, RT[, [LB]]
      switch ( modelIdx )
      {
      case 0: // 0 : LT, RT, LB
        break;

      case 1: // 1 : LT, RT, RB
        cMv[l][2].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][1].hor;
        cMv[l][2].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][1].ver;
        cMv[l][2].clipToStorageBitDepth();
        break;

      case 2: // 2 : LT, LB, RB
        cMv[l][1].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][2].hor;
        cMv[l][1].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][2].ver;
        cMv[l][1].clipToStorageBitDepth();
        break;

      case 3: // 3 : RT, LB, RB
        cMv[l][0].hor = cMv[l][1].hor + cMv[l][2].hor - cMv[l][3].hor;
        cMv[l][0].ver = cMv[l][1].ver + cMv[l][2].ver - cMv[l][3].ver;
        cMv[l][0].clipToStorageBitDepth();
        break;

      case 4: // 4 : LT, RT
        break;

      case 5: // 5 : LT, LB
        vx = (cMv[l][0].hor << shift) + ((cMv[l][2].ver - cMv[l][0].ver) << shiftHtoW);
        vy = (cMv[l][0].ver << shift) - ((cMv[l][2].hor - cMv[l][0].hor) << shiftHtoW);
        roundAffineMv( vx, vy, shift );
        cMv[l][1].set( vx, vy );
        cMv[l][1].clipToStorageBitDepth();
        break;

      default: CHECK_( 1, "Invalid model index!\n" );
        break;
      }
    }
    else
    {
      for ( int i = 0; i < 4; i++ )
      {
        cMv[l][i].hor = 0;
        cMv[l][i].ver = 0;
      }
    }
  }

  for ( int i = 0; i < 3; i++ )
  {
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].mv = cMv[0][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].refIdx = refIdx[0];

    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].mv = cMv[1][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].refIdx = refIdx[1];
  }
  affMrgType.interDirNeighbours[affMrgType.numValidMergeCand] = dir;
  affMrgType.affineType[affMrgType.numValidMergeCand] = curType;
  affMrgType.BcwIdx[affMrgType.numValidMergeCand] = (dir == 3) ? bcwIdx : BCW_DEFAULT;
  affMrgType.numValidMergeCand++;

  return;
}

const int getAvailableAffineNeighboursForLeftPredictor( const PredictionUnit &pu, const PredictionUnit* npu[] )
{
  const Position posLB = pu.Y().bottomLeft();
  int num = 0;
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;

  const PredictionUnit *puLeftBottom = pu.cs->getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  if (puLeftBottom && puLeftBottom->cu->affine && puLeftBottom->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel))
  {
    npu[num++] = puLeftBottom;
    return num;
  }

  const PredictionUnit* puLeft = pu.cs->getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  if (puLeft && puLeft->cu->affine && puLeft->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel))
  {
    npu[num++] = puLeft;
    return num;
  }

  return num;
}

const int getAvailableAffineNeighboursForAbovePredictor( const PredictionUnit &pu, const PredictionUnit* npu[], int numAffNeighLeft )
{
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  int num = numAffNeighLeft;

  const PredictionUnit* puAboveRight = pu.cs->getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  if (puAboveRight && puAboveRight->cu->affine && puAboveRight->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel))
  {
    npu[num++] = puAboveRight;
    return num;
  }

  const PredictionUnit* puAbove = pu.cs->getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  if (puAbove && puAbove->cu->affine && puAbove->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel))
  {
    npu[num++] = puAbove;
    return num;
  }

  const PredictionUnit *puAboveLeft = pu.cs->getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  if (puAboveLeft && puAboveLeft->cu->affine && puAboveLeft->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel))
  {
    npu[num++] = puAboveLeft;
    return num;
  }

  return num;
}

void PU::getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx )
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;

  for ( int i = 0; i < maxNumAffineMergeCand; i++ )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField( Mv(), -1 );
      affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField( Mv(), -1 );
    }
    affMrgCtx.interDirNeighbours[i] = 0;
    affMrgCtx.affineType[i] = AFFINEMODEL_4PARAM;
    affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affMrgCtx.BcwIdx[i] = BCW_DEFAULT;
  }

  affMrgCtx.numValidMergeCand = 0;
  affMrgCtx.maxNumMergeCand = maxNumAffineMergeCand;

  bool sbTmvpEnableFlag = slice.getSPS()->getSbTMVPEnabledFlag()
                          && !(slice.getPOC() == slice.getRefPic(REF_PIC_LIST_0, 0)->getPOC() && slice.isIRAP());
  bool isAvailableSubPu = false;
  if (sbTmvpEnableFlag && slice.getPicHeader()->getEnableTMVPFlag())
  {
    MergeCtx mrgCtx = *affMrgCtx.mrgCtx;
    bool tmpLICFlag = false;

    CHECK_( mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized" );
    mrgCtx.subPuMvpMiBuf.fill( MotionInfo() );

    int pos = 0;
    // Get spatial MV
    const Position posCurLB = pu.Y().bottomLeft();
    MotionInfo miLeft;

    //left
    const PredictionUnit* puLeft = cs.getPURestricted( posCurLB.offset( -1, 0 ), pu, pu.chType );
    const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posCurLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );
    if ( isAvailableA1 )
    {
      miLeft = puLeft->getMotionInfo( posCurLB.offset( -1, 0 ) );
      // get Inter Dir
      mrgCtx.interDirNeighbours[pos] = miLeft.interDir;

      // get Mv from Left
      mrgCtx.mvFieldNeighbours[pos << 1].setMvField( miLeft.mv[0], miLeft.refIdx[0] );

      if ( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[(pos << 1) + 1].setMvField( miLeft.mv[1], miLeft.refIdx[1] );
      }
      pos++;
    }

    mrgCtx.numValidMergeCand = pos;

    isAvailableSubPu = getInterMergeSubPuMvpCand(pu, mrgCtx, tmpLICFlag, pos, 0);
    if ( isAvailableSubPu )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 0].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 0].refIdx );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 1].refIdx );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = mrgCtx.interDirNeighbours[pos];

      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINE_MODEL_NUM;
      affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_SUBPU_ATMVP;
      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      affMrgCtx.numValidMergeCand++;

      // early termination
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
  }

  if ( slice.getSPS()->getUseAffine() )
  {
    ///> Start: inherited affine candidates
    const PredictionUnit* npu[5];
    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor( pu, npu );
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor( pu, npu, numAffNeighLeft );
    for ( int idx = 0; idx < numAffNeigh; idx++ )
    {
      // derive Mv from Neigh affine PU
      Mv cMv[2][3];
      const PredictionUnit* puNeigh = npu[idx];
      pu.cu->affineType = puNeigh->cu->affineType;
      if ( puNeigh->interDir != 2 )
      {
        xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_0, cMv[0] );
      }
      if ( slice.isInterB() )
      {
        if ( puNeigh->interDir != 1 )
        {
          xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_1, cMv[1] );
        }
      }

      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( cMv[0][mvNum], puNeigh->refIdx[0] );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( cMv[1][mvNum], puNeigh->refIdx[1] );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir;
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = (EAffineModel)(puNeigh->cu->affineType);
      affMrgCtx.BcwIdx[affMrgCtx.numValidMergeCand] = puNeigh->cu->BcwIdx;

      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool isAvailable[4] = { false };

      int8_t neighBcw[2] = { BCW_DEFAULT, BCW_DEFAULT };
      // control point: LT B2->B3->A2
      const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
      for ( int i = 0; i < 3; i++ )
      {
        const Position pos = posLT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
        {
          isAvailable[0] = true;
          mi[0] = puNeigh->getMotionInfo( pos );
          neighBcw[0] = puNeigh->cu->BcwIdx;
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { pu.Y().topRight().offset( 0, -1 ), pu.Y().topRight().offset( 1, -1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posRT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
        {
          isAvailable[1] = true;
          mi[1] = puNeigh->getMotionInfo( pos );
          neighBcw[1] = puNeigh->cu->BcwIdx;
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { pu.Y().bottomLeft().offset( -1, 0 ), pu.Y().bottomLeft().offset( -1, 1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posLB[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
        {
          isAvailable[2] = true;
          mi[2] = puNeigh->getMotionInfo( pos );
          break;
        }
      }

      // control point: RB
      if ( slice.getPicHeader()->getEnableTMVPFlag() )
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = pu.Y().bottomRight().offset( -3, -3 );

        const PreCalcValues& pcv = *cs.pcv;
        Position posC0;
        bool C0Avail = false;

        bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
        const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
        if (curSubPic.getTreatedAsPicFlag())
        {
          boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
            (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + 4 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(4, 4);
            C0Avail = true;
          }
        }

        Mv        cColMv;
        int       refIdx = 0;
        bool      bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_0, posC0, cColMv, refIdx, false );
        if ( bExistMV )
        {
          mi[3].mv[0] = cColMv;
          mi[3].refIdx[0] = refIdx;
          mi[3].interDir = 1;
          isAvailable[3] = true;
        }

        if ( slice.isInterB() )
        {
          bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_1, posC0, cColMv, refIdx, false );
          if ( bExistMV )
          {
            mi[3].mv[1] = cColMv;
            mi[3].refIdx[1] = refIdx;
            mi[3].interDir |= 2;
            isAvailable[3] = true;
          }
        }
      }

      //-------------------  insert model  -------------------//
      int order[6] = { 0, 1, 2, 3, 4, 5 };
      int modelNum = 6;
      int model[6][4] = {
        { 0, 1, 2 },          // 0:  LT, RT, LB
        { 0, 1, 3 },          // 1:  LT, RT, RB
        { 0, 2, 3 },          // 2:  LT, LB, RB
        { 1, 2, 3 },          // 3:  RT, LB, RB
        { 0, 1 },             // 4:  LT, RT
        { 0, 2 },             // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx = pu.cs->sps->getUseAffineType() ? 0 : 4;
      for ( int idx = startIdx; idx < modelNum; idx++ )
      {
        int modelIdx = order[idx];
        getAffineControlPointCand(pu, mi, isAvailable, model[modelIdx], ((modelIdx == 3) ? neighBcw[1] : neighBcw[0]), modelIdx, verNum[modelIdx], affMrgCtx);
        if ( affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx )
        {
          return;
        }

        // early termination
        if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
        {
          return;
        }
      }
    }
    ///> End: Constructed affine candidates
  }

  ///> zero padding
  int cnt = affMrgCtx.numValidMergeCand;
  while ( cnt < maxNumAffineMergeCand )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField( Mv( 0, 0 ), 0 );
    }
    affMrgCtx.interDirNeighbours[cnt] = 1;

    if ( slice.isInterB() )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField( Mv( 0, 0 ), 0 );
      }
      affMrgCtx.interDirNeighbours[cnt] = 3;
    }
    affMrgCtx.affineType[cnt] = AFFINEMODEL_4PARAM;

    if ( cnt == mrgCandIdx )
    {
      return;
    }
    cnt++;
    affMrgCtx.numValidMergeCand++;
  }
}

void PU::setAllAffineMvField( PredictionUnit &pu, MvField *mvField, RefPicList eRefList )
{
  // Set Mv
  Mv mv[3];
  for ( int i = 0; i < 3; i++ )
  {
    mv[i] = mvField[i].mv;
  }
  setAllAffineMv( pu, mv[0], mv[1], mv[2], eRefList );

  // Set RefIdx
  CHECK_( mvField[0].refIdx != mvField[1].refIdx || mvField[0].refIdx != mvField[2].refIdx, "Affine mv corners don't have the same refIdx." );
  pu.refIdx[eRefList] = mvField[0].refIdx;
}

void PU::setAllAffineMv(PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs)
{
  int width  = pu.Y().width;
  int shift = MAX_CU_DEPTH;
  if (clipCPMVs)
  {
    affLT.mvCliptoStorageBitDepth();
    affRT.mvCliptoStorageBitDepth();
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      affLB.mvCliptoStorageBitDepth();
    }
  }
  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
  deltaMvHorX = (affRT - affLT).getHor() << (shift - floorLog2(width));
  deltaMvHorY = (affRT - affLT).getVer() << (shift - floorLog2(width));
  int height = pu.Y().height;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = (affLB - affLT).getHor() << (shift - floorLog2(height));
    deltaMvVerY = (affLB - affLT).getVer() << (shift - floorLog2(height));
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY = deltaMvHorX;
  }

  int mvScaleHor = affLT.getHor() << shift;
  int mvScaleVer = affLT.getVer() << shift;

  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
  const int halfBW = blockWidth >> 1;
  const int halfBH = blockHeight >> 1;

  MotionBuf mb = pu.getMotionBuf();
  int mvScaleTmpHor, mvScaleTmpVer;
  const bool subblkMVSpreadOverLimit = InterPrediction::isSubblockVectorSpreadOverLimit( deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, pu.interDir );
  for ( int h = 0; h < pu.Y().height; h += blockHeight )
  {
    for ( int w = 0; w < pu.Y().width; w += blockWidth )
    {
      if ( !subblkMVSpreadOverLimit )
      {
        mvScaleTmpHor = mvScaleHor + deltaMvHorX * (halfBW + w) + deltaMvVerX * (halfBH + h);
        mvScaleTmpVer = mvScaleVer + deltaMvHorY * (halfBW + w) + deltaMvVerY * (halfBH + h);
      }
      else
      {
        mvScaleTmpHor = mvScaleHor + deltaMvHorX * ( pu.Y().width >> 1 ) + deltaMvVerX * ( pu.Y().height >> 1 );
        mvScaleTmpVer = mvScaleVer + deltaMvHorY * ( pu.Y().width >> 1 ) + deltaMvVerY * ( pu.Y().height >> 1 );
      }
      roundAffineMv( mvScaleTmpHor, mvScaleTmpVer, shift );
      Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
      curMv.clipToStorageBitDepth();

      for ( int y = (h >> MIN_CU_LOG2); y < ((h + blockHeight) >> MIN_CU_LOG2); y++ )
      {
        for ( int x = (w >> MIN_CU_LOG2); x < ((w + blockWidth) >> MIN_CU_LOG2); x++ )
        {
          mb.at(x, y).mv[eRefList] = curMv;
        }
      }
    }
  }

  pu.mvAffi[eRefList][0] = affLT;
  pu.mvAffi[eRefList][1] = affRT;
  pu.mvAffi[eRefList][2] = affLB;
}

void clipColPos(int& posX, int& posY, const PredictionUnit& pu)
{
  Position puPos = pu.lumaPos();
  int log2CtuSize = floorLog2(pu.cs->sps->getCTUSize());
  int ctuX = ((puPos.x >> log2CtuSize) << log2CtuSize);
  int ctuY = ((puPos.y >> log2CtuSize) << log2CtuSize);
  int horMax;
  const SubPic &curSubPic = pu.cu->slice->getPPS()->getSubPicFromPos(puPos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = std::min((int)curSubPic.getSubPicRight(), ctuX + (int)pu.cs->sps->getCTUSize() + 3);
  }
  else
  {
    horMax = std::min((int)pu.cs->pps->getPicWidthInLumaSamples() - 1, ctuX + (int)pu.cs->sps->getCTUSize() + 3);
  }
  int horMin = std::max((int)0, ctuX);
  int verMax = std::min( (int)pu.cs->pps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.cs->sps->getCTUSize() - 1 );
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
}

bool PU::getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count
  , int mmvdList
)
{
  const Slice   &slice = *pu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);

  const Picture *pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
  Mv cTMv;

  if ( count )
  {
    if ( (mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_0)) && slice.getRefPic( REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].refIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].mv;
    }
    else if ( slice.isInterB() && (mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_1)) && slice.getRefPic( REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].refIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].mv;
    }
  }

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////

  Mv cTempVector = cTMv;
  bool  tempLICFlag = false;

  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();
  int numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;

  Mv cColMv;
  int refIdx = 0;
  // use coldir.
  bool     bBSlice = slice.isInterB();

  Position centerPos;

  bool found = false;
  cTempVector = cTMv;

  cTempVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
  int tempX = cTempVector.getHor();
  int tempY = cTempVector.getVer();

  centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
  centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

  clipColPos(centerPos.x, centerPos.y, pu);

  centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

  // derivation of center motion parameters from the collocated CU
  const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

  if (mi.isInter && mi.isIBCmot == false)
  {
    mrgCtx.interDirNeighbours[count] = 0;

    for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList  currRefPicList = RefPicList(currRefListId);

      if (getColocatedMVP(pu, currRefPicList, centerPos, cColMv, refIdx, true))
      {
        // set as default, for further motion vector field spanning
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, 0);
        mrgCtx.interDirNeighbours[count] |= (1 << currRefListId);
        LICFlag = tempLICFlag;
        mrgCtx.BcwIdx[count] = BCW_DEFAULT;
        found = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(Mv(), NOT_VALID);
        mrgCtx.interDirNeighbours[count] &= ~(1 << currRefListId);
      }
    }
  }

  if (!found)
  {
    return false;
  }
  if (mmvdList != 1)
  {
    int xOff = (puWidth >> 1) + tempX;
    int yOff = (puHeight >> 1) + tempY;

    MotionBuf &mb = mrgCtx.subPuMvpMiBuf;

    const bool isBiPred = isBipredRestriction(pu);

    for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
    {
      for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
      {
        Position colPos{ x + xOff, y + yOff };

        clipColPos(colPos.x, colPos.y, pu);

        colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };

        const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);

        MotionInfo mi;

        found       = false;
        mi.isInter  = true;
        mi.sliceIdx = slice.getIndependentSliceIdx();
        mi.isIBCmot = false;
        if (colMi.isInter && colMi.isIBCmot == false)
        {
          for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
          {
            RefPicList currRefPicList = RefPicList(currRefListId);
            if (getColocatedMVP(pu, currRefPicList, colPos, cColMv, refIdx, true))
            {
              mi.refIdx[currRefListId] = 0;
              mi.mv[currRefListId]     = cColMv;
              found                    = true;
            }
          }
        }
        if (!found)
        {
          mi.mv[0]     = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mv;
          mi.mv[1]     = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mv;
          mi.refIdx[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].refIdx;
          mi.refIdx[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].refIdx;
        }

        mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);

        if (isBiPred && mi.interDir == 3)
        {
          mi.interDir  = 1;
          mi.mv[1]     = Mv();
          mi.refIdx[1] = NOT_VALID;
        }

        mb.subBuf(g_miScaling.scale(Position{ x, y } - pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight)))
          .fill(mi);
      }
    }
  }
  return true;
}

void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
{
  MotionBuf mb = pu.getMotionBuf();

  if (!pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N || pu.mergeType == MRG_TYPE_IBC)
  {
    MotionInfo mi;

    mi.isInter = !CU::isIntra(*pu.cu);
    mi.isIBCmot = CU::isIBC(*pu.cu);
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;
      mi.useAltHpelIf = pu.cu->imv == IMV_HPEL;

      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = pu.bv;
      }
    }

    if( pu.cu->affine )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &dest = mb.at( x, y );
          dest.isInter  = mi.isInter;
          dest.isIBCmot = false;
          dest.interDir = mi.interDir;
          dest.sliceIdx = mi.sliceIdx;
          for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
          {
            if( mi.refIdx[i] == -1 )
            {
              dest.mv[i] = Mv();
            }
            dest.refIdx[i] = mi.refIdx[i];
          }
        }
      }
    }
    else
    {
      mb.fill( mi );
    }
  }
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    CHECK_(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
  }
  else
  {
    if( isBipredRestriction( pu ) )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &mi = mb.at( x, y );
          if( mi.interDir == 3 )
          {
            mi.interDir  = 1;
            mi.mv    [1] = Mv();
            mi.refIdx[1] = NOT_VALID;
          }
        }
      }
    }
  }
}

void PU::applyImv( PredictionUnit& pu, MergeCtx &mrgCtx, InterPrediction *interPred )
{
  if( !pu.mergeFlag )
  {
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      pu.mvd[0].changeTransPrecAmvr2Internal(pu.cu->imv);
      unsigned mvp_idx = pu.mvpIdx[0];
      AMVPInfo amvpInfo;
      if (CU::isIBC(*pu.cu))
      {
        PU::fillIBCMvpCand(pu, amvpInfo);
      }
      else
      PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo);
      pu.mvpNum[0] = amvpInfo.numCand;
      pu.mvpIdx[0] = mvp_idx;
      pu.mv    [0] = amvpInfo.mvCand[mvp_idx] + pu.mvd[0];
      pu.mv[0].mvCliptoStorageBitDepth();
    }

    if (pu.interDir != 1 /* PRED_L0 */)
    {
      if( !( pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3 ) && pu.cu->imv )/* PRED_BI */
      {
        pu.mvd[1].changeTransPrecAmvr2Internal(pu.cu->imv);
      }
      unsigned mvp_idx = pu.mvpIdx[1];
      AMVPInfo amvpInfo;
      PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo);
      pu.mvpNum[1] = amvpInfo.numCand;
      pu.mvpIdx[1] = mvp_idx;
      pu.mv    [1] = amvpInfo.mvCand[mvp_idx] + pu.mvd[1];
      pu.mv[1].mvCliptoStorageBitDepth();
    }
  }
  else
  {
    // this function is never called for merge
    THROW("unexpected");
    PU::getInterMergeCandidates(pu, mrgCtx, 0);

    mrgCtx.setMergeInfo( pu, pu.mergeIdx );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}


bool PU::isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu)
{
  if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
  {
    if (pu.cu->slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[0])->longTerm
      || pu.cu->slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[1])->longTerm)
    {
      return false;
    }
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, pu.refIdx[0]);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, pu.refIdx[1]);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}

void PU::restrictBiPredMergeCandsOne(PredictionUnit &pu)
{
  if (PU::isBipredRestriction(pu))
  {
    if (pu.interDir == 3)
    {
      pu.interDir = 1;
      pu.refIdx[1] = -1;
      pu.mv[1] = Mv(0, 0);
      pu.cu->BcwIdx = BCW_DEFAULT;
    }
  }
}

void PU::getGeoMergeCandidates( const PredictionUnit &pu, MergeCtx& geoMrgCtx )
{
  MergeCtx tmpMergeCtx;

  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
  geoMrgCtx.numValidMergeCand = 0;

  for (int32_t i = 0; i < GEO_MAX_NUM_UNI_CANDS; i++)
  {
    geoMrgCtx.BcwIdx[i] = BCW_DEFAULT;
    geoMrgCtx.interDirNeighbours[i] = 0;
    geoMrgCtx.mrgTypeNeighbours[i] = MRG_TYPE_DEFAULT_N;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].mv = Mv();
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].mv = Mv();
    geoMrgCtx.useAltHpelIf[i] = false;
  }

  PU::getInterMergeCandidates(pu, tmpMergeCtx, 0);

  for (int32_t i = 0; i < maxNumMergeCand; i++)
  {
    int parity = i & 1;
    if( tmpMergeCtx.interDirNeighbours[i] & (0x01 + parity) )
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1 + parity;
      geoMrgCtx.mrgTypeNeighbours[geoMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = -1;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].refIdx;
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
      continue;
    }

    if (tmpMergeCtx.interDirNeighbours[i] & (0x02 - parity))
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2 - parity;
      geoMrgCtx.mrgTypeNeighbours[geoMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].refIdx;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = -1;
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
    }
  }
}

void PU::spanGeoMotionInfo( PredictionUnit &pu, MergeCtx &geoMrgCtx, const uint8_t splitDir, const uint8_t candIdx0, const uint8_t candIdx1)
{
  pu.geoSplitDir  = splitDir;
  pu.geoMergeIdx0 = candIdx0;
  pu.geoMergeIdx1 = candIdx1;
  MotionBuf mb = pu.getMotionBuf();

  MotionInfo biMv;
  biMv.isInter  = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir = 1;
    biMv.mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
    biMv.mv[1] = Mv(0, 0);
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx;
    biMv.refIdx[1] = -1;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir = 2;
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = -1;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }

  int16_t angle = g_GeoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_GeoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;
  if (distanceIdx > 0)
  {
    if (angle % 16 == 8 || (angle % 16 != 0 && pu.lheight() >= pu.lwidth()))
    {
      offsetY += angle < 16 ? ((distanceIdx * pu.lheight()) >> 3) : -(int)((distanceIdx * pu.lheight()) >> 3);
    }
    else
    {
      offsetX += angle < 16 ? ((distanceIdx * pu.lwidth()) >> 3) : -(int)((distanceIdx * pu.lwidth()) >> 3);
    }
  }
  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_Dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_Dis[distanceX] + lookUpY;
      tpmMask = abs(motionIdx) < 32 ? 2 : (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 2)
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = biMv.interDir;
        mb.at(x, y).refIdx[0] = biMv.refIdx[0];
        mb.at(x, y).refIdx[1] = biMv.refIdx[1];
        mb.at(x, y).mv[0] = biMv.mv[0];
        mb.at(x, y).mv[1] = biMv.mv[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
      else if (tpmMask == 0)
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = geoMrgCtx.interDirNeighbours[candIdx0];
        mb.at(x, y).refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx;
        mb.at(x, y).refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
        mb.at(x, y).mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
      else
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = geoMrgCtx.interDirNeighbours[candIdx1];
        mb.at(x, y).refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx;
        mb.at(x, y).refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
        mb.at(x, y).mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
    }
  }
}

bool CU::hasSubCUNonZeroMVd( const CodingUnit& cu )
{
  bool bNonZeroMvd = false;

  for( const auto &pu : CU::traversePUs( cu ) )
  {
    if( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getHor() != 0;
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getVer() != 0;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        if( !pu.cu->cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getHor() != 0;
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getVer() != 0;
        }
      }
    }
  }

  return bNonZeroMvd;
}

bool CU::hasSubCUNonZeroAffineMVd( const CodingUnit& cu )
{
  bool nonZeroAffineMvd = false;

  if ( !cu.affine || cu.firstPU->mergeFlag )
  {
    return false;
  }

  for ( const auto &pu : CU::traversePUs( cu ) )
  {
    if ( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if ( pu.interDir != 2 /* PRED_L1 */ )
      {
        for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
        {
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getHor() != 0;
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getVer() != 0;
        }
      }

      if ( pu.interDir != 1 /* PRED_L0 */ )
      {
        if ( !pu.cu->cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
          {
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getHor() != 0;
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getVer() != 0;
          }
        }
      }
    }
  }

  return nonZeroAffineMvd;
}

uint8_t CU::getSbtInfo( uint8_t idx, uint8_t pos )
{
  return ( pos << 4 ) + ( idx << 0 );
}

uint8_t CU::getSbtIdx( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 0 ) & 0xf;
}

uint8_t CU::getSbtPos( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 4 ) & 0x3;
}

uint8_t CU::getSbtMode( uint8_t sbtIdx, uint8_t sbtPos )
{
  uint8_t sbtMode = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: sbtMode = sbtPos + SBT_VER_H0;  break;
  case SBT_HOR_HALF: sbtMode = sbtPos + SBT_HOR_H0;  break;
  case SBT_VER_QUAD: sbtMode = sbtPos + SBT_VER_Q0;  break;
  case SBT_HOR_QUAD: sbtMode = sbtPos + SBT_HOR_Q0;  break;
  default:           assert( 0 );
  }

  assert( sbtMode < NUMBER_SBT_MODE );
  return sbtMode;
}

uint8_t CU::getSbtIdxFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
  {
    return SBT_VER_HALF;
  }
  else if( sbtMode <= SBT_HOR_H1 )
  {
    return SBT_HOR_HALF;
  }
  else if( sbtMode <= SBT_VER_Q1 )
  {
    return SBT_VER_QUAD;
  }
  else if( sbtMode <= SBT_HOR_Q1 )
  {
    return SBT_HOR_QUAD;
  }
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::getSbtPosFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
  {
    return sbtMode - SBT_VER_H0;
  }
  else if( sbtMode <= SBT_HOR_H1 )
  {
    return sbtMode - SBT_HOR_H0;
  }
  else if( sbtMode <= SBT_VER_Q1 )
  {
    return sbtMode - SBT_VER_Q0;
  }
  else if( sbtMode <= SBT_HOR_Q1 )
  {
    return sbtMode - SBT_HOR_Q0;
  }
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::targetSbtAllowed( uint8_t sbtIdx, uint8_t sbtAllowed )
{
  uint8_t val = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: val = ( ( sbtAllowed >> SBT_VER_HALF ) & 0x1 ); break;
  case SBT_HOR_HALF: val = ( ( sbtAllowed >> SBT_HOR_HALF ) & 0x1 ); break;
  case SBT_VER_QUAD: val = ( ( sbtAllowed >> SBT_VER_QUAD ) & 0x1 ); break;
  case SBT_HOR_QUAD: val = ( ( sbtAllowed >> SBT_HOR_QUAD ) & 0x1 ); break;
  default: CHECK_( 1, "unknown SBT type" );
  }
  return val;
}

uint8_t CU::numSbtModeRdo( uint8_t sbtAllowed )
{
  uint8_t num = 0;
  uint8_t sum = 0;
  num = targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) + targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  num = targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  return sum;
}

bool CU::isSbtMode( const uint8_t sbtInfo )
{
  uint8_t sbtIdx = getSbtIdx( sbtInfo );
  return sbtIdx >= SBT_VER_HALF && sbtIdx <= SBT_HOR_QUAD;
}

bool CU::isSameSbtSize( const uint8_t sbtInfo1, const uint8_t sbtInfo2 )
{
  uint8_t sbtIdx1 = getSbtIdxFromSbtMode( sbtInfo1 );
  uint8_t sbtIdx2 = getSbtIdxFromSbtMode( sbtInfo2 );
  if( sbtIdx1 == SBT_HOR_HALF || sbtIdx1 == SBT_VER_HALF )
  {
    return sbtIdx2 == SBT_HOR_HALF || sbtIdx2 == SBT_VER_HALF;
  }
  else if( sbtIdx1 == SBT_HOR_QUAD || sbtIdx1 == SBT_VER_QUAD )
  {
    return sbtIdx2 == SBT_HOR_QUAD || sbtIdx2 == SBT_VER_QUAD;
  }
  else
  {
    return false;
  }
}

bool CU::isPredRegDiffFromTB(const CodingUnit &cu, const ComponentID compID)
{
  return (compID == COMPONENT_Y)
         && (cu.ispMode == VER_INTRA_SUBPARTITIONS
             && CU::isMinWidthPredEnabledForBlkSize(cu.blocks[compID].width, cu.blocks[compID].height));
}

bool CU::isMinWidthPredEnabledForBlkSize(const int w, const int h)
{
  return ((w == 8 && h > 4) || w == 4);
}

bool CU::isFirstTBInPredReg(const CodingUnit& cu, const ComponentID compID, const CompArea &area)
{
  return (compID == COMPONENT_Y) && cu.ispMode && ((area.topLeft().x - cu.Y().topLeft().x) % PRED_REG_MIN_WIDTH == 0);
}

void CU::adjustPredArea(CompArea &area)
{
  area.width = std::max<int>(PRED_REG_MIN_WIDTH, area.width);
}

bool CU::isBcwIdxCoded( const CodingUnit &cu )
{
  if( cu.cs->sps->getUseBcw() == false )
  {
    CHECK_(cu.BcwIdx != BCW_DEFAULT, "Error: cu.BcwIdx != BCW_DEFAULT");
    return false;
  }

  if (cu.predMode == MODE_IBC)
  {
    return false;
  }

  if( cu.predMode == MODE_INTRA || cu.cs->slice->isInterP() )
  {
    return false;
  }

  if( cu.lwidth() * cu.lheight() < BCW_SIZE_CONSTRAINT )
  {
    return false;
  }

  if( !cu.firstPU->mergeFlag )
  {
    if( cu.firstPU->interDir == 3 )
    {
      const int refIdx0 = cu.firstPU->refIdx[REF_PIC_LIST_0];
      const int refIdx1 = cu.firstPU->refIdx[REF_PIC_LIST_1];

      const WPScalingParam *wp0 = cu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
      const WPScalingParam *wp1 = cu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

      return !(WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1));
    }
  }

  return false;
}

uint8_t CU::getValidBcwIdx( const CodingUnit &cu )
{
  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    return cu.BcwIdx;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    CHECK_(cu.BcwIdx != BCW_DEFAULT, " cu.BcwIdx != BCW_DEFAULT ");
  }
  else
  {
    CHECK_(cu.BcwIdx != BCW_DEFAULT, " cu.BcwIdx != BCW_DEFAULT ");
  }

  return BCW_DEFAULT;
}

void CU::setBcwIdx( CodingUnit &cu, uint8_t uh )
{
  int8_t uhCnt = 0;

  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    cu.BcwIdx = uh;
    ++uhCnt;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    cu.BcwIdx = BCW_DEFAULT;
  }
  else
  {
    cu.BcwIdx = BCW_DEFAULT;
  }

  CHECK_(uhCnt <= 0, " uhCnt <= 0 ");
}

uint8_t CU::deriveBcwIdx( uint8_t bcwLO, uint8_t bcwL1 )
{
  if( bcwLO == bcwL1 )
  {
    return bcwLO;
  }
  const int8_t w0 = getBcwWeight(bcwLO, REF_PIC_LIST_0);
  const int8_t w1 = getBcwWeight(bcwL1, REF_PIC_LIST_1);
  const int8_t th = g_BcwWeightBase >> 1;
  const int8_t off = 1;

  if( w0 == w1 || (w0 < (th - off) && w1 < (th - off)) || (w0 >(th + off) && w1 >(th + off)) )
  {
    return BCW_DEFAULT;
  }
  else
  {
    if( w0 > w1 )
    {
      return ( w0 >= th ? bcwLO : bcwL1 );
    }
    else
    {
      return ( w1 >= th ? bcwL1 : bcwLO );
    }
  }
}

bool CU::bdpcmAllowed( const CodingUnit& cu, const ComponentID compID )
{
  SizeType transformSkipMaxSize = 1 << cu.cs->sps->getLog2MaxTransformSkipBlockSize();

  bool bdpcmAllowed = cu.cs->sps->getBDPCMEnabledFlag();
       bdpcmAllowed &= CU::isIntra( cu );
       if (isLuma(compID))
       {
         bdpcmAllowed &= (cu.lwidth() <= transformSkipMaxSize && cu.lheight() <= transformSkipMaxSize);
       }
       else
       {
         bdpcmAllowed &=
           (cu.chromaSize().width <= transformSkipMaxSize && cu.chromaSize().height <= transformSkipMaxSize)
           && !cu.colorTransform;
       }
  return bdpcmAllowed;
}

bool CU::isMTSAllowed(const CodingUnit &cu, const ComponentID compID)
{
  SizeType tsMaxSize = 1 << cu.cs->sps->getLog2MaxTransformSkipBlockSize();
  const int maxSize  = CU::isIntra( cu ) ? MTS_INTRA_MAX_CU_SIZE : MTS_INTER_MAX_CU_SIZE;
  const int cuWidth  = cu.blocks[0].lumaSize().width;
  const int cuHeight = cu.blocks[0].lumaSize().height;
  bool mtsAllowed    = cu.chType == CHANNEL_TYPE_LUMA && compID == COMPONENT_Y;

  mtsAllowed &= CU::isIntra( cu ) ? cu.cs->sps->getUseIntraMTS() : cu.cs->sps->getUseInterMTS() && CU::isInter( cu );
  mtsAllowed &= cuWidth <= maxSize && cuHeight <= maxSize;
  mtsAllowed &= !cu.ispMode;
  mtsAllowed &= !cu.sbtInfo;
  mtsAllowed &= !(cu.bdpcmMode && cuWidth <= tsMaxSize && cuHeight <= tsMaxSize);
  return mtsAllowed;
}

// TU tools

bool TU::isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID)
{
  return tu.cs->sps->getSpsRangeExtension().getTransformSkipRotationEnabledFlag() && tu.blocks[compID].width == 4 && tu.cu->predMode == MODE_INTRA;
}

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
  return getCbfAtDepth( tu, compID, tu.depth );
}

bool TU::getCbfAtDepth(const TransformUnit &tu, const ComponentID &compID, const unsigned &depth)
{
  if( !tu.blocks[compID].valid() )
  {
    CHECK_(tu.cbf[compID] != 0, "cbf must be 0 if the component is not available");
  }
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1  << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}

bool TU::isTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  const int maxSize = tu.cs->sps->getLog2MaxTransformSkipBlockSize();

  bool tsAllowed = tu.cs->sps->getTransformSkipEnabledFlag();
  tsAllowed &= ( !tu.cu->ispMode || !isLuma(compID) );
  SizeType transformSkipMaxSize = 1 << maxSize;
  tsAllowed &= !(tu.cu->bdpcmMode && isLuma(compID));
  tsAllowed &= !(tu.cu->bdpcmModeChroma && isChroma(compID));
  tsAllowed &= tu.blocks[compID].width <= transformSkipMaxSize && tu.blocks[compID].height <= transformSkipMaxSize;
  tsAllowed &= !tu.cu->sbtInfo;

  return tsAllowed;
}


int TU::getICTMode( const TransformUnit& tu, int jointCbCr )
{
  if( jointCbCr < 0 )
  {
    jointCbCr = tu.jointCbCr;
  }
  return g_ictModes[ tu.cs->picHeader->getJointCbCrSignFlag() ][ jointCbCr ];
}


bool TU::needsSqrt2Scale( const TransformUnit &tu, const ComponentID &compID )
{
  const Size &size=tu.blocks[compID];
  const bool isTransformSkip = (tu.mtsIdx[compID] == MTS_SKIP);
  return (!isTransformSkip) && (((floorLog2(size.width) + floorLog2(size.height)) & 1) == 1);
}

bool TU::needsBlockSizeTrafoScale( const TransformUnit &tu, const ComponentID &compID )
{
  return needsSqrt2Scale( tu, compID ) || isNonLog2BlockSize( tu.blocks[compID] );
}

TransformUnit* TU::getPrevTU( const TransformUnit &tu, const ComponentID compID )
{
  TransformUnit* prevTU = tu.prev;

  if( prevTU != nullptr && ( prevTU->cu != tu.cu || !prevTU->blocks[compID].valid() ) )
  {
    prevTU = nullptr;
  }

  return prevTU;
}

bool TU::getPrevTuCbfAtDepth( const TransformUnit &currentTu, const ComponentID compID, const int trDepth )
{
  const TransformUnit* prevTU = getPrevTU( currentTu, compID );
  return ( prevTU != nullptr ) ? TU::getCbfAtDepth( *prevTU, compID, trDepth ) : false;
}


// other tools

uint32_t getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}

int getNumModesMip(const Size& block)
{
  switch( getMipSizeId(block) )
  {
  case 0: return 16;
  case 1: return  8;
  case 2: return  6;
  default: THROW( "Invalid mipSizeId" );
  }
}


int getMipSizeId(const Size& block)
{
  if( block.width == 4 && block.height == 4 )
  {
    return 0;
  }
  else if( block.width == 4 || block.height == 4 || (block.width == 8 && block.height == 8) )
  {
    return 1;
  }
  else
  {
    return 2;
  }

}

bool allowLfnstWithMip(const Size& block)
{
  if (block.width >= 16 && block.height >= 16)
  {
    return true;
  }
  return false;
}




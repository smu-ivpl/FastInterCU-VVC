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

/** \file     WeightPrediction.h
    \brief    weighting prediction class (header)
*/

// Include files
#include "CommonDef.h"
#include "Unit.h"
#include "InterpolationFilter.h"
#include "WeightPrediction.h"
#include "CodingStructure.h"


static inline Pel weightBidir( int w0, Pel P0, int w1, Pel P1, int round, int shift, int offset, const ClpRng& clpRng)
{
  return ClipPel( ( (w0*(P0 + IF_INTERNAL_OFFS) + w1*(P1 + IF_INTERNAL_OFFS) + round + (offset << (shift-1))) >> shift ), clpRng );
}

static inline Pel weightUnidir( int w0, Pel P0, int round, int shift, int offset, const ClpRng& clpRng)
{
  return ClipPel( ( (w0*(P0 + IF_INTERNAL_OFFS) + round) >> shift ) + offset, clpRng );
}

static inline Pel noWeightUnidir( Pel P0, int round, int shift, int offset, const ClpRng& clpRng)
{
  return ClipPel( ( ((P0 + IF_INTERNAL_OFFS) + round) >> shift ) + offset, clpRng );
}

static inline Pel noWeightOffsetUnidir( Pel P0, int round, int shift, const ClpRng& clpRng)
{
  return ClipPel( ( ((P0 + IF_INTERNAL_OFFS) + round) >> shift ), clpRng );
}


// ====================================================================================================================
// Class definition
// ====================================================================================================================

WeightPrediction::WeightPrediction()
{
}

void WeightPrediction::getWpScaling(Slice *pcSlice, const int &iRefIdx0, const int &iRefIdx1, WPScalingParam *&wp0,
                                    WPScalingParam *&wp1, const ComponentID maxNumComp)
{
  CHECK_(iRefIdx0 < 0 && iRefIdx1 < 0, "Both picture reference list indizes smaller than '0'");

  const bool wpBiPred        = pcSlice->getPPS()->getWPBiPred();
  const bool bBiPred         = (iRefIdx0 >= 0 && iRefIdx1 >= 0);
  const bool bUniPred        = !bBiPred;

  if (bUniPred || wpBiPred)
  {
    // explicit --------------------
    wp0 = pcSlice->getWpScaling(REF_PIC_LIST_0, iRefIdx0);
    wp1 = pcSlice->getWpScaling(REF_PIC_LIST_1, iRefIdx1);
  }
  else
  {
    THROW( "Unsupported WP configuration" );
  }

  if (iRefIdx0 < 0)
  {
    wp0 = nullptr;
  }
  if (iRefIdx1 < 0)
  {
    wp1 = nullptr;
  }

  const uint32_t numValidComponent = getNumberValidComponents(pcSlice->getSPS()->getChromaFormatIdc());
  const bool bUseHighPrecisionPredictionWeighting = pcSlice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();

  if (bBiPred)
  {
    // Bi-predictive case
    for (int yuv = 0; yuv < numValidComponent && yuv <= maxNumComp; yuv++)
    {
      const int bitDepth = pcSlice->getSPS()->getBitDepth(toChannelType(ComponentID(yuv)));
      const int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (bitDepth - 8));

      wp0[yuv].w      = wp0[yuv].codedWeight;
      wp1[yuv].w      = wp1[yuv].codedWeight;
      wp0[yuv].o      = wp0[yuv].codedOffset * offsetScalingFactor;
      wp1[yuv].o      = wp1[yuv].codedOffset * offsetScalingFactor;
      wp0[yuv].offset = wp0[yuv].o + wp1[yuv].o;
      wp0[yuv].shift  = wp0[yuv].log2WeightDenom + 1;
      wp0[yuv].round  = (1 << wp0[yuv].log2WeightDenom);
      wp1[yuv].offset = wp0[yuv].offset;
      wp1[yuv].shift = wp0[yuv].shift;
      wp1[yuv].round = wp0[yuv].round;
    }
  }
  else
  {
    // UniPred
    WPScalingParam *const pwp = (iRefIdx0 >= 0) ? wp0 : wp1;

    for (int yuv = 0; yuv < numValidComponent && yuv <= maxNumComp; yuv++)
    {
      const int bitDepth            = pcSlice->getSPS()->getBitDepth(toChannelType(ComponentID(yuv)));
      const int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (bitDepth - 8));

      pwp[yuv].w      = pwp[yuv].codedWeight;
      pwp[yuv].offset = pwp[yuv].codedOffset * offsetScalingFactor;
      pwp[yuv].shift  = pwp[yuv].log2WeightDenom;
      pwp[yuv].round  = (pwp[yuv].log2WeightDenom >= 1) ? (1 << (pwp[yuv].log2WeightDenom - 1)) : (0);
    }
  }
}

void WeightPrediction::addWeightBi(const CPelUnitBuf          &pcYuvSrc0,
                                   const CPelUnitBuf          &pcYuvSrc1,
                                   const ClpRngs              &clpRngs,
                                   const WPScalingParam *const wp0,
                                   const WPScalingParam *const wp1,
                                         PelUnitBuf           &rpcYuvDst,
                                   const bool                  bRoundLuma /*= true*/,
                                   const ComponentID           maxNumComp
                                  , bool                       lumaOnly
                                  , bool                       chromaOnly
)
{
  const bool enableRounding[MAX_NUM_COMPONENT] = { bRoundLuma, true, true };

  const uint32_t numValidComponent = (const uint32_t)pcYuvSrc0.bufs.size();

  CHECK_( lumaOnly && chromaOnly, "Not allowed to have both lumaOnly and chromaOnly selected" );
  int firstComponent = chromaOnly ? 1 : 0;
  int lastComponent = lumaOnly ? 0 : maxNumComp;
  for (int componentIndex = firstComponent; componentIndex < numValidComponent && componentIndex <= lastComponent; componentIndex++)
  {
    const ComponentID compID = ComponentID(componentIndex);

    const Pel* pSrc0 = pcYuvSrc0.bufs[compID].buf;
    const Pel* pSrc1 = pcYuvSrc1.bufs[compID].buf;
          Pel* pDst  = rpcYuvDst.bufs[compID].buf;

    // Luma : --------------------------------------------
    const ClpRng& clpRng = clpRngs.comp[compID];
    const int  w0       = wp0[compID].w;
    const int  offset   = wp0[compID].offset;
    const int  clipBD   = clpRng.bd;
    const int shiftNum = IF_INTERNAL_FRAC_BITS(clipBD);
    const int  shift    = wp0[compID].shift + shiftNum;
    const int  round    = (enableRounding[compID] && (shift > 0)) ? (1 << (shift - 1)) : 0;
    const int  w1       = wp1[compID].w;
    const int  iHeight  = rpcYuvDst.bufs[compID].height;
    const int  iWidth   = rpcYuvDst.bufs[compID].width;

    const uint32_t iSrc0Stride = pcYuvSrc0.bufs[compID].stride;
    const uint32_t iSrc1Stride = pcYuvSrc1.bufs[compID].stride;
    const uint32_t iDstStride =  rpcYuvDst.bufs[compID].stride;

    for (int y = iHeight - 1; y >= 0; y--)
    {
      // do it in batches of 4 (partial unroll)
      int x = iWidth - 1;

      for (; x >= 3; )
      {
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng ); x--;
      }
      for (; x >= 0; x--)
      {
        pDst[x] = weightBidir(w0, pSrc0[x], w1, pSrc1[x], round, shift, offset, clpRng );
      }

      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst += iDstStride;
    } // y loop
  } // compID loop
}

void WeightPrediction::addWeightBiComponent(const CPelUnitBuf          &pcYuvSrc0,
                                            const CPelUnitBuf          &pcYuvSrc1,
                                            const ClpRngs              &clpRngs,
                                            const WPScalingParam *const wp0,
                                            const WPScalingParam *const wp1,
                                                  PelUnitBuf           &rpcYuvDst,
                                            const bool                  bRoundLuma /*= true*/,
                                            const ComponentID           Comp)
{
  const bool enableRounding[MAX_NUM_COMPONENT] = { bRoundLuma, true, true };

  const ComponentID compID = ComponentID(Comp);

  const Pel* src0 = pcYuvSrc0.bufs[compID].buf;
  const Pel* src1 = pcYuvSrc1.bufs[compID].buf;
        Pel* dst  = rpcYuvDst.bufs[compID].buf;

  // Luma : --------------------------------------------
  const ClpRng& clpRng = clpRngs.comp[compID];
  const int  w0       = wp0[compID].w;
  const int  offset   = wp0[compID].offset;
  const int  clipBD   = clpRng.bd;
  const int shiftNum = IF_INTERNAL_FRAC_BITS(clipBD);
  const int  shift    = wp0[compID].shift + shiftNum;
  const int  round    = (enableRounding[compID] && (shift > 0)) ? (1 << (shift - 1)) : 0;
  const int  w1       = wp1[compID].w;
  const int  height  = rpcYuvDst.bufs[compID].height;
  const int  width   = rpcYuvDst.bufs[compID].width;

  const uint32_t src0Stride = pcYuvSrc0.bufs[compID].stride;
  const uint32_t src1Stride = pcYuvSrc1.bufs[compID].stride;
  const uint32_t dstStride =  rpcYuvDst.bufs[compID].stride;

  for (int y = height - 1; y >= 0; y--)
  {
    // do it in batches of 4 (partial unroll)
    int x = width - 1;

    for (; x >= 3; )
    {
      dst[x] = weightBidir(w0, src0[x], w1, src1[x], round, shift, offset, clpRng ); x--;
      dst[x] = weightBidir(w0, src0[x], w1, src1[x], round, shift, offset, clpRng ); x--;
      dst[x] = weightBidir(w0, src0[x], w1, src1[x], round, shift, offset, clpRng ); x--;
      dst[x] = weightBidir(w0, src0[x], w1, src1[x], round, shift, offset, clpRng ); x--;
    }
    for (; x >= 0; x--)
    {
      dst[x] = weightBidir(w0, src0[x], w1, src1[x], round, shift, offset, clpRng );
    }

    src0 += src0Stride;
    src1 += src1Stride;
    dst += dstStride;
  } // y loop
}

void  WeightPrediction::addWeightUni(const CPelUnitBuf          &pcYuvSrc0,
                                     const ClpRngs              &clpRngs,
                                     const WPScalingParam *const wp0,
                                           PelUnitBuf           &rpcYuvDst,
                                     const ComponentID           maxNumComp
                                    , bool                       lumaOnly
                                    , bool                       chromaOnly
)
{
  const uint32_t numValidComponent = (const uint32_t)pcYuvSrc0.bufs.size();

  CHECK_( lumaOnly && chromaOnly, "Not allowed to have both lumaOnly and chromaOnly selected" );
  int firstComponent = chromaOnly ? 1 : 0;
  int lastComponent  = lumaOnly ? 0 : maxNumComp;
  for (int componentIndex = firstComponent; componentIndex < numValidComponent && componentIndex <= lastComponent;
       componentIndex++)
  {
    const ComponentID compID = ComponentID(componentIndex);

    const Pel* pSrc0 = pcYuvSrc0.bufs[compID].buf;
          Pel* pDst  = rpcYuvDst.bufs[compID].buf;

    // Luma : --------------------------------------------
    const ClpRng& clpRng    = clpRngs.comp[compID];
    const int  w0           = wp0[compID].w;
    const int  offset       = wp0[compID].offset;
    const int  clipBD       = clpRng.bd;
    const int shiftNum      = IF_INTERNAL_FRAC_BITS(clipBD);
    const int  shift        = wp0[compID].shift + shiftNum;
    const uint32_t iSrc0Stride  = pcYuvSrc0.bufs[compID].stride;
    const uint32_t iDstStride   = rpcYuvDst.bufs[compID].stride;
    const int  iHeight      = rpcYuvDst.bufs[compID].height;
    const int  iWidth       = rpcYuvDst.bufs[compID].width;

    if (w0 != 1 << wp0[compID].shift)
    {
      const int  round = (shift > 0) ? (1 << (shift - 1)) : 0;
      for (int y = iHeight - 1; y >= 0; y--)
      {
        int x = iWidth - 1;
        for (; x >= 3; )
        {
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
        }
        for (; x >= 0; x--)
        {
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng);
        }
        pSrc0 += iSrc0Stride;
        pDst += iDstStride;
      }
    }
    else
    {
      const int  round = (shiftNum > 0) ? (1 << (shiftNum - 1)) : 0;
      if (offset == 0)
      {
        for (int y = iHeight - 1; y >= 0; y--)
        {
          int x = iWidth - 1;
          for (; x >= 3; )
          {
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
          }
          for (; x >= 0; x--)
          {
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng);
          }
          pSrc0 += iSrc0Stride;
          pDst += iDstStride;
        }
      }
      else
      {
        for (int y = iHeight - 1; y >= 0; y--)
        {
          int x = iWidth - 1;
          for (; x >= 3; )
          {
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
          }
          for (; x >= 0; x--)
          {
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng);
          }
          pSrc0 += iSrc0Stride;
          pDst += iDstStride;
        }
      }
    }
  }
}

void  WeightPrediction::xWeightedPredictionUni(const PredictionUnit       &pu,
                                               const CPelUnitBuf          &pcYuvSrc,
                                               const RefPicList           &eRefPicList,
                                                     PelUnitBuf           &pcYuvPred,
                                               const int                   iRefIdx_input/* = -1*/,
                                               const ComponentID           maxNumComp
                                              , bool                       lumaOnly
                                              , bool                       chromaOnly
)
{
  WPScalingParam  *pwp, *pwpTmp;

  int iRefIdx = iRefIdx_input;
  if (iRefIdx < 0)
  {
    iRefIdx = pu.refIdx[eRefPicList];
  }

  CHECK_(iRefIdx < 0, "Negative reference picture list index");

  if (eRefPicList == REF_PIC_LIST_0)
  {
    getWpScaling(pu.cs->slice, iRefIdx, -1, pwp, pwpTmp, maxNumComp);
  }
  else
  {
    getWpScaling(pu.cs->slice, -1, iRefIdx, pwpTmp, pwp, maxNumComp);
  }
  addWeightUni(pcYuvSrc, pu.cu->slice->clpRngs(), pwp, pcYuvPred, maxNumComp, lumaOnly, chromaOnly);
}

void  WeightPrediction::xWeightedPredictionBi(const PredictionUnit       &pu,
                                              const CPelUnitBuf          &pcYuvSrc0,
                                              const CPelUnitBuf          &pcYuvSrc1,
                                                    PelUnitBuf           &rpcYuvDst,
                                              const ComponentID           maxNumComp
                                              , bool                      lumaOnly
                                              , bool                      chromaOnly
)
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];
  WPScalingParam  *pwp0;
  WPScalingParam  *pwp1;

  CHECK_( !pu.cs->pps->getWPBiPred(), "Weighted Bi-prediction disabled" );

  if (iRefIdx0 < 0 && iRefIdx1 < 0) return;

  getWpScaling(pu.cu->slice, iRefIdx0, iRefIdx1, pwp0, pwp1, maxNumComp);

  if (iRefIdx0 >= 0 && iRefIdx1 >= 0)
  {
    addWeightBi(pcYuvSrc0, pcYuvSrc1, pu.cu->slice->clpRngs(), pwp0, pwp1, rpcYuvDst, true, maxNumComp, lumaOnly, chromaOnly);
  }
  else if (iRefIdx0 >= 0 && iRefIdx1 < 0)
  {
    addWeightUni(pcYuvSrc0, pu.cu->slice->clpRngs(), pwp0, rpcYuvDst, maxNumComp, lumaOnly, chromaOnly);
  }
  else if (iRefIdx0 < 0 && iRefIdx1 >= 0)
  {
    addWeightUni(pcYuvSrc1, pu.cu->slice->clpRngs(), pwp1, rpcYuvDst, maxNumComp, lumaOnly, chromaOnly);
  }
  else
  {
    THROW( "Both reference picture list indizes are negative" );
  }
}

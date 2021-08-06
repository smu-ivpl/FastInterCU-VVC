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

/** \file     RdCost.cpp
    \brief    RD cost computation class
*/

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

#include "RdCost.h"

#include "Rom.h"
#include "UnitPartitioner.h"

#include <limits>

//! \ingroup CommonLib
//! \{


FpDistFunc RdCost::m_afpDistortFunc[DF_TOTAL_FUNCTIONS] = { nullptr, };

RdCost::RdCost()
{
  init();
}

RdCost::~RdCost()
{
}

#if WCG_EXT
double RdCost::calcRdCost( uint64_t fracBits, Distortion distortion, bool useUnadjustedLambda )
#else
double RdCost::calcRdCost( uint64_t fracBits, Distortion distortion )
#endif
{
  if (m_costMode == COST_LOSSLESS_CODING && 0 != distortion && m_isLosslessRDCost)
  {
    return MAX_DOUBLE;
  }
#if WCG_EXT
  return ( useUnadjustedLambda ? m_DistScaleUnadjusted : m_DistScale ) * double( distortion ) + double( fracBits );
#else
  return m_DistScale * double( distortion ) + double( fracBits );
#endif
}

void RdCost::setLambda( double dLambda, const BitDepths &bitDepths )
{
  m_dLambda             = dLambda;
  m_DistScale           = double(1<<SCALE_BITS) / m_dLambda;
  m_dLambdaMotionSAD    = sqrt(m_dLambda);
}

void RdCost::lambdaAdjustColorTrans(bool forward, ComponentID componentID, bool applyChromaScale, int* resScaleInv)
{
  if (m_resetStore)
  {
    for (uint8_t component = 0; component < MAX_NUM_COMPONENT; component++)
    {
      ComponentID compID = (ComponentID)component;
      int       delta_QP = DELTA_QP_ACT[compID];
      double lamdbaAdjustRate = pow(2.0, delta_QP / 3.0);

      m_lambdaStore[0][component] = m_dLambda;
      m_DistScaleStore[0][component] = m_DistScale;

      m_lambdaStore[1][component] = m_dLambda * lamdbaAdjustRate;
      m_DistScaleStore[1][component] = double(1 << SCALE_BITS) / m_lambdaStore[1][component];
    }
    m_resetStore = false;
  }

  if (forward)
  {
    CHECK_(m_pairCheck == 1, "lambda has been already adjusted");
    m_pairCheck = 1;
  }
  else
  {
    CHECK_(m_pairCheck == 0, "lambda has not been adjusted");
    m_pairCheck = 0;
  }

  m_dLambda = m_lambdaStore[m_pairCheck][componentID];
  m_DistScale = m_DistScaleStore[m_pairCheck][componentID];
  if (applyChromaScale)
  {
    CHECK_(m_pairCheck == 0 || componentID == COMPONENT_Y, "wrong lambda adjustment for CS");
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)(*resScaleInv);
    m_dLambda = m_dLambda / (cResScale*cResScale);
    m_DistScale = double(1 << SCALE_BITS) / m_dLambda;
  }
  if (m_pairCheck == 0)
  {
    CHECK_(m_DistScale != m_DistScaleUnadjusted, "lambda should be adjusted to the original value");
  }
}

// Initialize Function Pointer by [eDFunc]
void RdCost::init()
{
  m_afpDistortFunc[DF_SSE    ] = RdCost::xGetSSE;
  m_afpDistortFunc[DF_SSE2   ] = RdCost::xGetSSE;
  m_afpDistortFunc[DF_SSE4   ] = RdCost::xGetSSE4;
  m_afpDistortFunc[DF_SSE8   ] = RdCost::xGetSSE8;
  m_afpDistortFunc[DF_SSE16  ] = RdCost::xGetSSE16;
  m_afpDistortFunc[DF_SSE32  ] = RdCost::xGetSSE32;
  m_afpDistortFunc[DF_SSE64  ] = RdCost::xGetSSE64;
  m_afpDistortFunc[DF_SSE16N ] = RdCost::xGetSSE16N;

  m_afpDistortFunc[DF_SAD    ] = RdCost::xGetSAD;
  m_afpDistortFunc[DF_SAD2   ] = RdCost::xGetSAD;
  m_afpDistortFunc[DF_SAD4   ] = RdCost::xGetSAD4;
  m_afpDistortFunc[DF_SAD8   ] = RdCost::xGetSAD8;
  m_afpDistortFunc[DF_SAD16  ] = RdCost::xGetSAD16;
  m_afpDistortFunc[DF_SAD32  ] = RdCost::xGetSAD32;
  m_afpDistortFunc[DF_SAD64  ] = RdCost::xGetSAD64;
  m_afpDistortFunc[DF_SAD16N ] = RdCost::xGetSAD16N;

  m_afpDistortFunc[DF_SAD12  ] = RdCost::xGetSAD12;
  m_afpDistortFunc[DF_SAD24  ] = RdCost::xGetSAD24;
  m_afpDistortFunc[DF_SAD48  ] = RdCost::xGetSAD48;

  m_afpDistortFunc[DF_HAD    ] = RdCost::xGetHADs;
  m_afpDistortFunc[DF_HAD2   ] = RdCost::xGetHADs;
  m_afpDistortFunc[DF_HAD4   ] = RdCost::xGetHADs;
  m_afpDistortFunc[DF_HAD8   ] = RdCost::xGetHADs;
  m_afpDistortFunc[DF_HAD16  ] = RdCost::xGetHADs;
  m_afpDistortFunc[DF_HAD32  ] = RdCost::xGetHADs;
  m_afpDistortFunc[DF_HAD64  ] = RdCost::xGetHADs;
  m_afpDistortFunc[DF_HAD16N ] = RdCost::xGetHADs;

  m_afpDistortFunc[DF_MRSAD    ] = RdCost::xGetMRSAD;
  m_afpDistortFunc[DF_MRSAD2   ] = RdCost::xGetMRSAD;
  m_afpDistortFunc[DF_MRSAD4   ] = RdCost::xGetMRSAD4;
  m_afpDistortFunc[DF_MRSAD8   ] = RdCost::xGetMRSAD8;
  m_afpDistortFunc[DF_MRSAD16  ] = RdCost::xGetMRSAD16;
  m_afpDistortFunc[DF_MRSAD32  ] = RdCost::xGetMRSAD32;
  m_afpDistortFunc[DF_MRSAD64  ] = RdCost::xGetMRSAD64;
  m_afpDistortFunc[DF_MRSAD16N ] = RdCost::xGetMRSAD16N;

  m_afpDistortFunc[DF_MRSAD12  ] = RdCost::xGetMRSAD12;
  m_afpDistortFunc[DF_MRSAD24  ] = RdCost::xGetMRSAD24;
  m_afpDistortFunc[DF_MRSAD48  ] = RdCost::xGetMRSAD48;

  m_afpDistortFunc[DF_MRHAD    ] = RdCost::xGetMRHADs;
  m_afpDistortFunc[DF_MRHAD2   ] = RdCost::xGetMRHADs;
  m_afpDistortFunc[DF_MRHAD4   ] = RdCost::xGetMRHADs;
  m_afpDistortFunc[DF_MRHAD8   ] = RdCost::xGetMRHADs;
  m_afpDistortFunc[DF_MRHAD16  ] = RdCost::xGetMRHADs;
  m_afpDistortFunc[DF_MRHAD32  ] = RdCost::xGetMRHADs;
  m_afpDistortFunc[DF_MRHAD64  ] = RdCost::xGetMRHADs;
  m_afpDistortFunc[DF_MRHAD16N ] = RdCost::xGetMRHADs;

  m_afpDistortFunc[DF_SAD_FULL_NBIT   ] = RdCost::xGetSAD_full;
  m_afpDistortFunc[DF_SAD_FULL_NBIT2  ] = RdCost::xGetSAD_full;
  m_afpDistortFunc[DF_SAD_FULL_NBIT4  ] = RdCost::xGetSAD_full;
  m_afpDistortFunc[DF_SAD_FULL_NBIT8  ] = RdCost::xGetSAD_full;
  m_afpDistortFunc[DF_SAD_FULL_NBIT16 ] = RdCost::xGetSAD_full;
  m_afpDistortFunc[DF_SAD_FULL_NBIT32 ] = RdCost::xGetSAD_full;
  m_afpDistortFunc[DF_SAD_FULL_NBIT64 ] = RdCost::xGetSAD_full;
  m_afpDistortFunc[DF_SAD_FULL_NBIT16N] = RdCost::xGetSAD_full;

#if WCG_EXT
  m_afpDistortFunc[DF_SSE_WTD   ] = RdCost::xGetSSE_WTD;
  m_afpDistortFunc[DF_SSE2_WTD  ] = RdCost::xGetSSE2_WTD;
  m_afpDistortFunc[DF_SSE4_WTD  ] = RdCost::xGetSSE4_WTD;
  m_afpDistortFunc[DF_SSE8_WTD  ] = RdCost::xGetSSE8_WTD;
  m_afpDistortFunc[DF_SSE16_WTD ] = RdCost::xGetSSE16_WTD;
  m_afpDistortFunc[DF_SSE32_WTD ] = RdCost::xGetSSE32_WTD;
  m_afpDistortFunc[DF_SSE64_WTD ] = RdCost::xGetSSE64_WTD;
  m_afpDistortFunc[DF_SSE16N_WTD] = RdCost::xGetSSE16N_WTD;
#endif

  m_afpDistortFunc[DF_SAD_INTERMEDIATE_BITDEPTH] = RdCost::xGetSAD;

  m_afpDistortFunc[DF_SAD_WITH_MASK] = RdCost::xGetSADwMask;

#if ENABLE_SIMD_OPT_DIST
#ifdef TARGET_SIMD_X86
  initRdCostX86();
#endif
#endif

  m_costMode                   = COST_STANDARD_LOSSY;

  m_motionLambda               = 0;
  m_iCostScale                 = 0;
  m_resetStore = true;
  m_pairCheck    = 0;
}


#if ENABLE_SPLIT_PARALLELISM

void RdCost::copyState( const RdCost& other )
{
  m_costMode      = other.m_costMode;
  m_dLambda       = other.m_dLambda;
  m_DistScale     = other.m_DistScale;
  memcpy( m_distortionWeight, other.m_distortionWeight, sizeof( m_distortionWeight ) );
  m_mvPredictor   = other.m_mvPredictor;
  m_motionLambda  = other.m_motionLambda;
  m_iCostScale    = other.m_iCostScale;
  m_dLambdaMotionSAD = other.m_dLambdaMotionSAD;
#if WCG_EXT
  m_dLambda_unadjusted  = other.m_dLambda_unadjusted ;
  m_DistScaleUnadjusted = other.m_DistScaleUnadjusted;
#endif
}
#endif

void RdCost::setDistParam( DistParam &rcDP, const CPelBuf &org, const Pel* piRefY, int iRefStride, int bitDepth, ComponentID compID, int subShiftMode, int step, bool useHadamard )
{
  rcDP.bitDepth   = bitDepth;
  rcDP.compID     = compID;

  // set Original & Curr Pointer / Stride
  rcDP.org        = org;

  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;

  // set Block Width / Height
  rcDP.cur.width    = org.width;
  rcDP.cur.height   = org.height;
  rcDP.step         = step;
  rcDP.maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();

  int DFOffset = ( rcDP.useMR ? DF_MRSAD - DF_SAD : 0 );
  if( !useHadamard )
  {
    if( org.width == 12 )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD12 + DFOffset ];
    }
    else if( org.width == 24 )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD24 + DFOffset ];
    }
    else if( org.width == 48 )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD48 + DFOffset ];
    }
    else if( isPowerOf2( org.width ) )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD + DFOffset + floorLog2( org.width ) ];
    }
    else
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD + DFOffset ];
    }
  }
  else if( isPowerOf2( org.width ) )
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_HAD + DFOffset + floorLog2( org.width ) ];
  }
  else
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_HAD + DFOffset ];
  }

  // initialize
  rcDP.subShift  = 0;

  if( subShiftMode == 1 )
  {
    if( rcDP.org.height > 32 && ( rcDP.org.height & 15 ) == 0 )
    {
      rcDP.subShift = 4;
    }
    else if( rcDP.org.height > 16 && ( rcDP.org.height & 7 ) == 0 )
    {
      rcDP.subShift = 3;
    }
    else if( rcDP.org.height > 8 && ( rcDP.org.height & 3 ) == 0 )
    {
      rcDP.subShift = 2;
    }
    else if( ( rcDP.org.height & 1 ) == 0 )
    {
      rcDP.subShift = 1;
    }
  }
  else if( subShiftMode == 2 )
  {
    if( rcDP.org.height > 8 && rcDP.org.width <= 64 )
    {
      rcDP.subShift = 1;
    }
  }
  else if( subShiftMode == 3 )
  {
    if (rcDP.org.height > 8 )
    {
      rcDP.subShift = 1;
    }
  }
}

void RdCost::setDistParam( DistParam &rcDP, const CPelBuf &org, const CPelBuf &cur, int bitDepth, ComponentID compID, bool useHadamard )
{
  rcDP.org          = org;
  rcDP.cur          = cur;
  rcDP.step         = 1;
  rcDP.subShift     = 0;
  rcDP.bitDepth     = bitDepth;
  rcDP.compID       = compID;

  const int DFOffset = ( rcDP.useMR ? DF_MRSAD - DF_SAD : 0 );

  if( !useHadamard )
  {
    if( org.width == 12 )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD12 + DFOffset ];
    }
    else if( org.width == 24 )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD24 + DFOffset ];
    }
    else if( org.width == 48 )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD48 + DFOffset ];
    }
    else if( isPowerOf2( org.width) )
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD + DFOffset + floorLog2( org.width ) ];
    }
    else
    {
      rcDP.distFunc = m_afpDistortFunc[ DF_SAD + DFOffset ];
    }
  }
  else
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_HAD + DFOffset + floorLog2( org.width ) ];
  }

  rcDP.maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();
}

void RdCost::setDistParam( DistParam &rcDP, const Pel* pOrg, const Pel* piRefY, int iOrgStride, int iRefStride, int bitDepth, ComponentID compID, int width, int height, int subShiftMode, int step, bool useHadamard, bool bioApplied )
{
  rcDP.bitDepth   = bitDepth;
  rcDP.compID     = compID;

  rcDP.org.buf    = pOrg;
  rcDP.org.stride = iOrgStride;
  rcDP.org.width  = width;
  rcDP.org.height = height;

  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;
  rcDP.cur.width  = width;
  rcDP.cur.height = height;
  rcDP.subShift = subShiftMode;
  rcDP.step       = step;
  rcDP.maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();
  CHECK_( useHadamard || rcDP.useMR, "only used in xDMVRCost with these default parameters (so far...)" );
  if ( bioApplied )
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_SAD_INTERMEDIATE_BITDEPTH ];
    return;
  }

  if( width == 12 )
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_SAD12 ];
  }
  else if( width == 24 )
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_SAD24 ];
  }
  else if( width == 48 )
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_SAD48 ];
  }
  else
  {
    rcDP.distFunc = m_afpDistortFunc[ DF_SAD + floorLog2( width ) ];
  }
}

#if WCG_EXT
Distortion RdCost::getDistPart( const CPelBuf &org, const CPelBuf &cur, int bitDepth, const ComponentID compID, DFunc eDFunc, const CPelBuf *orgLuma )
#else
Distortion RdCost::getDistPart( const CPelBuf &org, const CPelBuf &cur, int bitDepth, const ComponentID compID, DFunc eDFunc )
#endif
{
  DistParam cDtParam;

  cDtParam.org        = org;
  cDtParam.cur        = cur;
  cDtParam.step       = 1;
  cDtParam.bitDepth   = bitDepth;
  cDtParam.compID     = compID;

#if WCG_EXT
  if( orgLuma )
  {
    cDtParam.cShiftX = getComponentScaleX(compID,  m_cf);
    cDtParam.cShiftY = getComponentScaleY(compID,  m_cf);
    if( isChroma(compID) )
    {
      cDtParam.orgLuma  = *orgLuma;
    }
    else
    {
      cDtParam.orgLuma  = org;
    }
  }
#endif

  if( isPowerOf2( org.width ) )
  {
    cDtParam.distFunc = m_afpDistortFunc[eDFunc + floorLog2(org.width)];
  }
  else
  {
    cDtParam.distFunc = m_afpDistortFunc[eDFunc];
  }

  if (isChroma(compID))
  {
    return ((Distortion) (m_distortionWeight[ MAP_CHROMA(compID) ] * cDtParam.distFunc( cDtParam )));
  }
  else
  {
    return cDtParam.distFunc( cDtParam );
  }
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetSAD_full( const DistParam& rcDtParam )
{
  CHECK_( rcDtParam.applyWeight, "Cannot apply weight when using full-bit SAD!" );
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int  height      = rcDtParam.org.height;
  int  width       = rcDtParam.org.width;
  int  iSubShift   = rcDtParam.subShift;
  int  iSubStep    = ( 1 << iSubShift );
  int  iStrideCur  = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg  = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

#define SAD_OP( ADDR ) uiSum += abs( piOrg[ADDR] - piCur[ADDR] );
#define SAD_INC piOrg += iStrideOrg; piCur += iStrideCur;

  SIZE_AWARE_PER_EL_OP( SAD_OP, SAD_INC )

#undef SAD_OP
#undef SAD_INC

  uiSum <<= iSubShift;
  return uiSum;
}

Distortion RdCost::xGetSAD( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg           = rcDtParam.org.buf;
  const Pel* piCur           = rcDtParam.cur.buf;
  const int  iCols           = rcDtParam.org.width;
        int  iRows           = rcDtParam.org.height;
  const int  iSubShift       = rcDtParam.subShift;
  const int  iSubStep        = ( 1 << iSubShift );
  const int  iStrideCur      = rcDtParam.cur.stride * iSubStep;
  const int  iStrideOrg      = rcDtParam.org.stride * iSubStep;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    for (int n = 0; n < iCols; n++ )
    {
      uiSum += abs( piOrg[n] - piCur[n] );
    }
    if (rcDtParam.maximumDistortionForEarlyExit < ( uiSum >> distortionShift ))
    {
      return ( uiSum >> distortionShift );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return ( uiSum >> distortionShift );
}

Distortion RdCost::xGetSAD4( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iSubShift     = rcDtParam.subShift;
  int  iSubStep      = ( 1 << iSubShift );
  int  iStrideCur    = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg    = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD8( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD16( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD12( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD16N( const DistParam &rcDtParam )
{
  const Pel* piOrg  = rcDtParam.org.buf;
  const Pel* piCur  = rcDtParam.cur.buf;
  int  iRows        = rcDtParam.org.height;
  int  iCols        = rcDtParam.org.width;
  int  iSubShift    = rcDtParam.subShift;
  int  iSubStep     = ( 1 << iSubShift );
  int  iStrideCur   = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg   = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    for (int n = 0; n < iCols; n+=16 )
    {
      uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
      uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
      uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
      uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
      uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
      uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
      uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
      uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
      uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
      uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
      uiSum += abs( piOrg[n+10] - piCur[n+10] );
      uiSum += abs( piOrg[n+11] - piCur[n+11] );
      uiSum += abs( piOrg[n+12] - piCur[n+12] );
      uiSum += abs( piOrg[n+13] - piCur[n+13] );
      uiSum += abs( piOrg[n+14] - piCur[n+14] );
      uiSum += abs( piOrg[n+15] - piCur[n+15] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD32( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD24( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD64( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD48( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}




// --------------------------------------------------------------------------------------------------------------------
// MRSAD
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetMRSAD( const DistParam& rcDtParam )
{
  const Pel* piOrg           = rcDtParam.org.buf;
  const Pel* piCur           = rcDtParam.cur.buf;
  const int  iCols           = rcDtParam.org.width;
        int  iRows           = rcDtParam.org.height;
  const int  iSubShift       = rcDtParam.subShift;
  const int  iSubStep        = ( 1 << iSubShift );
  const int  iStrideCur      = rcDtParam.cur.stride * iSubStep;
  const int  iStrideOrg      = rcDtParam.org.stride * iSubStep;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    for( int n = 0; n < iCols; n++ )
    {
      deltaSum += ( piOrg[n] - piCur[n] );
    }
  }

  const Pel offset  = Pel( deltaSum / ( iCols * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum = 0;
  for( ; iRows != 0; iRows -= iSubStep )
  {
    for (int n = 0; n < iCols; n++ )
    {
      uiSum += abs( piOrg[n] - piCur[n] - offset );
    }
    if (rcDtParam.maximumDistortionForEarlyExit < ( uiSum >> distortionShift ))
    {
      return ( uiSum >> distortionShift );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }
  uiSum <<= iSubShift;
  return ( uiSum >> distortionShift );
}


Distortion RdCost::xGetMRSAD4( const DistParam& rcDtParam )
{
  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iSubShift     = rcDtParam.subShift;
  int  iSubStep      = ( 1 << iSubShift );
  int  iStrideCur    = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg    = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[0] - piCur[0] );
    deltaSum += ( piOrg[1] - piCur[1] );
    deltaSum += ( piOrg[2] - piCur[2] );
    deltaSum += ( piOrg[3] - piCur[3] );
  }

  const Pel offset  = Pel( deltaSum / ( 4 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] - offset );
    uiSum += abs( piOrg[1] - piCur[1] - offset );
    uiSum += abs( piOrg[2] - piCur[2] - offset );
    uiSum += abs( piOrg[3] - piCur[3] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


Distortion RdCost::xGetMRSAD8( const DistParam& rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[0] - piCur[0] );
    deltaSum += ( piOrg[1] - piCur[1] );
    deltaSum += ( piOrg[2] - piCur[2] );
    deltaSum += ( piOrg[3] - piCur[3] );
    deltaSum += ( piOrg[4] - piCur[4] );
    deltaSum += ( piOrg[5] - piCur[5] );
    deltaSum += ( piOrg[6] - piCur[6] );
    deltaSum += ( piOrg[7] - piCur[7] );
  }

  const Pel offset  = Pel( deltaSum / ( 8 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] - offset );
    uiSum += abs( piOrg[1] - piCur[1] - offset );
    uiSum += abs( piOrg[2] - piCur[2] - offset );
    uiSum += abs( piOrg[3] - piCur[3] - offset );
    uiSum += abs( piOrg[4] - piCur[4] - offset );
    uiSum += abs( piOrg[5] - piCur[5] - offset );
    uiSum += abs( piOrg[6] - piCur[6] - offset );
    uiSum += abs( piOrg[7] - piCur[7] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetMRSAD16( const DistParam& rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[ 0] - piCur[ 0] );
    deltaSum += ( piOrg[ 1] - piCur[ 1] );
    deltaSum += ( piOrg[ 2] - piCur[ 2] );
    deltaSum += ( piOrg[ 3] - piCur[ 3] );
    deltaSum += ( piOrg[ 4] - piCur[ 4] );
    deltaSum += ( piOrg[ 5] - piCur[ 5] );
    deltaSum += ( piOrg[ 6] - piCur[ 6] );
    deltaSum += ( piOrg[ 7] - piCur[ 7] );
    deltaSum += ( piOrg[ 8] - piCur[ 8] );
    deltaSum += ( piOrg[ 9] - piCur[ 9] );
    deltaSum += ( piOrg[10] - piCur[10] );
    deltaSum += ( piOrg[11] - piCur[11] );
    deltaSum += ( piOrg[12] - piCur[12] );
    deltaSum += ( piOrg[13] - piCur[13] );
    deltaSum += ( piOrg[14] - piCur[14] );
    deltaSum += ( piOrg[15] - piCur[15] );
  }

  const Pel offset  = Pel( deltaSum / ( 16 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[ 0] - piCur[ 0] - offset );
    uiSum += abs( piOrg[ 1] - piCur[ 1] - offset );
    uiSum += abs( piOrg[ 2] - piCur[ 2] - offset );
    uiSum += abs( piOrg[ 3] - piCur[ 3] - offset );
    uiSum += abs( piOrg[ 4] - piCur[ 4] - offset );
    uiSum += abs( piOrg[ 5] - piCur[ 5] - offset );
    uiSum += abs( piOrg[ 6] - piCur[ 6] - offset );
    uiSum += abs( piOrg[ 7] - piCur[ 7] - offset );
    uiSum += abs( piOrg[ 8] - piCur[ 8] - offset );
    uiSum += abs( piOrg[ 9] - piCur[ 9] - offset );
    uiSum += abs( piOrg[10] - piCur[10] - offset );
    uiSum += abs( piOrg[11] - piCur[11] - offset );
    uiSum += abs( piOrg[12] - piCur[12] - offset );
    uiSum += abs( piOrg[13] - piCur[13] - offset );
    uiSum += abs( piOrg[14] - piCur[14] - offset );
    uiSum += abs( piOrg[15] - piCur[15] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetMRSAD12( const DistParam& rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[ 0] - piCur[ 0] );
    deltaSum += ( piOrg[ 1] - piCur[ 1] );
    deltaSum += ( piOrg[ 2] - piCur[ 2] );
    deltaSum += ( piOrg[ 3] - piCur[ 3] );
    deltaSum += ( piOrg[ 4] - piCur[ 4] );
    deltaSum += ( piOrg[ 5] - piCur[ 5] );
    deltaSum += ( piOrg[ 6] - piCur[ 6] );
    deltaSum += ( piOrg[ 7] - piCur[ 7] );
    deltaSum += ( piOrg[ 8] - piCur[ 8] );
    deltaSum += ( piOrg[ 9] - piCur[ 9] );
    deltaSum += ( piOrg[10] - piCur[10] );
    deltaSum += ( piOrg[11] - piCur[11] );
  }

  const Pel offset  = Pel( deltaSum / ( 12 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[ 0] - piCur[ 0] - offset );
    uiSum += abs( piOrg[ 1] - piCur[ 1] - offset );
    uiSum += abs( piOrg[ 2] - piCur[ 2] - offset );
    uiSum += abs( piOrg[ 3] - piCur[ 3] - offset );
    uiSum += abs( piOrg[ 4] - piCur[ 4] - offset );
    uiSum += abs( piOrg[ 5] - piCur[ 5] - offset );
    uiSum += abs( piOrg[ 6] - piCur[ 6] - offset );
    uiSum += abs( piOrg[ 7] - piCur[ 7] - offset );
    uiSum += abs( piOrg[ 8] - piCur[ 8] - offset );
    uiSum += abs( piOrg[ 9] - piCur[ 9] - offset );
    uiSum += abs( piOrg[10] - piCur[10] - offset );
    uiSum += abs( piOrg[11] - piCur[11] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetMRSAD16N( const DistParam &rcDtParam )
{
  const Pel* piOrg  = rcDtParam.org.buf;
  const Pel* piCur  = rcDtParam.cur.buf;
  int  iRows        = rcDtParam.org.height;
  int  iCols        = rcDtParam.org.width;
  int  iSubShift    = rcDtParam.subShift;
  int  iSubStep     = ( 1 << iSubShift );
  int  iStrideCur   = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg   = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    for( int n = 0; n < iCols; n += 16 )
    {
      deltaSum += ( piOrg[n+ 0] - piCur[n+ 0] );
      deltaSum += ( piOrg[n+ 1] - piCur[n+ 1] );
      deltaSum += ( piOrg[n+ 2] - piCur[n+ 2] );
      deltaSum += ( piOrg[n+ 3] - piCur[n+ 3] );
      deltaSum += ( piOrg[n+ 4] - piCur[n+ 4] );
      deltaSum += ( piOrg[n+ 5] - piCur[n+ 5] );
      deltaSum += ( piOrg[n+ 6] - piCur[n+ 6] );
      deltaSum += ( piOrg[n+ 7] - piCur[n+ 7] );
      deltaSum += ( piOrg[n+ 8] - piCur[n+ 8] );
      deltaSum += ( piOrg[n+ 9] - piCur[n+ 9] );
      deltaSum += ( piOrg[n+10] - piCur[n+10] );
      deltaSum += ( piOrg[n+11] - piCur[n+11] );
      deltaSum += ( piOrg[n+12] - piCur[n+12] );
      deltaSum += ( piOrg[n+13] - piCur[n+13] );
      deltaSum += ( piOrg[n+14] - piCur[n+14] );
      deltaSum += ( piOrg[n+15] - piCur[n+15] );
    }
  }

  const Pel offset  = Pel( deltaSum / ( iCols * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows-=iSubStep )
  {
    for (int n = 0; n < iCols; n+=16 )
    {
      uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] - offset );
      uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] - offset );
      uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] - offset );
      uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] - offset );
      uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] - offset );
      uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] - offset );
      uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] - offset );
      uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] - offset );
      uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] - offset );
      uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] - offset );
      uiSum += abs( piOrg[n+10] - piCur[n+10] - offset );
      uiSum += abs( piOrg[n+11] - piCur[n+11] - offset );
      uiSum += abs( piOrg[n+12] - piCur[n+12] - offset );
      uiSum += abs( piOrg[n+13] - piCur[n+13] - offset );
      uiSum += abs( piOrg[n+14] - piCur[n+14] - offset );
      uiSum += abs( piOrg[n+15] - piCur[n+15] - offset );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetMRSAD32( const DistParam &rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[ 0] - piCur[ 0] );
    deltaSum += ( piOrg[ 1] - piCur[ 1] );
    deltaSum += ( piOrg[ 2] - piCur[ 2] );
    deltaSum += ( piOrg[ 3] - piCur[ 3] );
    deltaSum += ( piOrg[ 4] - piCur[ 4] );
    deltaSum += ( piOrg[ 5] - piCur[ 5] );
    deltaSum += ( piOrg[ 6] - piCur[ 6] );
    deltaSum += ( piOrg[ 7] - piCur[ 7] );
    deltaSum += ( piOrg[ 8] - piCur[ 8] );
    deltaSum += ( piOrg[ 9] - piCur[ 9] );
    deltaSum += ( piOrg[10] - piCur[10] );
    deltaSum += ( piOrg[11] - piCur[11] );
    deltaSum += ( piOrg[12] - piCur[12] );
    deltaSum += ( piOrg[13] - piCur[13] );
    deltaSum += ( piOrg[14] - piCur[14] );
    deltaSum += ( piOrg[15] - piCur[15] );
    deltaSum += ( piOrg[16] - piCur[16] );
    deltaSum += ( piOrg[17] - piCur[17] );
    deltaSum += ( piOrg[18] - piCur[18] );
    deltaSum += ( piOrg[19] - piCur[19] );
    deltaSum += ( piOrg[20] - piCur[20] );
    deltaSum += ( piOrg[21] - piCur[21] );
    deltaSum += ( piOrg[22] - piCur[22] );
    deltaSum += ( piOrg[23] - piCur[23] );
    deltaSum += ( piOrg[24] - piCur[24] );
    deltaSum += ( piOrg[25] - piCur[25] );
    deltaSum += ( piOrg[26] - piCur[26] );
    deltaSum += ( piOrg[27] - piCur[27] );
    deltaSum += ( piOrg[28] - piCur[28] );
    deltaSum += ( piOrg[29] - piCur[29] );
    deltaSum += ( piOrg[30] - piCur[30] );
    deltaSum += ( piOrg[31] - piCur[31] );
  }

  const Pel offset  = Pel( deltaSum / ( 32 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[ 0] - piCur[ 0] - offset );
    uiSum += abs( piOrg[ 1] - piCur[ 1] - offset );
    uiSum += abs( piOrg[ 2] - piCur[ 2] - offset );
    uiSum += abs( piOrg[ 3] - piCur[ 3] - offset );
    uiSum += abs( piOrg[ 4] - piCur[ 4] - offset );
    uiSum += abs( piOrg[ 5] - piCur[ 5] - offset );
    uiSum += abs( piOrg[ 6] - piCur[ 6] - offset );
    uiSum += abs( piOrg[ 7] - piCur[ 7] - offset );
    uiSum += abs( piOrg[ 8] - piCur[ 8] - offset );
    uiSum += abs( piOrg[ 9] - piCur[ 9] - offset );
    uiSum += abs( piOrg[10] - piCur[10] - offset );
    uiSum += abs( piOrg[11] - piCur[11] - offset );
    uiSum += abs( piOrg[12] - piCur[12] - offset );
    uiSum += abs( piOrg[13] - piCur[13] - offset );
    uiSum += abs( piOrg[14] - piCur[14] - offset );
    uiSum += abs( piOrg[15] - piCur[15] - offset );
    uiSum += abs( piOrg[16] - piCur[16] - offset );
    uiSum += abs( piOrg[17] - piCur[17] - offset );
    uiSum += abs( piOrg[18] - piCur[18] - offset );
    uiSum += abs( piOrg[19] - piCur[19] - offset );
    uiSum += abs( piOrg[20] - piCur[20] - offset );
    uiSum += abs( piOrg[21] - piCur[21] - offset );
    uiSum += abs( piOrg[22] - piCur[22] - offset );
    uiSum += abs( piOrg[23] - piCur[23] - offset );
    uiSum += abs( piOrg[24] - piCur[24] - offset );
    uiSum += abs( piOrg[25] - piCur[25] - offset );
    uiSum += abs( piOrg[26] - piCur[26] - offset );
    uiSum += abs( piOrg[27] - piCur[27] - offset );
    uiSum += abs( piOrg[28] - piCur[28] - offset );
    uiSum += abs( piOrg[29] - piCur[29] - offset );
    uiSum += abs( piOrg[30] - piCur[30] - offset );
    uiSum += abs( piOrg[31] - piCur[31] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetMRSAD24( const DistParam &rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[ 0] - piCur[ 0] );
    deltaSum += ( piOrg[ 1] - piCur[ 1] );
    deltaSum += ( piOrg[ 2] - piCur[ 2] );
    deltaSum += ( piOrg[ 3] - piCur[ 3] );
    deltaSum += ( piOrg[ 4] - piCur[ 4] );
    deltaSum += ( piOrg[ 5] - piCur[ 5] );
    deltaSum += ( piOrg[ 6] - piCur[ 6] );
    deltaSum += ( piOrg[ 7] - piCur[ 7] );
    deltaSum += ( piOrg[ 8] - piCur[ 8] );
    deltaSum += ( piOrg[ 9] - piCur[ 9] );
    deltaSum += ( piOrg[10] - piCur[10] );
    deltaSum += ( piOrg[11] - piCur[11] );
    deltaSum += ( piOrg[12] - piCur[12] );
    deltaSum += ( piOrg[13] - piCur[13] );
    deltaSum += ( piOrg[14] - piCur[14] );
    deltaSum += ( piOrg[15] - piCur[15] );
    deltaSum += ( piOrg[16] - piCur[16] );
    deltaSum += ( piOrg[17] - piCur[17] );
    deltaSum += ( piOrg[18] - piCur[18] );
    deltaSum += ( piOrg[19] - piCur[19] );
    deltaSum += ( piOrg[20] - piCur[20] );
    deltaSum += ( piOrg[21] - piCur[21] );
    deltaSum += ( piOrg[22] - piCur[22] );
    deltaSum += ( piOrg[23] - piCur[23] );
  }

  const Pel offset  = Pel( deltaSum / ( 24 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[ 0] - piCur[ 0] - offset );
    uiSum += abs( piOrg[ 1] - piCur[ 1] - offset );
    uiSum += abs( piOrg[ 2] - piCur[ 2] - offset );
    uiSum += abs( piOrg[ 3] - piCur[ 3] - offset );
    uiSum += abs( piOrg[ 4] - piCur[ 4] - offset );
    uiSum += abs( piOrg[ 5] - piCur[ 5] - offset );
    uiSum += abs( piOrg[ 6] - piCur[ 6] - offset );
    uiSum += abs( piOrg[ 7] - piCur[ 7] - offset );
    uiSum += abs( piOrg[ 8] - piCur[ 8] - offset );
    uiSum += abs( piOrg[ 9] - piCur[ 9] - offset );
    uiSum += abs( piOrg[10] - piCur[10] - offset );
    uiSum += abs( piOrg[11] - piCur[11] - offset );
    uiSum += abs( piOrg[12] - piCur[12] - offset );
    uiSum += abs( piOrg[13] - piCur[13] - offset );
    uiSum += abs( piOrg[14] - piCur[14] - offset );
    uiSum += abs( piOrg[15] - piCur[15] - offset );
    uiSum += abs( piOrg[16] - piCur[16] - offset );
    uiSum += abs( piOrg[17] - piCur[17] - offset );
    uiSum += abs( piOrg[18] - piCur[18] - offset );
    uiSum += abs( piOrg[19] - piCur[19] - offset );
    uiSum += abs( piOrg[20] - piCur[20] - offset );
    uiSum += abs( piOrg[21] - piCur[21] - offset );
    uiSum += abs( piOrg[22] - piCur[22] - offset );
    uiSum += abs( piOrg[23] - piCur[23] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetMRSAD64( const DistParam &rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[ 0] - piCur[ 0] );
    deltaSum += ( piOrg[ 1] - piCur[ 1] );
    deltaSum += ( piOrg[ 2] - piCur[ 2] );
    deltaSum += ( piOrg[ 3] - piCur[ 3] );
    deltaSum += ( piOrg[ 4] - piCur[ 4] );
    deltaSum += ( piOrg[ 5] - piCur[ 5] );
    deltaSum += ( piOrg[ 6] - piCur[ 6] );
    deltaSum += ( piOrg[ 7] - piCur[ 7] );
    deltaSum += ( piOrg[ 8] - piCur[ 8] );
    deltaSum += ( piOrg[ 9] - piCur[ 9] );
    deltaSum += ( piOrg[10] - piCur[10] );
    deltaSum += ( piOrg[11] - piCur[11] );
    deltaSum += ( piOrg[12] - piCur[12] );
    deltaSum += ( piOrg[13] - piCur[13] );
    deltaSum += ( piOrg[14] - piCur[14] );
    deltaSum += ( piOrg[15] - piCur[15] );
    deltaSum += ( piOrg[16] - piCur[16] );
    deltaSum += ( piOrg[17] - piCur[17] );
    deltaSum += ( piOrg[18] - piCur[18] );
    deltaSum += ( piOrg[19] - piCur[19] );
    deltaSum += ( piOrg[20] - piCur[20] );
    deltaSum += ( piOrg[21] - piCur[21] );
    deltaSum += ( piOrg[22] - piCur[22] );
    deltaSum += ( piOrg[23] - piCur[23] );
    deltaSum += ( piOrg[24] - piCur[24] );
    deltaSum += ( piOrg[25] - piCur[25] );
    deltaSum += ( piOrg[26] - piCur[26] );
    deltaSum += ( piOrg[27] - piCur[27] );
    deltaSum += ( piOrg[28] - piCur[28] );
    deltaSum += ( piOrg[29] - piCur[29] );
    deltaSum += ( piOrg[30] - piCur[30] );
    deltaSum += ( piOrg[31] - piCur[31] );
    deltaSum += ( piOrg[32] - piCur[32] );
    deltaSum += ( piOrg[33] - piCur[33] );
    deltaSum += ( piOrg[34] - piCur[34] );
    deltaSum += ( piOrg[35] - piCur[35] );
    deltaSum += ( piOrg[36] - piCur[36] );
    deltaSum += ( piOrg[37] - piCur[37] );
    deltaSum += ( piOrg[38] - piCur[38] );
    deltaSum += ( piOrg[39] - piCur[39] );
    deltaSum += ( piOrg[40] - piCur[40] );
    deltaSum += ( piOrg[41] - piCur[41] );
    deltaSum += ( piOrg[42] - piCur[42] );
    deltaSum += ( piOrg[43] - piCur[43] );
    deltaSum += ( piOrg[44] - piCur[44] );
    deltaSum += ( piOrg[45] - piCur[45] );
    deltaSum += ( piOrg[46] - piCur[46] );
    deltaSum += ( piOrg[47] - piCur[47] );
    deltaSum += ( piOrg[48] - piCur[48] );
    deltaSum += ( piOrg[49] - piCur[49] );
    deltaSum += ( piOrg[50] - piCur[50] );
    deltaSum += ( piOrg[51] - piCur[51] );
    deltaSum += ( piOrg[52] - piCur[52] );
    deltaSum += ( piOrg[53] - piCur[53] );
    deltaSum += ( piOrg[54] - piCur[54] );
    deltaSum += ( piOrg[55] - piCur[55] );
    deltaSum += ( piOrg[56] - piCur[56] );
    deltaSum += ( piOrg[57] - piCur[57] );
    deltaSum += ( piOrg[58] - piCur[58] );
    deltaSum += ( piOrg[59] - piCur[59] );
    deltaSum += ( piOrg[60] - piCur[60] );
    deltaSum += ( piOrg[61] - piCur[61] );
    deltaSum += ( piOrg[62] - piCur[62] );
    deltaSum += ( piOrg[63] - piCur[63] );
  }

  const Pel offset  = Pel( deltaSum / ( 64 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[ 0] - piCur[ 0] - offset );
    uiSum += abs( piOrg[ 1] - piCur[ 1] - offset );
    uiSum += abs( piOrg[ 2] - piCur[ 2] - offset );
    uiSum += abs( piOrg[ 3] - piCur[ 3] - offset );
    uiSum += abs( piOrg[ 4] - piCur[ 4] - offset );
    uiSum += abs( piOrg[ 5] - piCur[ 5] - offset );
    uiSum += abs( piOrg[ 6] - piCur[ 6] - offset );
    uiSum += abs( piOrg[ 7] - piCur[ 7] - offset );
    uiSum += abs( piOrg[ 8] - piCur[ 8] - offset );
    uiSum += abs( piOrg[ 9] - piCur[ 9] - offset );
    uiSum += abs( piOrg[10] - piCur[10] - offset );
    uiSum += abs( piOrg[11] - piCur[11] - offset );
    uiSum += abs( piOrg[12] - piCur[12] - offset );
    uiSum += abs( piOrg[13] - piCur[13] - offset );
    uiSum += abs( piOrg[14] - piCur[14] - offset );
    uiSum += abs( piOrg[15] - piCur[15] - offset );
    uiSum += abs( piOrg[16] - piCur[16] - offset );
    uiSum += abs( piOrg[17] - piCur[17] - offset );
    uiSum += abs( piOrg[18] - piCur[18] - offset );
    uiSum += abs( piOrg[19] - piCur[19] - offset );
    uiSum += abs( piOrg[20] - piCur[20] - offset );
    uiSum += abs( piOrg[21] - piCur[21] - offset );
    uiSum += abs( piOrg[22] - piCur[22] - offset );
    uiSum += abs( piOrg[23] - piCur[23] - offset );
    uiSum += abs( piOrg[24] - piCur[24] - offset );
    uiSum += abs( piOrg[25] - piCur[25] - offset );
    uiSum += abs( piOrg[26] - piCur[26] - offset );
    uiSum += abs( piOrg[27] - piCur[27] - offset );
    uiSum += abs( piOrg[28] - piCur[28] - offset );
    uiSum += abs( piOrg[29] - piCur[29] - offset );
    uiSum += abs( piOrg[30] - piCur[30] - offset );
    uiSum += abs( piOrg[31] - piCur[31] - offset );
    uiSum += abs( piOrg[32] - piCur[32] - offset );
    uiSum += abs( piOrg[33] - piCur[33] - offset );
    uiSum += abs( piOrg[34] - piCur[34] - offset );
    uiSum += abs( piOrg[35] - piCur[35] - offset );
    uiSum += abs( piOrg[36] - piCur[36] - offset );
    uiSum += abs( piOrg[37] - piCur[37] - offset );
    uiSum += abs( piOrg[38] - piCur[38] - offset );
    uiSum += abs( piOrg[39] - piCur[39] - offset );
    uiSum += abs( piOrg[40] - piCur[40] - offset );
    uiSum += abs( piOrg[41] - piCur[41] - offset );
    uiSum += abs( piOrg[42] - piCur[42] - offset );
    uiSum += abs( piOrg[43] - piCur[43] - offset );
    uiSum += abs( piOrg[44] - piCur[44] - offset );
    uiSum += abs( piOrg[45] - piCur[45] - offset );
    uiSum += abs( piOrg[46] - piCur[46] - offset );
    uiSum += abs( piOrg[47] - piCur[47] - offset );
    uiSum += abs( piOrg[48] - piCur[48] - offset );
    uiSum += abs( piOrg[49] - piCur[49] - offset );
    uiSum += abs( piOrg[50] - piCur[50] - offset );
    uiSum += abs( piOrg[51] - piCur[51] - offset );
    uiSum += abs( piOrg[52] - piCur[52] - offset );
    uiSum += abs( piOrg[53] - piCur[53] - offset );
    uiSum += abs( piOrg[54] - piCur[54] - offset );
    uiSum += abs( piOrg[55] - piCur[55] - offset );
    uiSum += abs( piOrg[56] - piCur[56] - offset );
    uiSum += abs( piOrg[57] - piCur[57] - offset );
    uiSum += abs( piOrg[58] - piCur[58] - offset );
    uiSum += abs( piOrg[59] - piCur[59] - offset );
    uiSum += abs( piOrg[60] - piCur[60] - offset );
    uiSum += abs( piOrg[61] - piCur[61] - offset );
    uiSum += abs( piOrg[62] - piCur[62] - offset );
    uiSum += abs( piOrg[63] - piCur[63] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetMRSAD48( const DistParam &rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  int32_t deltaSum = 0;
  for( int r = iRows; r != 0; r-=iSubStep, piOrg += iStrideOrg, piCur += iStrideCur )
  {
    deltaSum += ( piOrg[ 0] - piCur[ 0] );
    deltaSum += ( piOrg[ 1] - piCur[ 1] );
    deltaSum += ( piOrg[ 2] - piCur[ 2] );
    deltaSum += ( piOrg[ 3] - piCur[ 3] );
    deltaSum += ( piOrg[ 4] - piCur[ 4] );
    deltaSum += ( piOrg[ 5] - piCur[ 5] );
    deltaSum += ( piOrg[ 6] - piCur[ 6] );
    deltaSum += ( piOrg[ 7] - piCur[ 7] );
    deltaSum += ( piOrg[ 8] - piCur[ 8] );
    deltaSum += ( piOrg[ 9] - piCur[ 9] );
    deltaSum += ( piOrg[10] - piCur[10] );
    deltaSum += ( piOrg[11] - piCur[11] );
    deltaSum += ( piOrg[12] - piCur[12] );
    deltaSum += ( piOrg[13] - piCur[13] );
    deltaSum += ( piOrg[14] - piCur[14] );
    deltaSum += ( piOrg[15] - piCur[15] );
    deltaSum += ( piOrg[16] - piCur[16] );
    deltaSum += ( piOrg[17] - piCur[17] );
    deltaSum += ( piOrg[18] - piCur[18] );
    deltaSum += ( piOrg[19] - piCur[19] );
    deltaSum += ( piOrg[20] - piCur[20] );
    deltaSum += ( piOrg[21] - piCur[21] );
    deltaSum += ( piOrg[22] - piCur[22] );
    deltaSum += ( piOrg[23] - piCur[23] );
    deltaSum += ( piOrg[24] - piCur[24] );
    deltaSum += ( piOrg[25] - piCur[25] );
    deltaSum += ( piOrg[26] - piCur[26] );
    deltaSum += ( piOrg[27] - piCur[27] );
    deltaSum += ( piOrg[28] - piCur[28] );
    deltaSum += ( piOrg[29] - piCur[29] );
    deltaSum += ( piOrg[30] - piCur[30] );
    deltaSum += ( piOrg[31] - piCur[31] );
    deltaSum += ( piOrg[32] - piCur[32] );
    deltaSum += ( piOrg[33] - piCur[33] );
    deltaSum += ( piOrg[34] - piCur[34] );
    deltaSum += ( piOrg[35] - piCur[35] );
    deltaSum += ( piOrg[36] - piCur[36] );
    deltaSum += ( piOrg[37] - piCur[37] );
    deltaSum += ( piOrg[38] - piCur[38] );
    deltaSum += ( piOrg[39] - piCur[39] );
    deltaSum += ( piOrg[40] - piCur[40] );
    deltaSum += ( piOrg[41] - piCur[41] );
    deltaSum += ( piOrg[42] - piCur[42] );
    deltaSum += ( piOrg[43] - piCur[43] );
    deltaSum += ( piOrg[44] - piCur[44] );
    deltaSum += ( piOrg[45] - piCur[45] );
    deltaSum += ( piOrg[46] - piCur[46] );
    deltaSum += ( piOrg[47] - piCur[47] );
  }

  const Pel offset  = Pel( deltaSum / ( 48 * ( iRows >> iSubShift ) ) );
  piOrg             = rcDtParam.org.buf;
  piCur             = rcDtParam.cur.buf;
  Distortion uiSum  = 0;
  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[ 0] - piCur[ 0] - offset );
    uiSum += abs( piOrg[ 1] - piCur[ 1] - offset );
    uiSum += abs( piOrg[ 2] - piCur[ 2] - offset );
    uiSum += abs( piOrg[ 3] - piCur[ 3] - offset );
    uiSum += abs( piOrg[ 4] - piCur[ 4] - offset );
    uiSum += abs( piOrg[ 5] - piCur[ 5] - offset );
    uiSum += abs( piOrg[ 6] - piCur[ 6] - offset );
    uiSum += abs( piOrg[ 7] - piCur[ 7] - offset );
    uiSum += abs( piOrg[ 8] - piCur[ 8] - offset );
    uiSum += abs( piOrg[ 9] - piCur[ 9] - offset );
    uiSum += abs( piOrg[10] - piCur[10] - offset );
    uiSum += abs( piOrg[11] - piCur[11] - offset );
    uiSum += abs( piOrg[12] - piCur[12] - offset );
    uiSum += abs( piOrg[13] - piCur[13] - offset );
    uiSum += abs( piOrg[14] - piCur[14] - offset );
    uiSum += abs( piOrg[15] - piCur[15] - offset );
    uiSum += abs( piOrg[16] - piCur[16] - offset );
    uiSum += abs( piOrg[17] - piCur[17] - offset );
    uiSum += abs( piOrg[18] - piCur[18] - offset );
    uiSum += abs( piOrg[19] - piCur[19] - offset );
    uiSum += abs( piOrg[20] - piCur[20] - offset );
    uiSum += abs( piOrg[21] - piCur[21] - offset );
    uiSum += abs( piOrg[22] - piCur[22] - offset );
    uiSum += abs( piOrg[23] - piCur[23] - offset );
    uiSum += abs( piOrg[24] - piCur[24] - offset );
    uiSum += abs( piOrg[25] - piCur[25] - offset );
    uiSum += abs( piOrg[26] - piCur[26] - offset );
    uiSum += abs( piOrg[27] - piCur[27] - offset );
    uiSum += abs( piOrg[28] - piCur[28] - offset );
    uiSum += abs( piOrg[29] - piCur[29] - offset );
    uiSum += abs( piOrg[30] - piCur[30] - offset );
    uiSum += abs( piOrg[31] - piCur[31] - offset );
    uiSum += abs( piOrg[32] - piCur[32] - offset );
    uiSum += abs( piOrg[33] - piCur[33] - offset );
    uiSum += abs( piOrg[34] - piCur[34] - offset );
    uiSum += abs( piOrg[35] - piCur[35] - offset );
    uiSum += abs( piOrg[36] - piCur[36] - offset );
    uiSum += abs( piOrg[37] - piCur[37] - offset );
    uiSum += abs( piOrg[38] - piCur[38] - offset );
    uiSum += abs( piOrg[39] - piCur[39] - offset );
    uiSum += abs( piOrg[40] - piCur[40] - offset );
    uiSum += abs( piOrg[41] - piCur[41] - offset );
    uiSum += abs( piOrg[42] - piCur[42] - offset );
    uiSum += abs( piOrg[43] - piCur[43] - offset );
    uiSum += abs( piOrg[44] - piCur[44] - offset );
    uiSum += abs( piOrg[45] - piCur[45] - offset );
    uiSum += abs( piOrg[46] - piCur[46] - offset );
    uiSum += abs( piOrg[47] - piCur[47] - offset );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetSSE( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iCols            = rcDtParam.org.width;
  int  iStrideCur       = rcDtParam.cur.stride;
  int  iStrideOrg       = rcDtParam.org.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n++ )
    {
      iTemp = piOrg[n  ] - piCur[n  ];
      uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE4( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 4, "Invalid size" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {

    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE8( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 8, "Invalid size" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[4] - piCur[4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[5] - piCur[5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[6] - piCur[6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[7] - piCur[7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE16( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 16, "Invalid size" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE16N( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }
  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iCols         = rcDtParam.org.width;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n+=16 )
    {
      iTemp = piOrg[n+ 0] - piCur[n+ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 1] - piCur[n+ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 2] - piCur[n+ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 3] - piCur[n+ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 4] - piCur[n+ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 5] - piCur[n+ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 6] - piCur[n+ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 7] - piCur[n+ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 8] - piCur[n+ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 9] - piCur[n+ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+10] - piCur[n+10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+11] - piCur[n+11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+12] - piCur[n+12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+13] - piCur[n+13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+14] - piCur[n+14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+15] - piCur[n+15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE32( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 32, "Invalid size" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE64( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 64, "Invalid size" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[32] - piCur[32]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[33] - piCur[33]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[34] - piCur[34]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[35] - piCur[35]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[36] - piCur[36]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[37] - piCur[37]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[38] - piCur[38]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[39] - piCur[39]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[40] - piCur[40]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[41] - piCur[41]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[42] - piCur[42]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[43] - piCur[43]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[44] - piCur[44]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[45] - piCur[45]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[46] - piCur[46]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[47] - piCur[47]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[48] - piCur[48]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[49] - piCur[49]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[50] - piCur[50]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[51] - piCur[51]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[52] - piCur[52]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[53] - piCur[53]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[54] - piCur[54]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[55] - piCur[55]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[56] - piCur[56]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[57] - piCur[57]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[58] - piCur[58]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[59] - piCur[59]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[60] - piCur[60]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[61] - piCur[61]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[62] - piCur[62]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[63] - piCur[63]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xCalcHADs2x2( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iStep )
{
  Distortion satd = 0;
  TCoeff diff[4], m[4];
  CHECK_( iStep != 1, "Invalid step" );
  diff[0] = piOrg[0             ] - piCur[0];
  diff[1] = piOrg[1             ] - piCur[1];
  diff[2] = piOrg[iStrideOrg    ] - piCur[0 + iStrideCur];
  diff[3] = piOrg[iStrideOrg + 1] - piCur[1 + iStrideCur];
  m[0] = diff[0] + diff[2];
  m[1] = diff[1] + diff[3];
  m[2] = diff[0] - diff[2];
  m[3] = diff[1] - diff[3];

#if JVET_R0164_MEAN_SCALED_SATD
  satd += abs(m[0] + m[1]) >> 2;
#else
  satd += abs(m[0] + m[1]);
#endif
  satd += abs(m[0] - m[1]);
  satd += abs(m[2] + m[3]);
  satd += abs(m[2] - m[3]);

  return satd;
}

Distortion RdCost::xCalcHADs4x4( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iStep )
{
  int k;
  Distortion satd = 0;
  TCoeff diff[16], m[16], d[16];

  CHECK_( iStep != 1, "Invalid step" );
  for( k = 0; k < 16; k+=4 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  /*===== hadamard transform =====*/
  m[ 0] = diff[ 0] + diff[12];
  m[ 1] = diff[ 1] + diff[13];
  m[ 2] = diff[ 2] + diff[14];
  m[ 3] = diff[ 3] + diff[15];
  m[ 4] = diff[ 4] + diff[ 8];
  m[ 5] = diff[ 5] + diff[ 9];
  m[ 6] = diff[ 6] + diff[10];
  m[ 7] = diff[ 7] + diff[11];
  m[ 8] = diff[ 4] - diff[ 8];
  m[ 9] = diff[ 5] - diff[ 9];
  m[10] = diff[ 6] - diff[10];
  m[11] = diff[ 7] - diff[11];
  m[12] = diff[ 0] - diff[12];
  m[13] = diff[ 1] - diff[13];
  m[14] = diff[ 2] - diff[14];
  m[15] = diff[ 3] - diff[15];

  d[ 0] = m[ 0] + m[ 4];
  d[ 1] = m[ 1] + m[ 5];
  d[ 2] = m[ 2] + m[ 6];
  d[ 3] = m[ 3] + m[ 7];
  d[ 4] = m[ 8] + m[12];
  d[ 5] = m[ 9] + m[13];
  d[ 6] = m[10] + m[14];
  d[ 7] = m[11] + m[15];
  d[ 8] = m[ 0] - m[ 4];
  d[ 9] = m[ 1] - m[ 5];
  d[10] = m[ 2] - m[ 6];
  d[11] = m[ 3] - m[ 7];
  d[12] = m[12] - m[ 8];
  d[13] = m[13] - m[ 9];
  d[14] = m[14] - m[10];
  d[15] = m[15] - m[11];

  m[ 0] = d[ 0] + d[ 3];
  m[ 1] = d[ 1] + d[ 2];
  m[ 2] = d[ 1] - d[ 2];
  m[ 3] = d[ 0] - d[ 3];
  m[ 4] = d[ 4] + d[ 7];
  m[ 5] = d[ 5] + d[ 6];
  m[ 6] = d[ 5] - d[ 6];
  m[ 7] = d[ 4] - d[ 7];
  m[ 8] = d[ 8] + d[11];
  m[ 9] = d[ 9] + d[10];
  m[10] = d[ 9] - d[10];
  m[11] = d[ 8] - d[11];
  m[12] = d[12] + d[15];
  m[13] = d[13] + d[14];
  m[14] = d[13] - d[14];
  m[15] = d[12] - d[15];

  d[ 0] = m[ 0] + m[ 1];
  d[ 1] = m[ 0] - m[ 1];
  d[ 2] = m[ 2] + m[ 3];
  d[ 3] = m[ 3] - m[ 2];
  d[ 4] = m[ 4] + m[ 5];
  d[ 5] = m[ 4] - m[ 5];
  d[ 6] = m[ 6] + m[ 7];
  d[ 7] = m[ 7] - m[ 6];
  d[ 8] = m[ 8] + m[ 9];
  d[ 9] = m[ 8] - m[ 9];
  d[10] = m[10] + m[11];
  d[11] = m[11] - m[10];
  d[12] = m[12] + m[13];
  d[13] = m[12] - m[13];
  d[14] = m[14] + m[15];
  d[15] = m[15] - m[14];

  for (k=0; k<16; ++k)
  {
    satd += abs(d[k]);
  }

#if JVET_R0164_MEAN_SCALED_SATD
  satd -= abs(d[0]);
  satd += abs(d[0]) >> 2;
#endif
  satd  = ((satd+1)>>1);

  return satd;
}

Distortion RdCost::xCalcHADs8x8( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iStep )
{
  int k, i, j, jj;
  Distortion sad = 0;
  TCoeff diff[64], m1[8][8], m2[8][8], m3[8][8];
  CHECK_( iStep != 1, "Invalid step" );
  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= abs(m2[0][0]);
  sad += abs(m2[0][0]) >> 2;
#endif
  sad  = ((sad+2)>>2);

  return sad;
}

Distortion RdCost::xCalcHADs16x8( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur )
{   //need to add SIMD implementation ,JCA
  int k, i, j, jj, sad = 0;
  int diff[128], m1[8][16], m2[8][16];
  for( k = 0; k < 128; k += 16 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    diff[k + 8] = piOrg[8] - piCur[8];
    diff[k + 9] = piOrg[9] - piCur[9];
    diff[k + 10] = piOrg[10] - piCur[10];
    diff[k + 11] = piOrg[11] - piCur[11];
    diff[k + 12] = piOrg[12] - piCur[12];
    diff[k + 13] = piOrg[13] - piCur[13];
    diff[k + 14] = piOrg[14] - piCur[14];
    diff[k + 15] = piOrg[15] - piCur[15];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 8; j++ )
  {
    jj = j << 4;

    m2[j][0] = diff[jj    ] + diff[jj + 8];
    m2[j][1] = diff[jj + 1] + diff[jj + 9];
    m2[j][2] = diff[jj + 2] + diff[jj + 10];
    m2[j][3] = diff[jj + 3] + diff[jj + 11];
    m2[j][4] = diff[jj + 4] + diff[jj + 12];
    m2[j][5] = diff[jj + 5] + diff[jj + 13];
    m2[j][6] = diff[jj + 6] + diff[jj + 14];
    m2[j][7] = diff[jj + 7] + diff[jj + 15];
    m2[j][8] = diff[jj    ] - diff[jj + 8];
    m2[j][9] = diff[jj + 1] - diff[jj + 9];
    m2[j][10] = diff[jj + 2] - diff[jj + 10];
    m2[j][11] = diff[jj + 3] - diff[jj + 11];
    m2[j][12] = diff[jj + 4] - diff[jj + 12];
    m2[j][13] = diff[jj + 5] - diff[jj + 13];
    m2[j][14] = diff[jj + 6] - diff[jj + 14];
    m2[j][15] = diff[jj + 7] - diff[jj + 15];

    m1[j][0] = m2[j][0] + m2[j][4];
    m1[j][1] = m2[j][1] + m2[j][5];
    m1[j][2] = m2[j][2] + m2[j][6];
    m1[j][3] = m2[j][3] + m2[j][7];
    m1[j][4] = m2[j][0] - m2[j][4];
    m1[j][5] = m2[j][1] - m2[j][5];
    m1[j][6] = m2[j][2] - m2[j][6];
    m1[j][7] = m2[j][3] - m2[j][7];
    m1[j][8] = m2[j][8] + m2[j][12];
    m1[j][9] = m2[j][9] + m2[j][13];
    m1[j][10] = m2[j][10] + m2[j][14];
    m1[j][11] = m2[j][11] + m2[j][15];
    m1[j][12] = m2[j][8] - m2[j][12];
    m1[j][13] = m2[j][9] - m2[j][13];
    m1[j][14] = m2[j][10] - m2[j][14];
    m1[j][15] = m2[j][11] - m2[j][15];

    m2[j][0] = m1[j][0] + m1[j][2];
    m2[j][1] = m1[j][1] + m1[j][3];
    m2[j][2] = m1[j][0] - m1[j][2];
    m2[j][3] = m1[j][1] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][6];
    m2[j][5] = m1[j][5] + m1[j][7];
    m2[j][6] = m1[j][4] - m1[j][6];
    m2[j][7] = m1[j][5] - m1[j][7];
    m2[j][8] = m1[j][8] + m1[j][10];
    m2[j][9] = m1[j][9] + m1[j][11];
    m2[j][10] = m1[j][8] - m1[j][10];
    m2[j][11] = m1[j][9] - m1[j][11];
    m2[j][12] = m1[j][12] + m1[j][14];
    m2[j][13] = m1[j][13] + m1[j][15];
    m2[j][14] = m1[j][12] - m1[j][14];
    m2[j][15] = m1[j][13] - m1[j][15];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][5];
    m1[j][5] = m2[j][4] - m2[j][5];
    m1[j][6] = m2[j][6] + m2[j][7];
    m1[j][7] = m2[j][6] - m2[j][7];
    m1[j][8] = m2[j][8] + m2[j][9];
    m1[j][9] = m2[j][8] - m2[j][9];
    m1[j][10] = m2[j][10] + m2[j][11];
    m1[j][11] = m2[j][10] - m2[j][11];
    m1[j][12] = m2[j][12] + m2[j][13];
    m1[j][13] = m2[j][12] - m2[j][13];
    m1[j][14] = m2[j][14] + m2[j][15];
    m1[j][15] = m2[j][14] - m2[j][15];
  }

  //vertical
  for( i = 0; i < 16; i++ )
  {
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for( i = 0; i < 8; i++ )
  {
    for( j = 0; j < 16; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= abs(m2[0][0]);
  sad += abs(m2[0][0]) >> 2;
#endif
  sad  = ( int ) ( sad / sqrt( 16.0 * 8 ) * 2 );

  return sad;
}

Distortion RdCost::xCalcHADs8x16( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[128], m1[16][8], m2[16][8];
  for( k = 0; k < 128; k += 8 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 16; j++ )
  {
    jj = j << 3;

    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for( i = 0; i < 8; i++ )
  {
    m1[0][i] = m2[0][i] + m2[8][i];
    m1[1][i] = m2[1][i] + m2[9][i];
    m1[2][i] = m2[2][i] + m2[10][i];
    m1[3][i] = m2[3][i] + m2[11][i];
    m1[4][i] = m2[4][i] + m2[12][i];
    m1[5][i] = m2[5][i] + m2[13][i];
    m1[6][i] = m2[6][i] + m2[14][i];
    m1[7][i] = m2[7][i] + m2[15][i];
    m1[8][i] = m2[0][i] - m2[8][i];
    m1[9][i] = m2[1][i] - m2[9][i];
    m1[10][i] = m2[2][i] - m2[10][i];
    m1[11][i] = m2[3][i] - m2[11][i];
    m1[12][i] = m2[4][i] - m2[12][i];
    m1[13][i] = m2[5][i] - m2[13][i];
    m1[14][i] = m2[6][i] - m2[14][i];
    m1[15][i] = m2[7][i] - m2[15][i];

    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];
    m2[8][i] = m1[8][i] + m1[12][i];
    m2[9][i] = m1[9][i] + m1[13][i];
    m2[10][i] = m1[10][i] + m1[14][i];
    m2[11][i] = m1[11][i] + m1[15][i];
    m2[12][i] = m1[8][i] - m1[12][i];
    m2[13][i] = m1[9][i] - m1[13][i];
    m2[14][i] = m1[10][i] - m1[14][i];
    m2[15][i] = m1[11][i] - m1[15][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];
    m1[8][i] = m2[8][i] + m2[10][i];
    m1[9][i] = m2[9][i] + m2[11][i];
    m1[10][i] = m2[8][i] - m2[10][i];
    m1[11][i] = m2[9][i] - m2[11][i];
    m1[12][i] = m2[12][i] + m2[14][i];
    m1[13][i] = m2[13][i] + m2[15][i];
    m1[14][i] = m2[12][i] - m2[14][i];
    m1[15][i] = m2[13][i] - m2[15][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
    m2[8][i] = m1[8][i] + m1[9][i];
    m2[9][i] = m1[8][i] - m1[9][i];
    m2[10][i] = m1[10][i] + m1[11][i];
    m2[11][i] = m1[10][i] - m1[11][i];
    m2[12][i] = m1[12][i] + m1[13][i];
    m2[13][i] = m1[12][i] - m1[13][i];
    m2[14][i] = m1[14][i] + m1[15][i];
    m2[15][i] = m1[14][i] - m1[15][i];
  }

  for( i = 0; i < 16; i++ )
  {
    for( j = 0; j < 8; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= abs(m2[0][0]);
  sad += abs(m2[0][0]) >> 2;
#endif
  sad  = ( int ) ( sad / sqrt( 16.0 * 8 ) * 2 );

  return sad;
}
Distortion RdCost::xCalcHADs4x8( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[32], m1[8][4], m2[8][4];
  for( k = 0; k < 32; k += 4 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 8; j++ )
  {
    jj = j << 2;
    m2[j][0] = diff[jj] + diff[jj + 2];
    m2[j][1] = diff[jj + 1] + diff[jj + 3];
    m2[j][2] = diff[jj] - diff[jj + 2];
    m2[j][3] = diff[jj + 1] - diff[jj + 3];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
  }

  //vertical
  for( i = 0; i < 4; i++ )
  {
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for( i = 0; i < 8; i++ )
  {
    for( j = 0; j < 4; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= abs(m2[0][0]);
  sad += abs(m2[0][0]) >> 2;
#endif
  sad  = ( int ) ( sad / sqrt( 4.0 * 8 ) * 2 );

  return sad;
}

Distortion RdCost::xCalcHADs8x4( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[32], m1[4][8], m2[4][8];
  for( k = 0; k < 32; k += 8 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 4; j++ )
  {
    jj = j << 3;

    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for( i = 0; i < 8; i++ )
  {
    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
  }

  for( i = 0; i < 4; i++ )
  {
    for( j = 0; j < 8; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= abs(m2[0][0]);
  sad += abs(m2[0][0]) >> 2;
#endif
  sad  = ( int ) ( sad / sqrt( 4.0 * 8 ) * 2 );

  return sad;
}

Distortion RdCost::xGetHADs( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetHADsw( rcDtParam );
  }
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iRows = rcDtParam.org.height;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const int  iStep = rcDtParam.step;

  int  x = 0, y = 0;

  Distortion uiSum = 0;

  if( iCols > iRows && ( iRows & 7 ) == 0 && ( iCols & 15 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHADs16x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 8;
      piCur += iStrideCur * 8;
    }
  }
  else if( iCols < iRows && ( iCols & 7 ) == 0 && ( iRows & 15 ) == 0 )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x16( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 16;
      piCur += iStrideCur * 16;
    }
  }
  else if( iCols > iRows && ( iRows & 3 ) == 0 && ( iCols & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x4( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 4;
      piCur += iStrideCur * 4;
    }
  }
  else if( iCols < iRows && ( iCols & 3 ) == 0 && ( iRows & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHADs4x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 8;
      piCur += iStrideCur * 8;
    }
  }
  else if( ( iRows % 8 == 0 ) && ( iCols % 8 == 0 ) )
  {
    int  iOffsetOrg = iStrideOrg << 3;
    int  iOffsetCur = iStrideCur << 3;
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x8( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else if( ( iRows % 4 == 0 ) && ( iCols % 4 == 0 ) )
  {
    int  iOffsetOrg = iStrideOrg << 2;
    int  iOffsetCur = iStrideCur << 2;

    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHADs4x4( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else if( ( iRows % 2 == 0 ) && ( iCols % 2 == 0 ) )
  {
    int  iOffsetOrg = iStrideOrg << 1;
    int  iOffsetCur = iStrideCur << 1;
    for( y = 0; y < iRows; y += 2 )
    {
      for( x = 0; x < iCols; x += 2 )
      {
        uiSum += xCalcHADs2x2( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else
  {
    THROW( "Invalid size" );
  }

  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


#if WCG_EXT
uint32_t   RdCost::m_signalType                 = RESHAPE_SIGNAL_NULL;
double     RdCost::m_chromaWeight               = 1.0;
int        RdCost::m_lumaBD                     = 10;
std::vector<double> RdCost::m_reshapeLumaLevelToWeightPLUT;
std::vector<double> RdCost::m_lumaLevelToWeightPLUT;

void RdCost::saveUnadjustedLambda()
{
  m_dLambda_unadjusted = m_dLambda;
  m_DistScaleUnadjusted = m_DistScale;
}

void RdCost::initLumaLevelToWeightTable()
{
  for (int i = 0; i < LUMA_LEVEL_TO_DQP_LUT_MAXSIZE; i++)
  {
    double x = i;
    double y;

    y = 0.015 * x - 1.5
        - 6;   // this is the Equation used to derive the luma qp LUT for HDR in MPEG HDR anchor3.2 (JCTCX-X1020)
    y = y < -3 ? -3 : (y > 6 ? 6 : y);

    m_lumaLevelToWeightPLUT[i] = pow(2.0, y / 3.0);      // or power(10, dQp/10)      they are almost equal
  }
}

void RdCost::initLumaLevelToWeightTableReshape()
{
  int lutSize = 1 << m_lumaBD;
  if (m_reshapeLumaLevelToWeightPLUT.empty())
  {
    m_reshapeLumaLevelToWeightPLUT.resize(lutSize, 1.0);
  }
  if (m_lumaLevelToWeightPLUT.empty())
  {
    m_lumaLevelToWeightPLUT.resize(lutSize, 1.0);
  }
  if (m_signalType == RESHAPE_SIGNAL_PQ)
  {
    for (int i = 0; i < (1 << m_lumaBD); i++)
    {
      double x = m_lumaBD < 10 ? i << (10 - m_lumaBD) : m_lumaBD > 10 ? i >> (m_lumaBD - 10) : i;
      double y;
      y = 0.015*x - 1.5 - 6;
      y = y < -3 ? -3 : (y > 6 ? 6 : y);
      m_reshapeLumaLevelToWeightPLUT[i] = pow(2.0, y / 3.0);
      m_lumaLevelToWeightPLUT[i] = m_reshapeLumaLevelToWeightPLUT[i];
    }
  }
}

void RdCost::updateReshapeLumaLevelToWeightTableChromaMD(std::vector<Pel>& ILUT)
{
  for (int i = 0; i < (1 << m_lumaBD); i++)
  {
    m_reshapeLumaLevelToWeightPLUT[i] = m_lumaLevelToWeightPLUT[ILUT[i]];
  }
}

void RdCost::restoreReshapeLumaLevelToWeightTable()
{
  for (int i = 0; i < (1 << m_lumaBD); i++)
  {
    m_reshapeLumaLevelToWeightPLUT.at(i) = m_lumaLevelToWeightPLUT.at(i);
  }
}

void RdCost::updateReshapeLumaLevelToWeightTable(SliceReshapeInfo &sliceReshape, Pel *wtTable, double cwt)
{
  if (m_signalType == RESHAPE_SIGNAL_SDR || m_signalType == RESHAPE_SIGNAL_HLG)
  {
    if (sliceReshape.getSliceReshapeModelPresentFlag())
    {
      double wBin = 1.0;
      double weight = 1.0;
      int histLens = (1 << m_lumaBD) / PIC_CODE_CW_BINS;

      for (int i = 0; i < PIC_CODE_CW_BINS; i++)
      {
        if ((i < sliceReshape.reshaperModelMinBinIdx) || (i > sliceReshape.reshaperModelMaxBinIdx))
        {
          weight = 1.0;
        }
        else
        {
          if (sliceReshape.reshaperModelBinCWDelta[i] == 1 || (sliceReshape.reshaperModelBinCWDelta[i] == -1 * histLens))
          {
            weight = wBin;
          }
          else
          {
            weight = (double)wtTable[i] / (double)histLens;
            weight = weight*weight;
          }
        }
        for (int j = 0; j < histLens; j++)
        {
          int ii = i*histLens + j;
          m_reshapeLumaLevelToWeightPLUT[ii] = weight;
        }
      }
      m_chromaWeight = cwt;
    }
    else
    {
      THROW("updateReshapeLumaLevelToWeightTable ERROR_!!");
    }
  }
  else
  {
    THROW("updateReshapeLumaLevelToWeightTable not support other signal types!!");
  }
}

Distortion RdCost::getWeightedMSE(int compIdx, const Pel org, const Pel cur, const uint32_t uiShift, const Pel orgLuma)
{
  Distortion distortionVal = 0;
  Intermediate_Int iTemp = org - cur;
  CHECK_( org<0, "");

  if (compIdx == COMPONENT_Y)
  {
    CHECK_(org != orgLuma, "");
  }
  // use luma to get weight
  double weight = 1.0;
  if (m_signalType == RESHAPE_SIGNAL_SDR || m_signalType == RESHAPE_SIGNAL_HLG)
  {
    if (compIdx == COMPONENT_Y)
    {
      weight = m_reshapeLumaLevelToWeightPLUT[orgLuma];
    }
    else
    {
      weight = m_chromaWeight;
    }
  }
  else
  {
    weight = m_reshapeLumaLevelToWeightPLUT[orgLuma];
  }
  int64_t fixedPTweight = (int64_t)(weight * (double)(1 << 16));
  Intermediate_Int mse = Intermediate_Int((fixedPTweight*(iTemp*iTemp) + (1 << 15)) >> 16);
  distortionVal = Distortion( mse >> uiShift);
  return distortionVal;
}

Distortion RdCost::xGetSSE_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );  // ignore it for now
  }
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma.buf;
  const int  iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n++ )
    {
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n  ], piCur[n  ], uiShift, piOrgLuma[n<<cShift]);
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;

    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}

Distortion RdCost::xGetSSE2_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 2, "" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam ); // ignore it for now
  }

  int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma           = rcDtParam.orgLuma.buf;
  const size_t  iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  for( ; iRows != 0; iRows-- )
  {
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[0  ], piCur[0  ], uiShift, piOrgLuma[size_t(0)<<cShift]);   // piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[1  ], piCur[1  ], uiShift, piOrgLuma[size_t(1)<<cShift]);   // piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    piOrg += iStrideOrg;
    piCur += iStrideCur;
    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}

Distortion RdCost::xGetSSE4_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 4, "" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam ); // ignore it for now
  }

        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma.buf;
  const size_t  iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  for( ; iRows != 0; iRows-- )
  {
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[0  ], piCur[0  ], uiShift, piOrgLuma[size_t(0)<<cShift]);   // piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[1  ], piCur[1  ], uiShift, piOrgLuma[size_t(1)<<cShift] );   // piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[2  ], piCur[2  ], uiShift, piOrgLuma[size_t(2)<<cShift] );   // piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[3  ], piCur[3  ], uiShift, piOrgLuma[size_t(3)<<cShift] );   // piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    piOrg += iStrideOrg;
    piCur += iStrideCur;
    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}

Distortion RdCost::xGetSSE8_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 8, "" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }

        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma.buf;
  const size_t  iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  for( ; iRows != 0; iRows-- )
  {
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[0  ], piCur[0  ], uiShift, piOrgLuma[0  ]);   // piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[1  ], piCur[1  ], uiShift, piOrgLuma[size_t(1)<<cShift  ]);  // piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[2  ], piCur[2  ], uiShift, piOrgLuma[size_t(2)<<cShift  ]);  //piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[3  ], piCur[3  ], uiShift, piOrgLuma[size_t(3)<<cShift  ]);  // piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[4  ], piCur[4  ], uiShift, piOrgLuma[size_t(4)<<cShift  ]);  // piOrg[4] - piCur[4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[5  ], piCur[5  ], uiShift, piOrgLuma[size_t(5)<<cShift  ]);  // piOrg[5] - piCur[5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[6  ], piCur[6  ], uiShift, piOrgLuma[size_t(6)<<cShift  ]);  // piOrg[6] - piCur[6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[7  ], piCur[7  ], uiShift, piOrgLuma[size_t(7)<<cShift  ]);  // piOrg[7] - piCur[7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    piOrg += iStrideOrg;
    piCur += iStrideCur;
    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}

Distortion RdCost::xGetSSE16_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 16, "" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma.buf;
  const size_t  iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;
  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  for( ; iRows != 0; iRows-- )
  {
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[0  ], piCur[0  ], uiShift, piOrgLuma[0  ]);  // piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[1  ], piCur[1  ], uiShift, piOrgLuma[size_t(1)<<cShift  ]);  //piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[2  ], piCur[2  ], uiShift, piOrgLuma[size_t(2)<<cShift  ]);  //piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[3  ], piCur[3  ], uiShift, piOrgLuma[size_t(3)<<cShift  ]);  //piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[4  ], piCur[4  ], uiShift, piOrgLuma[size_t(4)<<cShift  ]);  //piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[5  ], piCur[5  ], uiShift, piOrgLuma[size_t(5)<<cShift  ]);  //piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[6  ], piCur[6  ], uiShift, piOrgLuma[size_t(6)<<cShift  ]);  //piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[7  ], piCur[7  ], uiShift, piOrgLuma[size_t(7)<<cShift  ]);  //piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[8  ], piCur[8  ], uiShift, piOrgLuma[size_t(8)<<cShift  ]);  //piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[9  ], piCur[9  ], uiShift, piOrgLuma[size_t(9)<<cShift  ]);  //piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[10 ], piCur[10 ], uiShift, piOrgLuma[size_t(10)<<cShift  ]);  //piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[11 ], piCur[11 ], uiShift, piOrgLuma[size_t(11)<<cShift  ]);  //piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[12 ], piCur[12 ], uiShift, piOrgLuma[size_t(12)<<cShift  ]);  //piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[13 ], piCur[13 ], uiShift, piOrgLuma[size_t(13)<<cShift  ]);  //piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[14 ], piCur[14 ], uiShift, piOrgLuma[size_t(14)<<cShift  ]);  //piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[15 ], piCur[15 ], uiShift, piOrgLuma[size_t(15)<<cShift  ]);  //piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    piOrg += iStrideOrg;
    piCur += iStrideCur;

    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}

Distortion RdCost::xGetSSE16N_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma.buf;
  const size_t  iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;
  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n+=16 )
    {
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+0 ], piCur[n+0 ], uiShift, piOrgLuma[size_t(n+0)<<cShift ]);  // iTemp = piOrg[n+ 0] - piCur[n+ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+1 ], piCur[n+1 ], uiShift, piOrgLuma[size_t(n+1)<<cShift ]);  // iTemp = piOrg[n+ 1] - piCur[n+ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+2 ], piCur[n+2 ], uiShift, piOrgLuma[size_t(n+2)<<cShift ]);  // iTemp = piOrg[n+ 2] - piCur[n+ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+3 ], piCur[n+3 ], uiShift, piOrgLuma[size_t(n+3)<<cShift ]);  // iTemp = piOrg[n+ 3] - piCur[n+ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+4 ], piCur[n+4 ], uiShift, piOrgLuma[size_t(n+4)<<cShift ]);  // iTemp = piOrg[n+ 4] - piCur[n+ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+5 ], piCur[n+5 ], uiShift, piOrgLuma[size_t(n+5)<<cShift ]);  // iTemp = piOrg[n+ 5] - piCur[n+ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+6 ], piCur[n+6 ], uiShift, piOrgLuma[size_t(n+6)<<cShift ]);  // iTemp = piOrg[n+ 6] - piCur[n+ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+7 ], piCur[n+7 ], uiShift, piOrgLuma[size_t(n+7)<<cShift ]);  // iTemp = piOrg[n+ 7] - piCur[n+ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+8 ], piCur[n+8 ], uiShift, piOrgLuma[size_t(n+8)<<cShift ]);  // iTemp = piOrg[n+ 8] - piCur[n+ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+9 ], piCur[n+9 ], uiShift, piOrgLuma[size_t(n+9)<<cShift ]);  // iTemp = piOrg[n+ 9] - piCur[n+ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+10], piCur[n+10], uiShift, piOrgLuma[size_t(n+10)<<cShift ]);  // iTemp = piOrg[n+10] - piCur[n+10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+11], piCur[n+11], uiShift, piOrgLuma[size_t(n+11)<<cShift ]);  // iTemp = piOrg[n+11] - piCur[n+11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+12], piCur[n+12], uiShift, piOrgLuma[size_t(n+12)<<cShift]);  // iTemp = piOrg[n+12] - piCur[n+12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+13], piCur[n+13], uiShift, piOrgLuma[size_t(n+13)<<cShift ]);  // iTemp = piOrg[n+13] - piCur[n+13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+14], piCur[n+14], uiShift, piOrgLuma[size_t(n+14)<<cShift ]);  // iTemp = piOrg[n+14] - piCur[n+14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      uiSum += getWeightedMSE(rcDtParam.compID, piOrg[n+15], piCur[n+15], uiShift, piOrgLuma[size_t(n+15)<<cShift ]);  // iTemp = piOrg[n+15] - piCur[n+15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}

Distortion RdCost::xGetSSE32_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 32, "" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma.buf;
  const size_t  iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  for( ; iRows != 0; iRows-- )
  {
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[0 ], piCur[0 ], uiShift, piOrgLuma[size_t(0) ]);  // iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[1 ], piCur[1 ], uiShift, piOrgLuma[size_t(1)<<cShift ]);  // iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[2 ], piCur[2 ], uiShift, piOrgLuma[size_t(2)<<cShift ]);  // iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[3 ], piCur[3 ], uiShift, piOrgLuma[size_t(3)<<cShift ]);  // iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[4 ], piCur[4 ], uiShift, piOrgLuma[size_t(4)<<cShift ]);  // iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[5 ], piCur[5 ], uiShift, piOrgLuma[size_t(5)<<cShift ]);  // iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[6 ], piCur[6 ], uiShift, piOrgLuma[size_t(6)<<cShift ]);  // iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[7 ], piCur[7 ], uiShift, piOrgLuma[size_t(7)<<cShift ]);  // iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[8 ], piCur[8 ], uiShift, piOrgLuma[size_t(8)<<cShift ]);  // iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[9 ], piCur[9 ], uiShift, piOrgLuma[size_t(9)<<cShift ]);  // iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[10], piCur[10], uiShift, piOrgLuma[size_t(10)<<cShift ]);  // iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[11], piCur[11], uiShift, piOrgLuma[size_t(11)<<cShift ]);  // iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[12], piCur[12], uiShift, piOrgLuma[size_t(12)<<cShift ]);  // iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[13], piCur[13], uiShift, piOrgLuma[size_t(13)<<cShift ]);  // iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[14], piCur[14], uiShift, piOrgLuma[size_t(14)<<cShift ]);  // iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[15], piCur[15], uiShift, piOrgLuma[size_t(15)<<cShift ]);  // iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[16], piCur[16], uiShift, piOrgLuma[size_t(16)<<cShift ]);  //  iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[17], piCur[17], uiShift, piOrgLuma[size_t(17)<<cShift ]);  //  iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[18], piCur[18], uiShift, piOrgLuma[size_t(18)<<cShift ]);  //  iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[19], piCur[19], uiShift, piOrgLuma[size_t(19)<<cShift ]);  //  iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[20], piCur[20], uiShift, piOrgLuma[size_t(20)<<cShift ]);  //  iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[21], piCur[21], uiShift, piOrgLuma[size_t(21)<<cShift ]);  //  iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[22], piCur[22], uiShift, piOrgLuma[size_t(22)<<cShift ]);  //  iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[23], piCur[23], uiShift, piOrgLuma[size_t(23)<<cShift ]);  //  iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[24], piCur[24], uiShift, piOrgLuma[size_t(24)<<cShift ]);  //  iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[25], piCur[25], uiShift, piOrgLuma[size_t(25)<<cShift ]);  //  iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[26], piCur[26], uiShift, piOrgLuma[size_t(26)<<cShift ]);  //  iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[27], piCur[27], uiShift, piOrgLuma[size_t(27)<<cShift ]);  //  iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[28], piCur[28], uiShift, piOrgLuma[size_t(28)<<cShift ]);  //  iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[29], piCur[29], uiShift, piOrgLuma[size_t(29)<<cShift ]);  //  iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[30], piCur[30], uiShift, piOrgLuma[size_t(30)<<cShift ]);  //  iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[31], piCur[31], uiShift, piOrgLuma[size_t(31)<<cShift ]);  //  iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    piOrg += iStrideOrg;
    piCur += iStrideCur;
    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}

Distortion RdCost::xGetSSE64_WTD( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    CHECK_( rcDtParam.org.width != 64, "" );
    return RdCostWeightPrediction::xGetSSEw( rcDtParam );
  }
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma.buf;
  const size_t iStrideOrgLuma   = rcDtParam.orgLuma.stride;
  const size_t  cShift  = rcDtParam.cShiftX;
  const size_t  cShiftY = rcDtParam.cShiftY;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT((rcDtParam.bitDepth)) << 1;
  for( ; iRows != 0; iRows-- )
  {
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[0 ], piCur[0 ], uiShift, piOrgLuma[size_t(0) ]);  // iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[1 ], piCur[1 ], uiShift, piOrgLuma[size_t(1)<<cShift ]);  // iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[2 ], piCur[2 ], uiShift, piOrgLuma[size_t(2)<<cShift ]);  // iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[3 ], piCur[3 ], uiShift, piOrgLuma[size_t(3)<<cShift ]);  // iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[4 ], piCur[4 ], uiShift, piOrgLuma[size_t(4)<<cShift ]);  // iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[5 ], piCur[5 ], uiShift, piOrgLuma[size_t(5)<<cShift ]);  // iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[6 ], piCur[6 ], uiShift, piOrgLuma[size_t(6)<<cShift ]);  // iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[7 ], piCur[7 ], uiShift, piOrgLuma[size_t(7)<<cShift ]);  // iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[8 ], piCur[8 ], uiShift, piOrgLuma[size_t(8)<<cShift ]);  // iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[9 ], piCur[9 ], uiShift, piOrgLuma[size_t(9)<<cShift ]);  // iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[10], piCur[10], uiShift, piOrgLuma[size_t(10)<<cShift]);  // iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[11], piCur[11], uiShift, piOrgLuma[size_t(11)<<cShift]);  // iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[12], piCur[12], uiShift, piOrgLuma[size_t(12)<<cShift]);  // iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[13], piCur[13], uiShift, piOrgLuma[size_t(13)<<cShift]);  // iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[14], piCur[14], uiShift, piOrgLuma[size_t(14)<<cShift]);  // iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[15], piCur[15], uiShift, piOrgLuma[size_t(15)<<cShift]);  // iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[16], piCur[16], uiShift, piOrgLuma[size_t(16)<<cShift]);  //  iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[17], piCur[17], uiShift, piOrgLuma[size_t(17)<<cShift]);  //  iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[18], piCur[18], uiShift, piOrgLuma[size_t(18)<<cShift]);  //  iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[19], piCur[19], uiShift, piOrgLuma[size_t(19)<<cShift]);  //  iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[20], piCur[20], uiShift, piOrgLuma[size_t(20)<<cShift]);  //  iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[21], piCur[21], uiShift, piOrgLuma[size_t(21)<<cShift]);  //  iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[22], piCur[22], uiShift, piOrgLuma[size_t(22)<<cShift]);  //  iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[23], piCur[23], uiShift, piOrgLuma[size_t(23)<<cShift]);  //  iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[24], piCur[24], uiShift, piOrgLuma[size_t(24)<<cShift]);  //  iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[25], piCur[25], uiShift, piOrgLuma[size_t(25)<<cShift]);  //  iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[26], piCur[26], uiShift, piOrgLuma[size_t(26)<<cShift]);  //  iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[27], piCur[27], uiShift, piOrgLuma[size_t(27)<<cShift]);  //  iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[28], piCur[28], uiShift, piOrgLuma[size_t(28)<<cShift]);  //  iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[29], piCur[29], uiShift, piOrgLuma[size_t(29)<<cShift]);  //  iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[30], piCur[30], uiShift, piOrgLuma[size_t(30)<<cShift]);  //  iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[31], piCur[31], uiShift, piOrgLuma[size_t(31)<<cShift]);  //  iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[32], piCur[32], uiShift, piOrgLuma[size_t(32)<<cShift]);  // iTemp = piOrg[32] - piCur[32]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[33], piCur[33], uiShift, piOrgLuma[size_t(33)<<cShift]);  // iTemp = piOrg[33] - piCur[33]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[34], piCur[34], uiShift, piOrgLuma[size_t(34)<<cShift]);  // iTemp = piOrg[34] - piCur[34]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[35], piCur[35], uiShift, piOrgLuma[size_t(35)<<cShift]);  // iTemp = piOrg[35] - piCur[35]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[36], piCur[36], uiShift, piOrgLuma[size_t(36)<<cShift]);  // iTemp = piOrg[36] - piCur[36]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[37], piCur[37], uiShift, piOrgLuma[size_t(37)<<cShift]);  // iTemp = piOrg[37] - piCur[37]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[38], piCur[38], uiShift, piOrgLuma[size_t(38)<<cShift]);  // iTemp = piOrg[38] - piCur[38]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[39], piCur[39], uiShift, piOrgLuma[size_t(39)<<cShift]);  // iTemp = piOrg[39] - piCur[39]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[40], piCur[40], uiShift, piOrgLuma[size_t(40)<<cShift]);  // iTemp = piOrg[40] - piCur[40]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[41], piCur[41], uiShift, piOrgLuma[size_t(41)<<cShift]);  // iTemp = piOrg[41] - piCur[41]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[42], piCur[42], uiShift, piOrgLuma[size_t(42)<<cShift]);  // iTemp = piOrg[42] - piCur[42]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[43], piCur[43], uiShift, piOrgLuma[size_t(43)<<cShift]);  // iTemp = piOrg[43] - piCur[43]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[44], piCur[44], uiShift, piOrgLuma[size_t(44)<<cShift]);  // iTemp = piOrg[44] - piCur[44]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[45], piCur[45], uiShift, piOrgLuma[size_t(45)<<cShift]);  // iTemp = piOrg[45] - piCur[45]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[46], piCur[46], uiShift, piOrgLuma[size_t(46)<<cShift]);  // iTemp = piOrg[46] - piCur[46]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[47], piCur[47], uiShift, piOrgLuma[size_t(47)<<cShift]);  // iTemp = piOrg[47] - piCur[47]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[48], piCur[48], uiShift, piOrgLuma[size_t(48)<<cShift]);  // iTemp = piOrg[48] - piCur[48]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[49], piCur[49], uiShift, piOrgLuma[size_t(49)<<cShift]);  // iTemp = piOrg[49] - piCur[49]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[50], piCur[50], uiShift, piOrgLuma[size_t(50)<<cShift]);  // iTemp = piOrg[50] - piCur[50]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[51], piCur[51], uiShift, piOrgLuma[size_t(51)<<cShift]);  // iTemp = piOrg[51] - piCur[51]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[52], piCur[52], uiShift, piOrgLuma[size_t(52)<<cShift]);  // iTemp = piOrg[52] - piCur[52]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[53], piCur[53], uiShift, piOrgLuma[size_t(53)<<cShift]);  // iTemp = piOrg[53] - piCur[53]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[54], piCur[54], uiShift, piOrgLuma[size_t(54)<<cShift]);  // iTemp = piOrg[54] - piCur[54]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[55], piCur[55], uiShift, piOrgLuma[size_t(55)<<cShift]);  // iTemp = piOrg[55] - piCur[55]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[56], piCur[56], uiShift, piOrgLuma[size_t(56)<<cShift]);  // iTemp = piOrg[56] - piCur[56]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[57], piCur[57], uiShift, piOrgLuma[size_t(57)<<cShift]);  // iTemp = piOrg[57] - piCur[57]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[58], piCur[58], uiShift, piOrgLuma[size_t(58)<<cShift]);  // iTemp = piOrg[58] - piCur[58]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[59], piCur[59], uiShift, piOrgLuma[size_t(59)<<cShift]);  // iTemp = piOrg[59] - piCur[59]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[60], piCur[60], uiShift, piOrgLuma[size_t(60)<<cShift]);  // iTemp = piOrg[60] - piCur[60]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[61], piCur[61], uiShift, piOrgLuma[size_t(61)<<cShift]);  // iTemp = piOrg[61] - piCur[61]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[62], piCur[62], uiShift, piOrgLuma[size_t(62)<<cShift]);  // iTemp = piOrg[62] - piCur[62]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    uiSum += getWeightedMSE(rcDtParam.compID, piOrg[63], piCur[63], uiShift, piOrgLuma[size_t(63)<<cShift]);  // iTemp = piOrg[63] - piCur[63]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    piOrg += iStrideOrg;
    piCur += iStrideCur;

    piOrgLuma += iStrideOrgLuma<<cShiftY;
  }
  return ( uiSum );
}
#endif


Pel orgCopy[MAX_CU_SIZE * MAX_CU_SIZE];

#if _OPENMP
#pragma omp threadprivate(orgCopy)
#endif

Distortion RdCost::xGetMRHADs( const DistParam &rcDtParam )
{
  const Pel offset = rcDtParam.org.meanDiff( rcDtParam.cur );

  PelBuf modOrg( orgCopy, rcDtParam.org );

  modOrg.copyFrom( rcDtParam.org );
  modOrg.subtract( offset );

  DistParam modDistParam = rcDtParam;
  modDistParam.org = modOrg;

  return m_afpDistortFunc[DF_HAD]( modDistParam );
}

void RdCost::setDistParam( DistParam &rcDP, const CPelBuf &org, const Pel* piRefY, int iRefStride, const Pel* mask, int iMaskStride, int stepX, int iMaskStride2, int bitDepth, ComponentID compID)
{
  rcDP.bitDepth     = bitDepth;
  rcDP.compID       = compID;

  // set Original & Curr Pointer / Stride
  rcDP.org          = org;
  rcDP.cur.buf      = piRefY;
  rcDP.cur.stride   = iRefStride;

  // set Mask
  rcDP.mask         = mask;
  rcDP.maskStride   = iMaskStride;
  rcDP.stepX = stepX;
  rcDP.maskStride2 = iMaskStride2;

  // set Block Width / Height
  rcDP.cur.width    = org.width;
  rcDP.cur.height   = org.height;
  rcDP.maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();

  // set Cost function for motion estimation with Mask
  rcDP.distFunc = m_afpDistortFunc[ DF_SAD_WITH_MASK ];
}

Distortion RdCost::xGetSADwMask( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    return RdCostWeightPrediction::xGetSADw( rcDtParam );
  }

  const Pel* org           = rcDtParam.org.buf;
  const Pel* cur           = rcDtParam.cur.buf;
  const Pel* mask          = rcDtParam.mask;
  const int  cols           = rcDtParam.org.width;
  int        rows           = rcDtParam.org.height;
  const int  subShift       = rcDtParam.subShift;
  const int  subStep        = ( 1 << subShift);
  const int  strideCur      = rcDtParam.cur.stride * subStep;
  const int  strideOrg      = rcDtParam.org.stride * subStep;
  const int  strideMask     = rcDtParam.maskStride * subStep;
  const int  stepX = rcDtParam.stepX;
  const int  strideMask2 = rcDtParam.maskStride2;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  Distortion sum = 0;
  for (; rows != 0; rows -= subStep)
  {
    for (int n = 0; n < cols; n++)
    {
      sum += abs(org[n] - cur[n]) * *mask;
      mask += stepX;
    }
    org += strideOrg;
    cur += strideCur;
    mask += strideMask;
    mask += strideMask2;
  }
  sum <<= subShift;
  return (sum >> distortionShift );
}
//! \}

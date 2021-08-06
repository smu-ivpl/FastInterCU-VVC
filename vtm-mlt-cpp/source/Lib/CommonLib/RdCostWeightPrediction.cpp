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

/** \file     RdCostWeightPrediction.cpp
    \brief    RD cost computation class with Weighted-Prediction
*/

#include "RdCostWeightPrediction.h"

#include "RdCost.h"

#include <math.h>

static Distortion xCalcHADs2x2w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCurr, int iStrideOrg, int iStrideCur, int iStep );
static Distortion xCalcHADs4x4w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCurr, int iStrideOrg, int iStrideCur, int iStep );
static Distortion xCalcHADs8x8w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCurr, int iStrideOrg, int iStrideCur, int iStep );


// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------
/** get weighted SAD cost
 * \param pcDtParam
 * \returns Distortion
 */
Distortion RdCostWeightPrediction::xGetSADw( const DistParam &rcDtParam )
{
  const Pel            *piOrg      = rcDtParam.org.buf;
  const Pel            *piCur      = rcDtParam.cur.buf;
  const int             iCols      = rcDtParam.org.width;
  const int             iStrideCur = rcDtParam.cur.stride;
  const int             iStrideOrg = rcDtParam.org.stride;
  const ComponentID     compID     = rcDtParam.compID;

  CHECK_( compID >= MAX_NUM_COMPONENT, "Invalid component" );

  const WPScalingParam &wpCur      = rcDtParam.wpCur[compID];

  const int             w0         = wpCur.w;
  const int             offset     = wpCur.offset;
  const int             shift      = wpCur.shift;
  const int             round      = wpCur.round;
  const int distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  Distortion uiSum = 0;

#if !U0040_MODIFIED_WEIGHTEDPREDICTION_WITH_BIPRED_AND_CLIPPING
  for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n++ )
    {
      const Pel pred = ( (w0*piCur[n] + round) >> shift ) + offset ;

      uiSum += abs( piOrg[n] - pred );
    }
    if (rcDtParam.maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
    {
      return uiSum >> distortionShift;
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  //rcDtParam.compID = MAX_NUM_COMPONENT;  // reset for DEBUG (assert test)
#else
  ClpRng clpRng; // this just affects the cost
  clpRng.min = 0;
  clpRng.max = (1 << rcDtParam.bitDepth) - 1;

  // Default weight
  if (w0 == 1 << shift)
  {
    // no offset
    if (offset == 0)
    {
      for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
      {
        for (int n = 0; n < iCols; n++ )
        {
          uiSum += abs( piOrg[n] - piCur[n] );
        }
        if (rcDtParam.maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
        {
          return uiSum >> distortionShift;
        }
        piOrg += iStrideOrg;
        piCur += iStrideCur;
      }
    }
    else
    {
      // Lets not clip for the bipredictive case since clipping should be done after
      // combining both elements. Unfortunately the code uses the suboptimal "subtraction"
      // method, which is faster but introduces the clipping issue (therefore Bipred is suboptimal).
      if (rcDtParam.isBiPred)
      {
        for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
        {
          for (int n = 0; n < iCols; n++ )
          {
            uiSum += abs( piOrg[n] - (piCur[n] + offset) );
          }
          if (rcDtParam.maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
          {
            return uiSum >> distortionShift;
          }

          piOrg += iStrideOrg;
          piCur += iStrideCur;
        }
      }
      else
      {
        for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
        {
          for (int n = 0; n < iCols; n++ )
          {
            const Pel pred = ClipPel( piCur[n] + offset, clpRng);

            uiSum += abs( piOrg[n] - pred );
          }
          if (rcDtParam.maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
          {
            return uiSum >> distortionShift;
          }
          piOrg += iStrideOrg;
          piCur += iStrideCur;
        }
      }
    }
  }
  else
  {
    // Lets not clip for the bipredictive case since clipping should be done after
    // combining both elements. Unfortunately the code uses the suboptimal "subtraction"
    // method, which is faster but introduces the clipping issue (therefore Bipred is suboptimal).
    if (rcDtParam.isBiPred)
    {
      for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
      {
        for (int n = 0; n < iCols; n++ )
        {
          const Pel pred = ( (w0*piCur[n] + round) >> shift ) + offset ;
          uiSum += abs( piOrg[n] - pred );
        }
        if (rcDtParam.maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
        {
          return uiSum >> distortionShift;
        }

        piOrg += iStrideOrg;
        piCur += iStrideCur;
      }
    }
    else
    {
      if (offset == 0)
      {
        for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
        {
          for (int n = 0; n < iCols; n++ )
          {
            const Pel pred = ClipPel( (w0*piCur[n] + round) >> shift, clpRng);

            uiSum += abs( piOrg[n] - pred );
          }
          if (rcDtParam.maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
          {
            return uiSum >> distortionShift;
          }
          piOrg += iStrideOrg;
          piCur += iStrideCur;
        }
      }
      else
      {
        for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
        {
          for (int n = 0; n < iCols; n++ )
          {
            const Pel pred = ClipPel(((w0*piCur[n] + round) >> shift ) + offset,  clpRng) ;

            uiSum += abs( piOrg[n] - pred );
          }
          if (rcDtParam.maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
          {
            return uiSum >> distortionShift;
          }
          piOrg += iStrideOrg;
          piCur += iStrideCur;
        }
      }
    }
  }
  //rcDtParam.compIdx = MAX_NUM_COMPONENT;  // reset for DEBUG (assert test)
#endif

  return uiSum >> distortionShift;
}


// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------
/** get weighted SSD cost
 * \param pcDtParam
 * \returns Distortion
 */
Distortion RdCostWeightPrediction::xGetSSEw( const DistParam &rcDtParam )
{
  const Pel            *piOrg      = rcDtParam.org.buf;
  const Pel            *piCur      = rcDtParam.cur.buf;
  const int             iCols      = rcDtParam.org.width;
  const int             iStrideCur = rcDtParam.cur.stride;
  const int             iStrideOrg = rcDtParam.org.stride;
  const ComponentID     compID     = rcDtParam.compID;

  CHECK_( rcDtParam.subShift != 0, "Subshift not supported" ); // NOTE: what is this protecting?
  CHECK_( compID >= MAX_NUM_COMPONENT, "Invalid channel" );

  const WPScalingParam &wpCur           = rcDtParam.wpCur[compID];
  const int             w0              = wpCur.w;
  const int             offset          = wpCur.offset;
  const int             shift           = wpCur.shift;
  const int             round           = wpCur.round;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Distortion sum = 0;

#if !U0040_MODIFIED_WEIGHTEDPREDICTION_WITH_BIPRED_AND_CLIPPING
  for(int iRows = rcDtParam.org.height ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n++ )
    {
      const Pel pred     = ( (w0*piCur[n] + round) >> shift ) + offset ;
      const Pel residual = piOrg[n] - pred;
      sum += ( Distortion(residual) * Distortion(residual) ) >> distortionShift;
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  //rcDtParam.compIdx = MAX_NUM_COMPONENT; // reset for DEBUG (assert test)
#else
  if (rcDtParam.isBiPred)
  {
    for(int iRows = rcDtParam.org.height ; iRows != 0; iRows-- )
    {
      for (int n = 0; n < iCols; n++ )
      {
        const Pel pred     = ( (w0*piCur[n] + round) >> shift ) + offset ;
        const Pel residual = piOrg[n] - pred;
        sum += ( Distortion(residual) * Distortion(residual) ) >> distortionShift;
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }
  }
  else
  {
    ClpRng clpRng;  // this just affects the cost - fix this later
    clpRng.min = 0;
    clpRng.max = (1 << rcDtParam.bitDepth) - 1;

    for(int iRows = rcDtParam.org.height; iRows != 0; iRows-- )
    {
      for (int n = 0; n < iCols; n++ )
      {
        const Pel pred     = ClipPel(( (w0*piCur[n] + round) >> shift ) + offset, clpRng) ;
        const Pel residual = piOrg[n] - pred;
        sum += ( Distortion(residual) * Distortion(residual) ) >> distortionShift;
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }
  }

  //rcDtParam.compIdx = MAX_NUM_COMPONENT; // reset for DEBUG (assert test)
#endif

  return sum;
}


// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------
//! get weighted Hadamard cost for 2x2 block
Distortion xCalcHADs2x2w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iStep )
{
  const int round  = wpCur.round;
  const int shift  = wpCur.shift;
  const int offset = wpCur.offset;
  const int w0     = wpCur.w;

  Distortion satd  = 0;
  TCoeff     diff[4];
  TCoeff     m[4];

  Pel   pred;

  pred    = ( (w0*piCur[0*iStep             ] + round) >> shift ) + offset ;
  diff[0] = piOrg[0             ] - pred;
  pred    = ( (w0*piCur[1*iStep             ] + round) >> shift ) + offset ;
  diff[1] = piOrg[1             ] - pred;
  pred    = ( (w0*piCur[0*iStep + iStrideCur] + round) >> shift ) + offset ;
  diff[2] = piOrg[iStrideOrg    ] - pred;
  pred    = ( (w0*piCur[1*iStep + iStrideCur] + round) >> shift ) + offset ;
  diff[3] = piOrg[iStrideOrg + 1] - pred;

  m[0] = diff[0] + diff[2];
  m[1] = diff[1] + diff[3];
  m[2] = diff[0] - diff[2];
  m[3] = diff[1] - diff[3];

  satd += abs(m[0] + m[1]);
  satd += abs(m[0] - m[1]);
  satd += abs(m[2] + m[3]);
  satd += abs(m[2] - m[3]);

  return satd;
}


//! get weighted Hadamard cost for 4x4 block
Distortion xCalcHADs4x4w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iStep )
{
  const int round  = wpCur.round;
  const int shift  = wpCur.shift;
  const int offset = wpCur.offset;
  const int w0     = wpCur.w;

  Distortion satd = 0;
  TCoeff     diff[16];
  TCoeff     m[16];
  TCoeff     d[16];


  for(int k = 0; k < 16; k+=4 )
  {
    Pel pred;
    pred      = ( (w0*piCur[0*iStep] + round) >> shift ) + offset ;
    diff[k+0] = piOrg[0] - pred;
    pred      = ( (w0*piCur[1*iStep] + round) >> shift ) + offset ;
    diff[k+1] = piOrg[1] - pred;
    pred      = ( (w0*piCur[2*iStep] + round) >> shift ) + offset ;
    diff[k+2] = piOrg[2] - pred;
    pred      = ( (w0*piCur[3*iStep] + round) >> shift ) + offset ;
    diff[k+3] = piOrg[3] - pred;

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

  for (int k=0; k<16; ++k)
  {
    satd += abs(d[k]);
  }
  satd = ((satd+1)>>1);

  return satd;
}


//! get weighted Hadamard cost for 8x8 block
Distortion xCalcHADs8x8w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iStep )
{
  Distortion sad=0;
  TCoeff diff[64], m1[8][8], m2[8][8], m3[8][8];
  int iStep2 = iStep<<1;
  int iStep3 = iStep2 + iStep;
  int iStep4 = iStep3 + iStep;
  int iStep5 = iStep4 + iStep;
  int iStep6 = iStep5 + iStep;
  int iStep7 = iStep6 + iStep;
  const int round  = wpCur.round;
  const int shift  = wpCur.shift;
  const int offset = wpCur.offset;
  const int w0     = wpCur.w;

  Pel   pred;

  for(int k = 0; k < 64; k+=8 )
  {
    pred      = ( (w0*piCur[     0] + round) >> shift ) + offset ;
    diff[k+0] = piOrg[0] - pred;
    pred      = ( (w0*piCur[iStep ] + round) >> shift ) + offset ;
    diff[k+1] = piOrg[1] - pred;
    pred      = ( (w0*piCur[iStep2] + round) >> shift ) + offset ;
    diff[k+2] = piOrg[2] - pred;
    pred      = ( (w0*piCur[iStep3] + round) >> shift ) + offset ;
    diff[k+3] = piOrg[3] - pred;
    pred      = ( (w0*piCur[iStep4] + round) >> shift ) + offset ;
    diff[k+4] = piOrg[4] - pred;
    pred      = ( (w0*piCur[iStep5] + round) >> shift ) + offset ;
    diff[k+5] = piOrg[5] - pred;
    pred      = ( (w0*piCur[iStep6] + round) >> shift ) + offset ;
    diff[k+6] = piOrg[6] - pred;
    pred      = ( (w0*piCur[iStep7] + round) >> shift ) + offset ;
    diff[k+7] = piOrg[7] - pred;

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (int j=0; j < 8; j++)
  {
    const int jj = j << 3;
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
  for (int i=0; i < 8; i++)
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

  for (int j=0; j < 8; j++)
  {
    for (int i=0; i < 8; i++)
    {
      sad += (abs(m2[j][i]));
    }
  }

  sad=((sad+2)>>2);

  return sad;
}


//! get weighted Hadamard cost
Distortion RdCostWeightPrediction::xGetHADsw( const DistParam &rcDtParam )
{
  const Pel        *piOrg      = rcDtParam.org.buf;
  const Pel        *piCur      = rcDtParam.cur.buf;
  const int         iRows      = rcDtParam.org.height;
  const int         iCols      = rcDtParam.org.width;
  const int         iStrideCur = rcDtParam.cur.stride;
  const int         iStrideOrg = rcDtParam.org.stride;
  const int         iStep      = rcDtParam.step;
  const ComponentID compIdx    = rcDtParam.compID;
  CHECK_(compIdx>=MAX_NUM_COMPONENT, "Invalid component");
  const WPScalingParam &wpCur  = rcDtParam.wpCur[compIdx];

  Distortion uiSum = 0;


  if( ( iRows % 8 == 0) && (iCols % 8 == 0) )
  {
    const int iOffsetOrg = iStrideOrg<<3;
    const int iOffsetCur = iStrideCur<<3;
    for (int y=0; y<iRows; y+= 8 )
    {
      for (int x=0; x<iCols; x+= 8 )
      {
        uiSum += xCalcHADs8x8w( wpCur, &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else if( ( iRows % 4 == 0) && (iCols % 4 == 0) )
  {
    const int iOffsetOrg = iStrideOrg<<2;
    const int iOffsetCur = iStrideCur<<2;

    for (int y=0; y<iRows; y+= 4 )
    {
      for (int x=0; x<iCols; x+= 4 )
      {
        uiSum += xCalcHADs4x4w( wpCur, &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else
  {
    for (int y=0; y<iRows; y+=2 )
    {
      for (int x=0; x<iCols; x+=2 )
      {
        uiSum += xCalcHADs2x2w( wpCur, &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }
  }

  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

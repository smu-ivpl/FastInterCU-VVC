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

#ifndef __WEIGHTPREDICTION__
#define __WEIGHTPREDICTION__

#include "CommonDef.h"
#include "Slice.h"
#include "Unit.h"
#include "Buffer.h"

// forward declarations
struct WPScalingParam;

// ====================================================================================================================
// Class definition
// ====================================================================================================================
/// weighting prediction class
class WeightPrediction
{
public:
  WeightPrediction();

  void getWpScaling(Slice *slice, const int &iRefIdx0, const int &iRefIdx1, WPScalingParam *&wp0, WPScalingParam *&wp1,
                    const ComponentID maxNumComp = MAX_NUM_COMPONENT);

  void addWeightBi(             const CPelUnitBuf          &pcYuvSrc0,
                                const CPelUnitBuf          &pcYuvSrc1,
                                const ClpRngs              &clpRngs,
                                const WPScalingParam *const wp0,
                                const WPScalingParam *const wp1,
                                      PelUnitBuf           &rpcYuvDst,
                                const bool                  bRoundLuma = true,
                                const ComponentID           maxNumComp = MAX_NUM_COMPONENT
                                , bool                      lumaOnly = false
                                , bool                      chromaOnly = false
                                );

  void addWeightBiComponent(    const CPelUnitBuf          &pcYuvSrc0,
                                const CPelUnitBuf          &pcYuvSrc1,
                                const ClpRngs              &clpRngs,
                                const WPScalingParam *const wp0,
                                const WPScalingParam *const wp1,
                                      PelUnitBuf           &rpcYuvDst,
                                const bool                  bRoundLuma = true,
                                const ComponentID           Comp = COMPONENT_Y);

  void  addWeightUni(           const CPelUnitBuf          &pcYuvSrc0,
                                const ClpRngs              &clpRngs,
                                const WPScalingParam *const wp0,
                                      PelUnitBuf           &rpcYuvDst,
                                const ComponentID           maxNumComp = MAX_NUM_COMPONENT
                                , bool                      lumaOnly = false
                                , bool                      chromaOnly = false
                                );

  void  xWeightedPredictionUni( const PredictionUnit       &pu,
                                const CPelUnitBuf          &pcYuvSrc,
                                const RefPicList           &eRefPicList,
                                      PelUnitBuf           &pcYuvPred,
                                const int                   iRefIdx=-1,
                                const ComponentID           maxNumComp = MAX_NUM_COMPONENT
                                , bool                      lumaOnly = false
                                , bool                      chromaOnly = false
                                );

  void  xWeightedPredictionBi(  const PredictionUnit       &pu,
                                const CPelUnitBuf          &pcYuvSrc0,
                                const CPelUnitBuf          &pcYuvSrc1,
                                      PelUnitBuf           &pcYuvDst,
                                const ComponentID           maxNumComp = MAX_NUM_COMPONENT
                                , bool                      lumaOnly = false
                                , bool                      chromaOnly = false
                                );
};

#endif

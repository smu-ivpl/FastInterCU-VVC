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

/** \file     AQp.cpp
    \brief    class of picture which includes side information for encoder
*/

#include "AQp.h"
#include <float.h>

//! \ingroup EncoderLib
//! \{

/** Constructor
 */
AQpLayer::AQpLayer( int iWidth, int iHeight, uint32_t uiAQPartWidth, uint32_t uiAQPartHeight )
: m_uiAQPartWidth(uiAQPartWidth)
, m_uiAQPartHeight(uiAQPartHeight)
, m_uiNumAQPartInWidth((iWidth + uiAQPartWidth-1) / uiAQPartWidth)
, m_uiNumAQPartInHeight((iHeight + uiAQPartHeight-1) / uiAQPartHeight)
, m_dAvgActivity(0.0)
, m_acEncAQU( m_uiNumAQPartInWidth * m_uiNumAQPartInHeight, 0.0 )
{
}

/** Destructor
 */
AQpLayer::~AQpLayer()
{
}



/** Analyze source picture and compute local image characteristics used for QP adaptation
 * \param pcEPic Picture object to be analyzed
 * \return void
 */

void AQpPreanalyzer::preanalyze( Picture* pcEPic )
{
  const CPelBuf lumaPlane = pcEPic->getOrigBuf().Y();
  const int iWidth  = lumaPlane.width;
  const int iHeight = lumaPlane.height;
  const int iStride = lumaPlane.stride;

  for ( uint32_t d = 0; d < pcEPic->aqlayer.size(); d++ )
  {
    const Pel* pLineY = lumaPlane.bufAt( 0, 0);
    AQpLayer* pcAQLayer = pcEPic->aqlayer[d];
    const uint32_t uiAQPartWidth = pcAQLayer->getAQPartWidth();
    const uint32_t uiAQPartHeight = pcAQLayer->getAQPartHeight();
    double* pcAQU = &pcAQLayer->getQPAdaptationUnit()[0];

    double dSumAct = 0.0;
    for ( uint32_t y = 0; y < iHeight; y += uiAQPartHeight )
    {
      const uint32_t uiCurrAQPartHeight = std::min(uiAQPartHeight, iHeight-y);
      for ( uint32_t x = 0; x < iWidth; x += uiAQPartWidth, pcAQU++ )
      {
        const uint32_t uiCurrAQPartWidth = std::min(uiAQPartWidth, iWidth-x);
        const Pel* pBlkY = &pLineY[x];
        uint64_t uiSum[4] = {0, 0, 0, 0};
        uint64_t uiSumSq[4] = {0, 0, 0, 0};
        uint32_t by = 0;
        for ( ; by < uiCurrAQPartHeight>>1; by++ )
        {
          uint32_t bx = 0;
          for ( ; bx < uiCurrAQPartWidth>>1; bx++ )
          {
            uiSum  [0] += pBlkY[bx];
            uiSumSq[0] += pBlkY[bx] * pBlkY[bx];
          }
          for ( ; bx < uiCurrAQPartWidth; bx++ )
          {
            uiSum  [1] += pBlkY[bx];
            uiSumSq[1] += pBlkY[bx] * pBlkY[bx];
          }
          pBlkY += iStride;
        }
        for ( ; by < uiCurrAQPartHeight; by++ )
        {
          uint32_t bx = 0;
          for ( ; bx < uiCurrAQPartWidth>>1; bx++ )
          {
            uiSum  [2] += pBlkY[bx];
            uiSumSq[2] += pBlkY[bx] * pBlkY[bx];
          }
          for ( ; bx < uiCurrAQPartWidth; bx++ )
          {
            uiSum  [3] += pBlkY[bx];
            uiSumSq[3] += pBlkY[bx] * pBlkY[bx];
          }
          pBlkY += iStride;
        }

        CHECK_((uiCurrAQPartWidth&1)!=0,  "Odd part width unsupported");
        CHECK_((uiCurrAQPartHeight&1)!=0, "Odd part height unsupported");
        const uint32_t pixelWidthOfQuadrants  = uiCurrAQPartWidth >>1;
        const uint32_t pixelHeightOfQuadrants = uiCurrAQPartHeight>>1;
        const uint32_t numPixInAQPart         = pixelWidthOfQuadrants * pixelHeightOfQuadrants;

        double dMinVar = DBL_MAX;
        if (numPixInAQPart!=0)
        {
          for ( int i=0; i<4; i++)
          {
            const double dAverage = double(uiSum[i]) / numPixInAQPart;
            const double dVariance = double(uiSumSq[i]) / numPixInAQPart - dAverage * dAverage;
            dMinVar = std::min(dMinVar, dVariance);
          }
        }
        else
        {
          dMinVar = 0.0;
        }
        const double dActivity = 1.0 + dMinVar;
        *pcAQU = dActivity;
        dSumAct += dActivity;
      }
      pLineY += iStride * uiCurrAQPartHeight;
    }

    const double dAvgAct = dSumAct / (pcAQLayer->getNumAQPartInWidth() * pcAQLayer->getNumAQPartInHeight());
    pcAQLayer->setAvgActivity( dAvgAct );
  }
}



//! \}


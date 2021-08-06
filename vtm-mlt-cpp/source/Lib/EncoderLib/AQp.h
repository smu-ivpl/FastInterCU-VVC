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

/** \file     AQp.h
    \brief    class of picture which includes side information for encoder (header)
*/

#ifndef __AQP__
#define __AQP__

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Local image characteristics for CUs on a specific depth
class AQpLayer
{
private:
  uint32_t                  m_uiAQPartWidth;
  uint32_t                  m_uiAQPartHeight;
  uint32_t                  m_uiNumAQPartInWidth;
  uint32_t                  m_uiNumAQPartInHeight;
  double                m_dAvgActivity;
  std::vector<double>   m_acEncAQU;

public:
  AQpLayer( int iWidth, int iHeight, uint32_t uiAQPartWidth, uint32_t uiAQPartHeight );
  virtual ~AQpLayer();

  uint32_t                   getAQPartWidth()        { return m_uiAQPartWidth;       }
  uint32_t                   getAQPartHeight()       { return m_uiAQPartHeight;      }
  uint32_t                   getNumAQPartInWidth()   { return m_uiNumAQPartInWidth;  }
  uint32_t                   getNumAQPartInHeight()  { return m_uiNumAQPartInHeight; }
  uint32_t                   getAQPartStride()       { return m_uiNumAQPartInWidth;  }
  std::vector<double>&   getQPAdaptationUnit()   { return m_acEncAQU;           }
  double getActivity( const Position& pos)
  {
    uint32_t uiAQUPosX = pos.x / m_uiAQPartWidth;
    uint32_t uiAQUPosY = pos.y / m_uiAQPartHeight;
    return m_acEncAQU[uiAQUPosY * m_uiNumAQPartInWidth + uiAQUPosX];
  }

  double                 getAvgActivity()        { return m_dAvgActivity;        }

  void                   setAvgActivity( double d )  { m_dAvgActivity = d; }
};

/// Source picture analyzer class
class AQpPreanalyzer
{
protected:
  AQpPreanalyzer() {}
  virtual ~AQpPreanalyzer() {}
public:
  static void preanalyze( Picture* picture );
};

//! \}

#endif // __ENCPIC__

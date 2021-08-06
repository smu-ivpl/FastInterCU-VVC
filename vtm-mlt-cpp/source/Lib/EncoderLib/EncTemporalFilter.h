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

/** \file     EncTemporalFilter.h
\brief    EncTemporalFilter class (header)
*/

#ifndef __TEMPORAL_FILTER__
#define __TEMPORAL_FILTER__
#include "EncLib.h"
#include "CommonLib/Buffer.h"
#include <sstream>
#include <map>
#include <deque>


//! \ingroup EncoderLib
//! \{

struct MotionVector
{
  int x, y;
  int error;
  MotionVector() : x(0), y(0), error(INT_LEAST32_MAX) {}
  void set(int vectorX, int vectorY, int errorValue) { x = vectorX; y = vectorY; error = errorValue; }
};

template <class T>
struct Array2D
{
private:
  int m_width, m_height;
  std::vector< T > v;
public:
  Array2D() : m_width(0), m_height(0), v() { }
  Array2D(int width, int height, const T& value=T()) : m_width(0), m_height(0), v() { allocate(width, height, value); }

  void allocate(int width, int height, const T& value=T())
  {
    m_width=width;
    m_height=height;
    v.resize(std::size_t(m_width*m_height), value);
  }

  T& get(int x, int y)
  {
    assert(x<m_width && y<m_height);
    return v[y*m_width+x];
  }

  const T& get(int x, int y) const
  {
    assert(x<m_width && y<m_height);
    return v[y*m_width+x];
  }
};

struct TemporalFilterSourcePicInfo
{
  TemporalFilterSourcePicInfo() : picBuffer(), mvs(), origOffset(0) { }
  PelStorage            picBuffer;
  Array2D<MotionVector> mvs;
  int                   origOffset;
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class EncTemporalFilter
{
public:
  EncTemporalFilter();
  ~EncTemporalFilter() {}

  void init(const int frameSkip,
    const int inputBitDepth[MAX_NUM_CHANNEL_TYPE],
    const int msbExtendedBitDepth[MAX_NUM_CHANNEL_TYPE],
    const int internalBitDepth[MAX_NUM_CHANNEL_TYPE],
    const int width,
    const int height,
    const int *pad,
    const bool rec709,
    const std::string &filename,
    const ChromaFormat inputChroma,
    const InputColourSpaceConversion colorSpaceConv,
    const int qp,
    const std::map<int, double> &temporalFilterStrengths,
    const bool gopBasedTemporalFilterFutureReference);

  bool filter(PelStorage *orgPic, int frame);

private:
  // Private static member variables
  static const int m_range;
  static const double m_chromaFactor;
  static const double m_sigmaMultiplier;
  static const double m_sigmaZeroPoint;
  static const int m_motionVectorFactor;
  static const int m_padding;
  static const int m_interpolationFilter[16][8];
  static const double m_refStrengths[3][2];

  // Private member variables
  int m_FrameSkip;
  std::string m_inputFileName;
  int m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];
  int m_MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE];
  int m_internalBitDepth[MAX_NUM_CHANNEL_TYPE];
  ChromaFormat m_chromaFormatIDC;
  int m_sourceWidth;
  int m_sourceHeight;
  int m_QP;
  std::map<int, double> m_temporalFilterStrengths;
  int m_pad[2];
  bool m_clipInputVideoToRec709Range;
  InputColourSpaceConversion m_inputColourSpaceConvert;
  Area m_area;
  bool m_gopBasedTemporalFilterFutureReference;

  // Private functions
  void subsampleLuma(const PelStorage &input, PelStorage &output, const int factor = 2) const;
  int motionErrorLuma(const PelStorage &orig, const PelStorage &buffer, const int x, const int y, int dx, int dy, const int bs, const int besterror) const;
  void motionEstimationLuma(Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int bs,
    const Array2D<MotionVector> *previous=0, const int factor = 1, const bool doubleRes = false) const;
  void motionEstimation(Array2D<MotionVector> &mvs, const PelStorage &orgPic, const PelStorage &buffer, const PelStorage &origSubsampled2, const PelStorage &origSubsampled4) const;

  void bilateralFilter(const PelStorage &orgPic, const std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo, PelStorage &newOrgPic, double overallStrength) const;
  void applyMotion(const Array2D<MotionVector> &mvs, const PelStorage &input, PelStorage &output) const;
}; // END CLASS DEFINITION EncTemporalFilter

   //! \}


#endif // __TEMPORAL_FILTER__

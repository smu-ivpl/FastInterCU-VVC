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

/**
 * \file
 * \brief Implementation of InterpolationFilter class
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"
#include "../Rom.h"
#include "../InterpolationFilter.h"

//#include "../ChromaFormat.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86

#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

// because of sub pu atmvp and tripple split
#define JEM_UNALIGNED_DST 1

// ===========================
// Full-pel copy 8-bit/16-bit
// ===========================
template<typename Tsrc, int N, bool isFirst, bool isLast>
static void fullPelCopySSE( const ClpRng& clpRng, const void*_src, int srcStride, int16_t *dst, int dstStride, int width, int height )
{
  Tsrc* src = (Tsrc*)_src;

  int headroom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int headroom_offset = 1 << ( headroom - 1 );
  int offset   = IF_INTERNAL_OFFS;
  __m128i voffset  = _mm_set1_epi16( offset );
  __m128i voffset_headroom  = _mm_set1_epi16( headroom_offset );

  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vsrc, vsum;

  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=N )
    {
      _mm_prefetch( (const char*)src+2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+( width>>1 ) + 2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+width-1 + 2*srcStride, _MM_HINT_T0 );
      for( int i=0; i<N; i+=8 )
      {
        if( sizeof( Tsrc )==1 )
        {
          vsrc = _mm_cvtepu8_epi16( _mm_lddqu_si128( ( __m128i const * )&src[col+i] ) );
        }
        else
        {
          vsrc = _mm_lddqu_si128( ( __m128i const * )&src[col+i] );
        }

        if( isFirst == isLast )
        {
          vsum =  _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsrc ) );
        }
        else if( isFirst )
        {
          vsrc = _mm_slli_epi16( vsrc, headroom );
          vsum = _mm_sub_epi16( vsrc, voffset );
        }
        else
        {
          vsrc = _mm_add_epi16( vsrc, voffset );
          vsrc = _mm_add_epi16( vsrc, voffset_headroom );
          vsrc = _mm_srai_epi16( vsrc, headroom );
          vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsrc ) );
        }
#if JEM_UNALIGNED_DST
        _mm_storeu_si128( ( __m128i * )&dst[col+i], vsum );
#else
        _mm_store_si128( ( __m128i * )&dst[col+i], vsum );
#endif
      }
    }
    src += srcStride;
    dst += dstStride;
  }
}


template<typename Tsrc, int N, bool isFirst, bool isLast>
static void fullPelCopyAVX2( const ClpRng& clpRng, const void*_src, int srcStride, short *dst, int dstStride, int width, int height )
{
#ifdef USE_AVX2
  Tsrc* src = (Tsrc*)_src;

  int headroom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int offset   = 1 << ( headroom - 1 );
  int internal_offset = IF_INTERNAL_OFFS;

  __m256i vinternal_offset = _mm256_set1_epi16( internal_offset );
  __m256i vheadroom_offset = _mm256_set1_epi16( offset );

  __m256i vibdimin = _mm256_set1_epi16( clpRng.min );
  __m256i vibdimax = _mm256_set1_epi16( clpRng.max );
  __m256i vsrc, vsum;


  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=N )
    {
      _mm_prefetch( (const char*)( src+3*srcStride ), _MM_HINT_T0 );
      _mm_prefetch( (const char*)( src+( width>>1 ) + 3*srcStride ), _MM_HINT_T0 );
      _mm_prefetch( (const char*)( src+width-1 + 3*srcStride ), _MM_HINT_T0 );
      for( int i=0; i<N; i+=16 )
      {
        if( sizeof( Tsrc )==1 )
        {
          vsrc = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )&src[col+i] ) );
        }
        else
        {
          vsrc = _mm256_lddqu_si256( ( const __m256i * )&src[col+i] );
        }

        if( isFirst == isLast )
        {
          vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsrc ) );
        }
        else if( isFirst )
        {
          vsrc = _mm256_slli_epi16( vsrc, headroom );
          vsum = _mm256_sub_epi16( vsrc, vinternal_offset );
        }
        else
        {
          vsrc = _mm256_add_epi16( vsrc, vinternal_offset );
          vsrc = _mm256_add_epi16( vsrc, vheadroom_offset );
          vsrc = _mm256_srai_epi16( vsrc, headroom );
          vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsrc ) );
        }
#if JEM_UNALIGNED_DST
        _mm256_storeu_si256( ( __m256i * )&dst[col+i], vsum );
#else
        _mm256_store_si256( ( __m256i * )&dst[col+i], vsum );
#endif

      }
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}


template<X86_VEXT vext, bool isFirst, bool isLast>
static void simdFilterCopy( const ClpRng& clpRng, const Pel* src, int srcStride, int16_t* dst, int dstStride, int width, int height, bool biMCForDMVR)
{
  { //Scalar
    InterpolationFilter::filterCopy<isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
}


// SIMD interpolation horizontal, block width modulo 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM4( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  _mm_prefetch( (const char*)src + srcStride, _MM_HINT_T0 );
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vcoeffh = _mm_lddqu_si128( ( __m128i const * )coeff );

  __m128i vzero, vshufc0, vshufc1;
  __m128i vsum;

  if( N != 8 ){
    vcoeffh = _mm_shuffle_epi32( vcoeffh, 0x44 );
    vzero = _mm_setzero_si128();
    vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
  }

  vzero = _mm_setzero_si128();

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );
    for( int col = 0; col < width; col += 4 )
    {
      if( N == 8 )
      {
        __m128i vtmp[2];
        for( int i = 0; i < 4; i += 2 )
        {
          __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col + i] );
          __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col + i + 1] );
          vsrc0 = _mm_madd_epi16( vsrc0, vcoeffh );
          vsrc1 = _mm_madd_epi16( vsrc1, vcoeffh );
          vtmp[i / 2] = _mm_hadd_epi32( vsrc0, vsrc1 );
        }
        vsum = _mm_hadd_epi32( vtmp[0], vtmp[1] );
      }
      else
      {
        __m128i vtmp0, vtmp1;
        __m128i vsrc = _mm_lddqu_si128( ( __m128i const * )&src[col] );
        vtmp0 = _mm_shuffle_epi8( vsrc, vshufc0 );
        vtmp1 = _mm_shuffle_epi8( vsrc, vshufc1 );

        vtmp0 = _mm_madd_epi16( vtmp0, vcoeffh );
        vtmp1 = _mm_madd_epi16( vtmp1, vcoeffh );
        vsum = _mm_hadd_epi32( vtmp0, vtmp1 );
      }

      {
        vsum = _mm_add_epi32( vsum, voffset );
        vsum = _mm_srai_epi32( vsum, shift );
        vsum = _mm_packs_epi32( vsum, vzero );
      }

      if( shiftBack )
      { //clip
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }
      _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
}


// SIMD interpolation horizontal, block width modulo 8
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM8( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  const int filterSpan = ( N - 1 );
  _mm_prefetch( (const char*)src + srcStride, _MM_HINT_T0 );
  _mm_prefetch( (const char*)src + ( width >> 1 ) + srcStride, _MM_HINT_T0 );
  _mm_prefetch( (const char*)src + width + filterSpan + srcStride, _MM_HINT_T0 );

  __m128i voffset  = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vcoeffh  = _mm_lddqu_si128( ( __m128i const * )coeff );

  __m128i vshufc0, vshufc1;
  __m128i vsum, vsuma, vsumb;

  if( N != 8 ){
    vcoeffh = _mm_shuffle_epi32( vcoeffh, 0x44 );
    vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
  }

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );
    _mm_prefetch( (const char*)src + ( width >> 1 ) + 2 * srcStride, _MM_HINT_T0 );
    _mm_prefetch( (const char*)src + width + filterSpan + 2 * srcStride, _MM_HINT_T0 );
    for( int col = 0; col < width; col += 8 )
    {
      if( N == 8 )
      {
        __m128i vtmp[4];
        for( int i = 0; i < 8; i += 2 ){
          __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col + i] );
          __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col + i + 1] );
          vsrc0 = _mm_madd_epi16( vsrc0, vcoeffh );
          vsrc1 = _mm_madd_epi16( vsrc1, vcoeffh );
          vtmp[i / 2] = _mm_hadd_epi32( vsrc0, vsrc1 );
        }
        vsuma = _mm_hadd_epi32( vtmp[0], vtmp[1] );
        vsumb = _mm_hadd_epi32( vtmp[2], vtmp[3] );
      }
      else
      {
        __m128i vtmp00, vtmp01, vtmp10, vtmp11;
        __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col] );
        __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col + 4] );
        vtmp00 = _mm_shuffle_epi8( vsrc0, vshufc0 );
        vtmp01 = _mm_shuffle_epi8( vsrc0, vshufc1 );
        vtmp10 = _mm_shuffle_epi8( vsrc1, vshufc0 );
        vtmp11 = _mm_shuffle_epi8( vsrc1, vshufc1 );

        vtmp00 = _mm_madd_epi16( vtmp00, vcoeffh );
        vtmp01 = _mm_madd_epi16( vtmp01, vcoeffh );
        vtmp10 = _mm_madd_epi16( vtmp10, vcoeffh );
        vtmp11 = _mm_madd_epi16( vtmp11, vcoeffh );

        vsuma = _mm_hadd_epi32( vtmp00, vtmp01 );
        vsumb = _mm_hadd_epi32( vtmp10, vtmp11 );
      }

      {
        vsuma = _mm_add_epi32( vsuma, voffset );
        vsumb = _mm_add_epi32( vsumb, voffset );

        vsuma = _mm_srai_epi32( vsuma, shift );
        vsumb = _mm_srai_epi32( vsumb, shift );

        vsum = _mm_packs_epi32( vsuma, vsumb );
      }

      if( shiftBack ){ //clip
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }
#if JEM_UNALIGNED_DST
      _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
#else
      _mm_store_si128( ( __m128i * )&dst[col], vsum );
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM8_AVX2( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  const int filterSpan =/* bChromaIntl ? 2* ( N-1 ) : */( N-1 );
  _mm_prefetch( (const char*)( src+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+srcStride ), _MM_HINT_T0 );

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max );

  __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                    0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
  __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                    0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );

  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

//   __m256i vw0, vw1, voffsetW, vwp;
  __m256i vsum;

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+width+filterSpan + 2*srcStride ), _MM_HINT_T0 );
    for( int col = 0; col < width; col+=8 )
    {
      if( N==8 )
      {
        __m128i vsrc[3];
        for( int i=0; i<3; i++ )
        {
          vsrc[i] = _mm_loadu_si128( ( const __m128i * )&src[col+i*4] );
        }
        vsum = _mm256_setzero_si256();
        for( int i=0; i<2; i++ )
        {
          __m256i vsrc0 = _mm256_castsi128_si256( vsrc[i] );
          vsrc0 = _mm256_inserti128_si256( vsrc0, vsrc[i+1], 1 );
          __m256i vsrca0 = _mm256_shuffle_epi8( vsrc0, vshuf0 );
          __m256i vsrca1 = _mm256_shuffle_epi8( vsrc0, vshuf1 );
          vsum  = _mm256_add_epi32( vsum, _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[2*i] ), _mm256_madd_epi16( vsrca1, vcoeff[2*i+1] ) ) );
        }
      }
      else
      {
        __m256i vtmp02, vtmp13;
        {
          __m256i vsrc = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* )&src[col] ) );
          vsrc = _mm256_inserti128_si256( vsrc, _mm_loadu_si128( ( const __m128i* )&src[col+4] ), 1 );

          vtmp02 = _mm256_shuffle_epi8( vsrc, vshuf0 );
          vtmp13 = _mm256_shuffle_epi8( vsrc, vshuf1 );
        }
        vtmp02 = _mm256_madd_epi16( vtmp02, vcoeff[0] );
        vtmp13 = _mm256_madd_epi16( vtmp13, vcoeff[1] );
        vsum = _mm256_add_epi32( vtmp02, vtmp13 );
      }
      {
        vsum = _mm256_add_epi32( vsum, voffset );
        vsum = _mm256_srai_epi32( vsum, shift );
      }

      __m128i vsump = _mm256_castsi256_si128( _mm256_permute4x64_epi64( _mm256_packs_epi32( vsum, vsum ), 0x88 ) );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }
#if JEM_UNALIGNED_DST
      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );
#else
      _mm_store_si128( ( __m128i * )&dst[col], vsump );
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM16_AVX2( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  const int filterSpan = ( N-1 );
  _mm_prefetch( (const char*)( src+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+3*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+3*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+3*srcStride ), _MM_HINT_T0 );

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m256i vibdimin   = _mm256_set1_epi16( clpRng.min );
  __m256i vibdimax   = _mm256_set1_epi16( clpRng.max );
  __m256i vzero      = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;

  __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                    0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
  __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                    0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+width+filterSpan + 2*srcStride ), _MM_HINT_T0 );

//     _mm_prefetch( (const char*)( src+4*srcStride ), _MM_HINT_T0 );
//     _mm_prefetch( (const char*)( src+( width>>1 )+4*srcStride ), _MM_HINT_T0 );
//     _mm_prefetch( (const char*)( src+width+filterSpan + 4*srcStride ), _MM_HINT_T0 );
    for( int col = 0; col < width; col+=16 )
    {
      __m256i vsrc[3];
      for( int i=0; i<3; i++ )
      {
        vsrc[i] = _mm256_lddqu_si256( ( const __m256i * )&src[col+i*4] );
      }
      if( N==8 )
      {
        vsuma = vsumb = vzero;
        for( int i=0; i<2; i++ )
        {
          __m256i vsrca0 = _mm256_shuffle_epi8( vsrc[i], vshuf0 );
          __m256i vsrca1 = _mm256_shuffle_epi8( vsrc[i], vshuf1 );
          __m256i vsrcb0 = _mm256_shuffle_epi8( vsrc[i+1], vshuf0 );
          __m256i vsrcb1 = _mm256_shuffle_epi8( vsrc[i+1], vshuf1 );
          vsuma  = _mm256_add_epi32( vsuma, _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[2*i] ), _mm256_madd_epi16( vsrca1, vcoeff[2*i+1] ) ) );
          vsumb  = _mm256_add_epi32( vsumb, _mm256_add_epi32( _mm256_madd_epi16( vsrcb0, vcoeff[2*i] ), _mm256_madd_epi16( vsrcb1, vcoeff[2*i+1] ) ) );
        }
      }
      else
      {
        __m256i vtmp00, vtmp01, vtmp10, vtmp11;
        {
          vtmp00 = _mm256_shuffle_epi8( vsrc[0], vshuf0 );
          vtmp01 = _mm256_shuffle_epi8( vsrc[0], vshuf1 );
          vtmp10 = _mm256_shuffle_epi8( vsrc[1], vshuf0 );
          vtmp11 = _mm256_shuffle_epi8( vsrc[1], vshuf1 );
        }
        vtmp00 = _mm256_madd_epi16( vtmp00, vcoeff[0] );
        vtmp01 = _mm256_madd_epi16( vtmp01, vcoeff[1] );
        vtmp10 = _mm256_madd_epi16( vtmp10, vcoeff[0] );
        vtmp11 = _mm256_madd_epi16( vtmp11, vcoeff[1] );

        vsuma = _mm256_add_epi32( vtmp00, vtmp01 );
        vsumb = _mm256_add_epi32( vtmp10, vtmp11 );
      }

      {
        vsuma = _mm256_add_epi32( vsuma, voffset );
        vsumb = _mm256_add_epi32( vsumb, voffset );
        vsuma = _mm256_srai_epi32( vsuma, shift );
        vsumb = _mm256_srai_epi32( vsumb, shift );
        vsum  = _mm256_packs_epi32( vsuma, vsumb );
      }

      if( shiftBack )
      { //clip
        vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
      }
#if JEM_UNALIGNED_DST
      _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );
#else
      _mm256_store_si256( ( __m256i * )&dst[col], vsum );
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM4( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  const int16_t *srcOrig = src;
  int16_t *dstOrig = dst;

  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );

  __m128i vsum;

  for( int i = 0; i < N; i += 2 )
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16( _mm_set1_epi16( coeff[i] ), _mm_set1_epi16( coeff[i + 1] ) );
  }

  for( int col = 0; col < width; col += 4 )
  {
    for( int i = 0; i < N - 1; i++ )
    {
      vsrc[i] = _mm_loadl_epi64( ( __m128i const * )&src[col + i * srcStride] );
    }
    for( int row = 0; row < height; row++ )
    {
      vsrc[N - 1] = _mm_loadl_epi64( ( __m128i const * )&src[col + ( N - 1 ) * srcStride] );

      vsum = vzero;
      for( int i = 0; i < N; i += 2 )
      {
        __m128i vsrc0 = _mm_unpacklo_epi16( vsrc[i], vsrc[i + 1] );
        vsum = _mm_add_epi32( vsum, _mm_madd_epi16( vsrc0, vcoeff[i / 2] ) );
      }

      for( int i = 0; i < N - 1; i++ )
      {
        vsrc[i] = vsrc[i + 1];
      }

//       if( shiftBack )
      {
        vsum = _mm_add_epi32( vsum, voffset );
        vsum = _mm_srai_epi32( vsum, shift );
        vsum = _mm_packs_epi32( vsum, vzero );
      }
//       else
//       {
//         vsum = _mm_srai_epi32( vsum, shift );       // val = sum >> shift;
//         vsum = _mm_packs_epi32( vsum, vzero );
//       }
      if( shiftBack ) //clip
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }

      _mm_storel_epi64( ( __m128i* )&dst[col], vsum );

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
//   src -= ( N / 2 - 1 ) * srcStride;
  const Pel *srcOrig = src;
  int16_t *dstOrig = dst;

  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );

  __m128i vsum, vsuma, vsumb;

  for( int i = 0; i < N; i += 2 )
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16( _mm_set1_epi16( coeff[i] ), _mm_set1_epi16( coeff[i + 1] ) );
  }

  for( int col = 0; col < width; col += 8 )
  {
    for( int i = 0; i < N - 1; i++ )
    {
      vsrc[i] = _mm_lddqu_si128( ( __m128i const * )&src[col + i * srcStride] );
    }

    for( int row = 0; row < height; row++ )
    {
      vsrc[N - 1] = _mm_lddqu_si128( ( __m128i const * )&src[col + ( N - 1 ) * srcStride] );
      vsuma = vsumb = vzero;
      for( int i = 0; i < N; i += 2 )
      {
        __m128i vsrca = _mm_unpacklo_epi16( vsrc[i], vsrc[i + 1] );
        __m128i vsrcb = _mm_unpackhi_epi16( vsrc[i], vsrc[i + 1] );
        vsuma = _mm_add_epi32( vsuma, _mm_madd_epi16( vsrca, vcoeff[i / 2] ) );
        vsumb = _mm_add_epi32( vsumb, _mm_madd_epi16( vsrcb, vcoeff[i / 2] ) );
      }
      for( int i = 0; i < N - 1; i++ )
      {
        vsrc[i] = vsrc[i + 1];
      }

//       if( shiftBack )
      {
        vsuma = _mm_add_epi32( vsuma, voffset );
        vsumb = _mm_add_epi32( vsumb, voffset );

        vsuma = _mm_srai_epi32( vsuma, shift );
        vsumb = _mm_srai_epi32( vsumb, shift );

        vsum = _mm_packs_epi32( vsuma, vsumb );
      }
//       else
//       {
//         vsuma = _mm_srai_epi32( vsuma, shift );       // val = sum >> shift;
//         vsumb = _mm_srai_epi32( vsumb, shift );
//
//         vsum = _mm_packs_epi32( vsuma, vsumb );
//         }
//       }
      if( shiftBack ) //clip
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }

#if JEM_UNALIGNED_DST
      _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
#else
      _mm_store_si128( ( __m128i * )&dst[col], vsum );
#endif

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}

// template<typename Tdst, int N, bool shiftBack, bool biPred, bool bWeight, bool bChromaIntl>
// static void qpelV16AVX2M8( const short* src, int srcStride, Tdst *dst, int dstStride, int width, int height, int shift, int bitdepth, short const *coeff, wpPredParam *pwp1=NULL, wpPredParam *pwp2=NULL )
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8_AVX2( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max );

  __m256i vsum;
  __m256i vsrc[N];
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  const short *srcOrig = src;
  int16_t *dstOrig = dst;

  for( int col = 0; col < width; col+=8 )
  {
    for( int i=0; i<N-1; i++ )
    {
      vsrc[i]= _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * )&src[col + i * srcStride] ) );
      vsrc[i]= _mm256_permute4x64_epi64( vsrc[i], 0x50 );
    }
    for( int row = 0; row < height; row++ )
    {
      vsrc[N-1]= _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * )&src[col + ( N-1 ) * srcStride] ) );
      vsrc[N-1]= _mm256_permute4x64_epi64( vsrc[N-1], 0x50 );
      vsum = _mm256_setzero_si256();
      for( int i=0; i<N; i+=2 )
      {
        __m256i vsrc0 = _mm256_unpacklo_epi16( vsrc[i], vsrc[i+1] );
        vsum  = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc0, vcoeff[i/2] ) );
      }
      for( int i=0; i<N-1; i++ )
      {
        vsrc[i] = vsrc[i+1];
      }

      {
        vsum = _mm256_add_epi32( vsum, voffset );
        vsum = _mm256_srai_epi32( vsum, shift );
      }

      __m128i vsump = _mm256_castsi256_si128( _mm256_permute4x64_epi64( _mm256_packs_epi32( vsum, vsum ), 0x88 ) );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }
      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );

      src += srcStride;
      dst += dstStride;
    }
    src= srcOrig;
    dst= dstOrig;
  }
#endif
}


// template<typename Tdst, int N, bool shiftBack, bool biPred, bool bWeight, bool bChromaIntl>
// static void qpelV16AVX2M16( const short *src, int srcStride, Tdst *dst, int dstStride, int width, int height, int shift, int bitdepth, short const *coeff, wpPredParam *pwp1=NULL, wpPredParam *pwp2=NULL )
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM16_AVX2( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  __m256i voffset    = _mm256_set1_epi32( offset );
  __m256i vibdimin   = _mm256_set1_epi16( clpRng.min );
  __m256i vibdimax   = _mm256_set1_epi16( clpRng.max );
  __m256i vzero      = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;

  __m256i vsrc[N];
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  const short *srcOrig = src;
  int16_t *dstOrig = dst;

  for( int col = 0; col < width; col+=16 )
  {
    for( int i=0; i<N-1; i++ )
    {
      vsrc[i] = _mm256_lddqu_si256( ( const __m256i * )&src[col + i * srcStride] );
    }
    for( int row = 0; row < height; row++ )
    {
      vsrc[N-1]= _mm256_lddqu_si256( ( const __m256i * )&src[col + ( N-1 ) * srcStride] );
      vsuma = vsumb = vzero;
      for( int i=0; i<N; i+=2 )
      {
        __m256i vsrca = _mm256_unpacklo_epi16( vsrc[i], vsrc[i+1] );
        __m256i vsrcb = _mm256_unpackhi_epi16( vsrc[i], vsrc[i+1] );
        vsuma  = _mm256_add_epi32( vsuma, _mm256_madd_epi16( vsrca, vcoeff[i/2] ) );
        vsumb  = _mm256_add_epi32( vsumb, _mm256_madd_epi16( vsrcb, vcoeff[i/2] ) );
      }
      for( int i=0; i<N-1; i++ )
      {
        vsrc[i] = vsrc[i+1];
      }

      {
        vsuma = _mm256_add_epi32( vsuma, voffset );
        vsumb = _mm256_add_epi32( vsumb, voffset );
        vsuma = _mm256_srai_epi32( vsuma, shift );
        vsumb = _mm256_srai_epi32( vsumb, shift );
        vsum  = _mm256_packs_epi32( vsuma, vsumb );
      }

      if( shiftBack )
      { //clip
        vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
      }

      _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );

      src += srcStride;
      dst += dstStride;
    }
    src= srcOrig;
    dst= dstOrig;
  }
#endif
}


template<int N, bool isLast>
inline void interpolate( const int16_t* src, int cStride, int16_t *dst, int width, int shift, int offset, int bitdepth, int maxVal, int16_t const *c )
{
  for( int col = 0; col < width; col++ )
  {
    int sum;

    sum = src[col + 0 * cStride] * c[0];
    sum += src[col + 1 * cStride] * c[1];
    if( N >= 4 )
    {
      sum += src[col + 2 * cStride] * c[2];
      sum += src[col + 3 * cStride] * c[3];
    }
    if( N >= 6 )
    {
      sum += src[col + 4 * cStride] * c[4];
      sum += src[col + 5 * cStride] * c[5];
    }
    if( N == 8 )
    {
      sum += src[col + 6 * cStride] * c[6];
      sum += src[col + 7 * cStride] * c[7];
    }

    Pel val = ( sum + offset ) >> shift;
    if( isLast )
    {
      val = ( val < 0 ) ? 0 : val;
      val = ( val > maxVal ) ? maxVal : val;
    }
    dst[col] = val;
  }
}


static inline __m128i simdInterpolateLuma2P8( int16_t const *src, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( int n = 0; n < 2; n++ )
  {
    __m128i mmPix = _mm_loadu_si128( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix, mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix, mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi, _mm_unpackhi_epi16( lo, hi ) );
    sumLo = _mm_add_epi32( sumLo, _mm_unpacklo_epi16( lo, hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi, mmOffset ), shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo, mmOffset ), shift );
  return( _mm_packs_epi32( sumLo, sumHi ) );
}

static inline __m128i simdInterpolateLuma2P4( int16_t const *src, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( int n = 0; n < 2; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix, mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix, mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi, _mm_unpackhi_epi16( lo, hi ) );
    sumLo = _mm_add_epi32( sumLo, _mm_unpacklo_epi16( lo, hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi, mmOffset ), shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo, mmOffset ), shift );
  return( _mm_packs_epi32( sumLo, sumHi ) );
}

static inline __m128i simdClip3( __m128i mmMin, __m128i mmMax, __m128i mmPix )
{
  __m128i mmMask = _mm_cmpgt_epi16( mmPix, mmMin );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask, mmPix ), _mm_andnot_si128( mmMask, mmMin ) );
  mmMask = _mm_cmplt_epi16( mmPix, mmMax );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask, mmPix ), _mm_andnot_si128( mmMask, mmMax ) );
  return( mmPix );
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M8( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c )
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi32( offset );
  __m128i mmCoeff[2];
  __m128i mmMin = _mm_set1_epi16( clpRng.min );
  __m128i mmMax = _mm_set1_epi16( clpRng.max );
  for( int n = 0; n < 2; n++ )
    mmCoeff[n] = _mm_set1_epi16( c[n] );
  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col += 8 )
    {
      __m128i mmFiltered = simdInterpolateLuma2P8( src + col, cStride, mmCoeff, mmOffset, shift );
      if( isLast )
      {
        mmFiltered = simdClip3( mmMin, mmMax, mmFiltered );
      }
      _mm_storeu_si128( ( __m128i * )( dst + col ), mmFiltered );
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M4( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c )
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi32( offset );
  __m128i mmCoeff[8];
  __m128i mmMin = _mm_set1_epi16( clpRng.min );
  __m128i mmMax = _mm_set1_epi16( clpRng.max );
  for( int n = 0; n < 2; n++ )
    mmCoeff[n] = _mm_set1_epi16( c[n] );
  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col += 4 )
    {
      __m128i mmFiltered = simdInterpolateLuma2P4( src + col, cStride, mmCoeff, mmOffset, shift );
      if( isLast )
      {
        mmFiltered = simdClip3( mmMin, mmMax, mmFiltered );
      }
      _mm_storel_epi64( ( __m128i * )( dst + col ), mmFiltered );
    }
    src += srcStride;
    dst += dstStride;
  }
}
#ifdef USE_AVX2
static inline __m256i simdInterpolateLuma10Bit2P16(int16_t const *src1, int srcStride, __m256i *mmCoeff, const __m256i & mmOffset, __m128i &mmShift)
{
  __m256i sumLo;
  {
    __m256i mmPix = _mm256_loadu_si256((__m256i*)src1);
    __m256i mmPix1 = _mm256_loadu_si256((__m256i*)(src1 + srcStride));
    __m256i lo0 = _mm256_mullo_epi16(mmPix, mmCoeff[0]);
    __m256i lo1 = _mm256_mullo_epi16(mmPix1, mmCoeff[1]);
    sumLo = _mm256_add_epi16(lo0, lo1);
  }
  sumLo = _mm256_sra_epi16(_mm256_add_epi16(sumLo, mmOffset), mmShift);
  return(sumLo);
}
#endif

static inline __m128i simdInterpolateLuma10Bit2P8(int16_t const *src1, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i sumLo;
  {
    __m128i mmPix = _mm_loadu_si128((__m128i*)src1);
    __m128i mmPix1 = _mm_loadu_si128((__m128i*)(src1 + srcStride));
    __m128i lo0 = _mm_mullo_epi16(mmPix, mmCoeff[0]);
    __m128i lo1 = _mm_mullo_epi16(mmPix1, mmCoeff[1]);
    sumLo = _mm_add_epi16(lo0, lo1);
  }
  sumLo = _mm_sra_epi16(_mm_add_epi16(sumLo, mmOffset), mmShift);
  return(sumLo);
}

static inline __m128i simdInterpolateLuma10Bit2P4(int16_t const *src, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i sumLo;
  {
    __m128i mmPix = _mm_loadl_epi64((__m128i*)src);
    __m128i mmPix1 = _mm_loadl_epi64((__m128i*)(src + srcStride));
    __m128i lo0 = _mm_mullo_epi16(mmPix, mmCoeff[0]);
    __m128i lo1 = _mm_mullo_epi16(mmPix1, mmCoeff[1]);
    sumLo = _mm_add_epi16(lo0, lo1);
  }
  sumLo = _mm_sra_epi16(_mm_add_epi16(sumLo, mmOffset), mmShift);
  return sumLo;
}

#ifdef USE_AVX2
static inline __m256i simdInterpolateLumaHighBit2P16(int16_t const *src1, int srcStride, __m256i *mmCoeff, const __m256i & mmOffset, __m128i &mmShift)
{
  __m256i mm_mul_lo = _mm256_setzero_si256();
  __m256i mm_mul_hi = _mm256_setzero_si256();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m256i mmPix = _mm256_lddqu_si256((__m256i*)(src1 + coefIdx * srcStride));
    __m256i mm_hi = _mm256_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m256i mm_lo = _mm256_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo = _mm256_add_epi32(mm_mul_lo, _mm256_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi = _mm256_add_epi32(mm_mul_hi, _mm256_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m256i mm_sum = _mm256_packs_epi32(mm_mul_lo, mm_mul_hi);
  return (mm_sum);
}
#endif

static inline __m128i simdInterpolateLumaHighBit2P8(int16_t const *src1, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i mm_mul_lo = _mm_setzero_si128();
  __m128i mm_mul_hi = _mm_setzero_si128();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix = _mm_loadu_si128((__m128i*)(src1 + coefIdx * srcStride));
    __m128i mm_hi = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo = _mm_add_epi32(mm_mul_lo, _mm_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi = _mm_add_epi32(mm_mul_hi, _mm_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo = _mm_sra_epi32(_mm_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi = _mm_sra_epi32(_mm_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m128i mm_sum = _mm_packs_epi32(mm_mul_lo, mm_mul_hi);
  return(mm_sum);
}

static inline __m128i simdInterpolateLumaHighBit2P4(int16_t const *src1, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i mm_sum = _mm_setzero_si128();
  __m128i mm_zero = _mm_setzero_si128();
  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix = _mm_loadl_epi64((__m128i*)(src1 + coefIdx * srcStride));
    __m128i mm_hi = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_mul = _mm_unpacklo_epi16(mm_lo, mm_hi);
    mm_sum = _mm_add_epi32(mm_sum, mm_mul);
  }
  mm_sum = _mm_sra_epi32(_mm_add_epi32(mm_sum, mmOffset), mmShift);
  mm_sum = _mm_packs_epi32(mm_sum, mm_zero);
  return(mm_sum);
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_HIGHBIT_M4(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c)
{
#if USE_AVX2
  __m256i mm256Offset = _mm256_set1_epi32(offset);
  __m256i mm256Coeff[2];
  for (int n = 0; n < 2; n++)
  {
    mm256Coeff[n] = _mm256_set1_epi16(c[n]);
  }
#endif
  __m128i mmOffset = _mm_set1_epi32(offset);
  __m128i mmCoeff[2];
  for (int n = 0; n < 2; n++)
    mmCoeff[n] = _mm_set1_epi16(c[n]);

  __m128i mmShift = _mm_cvtsi64_si128(shift);

  CHECK_(isLast, "Not Supported");
  CHECK_(width % 4 != 0, "Not Supported");

  for (int row = 0; row < height; row++)
  {
    int col = 0;
#if USE_AVX2
    for (; col < ((width >> 4) << 4); col += 16)
    {
      __m256i mmFiltered = simdInterpolateLumaHighBit2P16(src + col, cStride, mm256Coeff, mm256Offset, mmShift);
      _mm256_storeu_si256((__m256i *)(dst + col), mmFiltered);
    }
#endif
    for (; col < ((width >> 3) << 3); col += 8)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P8(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }

    for (; col < ((width >> 2) << 2); col += 4)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P4(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_10BIT_M4(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c)
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi16(offset);
  __m128i mmShift = _mm_set_epi64x(0, shift);
  __m128i mmCoeff[2];
  for (int n = 0; n < 2; n++)
    mmCoeff[n] = _mm_set1_epi16(c[n]);

  CHECK_(isLast, "Not Supported");

#if USE_AVX2
  __m256i mm256Offset = _mm256_set1_epi16(offset);
  __m256i mm256Coeff[2];
  for (int n = 0; n < 2; n++)
    mm256Coeff[n] = _mm256_set1_epi16(c[n]);
#endif
  for (row = 0; row < height; row++)
  {
    col = 0;
#if USE_AVX2
    // multiple of 16
    for (; col < ((width >> 4) << 4); col += 16)
    {
      __m256i mmFiltered = simdInterpolateLuma10Bit2P16(src + col, cStride, mm256Coeff, mm256Offset, mmShift);
      _mm256_storeu_si256((__m256i *)(dst + col), mmFiltered);
    }
#endif
    // multiple of 8
    for (; col < ((width >> 3) << 3); col += 8)
    {
      __m128i mmFiltered = simdInterpolateLuma10Bit2P8(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }

    // last 4 samples
    __m128i mmFiltered = simdInterpolateLuma10Bit2P4(src + col, cStride, mmCoeff, mmOffset, mmShift);
    _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, int N, bool isVertical, bool isFirst, bool isLast>
static void simdFilter( const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR)
{
  int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  int offset;
  int headRoom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK_( shift < 0, "Negative shift" );


  if( isLast )
  {
    shift += ( isFirst ) ? 0 : headRoom;
    offset = 1 << ( shift - 1 );
    offset += ( isFirst ) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift -= ( isFirst ) ? headRoom : 0;
    offset = ( isFirst ) ? -IF_INTERNAL_OFFS << shift : 0;
  }

  if (biMCForDMVR)
  {
    if( isFirst )
    {
      shift = IF_FILTER_PREC_BILINEAR - (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
      offset = 1 << (shift - 1);
    }
    else
    {
      shift = 4;
      offset = 1 << (shift - 1);
    }
  }
  {
    if( N == 8 && !( width & 0x07 ) )
    {
      if( !isVertical )
      {
        if( vext>= AVX2 )
          simdInterpolateHorM8_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        else
          simdInterpolateHorM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
      {
        if( vext>= AVX2 )
          simdInterpolateVerM8_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        else
          simdInterpolateVerM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      return;
    }
    else if( N == 8 && !( width & 0x03 ) )
    {
      if( !isVertical )
      {
        simdInterpolateHorM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
        simdInterpolateVerM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if( N == 4 && !( width & 0x03 ) )
    {
      if( !isVertical )
      {
        if( ( width % 8 ) == 0 )
        {
          if( vext>= AVX2 )
            simdInterpolateHorM8_AVX2<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          else
            simdInterpolateHorM8<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
          simdInterpolateHorM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
        simdInterpolateVerM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if (biMCForDMVR)
    {
      if (N == 2 && !(width & 0x03))
      {
        if (clpRng.bd <= 10)
        {
        simdInterpolateN2_10BIT_M4<vext, isLast>(src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c);
        }
        else
        {
          simdInterpolateN2_HIGHBIT_M4<vext, isLast>(src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c);
        }
        return;
      }
    }
    else if( N == 2 && !( width & 0x07 ) )
    {
      simdInterpolateN2_M8<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if( N == 2 && !( width & 0x03 ) )
    {
      simdInterpolateN2_M4<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
      return;
    }
  }

  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col++ )
    {
      int sum;

      sum  = src[col + 0 * cStride] * c[0];
      sum += src[col + 1 * cStride] * c[1];
      if( N >= 4 )
      {
        sum += src[col + 2 * cStride] * c[2];
        sum += src[col + 3 * cStride] * c[3];
      }
      if( N >= 6 )
      {
        sum += src[col + 4 * cStride] * c[4];
        sum += src[col + 5 * cStride] * c[5];
      }
      if( N == 8 )
      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

template< X86_VEXT vext >
void xWeightedGeoBlk_SSE(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  Pel* dst = predDst.get(compIdx).buf;
  Pel* src0 = predSrc0.get(compIdx).buf;
  Pel* src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst = predDst.get(compIdx).stride;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride;

  const char    log2WeightBase = 3;
  const ClpRng  clpRng = pu.cu->slice->clpRngs().comp[compIdx];
  const int32_t shiftWeighted = IF_INTERNAL_FRAC_BITS(clpRng.bd) + log2WeightBase;
  const int32_t offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);

  int16_t wIdx = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t angle = g_GeoParams[splitDir][0];
  int16_t stepY = 0;
  int16_t* weight = nullptr;
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }

  const __m128i mmEight = _mm_set1_epi16(8);
  const __m128i mmOffset = _mm_set1_epi32(offsetWeighted);
  const __m128i mmShift = _mm_cvtsi32_si128(shiftWeighted);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);

  if (compIdx != COMPONENT_Y && pu.chromaFormat == CHROMA_420)
    stepY <<= 1;
  if (width == 4)
  {
    // it will occur to chroma only
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *) (src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *) (src1));
      __m128i w0;
      if (g_angle2mirror[angle] == 1)
      {
        w0 = _mm_loadu_si128((__m128i *) (weight - (8 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
        w0 = _mm_shuffle_epi8(w0, shuffle_mask);
      }
      else
      {
        w0 = _mm_loadu_si128((__m128i *) (weight));
      }
      w0 = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0 = _mm_unpacklo_epi16(s0, s1);
      w0 = _mm_unpacklo_epi16(w0, w1);
      s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0 = _mm_sra_epi32(s0, mmShift);
      s0 = _mm_packs_epi32(s0, s0);
      s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
      _mm_storel_epi64((__m128i *) (dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (width >= 16)
  {
    const __m256i mmEightAVX2 = _mm256_set1_epi16(8);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(offsetWeighted);
    const __m256i mmMinAVX2 = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMaxAVX2 = _mm256_set1_epi16(clpRng.max);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *) (src0 + x)); // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *) (src1 + x));

        __m256i w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
        if (compIdx != COMPONENT_Y &&  pu.chromaFormat != CHROMA_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - (16 - 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0 = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1 = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1 = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0 = _mm256_packs_epi16(w0p0, w0p1);
          w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        s0 = _mm256_min_epi16(mmMaxAVX2, _mm256_max_epi16(s0, mmMinAVX2));
        _mm256_storeu_si256((__m256i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *) (src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *) (src1 + x));
        __m128i w0;
        if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - (8 - 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1 = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0 = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
        _mm_storeu_si128((__m128i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}

template <X86_VEXT vext>
void InterpolationFilter::_initInterpolationFilterX86()
{
  // [taps][bFirst][bLast]
  m_filterHor[0][0][0] = simdFilter<vext, 8, false, false, false>;
  m_filterHor[0][0][1] = simdFilter<vext, 8, false, false, true>;
  m_filterHor[0][1][0] = simdFilter<vext, 8, false, true, false>;
  m_filterHor[0][1][1] = simdFilter<vext, 8, false, true, true>;

  m_filterHor[1][0][0] = simdFilter<vext, 4, false, false, false>;
  m_filterHor[1][0][1] = simdFilter<vext, 4, false, false, true>;
  m_filterHor[1][1][0] = simdFilter<vext, 4, false, true, false>;
  m_filterHor[1][1][1] = simdFilter<vext, 4, false, true, true>;

  m_filterHor[2][0][0] = simdFilter<vext, 2, false, false, false>;
  m_filterHor[2][0][1] = simdFilter<vext, 2, false, false, true>;
  m_filterHor[2][1][0] = simdFilter<vext, 2, false, true, false>;
  m_filterHor[2][1][1] = simdFilter<vext, 2, false, true, true>;

  m_filterVer[0][0][0] = simdFilter<vext, 8, true, false, false>;
  m_filterVer[0][0][1] = simdFilter<vext, 8, true, false, true>;
  m_filterVer[0][1][0] = simdFilter<vext, 8, true, true, false>;
  m_filterVer[0][1][1] = simdFilter<vext, 8, true, true, true>;

  m_filterVer[1][0][0] = simdFilter<vext, 4, true, false, false>;
  m_filterVer[1][0][1] = simdFilter<vext, 4, true, false, true>;
  m_filterVer[1][1][0] = simdFilter<vext, 4, true, true, false>;
  m_filterVer[1][1][1] = simdFilter<vext, 4, true, true, true>;

  m_filterVer[2][0][0] = simdFilter<vext, 2, true, false, false>;
  m_filterVer[2][0][1] = simdFilter<vext, 2, true, false, true>;
  m_filterVer[2][1][0] = simdFilter<vext, 2, true, true, false>;
  m_filterVer[2][1][1] = simdFilter<vext, 2, true, true, true>;

  m_filterCopy[0][0]   = simdFilterCopy<vext, false, false>;
  m_filterCopy[0][1]   = simdFilterCopy<vext, false, true>;
  m_filterCopy[1][0]   = simdFilterCopy<vext, true, false>;
  m_filterCopy[1][1]   = simdFilterCopy<vext, true, true>;

  m_weightedGeoBlk = xWeightedGeoBlk_SSE<vext>;
}

template void InterpolationFilter::_initInterpolationFilterX86<SIMDX86>();

#endif //#ifdef TARGET_SIMD_X86
//! \}

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

/** \file     VideoIOYuv.cpp
    \brief    YUV file I/O class
*/

#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include <memory.h>

#include "CommonLib/Rom.h"
#include "VideoIOYuv.h"
#include "CommonLib/Unit.h"

using namespace std;

#define FLIP_PIC 0

// ====================================================================================================================
// Local Functions
// ====================================================================================================================

/**
 * Scale all pixels in img depending upon sign of shiftbits by a factor of
 * 2<sup>shiftbits</sup>.
 *
 * @param areabuf buffer to be scaled
 * @param shiftbits if zero, no operation performed
 *                  if > 0, multiply by 2<sup>shiftbits</sup>, see scalePlane()
 *                  if < 0, divide and round by 2<sup>shiftbits</sup> and clip,
 *                          see invScalePlane().
 * @param minval  minimum clipping value when dividing.
 * @param maxval  maximum clipping value when dividing.
 */
static void scalePlane( PelBuf& areaBuf, const int shiftbits, const Pel minval, const Pel maxval)
{
  const unsigned width  = areaBuf.width;
  const unsigned height = areaBuf.height;
  const unsigned stride = areaBuf.stride;
        Pel*        img = areaBuf.bufAt(0,0);

  if( 0 == shiftbits )
  {
    return;
  }

  if( shiftbits > 0)
  {
    for( unsigned y = 0; y < height; y++, img+=stride)
    {
      for( unsigned x = 0; x < width; x++)
      {
        img[x] <<= shiftbits;
      }
    }
  }
  else if (shiftbits < 0)
  {
    const int shiftbitsr =- shiftbits;
    const Pel rounding = 1 << (shiftbitsr-1);

    for( unsigned y = 0; y < height; y++, img+=stride)
    {
      for( unsigned x = 0; x < width; x++)
      {
        img[x] = Clip3(minval, maxval, Pel((img[x] + rounding) >> shiftbitsr));
      }
    }
  }
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * Open file for reading/writing Y'CbCr frames.
 *
 * Frames read/written have bitdepth fileBitDepth, and are automatically
 * formatted as 8 or 16 bit word values (see VideoIOYuv::write()).
 *
 * Image data read or written is converted to/from internalBitDepth
 * (See scalePlane(), VideoIOYuv::read() and VideoIOYuv::write() for
 * further details).
 *
 * \param pchFile          file name string
 * \param bWriteMode       file open mode: true=write, false=read
 * \param fileBitDepth     bit-depth array of input/output file data.
 * \param MSBExtendedBitDepth
 * \param internalBitDepth bit-depth array to scale image data to/from when reading/writing.
 */
void VideoIOYuv::open( const std::string &fileName, bool bWriteMode, const int fileBitDepth[MAX_NUM_CHANNEL_TYPE], const int MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] )
{
  //NOTE: files cannot have bit depth greater than 16
  for(uint32_t ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_fileBitdepth       [ch] = std::min<uint32_t>(fileBitDepth[ch], 16);
    m_MSBExtendedBitDepth[ch] = MSBExtendedBitDepth[ch];
    m_bitdepthShift      [ch] = internalBitDepth[ch] - m_MSBExtendedBitDepth[ch];

    if (m_fileBitdepth[ch] > 16)
    {
      if (bWriteMode)
      {
        std::cerr << "\nWARNING: Cannot write a yuv file of bit depth greater than 16 - output will be right-shifted down to 16-bit precision\n" << std::endl;
      }
      else
      {
        EXIT( "ERROR_: Cannot read a yuv file of bit depth greater than 16" );
      }
    }
  }

  if ( bWriteMode )
  {
    m_cHandle.open( fileName.c_str(), ios::binary | ios::out );

    if( m_cHandle.fail() )
    {
      EXIT( "Failed to write reconstructed YUV file: " << fileName.c_str() );
    }
  }
  else
  {
    m_cHandle.open( fileName.c_str(), ios::binary | ios::in );

    if( m_cHandle.fail() )
    {
      EXIT( "Failed to open input YUV file: " << fileName.c_str() );
    }
  }

  return;
}

void VideoIOYuv::close()
{
  m_cHandle.close();
}

bool VideoIOYuv::isEof()
{
  return m_cHandle.eof();
}

bool VideoIOYuv::isFail()
{
  return m_cHandle.fail();
}

/**
 * Skip numFrames in input.
 *
 * This function correctly handles cases where the input file is not
 * seekable, by consuming bytes.
 */
#if EXTENSION_360_VIDEO
void VideoIOYuv::skipFrames(int numFrames, uint32_t width, uint32_t height, ChromaFormat format)
#else
void VideoIOYuv::skipFrames(uint32_t numFrames, uint32_t width, uint32_t height, ChromaFormat format)
#endif
{
  if (!numFrames)
  {
    return;
  }

  //------------------
  //set the frame size according to the chroma format
  streamoff frameSize = 0;
  uint32_t wordsize=1; // default to 8-bit, unless a channel with more than 8-bits is detected.
  for (uint32_t component = 0; component < getNumberValidComponents(format); component++)
  {
    ComponentID compID=ComponentID(component);
    frameSize += (width >> getComponentScaleX(compID, format)) * (height >> getComponentScaleY(compID, format));
    if (m_fileBitdepth[toChannelType(compID)] > 8)
    {
      wordsize=2;
    }
  }
  frameSize *= wordsize;
  //------------------

  const streamoff offset = frameSize * numFrames;

  /* attempt to seek */
  if (!!m_cHandle.seekg(offset, ios::cur))
  {
    return; /* success */
  }
  m_cHandle.clear();

  /* fall back to consuming the input */
  char buf[512];
  const streamoff offset_mod_bufsize = offset % sizeof(buf);
  for (streamoff i = 0; i < offset - offset_mod_bufsize; i += sizeof(buf))
  {
    m_cHandle.read(buf, sizeof(buf));
  }
  m_cHandle.read(buf, offset_mod_bufsize);
}

/**
 * Read width*height pixels from fd into dst, optionally
 * padding the left and right edges by edge-extension.  Input may be
 * either 8bit or 16bit little-endian lsb-aligned words.
 *
 * @param dst          destination image plane
 * @param fd           input file stream
 * @param is16bit      true if input file carries > 8bit data, false otherwise.
 * @param stride444    distance between vertically adjacent pixels of dst.
 * @param width444     width of active area in dst.
 * @param height444    height of active area in dst.
 * @param pad_x444     length of horizontal padding.
 * @param pad_y444     length of vertical padding.
 * @param compID       chroma component
 * @param destFormat   chroma format of image
 * @param fileFormat   chroma format of file
 * @param fileBitDepth component bit depth in file
 * @return true for success, false in case of error
 */
static bool readPlane(Pel* dst,
                      istream& fd,
                      bool is16bit,
                      uint32_t stride444,
                      uint32_t width444,
                      uint32_t height444,
                      uint32_t pad_x444,
                      uint32_t pad_y444,
                      const ComponentID compID,
                      const ChromaFormat destFormat,
                      const ChromaFormat fileFormat,
                      const uint32_t fileBitDepth)
{
  const uint32_t csx_file =getComponentScaleX(compID, fileFormat);
  const uint32_t csy_file =getComponentScaleY(compID, fileFormat);
  const uint32_t csx_dest =getComponentScaleX(compID, destFormat);
  const uint32_t csy_dest =getComponentScaleY(compID, destFormat);

  const uint32_t width_dest       = width444 >>csx_dest;
  const uint32_t height_dest      = height444>>csy_dest;
  const uint32_t pad_x_dest       = pad_x444>>csx_dest;
  const uint32_t pad_y_dest       = pad_y444>>csy_dest;
#if EXTENSION_360_VIDEO
  const uint32_t stride_dest = stride444;
#else
  const uint32_t stride_dest      = stride444>>csx_dest;
#endif
  const uint32_t full_width_dest  = width_dest+pad_x_dest;
  const uint32_t full_height_dest = height_dest+pad_y_dest;

  const uint32_t stride_file      = (width444 * (is16bit ? 2 : 1)) >> csx_file;
  std::vector<uint8_t> bufVec(stride_file);
  uint8_t *buf=&(bufVec[0]);

  Pel  *pDstPad              = dst + stride_dest * height_dest;
  Pel  *pDstBuf              = dst;
  const int dstbuf_stride    = stride_dest;

  if (compID!=COMPONENT_Y && (fileFormat==CHROMA_400 || destFormat==CHROMA_400))
  {
    if (destFormat!=CHROMA_400)
    {
      // set chrominance data to mid-range: (1<<(fileBitDepth-1))
      const Pel value=Pel(1<<(fileBitDepth-1));
      for (uint32_t y = 0; y < full_height_dest; y++, pDstBuf+= dstbuf_stride)
      {
        for (uint32_t x = 0; x < full_width_dest; x++)
        {
          pDstBuf[x] = value;
        }
      }
    }

    if (fileFormat!=CHROMA_400)
    {
      const uint32_t height_file      = height444>>csy_file;
      fd.seekg(height_file*stride_file, ios::cur);
      if (fd.eof() || fd.fail() )
      {
        return false;
      }
    }
  }
  else
  {
    const uint32_t mask_y_file=(1<<csy_file)-1;
    const uint32_t mask_y_dest=(1<<csy_dest)-1;
    for(uint32_t y444=0; y444<height444; y444++)
    {
      if ((y444&mask_y_file)==0)
      {
        // read a new line
        fd.read(reinterpret_cast<char*>(buf), stride_file);
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if ((y444&mask_y_dest)==0)
      {
        // process current destination line
        if (csx_file < csx_dest)
        {
          // eg file is 444, dest is 422.
          const uint32_t sx=csx_dest-csx_file;
          if (!is16bit)
          {
            for (uint32_t x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = buf[x<<sx];
            }
          }
          else
          {
            for (uint32_t x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = Pel(buf[(x<<sx)*2+0]) | (Pel(buf[(x<<sx)*2+1])<<8);
            }
          }
        }
        else
        {
          // eg file is 422, dest is 444.
          const uint32_t sx=csx_file-csx_dest;
          if (!is16bit)
          {
            for (uint32_t x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = buf[x>>sx];
            }
          }
          else
          {
            for (uint32_t x = 0; x < width_dest; x++)
            {
              pDstBuf[x] = Pel(buf[(x>>sx)*2+0]) | (Pel(buf[(x>>sx)*2+1])<<8);
            }
          }
        }

        // process right hand side padding
        const Pel val=dst[width_dest-1];
        for (uint32_t x = width_dest; x < full_width_dest; x++)
        {
          pDstBuf[x] = val;
        }

        pDstBuf+= dstbuf_stride;
      }
    }

    // process lower padding
    for (uint32_t y = height_dest; y < full_height_dest; y++, pDstPad+=stride_dest)
    {
      for (uint32_t x = 0; x < full_width_dest; x++)
      {
        pDstPad[x] = (pDstPad - stride_dest)[x];
      }
    }
  }
  return true;
}

static bool verifyPlane(Pel* dst,
                      uint32_t stride444,
                      uint32_t width444,
                      uint32_t height444,
                      uint32_t padX444,
                      uint32_t padY444,
                      const ComponentID compID,
                      const ChromaFormat cFormat,
                      const uint32_t bitDepth)
{
  const uint32_t csx =getComponentScaleX(compID, cFormat);
  const uint32_t csy =getComponentScaleY(compID, cFormat);

#if EXTENSION_360_VIDEO
  const uint32_t stride = stride444;
#else
  const uint32_t stride = stride444>>csx;
#endif
  const uint32_t fullWidth  = (width444 + padX444) >> csx;
  const uint32_t fullHeight = (height444 +padY444) >> csy;

  Pel  *dstBuf              = dst;

  const Pel mask = ~((1 << bitDepth) - 1);

  for (uint32_t y = 0; y < fullHeight; y++, dstBuf+= stride)
  {
    for (uint32_t x = 0; x < fullWidth; x++)
    {
      if ( (dstBuf[x] & mask) != 0)
      {
        return false;
      }
    }
  }

  return true;
}


/**
 * Write an image plane (width444*height444 pixels) from src into output stream fd.
 *
 * @param fd         output file stream
 * @param src        source image
 * @param is16bit    true if input file carries > 8bit data, false otherwise.
 * @param stride444  distance between vertically adjacent pixels of src.
 * @param width444   width of active area in src.
 * @param height444  height of active area in src.
 * @param compID       chroma component
 * @param srcFormat    chroma format of image
 * @param fileFormat   chroma format of file
 * @param fileBitDepth component bit depth in file
 * @return true for success, false in case of error
 */
static bool writePlane( uint32_t orgWidth, uint32_t orgHeight, ostream& fd, const Pel* src,
                       const bool is16bit,
                       const uint32_t stride_src,
                       uint32_t width444, uint32_t height444,
                       const ComponentID compID,
                       const ChromaFormat srcFormat,
                       const ChromaFormat fileFormat,
                       const uint32_t fileBitDepth,
                       const uint32_t packedYUVOutputMode = 0)
{
  const uint32_t csx_file =getComponentScaleX(compID, fileFormat);
  const uint32_t csy_file =getComponentScaleY(compID, fileFormat);
  const uint32_t csx_src  =getComponentScaleX(compID, srcFormat);
  const uint32_t csy_src  =getComponentScaleY(compID, srcFormat);

  const uint32_t width_file  = width444  >> csx_file;
  const uint32_t height_file = height444 >> csy_file;
  const bool     writePYUV   = (packedYUVOutputMode > 0) && (fileBitDepth == 10 || fileBitDepth == 12) && ((width_file & (1 + (fileBitDepth & 3))) == 0);

  CHECK_( csx_file != csx_src, "Not supported" );
  const uint32_t stride_file = writePYUV ? ( orgWidth * fileBitDepth ) >> ( csx_file + 3 ) : ( orgWidth * ( is16bit ? 2 : 1 ) ) >> csx_file;

  std::vector<uint8_t> bufVec(stride_file);
  uint8_t *buf=&(bufVec[0]);

  const Pel *pSrcBuf         = src;
  const int srcbuf_stride    = stride_src;

  if (writePYUV)
  {
    const uint32_t mask_y_file = (1 << csy_file) - 1;
    const uint32_t mask_y_src  = (1 << csy_src ) - 1;
    const uint32_t widthS_file = width_file >> (fileBitDepth == 12 ? 1 : 2);

    for (uint32_t y444 = 0; y444 < height444; y444++)
    {
      if ((y444 & mask_y_file) == 0)  // write a new line to file
      {
        if (csx_file < csx_src)
        {
          // eg file is 444, source is 422.
          const uint32_t sx = csx_src - csx_file;

          if (fileBitDepth == 10)  // write 4 values into 5 bytes
          {
            for (uint32_t x = 0; x < widthS_file; x++)
            {
              const uint32_t src0 = pSrcBuf[(4*x  ) >> sx];
              const uint32_t src1 = pSrcBuf[(4*x+1) >> sx];
              const uint32_t src2 = pSrcBuf[(4*x+2) >> sx];
              const uint32_t src3 = pSrcBuf[(4*x+3) >> sx];

              buf[5*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[5*x+1] = ((src1 << 2) & 0xfc) + ((src0 >> 8) & 0x03);
              buf[5*x+2] = ((src2 << 4) & 0xf0) + ((src1 >> 6) & 0x0f);
              buf[5*x+3] = ((src3 << 6) & 0xc0) + ((src2 >> 4) & 0x3f);
              buf[5*x+4] = ((src3 >> 2) & 0xff); // src3:98765432
            }
          }
          else if (fileBitDepth == 12) //...2 values into 3 bytes
          {
            for (uint32_t x = 0; x < widthS_file; x++)
            {
              const uint32_t src0 = pSrcBuf[(2*x  ) >> sx];
              const uint32_t src1 = pSrcBuf[(2*x+1) >> sx];

              buf[3*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[3*x+1] = ((src1 << 4) & 0xf0) + ((src0 >> 8) & 0x0f);
              buf[3*x+2] = ((src1 >> 4) & 0xff); // src1:BA987654
            }
          }
        }
        else
        {
          // eg file is 422, source is 444.
          const uint32_t sx = csx_file - csx_src;

          if (fileBitDepth == 10)  // write 4 values into 5 bytes
          {
            for (uint32_t x = 0; x < widthS_file; x++)
            {
              const uint32_t src0 = pSrcBuf[(4*x  ) << sx];
              const uint32_t src1 = pSrcBuf[(4*x+1) << sx];
              const uint32_t src2 = pSrcBuf[(4*x+2) << sx];
              const uint32_t src3 = pSrcBuf[(4*x+3) << sx];

              buf[5*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[5*x+1] = ((src1 << 2) & 0xfc) + ((src0 >> 8) & 0x03);
              buf[5*x+2] = ((src2 << 4) & 0xf0) + ((src1 >> 6) & 0x0f);
              buf[5*x+3] = ((src3 << 6) & 0xc0) + ((src2 >> 4) & 0x3f);
              buf[5*x+4] = ((src3 >> 2) & 0xff); // src3:98765432
            }
          }
          else if (fileBitDepth == 12) //...2 values into 3 bytes
          {
            for (uint32_t x = 0; x < widthS_file; x++)
            {
              const uint32_t src0 = pSrcBuf[(2*x  ) << sx];
              const uint32_t src1 = pSrcBuf[(2*x+1) << sx];

              buf[3*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[3*x+1] = ((src1 << 4) & 0xf0) + ((src0 >> 8) & 0x0f);
              buf[3*x+2] = ((src1 >> 4) & 0xff); // src1:BA987654
            }
          }
        }

        fd.write (reinterpret_cast<const char*>(buf), stride_file);
        if (fd.eof() || fd.fail())
        {
          return false;
        }
      }

      if ((y444 & mask_y_src) == 0)
      {
        pSrcBuf += srcbuf_stride;
      }
    }

    // here height444 and orgHeight are luma heights
    if ((compID == COMPONENT_Y) || (fileFormat != CHROMA_400 && srcFormat != CHROMA_400))
    {
      for (uint32_t y444 = height444; y444 < orgHeight; y444++)
      {
        if ((y444 & mask_y_file) == 0) // if this is chroma, determine whether to skip every other row
        {
          memset (reinterpret_cast<char*>(buf), 0, stride_file);

          fd.write (reinterpret_cast<const char*>(buf), stride_file);
          if (fd.eof() || fd.fail())
          {
            return false;
          }
        }

        if ((y444 & mask_y_src) == 0)
        {
          pSrcBuf += srcbuf_stride;
        }
      }
    }
  }
  else // !writePYUV
  if (compID!=COMPONENT_Y && (fileFormat==CHROMA_400 || srcFormat==CHROMA_400))
  {
    if (fileFormat!=CHROMA_400)
    {
      const uint32_t value = 1 << (fileBitDepth - 1);

      for (uint32_t y = 0; y < height_file; y++)
      {
        if (!is16bit)
        {
          uint8_t val(value);
          for (uint32_t x = 0; x < width_file; x++)
          {
            buf[x]=val;
          }
        }
        else
        {
          uint16_t val(value);
          for (uint32_t x = 0; x < width_file; x++)
          {
            buf[2*x  ]= (val>>0) & 0xff;
            buf[2*x+1]= (val>>8) & 0xff;
          }
        }

        fd.write(reinterpret_cast<const char*>(buf), stride_file);
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }
    }
  }
  else
  {
    const uint32_t mask_y_file = (1 << csy_file) - 1;
    const uint32_t mask_y_src  = (1 << csy_src ) - 1;

    for (uint32_t y444 = 0; y444 < height444; y444++)
    {
      if ((y444 & mask_y_file) == 0)
      {
        // write a new line
        if (csx_file < csx_src)
        {
          // eg file is 444, source is 422.
          const uint32_t sx = csx_src - csx_file;
          if (!is16bit)
          {
            for (uint32_t x = 0; x < width_file; x++)
            {
              buf[x] = (uint8_t)(pSrcBuf[x>>sx]);
            }
          }
          else
          {
            for (uint32_t x = 0; x < width_file; x++)
            {
              buf[2*x  ] = (pSrcBuf[x>>sx]>>0) & 0xff;
              buf[2*x+1] = (pSrcBuf[x>>sx]>>8) & 0xff;
            }
          }
        }
        else
        {
          // eg file is 422, source is 444.
          const uint32_t sx = csx_file - csx_src;
          if (!is16bit)
          {
            for (uint32_t x = 0; x < width_file; x++)
            {
              buf[x] = (uint8_t)(pSrcBuf[x<<sx]);
            }
          }
          else
          {
            for (uint32_t x = 0; x < width_file; x++)
            {
              buf[2*x  ] = (pSrcBuf[x<<sx]>>0) & 0xff;
              buf[2*x+1] = (pSrcBuf[x<<sx]>>8) & 0xff;
            }
          }
        }

        fd.write (reinterpret_cast<const char*>(buf), stride_file);
        if (fd.eof() || fd.fail())
        {
          return false;
        }
      }

      if ((y444 & mask_y_src) == 0)
      {
        pSrcBuf += srcbuf_stride;
      }
    }

    // here height444 and orgHeight are luma heights
    for( uint32_t y444 = height444; y444 < orgHeight; y444++ )
    {
      if( ( y444 & mask_y_file ) == 0 ) // if this is chroma, determine whether to skip every other row
      {
        if( !is16bit )
        {
          for( uint32_t x = 0; x < ( orgWidth >> csx_file ); x++ )
          {
            buf[x] = 0;
          }
        }
        else
        {
          for( uint32_t x = 0; x < ( orgWidth >> csx_file ); x++ )
          {
            buf[2 * x] = 0;
            buf[2 * x + 1] = 0;
          }
        }
        fd.write( reinterpret_cast<const char*>( buf ), stride_file );
        if( fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if( ( y444 & mask_y_src ) == 0 )
      {
        pSrcBuf += srcbuf_stride;
      }
    }

  }
  return true;
}

static bool writeField(ostream& fd, const Pel* top, const Pel* bottom,
                       const bool is16bit,
                       const uint32_t stride_src,
                       uint32_t width444, uint32_t height444,
                       const ComponentID compID,
                       const ChromaFormat srcFormat,
                       const ChromaFormat fileFormat,
                       const uint32_t fileBitDepth, const bool isTff,
                       const uint32_t packedYUVOutputMode = 0)
{
  const uint32_t csx_file =getComponentScaleX(compID, fileFormat);
  const uint32_t csy_file =getComponentScaleY(compID, fileFormat);
  const uint32_t csx_src  =getComponentScaleX(compID, srcFormat);
  const uint32_t csy_src  =getComponentScaleY(compID, srcFormat);

  const uint32_t width_file  = width444  >> csx_file;
  const uint32_t height_file = height444 >> csy_file;
  const bool     writePYUV   = (packedYUVOutputMode > 0) && (fileBitDepth == 10 || fileBitDepth == 12) && ((width_file & (1 + (fileBitDepth & 3))) == 0);
  const uint32_t stride_file = writePYUV ? (width444 * fileBitDepth) >> (csx_file + 3) : (width444 * (is16bit ? 2 : 1)) >> csx_file;

  std::vector<uint8_t> bufVec(stride_file * 2);
  uint8_t *buf=&(bufVec[0]);

  if (writePYUV)
  {
    // TODO
  }
  else // !writePYUV
  if (compID!=COMPONENT_Y && (fileFormat==CHROMA_400 || srcFormat==CHROMA_400))
  {
    if (fileFormat!=CHROMA_400)
    {
      const uint32_t value = 1 << (fileBitDepth - 1);

      for (uint32_t y = 0; y < height_file; y++)
      {
        for (uint32_t field = 0; field < 2; field++)
        {
          uint8_t *fieldBuffer = buf + (field * stride_file);

          if (!is16bit)
          {
            uint8_t val(value);
            for (uint32_t x = 0; x < width_file; x++)
            {
              fieldBuffer[x]=val;
            }
          }
          else
          {
            uint16_t val(value);
            for (uint32_t x = 0; x < width_file; x++)
            {
              fieldBuffer[2*x  ]= (val>>0) & 0xff;
              fieldBuffer[2*x+1]= (val>>8) & 0xff;
            }
          }
        }

        fd.write(reinterpret_cast<const char*>(buf), (stride_file * 2));
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }
    }
  }
  else
  {
    const uint32_t mask_y_file=(1<<csy_file)-1;
    const uint32_t mask_y_src =(1<<csy_src )-1;
    for(uint32_t y444=0; y444<height444; y444++)
    {
      if ((y444&mask_y_file)==0)
      {
        for (uint32_t field = 0; field < 2; field++)
        {
          uint8_t *fieldBuffer = buf + (field * stride_file);
          const Pel *src     = (((field == 0) && isTff) || ((field == 1) && (!isTff))) ? top : bottom;

          // write a new line
          if (csx_file < csx_src)
          {
            // eg file is 444, source is 422.
            const uint32_t sx=csx_src-csx_file;
            if (!is16bit)
            {
              for (uint32_t x = 0; x < width_file; x++)
              {
                fieldBuffer[x] = (uint8_t)(src[x>>sx]);
              }
            }
            else
            {
              for (uint32_t x = 0; x < width_file; x++)
              {
                fieldBuffer[2*x  ] = (src[x>>sx]>>0) & 0xff;
                fieldBuffer[2*x+1] = (src[x>>sx]>>8) & 0xff;
              }
            }
          }
          else
          {
            // eg file is 422, src is 444.
            const uint32_t sx=csx_file-csx_src;
            if (!is16bit)
            {
              for (uint32_t x = 0; x < width_file; x++)
              {
                fieldBuffer[x] = (uint8_t)(src[x<<sx]);
              }
            }
            else
            {
              for (uint32_t x = 0; x < width_file; x++)
              {
                fieldBuffer[2*x  ] = (src[x<<sx]>>0) & 0xff;
                fieldBuffer[2*x+1] = (src[x<<sx]>>8) & 0xff;
              }
            }
          }
        }

        fd.write(reinterpret_cast<const char*>(buf), (stride_file * 2));
        if (fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if ((y444&mask_y_src)==0)
      {
        top    += stride_src;
        bottom += stride_src;
      }

    }
  }
  return true;
}

/**
 * Read one Y'CbCr frame, performing any required input scaling to change
 * from the bitdepth of the input file to the internal bit-depth.
 *
 * If a bit-depth reduction is required, and internalBitdepth >= 8, then
 * the input file is assumed to be ITU-R BT.601/709 compliant, and the
 * resulting data is clipped to the appropriate legal range, as if the
 * file had been provided at the lower-bitdepth compliant to Rec601/709.
 *
 * @param pPicYuvUser      input picture YUV buffer class pointer
 * @param pPicYuvTrueOrg
 * @param ipcsc
 * @param aiPad            source padding size, aiPad[0] = horizontal, aiPad[1] = vertical
 * @param format           chroma format
 * @return true for success, false in case of error
 */
bool VideoIOYuv::read ( PelUnitBuf& pic, PelUnitBuf& picOrg, const InputColourSpaceConversion ipcsc, int aiPad[2], ChromaFormat format, const bool bClipToRec709 )
{
  // check end-of-file
  if ( isEof() )
  {
    return false;
  }

  if (format >= NUM_CHROMA_FORMAT)
  {
    format = picOrg.chromaFormat;
  }

  bool is16bit = false;

  for(uint32_t ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (m_fileBitdepth[ch] > 8)
    {
      is16bit=true;
    }
  }

  const PelBuf areaBufY = picOrg.get(COMPONENT_Y);
#if !EXTENSION_360_VIDEO
  const uint32_t stride444      = areaBufY.stride;
#endif
  // compute actual YUV width & height excluding padding size
  const uint32_t pad_h444       = aiPad[0];
  const uint32_t pad_v444       = aiPad[1];

  const uint32_t width_full444  = areaBufY.width;
  const uint32_t height_full444 = areaBufY.height;

  const uint32_t width444       = width_full444 - pad_h444;
  const uint32_t height444      = height_full444 - pad_v444;

  for( uint32_t comp=0; comp < ::getNumberValidComponents(format); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const ChannelType chType=toChannelType(compID);

    const int desired_bitdepth = m_MSBExtendedBitDepth[chType] + m_bitdepthShift[chType];

    const bool b709Compliance=(bClipToRec709) && (m_bitdepthShift[chType] < 0 && desired_bitdepth >= 8);     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
    const Pel minval = b709Compliance? ((   1 << (desired_bitdepth - 8))   ) : 0;
    const Pel maxval = b709Compliance? ((0xff << (desired_bitdepth - 8)) -1) : (1 << desired_bitdepth) - 1;
    const bool processComponent = (size_t)compID < picOrg.bufs.size();
    Pel* const dst = processComponent ? picOrg.get(compID).bufAt(0,0) : nullptr;
#if EXTENSION_360_VIDEO
    const uint32_t stride444 = picOrg.get(compID).stride;
#endif
    if ( ! readPlane( dst, m_cHandle, is16bit, stride444, width444, height444, pad_h444, pad_v444, compID, picOrg.chromaFormat, format, m_fileBitdepth[chType]))
    {
      return false;
    }

    if (processComponent)
    {
      if (! verifyPlane( dst, stride444, width444, height444, pad_h444, pad_v444, compID, format, m_fileBitdepth[chType]) )
      {
         EXIT("Source image contains values outside the specified bit range!");
      }
#if !JVET_R0351_HIGH_BIT_DEPTH_ENABLED
      if (m_fileBitdepth[chType] > 14 && m_bitdepthShift[chType] < 0)
      {
        EXIT("JVET_R0351_HIGH_BIT_DEPTH_ENABLED must be enabled for bit depths above 14 if INTERNALBITDEPTH < INPUTBITDEPTH");
      }
#endif
      scalePlane( picOrg.get(compID), m_bitdepthShift[chType], minval, maxval);
    }
  }

#if EXTENSION_360_VIDEO
  if (pic.chromaFormat != NUM_CHROMA_FORMAT)
    ColourSpaceConvert(picOrg, pic, ipcsc, true);
#else
  ColourSpaceConvert( picOrg, pic, ipcsc, true);
#endif

  picOrg.copyFrom(pic);

  return true;
}

/**
 * Write one Y'CbCr frame. No bit-depth conversion is performed, pcPicYuv is
 * assumed to be at TVideoIO::m_fileBitdepth depth.
 *
 * @param pPicYuvUser      input picture YUV buffer class pointer
 * @param ipCSC
 * @param confLeft         conformance window left border
 * @param confRight        conformance window right border
 * @param confTop          conformance window top border
 * @param confBottom       conformance window bottom border
 * @param format           chroma format
 * @return true for success, false in case of error
 */
 // here orgWidth and orgHeight are for luma
bool VideoIOYuv::write( uint32_t orgWidth, uint32_t orgHeight, const CPelUnitBuf& pic,
                        const InputColourSpaceConversion ipCSC,
                        const bool bPackedYUVOutputMode,
                        int confLeft, int confRight, int confTop, int confBottom, ChromaFormat format, const bool bClipToRec709, const bool subtractConfWindowOffsets )
{
  PelStorage interm;

  if (ipCSC!=IPCOLOURSPACE_UNCHANGED)
  {
    interm.create( pic.chromaFormat, Area( Position(), pic.Y()) );
    ColourSpaceConvert(pic, interm, ipCSC, false);
  }

  const CPelUnitBuf& picC = (ipCSC==IPCOLOURSPACE_UNCHANGED) ? pic : interm;

  // compute actual YUV frame size excluding padding size
  bool is16bit = false;
  bool nonZeroBitDepthShift=false;

  for(uint32_t ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (m_fileBitdepth[ch] > 8)
    {
      is16bit=true;
    }
    if (m_bitdepthShift[ch] != 0)
    {
      nonZeroBitDepthShift=true;
    }
  }

  bool retval = true;
  if (format>=NUM_CHROMA_FORMAT)
  {
    format= picC.chromaFormat;
  }

  PelStorage picZ;
  if (nonZeroBitDepthShift)
  {
    picZ.create( picC.chromaFormat, Area( Position(), picC.Y() ) );
    picZ.copyFrom( picC );

    for(uint32_t comp=0; comp < ::getNumberValidComponents( picZ.chromaFormat ); comp++)
    {
      const ComponentID compID=ComponentID(comp);
      const ChannelType ch=toChannelType(compID);
      const bool b709Compliance = bClipToRec709 && (-m_bitdepthShift[ch] < 0 && m_MSBExtendedBitDepth[ch] >= 8);     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
      const Pel minval = b709Compliance? ((   1 << (m_MSBExtendedBitDepth[ch] - 8))   ) : 0;
      const Pel maxval = b709Compliance? ((0xff << (m_MSBExtendedBitDepth[ch] - 8)) -1) : (1 << m_MSBExtendedBitDepth[ch]) - 1;

      scalePlane( picZ.get(compID), -m_bitdepthShift[ch], minval, maxval);
    }
  }

  const CPelUnitBuf& picO = nonZeroBitDepthShift ? picZ : picC;

  const CPelBuf areaY     = picO.get(COMPONENT_Y);
  const uint32_t    width444  = areaY.width - confLeft - confRight;
  const uint32_t    height444 = areaY.height -  confTop  - confBottom;

  if( subtractConfWindowOffsets )
  {
    orgWidth -= confLeft + confRight;
    orgHeight -= confTop + confBottom;
  }

  if ((width444 == 0) || (height444 == 0))
  {
    msg(WARNING_, "\nWarning: writing %d x %d luma sample output picture!", width444, height444);
  }

  for(uint32_t comp=0; retval && comp < ::getNumberValidComponents(format); comp++)
  {
    const ComponentID compID      = ComponentID(comp);
    const ChannelType ch          = toChannelType(compID);
    const uint32_t    csx         = ::getComponentScaleX(compID, format);
    const uint32_t    csy         = ::getComponentScaleY(compID, format);
    const CPelBuf     area        = picO.get(compID);
    const int         planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;
    if( !writePlane( orgWidth, orgHeight, m_cHandle, area.bufAt( 0, 0 ) + planeOffset, is16bit, area.stride,
                     width444, height444, compID, picO.chromaFormat, format, m_fileBitdepth[ch],
                     bPackedYUVOutputMode ? 1 : 0))
    {
      retval = false;
    }
  }

  return retval;
}

bool VideoIOYuv::write( const CPelUnitBuf& picTop, const CPelUnitBuf& picBottom,
                        const InputColourSpaceConversion ipCSC,
                        const bool bPackedYUVOutputMode,
                        int confLeft, int confRight, int confTop, int confBottom, ChromaFormat format, const bool isTff, const bool bClipToRec709 )
{
  PelStorage intermTop;
  PelStorage intermBottom;

  if( ipCSC != IPCOLOURSPACE_UNCHANGED )
  {
    intermTop   .create( picTop.   chromaFormat, Area( Position(), picTop.   Y()) );
    intermBottom.create( picBottom.chromaFormat, Area( Position(), picBottom.Y()) );
    ColourSpaceConvert( picTop,    intermTop, ipCSC, false);
    ColourSpaceConvert( picBottom, intermBottom, ipCSC, false);
  }
  const CPelUnitBuf& picTopC    = (ipCSC==IPCOLOURSPACE_UNCHANGED) ? picTop    : intermTop;
  const CPelUnitBuf& picBottomC = (ipCSC==IPCOLOURSPACE_UNCHANGED) ? picBottom : intermBottom;

  bool is16bit = false;
  bool nonZeroBitDepthShift=false;

  for(uint32_t ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (m_fileBitdepth[ch] > 8)
    {
      is16bit=true;
    }
    if (m_bitdepthShift[ch] != 0)
    {
      nonZeroBitDepthShift=true;
    }
  }

  PelStorage picTopZ;
  PelStorage picBottomZ;

  for (uint32_t field = 0; field < 2; field++)
  {
    const CPelUnitBuf& picC    = (field == 0) ? picTopC : picBottomC;

    if (format>=NUM_CHROMA_FORMAT)
    {
      format = picC.chromaFormat;
    }

    PelStorage& picZ    = (field == 0) ? picTopZ : picBottomZ;

    if (nonZeroBitDepthShift)
    {
      picZ.create( picC.chromaFormat, Area( Position(), picC.Y() ) );
      picZ.copyFrom( picC );

      for(uint32_t comp=0; comp < ::getNumberValidComponents( picZ.chromaFormat ); comp++)
      {
        const ComponentID compID=ComponentID(comp);
        const ChannelType ch=toChannelType(compID);
        const bool b709Compliance=bClipToRec709 && (-m_bitdepthShift[ch] < 0 && m_MSBExtendedBitDepth[ch] >= 8);     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
        const Pel minval = b709Compliance? ((   1 << (m_MSBExtendedBitDepth[ch] - 8))   ) : 0;
        const Pel maxval = b709Compliance? ((0xff << (m_MSBExtendedBitDepth[ch] - 8)) -1) : (1 << m_MSBExtendedBitDepth[ch]) - 1;

        scalePlane( picZ.get(compID), -m_bitdepthShift[ch], minval, maxval);
      }
    }
  }

  const CPelUnitBuf& picTopO     = nonZeroBitDepthShift ? picTopZ    : picTopC;
  const CPelUnitBuf& picBottomO  = nonZeroBitDepthShift ? picBottomZ : picBottomC;

  bool retval = true;
  CHECK_( picTopO.chromaFormat != picBottomO.chromaFormat, "Incompatible formats of bottom and top fields" );

  const ChromaFormat dstChrFormat = picTopO.chromaFormat;
  for (uint32_t comp = 0; retval && comp < ::getNumberValidComponents(dstChrFormat); comp++)
  {
    const ComponentID compID     = ComponentID(comp);
    const ChannelType ch         = toChannelType(compID);
    const CPelBuf     areaTop    = picTopO.   get( compID );
    const CPelBuf     areaBottom = picBottomO.get( compID );
    const CPelBuf     areaTopY   = picTopO.Y();
    const uint32_t    width444   = areaTopY.width  - (confLeft + confRight);
    const uint32_t    height444  = areaTopY.height - (confTop + confBottom);

    CHECK_(areaTop.width  != areaBottom.width , "Incompatible formats");
    CHECK_(areaTop.height != areaBottom.height, "Incompatible formats");
    CHECK_(areaTop.stride != areaBottom.stride, "Incompatible formats");

    if ((width444 == 0) || (height444 == 0))
    {
      msg(WARNING_, "\nWarning: writing %d x %d luma sample output picture!", width444, height444);
    }

    const uint32_t csx = ::getComponentScaleX(compID, dstChrFormat );
    const uint32_t csy = ::getComponentScaleY(compID, dstChrFormat );
    const int planeOffset  = (confLeft>>csx) + ( confTop>>csy) * areaTop.stride; //offset is for entire frame - round up for top field and down for bottom field

    if (!writeField (m_cHandle,
                     (areaTop.   bufAt(0,0) + planeOffset),
                     (areaBottom.bufAt(0,0) + planeOffset),
                     is16bit,
                     areaTop.stride,
                     width444, height444, compID, dstChrFormat, format, m_fileBitdepth[ch], isTff,
                     bPackedYUVOutputMode ? 1 : 0))
    {
      retval=false;
    }
  }

  return retval;
}


// static member
void VideoIOYuv::ColourSpaceConvert(const CPelUnitBuf &src, PelUnitBuf &dest, const InputColourSpaceConversion conversion, bool bIsForwards)
{
  const ChromaFormat  format       = src.chromaFormat;
  const uint32_t          numValidComp = ::getNumberValidComponents(format);

  switch (conversion)
  {
    case IPCOLOURSPACE_YCbCrtoYYY:
      if (format!=CHROMA_444)
      {
        // only 444 is handled.
        CHECK_( format != CHROMA_444, "Chroma format other than 444 not supported" );
      }

      {
        for(uint32_t comp=0; comp<numValidComp; comp++)
        {
          dest.get( ComponentID(comp) ).copyFrom( src.get( ComponentID(bIsForwards?0:comp) ) );
        }
      }
      break;
    case IPCOLOURSPACE_YCbCrtoYCrCb:
      {
        for(uint32_t comp=0; comp<numValidComp; comp++)
        {
          dest.get( ComponentID((numValidComp-comp)%numValidComp) ).copyFrom( src.get( ComponentID(comp) ) );
        }
      }
      break;

    case IPCOLOURSPACE_RGBtoGBR:
      {
        if (format!=CHROMA_444)
        {
          // only 444 is handled.
          CHECK_(format!=CHROMA_444, "Chroma format other than 444 not supported");
        }

        // channel re-mapping
        for(uint32_t comp=0; comp<numValidComp; comp++)
        {
          const ComponentID compIDsrc=ComponentID((comp+1)%numValidComp);
          const ComponentID compIDdst=ComponentID(comp);

          dest.get( bIsForwards?compIDdst:compIDsrc ).copyFrom( src.get( bIsForwards?compIDsrc:compIDdst ) );
        }
      }
      break;

    case IPCOLOURSPACE_UNCHANGED:
    default:
      {
        for(uint32_t comp=0; comp<numValidComp; comp++)
        {
          dest.get( ComponentID(comp) ).copyFrom( src.get( ComponentID(comp) ) );
        }
      }
      break;
  }
}

bool VideoIOYuv::writeUpscaledPicture( const SPS& sps, const PPS& pps, const CPelUnitBuf& pic, const InputColourSpaceConversion ipCSC, const bool bPackedYUVOutputMode, int outputChoice, ChromaFormat format, const bool bClipToRec709 )
{
  ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  bool ret = false;

  static Window confFullResolution;
  static Window afterScaleWindowFullResolution;

  // decoder does not have information about upscaled picture scaling and conformance windows, store this information when full resolution picutre is encountered
  if( sps.getMaxPicWidthInLumaSamples() == pps.getPicWidthInLumaSamples() && sps.getMaxPicHeightInLumaSamples() == pps.getPicHeightInLumaSamples() )
  {
    afterScaleWindowFullResolution = pps.getScalingWindow();
    afterScaleWindowFullResolution = pps.getConformanceWindow();
  }

  if( outputChoice && ( sps.getMaxPicWidthInLumaSamples() != pic.get( COMPONENT_Y ).width || sps.getMaxPicHeightInLumaSamples() != pic.get( COMPONENT_Y ).height ) )
  {
    if( outputChoice == 2 )
    {
      PelStorage upscaledPic;
      upscaledPic.create( chromaFormatIDC, Area( Position(), Size( sps.getMaxPicWidthInLumaSamples(), sps.getMaxPicHeightInLumaSamples() ) ) );

      int curPicWidth = sps.getMaxPicWidthInLumaSamples()   - SPS::getWinUnitX( sps.getChromaFormatIdc() ) * ( afterScaleWindowFullResolution.getWindowLeftOffset() + afterScaleWindowFullResolution.getWindowRightOffset() );
      int curPicHeight = sps.getMaxPicHeightInLumaSamples() - SPS::getWinUnitY( sps.getChromaFormatIdc() ) * ( afterScaleWindowFullResolution.getWindowTopOffset()  + afterScaleWindowFullResolution.getWindowBottomOffset() );

      const Window& beforeScalingWindow = pps.getScalingWindow();
      int refPicWidth = pps.getPicWidthInLumaSamples()   - SPS::getWinUnitX( sps.getChromaFormatIdc() ) * ( beforeScalingWindow.getWindowLeftOffset() + beforeScalingWindow.getWindowRightOffset() );
      int refPicHeight = pps.getPicHeightInLumaSamples() - SPS::getWinUnitY( sps.getChromaFormatIdc() ) * ( beforeScalingWindow.getWindowTopOffset()  + beforeScalingWindow.getWindowBottomOffset() );

      int xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
      int yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;

      Picture::rescalePicture( std::pair<int, int>( xScale, yScale ), pic, pps.getScalingWindow(), upscaledPic, afterScaleWindowFullResolution, chromaFormatIDC, sps.getBitDepths(), false, false, sps.getHorCollocatedChromaFlag(), sps.getVerCollocatedChromaFlag() );

      ret = write( sps.getMaxPicWidthInLumaSamples(), sps.getMaxPicHeightInLumaSamples(), upscaledPic,
        ipCSC,
        bPackedYUVOutputMode,
        confFullResolution.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
        confFullResolution.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
        confFullResolution.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
        confFullResolution.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
        NUM_CHROMA_FORMAT, bClipToRec709, false );
    }
    else
    {
      const Window &conf = pps.getConformanceWindow();

      ret = write( sps.getMaxPicWidthInLumaSamples(), sps.getMaxPicHeightInLumaSamples(), pic,
        ipCSC,
        bPackedYUVOutputMode,
        conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
        conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
        conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
        conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
        NUM_CHROMA_FORMAT, bClipToRec709, false );
    }
  }
  else
  {
    const Window &conf = pps.getConformanceWindow();

    ret = write( pic.get( COMPONENT_Y ).width, pic.get( COMPONENT_Y ).height, pic,
      ipCSC,
      bPackedYUVOutputMode,
      conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
      conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
      conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
      conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
      NUM_CHROMA_FORMAT, bClipToRec709 );
  }

  return ret;
}

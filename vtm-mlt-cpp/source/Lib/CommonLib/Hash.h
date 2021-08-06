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

 /** \file     Hash.h
     \brief    Hash class (header)
 */


#ifndef __HASH__
#define __HASH__

 // Include files

#include "CommonLib/Buffer.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include <vector>


struct BlockHash
{
  short x;
  short y;
  uint32_t hashValue2;
};

typedef std::vector<BlockHash>::iterator MapIterator;

// ====================================================================================================================
// Class definitions
// ====================================================================================================================


struct TCRCCalculatorLight
{
public:
  TCRCCalculatorLight(uint32_t bits, uint32_t truncPoly);
  ~TCRCCalculatorLight();

public:
  void processData(unsigned char* curData, uint32_t dataLength);
  void reset() { m_remainder = 0; }
  uint32_t getCRC() { return m_remainder & m_finalResultMask; }

private:
  void xInitTable();

private:
  uint32_t m_remainder;
  uint32_t m_truncPoly;
  uint32_t m_bits;
  uint32_t m_table[256];
  uint32_t m_finalResultMask;
};


struct TComHash
{
public:
  TComHash();
  ~TComHash();
  void create(int picWidth, int picHeight);
  void clearAll();
  void addToTable(uint32_t hashValue, const BlockHash& blockHash);
  int count(uint32_t hashValue);
  int count(uint32_t hashValue) const;
  MapIterator getFirstIterator(uint32_t hashValue);
  const MapIterator getFirstIterator(uint32_t hashValue) const;
  bool hasExactMatch(uint32_t hashValue1, uint32_t hashValue2);

  void generateBlock2x2HashValue(const PelUnitBuf &curPicBuf, int picWidth, int picHeight, const BitDepths bitDepths, uint32_t* picBlockHash[2], bool* picBlockSameInfo[3]);
  void generateBlockHashValue(int picWidth, int picHeight, int width, int height, uint32_t* srcPicBlockHash[2], uint32_t* dstPicBlockHash[2], bool* srcPicBlockSameInfo[3], bool* dstPicBlockSameInfo[3]);
  void addToHashMapByRowWithPrecalData(uint32_t* srcHash[2], bool* srcIsSame, int picWidth, int picHeight, int width, int height);
  bool isInitial() { return tableHasContent; }
  void setInitial() { tableHasContent = true; }
  uint16_t* getHashPic(int baseSize) const { return hashPic[floorLog2(baseSize) - 2]; }


public:
  static uint32_t getCRCValue1(unsigned char* p, int length);
  static uint32_t getCRCValue2(unsigned char* p, int length);
  static void getPixelsIn1DCharArrayByBlock2x2(const PelUnitBuf &curPicBuf, unsigned char* pixelsIn1D, int xStart, int yStart, const BitDepths& bitDepths, bool includeAllComponent = true);
  static bool isBlock2x2RowSameValue(unsigned char* p, bool includeAllComponent = true);
  static bool isBlock2x2ColSameValue(unsigned char* p, bool includeAllComponent = true);
  static bool getBlockHashValue(const PelUnitBuf &curPicBuf, int width, int height, int xStart, int yStart, const BitDepths bitDepths, uint32_t& hashValue1, uint32_t& hashValue2);
  static void initBlockSizeToIndex();
  static bool isHorizontalPerfectLuma(const Pel* srcPel, int stride, int width, int height);
  static bool isVerticalPerfectLuma(const Pel* srcPel, int stride, int width, int height);

private:
  std::vector<BlockHash>** m_lookupTable;
  bool tableHasContent;
  uint16_t* hashPic[5];//4x4 ~ 64x64

private:
  static const int m_CRCBits = 16;
  static const int m_blockSizeBits = 3;
  static int m_blockSizeToIndex[65][65];

  static TCRCCalculatorLight m_crcCalculator1;
  static TCRCCalculatorLight m_crcCalculator2;
};

#endif // __HASH__

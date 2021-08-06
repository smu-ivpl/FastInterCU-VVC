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

/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#ifndef __PICTURE__
#define __PICTURE__

#include "CommonDef.h"

#include "Common.h"
#include "Unit.h"
#include "Buffer.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"
#include "Hash.h"
#include "MCTS.h"
#include <deque>

#if ENABLE_SPLIT_PARALLELISM

#define CURR_THREAD_ID -1

class Scheduler
{
public:
  Scheduler();
  ~Scheduler();

#if ENABLE_SPLIT_PARALLELISM
  unsigned getSplitDataId( int jobId = CURR_THREAD_ID ) const;
  unsigned getSplitPicId ( int tId   = CURR_THREAD_ID ) const;
  unsigned getSplitJobId () const;
  void     setSplitJobId ( const int jobId );
  void     startParallel ();
  void     finishParallel();
  void     setSplitThreadId( const int tId = CURR_THREAD_ID );
  unsigned getNumSplitThreads() const { return m_numSplitThreads; };
#endif
  unsigned getDataId     () const;
  bool init              ( const int ctuYsize, const int ctuXsize, const int numWppThreadsRunning, const int numWppExtraLines, const int numSplitThreads );
  int  getNumPicInstances() const;
#if ENABLE_SPLIT_PARALLELISM

  int   m_numSplitThreads;
  bool  m_hasParallelBuffer;
#endif
};
#endif

class SEI;
class AQpLayer;

typedef std::list<SEI*> SEIMessages;



#if ENABLE_SPLIT_PARALLELISM
#define M_BUFS(JID,PID) m_bufs[JID][PID]
#else
#define M_BUFS(JID,PID) m_bufs[PID]
#endif

struct Picture : public UnitArea
{
  uint32_t margin;
  Picture();

  void create( const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned margin, const bool bDecoder, const int layerId, const bool gopBasedTemporalFilterEnabled = false );
  void destroy();

  void createTempBuffers( const unsigned _maxCUSize );
  void destroyTempBuffers();

         PelBuf     getOrigBuf(const CompArea &blk);
  const CPelBuf     getOrigBuf(const CompArea &blk) const;
         PelUnitBuf getOrigBuf(const UnitArea &unit);
  const CPelUnitBuf getOrigBuf(const UnitArea &unit) const;
         PelUnitBuf getOrigBuf();
  const CPelUnitBuf getOrigBuf() const;
         PelBuf     getOrigBuf(const ComponentID compID);
  const CPelBuf     getOrigBuf(const ComponentID compID) const;
         PelUnitBuf getTrueOrigBuf();
  const CPelUnitBuf getTrueOrigBuf() const;
        PelBuf      getTrueOrigBuf(const CompArea &blk);
  const CPelBuf     getTrueOrigBuf(const CompArea &blk) const;

         PelUnitBuf getFilteredOrigBuf();
  const CPelUnitBuf getFilteredOrigBuf() const;
         PelBuf     getFilteredOrigBuf(const CompArea &blk);
  const CPelBuf     getFilteredOrigBuf(const CompArea &blk) const;

         PelBuf     getPredBuf(const CompArea &blk);
  const CPelBuf     getPredBuf(const CompArea &blk) const;
         PelUnitBuf getPredBuf(const UnitArea &unit);
  const CPelUnitBuf getPredBuf(const UnitArea &unit) const;

         PelBuf     getResiBuf(const CompArea &blk);
  const CPelBuf     getResiBuf(const CompArea &blk) const;
         PelUnitBuf getResiBuf(const UnitArea &unit);
  const CPelUnitBuf getResiBuf(const UnitArea &unit) const;

         PelBuf     getRecoBuf(const ComponentID compID, bool wrap=false);
  const CPelBuf     getRecoBuf(const ComponentID compID, bool wrap=false) const;
         PelBuf     getRecoBuf(const CompArea &blk, bool wrap=false);
  const CPelBuf     getRecoBuf(const CompArea &blk, bool wrap=false) const;
         PelUnitBuf getRecoBuf(const UnitArea &unit, bool wrap=false);
  const CPelUnitBuf getRecoBuf(const UnitArea &unit, bool wrap=false) const;
         PelUnitBuf getRecoBuf(bool wrap=false);
  const CPelUnitBuf getRecoBuf(bool wrap=false) const;

         PelBuf     getBuf(const ComponentID compID, const PictureType &type);
  const CPelBuf     getBuf(const ComponentID compID, const PictureType &type) const;
         PelBuf     getBuf(const CompArea &blk,      const PictureType &type);
  const CPelBuf     getBuf(const CompArea &blk,      const PictureType &type) const;
         PelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type);
  const CPelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type) const;

  void extendPicBorder( const PPS *pps );
  void extendWrapBorder( const PPS *pps );
  void finalInit( const VPS* vps, const SPS& sps, const PPS& pps, PicHeader *picHeader, APS** alfApss, APS* lmcsAps, APS* scalingListAps );

  int  getPOC()                               const { return poc; }
  int  getDecodingOrderNumber()               const { return m_decodingOrderNumber; }
  void setDecodingOrderNumber(const int val)        { m_decodingOrderNumber = val;  }
  NalUnitType getPictureType()                const { return m_pictureType;         }
  void setPictureType(const NalUnitType val)        { m_pictureType = val;          }
  void setBorderExtension( bool bFlag)              { m_bIsBorderExtended = bFlag;}
  Pel* getOrigin( const PictureType &type, const ComponentID compID ) const;

  void setLossyQPValue(int i)                 { m_lossyQP = i; }
  int getLossyQPValue()                       const { return m_lossyQP; }
  void      fillSliceLossyLosslessArray(std::vector<uint16_t> sliceLosslessArray, bool mixedLossyLossless);
  bool      losslessSlice(uint32_t sliceIdx)  const { return m_lossylosslessSliceArray[sliceIdx]; }

  int           getSpliceIdx(uint32_t idx) const { return m_spliceIdx[idx]; }
  void          setSpliceIdx(uint32_t idx, int poc) { m_spliceIdx[idx] = poc; }
  void          createSpliceIdx(int nums);
  bool          getSpliceFull();
  static void   sampleRateConv( const std::pair<int, int> scalingRatio, const std::pair<int, int> compScale,
                                const CPelBuf& beforeScale, const int beforeScaleLeftOffset, const int beforeScaleTopOffset,
                                const PelBuf& afterScale, const int afterScaleLeftOffset, const int afterScaleTopOffset,
                                const int bitDepth, const bool useLumaFilter, const bool downsampling,
                                const bool horCollocatedPositionFlag, const bool verCollocatedPositionFlag );

  static void   rescalePicture( const std::pair<int, int> scalingRatio,
                                const CPelUnitBuf& beforeScaling, const Window& scalingWindowBefore,
                                const PelUnitBuf& afterScaling, const Window& scalingWindowAfter,
                                const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling,
                                const bool horCollocatedChromaFlag, const bool verCollocatedChromaFlag );

private:
  Window        m_conformanceWindow;
  Window        m_scalingWindow;
  int           m_decodingOrderNumber;
  NalUnitType   m_pictureType;

public:
  bool m_isSubPicBorderSaved;

  PelStorage m_bufSubPicAbove;
  PelStorage m_bufSubPicBelow;
  PelStorage m_bufSubPicLeft;
  PelStorage m_bufSubPicRight;

  PelStorage m_bufWrapSubPicAbove;
  PelStorage m_bufWrapSubPicBelow;

  void    saveSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);
  void  extendSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);
  void restoreSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);

  bool getSubPicSaved()          { return m_isSubPicBorderSaved; }
  void setSubPicSaved(bool bVal) { m_isSubPicBorderSaved = bVal; }
  bool m_bIsBorderExtended;
  bool m_wrapAroundValid;
  unsigned m_wrapAroundOffset;
  bool referenced;
  bool reconstructed;
  bool neededForOutput;
  bool usedByCurr;
  bool longTerm;
  bool topField;
  bool fieldPic;
  int  m_prevQP[MAX_NUM_CHANNEL_TYPE];
  bool precedingDRAP; // preceding a DRAP picture in decoding order
  bool nonReferencePictureFlag;

  int  poc;
  uint32_t temporalId;
  int      layerId;
  std::vector<SubPic> subPictures;
  int numSlices;
  std::vector<int> sliceSubpicIdx;

  bool subLayerNonReferencePictureDueToSTSA;

  int* m_spliceIdx;
  int  m_ctuNums;
  int m_lossyQP;
  std::vector<bool> m_lossylosslessSliceArray;
  bool interLayerRefPicFlag;


#if ENABLE_SPLIT_PARALLELISM
  PelStorage m_bufs[PARL_SPLIT_MAX_NUM_JOBS][NUM_PIC_TYPES];
#else
  PelStorage m_bufs[NUM_PIC_TYPES];
#endif
  const Picture*           unscaledPic;

  TComHash           m_hashMap;
  TComHash*          getHashMap() { return &m_hashMap; }
  const TComHash*    getHashMap() const { return &m_hashMap; }
  void               addPictureToHashMapForInter();

  CodingStructure*   cs;
  std::deque<Slice*> slices;
  SEIMessages        SEIs;

  uint32_t           getPicWidthInLumaSamples() const                                { return  getRecoBuf( COMPONENT_Y ).width; }
  uint32_t           getPicHeightInLumaSamples() const                               { return  getRecoBuf( COMPONENT_Y ).height; }
  Window&            getConformanceWindow()                                          { return  m_conformanceWindow; }
  const Window&      getConformanceWindow() const                                    { return  m_conformanceWindow; }
  Window&            getScalingWindow()                                              { return  m_scalingWindow; }
  const Window&      getScalingWindow()                                        const { return  m_scalingWindow; }
  bool               isRefScaled( const PPS* pps ) const                             { return  unscaledPic->getPicWidthInLumaSamples()    != pps->getPicWidthInLumaSamples()                ||
                                                                                               unscaledPic->getPicHeightInLumaSamples()   != pps->getPicHeightInLumaSamples()               ||
                                                                                               getScalingWindow().getWindowLeftOffset()   != pps->getScalingWindow().getWindowLeftOffset()  ||
                                                                                               getScalingWindow().getWindowRightOffset()  != pps->getScalingWindow().getWindowRightOffset() ||
                                                                                               getScalingWindow().getWindowTopOffset()    != pps->getScalingWindow().getWindowTopOffset()   ||
                                                                                               getScalingWindow().getWindowBottomOffset() != pps->getScalingWindow().getWindowBottomOffset(); }
  bool               isWrapAroundEnabled( const PPS* pps ) const                     { return  pps->getWrapAroundEnabledFlag() && !isRefScaled( pps ); }

  void         allocateNewSlice();
  Slice        *swapSliceObject(Slice * p, uint32_t i);
  void         clearSliceBuffer();

  MCTSInfo     mctsInfo;
  std::vector<AQpLayer*> aqlayer;

#if !KEEP_PRED_AND_RESI_SIGNALS
private:
  UnitArea m_ctuArea;
#endif

#if ENABLE_SPLIT_PARALLELISM
public:
  void finishParallelPart   ( const UnitArea& ctuArea );
#endif
#if ENABLE_SPLIT_PARALLELISM
public:
  Scheduler                  scheduler;
#endif

public:
  SAOBlkParam    *getSAO(int id = 0)                        { return &m_sao[id][0]; };
  void            resizeSAO(unsigned numEntries, int dstid) { m_sao[dstid].resize(numEntries); }
  void            copySAO(const Picture& src, int dstid)    { std::copy(src.m_sao[0].begin(), src.m_sao[0].end(), m_sao[dstid].begin()); }

#if ENABLE_QPA
  std::vector<double>     m_uEnerHpCtu;                         ///< CTU-wise L2 or squared L1 norm of high-passed luma input
  std::vector<Pel>        m_iOffsetCtu;                         ///< CTU-wise DC offset (later QP index offset) of luma input
 #if ENABLE_QPA_SUB_CTU
  std::vector<int8_t>     m_subCtuQP;                           ///< sub-CTU-wise adapted QPs for delta-QP depth of 1 or more
 #endif
#endif

  std::vector<SAOBlkParam> m_sao[2];

  std::vector<uint8_t> m_alfCtuEnableFlag[MAX_NUM_COMPONENT];
  uint8_t* getAlfCtuEnableFlag( int compIdx ) { return m_alfCtuEnableFlag[compIdx].data(); }
  std::vector<uint8_t>* getAlfCtuEnableFlag() { return m_alfCtuEnableFlag; }
  void resizeAlfCtuEnableFlag( int numEntries )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      m_alfCtuEnableFlag[compIdx].resize( numEntries );
      std::fill( m_alfCtuEnableFlag[compIdx].begin(), m_alfCtuEnableFlag[compIdx].end(), 0 );
    }
  }
  std::vector<short> m_alfCtbFilterIndex;
  short* getAlfCtbFilterIndex() { return m_alfCtbFilterIndex.data(); }
  std::vector<short>& getAlfCtbFilterIndexVec() { return m_alfCtbFilterIndex; }
  void resizeAlfCtbFilterIndex(int numEntries)
  {
    m_alfCtbFilterIndex.resize(numEntries);
    for (int i = 0; i < numEntries; i++)
    {
      m_alfCtbFilterIndex[i] = 0;
    }
  }
  std::vector<uint8_t> m_alfCtuAlternative[MAX_NUM_COMPONENT];
  std::vector<uint8_t>& getAlfCtuAlternative( int compIdx ) { return m_alfCtuAlternative[compIdx]; }
  uint8_t* getAlfCtuAlternativeData( int compIdx ) { return m_alfCtuAlternative[compIdx].data(); }
  void resizeAlfCtuAlternative( int numEntries )
  {
    for( int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      m_alfCtuAlternative[compIdx].resize( numEntries );
      std::fill( m_alfCtuAlternative[compIdx].begin(), m_alfCtuAlternative[compIdx].end(), 0 );
    }
  }
};

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const class SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);

uint32_t calcMD5(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcMD5WithCropping(const CPelUnitBuf &pic, PictureHash &digest, const BitDepths &bitDepths,
                             const int leftOffset, const int rightOffset, const int topOffset, const int bottomOffset);

std::string hashToString(const PictureHash &digest, int numChar);

typedef std::list<Picture*> PicList;

#endif

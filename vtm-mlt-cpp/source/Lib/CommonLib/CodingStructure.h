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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#ifndef __CODINGSTRUCTURE__
#define __CODINGSTRUCTURE__

#include "Unit.h"
#include "Buffer.h"
#include "CommonDef.h"
#include "UnitPartitioner.h"
#include "Slice.h"
#include <vector>


struct Picture;


enum PictureType
{
  PIC_RECONSTRUCTION = 0,
  PIC_ORIGINAL,
  PIC_TRUE_ORIGINAL,
  PIC_FILTERED_ORIGINAL,
  PIC_PREDICTION,
  PIC_RESIDUAL,
  PIC_ORG_RESI,
  PIC_RECON_WRAP,
  PIC_ORIGINAL_INPUT,
  PIC_TRUE_ORIGINAL_INPUT,
  PIC_FILTERED_ORIGINAL_INPUT,
  NUM_PIC_TYPES
};
extern XUCache g_globalUnitCache;

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:

  UnitArea         area;

  Picture         *picture;
  CodingStructure *parent;
  CodingStructure *bestCS;
  Slice           *slice;

  UnitScale        unitScale[MAX_NUM_COMPONENT];

  int         baseQP;
  int         prevQP[MAX_NUM_CHANNEL_TYPE];
  int         currQP[MAX_NUM_CHANNEL_TYPE];
  int         chromaQpAdj;
  const SPS *sps;
  const PPS *pps;
  PicHeader *picHeader;
  APS*       alfApss[ALF_CTB_MAX_NUM_APS];
  APS *      lmcsAps;
  APS *      scalinglistAps;
  const VPS *vps;
  const PreCalcValues* pcv;

  CodingStructure(CUCache&, PUCache&, TUCache&);

  void create(const UnitArea &_unit, const bool isTopLayer, const bool isPLTused);
  void create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer, const bool isPLTused);

  void destroy();
  void releaseIntermediateData();

  void rebindPicBufs();

  void createCoeffs(const bool isPLTused);
  void destroyCoeffs();

  void allocateVectorsAtPicLevel();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

  bool isDecomp (const Position &pos, const ChannelType _chType) const;
  bool isDecomp (const Position &pos, const ChannelType _chType);
  void setDecomp(const CompArea &area, const bool _isCoded = true);
  void setDecomp(const UnitArea &area, const bool _isCoded = true);

  const CodingUnit     *getCU(const Position &pos, const ChannelType _chType) const;
  const PredictionUnit *getPU(const Position &pos, const ChannelType _chType) const;
  const TransformUnit  *getTU(const Position &pos, const ChannelType _chType, const int subTuIdx = -1) const;

  CodingUnit     *getCU(const Position &pos, const ChannelType _chType);
  CodingUnit     *getLumaCU( const Position &pos );
  PredictionUnit *getPU(const Position &pos, const ChannelType _chType);
  TransformUnit  *getTU(const Position &pos, const ChannelType _chType, const int subTuIdx = -1);

  const CodingUnit     *getCU(const ChannelType &_chType) const { return getCU(area.blocks[_chType].pos(), _chType); }
  const PredictionUnit *getPU(const ChannelType &_chType) const { return getPU(area.blocks[_chType].pos(), _chType); }
  const TransformUnit  *getTU(const ChannelType &_chType) const { return getTU(area.blocks[_chType].pos(), _chType); }

  CodingUnit     *getCU(const ChannelType &_chType ) { return getCU(area.blocks[_chType].pos(), _chType); }
  PredictionUnit *getPU(const ChannelType &_chType ) { return getPU(area.blocks[_chType].pos(), _chType); }
  TransformUnit  *getTU(const ChannelType &_chType ) { return getTU(area.blocks[_chType].pos(), _chType); }

  const CodingUnit     *getCURestricted(const Position &pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType) const;
  const CodingUnit     *getCURestricted(const Position &pos, const CodingUnit& curCu,                               const ChannelType _chType) const;
  const PredictionUnit *getPURestricted(const Position &pos, const PredictionUnit& curPu,                           const ChannelType _chType) const;
  const TransformUnit  *getTURestricted(const Position &pos, const TransformUnit& curTu,                            const ChannelType _chType) const;

  CodingUnit&     addCU(const UnitArea &unit, const ChannelType _chType);
  PredictionUnit& addPU(const UnitArea &unit, const ChannelType _chType);
  TransformUnit&  addTU(const UnitArea &unit, const ChannelType _chType);
  void            addEmptyTUs(Partitioner &partitioner);

  CUTraverser     traverseCUs(const UnitArea& _unit, const ChannelType _chType);
  PUTraverser     traversePUs(const UnitArea& _unit, const ChannelType _chType);
  TUTraverser     traverseTUs(const UnitArea& _unit, const ChannelType _chType);

  cCUTraverser    traverseCUs(const UnitArea& _unit, const ChannelType _chType) const;
  cPUTraverser    traversePUs(const UnitArea& _unit, const ChannelType _chType) const;
  cTUTraverser    traverseTUs(const UnitArea& _unit, const ChannelType _chType) const;
  // ---------------------------------------------------------------------------
  // encoding search utilities
  // ---------------------------------------------------------------------------

  static_vector<double, NUM_ENC_FEATURES> features;

  double      cost;
  bool        useDbCost;
  double      costDbOffset;
  double      lumaCost;
  uint64_t      fracBits;
  Distortion  dist;
  Distortion  interHad;
  TreeType    treeType; //because partitioner can not go deep to tu and cu coding (e.g., addCU()), need another variable for indicating treeType
  ModeType    modeType;

  void initStructData  (const int &QP = MAX_INT, const bool &skipMotBuf = false);
  void initSubStructure(      CodingStructure& cs, const ChannelType chType, const UnitArea &subArea, const bool &isTuEnc);

  void copyStructure   (const CodingStructure& cs, const ChannelType chType, const bool copyTUs = false, const bool copyRecoBuffer = false);
  void useSubStructure (const CodingStructure& cs, const ChannelType chType, const UnitArea &subArea, const bool cpyPred, const bool cpyReco, const bool cpyOrgResi, const bool cpyResi, const bool updateCost);
  void useSubStructure (const CodingStructure& cs, const ChannelType chType,                          const bool cpyPred, const bool cpyReco, const bool cpyOrgResi, const bool cpyResi, const bool updateCost) { useSubStructure(cs, chType, cs.area, cpyPred, cpyReco, cpyOrgResi, cpyResi, updateCost); }

  void clearTUs();
  void clearPUs();
  void clearCUs();
  const int signalModeCons( const PartSplit split, Partitioner &partitioner, const ModeType modeTypeParent ) const;
  void clearCuPuTuIdxMap  ( const UnitArea &_area, uint32_t numCu, uint32_t numPu, uint32_t numTu, uint32_t* pOffset );
  void getNumCuPuTuOffset ( uint32_t* pArray )
  {
    pArray[0] = m_numCUs;     pArray[1] = m_numPUs;     pArray[2] = m_numTUs;
    pArray[3] = m_offsets[0]; pArray[4] = m_offsets[1]; pArray[5] = m_offsets[2];
  }


private:
  void createInternals(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused);

public:

  std::vector<    CodingUnit*> cus;
  std::vector<PredictionUnit*> pus;
  std::vector< TransformUnit*> tus;

  LutMotionCand motionLut;

  void addMiToLut(static_vector<MotionInfo, MAX_NUM_HMVP_CANDS>& lut, const MotionInfo &mi);

  PLTBuf prevPLT;
  void resetPrevPLT(PLTBuf& prevPLT);
  void reorderPrevPLT(PLTBuf& prevPLT, uint8_t curPLTSize[MAX_NUM_CHANNEL_TYPE], Pel curPLT[MAX_NUM_COMPONENT][MAXPLTSIZE], bool reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE], uint32_t compBegin, uint32_t numComp, bool jointPLT);
  void setPrevPLT(PLTBuf predictor);
  void storePrevPLT(PLTBuf& predictor);
private:

  // needed for TU encoding
  bool m_isTuEnc;

  unsigned *m_cuIdx   [MAX_NUM_CHANNEL_TYPE];
  unsigned *m_puIdx   [MAX_NUM_CHANNEL_TYPE];
  unsigned *m_tuIdx   [MAX_NUM_CHANNEL_TYPE];
  bool     *m_isDecomp[MAX_NUM_CHANNEL_TYPE];

  unsigned m_numCUs;
  unsigned m_numPUs;
  unsigned m_numTUs;

  CUCache& m_cuCache;
  PUCache& m_puCache;
  TUCache& m_tuCache;

  std::vector<SAOBlkParam> m_sao;

  PelStorage m_pred;
  PelStorage m_resi;
  PelStorage m_reco;
  PelStorage m_orgr;

  TCoeff *m_coeffs [ MAX_NUM_COMPONENT ];
  Pel    *m_pcmbuf [ MAX_NUM_COMPONENT ];
  bool   *m_runType[ MAX_NUM_CHANNEL_TYPE ];
  int     m_offsets[ MAX_NUM_COMPONENT ];

  MotionInfo *m_motionBuf;

public:
  CodingStructure *bestParent;
  double        tmpColorSpaceCost;
  bool          firstColorSpaceSelected;
  double        tmpColorSpaceIntraCost[2];
  bool          firstColorSpaceTestOnly;
  bool resetIBCBuffer;

  MotionBuf getMotionBuf( const     Area& _area );
  MotionBuf getMotionBuf( const UnitArea& _area ) { return getMotionBuf( _area.Y() ); }
  MotionBuf getMotionBuf()                        { return getMotionBuf(  area.Y() ); }

  const CMotionBuf getMotionBuf( const     Area& _area ) const;
  const CMotionBuf getMotionBuf( const UnitArea& _area ) const { return getMotionBuf( _area.Y() ); }
  const CMotionBuf getMotionBuf()                        const { return getMotionBuf(  area.Y() ); }

  MotionInfo& getMotionInfo( const Position& pos );
  const MotionInfo& getMotionInfo( const Position& pos ) const;


public:
  // ---------------------------------------------------------------------------
  // temporary (shadowed) data accessors
  // ---------------------------------------------------------------------------
         PelBuf       getPredBuf(const CompArea &blk);
  const CPelBuf       getPredBuf(const CompArea &blk) const;
         PelUnitBuf   getPredBuf(const UnitArea &unit);
  const CPelUnitBuf   getPredBuf(const UnitArea &unit) const;

         PelBuf       getResiBuf(const CompArea &blk);
  const CPelBuf       getResiBuf(const CompArea &blk) const;
         PelUnitBuf   getResiBuf(const UnitArea &unit);
  const CPelUnitBuf   getResiBuf(const UnitArea &unit) const;

         PelBuf       getRecoBuf(const CompArea &blk);
  const CPelBuf       getRecoBuf(const CompArea &blk) const;
         PelUnitBuf   getRecoBuf(const UnitArea &unit);
  const CPelUnitBuf   getRecoBuf(const UnitArea &unit) const;
         PelUnitBuf&  getRecoBufRef() { return m_reco; }

         PelBuf       getOrgResiBuf(const CompArea &blk);
  const CPelBuf       getOrgResiBuf(const CompArea &blk) const;
         PelUnitBuf   getOrgResiBuf(const UnitArea &unit);
  const CPelUnitBuf   getOrgResiBuf(const UnitArea &unit) const;

         PelBuf       getOrgBuf(const CompArea &blk);
  const CPelBuf       getOrgBuf(const CompArea &blk) const;
         PelUnitBuf   getOrgBuf(const UnitArea &unit);
  const CPelUnitBuf   getOrgBuf(const UnitArea &unit) const;

         PelBuf       getOrgBuf(const ComponentID &compID);
  const CPelBuf       getOrgBuf(const ComponentID &compID) const;
         PelUnitBuf   getOrgBuf();
  const CPelUnitBuf   getOrgBuf() const;


  // pred buffer
         PelBuf       getPredBuf(const ComponentID &compID)       { return m_pred.get(compID); }
  const CPelBuf       getPredBuf(const ComponentID &compID) const { return m_pred.get(compID); }
         PelUnitBuf   getPredBuf()                                { return m_pred; }
  const CPelUnitBuf   getPredBuf()                          const { return m_pred; }

  // resi buffer
         PelBuf       getResiBuf(const ComponentID compID)        { return m_resi.get(compID); }
  const CPelBuf       getResiBuf(const ComponentID compID)  const { return m_resi.get(compID); }
         PelUnitBuf   getResiBuf()                                { return m_resi; }
  const CPelUnitBuf   getResiBuf()                          const { return m_resi; }

  // org-resi buffer
         PelBuf       getOrgResiBuf(const ComponentID &compID)       { return m_orgr.get(compID); }
  const CPelBuf       getOrgResiBuf(const ComponentID &compID) const { return m_orgr.get(compID); }
         PelUnitBuf   getOrgResiBuf()                                { return m_orgr; }
  const CPelUnitBuf   getOrgResiBuf()                          const { return m_orgr; }

  // reco buffer
         PelBuf       getRecoBuf(const ComponentID compID)         { return m_reco.get(compID); }
  const CPelBuf       getRecoBuf(const ComponentID compID)   const { return m_reco.get(compID); }
         PelUnitBuf   getRecoBuf()                                 { return m_reco; }
  const CPelUnitBuf   getRecoBuf()                           const { return m_reco; }

private:

  inline        PelBuf       getBuf(const CompArea &blk,  const PictureType &type);
  inline const CPelBuf       getBuf(const CompArea &blk,  const PictureType &type) const;
  inline        PelUnitBuf   getBuf(const UnitArea &unit, const PictureType &type);
  inline const CPelUnitBuf   getBuf(const UnitArea &unit, const PictureType &type) const;
};


static inline uint32_t getNumberValidTBlocks(const PreCalcValues& pcv) { return (pcv.chrFormat==CHROMA_400) ? 1 : ( pcv.multiBlock422 ? MAX_NUM_TBLOCKS : MAX_NUM_COMPONENT ); }

#endif


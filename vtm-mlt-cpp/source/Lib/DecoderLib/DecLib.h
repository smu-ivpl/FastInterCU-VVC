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

/** \file     DecLib.h
    \brief    decoder class (header)
*/

#ifndef __DECLIB__
#define __DECLIB__

#include "DecSlice.h"
#include "CABACReader.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CacheModel.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Reshape.h"

class InputNALUnit;

//! \ingroup DecoderLib
//! \{

bool tryDecodePicture( Picture* pcPic, const int expectedPoc, const std::string& bitstreamFileName, ParameterSetMap<APS> *apsMap = nullptr, bool bDecodeUntilPocFound = false, int debugCTU = -1, int debugPOC = -1 );
// Class definition
// ====================================================================================================================

/// decoder class
class DecLib
{
private:
  int                     m_iMaxRefPicNum;
  bool m_isFirstGeneralHrd;
  GeneralHrdParams        m_prevGeneralHrdParams;

  int                     m_prevGDRInSameLayerPOC[MAX_VPS_LAYERS]; ///< POC number of the latest GDR picture
  NalUnitType             m_associatedIRAPType[MAX_VPS_LAYERS]; ///< NAL unit type of the previous IRAP picture
  int                     m_pocCRA[MAX_VPS_LAYERS];            ///< POC number of the previous CRA picture
  int                     m_associatedIRAPDecodingOrderNumber[MAX_VPS_LAYERS]; ///< Decoding order number of the previous IRAP picture
  int                     m_decodingOrderCounter;
  int                     m_puCounter;
  bool                    m_seiInclusionFlag;
  int                     m_prevGDRSubpicPOC[MAX_VPS_LAYERS][MAX_NUM_SUB_PICS];
  int                     m_prevIRAPSubpicPOC[MAX_VPS_LAYERS][MAX_NUM_SUB_PICS];
  NalUnitType             m_prevIRAPSubpicType[MAX_VPS_LAYERS][MAX_NUM_SUB_PICS];
  int                     m_prevIRAPSubpicDecOrderNo[MAX_VPS_LAYERS][MAX_NUM_SUB_PICS];
  int                     m_pocRandomAccess;   ///< POC number of the random access point (the first IDR or CRA picture)
  int                     m_lastRasPoc;
  bool                    m_prevEOS[MAX_VPS_LAYERS];

  PicList                 m_cListPic;         //  Dynamic buffer
  ParameterSetManager     m_parameterSetManager;  // storage for parameter sets
  PicHeader               m_picHeader;            // picture header
  Slice*                  m_apcSlicePilot;


  SEIMessages             m_SEIs; ///< List of SEI messages that have been received before the first slice and between slices, excluding prefix SEIs...


  // functional classes
  IntraPrediction         m_cIntraPred;
  InterPrediction         m_cInterPred;
  TrQuant                 m_cTrQuant;
  DecSlice                m_cSliceDecoder;
  TrQuant                 m_cTrQuantScalingList;
  DecCu                   m_cCuDecoder;
  HLSyntaxReader          m_HLSReader;
  CABACDecoder            m_CABACDecoder;
  SEIReader               m_seiReader;
#if JVET_S0257_DUMP_360SEI_MESSAGE
  SeiCfgFileDump          m_seiCfgDump;
#endif
  LoopFilter              m_cLoopFilter;
  SampleAdaptiveOffset    m_cSAO;
  AdaptiveLoopFilter      m_cALF;
  Reshape                 m_cReshaper;                        ///< reshaper class
  HRD                     m_HRD;
  // decoder side RD cost computation
  RdCost                  m_cRdCost;                      ///< RD cost computation class
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel              m_cacheModel;
#endif
  bool isRandomAccessSkipPicture(int& iSkipFrame,  int& iPOCLastDisplay);
  Picture*                m_pcPic;
  uint32_t                m_uiSliceSegmentIdx;
  uint32_t                m_prevLayerID;
  int                     m_prevPOC;
  int                     m_prevPicPOC;
  int                     m_prevTid0POC;
  bool                    m_bFirstSliceInPicture;
  bool                    m_firstSliceInSequence[MAX_VPS_LAYERS];
  bool                    m_firstSliceInBitstream;
  bool                    m_isFirstAuInCvs;
  bool                    m_accessUnitEos[MAX_VPS_LAYERS];
  bool                    m_prevSliceSkipped;
  int                     m_skippedPOC;
  int                     m_lastPOCNoOutputPriorPics;
  bool                    m_isNoOutputPriorPics;
  bool                    m_lastNoOutputBeforeRecoveryFlag[MAX_VPS_LAYERS];    //value of variable NoOutputBeforeRecoveryFlag of the assocated CRA/GDR pic
  int                     m_sliceLmcsApsId;         //value of LmcsApsId, constraint is same id for all slices in one picture
  std::ostream           *m_pDecodedSEIOutputStream;
  uint32_t                m_audIrapOrGdrAuFlag;
#if JVET_S0257_DUMP_360SEI_MESSAGE
  std::string             m_decoded360SeiDumpFileName;
#endif

  int                     m_decodedPictureHashSEIEnabled;  ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  uint32_t                m_numberOfChecksumErrorsDetected;

  bool                    m_warningMessageSkipPicture;

  std::list<InputNALUnit*> m_prefixSEINALUs; /// Buffered up prefix SEI NAL Units.
  int                     m_debugPOC;
  int                     m_debugCTU;

  struct AccessUnitInfo
  {
    NalUnitType     m_nalUnitType; ///< nal_unit_type
    uint32_t        m_temporalId;  ///< temporal_id
    uint32_t        m_nuhLayerId;  ///< nuh_layer_id
  };
  std::vector<AccessUnitInfo> m_accessUnitNals;
  struct AccessUnitPicInfo
  {
    NalUnitType     m_nalUnitType; ///< nal_unit_type
    uint32_t        m_temporalId;  ///< temporal_id
    uint32_t        m_nuhLayerId;  ///< nuh_layer_id
    int             m_POC;
  };
  std::vector<AccessUnitPicInfo> m_accessUnitPicInfo;
  std::vector<AccessUnitPicInfo> m_firstAccessUnitPicInfo;
  struct NalUnitInfo
  {
    NalUnitType     m_nalUnitType; ///< nal_unit_type
    uint32_t        m_nuhLayerId;  ///< nuh_layer_id
    uint32_t        m_firstCTUinSlice; /// the first CTU in slice, specified with raster scan order ctu address
    int             m_POC;             /// the picture order
  };
  std::vector<NalUnitInfo> m_nalUnitInfo[MAX_VPS_LAYERS];
  std::vector<int> m_accessUnitApsNals;
  std::vector<int> m_accessUnitSeiTids;
  std::vector<bool> m_accessUnitNoOutputPriorPicFlags;

  // NAL unit type, layer ID, and SEI payloadType
  std::vector<std::tuple<NalUnitType, int, SEI::PayloadType>> m_accessUnitSeiPayLoadTypes;

  std::vector<NalUnitType> m_pictureUnitNals;
  std::list<InputNALUnit*> m_pictureSeiNalus; 

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  OPI*                    m_opi;
  bool                    m_mTidExternalSet;
  bool                    m_mTidOpiSet;
  bool                    m_tOlsIdxTidExternalSet;
  bool                    m_tOlsIdxTidOpiSet;
#endif
  VPS*                    m_vps;
  int                     m_maxDecSubPicIdx;
  int                     m_maxDecSliceAddrInSubPic;
  int                     m_clsVPSid;

public:
  int                     m_targetSubPicIdx;

  DCI*                    m_dci;
  ParameterSetMap<APS>*   m_apsMapEnc;
public:
  DecLib();
  virtual ~DecLib();

  void  create  ();
  void  destroy ();

  void  setDecodedPictureHashSEIEnabled(int enabled) { m_decodedPictureHashSEIEnabled=enabled; }

  void  init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    const std::string& cacheCfgFileName
#endif
  );
  bool  decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay, int iTargetOlsIdx);
  void  deletePicBuffer();

  void  executeLoopFilters();
  void finishPicture(int &poc, PicList *&rpcListPic, MsgLevel msgl = INFO_, bool associatedWithNewClvs = false);
  void  finishPictureLight(int& poc, PicList*& rpcListPic );
  void  checkNoOutputPriorPics (PicList* rpcListPic);
  void  checkNalUnitConstraints( uint32_t naluType );
  void  checkPicTypeAfterEos();
  void  updateAssociatedIRAP();
  void  updatePrevGDRInSameLayer();
  void  updatePrevIRAPAndGDRSubpic();

#if JVET_S0078_NOOUTPUTPRIORPICFLAG
  bool  getAudIrapOrGdrAuFlag() const       { return m_audIrapOrGdrAuFlag;  }
#endif
  bool  getNoOutputPriorPicsFlag () const   { return m_isNoOutputPriorPics; }
  void  setNoOutputPriorPicsFlag (bool val) { m_isNoOutputPriorPics = val; }
  void  setFirstSliceInPicture (bool val)  { m_bFirstSliceInPicture = val; }
  bool  getFirstSliceInPicture () const  { return m_bFirstSliceInPicture; }
  bool  getFirstSliceInSequence(int layerId) const { return m_firstSliceInSequence[layerId]; }
  void  setFirstSliceInSequence(bool val, int layerId) { m_firstSliceInSequence[layerId] = val; }
  void  setDecodedSEIMessageOutputStream(std::ostream *pOpStream) { m_pDecodedSEIOutputStream = pOpStream; }
#if JVET_S0257_DUMP_360SEI_MESSAGE
  void  setDecoded360SEIMessageFileName(std::string &Dump360SeiFileName) { m_decoded360SeiDumpFileName = Dump360SeiFileName; }
#endif
  uint32_t  getNumberOfChecksumErrorsDetected() const { return m_numberOfChecksumErrorsDetected; }

  int  getDebugCTU( )               const { return m_debugCTU; }
  void setDebugCTU( int debugCTU )        { m_debugCTU = debugCTU; }
  int  getDebugPOC( )               const { return m_debugPOC; };
  void setDebugPOC( int debugPOC )        { m_debugPOC = debugPOC; };
  void resetAccessUnitNals()              { m_accessUnitNals.clear();    }
  void resetAccessUnitPicInfo()              { m_accessUnitPicInfo.clear();    }
  void resetAccessUnitApsNals()           { m_accessUnitApsNals.clear(); }
  void resetAccessUnitSeiTids()           { m_accessUnitSeiTids.clear(); }
  void resetAudIrapOrGdrAuFlag()          { m_audIrapOrGdrAuFlag = false; }
  void resetAccessUnitEos()               { memset(m_accessUnitEos, false, sizeof(m_accessUnitEos)); }
  void checkTidLayerIdInAccessUnit();
  void resetAccessUnitSeiPayLoadTypes()   { m_accessUnitSeiPayLoadTypes.clear(); }
  void checkSEIInAccessUnit();
  void checkLayerIdIncludedInCvss();
  void CheckNoOutputPriorPicFlagsInAccessUnit();
  void resetAccessUnitNoOutputPriorPicFlags() { m_accessUnitNoOutputPriorPicFlags.clear(); }
  void checkSeiInPictureUnit();
  void resetPictureSeiNalus();
  bool isSliceNaluFirstInAU( bool newPicture, InputNALUnit &nalu );

  void checkAPSInPictureUnit();
  void resetPictureUnitNals() { m_pictureUnitNals.clear(); }

  const VPS* getVPS()                     { return m_vps; }
  void deriveTargetOutputLayerSet( const int targetOlsIdx ) { if( m_vps != nullptr ) m_vps->deriveTargetOutputLayerSet( targetOlsIdx ); }

  void  initScalingList()
  {
    m_cTrQuantScalingList.init(nullptr, MAX_TB_SIZEY, false, false, false, false);
  }

  void  setAPSMapEnc( ParameterSetMap<APS>* apsMap ) { m_apsMapEnc = apsMap;  }
  bool  isNewPicture( std::ifstream *bitstreamFile, class InputByteStream *bytestream );
  bool  isNewAccessUnit( bool newPicture, std::ifstream *bitstreamFile, class InputByteStream *bytestream );

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  bool      getHTidExternalSetFlag()               const { return m_mTidExternalSet; }
  void      setHTidExternalSetFlag(bool mTidExternalSet)  { m_mTidExternalSet = mTidExternalSet; }
  bool      getHTidOpiSetFlag()               const { return m_mTidOpiSet; }
  void      setHTidOpiSetFlag(bool mTidOpiSet)  { m_mTidOpiSet = mTidOpiSet; }
  bool      getTOlsIdxExternalFlag()               const { return m_tOlsIdxTidExternalSet; }
  void      setTOlsIdxExternalFlag (bool tOlsIdxExternalSet)  { m_tOlsIdxTidExternalSet = tOlsIdxExternalSet; }
  bool      getTOlsIdxOpiFlag()               const { return m_tOlsIdxTidOpiSet; }
  void      setTOlsIdxOpiFlag(bool tOlsIdxOpiSet)  { m_tOlsIdxTidOpiSet = tOlsIdxOpiSet; }
  const OPI* getOPI()                     { return m_opi; }
#endif

protected:
  void  xUpdateRasInit(Slice* slice);

  Picture * xGetNewPicBuffer( const SPS &sps, const PPS &pps, const uint32_t temporalLayer, const int layerId );
  void  xCreateLostPicture( int iLostPOC, const int layerId );
  void  xCreateUnavailablePicture( const PPS *pps, const int iUnavailablePoc, const bool longTermFlag, const int temporalId, const int layerId, const bool interLayerRefPicFlag );
  void  checkParameterSetsInclusionSEIconstraints(const InputNALUnit nalu);
  void  xActivateParameterSets( const InputNALUnit nalu );
  void  xCheckParameterSetConstraints( const int layerId );
  void      xDecodePicHeader( InputNALUnit& nalu );
  bool      xDecodeSlice(InputNALUnit &nalu, int &iSkipFrame, int iPOCLastDisplay);
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  void      xDecodeOPI( InputNALUnit& nalu );
#endif
  void      xDecodeVPS( InputNALUnit& nalu );
  void      xDecodeDCI( InputNALUnit& nalu );
  void      xDecodeSPS( InputNALUnit& nalu );
  void      xDecodePPS( InputNALUnit& nalu );
  void      xDecodeAPS(InputNALUnit& nalu);
  void      xUpdatePreviousTid0POC(Slice *pSlice)
  {
    if( (pSlice->getTLayer() == 0) && (pSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL) && (pSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL) && !pSlice->getPicHeader()->getNonReferencePictureFlag() )
    { 
      m_prevTid0POC = pSlice->getPOC(); 
    }  
  }
  void      xParsePrefixSEImessages();
  void      xParsePrefixSEIsForUnknownVCLNal();
  void      xCheckPrefixSEIMessages( SEIMessages& prefixSEIs );

  void  xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType );
  void     xCheckMixedNalUnit(Slice* pcSlice, SPS *sps, InputNALUnit &nalu);
};// END CLASS DEFINITION DecLib


//! \}

#endif // __DECTOP__


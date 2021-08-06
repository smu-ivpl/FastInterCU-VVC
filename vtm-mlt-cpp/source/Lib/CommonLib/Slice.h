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

/** \file     Slice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __SLICE__
#define __SLICE__

#include <cstring>
#include <list>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "Rom.h"
#include "ChromaFormat.h"
#include "Common.h"
#include "HRD.h"
#include <unordered_map>
#include "AlfParameters.h"

//! \ingroup CommonLib
//! \{
#include "CommonLib/MotionInfo.h"
struct MotionInfo;


struct Picture;
class Pic;
class TrQuant;
// ====================================================================================================================
// Constants
// ====================================================================================================================
class PreCalcValues;
static const uint32_t REF_PIC_LIST_NUM_IDX=32;

typedef std::list<Picture*> PicList;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct DpbParameters
{
  int m_maxDecPicBuffering[MAX_TLAYER] = { 0 };
  int m_maxNumReorderPics[MAX_TLAYER] = { 0 };
  int m_maxLatencyIncreasePlus1[MAX_TLAYER] = { 0 };
};

class ReferencePictureList
{
private:
  int   m_numberOfShorttermPictures;
  int   m_numberOfLongtermPictures;
  int   m_isLongtermRefPic[MAX_NUM_REF_PICS];
  int   m_refPicIdentifier[MAX_NUM_REF_PICS];  //This can be delta POC for STRP or POC LSB for LTRP
  int   m_POC[MAX_NUM_REF_PICS];
  int   m_numberOfActivePictures;
  bool  m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];
  int   m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  bool  m_ltrp_in_slice_header_flag;
  bool  m_interLayerPresentFlag;
  bool  m_isInterLayerRefPic[MAX_NUM_REF_PICS];
  int   m_interLayerRefPicIdx[MAX_NUM_REF_PICS];
  int   m_numberOfInterLayerPictures;

public:
  ReferencePictureList( const bool interLayerPicPresentFlag = false );
  virtual ~ReferencePictureList();

  void    setRefPicIdentifier( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx );
  int     getRefPicIdentifier(int idx) const;
  bool    isRefPicLongterm(int idx) const;

  void    setNumberOfShorttermPictures(int numberOfStrp);
  int     getNumberOfShorttermPictures() const;

  void    setNumberOfLongtermPictures(int numberOfLtrp);
  int     getNumberOfLongtermPictures() const;

  void    setLtrpInSliceHeaderFlag(bool flag) { m_ltrp_in_slice_header_flag = flag; }
  bool    getLtrpInSliceHeaderFlag() const { return m_ltrp_in_slice_header_flag; }

  void    setNumberOfInterLayerPictures( const int numberOfIlrp ) { m_numberOfInterLayerPictures = numberOfIlrp; }
  int     getNumberOfInterLayerPictures() const { return m_numberOfInterLayerPictures; }

  int     getNumRefEntries() const { return m_numberOfShorttermPictures + m_numberOfLongtermPictures + m_numberOfInterLayerPictures; }

  void    setPOC(int idx, int POC);
  int     getPOC(int idx) const;

  void    setNumberOfActivePictures(int numberOfLtrp);
  int     getNumberOfActivePictures() const;

  int     getDeltaPocMSBCycleLT(int i) const { return m_deltaPOCMSBCycleLT[i]; }
  void    setDeltaPocMSBCycleLT(int i, int x) { m_deltaPOCMSBCycleLT[i] = x; }
  bool    getDeltaPocMSBPresentFlag(int i) const { return m_deltaPocMSBPresentFlag[i]; }
  void    setDeltaPocMSBPresentFlag(int i, bool x) { m_deltaPocMSBPresentFlag[i] = x; }

  void    printRefPicInfo() const;

  bool      getInterLayerPresentFlag()                   const { return m_interLayerPresentFlag; }
  void      setInterLayerPresentFlag( bool b )                 { m_interLayerPresentFlag = b; }
  bool      isInterLayerRefPic( int idx )                const { return m_isInterLayerRefPic[idx]; }
  int       getInterLayerRefPicIdx( int idx )            const { return m_interLayerRefPicIdx[idx]; }
  void      setInterLayerRefPicIdx( int idx, int layerIdc )    { m_interLayerRefPicIdx[idx] = layerIdc; }
};

/// Reference Picture List set class
class RPLList
{
private:
  std::vector<ReferencePictureList> m_referencePictureLists;

public:
  RPLList() { }
  virtual                        ~RPLList() { }

  void                           create(int numberOfEntries) { m_referencePictureLists.resize(numberOfEntries); }
  void                           destroy() { }


  ReferencePictureList*          getReferencePictureList(int referencePictureListIdx) { return &m_referencePictureLists[referencePictureListIdx]; }
  const ReferencePictureList*    getReferencePictureList(int referencePictureListIdx) const { return &m_referencePictureLists[referencePictureListIdx]; }

  int                            getNumberOfReferencePictureLists() const { return int(m_referencePictureLists.size()); }
};

/// SCALING_LIST class
class ScalingList
{
public:
             ScalingList();
  virtual    ~ScalingList()                                                 { }
  
  int*       getScalingListAddress(uint32_t scalingListId)                    { return &(m_scalingListCoef[scalingListId][0]);            } //!< get matrix coefficient
  const int* getScalingListAddress(uint32_t scalingListId) const              { return &(m_scalingListCoef[scalingListId][0]);            } //!< get matrix coefficient
  void       checkPredMode(uint32_t scalingListId);

  void       setRefMatrixId(uint32_t scalingListId, uint32_t u)               { m_refMatrixId[scalingListId] = u;                         } //!< set reference matrix ID
  uint32_t       getRefMatrixId(uint32_t scalingListId) const                 { return m_refMatrixId[scalingListId];                      } //!< get reference matrix ID

  static const int* getScalingListDefaultAddress(uint32_t scalinListId);                                                                           //!< get default matrix coefficient
  void       processDefaultMatrix(uint32_t scalinListId);

  void       setScalingListDC(uint32_t scalinListId, uint32_t u)              { m_scalingListDC[scalinListId] = u;                        } //!< set DC value
  int        getScalingListDC(uint32_t scalinListId) const                    { return m_scalingListDC[scalinListId];                     } //!< get DC value

  void       setScalingListCopyModeFlag(uint32_t scalinListId, bool bIsCopy)  { m_scalingListPredModeFlagIsCopy[scalinListId] = bIsCopy;  }
  bool       getScalingListCopyModeFlag(uint32_t scalinListId) const          { return m_scalingListPredModeFlagIsCopy[scalinListId];     } //getScalingListPredModeFlag
  void       processRefMatrix(uint32_t scalingListId, uint32_t refListId);

  int        lengthUvlc(int uiCode);
  int        lengthSvlc(int uiCode);
  void       CheckBestPredScalingList(int scalingListId, int predListIdx, int& BitsCount);
  void       codePredScalingList(int* scalingList, const int* scalingListPred, int scalingListDC, int scalingListPredDC, int scalinListId, int& bitsCost);
  void       codeScalingList(int* scalingList, int scalingListDC, int scalinListId, int& bitsCost);
  void       setScalingListPreditorModeFlag(uint32_t scalingListId, bool bIsPred) { m_scalingListPreditorModeFlag[scalingListId] = bIsPred; }
  bool       getScalingListPreditorModeFlag(uint32_t scalingListId) const { return m_scalingListPreditorModeFlag[scalingListId]; }
  bool       getChromaScalingListPresentFlag() const {return m_chromaScalingListPresentFlag;}
  void       setChromaScalingListPresentFlag( bool flag) { m_chromaScalingListPresentFlag = flag;}
  bool       isLumaScalingList( int scalingListId) const;
  void       checkDcOfMatrix();
  bool       xParseScalingList(const std::string &fileName);
  void       setDefaultScalingList();
  bool       isNotDefaultScalingList();

  bool operator==( const ScalingList& other )
  {
    if (memcmp(m_scalingListPredModeFlagIsCopy, other.m_scalingListPredModeFlagIsCopy, sizeof(m_scalingListPredModeFlagIsCopy)))
    {
      return false;
    }
    if( memcmp( m_scalingListDC, other.m_scalingListDC, sizeof( m_scalingListDC ) ) )
    {
      return false;
    }
    if( memcmp( m_refMatrixId, other.m_refMatrixId, sizeof( m_refMatrixId ) ) )
    {
      return false;
    }
    if( memcmp( m_scalingListCoef, other.m_scalingListCoef, sizeof( m_scalingListCoef ) ) )
    {
      return false;
    }

    return true;
  }

  bool operator!=( const ScalingList& other )
  {
    return !( *this == other );
  }

private:
  void             outputScalingLists(std::ostream &os) const;
  bool             m_scalingListPredModeFlagIsCopy [30]; //!< reference list index
  int              m_scalingListDC                 [30]; //!< the DC value of the matrix coefficient for 16x16
  uint32_t         m_refMatrixId                   [30]; //!< RefMatrixID
  bool             m_scalingListPreditorModeFlag   [30]; //!< reference list index
  std::vector<int> m_scalingListCoef               [30]; //!< quantization matrix
  bool             m_chromaScalingListPresentFlag;
};

class ConstraintInfo
{
  bool              m_gciPresentFlag;
  bool              m_noRprConstraintFlag;
  bool              m_noResChangeInClvsConstraintFlag;
  bool              m_oneTilePerPicConstraintFlag;
  bool              m_picHeaderInSliceHeaderConstraintFlag;
  bool              m_oneSlicePerPicConstraintFlag;
  bool              m_noIdrRplConstraintFlag;
  bool              m_noRectSliceConstraintFlag;
  bool              m_oneSlicePerSubpicConstraintFlag;
  bool              m_noSubpicInfoConstraintFlag;
  bool              m_intraOnlyConstraintFlag;
  uint32_t          m_maxBitDepthConstraintIdc;
  int               m_maxChromaFormatConstraintIdc;
  bool              m_onePictureOnlyConstraintFlag;
  bool              m_lowerBitRateConstraintFlag;
  bool              m_allLayersIndependentConstraintFlag;
  bool              m_noMrlConstraintFlag;
  bool              m_noIspConstraintFlag;
  bool              m_noMipConstraintFlag;
  bool              m_noLfnstConstraintFlag;
  bool              m_noMmvdConstraintFlag;
  bool              m_noSmvdConstraintFlag;
  bool              m_noProfConstraintFlag;
  bool              m_noPaletteConstraintFlag;
  bool              m_noActConstraintFlag;
  bool              m_noLmcsConstraintFlag;

  bool              m_noExplicitScaleListConstraintFlag;
  bool              m_noVirtualBoundaryConstraintFlag;
  bool              m_noMttConstraintFlag;
  bool              m_noChromaQpOffsetConstraintFlag;
  bool              m_noQtbttDualTreeIntraConstraintFlag;
  int               m_maxLog2CtuSizeConstraintIdc;
  bool              m_noPartitionConstraintsOverrideConstraintFlag;
  bool              m_noSaoConstraintFlag;
  bool              m_noAlfConstraintFlag;
  bool              m_noCCAlfConstraintFlag;
  bool              m_noWeightedPredictionConstraintFlag;
  bool              m_noRefWraparoundConstraintFlag;
  bool              m_noTemporalMvpConstraintFlag;
  bool              m_noSbtmvpConstraintFlag;
  bool              m_noAmvrConstraintFlag;
  bool              m_noBdofConstraintFlag;
  bool              m_noDmvrConstraintFlag;
  bool              m_noCclmConstraintFlag;
  bool              m_noMtsConstraintFlag;
  bool              m_noSbtConstraintFlag;
  bool              m_noAffineMotionConstraintFlag;
  bool              m_noBcwConstraintFlag;
  bool              m_noIbcConstraintFlag;
  bool              m_noCiipConstraintFlag;
  bool              m_noGeoConstraintFlag;
  bool              m_noLadfConstraintFlag;
  bool              m_noTransformSkipConstraintFlag;
  bool              m_noLumaTransformSize64ConstraintFlag;
  bool              m_noBDPCMConstraintFlag;
  bool              m_noJointCbCrConstraintFlag;
  bool              m_noCuQpDeltaConstraintFlag;
  bool              m_noDepQuantConstraintFlag;
  bool              m_noSignDataHidingConstraintFlag;
  bool              m_noMixedNaluTypesInPicConstraintFlag;
  bool              m_noTrailConstraintFlag;
  bool              m_noStsaConstraintFlag;
  bool              m_noRaslConstraintFlag;
  bool              m_noRadlConstraintFlag;
  bool              m_noIdrConstraintFlag;
  bool              m_noCraConstraintFlag;
  bool              m_noGdrConstraintFlag;
  bool              m_noApsConstraintFlag;

public:
  ConstraintInfo()
    : m_gciPresentFlag(false)
    , m_noRprConstraintFlag(false)
    , m_noResChangeInClvsConstraintFlag(false)
    , m_oneTilePerPicConstraintFlag(false)
    , m_picHeaderInSliceHeaderConstraintFlag(false)
    , m_oneSlicePerPicConstraintFlag(false)
    , m_noIdrRplConstraintFlag(false)
    , m_noRectSliceConstraintFlag(false)
    , m_oneSlicePerSubpicConstraintFlag(false)
    , m_noSubpicInfoConstraintFlag(false)
    , m_intraOnlyConstraintFlag  (false)
    , m_maxBitDepthConstraintIdc  (  16)
    , m_maxChromaFormatConstraintIdc(CHROMA_444)
    , m_onePictureOnlyConstraintFlag (false)
    , m_lowerBitRateConstraintFlag (false )
    , m_allLayersIndependentConstraintFlag(false)
    , m_noMrlConstraintFlag(false)
    , m_noIspConstraintFlag(false)
    , m_noMipConstraintFlag(false)
    , m_noLfnstConstraintFlag(false)
    , m_noMmvdConstraintFlag(false)
    , m_noSmvdConstraintFlag(false)
    , m_noProfConstraintFlag(false)
    , m_noPaletteConstraintFlag(false)
    , m_noActConstraintFlag(false)
    , m_noLmcsConstraintFlag(false)
    , m_noExplicitScaleListConstraintFlag(false)
    , m_noVirtualBoundaryConstraintFlag(false)
    , m_noMttConstraintFlag(false)
    , m_noChromaQpOffsetConstraintFlag(false)
    , m_noQtbttDualTreeIntraConstraintFlag(false)
    , m_maxLog2CtuSizeConstraintIdc(8)
    , m_noPartitionConstraintsOverrideConstraintFlag(false)
    , m_noSaoConstraintFlag      (false)
    , m_noAlfConstraintFlag      (false)
    , m_noCCAlfConstraintFlag      (false)
    , m_noWeightedPredictionConstraintFlag(false)
    , m_noRefWraparoundConstraintFlag(false)
    , m_noTemporalMvpConstraintFlag(false)
    , m_noSbtmvpConstraintFlag   (false)
    , m_noAmvrConstraintFlag     (false)
    , m_noBdofConstraintFlag     (false)
    , m_noDmvrConstraintFlag     (false)
    , m_noCclmConstraintFlag     (false)
    , m_noMtsConstraintFlag      (false)
    , m_noSbtConstraintFlag      (false)
    , m_noAffineMotionConstraintFlag(false)
    , m_noBcwConstraintFlag      (false)
    , m_noIbcConstraintFlag      (false)
    , m_noCiipConstraintFlag  (false)
    , m_noGeoConstraintFlag      (false)
    , m_noLadfConstraintFlag     (false)
    , m_noTransformSkipConstraintFlag(false)
    , m_noLumaTransformSize64ConstraintFlag(false)
    , m_noBDPCMConstraintFlag    (false)
    , m_noJointCbCrConstraintFlag (false)
    , m_noCuQpDeltaConstraintFlag  (false)
    , m_noDepQuantConstraintFlag (false)
    , m_noSignDataHidingConstraintFlag(false)
    , m_noMixedNaluTypesInPicConstraintFlag(false)
    , m_noTrailConstraintFlag (false)
    , m_noStsaConstraintFlag (false)
    , m_noRaslConstraintFlag (false)
    , m_noRadlConstraintFlag (false)
    , m_noIdrConstraintFlag (false)
    , m_noCraConstraintFlag (false)
    , m_noGdrConstraintFlag (false)
    , m_noApsConstraintFlag (false)
  {}


  bool          getGciPresentFlag() const { return m_gciPresentFlag; }
  void          setGciPresentFlag(bool b) { m_gciPresentFlag = b; }

  uint32_t      getMaxBitDepthConstraintIdc() const { return m_maxBitDepthConstraintIdc; }
  void          setMaxBitDepthConstraintIdc(uint32_t bitDepth) { m_maxBitDepthConstraintIdc = bitDepth; }

  int           getMaxChromaFormatConstraintIdc() const { return m_maxChromaFormatConstraintIdc; }
  void          setMaxChromaFormatConstraintIdc(int fmt) { m_maxChromaFormatConstraintIdc = fmt; }


  bool          getNoRprConstraintFlag() const { return m_noRprConstraintFlag; }
  void          setNoRprConstraintFlag(bool b) { m_noRprConstraintFlag = b; }

  bool          getNoResChangeInClvsConstraintFlag() const { return m_noResChangeInClvsConstraintFlag; }
  void          setNoResChangeInClvsConstraintFlag(bool b) { m_noResChangeInClvsConstraintFlag = b; }

  bool          getOneTilePerPicConstraintFlag() const { return m_oneTilePerPicConstraintFlag; }
  void          setOneTilePerPicConstraintFlag(bool b) { m_oneTilePerPicConstraintFlag = b; }

  bool          getPicHeaderInSliceHeaderConstraintFlag() const { return m_picHeaderInSliceHeaderConstraintFlag; }
  void          setPicHeaderInSliceHeaderConstraintFlag(bool b) { m_picHeaderInSliceHeaderConstraintFlag = b; }

  bool          getOneSlicePerPicConstraintFlag() const { return m_oneSlicePerPicConstraintFlag; }
  void          setOneSlicePerPicConstraintFlag(bool b) { m_oneSlicePerPicConstraintFlag = b; }

  bool          getNoIdrRplConstraintFlag() const          { return m_noIdrRplConstraintFlag; }
  void          setNoIdrRplConstraintFlag(bool b)          { m_noIdrRplConstraintFlag = b; }

  bool          getNoRectSliceConstraintFlag() const       { return m_noRectSliceConstraintFlag; }
  void          setNoRectSliceConstraintFlag(bool b)       { m_noRectSliceConstraintFlag = b; }

  bool          getOneSlicePerSubpicConstraintFlag() const { return m_oneSlicePerSubpicConstraintFlag; }
  void          setOneSlicePerSubpicConstraintFlag(bool b) { m_oneSlicePerSubpicConstraintFlag = b; }

  bool          getNoSubpicInfoConstraintFlag() const      { return m_noSubpicInfoConstraintFlag; }
  void          setNoSubpicInfoConstraintFlag(bool b)      { m_noSubpicInfoConstraintFlag = b; }

  bool          getIntraOnlyConstraintFlag() const { return m_intraOnlyConstraintFlag; }
  void          setIntraOnlyConstraintFlag(bool b) { m_intraOnlyConstraintFlag = b; }

  bool          getOnePictureOnlyConstraintFlag() const { return m_onePictureOnlyConstraintFlag; }
  void          setOnePictureOnlyConstraintFlag(bool b) { m_onePictureOnlyConstraintFlag = b; }

  bool          getLowerBitRateConstraintFlag() const { return m_lowerBitRateConstraintFlag; }
  void          setLowerBitRateConstraintFlag(bool b) { m_lowerBitRateConstraintFlag = b; }
  bool          getAllLayersIndependentConstraintFlag() const { return m_allLayersIndependentConstraintFlag; }
  void          setAllLayersIndependentConstraintFlag(bool b) { m_allLayersIndependentConstraintFlag = b; }
  bool          getNoMrlConstraintFlag() const { return m_noMrlConstraintFlag; }
  void          setNoMrlConstraintFlag(bool b) { m_noMrlConstraintFlag = b; }
  bool          getNoIspConstraintFlag() const { return m_noIspConstraintFlag; }
  void          setNoIspConstraintFlag(bool b) { m_noIspConstraintFlag = b; }
  bool          getNoMipConstraintFlag() const { return m_noMipConstraintFlag; }
  void          setNoMipConstraintFlag(bool b) { m_noMipConstraintFlag = b; }
  bool          getNoLfnstConstraintFlag() const { return m_noLfnstConstraintFlag; }
  void          setNoLfnstConstraintFlag(bool b) { m_noLfnstConstraintFlag = b; }
  bool          getNoMmvdConstraintFlag() const { return m_noMmvdConstraintFlag; }
  void          setNoMmvdConstraintFlag(bool b) { m_noMmvdConstraintFlag = b; }
  bool          getNoSmvdConstraintFlag() const { return m_noSmvdConstraintFlag; }
  void          setNoSmvdConstraintFlag(bool b) { m_noSmvdConstraintFlag = b; }
  bool          getNoProfConstraintFlag() const { return m_noProfConstraintFlag; }
  void          setNoProfConstraintFlag(bool b) { m_noProfConstraintFlag = b; }
  bool          getNoPaletteConstraintFlag() const { return m_noPaletteConstraintFlag; }
  void          setNoPaletteConstraintFlag(bool b) { m_noPaletteConstraintFlag = b; }
  bool          getNoActConstraintFlag() const { return m_noActConstraintFlag; }
  void          setNoActConstraintFlag(bool b) { m_noActConstraintFlag = b; }
  bool          getNoLmcsConstraintFlag() const { return m_noLmcsConstraintFlag; }
  void          setNoLmcsConstraintFlag(bool b) { m_noLmcsConstraintFlag = b; }
  bool          getNoExplicitScaleListConstraintFlag() const { return m_noExplicitScaleListConstraintFlag; }
  void          setNoExplicitScaleListConstraintFlag(bool b) { m_noExplicitScaleListConstraintFlag = b; }
  bool          getNoVirtualBoundaryConstraintFlag() const { return m_noVirtualBoundaryConstraintFlag; }
  void          setNoVirtualBoundaryConstraintFlag(bool b) { m_noVirtualBoundaryConstraintFlag = b; }
  bool          getNoMttConstraintFlag() const { return m_noMttConstraintFlag; }
  void          setNoMttConstraintFlag(bool bVal) { m_noMttConstraintFlag = bVal; }
  bool          getNoChromaQpOffsetConstraintFlag() const { return m_noChromaQpOffsetConstraintFlag; }
  void          setNoChromaQpOffsetConstraintFlag(bool b) { m_noChromaQpOffsetConstraintFlag = b; }
  bool          getNoQtbttDualTreeIntraConstraintFlag() const { return m_noQtbttDualTreeIntraConstraintFlag; }
  void          setNoQtbttDualTreeIntraConstraintFlag(bool bVal) { m_noQtbttDualTreeIntraConstraintFlag = bVal; }
  int           getMaxLog2CtuSizeConstraintIdc() const { return m_maxLog2CtuSizeConstraintIdc; }
  void          setMaxLog2CtuSizeConstraintIdc(int idc) { m_maxLog2CtuSizeConstraintIdc = idc; }
  bool          getNoPartitionConstraintsOverrideConstraintFlag() const { return m_noPartitionConstraintsOverrideConstraintFlag; }
  void          setNoPartitionConstraintsOverrideConstraintFlag(bool bVal) { m_noPartitionConstraintsOverrideConstraintFlag = bVal; }
  bool          getNoSaoConstraintFlag() const { return m_noSaoConstraintFlag; }
  void          setNoSaoConstraintFlag(bool bVal) { m_noSaoConstraintFlag = bVal; }
  bool          getNoAlfConstraintFlag() const { return m_noAlfConstraintFlag; }
  void          setNoAlfConstraintFlag(bool bVal) { m_noAlfConstraintFlag = bVal; }
  bool          getNoCCAlfConstraintFlag() const { return m_noCCAlfConstraintFlag; }
  void          setNoCCAlfConstraintFlag(bool val) { m_noCCAlfConstraintFlag = val; }
  bool          getNoJointCbCrConstraintFlag() const { return m_noJointCbCrConstraintFlag; }
  void          setNoJointCbCrConstraintFlag(bool bVal) { m_noJointCbCrConstraintFlag = bVal; }
  bool          getNoWeightedPredictionConstraintFlag() const { return m_noWeightedPredictionConstraintFlag; }
  void          setNoWeightedPredictionConstraintFlag(bool bVal) { m_noWeightedPredictionConstraintFlag = bVal; }
  bool          getNoRefWraparoundConstraintFlag() const { return m_noRefWraparoundConstraintFlag; }
  void          setNoRefWraparoundConstraintFlag(bool bVal) { m_noRefWraparoundConstraintFlag = bVal; }
  bool          getNoTemporalMvpConstraintFlag() const { return m_noTemporalMvpConstraintFlag; }
  void          setNoTemporalMvpConstraintFlag(bool bVal) { m_noTemporalMvpConstraintFlag = bVal; }
  bool          getNoSbtmvpConstraintFlag() const { return m_noSbtmvpConstraintFlag; }
  void          setNoSbtmvpConstraintFlag(bool bVal) { m_noSbtmvpConstraintFlag = bVal; }
  bool          getNoAmvrConstraintFlag() const { return m_noAmvrConstraintFlag; }
  void          setNoAmvrConstraintFlag(bool bVal) { m_noAmvrConstraintFlag = bVal; }
  bool          getNoBdofConstraintFlag() const { return m_noBdofConstraintFlag; }
  void          setNoBdofConstraintFlag(bool bVal) { m_noBdofConstraintFlag = bVal; }
  bool          getNoDmvrConstraintFlag() const { return m_noDmvrConstraintFlag; }
  void          setNoDmvrConstraintFlag(bool bVal) { m_noDmvrConstraintFlag = bVal; }
  bool          getNoCclmConstraintFlag() const { return m_noCclmConstraintFlag; }
  void          setNoCclmConstraintFlag(bool bVal) { m_noCclmConstraintFlag = bVal; }
  bool          getNoMtsConstraintFlag() const { return m_noMtsConstraintFlag; }
  void          setNoMtsConstraintFlag(bool bVal) { m_noMtsConstraintFlag = bVal; }
  bool          getNoSbtConstraintFlag() const { return m_noSbtConstraintFlag; }
  void          setNoSbtConstraintFlag(bool bVal) { m_noSbtConstraintFlag = bVal; }
  bool          getNoAffineMotionConstraintFlag() const { return m_noAffineMotionConstraintFlag; }
  void          setNoAffineMotionConstraintFlag(bool bVal) { m_noAffineMotionConstraintFlag = bVal; }
  bool          getNoBcwConstraintFlag() const { return m_noBcwConstraintFlag; }
  void          setNoBcwConstraintFlag(bool bVal) { m_noBcwConstraintFlag = bVal; }
  bool          getNoIbcConstraintFlag() const { return m_noIbcConstraintFlag; }
  void          setNoIbcConstraintFlag(bool bVal) { m_noIbcConstraintFlag = bVal; }
  bool          getNoCiipConstraintFlag() const { return m_noCiipConstraintFlag; }
  void          setNoCiipConstraintFlag(bool bVal) { m_noCiipConstraintFlag = bVal; }
  bool          getNoGeoConstraintFlag() const { return m_noGeoConstraintFlag; }
  void          setNoGeoConstraintFlag(bool bVal) { m_noGeoConstraintFlag = bVal; }
  bool          getNoLadfConstraintFlag() const { return m_noLadfConstraintFlag; }
  void          setNoLadfConstraintFlag(bool bVal) { m_noLadfConstraintFlag = bVal; }
  bool          getNoTransformSkipConstraintFlag() const { return m_noTransformSkipConstraintFlag; }
  void          setNoTransformSkipConstraintFlag(bool bVal) { m_noTransformSkipConstraintFlag = bVal; }
  bool          getNoLumaTransformSize64ConstraintFlag() const { return m_noLumaTransformSize64ConstraintFlag; }
  void          setNoLumaTransformSize64ConstraintFlag(bool bVal) { m_noLumaTransformSize64ConstraintFlag = bVal; }
  bool          getNoBDPCMConstraintFlag() const { return m_noBDPCMConstraintFlag; }
  void          setNoBDPCMConstraintFlag(bool bVal) { m_noBDPCMConstraintFlag = bVal; }
  bool          getNoCuQpDeltaConstraintFlag() const { return m_noCuQpDeltaConstraintFlag; }
  void          setNoCuQpDeltaConstraintFlag(bool bVal) { m_noCuQpDeltaConstraintFlag = bVal; }
  bool          getNoDepQuantConstraintFlag() const { return m_noDepQuantConstraintFlag; }
  void          setNoDepQuantConstraintFlag(bool bVal) { m_noDepQuantConstraintFlag = bVal; }
  bool          getNoSignDataHidingConstraintFlag() const { return m_noSignDataHidingConstraintFlag; }
  void          setNoSignDataHidingConstraintFlag(bool bVal) { m_noSignDataHidingConstraintFlag = bVal; }
  bool          getNoMixedNaluTypesInPicConstraintFlag() const    { return m_noMixedNaluTypesInPicConstraintFlag; }
  void          setNoMixedNaluTypesInPicConstraintFlag(bool bVal) { m_noMixedNaluTypesInPicConstraintFlag = bVal; }
  bool          getNoTrailConstraintFlag() const { return m_noTrailConstraintFlag; }
  void          setNoTrailConstraintFlag(bool bVal) { m_noTrailConstraintFlag = bVal; }
  bool          getNoStsaConstraintFlag() const { return m_noStsaConstraintFlag; }
  void          setNoStsaConstraintFlag(bool bVal) { m_noStsaConstraintFlag = bVal; }
  bool          getNoRaslConstraintFlag() const { return m_noRaslConstraintFlag; }
  void          setNoRaslConstraintFlag(bool bVal) { m_noRaslConstraintFlag = bVal; }
  bool          getNoRadlConstraintFlag() const { return m_noRadlConstraintFlag; }
  void          setNoRadlConstraintFlag(bool bVal) { m_noRadlConstraintFlag = bVal; }
  bool          getNoIdrConstraintFlag() const { return m_noIdrConstraintFlag; }
  void          setNoIdrConstraintFlag(bool bVal) { m_noIdrConstraintFlag = bVal; }
  bool          getNoCraConstraintFlag() const { return m_noCraConstraintFlag; }
  void          setNoCraConstraintFlag(bool bVal) { m_noCraConstraintFlag = bVal; }
  bool          getNoGdrConstraintFlag() const { return m_noGdrConstraintFlag; }
  void          setNoGdrConstraintFlag(bool bVal) { m_noGdrConstraintFlag = bVal; }
  bool          getNoApsConstraintFlag() const { return m_noApsConstraintFlag; }
  void          setNoApsConstraintFlag(bool bVal) { m_noApsConstraintFlag = bVal; }

  friend bool             operator == (const ConstraintInfo& op1, const ConstraintInfo& op2);
  friend bool             operator != (const ConstraintInfo& op1, const ConstraintInfo& op2);
};

class ProfileTierLevel
{
  Level::Tier       m_tierFlag;
  Profile::Name     m_profileIdc;
  uint8_t           m_numSubProfile;
  std::vector<uint32_t>          m_subProfileIdc;
  Level::Name       m_levelIdc;
  bool              m_frameOnlyConstraintFlag;
  bool              m_multiLayerEnabledFlag;
  ConstraintInfo    m_constraintInfo;
  bool              m_subLayerLevelPresentFlag[MAX_TLAYER - 1];
  Level::Name       m_subLayerLevelIdc[MAX_TLAYER];

public:
                ProfileTierLevel();

  Level::Tier   getTierFlag() const                         { return m_tierFlag;                    }
  void          setTierFlag(Level::Tier x)                  { m_tierFlag = x;                       }

  Profile::Name getProfileIdc() const                       { return m_profileIdc;                  }
  void          setProfileIdc(Profile::Name x)              { m_profileIdc = x;                     }

  uint32_t      getSubProfileIdc(int i) const               { return m_subProfileIdc[i]; }
  void          setSubProfileIdc(int i, uint32_t x)         { m_subProfileIdc[i] = x; }

  uint8_t       getNumSubProfile() const                    { return m_numSubProfile; }
  void          setNumSubProfile(uint8_t x)                 { m_numSubProfile = x; m_subProfileIdc.resize(m_numSubProfile); }

  Level::Name   getLevelIdc() const                         { return m_levelIdc;                    }
  void          setLevelIdc(Level::Name x)                  { m_levelIdc = x;                       }

  bool                    getFrameOnlyConstraintFlag() const { return m_frameOnlyConstraintFlag; }
  void                    setFrameOnlyConstraintFlag(bool x) { m_frameOnlyConstraintFlag = x; }

  bool                    getMultiLayerEnabledFlag() const { return m_multiLayerEnabledFlag; }
  void                    setMultiLayerEnabledFlag(bool x) { m_multiLayerEnabledFlag = x; }

  ConstraintInfo*         getConstraintInfo()              { return &m_constraintInfo; }
  const ConstraintInfo*   getConstraintInfo() const        { return &m_constraintInfo; }

  bool                    getSubLayerLevelPresentFlag(int i) const     { return m_subLayerLevelPresentFlag[i];   }
  void                    setSubLayerLevelPresentFlag(int i, bool x)   { m_subLayerLevelPresentFlag[i] = x;      }

  Level::Name             getSubLayerLevelIdc(int i) const             { return m_subLayerLevelIdc[i];   }
  void                    setSubLayerLevelIdc(int i, Level::Name x)    { m_subLayerLevelIdc[i] = x;      }
  friend bool             operator == (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
  friend bool             operator != (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
};



class SliceReshapeInfo
{
public:
  bool      sliceReshaperEnableFlag;
  bool      sliceReshaperModelPresentFlag;
  unsigned  enableChromaAdj;
  uint32_t  reshaperModelMinBinIdx;
  uint32_t  reshaperModelMaxBinIdx;
  int       reshaperModelBinCWDelta[PIC_CODE_CW_BINS];
  int       maxNbitsNeededDeltaCW;
  int       chrResScalingOffset;
  void      setUseSliceReshaper(bool b)                                { sliceReshaperEnableFlag = b;            }
  bool      getUseSliceReshaper() const                                { return sliceReshaperEnableFlag;         }
  void      setSliceReshapeModelPresentFlag(bool b)                    { sliceReshaperModelPresentFlag = b;      }
  bool      getSliceReshapeModelPresentFlag() const                    { return   sliceReshaperModelPresentFlag; }
  void      setSliceReshapeChromaAdj(unsigned adj)                     { enableChromaAdj = adj;                  }
  unsigned  getSliceReshapeChromaAdj() const                           { return enableChromaAdj;                 }

  bool operator==( const SliceReshapeInfo& other )
  {
    if( sliceReshaperEnableFlag != other.sliceReshaperEnableFlag )
    {
      return false;
    }
    if( sliceReshaperModelPresentFlag != other.sliceReshaperModelPresentFlag )
    {
      return false;
    }
    if( enableChromaAdj != other.enableChromaAdj )
    {
      return false;
    }
    if( reshaperModelMinBinIdx != other.reshaperModelMinBinIdx )
    {
      return false;
    }
    if( reshaperModelMaxBinIdx != other.reshaperModelMaxBinIdx )
    {
      return false;
    }
    if( maxNbitsNeededDeltaCW != other.maxNbitsNeededDeltaCW )
    {
      return false;
    }
    if (chrResScalingOffset != other.chrResScalingOffset)
    {
      return false;
    }
    if( memcmp( reshaperModelBinCWDelta, other.reshaperModelBinCWDelta, sizeof( reshaperModelBinCWDelta ) ) )
    {
      return false;
    }

    return true;
  }

  bool operator!=( const SliceReshapeInfo& other )
  {
    return !( *this == other );
  }
};

struct ReshapeCW
{
  std::vector<uint32_t> binCW;
  int       updateCtrl;
  int       adpOption;
  uint32_t  initialCW;
  int rspPicSize;
  int rspFps;
  int rspBaseQP;
  int rspTid;
  int rspSliceQP;
  int rspFpsToIp;
};

struct ChromaQpAdj
{
  union
  {
    struct {
      int CbOffset;
      int CrOffset;
      int JointCbCrOffset;
    } comp;
    int offset[3];
  } u;
};
struct ChromaQpMappingTableParams {
  int               m_qpBdOffset;
  bool              m_sameCQPTableForAllChromaFlag;
  int               m_numQpTables;
  int               m_qpTableStartMinus26[MAX_NUM_CQP_MAPPING_TABLES];
  int               m_numPtsInCQPTableMinus1[MAX_NUM_CQP_MAPPING_TABLES];
  std::vector<int>  m_deltaQpInValMinus1[MAX_NUM_CQP_MAPPING_TABLES];
  std::vector<int>  m_deltaQpOutVal[MAX_NUM_CQP_MAPPING_TABLES];

  ChromaQpMappingTableParams()
  {
    m_qpBdOffset = 12;
    m_sameCQPTableForAllChromaFlag = true;
    m_numQpTables = 1;
    m_numPtsInCQPTableMinus1[0] = 0;
    m_qpTableStartMinus26[0] = 0;
    m_deltaQpInValMinus1[0] = { 0 };
    m_deltaQpOutVal[0] = { 0 };
  }

  void      setSameCQPTableForAllChromaFlag(bool b)                             { m_sameCQPTableForAllChromaFlag = b; }
  bool      getSameCQPTableForAllChromaFlag()                             const { return m_sameCQPTableForAllChromaFlag; }
  void      setNumQpTables(int n)                                     { m_numQpTables = n; }
  int       getNumQpTables()                                     const { return m_numQpTables; }
  void      setQpTableStartMinus26(int tableIdx, int n)                         { m_qpTableStartMinus26[tableIdx] = n; }
  int       getQpTableStartMinus26(int tableIdx)                          const { return m_qpTableStartMinus26[tableIdx]; }
  void      setNumPtsInCQPTableMinus1(int tableIdx, int n)                      { m_numPtsInCQPTableMinus1[tableIdx] = n; }
  int       getNumPtsInCQPTableMinus1(int tableIdx)                       const { return m_numPtsInCQPTableMinus1[tableIdx]; }
  void      setDeltaQpInValMinus1(int tableIdx, std::vector<int> &inVals)       { m_deltaQpInValMinus1[tableIdx] = inVals; }
  void      setDeltaQpInValMinus1(int tableIdx, int idx, int n)                 { m_deltaQpInValMinus1[tableIdx][idx] = n; }
  int       getDeltaQpInValMinus1(int tableIdx, int idx)                  const { return m_deltaQpInValMinus1[tableIdx][idx]; }
  void      setDeltaQpOutVal(int tableIdx, std::vector<int> &outVals)           { m_deltaQpOutVal[tableIdx] = outVals; }
  void      setDeltaQpOutVal(int tableIdx, int idx, int n)                      { m_deltaQpOutVal[tableIdx][idx] = n; }
  int       getDeltaQpOutVal(int tableIdx, int idx)                       const { return m_deltaQpOutVal[tableIdx][idx]; }
};
struct ChromaQpMappingTable : ChromaQpMappingTableParams
{
  std::map<int, int> m_chromaQpMappingTables[MAX_NUM_CQP_MAPPING_TABLES];

  int       getMappedChromaQpValue(ComponentID compID, const int qpVal)  const { return m_chromaQpMappingTables[m_sameCQPTableForAllChromaFlag ? 0 : (int)compID - 1].at(qpVal); }
  void      derivedChromaQPMappingTables();
  void      setParams(const ChromaQpMappingTableParams &params, const int qpBdOffset);
};

class SliceMap
{
private:
  uint32_t               m_sliceID;                           //!< slice identifier (slice index for rectangular slices, slice address for raser-scan slices)
  uint32_t               m_numTilesInSlice;                   //!< number of tiles in slice (raster-scan slices only)
  uint32_t               m_numCtuInSlice;                     //!< number of CTUs in the slice
  std::vector<uint32_t>  m_ctuAddrInSlice;                    //!< raster-scan addresses of all the CTUs in the slice

public:
  SliceMap();
  virtual ~SliceMap();

  void                   setSliceID( uint32_t u )             { m_sliceID = u;            }
  uint32_t               getSliceID() const                   { return m_sliceID;         }
  void                   setNumTilesInSlice( uint32_t u )     { m_numTilesInSlice = u;    }
  uint32_t               getNumTilesInSlice() const           { return m_numTilesInSlice; }
  void                   setNumCtuInSlice( uint32_t u )       { m_numCtuInSlice = u;      }
  uint32_t               getNumCtuInSlice() const             { return m_numCtuInSlice;   }
  std::vector<uint32_t>  getCtuAddrList( ) const              { return m_ctuAddrInSlice;  }
  uint32_t               getCtuAddrInSlice( int idx ) const   {
    CHECK_(idx >= m_ctuAddrInSlice.size(), "CTU index exceeds number of CTUs in slice."); return m_ctuAddrInSlice[idx]; }
  void                   pushToCtuAddrInSlice( uint32_t u )   { m_ctuAddrInSlice.push_back(u); m_numCtuInSlice++;}

  void  initSliceMap()
  {
    m_sliceID = 0;
    m_numTilesInSlice = 0;
    m_numCtuInSlice = 0;
    m_ctuAddrInSlice.clear();
  }

  void  addCtusToSlice( uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY )
  {
    CHECK_( startX >= stopX || startY >= stopY, "Invalid slice definition");
    for( uint32_t ctbY = startY; ctbY < stopY; ctbY++ )
    {
      for( uint32_t ctbX = startX; ctbX < stopX; ctbX++ )
      {
        m_ctuAddrInSlice.push_back( ctbY * picWidthInCtbsY + ctbX );
        m_numCtuInSlice++;
      }
    }
  }
};

class RectSlice
{
private:
  uint32_t         m_tileIdx;                           //!< tile index corresponding to the first CTU in the slice
  uint32_t         m_sliceWidthInTiles;                 //!< slice width in units of tiles
  uint32_t         m_sliceHeightInTiles;                //!< slice height in units of tiles
  uint32_t         m_numSlicesInTile;                   //!< number of slices in current tile for the special case of multiple slices inside a single tile
  uint32_t         m_sliceHeightInCtu;                  //!< slice height in units of CTUs for the special case of multiple slices inside a single tile

public:
  RectSlice();
  virtual ~RectSlice();

  void             setSliceWidthInTiles( uint32_t u )   { m_sliceWidthInTiles = u;      }
  uint32_t         getSliceWidthInTiles( ) const        { return  m_sliceWidthInTiles;  }
  void             setSliceHeightInTiles( uint32_t u )  { m_sliceHeightInTiles = u;     }
  uint32_t         getSliceHeightInTiles( ) const       { return  m_sliceHeightInTiles; }
  void             setNumSlicesInTile( uint32_t u )     { m_numSlicesInTile = u;        }
  uint32_t         getNumSlicesInTile( ) const          { return  m_numSlicesInTile;    }
  void             setSliceHeightInCtu( uint32_t u )    { m_sliceHeightInCtu = u;       }
  uint32_t         getSliceHeightInCtu( ) const         { return  m_sliceHeightInCtu;   }
  void             setTileIdx( uint32_t u )             { m_tileIdx = u;                }
  uint32_t         getTileIdx( ) const                  { return  m_tileIdx;            }

};

class SubPic
{
private:
  uint32_t         m_subPicID;                                  //!< ID of subpicture
  uint32_t         m_subPicIdx;                                 //!< Index of subpicture
  uint32_t         m_numCTUsInSubPic;                           //!< number of CTUs contained in this sub-picture
  uint32_t         m_subPicCtuTopLeftX;                         //!< horizontal position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicCtuTopLeftY;                         //!< vertical position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicWidth;                               //!< the width of subpicture in units of CTU
  uint32_t         m_subPicHeight;                              //!< the height of subpicture in units of CTU
  uint32_t         m_subPicWidthInLumaSample;                   //!< the width of subpicture in units of luma sample
  uint32_t         m_subPicHeightInLumaSample;                  //!< the height of subpicture in units of luma sample
  uint32_t         m_firstCtuInSubPic;                          //!< the raster scan index of the first CTU in a subpicture
  uint32_t         m_lastCtuInSubPic;                           //!< the raster scan index of the last CTU in a subpicture
  uint32_t         m_subPicLeft;                                //!< the position of left boundary
  uint32_t         m_subPicRight;                               //!< the position of right boundary
  uint32_t         m_subPicTop;                                 //!< the position of top boundary
  uint32_t         m_subPicBottom;                              //!< the position of bottom boundary
  std::vector<uint32_t> m_ctuAddrInSubPic;                      //!< raster scan addresses of all the CTUs in the slice

  bool             m_treatedAsPicFlag;                          //!< whether the subpicture is treated as a picture in the decoding process excluding in-loop filtering operations
  bool             m_loopFilterAcrossSubPicEnabledFlag;         //!< whether in-loop filtering operations may be performed across the boundaries of the subpicture
  uint32_t         m_numSlicesInSubPic;                         //!< Number of slices contained in this subpicture

public:
  SubPic();
  virtual ~SubPic();

  void             setSubPicID (uint32_t u)                {         m_subPicID = u;       }
  uint32_t         getSubPicID   ()                  const { return  m_subPicID;           }
  void             setSubPicIdx (uint32_t u)               {         m_subPicIdx = u;      }
  uint32_t         getSubPicIdx ()                   const { return  m_subPicIdx;          }
  void             setNumCTUsInSubPic   (uint32_t u)       {         m_numCTUsInSubPic = u;       }
  uint32_t         getNumCTUsInSubPic   ()           const { return  m_numCTUsInSubPic;           }
  void             setSubPicCtuTopLeftX (uint32_t u)       {         m_subPicCtuTopLeftX = u;     }
  uint32_t         getSubPicCtuTopLeftX ()           const { return  m_subPicCtuTopLeftX;         }
  void             setSubPicCtuTopLeftY (uint32_t u)       {         m_subPicCtuTopLeftY = u;     }
  uint32_t         getSubPicCtuTopLeftY ()           const { return  m_subPicCtuTopLeftY;         }
  void             setSubPicWidthInCTUs (uint32_t u)       {         m_subPicWidth = u;           }
  uint32_t         getSubPicWidthInCTUs ()           const { return  m_subPicWidth;               }
  void             setSubPicHeightInCTUs(uint32_t u)       {         m_subPicHeight = u;          }
  uint32_t         getSubPicHeightInCTUs()           const { return  m_subPicHeight;              }
  void             setFirstCTUInSubPic  (uint32_t u)       {         m_firstCtuInSubPic = u;      }
  uint32_t         getFirstCTUInSubPic  ()           const { return  m_firstCtuInSubPic;          }
  void             setLastCTUInSubPic   (uint32_t u)       {         m_lastCtuInSubPic = u;       }
  uint32_t         getLastCTUInSubPic   ()           const { return  m_lastCtuInSubPic;           }
  void             setSubPicLeft        (uint32_t u)       {         m_subPicLeft = u;            }
  uint32_t         getSubPicLeft        ()           const { return  m_subPicLeft;                }
  void             setSubPicRight       (uint32_t u)       {         m_subPicRight = u;           }
  uint32_t         getSubPicRight       ()           const { return  m_subPicRight;               }
  void             setSubPicTop         (uint32_t u)       {         m_subPicTop = u;             }
  uint32_t         getSubPicTop         ()           const { return  m_subPicTop;                 }
  void             setSubPicBottom      (uint32_t u)       {         m_subPicBottom = u;          }
  uint32_t         getSubPicBottom      ()           const { return  m_subPicBottom;              }

  void             setSubPicWidthInLumaSample (uint32_t u) {         m_subPicWidthInLumaSample = u;   }
  uint32_t         getSubPicWidthInLumaSample()      const { return  m_subPicWidthInLumaSample;       }
  void             setSubPicHeightInLumaSample(uint32_t u) {         m_subPicHeightInLumaSample = u;  }
  uint32_t         getSubPicHeightInLumaSample()     const { return  m_subPicHeightInLumaSample;      }

  std::vector<uint32_t> getCtuAddrList  ()           const { return  m_ctuAddrInSubPic;           }
  void                  clearCTUAddrList()                 { m_ctuAddrInSubPic.clear(); }
  void                  addCTUsToSubPic(std::vector<uint32_t> ctuAddrInSlice)
  {
    for (auto ctu:ctuAddrInSlice)
      m_ctuAddrInSubPic.push_back(ctu);
  }
  void  addAllCtusInPicToSubPic(uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY)
  {
    CHECK_(startX >= stopX || startY >= stopY, "Invalid slice definition");
    for (uint32_t ctbY = startY; ctbY < stopY; ctbY++)
    {
      for (uint32_t ctbX = startX; ctbX < stopX; ctbX++)
      {
        m_ctuAddrInSubPic.push_back(ctbY * picWidthInCtbsY + ctbX);
      }
    }
  }
  bool                 isContainingPos(const Position& pos) const
  {
    return pos.x >= m_subPicLeft && pos.x <= m_subPicRight && pos.y >= m_subPicTop  && pos.y <= m_subPicBottom;
  }
  void             setTreatedAsPicFlag           (bool u)  {         m_treatedAsPicFlag = u;   }
  bool             getTreatedAsPicFlag           ()  const { return  m_treatedAsPicFlag;       }
  void             setloopFilterAcrossEnabledFlag(bool u)  {         m_loopFilterAcrossSubPicEnabledFlag = u; }
  bool             getloopFilterAcrossEnabledFlag()  const { return  m_loopFilterAcrossSubPicEnabledFlag;     }

  bool             isFirstCTUinSubPic(uint32_t ctuAddr) const { return  ctuAddr == m_firstCtuInSubPic;  }
  bool              isLastCTUinSubPic(uint32_t ctuAddr) const { return  ctuAddr == m_lastCtuInSubPic;   }
  void             setNumSlicesInSubPic( uint32_t val )    { m_numSlicesInSubPic = val; }
  uint32_t         getNumSlicesInSubPic() const            { return m_numSlicesInSubPic; }
  bool             containsCtu(const Position& pos) const
  {
    return pos.x >= m_subPicCtuTopLeftX && pos.x < m_subPicCtuTopLeftX + m_subPicWidth &&
           pos.y >= m_subPicCtuTopLeftY && pos.y < m_subPicCtuTopLeftY + m_subPicHeight;
  }
  bool             containsCtu(int ctuAddr) const
  {
    for (auto & addr : m_ctuAddrInSubPic)
    {
      if (addr == ctuAddr)
      {
        return true;
      }
    }
    return false;
  }
};

class DCI
{
private:
  int m_maxSubLayersMinus1;
  std::vector<ProfileTierLevel> m_profileTierLevel;

public:
  DCI()
    : m_maxSubLayersMinus1(0)
  {};

  virtual ~DCI() {};

  int  getMaxSubLayersMinus1() const { return m_maxSubLayersMinus1; }
  void setMaxSubLayersMinus1(int val) { m_maxSubLayersMinus1 = val; }

  size_t getNumPTLs() const { return m_profileTierLevel.size(); }
  void  setProfileTierLevel(const std::vector<ProfileTierLevel>& val) { m_profileTierLevel = val; }
  const ProfileTierLevel& getProfileTierLevel(int idx) const { return m_profileTierLevel[idx]; }
  bool  IsIndenticalDCI(const DCI& comparedDCI) const
  {
    if(m_maxSubLayersMinus1 != comparedDCI.m_maxSubLayersMinus1) return false;
    if(m_profileTierLevel != comparedDCI.m_profileTierLevel) return false;
    return true;
  }
};

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
class OPI
{
private:
  bool m_olsinfopresentflag;
  bool m_htidinfopresentflag;
  uint32_t  m_opiolsidx;
  uint32_t  m_opihtidplus1;

public:
  OPI()
    : m_olsinfopresentflag (false)
    ,  m_htidinfopresentflag (false)
    ,  m_opiolsidx (-1)
    ,  m_opihtidplus1 (-1)
  {};

  virtual ~OPI() {};

  bool getOlsInfoPresentFlag() const { return m_olsinfopresentflag; }
  void setOlsInfoPresentFlag(bool val) { m_olsinfopresentflag = val; }
  bool getHtidInfoPresentFlag() const { return m_htidinfopresentflag; }
  void setHtidInfoPresentFlag(bool val) { m_htidinfopresentflag = val; }
  uint32_t getOpiOlsIdx() const { return m_opiolsidx; }
  void setOpiOlsIdx(uint32_t val) { m_opiolsidx = val; }
  uint32_t getOpiHtidPlus1() const { return m_opihtidplus1; }
  void setOpiHtidPlus1(uint32_t val) { m_opihtidplus1 = val; }

};
#endif

class VPS
{
private:
  int                   m_VPSId;
  uint32_t              m_maxLayers;

  uint32_t              m_vpsMaxSubLayers;
  uint32_t              m_vpsLayerId[MAX_VPS_LAYERS];
  bool                  m_vpsDefaultPtlDpbHrdMaxTidFlag;
  bool                  m_vpsAllIndependentLayersFlag;
  uint32_t              m_vpsCfgPredDirection[MAX_VPS_SUBLAYERS];
  bool                  m_vpsIndependentLayerFlag[MAX_VPS_LAYERS];
  bool                  m_vpsDirectRefLayerFlag[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
#if JVET_R0193
  std::vector<std::vector<uint32_t>>              m_vpsMaxTidIlRefPicsPlus1;
#else
  uint32_t              m_vpsMaxTidIlRefPicsPlus1[MAX_VPS_LAYERS];
#endif
  bool                  m_vpsEachLayerIsAnOlsFlag;
  uint32_t              m_vpsOlsModeIdc;
  uint32_t              m_vpsNumOutputLayerSets;
  bool                  m_vpsOlsOutputLayerFlag[MAX_NUM_OLSS][MAX_VPS_LAYERS];
  uint32_t              m_directRefLayerIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  uint32_t              m_generalLayerIdx[MAX_VPS_LAYERS];

  uint32_t              m_vpsNumPtls;
  bool                  m_ptPresentFlag[MAX_NUM_OLSS];
  uint32_t              m_ptlMaxTemporalId[MAX_NUM_OLSS];
  std::vector<ProfileTierLevel> m_vpsProfileTierLevel;
  uint32_t              m_olsPtlIdx[MAX_NUM_OLSS];

  // stores index ( ilrp_idx within 0 .. NumDirectRefLayers ) of the dependent reference layers
  uint32_t              m_interLayerRefIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  bool                  m_vpsExtensionFlag;
  bool                  m_vpsGeneralHrdParamsPresentFlag;
  bool                  m_vpsSublayerCpbParamsPresentFlag;
  uint32_t              m_numOlsTimingHrdParamsMinus1;
  uint32_t              m_hrdMaxTid[MAX_NUM_OLSS];
  uint32_t              m_olsTimingHrdIdx[MAX_NUM_OLSS];
  GeneralHrdParams      m_generalHrdParams;
  std::vector<Size>             m_olsDpbPicSize;
  std::vector<int>              m_olsDpbParamsIdx;
  std::vector<std::vector<int>> m_outputLayerIdInOls;
  std::vector<std::vector<int>> m_numSubLayersInLayerInOLS;

  std::vector<int> m_multiLayerOlsIdxToOlsIdx; // mapping from multi-layer OLS index to OLS index. Initialized in deriveOutputLayerSets()
                                               // m_multiLayerOlsIdxToOlsIdx[n] is the OLSidx of the n-th multi-layer OLS.
public:
  std::vector<std::vector<OlsHrdParams>> m_olsHrdParams;
  int                           m_totalNumOLSs;
  int                           m_numMultiLayeredOlss;
  uint32_t                      m_multiLayerOlsIdx[MAX_NUM_OLSS];
  int                           m_numDpbParams;
  std::vector<DpbParameters>    m_dpbParameters;
  bool                          m_sublayerDpbParamsPresentFlag;
  std::vector<int>              m_dpbMaxTemporalId;
  std::vector<int>              m_targetOutputLayerIdSet;          ///< set of LayerIds to be outputted
  std::vector<int>              m_targetLayerIdSet;                ///< set of LayerIds to be included in the sub-bitstream extraction process.
  int                           m_targetOlsIdx;
  std::vector<int>              m_numOutputLayersInOls;
  std::vector<int>              m_numLayersInOls;
  std::vector<std::vector<int>> m_layerIdInOls;
  std::vector<int>              m_olsDpbChromaFormatIdc;
  std::vector<int>              m_olsDpbBitDepthMinus8;

public:
                    VPS();

  virtual           ~VPS();

  int               getVPSId() const                                     { return m_VPSId;                                                  }
  void              setVPSId(int i)                                      { m_VPSId = i;                                                     }

  uint32_t          getMaxLayers() const                                 { return m_maxLayers;                                            }
  void              setMaxLayers(uint32_t l)                             { m_maxLayers = l;                                               }

  uint32_t          getMaxSubLayers() const                              { return m_vpsMaxSubLayers;                                        }
  void              setMaxSubLayers(uint32_t value)                      { m_vpsMaxSubLayers = value;                                       }
  bool              getDefaultPtlDpbHrdMaxTidFlag() const { return m_vpsDefaultPtlDpbHrdMaxTidFlag; }
  void              setDefaultPtlDpbHrdMaxTidFlag(bool t) { m_vpsDefaultPtlDpbHrdMaxTidFlag = t; }

  uint32_t          getLayerId(uint32_t layerIdx) const { return m_vpsLayerId[layerIdx]; }
  void              setLayerId(uint32_t layerIdx, uint32_t layerId) { m_vpsLayerId[layerIdx] = layerId; }

  bool              getAllIndependentLayersFlag() const { return m_vpsAllIndependentLayersFlag; }
  void              setAllIndependentLayersFlag(bool t) { m_vpsAllIndependentLayersFlag = t; }
  uint32_t          getPredDirection(uint32_t tmplayer) const { return m_vpsCfgPredDirection[tmplayer]; }
  void              setPredDirection(uint32_t tmplayer, uint32_t t) { m_vpsCfgPredDirection[tmplayer] = t; }

  bool              getIndependentLayerFlag(uint32_t layerIdx) const { return m_vpsIndependentLayerFlag[layerIdx]; }
  void              setIndependentLayerFlag(uint32_t layerIdx, bool t) { m_vpsIndependentLayerFlag[layerIdx] = t; }
#if JVET_R0193
  uint32_t          getMaxTidIlRefPicsPlus1(uint32_t layerIdx, uint32_t refLayerIdx) const { return m_vpsMaxTidIlRefPicsPlus1[layerIdx][refLayerIdx]; }
  void              setMaxTidIlRefPicsPlus1(uint32_t layerIdx, uint32_t refLayerIdx, uint32_t i) { m_vpsMaxTidIlRefPicsPlus1[layerIdx][refLayerIdx] = i; }
  void              setMaxTidIlRefPicsPlus1(std::vector<std::vector<uint32_t>> i) { m_vpsMaxTidIlRefPicsPlus1 = i; }
#else
  uint32_t          getMaxTidIlRefPicsPlus1(uint32_t layerIdx) const { return m_vpsMaxTidIlRefPicsPlus1[layerIdx]; }
  void              setMaxTidIlRefPicsPlus1(uint32_t layerIdx, uint32_t i) { m_vpsMaxTidIlRefPicsPlus1[layerIdx] = i; }
#endif

  bool              getDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx) const { return m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx]; }
  void              setDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx, bool t) { m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx] = t; }

  uint32_t          getDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc ) const { return m_directRefLayerIdx[layerIdx][refLayerIdc]; }
  void              setDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc, uint32_t refLayerIdx ) { m_directRefLayerIdx[layerIdx][refLayerIdc] = refLayerIdx; }

  uint32_t          getInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx ) const { return m_interLayerRefIdx[layerIdx][refLayerIdx]; }
  void              setInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx, uint32_t refLayerIdc ) { m_interLayerRefIdx[layerIdx][refLayerIdx] = refLayerIdc; }

  uint32_t          getGeneralLayerIdx(uint32_t layerId) const { return m_generalLayerIdx[layerId]; }
  void              setGeneralLayerIdx(uint32_t layerId, uint32_t layerIdc) { m_generalLayerIdx[layerId] = layerIdc; }

  bool              getEachLayerIsAnOlsFlag() const { return m_vpsEachLayerIsAnOlsFlag; }
  void              setEachLayerIsAnOlsFlag(bool t) { m_vpsEachLayerIsAnOlsFlag = t; }

  uint32_t          getOlsModeIdc() const { return m_vpsOlsModeIdc; }
  void              setOlsModeIdc(uint32_t t) { m_vpsOlsModeIdc = t; }

  uint32_t          getNumOutputLayerSets() const { return m_vpsNumOutputLayerSets; }
  void              setNumOutputLayerSets(uint8_t t) { m_vpsNumOutputLayerSets = t; }

  bool              getOlsOutputLayerFlag(uint32_t ols, uint32_t layer) const { return m_vpsOlsOutputLayerFlag[ols][layer]; }
  void              setOlsOutputLayerFlag(uint32_t ols, uint32_t layer, bool t) { m_vpsOlsOutputLayerFlag[ols][layer] = t; }

  uint32_t          getNumPtls()                                   const { return m_vpsNumPtls; }
  void              setNumPtls(uint32_t val)                             { m_vpsNumPtls = val; }

  bool              getPtPresentFlag(int idx)                      const { return m_ptPresentFlag[idx]; }
  void              setPtPresentFlag(int idx, bool val)                  { m_ptPresentFlag[idx] = val; }

  uint32_t          getPtlMaxTemporalId(int idx)                   const { return m_ptlMaxTemporalId[idx]; }
  void              setPtlMaxTemporalId(int idx, uint32_t val)           { m_ptlMaxTemporalId[idx] = val; }

  void              setProfileTierLevel(const std::vector<ProfileTierLevel> &val)   { m_vpsProfileTierLevel = val; }
  const ProfileTierLevel& getProfileTierLevel(int idx)             const { return m_vpsProfileTierLevel[idx]; }

  uint32_t          getOlsPtlIdx(int idx)                          const { return m_olsPtlIdx[idx]; }
  void              setOlsPtlIdx(int idx, uint32_t val)                  { m_olsPtlIdx[idx] = val; }

  bool              getVPSExtensionFlag() const                          { return m_vpsExtensionFlag;                                 }
  void              setVPSExtensionFlag(bool t)                          { m_vpsExtensionFlag = t;                                    }
  bool              getVPSGeneralHrdParamsPresentFlag() const { return m_vpsGeneralHrdParamsPresentFlag; }
  void              setVPSGeneralHrdParamsPresentFlag(bool t) { m_vpsGeneralHrdParamsPresentFlag = t; }
  bool              getVPSSublayerCpbParamsPresentFlag() const { return m_vpsSublayerCpbParamsPresentFlag; }
  void              setVPSSublayerCpbParamsPresentFlag(bool t) { m_vpsSublayerCpbParamsPresentFlag = t; }
  uint32_t          getNumOlsTimingHrdParamsMinus1()               const { return m_numOlsTimingHrdParamsMinus1; }
  void              setNumOlsTimingHrdParamsMinus1(uint32_t val)         { m_numOlsTimingHrdParamsMinus1 = val; }
  uint32_t          getHrdMaxTid(int olsIdx)                   const { return m_hrdMaxTid[olsIdx]; }
  void              setHrdMaxTid(int olsIdx, uint32_t val)           { m_hrdMaxTid[olsIdx] = val; }
  uint32_t          getOlsTimingHrdIdx(int olsIdx)                   const { return m_olsTimingHrdIdx[olsIdx]; }
  void              setOlsTimingHrdIdx(int olsIdx, uint32_t val)           { m_olsTimingHrdIdx[olsIdx] = val; }

  OlsHrdParams*          getOlsHrdParameters(int olsIdx) { return &m_olsHrdParams[olsIdx][0]; }
  const OlsHrdParams*    getOlsHrdParameters(int olsIdx) const { return &m_olsHrdParams[olsIdx][0]; }

  GeneralHrdParams*          getGeneralHrdParameters() { return &m_generalHrdParams; }
  const GeneralHrdParams*    getGeneralHrdParameters() const { return &m_generalHrdParams; }
  int               getTargetOlsIdx() { return m_targetOlsIdx; }
  void              setTargetOlsIdx(uint32_t t) { m_targetOlsIdx = t; }

  int               getMaxDecPicBuffering( int temporalId ) const        { return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].m_maxDecPicBuffering[temporalId]; }
  int               getMaxNumReorderPics( int temporalId ) const         { return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].m_maxNumReorderPics[temporalId]; }
  int               getTotalNumOLSs() const                              { return m_totalNumOLSs; }
  int               getNumMultiLayeredOlss() const                       { return m_numMultiLayeredOlss; }
  Size              getOlsDpbPicSize( int olsIdx ) const                 { return m_olsDpbPicSize[olsIdx];          }
  void              setOlsDpbPicSize( int olsIdx, Size size )            { m_olsDpbPicSize[olsIdx] = size;          }
  void              setOlsDpbPicWidth( int olsIdx, int width )           { m_olsDpbPicSize[olsIdx].width = width;   }
  void              setOlsDpbPicHeight( int olsIdx, int height )         { m_olsDpbPicSize[olsIdx].height = height; }
  int               getOlsDpbChromaFormatIdc(int olsIdx) const           { return m_olsDpbChromaFormatIdc[olsIdx];  }
  int               getOlsDpbBitDepthMinus8(int olsIdx) const            { return m_olsDpbBitDepthMinus8[olsIdx];   }
  void              setOlsDpbChromaFormatIdc(int olsIdx, int chromaFormatIdc)  { m_olsDpbChromaFormatIdc[olsIdx] = chromaFormatIdc; }
  void              setOlsDpbBitDepthMinus8(int olsIdx, int bitDepthMinus8) { m_olsDpbBitDepthMinus8[olsIdx] = bitDepthMinus8; }

  int               getOlsDpbParamsIdx( int olsIdx ) const               { return m_olsDpbParamsIdx[olsIdx];        }
  void              setOlsDpbParamsIdx( int olsIdx, int paramIdx )       { m_olsDpbParamsIdx[olsIdx] = paramIdx;    }

  void              deriveOutputLayerSets();
  void              deriveTargetOutputLayerSet( int targetOlsIdx );
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  int               deriveTargetOLSIdx();
  uint32_t          getMaxTidinTOls(int m_targetOlsIdx);
#endif

  void              checkVPS();

  void              setNumLayersInOls(int olsIdx, int numLayers)         { m_numLayersInOls[olsIdx]  = numLayers; }
  int               getNumLayersInOls(int olsIdx)      const             { return m_numLayersInOls[olsIdx]; }

  void              setLayerIdInOls  (int olsIdx, int layerIdx, int layerId)    { m_layerIdInOls[olsIdx][layerIdx] = layerId; }
  uint32_t          getLayerIdInOls  (int olsIdx, int layerIdx) const           { return m_layerIdInOls[olsIdx][layerIdx]   ; }
  std::vector<int>  getLayerIdsInOls(int targetOlsIdx)                    { return m_layerIdInOls[targetOlsIdx];     }

  int               getNumSubLayersInLayerInOLS (int olsIdx, int layerIdx) const    { return m_numSubLayersInLayerInOLS[olsIdx][layerIdx]   ; }
};

class Window
{
private:
  bool m_enabledFlag;
  int  m_winLeftOffset;
  int  m_winRightOffset;
  int  m_winTopOffset;
  int  m_winBottomOffset;
public:
  Window()
  : m_enabledFlag    (false)
  , m_winLeftOffset  (0)
  , m_winRightOffset (0)
  , m_winTopOffset   (0)
  , m_winBottomOffset(0)
  { }

  bool getWindowEnabledFlag() const   { return m_enabledFlag;                          }
  int  getWindowLeftOffset() const    { return m_enabledFlag ? m_winLeftOffset : 0;    }
  void setWindowLeftOffset(int val)   { m_winLeftOffset = val; m_enabledFlag |=  (val!=0);   }
  int  getWindowRightOffset() const   { return m_enabledFlag ? m_winRightOffset : 0;   }
  void setWindowRightOffset(int val)  { m_winRightOffset = val; m_enabledFlag |= (val!=0);  }
  int  getWindowTopOffset() const     { return m_enabledFlag ? m_winTopOffset : 0;     }
  void setWindowTopOffset(int val)    { m_winTopOffset = val; m_enabledFlag |= (val!=0);    }
  int  getWindowBottomOffset() const  { return m_enabledFlag ? m_winBottomOffset: 0;   }
  void setWindowBottomOffset(int val) { m_winBottomOffset = val; m_enabledFlag |= (val!=0); }

  void setWindow(int offsetLeft, int offsetRight, int offsetTop, int offsetBottom)
  {
    m_enabledFlag     = (offsetLeft || offsetRight || offsetTop || offsetBottom);
    m_winLeftOffset   = offsetLeft;
    m_winRightOffset  = offsetRight;
    m_winTopOffset    = offsetTop;
    m_winBottomOffset = offsetBottom;
  }
};


class VUI
{
private:
  bool       m_progressiveSourceFlag;
  bool       m_interlacedSourceFlag;
  bool       m_nonPackedFlag;
  bool       m_nonProjectedFlag;
  bool       m_aspectRatioInfoPresentFlag;
  bool       m_aspectRatioConstantFlag;
  int        m_aspectRatioIdc;
  int        m_sarWidth;
  int        m_sarHeight;
  bool       m_overscanInfoPresentFlag;
  bool       m_overscanAppropriateFlag;
  bool       m_colourDescriptionPresentFlag;
  int        m_colourPrimaries;
  int        m_transferCharacteristics;
  int        m_matrixCoefficients;
  bool       m_videoFullRangeFlag;
  bool       m_chromaLocInfoPresentFlag;
  int        m_chromaSampleLocTypeTopField;
  int        m_chromaSampleLocTypeBottomField;
  int        m_chromaSampleLocType;

public:
  VUI()
    : m_progressiveSourceFlag             (false) // Default values as documented in VVC D10 are used
    , m_interlacedSourceFlag              (false)
    , m_nonPackedFlag                     (false)
    , m_nonProjectedFlag                  (false)
    , m_aspectRatioInfoPresentFlag        (false) 
    , m_aspectRatioConstantFlag           (false)
    , m_aspectRatioIdc                    (0)
    , m_sarWidth                          (0)
    , m_sarHeight                         (0)
    , m_overscanInfoPresentFlag           (false)
    , m_overscanAppropriateFlag           (false)
    , m_colourDescriptionPresentFlag      (false)
    , m_colourPrimaries                   (2)
    , m_transferCharacteristics           (2)
    , m_matrixCoefficients                (2)
    , m_videoFullRangeFlag                (false)
    , m_chromaLocInfoPresentFlag          (false)
    , m_chromaSampleLocTypeTopField       (6)
    , m_chromaSampleLocTypeBottomField    (6)
    , m_chromaSampleLocType               (6)
  {}

  virtual           ~VUI() {}

  bool              getAspectRatioInfoPresentFlag() const                  { return m_aspectRatioInfoPresentFlag;           }
  void              setAspectRatioInfoPresentFlag(bool i)                  { m_aspectRatioInfoPresentFlag = i;              }
  bool              getAspectRatioConstantFlag() const                     { return m_aspectRatioConstantFlag;           }
  void              setAspectRatioConstantFlag(bool b)                     { m_aspectRatioConstantFlag = b;              }

  int               getAspectRatioIdc() const                              { return m_aspectRatioIdc;                       }
  void              setAspectRatioIdc(int i)                               { m_aspectRatioIdc = i;                          }

  int               getSarWidth() const                                    { return m_sarWidth;                             }
  void              setSarWidth(int i)                                     { m_sarWidth = i;                                }

  int               getSarHeight() const                                   { return m_sarHeight;                            }
  void              setSarHeight(int i)                                    { m_sarHeight = i;                               }

  bool              getColourDescriptionPresentFlag() const                { return m_colourDescriptionPresentFlag;         }
  void              setColourDescriptionPresentFlag(bool i)                { m_colourDescriptionPresentFlag = i;            }

  int               getColourPrimaries() const                             { return m_colourPrimaries;                      }
  void              setColourPrimaries(int i)                              { m_colourPrimaries = i;                         }

  int               getTransferCharacteristics() const                     { return m_transferCharacteristics;              }
  void              setTransferCharacteristics(int i)                      { m_transferCharacteristics = i;                 }

  int               getMatrixCoefficients() const                          { return m_matrixCoefficients;                   }
  void              setMatrixCoefficients(int i)                           { m_matrixCoefficients = i;                      }

  bool              getProgressiveSourceFlag() const                       { return m_progressiveSourceFlag; }
  void              setProgressiveSourceFlag(bool b)                       { m_progressiveSourceFlag = b; }

  bool              getInterlacedSourceFlag() const                        { return m_interlacedSourceFlag; }
  void              setInterlacedSourceFlag(bool b)                        { m_interlacedSourceFlag = b; }

  bool              getNonPackedFlag() const                               { return m_nonPackedFlag; }
  void              setNonPackedFlag(bool b)                               { m_nonPackedFlag = b; }

  bool              getNonProjectedFlag() const                            { return m_nonProjectedFlag; }
  void              setNonProjectedFlag(bool b)                            { m_nonProjectedFlag = b; }

  bool              getChromaLocInfoPresentFlag() const                    { return m_chromaLocInfoPresentFlag;             }
  void              setChromaLocInfoPresentFlag(bool i)                    { m_chromaLocInfoPresentFlag = i;                }

  int               getChromaSampleLocTypeTopField() const                 { return m_chromaSampleLocTypeTopField;          }
  void              setChromaSampleLocTypeTopField(int i)                  { m_chromaSampleLocTypeTopField = i;             }

  int               getChromaSampleLocTypeBottomField() const              { return m_chromaSampleLocTypeBottomField;       }
  void              setChromaSampleLocTypeBottomField(int i)               { m_chromaSampleLocTypeBottomField = i;          }

  int               getChromaSampleLocType() const                         { return m_chromaSampleLocType;          }
  void              setChromaSampleLocType(int i)                          { m_chromaSampleLocType = i;             }

  bool              getOverscanInfoPresentFlag() const                     { return m_overscanInfoPresentFlag;              }
  void              setOverscanInfoPresentFlag(bool i)                     { m_overscanInfoPresentFlag = i;                 }

  bool              getOverscanAppropriateFlag() const                     { return m_overscanAppropriateFlag;              }
  void              setOverscanAppropriateFlag(bool i)                     { m_overscanAppropriateFlag = i;                 }

  bool              getVideoFullRangeFlag() const                          { return m_videoFullRangeFlag;                   }
  void              setVideoFullRangeFlag(bool i)                          { m_videoFullRangeFlag = i;                      }

};

/// SPS RExt class
class SPSRExt // Names aligned to text specification
{
private:
  bool             m_transformSkipRotationEnabledFlag;
  bool             m_transformSkipContextEnabledFlag;
  bool             m_extendedPrecisionProcessingFlag;
  bool             m_intraSmoothingDisabledFlag;
  bool             m_highPrecisionOffsetsEnabledFlag;
  bool             m_persistentRiceAdaptationEnabledFlag;
  bool             m_cabacBypassAlignmentEnabledFlag;

public:
  SPSRExt();

  bool settingsDifferFromDefaults() const
  {
    return getTransformSkipRotationEnabledFlag()
        || getTransformSkipContextEnabledFlag()
        || getExtendedPrecisionProcessingFlag()
        || getIntraSmoothingDisabledFlag()
        || getHighPrecisionOffsetsEnabledFlag()
        || getPersistentRiceAdaptationEnabledFlag()
        || getCabacBypassAlignmentEnabledFlag();
  }


  bool getTransformSkipRotationEnabledFlag() const                                     { return m_transformSkipRotationEnabledFlag;     }
  void setTransformSkipRotationEnabledFlag(const bool value)                           { m_transformSkipRotationEnabledFlag = value;    }

  bool getTransformSkipContextEnabledFlag() const                                      { return m_transformSkipContextEnabledFlag;      }
  void setTransformSkipContextEnabledFlag(const bool value)                            { m_transformSkipContextEnabledFlag = value;     }

  bool getExtendedPrecisionProcessingFlag() const                                      { return m_extendedPrecisionProcessingFlag;      }
  void setExtendedPrecisionProcessingFlag(bool value)                                  { m_extendedPrecisionProcessingFlag = value;     }

  bool getIntraSmoothingDisabledFlag() const                                           { return m_intraSmoothingDisabledFlag;           }
  void setIntraSmoothingDisabledFlag(bool bValue)                                      { m_intraSmoothingDisabledFlag=bValue;           }

  bool getHighPrecisionOffsetsEnabledFlag() const                                      { return m_highPrecisionOffsetsEnabledFlag;      }
  void setHighPrecisionOffsetsEnabledFlag(bool value)                                  { m_highPrecisionOffsetsEnabledFlag = value;     }

  bool getPersistentRiceAdaptationEnabledFlag() const                                  { return m_persistentRiceAdaptationEnabledFlag;  }
  void setPersistentRiceAdaptationEnabledFlag(const bool value)                        { m_persistentRiceAdaptationEnabledFlag = value; }

  bool getCabacBypassAlignmentEnabledFlag() const                                      { return m_cabacBypassAlignmentEnabledFlag;      }
  void setCabacBypassAlignmentEnabledFlag(const bool value)                            { m_cabacBypassAlignmentEnabledFlag = value;     }
};


/// SPS class
class SPS
{
private:
  int               m_SPSId;
  int               m_VPSId;
  int               m_layerId;
  bool              m_affineAmvrEnabledFlag;
  bool              m_DMVR;
  bool              m_MMVD;
  bool              m_SBT;
  bool              m_ISP;
  ChromaFormat      m_chromaFormatIdc;

  uint32_t              m_uiMaxTLayers;           // maximum number of temporal layers

  bool              m_ptlDpbHrdParamsPresentFlag;
  bool              m_SubLayerDpbParamsFlag;

  // Structure
  uint32_t              m_maxWidthInLumaSamples;
  uint32_t              m_maxHeightInLumaSamples;
  Window                m_conformanceWindow;
  bool                  m_subPicInfoPresentFlag;                // indicates the presence of sub-picture info
  uint32_t              m_numSubPics;                        //!< number of sub-pictures used
  bool                  m_independentSubPicsFlag;
  bool                  m_subPicSameSizeFlag;
  std::vector<uint32_t> m_subPicCtuTopLeftX;
  std::vector<uint32_t> m_subPicCtuTopLeftY;
  std::vector<uint32_t> m_subPicWidth;
  std::vector<uint32_t> m_subPicHeight;
  std::vector<bool>     m_subPicTreatedAsPicFlag;
  std::vector<bool>     m_loopFilterAcrossSubpicEnabledFlag;
  bool                  m_subPicIdMappingExplicitlySignalledFlag;
  bool                  m_subPicIdMappingPresentFlag;
  uint32_t              m_subPicIdLen;                       //!< sub-picture ID length in bits
  std::vector<uint16_t> m_subPicId;                          //!< sub-picture ID for each sub-picture in the sequence

  int               m_log2MinCodingBlockSize;
  unsigned    m_CTUSize;
  unsigned    m_partitionOverrideEnalbed;       // enable partition constraints override function
  unsigned    m_minQT[3];   // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned    m_maxMTTHierarchyDepth[3];
  unsigned    m_maxBTSize[3];
  unsigned    m_maxTTSize[3];
  bool        m_idrRefParamList;
  unsigned    m_dualITree;
  uint32_t              m_uiMaxCUWidth;
  uint32_t              m_uiMaxCUHeight;

  RPLList           m_RPLList0;
  RPLList           m_RPLList1;
  uint32_t          m_numRPL0;
  uint32_t          m_numRPL1;

  bool              m_rpl1CopyFromRpl0Flag;
  bool              m_rpl1IdxPresentFlag;
  bool              m_allRplEntriesHasSameSignFlag;
  bool              m_bLongTermRefsPresent;
  bool              m_SPSTemporalMVPEnabledFlag;
  int               m_maxNumReorderPics[MAX_TLAYER];

  // Tool list

  bool              m_transformSkipEnabledFlag;
  int               m_log2MaxTransformSkipBlockSize;
  bool              m_BDPCMEnabledFlag;
  bool              m_JointCbCrEnabledFlag;
  // Parameter
  BitDepths         m_bitDepths;
  bool              m_entropyCodingSyncEnabledFlag;                    //!< Flag for enabling WPP
  bool              m_entryPointPresentFlag;                           //!< Flag for indicating the presence of entry points
  int               m_qpBDOffset[MAX_NUM_CHANNEL_TYPE];
  int               m_internalMinusInputBitDepth[MAX_NUM_CHANNEL_TYPE]; //  max(0, internal bitdepth - input bitdepth);                                          }

  bool              m_sbtmvpEnabledFlag;
  bool              m_bdofEnabledFlag;
  bool              m_fpelMmvdEnabledFlag;
  bool              m_BdofControlPresentInPhFlag;
  bool              m_DmvrControlPresentInPhFlag;
  bool              m_ProfControlPresentInPhFlag;
  uint32_t          m_uiBitsForPOC;
  bool              m_pocMsbCycleFlag;
  uint32_t          m_pocMsbCycleLen;
  int               m_numExtraPHBytes;
  int               m_numExtraSHBytes;

  std::vector<bool> m_extraPHBitPresentFlag;
  std::vector<bool> m_extraSHBitPresentFlag;
  uint32_t          m_numLongTermRefPicSPS;
  uint32_t          m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  bool              m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS];
  uint32_t          m_log2MaxTbSize;
  bool              m_useWeightPred;                     //!< Use of Weighting Prediction (P_SLICE)
  bool              m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)

  bool              m_saoEnabledFlag;

  bool              m_bTemporalIdNestingFlag; // temporal_id_nesting_flag

  bool              m_scalingListEnabledFlag;
  bool              m_depQuantEnabledFlag;            //!< dependent quantization enabled flag
  bool              m_signDataHidingEnabledFlag;      //!< sign data hiding enabled flag
  bool              m_virtualBoundariesEnabledFlag;   //!< Enable virtual boundaries tool
  bool              m_virtualBoundariesPresentFlag;   //!< disable loop filtering across virtual boundaries
  unsigned          m_numVerVirtualBoundaries;                         //!< number of vertical virtual boundaries
  unsigned          m_numHorVirtualBoundaries;                         //!< number of horizontal virtual boundaries
  unsigned          m_virtualBoundariesPosX[3];                        //!< horizontal position of each vertical virtual boundary
  unsigned          m_virtualBoundariesPosY[3];                        //!< vertical position of each horizontal virtual boundary
  uint32_t          m_uiMaxDecPicBuffering[MAX_TLAYER];
  uint32_t          m_uiMaxLatencyIncreasePlus1[MAX_TLAYER];


  bool              m_generalHrdParametersPresentFlag;
  GeneralHrdParams m_generalHrdParams;
  OlsHrdParams     m_olsHrdParams[MAX_TLAYER];

  bool              m_fieldSeqFlag;
  bool              m_vuiParametersPresentFlag;
  unsigned          m_vuiPayloadSize;
  VUI               m_vuiParameters;

  SPSRExt           m_spsRangeExtension;

  static const int  m_winUnitX[NUM_CHROMA_FORMAT];
  static const int  m_winUnitY[NUM_CHROMA_FORMAT];
  ProfileTierLevel  m_profileTierLevel;

  bool              m_alfEnabledFlag;
  bool              m_ccalfEnabledFlag;
  bool              m_wrapAroundEnabledFlag;
  unsigned          m_IBCFlag;
  bool              m_useColorTrans;
  unsigned          m_PLTMode;

  bool              m_lmcsEnabled;
  bool              m_AMVREnabledFlag;
  bool              m_LMChroma;
  bool              m_horCollocatedChromaFlag;
  bool              m_verCollocatedChromaFlag;
  bool              m_MTS;
  bool              m_IntraMTS;                   // 18
  bool              m_InterMTS;                   // 19
  bool              m_LFNST;
  bool              m_SMVD;
  bool              m_Affine;
  bool              m_AffineType;
  bool              m_PROF;
  bool              m_bcw;                        //
  bool              m_ciip;
  bool              m_Geo;
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool              m_LadfEnabled;
  int               m_LadfNumIntervals;
  int               m_LadfQpOffset[MAX_LADF_INTERVALS];
  int               m_LadfIntervalLowerBound[MAX_LADF_INTERVALS];
#endif
  bool              m_MRL;
  bool              m_MIP;
  ChromaQpMappingTable m_chromaQpMappingTable;
  bool m_GDREnabledFlag;
  bool              m_SubLayerCbpParametersPresentFlag;

  bool              m_rprEnabledFlag;
  bool              m_resChangeInClvsEnabledFlag;
  bool              m_interLayerPresentFlag;

  uint32_t          m_log2ParallelMergeLevelMinus2;
  bool              m_ppsValidFlag[64];
  Size              m_scalingWindowSizeInPPS[64];
  uint32_t          m_maxNumMergeCand;
  uint32_t          m_maxNumAffineMergeCand;
  uint32_t          m_maxNumIBCMergeCand;
  uint32_t          m_maxNumGeoCand;
  bool              m_scalingMatrixAlternativeColourSpaceDisabledFlag;
  bool              m_scalingMatrixDesignatedColourSpaceFlag;

  bool m_disableScalingMatrixForLfnstBlks; 

public:

  SPS();
  virtual                 ~SPS();

  int                     getSPSId() const                                                                { return m_SPSId;                                                      }
  void                    setSPSId(int i)                                                                 { m_SPSId = i;                                                         }
  int                     getVPSId() const                                                                { return m_VPSId; }
  void                    setVPSId(int i)                                                                 { m_VPSId = i; }
  void                    setLayerId( int i )                                                             { m_layerId = i;                                                       }
  int                     getLayerId() const                                                              { return m_layerId;                                                    }
  ChromaFormat            getChromaFormatIdc () const                                                     { return m_chromaFormatIdc;                                            }
  void                    setChromaFormatIdc (ChromaFormat i)                                             { m_chromaFormatIdc = i;                                               }

  static int              getWinUnitX (int chromaFormatIdc)                                               {
    CHECK_(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitX[chromaFormatIdc]; }
  static int              getWinUnitY (int chromaFormatIdc)                                               {
    CHECK_(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitY[chromaFormatIdc]; }

  // structure
  void                    setMaxPicWidthInLumaSamples( uint32_t u )                                       { m_maxWidthInLumaSamples = u; }
  uint32_t                getMaxPicWidthInLumaSamples() const                                             { return  m_maxWidthInLumaSamples; }
  void                    setMaxPicHeightInLumaSamples( uint32_t u )                                      { m_maxHeightInLumaSamples = u; }
  uint32_t                getMaxPicHeightInLumaSamples() const                                            { return  m_maxHeightInLumaSamples; }
  Window&                 getConformanceWindow()                                                          { return  m_conformanceWindow; }
  const Window&           getConformanceWindow() const                                                    { return  m_conformanceWindow; }
  void                    setConformanceWindow( Window& conformanceWindow )                               { m_conformanceWindow = conformanceWindow; }

  void      setSubPicInfoPresentFlag(bool b)                                                { m_subPicInfoPresentFlag = b;            }
  bool      getSubPicInfoPresentFlag() const                                                { return m_subPicInfoPresentFlag;         }

  void      setNumSubPics( uint32_t u )                                                     {
    CHECK_( u >= MAX_NUM_SUB_PICS, "Maximum number of subpictures exceeded" );
                                                                                              m_numSubPics = u;
                                                                                              m_subPicCtuTopLeftX.resize(m_numSubPics);
                                                                                              m_subPicCtuTopLeftY.resize(m_numSubPics);
                                                                                              m_subPicWidth.resize(m_numSubPics);
                                                                                              m_subPicHeight.resize(m_numSubPics);
                                                                                              m_subPicTreatedAsPicFlag.resize(m_numSubPics);
                                                                                              m_loopFilterAcrossSubpicEnabledFlag.resize(m_numSubPics);
                                                                                              m_subPicId.resize(m_numSubPics);
                                                                                            }
  void      setIndependentSubPicsFlag(bool b)                                                { m_independentSubPicsFlag = b;                    }
  bool      getIndependentSubPicsFlag() const                                                { return m_independentSubPicsFlag;                 }
  void      setSubPicSameSizeFlag(bool b)                                                   { m_subPicSameSizeFlag = b;                       }
  bool      getSubPicSameSizeFlag() const                                                   { return m_subPicSameSizeFlag;                    }
  uint32_t  getNumSubPics( ) const                                                          { return  m_numSubPics;                           }
  void      setSubPicCtuTopLeftX( int i, uint32_t u )                                       { m_subPicCtuTopLeftX[i] = u;                     }
  uint32_t  getSubPicCtuTopLeftX( int i ) const                                             { return  m_subPicCtuTopLeftX[i];                 }
  void      setSubPicCtuTopLeftY( int i, uint32_t u )                                       { m_subPicCtuTopLeftY[i] = u;                     }
  uint32_t  getSubPicCtuTopLeftY( int i ) const                                             { return  m_subPicCtuTopLeftY[i];                 }
  void      setSubPicWidth( int i, uint32_t u )                                             { m_subPicWidth[i] = u;                           }
  uint32_t  getSubPicWidth( int i ) const                                                   { return  m_subPicWidth[i];                       }
  void      setSubPicHeight( int i, uint32_t u )                                            { m_subPicHeight[i] = u;                          }
  uint32_t  getSubPicHeight( int i ) const                                                  { return  m_subPicHeight[i];                      }
  void      setSubPicTreatedAsPicFlag( int i, bool u )                                      { m_subPicTreatedAsPicFlag[i] = u;                }
  bool      getSubPicTreatedAsPicFlag( int i ) const                                        { return  m_subPicTreatedAsPicFlag[i];            }
  void      setLoopFilterAcrossSubpicEnabledFlag( int i, bool u )                           { m_loopFilterAcrossSubpicEnabledFlag[i] = u;     }
  bool      getLoopFilterAcrossSubpicEnabledFlag( int i ) const                             { return  m_loopFilterAcrossSubpicEnabledFlag[i]; }

  void      setSubPicCtuTopLeftX                        (const std::vector<uint32_t> &v)   {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicCtuTopLeftX = v; }
  void      setSubPicCtuTopLeftY                        (const std::vector<uint32_t> &v)   {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicCtuTopLeftY = v; }
  void      setSubPicWidth                              (const std::vector<uint32_t> &v)   {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicWidth = v; }
  void      setSubPicHeight                             (const std::vector<uint32_t> &v)   {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicHeight = v; }
  void      setSubPicTreatedAsPicFlag                   (const std::vector<bool> &v)       {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicTreatedAsPicFlag = v; }
  void      setLoopFilterAcrossSubpicEnabledFlag        (const std::vector<bool> &v)       {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_loopFilterAcrossSubpicEnabledFlag = v; }

  bool       getDisableScalingMatrixForLfnstBlks() const { return m_disableScalingMatrixForLfnstBlks; }
  void       setDisableScalingMatrixForLfnstBlks(bool flag) { m_disableScalingMatrixForLfnstBlks = flag; }

  void                    setSubPicIdMappingExplicitlySignalledFlag( bool b )                             { m_subPicIdMappingExplicitlySignalledFlag = b;    }
  bool                    getSubPicIdMappingExplicitlySignalledFlag() const                               { return m_subPicIdMappingExplicitlySignalledFlag; }
  void                    setSubPicIdMappingPresentFlag( bool b )                                           { m_subPicIdMappingPresentFlag = b;                  }
  bool                    getSubPicIdMappingPresentFlag() const                                             { return  m_subPicIdMappingPresentFlag;              }
  void                    setSubPicIdLen( uint32_t u )                                                    { m_subPicIdLen = u;                       }
  uint32_t                getSubPicIdLen() const                                                          { return  m_subPicIdLen;                   }
  void                    setSubPicId( int i, uint16_t u )                                                { m_subPicId[i] = u;     }
  uint16_t                getSubPicId( int i ) const                                                      { return  m_subPicId[i]; }
  void                    setSubPicId(const std::vector<uint16_t> &v)                                     {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ; m_subPicId = v; }
  const std::vector<uint16_t> getSubPicIds() const                                                        { return  m_subPicId; }

  uint32_t                getNumLongTermRefPicSPS() const                                                 { return m_numLongTermRefPicSPS;                                       }
  void                    setNumLongTermRefPicSPS(uint32_t val)                                           { m_numLongTermRefPicSPS = val;                                        }

  uint32_t                getLtRefPicPocLsbSps(uint32_t index) const                                      {
    CHECK_( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_ltRefPicPocLsbSps[index]; }
  void                    setLtRefPicPocLsbSps(uint32_t index, uint32_t val)                              {
    CHECK_( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_ltRefPicPocLsbSps[index] = val;  }

  bool                    getUsedByCurrPicLtSPSFlag(int i) const                                          {
    CHECK_( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_usedByCurrPicLtSPSFlag[i];    }
  void                    setUsedByCurrPicLtSPSFlag(int i, bool x)                                        {
    CHECK_( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_usedByCurrPicLtSPSFlag[i] = x;       }

  int                     getLog2MinCodingBlockSize() const                                               { return m_log2MinCodingBlockSize;                                     }
  void                    setLog2MinCodingBlockSize(int val)                                              { m_log2MinCodingBlockSize = val;                                      }
  void                    setCTUSize(unsigned    ctuSize)                                                 { m_CTUSize = ctuSize; }
  unsigned                getCTUSize()                                                              const { return  m_CTUSize; }
  void                    setSplitConsOverrideEnabledFlag(bool b)                                         { m_partitionOverrideEnalbed = b; }
  bool                    getSplitConsOverrideEnabledFlag()                                         const { return m_partitionOverrideEnalbed; }
  void                    setMinQTSizes(unsigned*   minQT)                                                { m_minQT[0] = minQT[0]; m_minQT[1] = minQT[1]; m_minQT[2] = minQT[2]; }
  unsigned                getMinQTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA)
                                                                                                    const { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_minQT[0] : m_minQT[2]) : m_minQT[1]; }
  void                    setMaxMTTHierarchyDepth(unsigned    maxMTTHierarchyDepth,
                                        unsigned    maxMTTHierarchyDepthI,
                                        unsigned    maxMTTHierarchyDepthIChroma)
                                                                                                          { m_maxMTTHierarchyDepth[1] = maxMTTHierarchyDepth; m_maxMTTHierarchyDepth[0] = maxMTTHierarchyDepthI; m_maxMTTHierarchyDepth[2] = maxMTTHierarchyDepthIChroma; }
  unsigned                getMaxMTTHierarchyDepth()                                                 const { return m_maxMTTHierarchyDepth[1]; }
  unsigned                getMaxMTTHierarchyDepthI()                                                const { return m_maxMTTHierarchyDepth[0]; }
  unsigned                getMaxMTTHierarchyDepthIChroma()                                          const { return m_maxMTTHierarchyDepth[2]; }
  void                    setMaxBTSize(unsigned    maxBTSize,
                                       unsigned    maxBTSizeI,
                                       unsigned    maxBTSizeC)
                                                                                                          { m_maxBTSize[1] = maxBTSize; m_maxBTSize[0] = maxBTSizeI; m_maxBTSize[2] = maxBTSizeC; }
  unsigned                getMaxBTSize()                                                            const { return m_maxBTSize[1]; }
  unsigned                getMaxBTSizeI()                                                           const { return m_maxBTSize[0]; }
  unsigned                getMaxBTSizeIChroma()                                                     const { return m_maxBTSize[2]; }
  void                    setMaxTTSize(unsigned    maxTTSize,
                                       unsigned    maxTTSizeI,
                                       unsigned    maxTTSizeC)
                                                                                                          { m_maxTTSize[1] = maxTTSize; m_maxTTSize[0] = maxTTSizeI; m_maxTTSize[2] = maxTTSizeC; }
  unsigned                getMaxTTSize()                                                            const { return m_maxTTSize[1]; }
  unsigned                getMaxTTSizeI()                                                           const { return m_maxTTSize[0]; }
  unsigned                getMaxTTSizeIChroma()                                                     const { return m_maxTTSize[2]; }
  unsigned*               getMinQTSizes()                                                          const { return (unsigned *)m_minQT;                }
  unsigned*               getMaxMTTHierarchyDepths()                                               const { return (unsigned *)m_maxMTTHierarchyDepth; }
  unsigned*               getMaxBTSizes()                                                          const { return (unsigned *)m_maxBTSize;            }
  unsigned*               getMaxTTSizes()                                                          const { return (unsigned *)m_maxTTSize;            }
  void                    setIDRRefParamListPresent(bool b)                             { m_idrRefParamList = b; }
  bool                    getIDRRefParamListPresent()                             const { return m_idrRefParamList; }
  void                    setUseDualITree(bool b) { m_dualITree = b; }
  bool                    getUseDualITree()                                      const { return m_dualITree; }

  void                    setMaxCUWidth( uint32_t u )                                                         { m_uiMaxCUWidth = u;                                                  }
  uint32_t                    getMaxCUWidth() const                                                           { return  m_uiMaxCUWidth;                                              }
  void                    setMaxCUHeight( uint32_t u )                                                        { m_uiMaxCUHeight = u;                                                 }
  uint32_t                    getMaxCUHeight() const                                                          { return  m_uiMaxCUHeight;                                             }
  bool                    getTransformSkipEnabledFlag() const                                                 { return m_transformSkipEnabledFlag;                                   }
  void                    setTransformSkipEnabledFlag( bool b )                                               { m_transformSkipEnabledFlag = b;                                      }
  uint32_t                getLog2MaxTransformSkipBlockSize() const                                            { return m_log2MaxTransformSkipBlockSize;                              }
  void                    setLog2MaxTransformSkipBlockSize(uint32_t u)                                        { m_log2MaxTransformSkipBlockSize = u;                                 }
  bool                    getBDPCMEnabledFlag() const                                                         { return m_BDPCMEnabledFlag;                                           }
  void                    setBDPCMEnabledFlag( bool b )                                                       { m_BDPCMEnabledFlag = b;                                              }
  void                    setBitsForPOC( uint32_t u )                                                         { m_uiBitsForPOC = u;                                                  }
  uint32_t                    getBitsForPOC() const                                                           { return m_uiBitsForPOC;                                               }
  void                    setPocMsbCycleFlag(bool b)                                                          { m_pocMsbCycleFlag = b;                                               }
  bool                    getPocMsbCycleFlag() const                                                          { return m_pocMsbCycleFlag;                                            }
  void                    setPocMsbCycleLen(uint32_t u)                                                       { m_pocMsbCycleLen = u;                                                }
  uint32_t                getPocMsbCycleLen() const                                                           { return m_pocMsbCycleLen;                                             }
  void                    setNumExtraPHBytes(int i)                                                           { m_numExtraPHBytes = i;                                               }
  int                     getNumExtraPHBytes() const                                                          { return m_numExtraPHBytes;                                            }
  void                    setNumExtraSHBytes(int i)                                                           { m_numExtraSHBytes = i;                                               }
  int                     getNumExtraSHBytes() const                                                          { return m_numExtraSHBytes;                                            }
  void                    setExtraPHBitPresentFlags(const std::vector<bool> &b)                               { m_extraPHBitPresentFlag = b;                                         }
  const std::vector<bool> getExtraPHBitPresentFlags() const                                                   { return m_extraPHBitPresentFlag;                                      }
  void                    setExtraSHBitPresentFlags(const std::vector<bool> &b)                               { m_extraSHBitPresentFlag = b;                                         }
  const std::vector<bool> getExtraSHBitPresentFlags() const                                                   { return m_extraSHBitPresentFlag;                                      }
  void                    setMaxNumReorderPics(int i, uint32_t tlayer)                                        { m_maxNumReorderPics[tlayer] = i;                                        }
  int                     getMaxNumReorderPics(uint32_t tlayer) const                                         { return m_maxNumReorderPics[tlayer];                                     }
  void                    createRPLList0(int numRPL);
  void                    createRPLList1(int numRPL);
  const RPLList*          getRPLList( bool b ) const                                                          { return b==1 ? &m_RPLList1 : &m_RPLList0;                             }
  RPLList*                getRPLList( bool b )                                                                { return b==1 ? &m_RPLList1 : &m_RPLList0;                             }
  uint32_t                getNumRPL( bool b ) const                                                           { return b==1 ? m_numRPL1   : m_numRPL0;                               }
  const RPLList*          getRPLList0() const                                                                 { return &m_RPLList0;                                                  }
  RPLList*                getRPLList0()                                                                       { return &m_RPLList0;                                                  }
  const RPLList*          getRPLList1() const                                                                 { return &m_RPLList1;                                                  }
  RPLList*                getRPLList1()                                                                       { return &m_RPLList1;                                                  }
  uint32_t                getNumRPL0() const                                                                  { return m_numRPL0;                                                    }
  uint32_t                getNumRPL1() const                                                                  { return m_numRPL1;                                                    }
  void                    setRPL1CopyFromRPL0Flag(bool isCopy)                                                { m_rpl1CopyFromRpl0Flag = isCopy;                                     }
  bool                    getRPL1CopyFromRPL0Flag() const                                                     { return m_rpl1CopyFromRpl0Flag;                                       }
  bool                    getRPL1IdxPresentFlag() const                                                       { return m_rpl1IdxPresentFlag;                                         }
  void                    setAllActiveRplEntriesHasSameSignFlag(bool isAllSame)                               { m_allRplEntriesHasSameSignFlag = isAllSame;                          }
  bool                    getAllActiveRplEntriesHasSameSignFlag() const                                       { return m_allRplEntriesHasSameSignFlag;                               }
  bool                    getLongTermRefsPresent() const                                                  { return m_bLongTermRefsPresent;                                       }
  void                    setLongTermRefsPresent(bool b)                                                  { m_bLongTermRefsPresent=b;                                            }
  bool                    getSPSTemporalMVPEnabledFlag() const                                            { return m_SPSTemporalMVPEnabledFlag;                                  }
  void                    setSPSTemporalMVPEnabledFlag(bool b)                                            { m_SPSTemporalMVPEnabledFlag=b;                                       }
  void                    setLog2MaxTbSize( uint32_t u )                                                  { m_log2MaxTbSize = u;                                                 }
  uint32_t                getLog2MaxTbSize() const                                                        { return  m_log2MaxTbSize;                                             }
  uint32_t                getMaxTbSize() const                                                            { return  1 << m_log2MaxTbSize;                                        }
  // Bit-depth
  int                     getBitDepth(ChannelType type) const                                             { return m_bitDepths.recon[type];                                      }
  void                    setBitDepth(ChannelType type, int u )                                           { m_bitDepths.recon[type] = u;                                         }
  const BitDepths&        getBitDepths() const                                                            { return m_bitDepths;                                                  }

  bool                    getEntropyCodingSyncEnabledFlag() const                                         { return m_entropyCodingSyncEnabledFlag;                               }
  void                    setEntropyCodingSyncEnabledFlag(bool val)                                       { m_entropyCodingSyncEnabledFlag = val;                                }
  bool                    getEntryPointsPresentFlag() const                                               { return m_entryPointPresentFlag;                                      }
  void                    setEntryPointsPresentFlag(bool val)                                             { m_entryPointPresentFlag = val;                                       }
  int                     getMaxLog2TrDynamicRange(ChannelType channelType) const                         { return getSpsRangeExtension().getExtendedPrecisionProcessingFlag() ? std::max<int>(15, int(m_bitDepths.recon[channelType] + 6)) : 15; }

  int                     getDifferentialLumaChromaBitDepth() const                                       { return int(m_bitDepths.recon[CHANNEL_TYPE_LUMA]) - int(m_bitDepths.recon[CHANNEL_TYPE_CHROMA]); }
  int                     getQpBDOffset(ChannelType type) const                                           { return m_qpBDOffset[type];                                           }
  void                    setQpBDOffset(ChannelType type, int i)                                          { m_qpBDOffset[type] = i;                                              }
  int                     getInternalMinusInputBitDepth(ChannelType type) const                           { return m_internalMinusInputBitDepth[type];                                           }
  void                    setInternalMinusInputBitDepth(ChannelType type, int i)                          { m_internalMinusInputBitDepth[type] = i;                                              }

  void                    setSAOEnabledFlag(bool bVal)                                                    { m_saoEnabledFlag = bVal;                                                    }
  bool                    getSAOEnabledFlag() const                                                       { return m_saoEnabledFlag;                                                    }

  bool                    getALFEnabledFlag() const                                                       { return m_alfEnabledFlag; }
  void                    setALFEnabledFlag( bool b )                                                     { m_alfEnabledFlag = b; }
bool                    getCCALFEnabledFlag() const                                                       { return m_ccalfEnabledFlag; }
void                    setCCALFEnabledFlag( bool b )                                                     { m_ccalfEnabledFlag = b; }
  void                    setJointCbCrEnabledFlag(bool bVal)                                              { m_JointCbCrEnabledFlag = bVal; }
  bool                    getJointCbCrEnabledFlag() const                                                 { return m_JointCbCrEnabledFlag; }

  bool getSbTMVPEnabledFlag() const { return m_sbtmvpEnabledFlag; }
  void setSbTMVPEnabledFlag(bool b) { m_sbtmvpEnabledFlag = b; }

  void                    setBDOFEnabledFlag(bool b)                                                      { m_bdofEnabledFlag = b; }
  bool                    getBDOFEnabledFlag() const                                                      { return m_bdofEnabledFlag; }

  bool                    getFpelMmvdEnabledFlag() const                                                  { return m_fpelMmvdEnabledFlag; }
  void                    setFpelMmvdEnabledFlag( bool b )                                                { m_fpelMmvdEnabledFlag = b;    }
  bool                    getUseDMVR()const                                                               { return m_DMVR; }
  void                    setUseDMVR(bool b)                                                              { m_DMVR = b;    }
  bool                    getUseMMVD()const                                                               { return m_MMVD; }
  void                    setUseMMVD(bool b)                                                              { m_MMVD = b;    }
  bool                    getBdofControlPresentInPhFlag()const                                            { return m_BdofControlPresentInPhFlag; }
  void                    setBdofControlPresentInPhFlag(bool b)                                           { m_BdofControlPresentInPhFlag = b;    }

  bool                    getDmvrControlPresentInPhFlag()const                                            { return m_DmvrControlPresentInPhFlag; }
  void                    setDmvrControlPresentInPhFlag(bool b)                                           { m_DmvrControlPresentInPhFlag = b;    }

  bool                    getProfControlPresentInPhFlag()const                                            { return m_ProfControlPresentInPhFlag; }
  void                    setProfControlPresentInPhFlag(bool b)                                           { m_ProfControlPresentInPhFlag = b;    }
  uint32_t                getMaxTLayers() const                                                           { return m_uiMaxTLayers; }
  void                    setMaxTLayers( uint32_t uiMaxTLayers )                                          {
    CHECK_( uiMaxTLayers > MAX_TLAYER, "Invalid number T-layers" ); m_uiMaxTLayers = uiMaxTLayers; }

  bool                    getPtlDpbHrdParamsPresentFlag()  const                                          { return m_ptlDpbHrdParamsPresentFlag;     }
  void                    setPtlDpbHrdParamsPresentFlag(bool b)                                           {        m_ptlDpbHrdParamsPresentFlag = b; }
  bool                    getSubLayerDpbParamsFlag()  const                                               { return m_SubLayerDpbParamsFlag;          }
  void                    setSubLayerDpbParamsFlag(bool b)                                                {        m_SubLayerDpbParamsFlag = b;      }
  bool                    getTemporalIdNestingFlag() const                                                { return m_bTemporalIdNestingFlag;                                     }
  void                    setTemporalIdNestingFlag( bool bValue )                                         { m_bTemporalIdNestingFlag = bValue;                                   }

  bool                    getScalingListFlag() const                                                      { return m_scalingListEnabledFlag;                                     }
  void                    setScalingListFlag( bool b )                                                    { m_scalingListEnabledFlag  = b;                                       }
  void                    setDepQuantEnabledFlag(bool b)                                                  { m_depQuantEnabledFlag = b; }
  bool                    getDepQuantEnabledFlag() const                                                  { return m_depQuantEnabledFlag; }
  void                    setSignDataHidingEnabledFlag(bool b)                                            { m_signDataHidingEnabledFlag = b; }
  bool                    getSignDataHidingEnabledFlag() const                                            { return m_signDataHidingEnabledFlag; }
  void                    setVirtualBoundariesEnabledFlag( bool b )                                       { m_virtualBoundariesEnabledFlag = b;                                  }
  bool                    getVirtualBoundariesEnabledFlag() const                                         { return m_virtualBoundariesEnabledFlag;                               }
  void                    setVirtualBoundariesPresentFlag( bool b )                                       { m_virtualBoundariesPresentFlag = b; }
  bool                    getVirtualBoundariesPresentFlag() const                                         { return m_virtualBoundariesPresentFlag; }
  void                    setNumVerVirtualBoundaries(unsigned u)                                          { m_numVerVirtualBoundaries = u;                                       }
  unsigned                getNumVerVirtualBoundaries() const                                              { return m_numVerVirtualBoundaries;                                    }
  void                    setNumHorVirtualBoundaries(unsigned u)                                          { m_numHorVirtualBoundaries = u;                                       }
  unsigned                getNumHorVirtualBoundaries() const                                              { return m_numHorVirtualBoundaries;                                    }
  void                    setVirtualBoundariesPosX(unsigned u, unsigned idx)                              {
    CHECK_( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;    }
  unsigned                getVirtualBoundariesPosX(unsigned idx) const                                    {
    CHECK_( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx]; }
  void                    setVirtualBoundariesPosY(unsigned u, unsigned idx)                              {
    CHECK_( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;    }
  unsigned                getVirtualBoundariesPosY(unsigned idx) const                                    {
    CHECK_( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx]; }
  uint32_t                getMaxDecPicBuffering(uint32_t tlayer) const                                    { return m_uiMaxDecPicBuffering[tlayer];                               }
  void                    setMaxDecPicBuffering( uint32_t ui, uint32_t tlayer )                           {
    CHECK_(tlayer >= MAX_TLAYER, "Invalid T-layer"); m_uiMaxDecPicBuffering[tlayer] = ui;    }
  uint32_t                getMaxLatencyIncreasePlus1(uint32_t tlayer) const                               { return m_uiMaxLatencyIncreasePlus1[tlayer];                          }
  void                    setMaxLatencyIncreasePlus1( uint32_t ui , uint32_t tlayer)                      { m_uiMaxLatencyIncreasePlus1[tlayer] = ui;                            }
  uint32_t                getMaxNumMergeCand() const { return m_maxNumMergeCand; }
  void                    setMaxNumMergeCand(uint32_t u) { m_maxNumMergeCand = u; }
  uint32_t                getMaxNumAffineMergeCand() const { return m_maxNumAffineMergeCand; }
  void                    setMaxNumAffineMergeCand(uint32_t u) { m_maxNumAffineMergeCand = u; }
  uint32_t                getMaxNumIBCMergeCand() const { return m_maxNumIBCMergeCand; }
  void                    setMaxNumIBCMergeCand(uint32_t u) { m_maxNumIBCMergeCand = u; }
  uint32_t                getMaxNumGeoCand() const { return m_maxNumGeoCand; }
  void                    setMaxNumGeoCand(uint32_t u) { m_maxNumGeoCand = u; }
  void                    setAffineAmvrEnabledFlag( bool val )                                            { m_affineAmvrEnabledFlag = val;                                       }
  bool                    getAffineAmvrEnabledFlag() const                                                { return m_affineAmvrEnabledFlag;                                      }
  bool                    getGeneralHrdParametersPresentFlag() const { return m_generalHrdParametersPresentFlag; }
  void                    setGeneralHrdParametersPresentFlag(bool b) { m_generalHrdParametersPresentFlag = b; }
  OlsHrdParams*          getOlsHrdParameters() { return &m_olsHrdParams[0]; }
  const OlsHrdParams*    getOlsHrdParameters() const { return &m_olsHrdParams[0]; }

  GeneralHrdParams*          getGeneralHrdParameters() { return &m_generalHrdParams; }
  const GeneralHrdParams*    getGeneralHrdParameters() const { return &m_generalHrdParams; }
  bool                    getFieldSeqFlag() const                                                         { return m_fieldSeqFlag;                         }
  void                    setFieldSeqFlag(bool i)                                                         { m_fieldSeqFlag = i;                            }
  bool                    getVuiParametersPresentFlag() const                                             { return m_vuiParametersPresentFlag;                                   }
  void                    setVuiParametersPresentFlag(bool b)                                             { m_vuiParametersPresentFlag = b;                                      }
  unsigned                getVuiPayloadSize() const                                                       { return m_vuiPayloadSize; }
  void                    setVuiPayloadSize(unsigned i)                                                   { m_vuiPayloadSize = i; }
  VUI*                    getVuiParameters()                                                              { return &m_vuiParameters;                                             }
  const VUI*              getVuiParameters() const                                                        { return &m_vuiParameters;                                             }
  const ProfileTierLevel* getProfileTierLevel() const                                                     { return &m_profileTierLevel; }
  ProfileTierLevel*       getProfileTierLevel()                                                           { return &m_profileTierLevel; }

  const SPSRExt&          getSpsRangeExtension() const                                                    { return m_spsRangeExtension;                                          }
  SPSRExt&                getSpsRangeExtension()                                                          { return m_spsRangeExtension;                                          }

  void                    setWrapAroundEnabledFlag(bool b)                                                { m_wrapAroundEnabledFlag = b;                                         }
  bool                    getWrapAroundEnabledFlag() const                                                { return m_wrapAroundEnabledFlag;                                      }
  void                    setUseLmcs(bool b)                                                              { m_lmcsEnabled = b;                                                   }
  bool                    getUseLmcs() const                                                              { return m_lmcsEnabled;                                                }
  void                    setIBCFlag(unsigned IBCFlag)                                                    { m_IBCFlag = IBCFlag; }
  unsigned                getIBCFlag() const                                                              { return m_IBCFlag; }
  void                    setUseColorTrans(bool value) { m_useColorTrans = value; }
  bool                    getUseColorTrans() const { return m_useColorTrans; }
  void                    setPLTMode(unsigned PLTMode)                                                    { m_PLTMode = PLTMode; }
  unsigned                getPLTMode() const                                                              { return m_PLTMode; }
  void                    setUseSBT( bool b )                                                             { m_SBT = b; }
  bool                    getUseSBT() const                                                               { return m_SBT; }
  void                    setUseISP( bool b )                                                             { m_ISP = b; }
  bool                    getUseISP() const                                                               { return m_ISP; }

  void      setAMVREnabledFlag    ( bool b )                                        { m_AMVREnabledFlag = b; }
  bool      getAMVREnabledFlag    ()                                      const     { return m_AMVREnabledFlag; }
  void      setUseAffine          ( bool b )                                        { m_Affine = b; }
  bool      getUseAffine          ()                                      const     { return m_Affine; }
  void      setUseAffineType      ( bool b )                                        { m_AffineType = b; }
  bool      getUseAffineType      ()                                      const     { return m_AffineType; }
  void      setUsePROF            ( bool b )                                        { m_PROF = b; }
  bool      getUsePROF            ()                                      const     { return m_PROF; }
  void      setUseLMChroma        ( bool b )                                        { m_LMChroma = b; }
  bool      getUseLMChroma        ()                                      const     { return m_LMChroma; }
  void      setHorCollocatedChromaFlag( bool b )                                    { m_horCollocatedChromaFlag = b;    }
  bool      getHorCollocatedChromaFlag()                                  const     { return m_horCollocatedChromaFlag; }
  void      setVerCollocatedChromaFlag( bool b )                                    { m_verCollocatedChromaFlag = b;    }
  bool      getVerCollocatedChromaFlag()                                  const     { return m_verCollocatedChromaFlag; }
  bool      getCclmCollocatedChromaFlag()                                 const     { return m_verCollocatedChromaFlag; }
  void      setUseMTS             ( bool b )                                        { m_MTS = b; }
  bool      getUseMTS             ()                                      const     { return m_MTS; }
  bool      getUseImplicitMTS     ()                                      const     { return m_MTS && !m_IntraMTS; }
  void      setUseIntraMTS        ( bool b )                                        { m_IntraMTS = b; }
  bool      getUseIntraMTS        ()                                      const     { return m_IntraMTS; }
  void      setUseInterMTS        ( bool b )                                        { m_InterMTS = b; }
  bool      getUseInterMTS        ()                                      const     { return m_InterMTS; }
  void      setUseLFNST           ( bool b )                                        { m_LFNST = b; }
  bool      getUseLFNST           ()                                      const     { return m_LFNST; }
  void      setUseSMVD(bool b)                                                      { m_SMVD = b; }
  bool      getUseSMVD()                                                  const     { return m_SMVD; }
  void      setUseBcw             ( bool b )                                        { m_bcw = b; }
  bool      getUseBcw             ()                                      const     { return m_bcw; }
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  void      setLadfEnabled        ( bool b )                                        { m_LadfEnabled = b; }
  bool      getLadfEnabled        ()                                      const     { return m_LadfEnabled; }
  void      setLadfNumIntervals   ( int i )                                         { m_LadfNumIntervals = i; }
  int       getLadfNumIntervals   ()                                      const     { return m_LadfNumIntervals; }
  void      setLadfQpOffset       ( int value, int idx )                            { m_LadfQpOffset[ idx ] = value; }
  int       getLadfQpOffset       ( int idx )                             const     { return m_LadfQpOffset[ idx ]; }
  void      setLadfIntervalLowerBound( int value, int idx )                         { m_LadfIntervalLowerBound[ idx ] = value; }
  int       getLadfIntervalLowerBound( int idx )                          const     { return m_LadfIntervalLowerBound[ idx ]; }
#endif

  void      setUseCiip         ( bool b )                                        { m_ciip = b; }
  bool      getUseCiip         ()                                      const     { return m_ciip; }
  void      setUseGeo             ( bool b )                                        { m_Geo = b; }
  bool      getUseGeo             ()                                      const     { return m_Geo; }
  void      setUseMRL             ( bool b )                                        { m_MRL = b; }
  bool      getUseMRL             ()                                      const     { return m_MRL; }
  void      setUseMIP             ( bool b )                                        { m_MIP = b; }
  bool      getUseMIP             ()                                      const     { return m_MIP; }

  bool      getUseWP              ()                                      const     { return m_useWeightPred; }
  bool      getUseWPBiPred        ()                                      const     { return m_useWeightedBiPred; }
  void      setUseWP              ( bool b )                                        { m_useWeightPred = b; }
  void      setUseWPBiPred        ( bool b )                                        { m_useWeightedBiPred = b; }
  void      setChromaQpMappingTableFromParams(const ChromaQpMappingTableParams &params, const int qpBdOffset)   { m_chromaQpMappingTable.setParams(params, qpBdOffset); }
  void      derivedChromaQPMappingTables()                                          { m_chromaQpMappingTable.derivedChromaQPMappingTables(); }
  const ChromaQpMappingTable& getChromaQpMappingTable()                   const     { return m_chromaQpMappingTable;}
  int       getMappedChromaQpValue(ComponentID compID, int qpVal)         const     { return m_chromaQpMappingTable.getMappedChromaQpValue(compID, qpVal); }
  void setGDREnabledFlag(bool flag) { m_GDREnabledFlag = flag; }
  bool getGDREnabledFlag() const { return m_GDREnabledFlag; }
  void      setSubLayerParametersPresentFlag(bool flag)                             { m_SubLayerCbpParametersPresentFlag = flag; }
  bool      getSubLayerParametersPresentFlag()                            const     { return m_SubLayerCbpParametersPresentFlag;  }

  bool      getRprEnabledFlag()                                           const     { return m_rprEnabledFlag; }
  void      setRprEnabledFlag( bool flag )                                          { m_rprEnabledFlag = flag; }
  bool      getInterLayerPresentFlag()                                        const { return m_interLayerPresentFlag; }
  void      setInterLayerPresentFlag( bool b )                                      { m_interLayerPresentFlag = b; }
  bool      getResChangeInClvsEnabledFlag()                               const     { return m_resChangeInClvsEnabledFlag; }
  void      setResChangeInClvsEnabledFlag(bool flag)                                { m_resChangeInClvsEnabledFlag = flag; }

  uint32_t  getLog2ParallelMergeLevelMinus2() const { return m_log2ParallelMergeLevelMinus2; }
  void      setLog2ParallelMergeLevelMinus2(uint32_t mrgLevel) { m_log2ParallelMergeLevelMinus2 = mrgLevel; }
  void          setPPSValidFlag(int i, bool b) { m_ppsValidFlag[i] = b; }
  bool          getPPSValidFlag(int i)         { return m_ppsValidFlag[i]; }
  void          setScalingWindowSizeInPPS(int i, int scWidth, int scHeight) { m_scalingWindowSizeInPPS[i].width = scWidth; m_scalingWindowSizeInPPS[i].height = scHeight;}
  const Size&   getScalingWindowSizeInPPS(int i)                            { return m_scalingWindowSizeInPPS[i]; }
  void      setScalingMatrixForAlternativeColourSpaceDisabledFlag(bool b)           { m_scalingMatrixAlternativeColourSpaceDisabledFlag = b; }
  bool      getScalingMatrixForAlternativeColourSpaceDisabledFlag()           const { return m_scalingMatrixAlternativeColourSpaceDisabledFlag; }
  void      setScalingMatrixDesignatedColourSpaceFlag(bool b)                       { m_scalingMatrixDesignatedColourSpaceFlag = b; }
  bool      getScalingMatrixDesignatedColourSpaceFlag()                       const { return m_scalingMatrixDesignatedColourSpaceFlag; }
};


/// PPS class
class PPS
{
private:
  int              m_PPSId;                    // pic_parameter_set_id
  int              m_SPSId;                    // seq_parameter_set_id
  int              m_picInitQPMinus26;
  bool             m_useDQP;
  bool             m_usePPSChromaTool;
  bool             m_bSliceChromaQpFlag;       // slicelevel_chroma_qp_flag

  int              m_layerId;
  int              m_temporalId;
  int              m_puCounter;

  // access channel

  int              m_chromaCbQpOffset;
  int              m_chromaCrQpOffset;
  bool             m_chromaJointCbCrQpOffsetPresentFlag;
  int              m_chromaCbCrQpOffset;

  // Chroma QP Adjustments
  int              m_chromaQpOffsetListLen; // size (excludes the null entry used in the following array).
  ChromaQpAdj      m_ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  uint32_t             m_numRefIdxL0DefaultActive;
  uint32_t             m_numRefIdxL1DefaultActive;

  bool             m_rpl1IdxPresentFlag;

  bool             m_bUseWeightPred;                    //!< Use of Weighting Prediction (P_SLICE)
  bool             m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)
  bool             m_OutputFlagPresentFlag;             //!< Indicates the presence of output_flag in slice header
  uint32_t         m_numSubPics;                        //!< number of sub-pictures used - must match SPS
  bool             m_subPicIdMappingInPpsFlag;
  uint32_t         m_subPicIdLen;                       //!< sub-picture ID length in bits
  std::vector<uint16_t> m_subPicId;                     //!< sub-picture ID for each sub-picture in the sequence
  bool             m_noPicPartitionFlag;                //!< no picture partitioning flag - single slice, single tile
  uint8_t          m_log2CtuSize;                       //!< log2 of the CTU size - required to match corresponding value in SPS
  uint8_t          m_ctuSize;                           //!< CTU size
  uint32_t         m_picWidthInCtu;                     //!< picture width in units of CTUs
  uint32_t         m_picHeightInCtu;                    //!< picture height in units of CTUs
  uint32_t         m_numExpTileCols;                    //!< number of explicitly specified tile columns
  uint32_t         m_numExpTileRows;                    //!< number of explicitly specified tile rows
  uint32_t         m_numTileCols;                       //!< number of tile columns
  uint32_t         m_numTileRows;                       //!< number of tile rows
  std::vector<uint32_t> m_tileColWidth;                 //!< tile column widths in units of CTUs
  std::vector<uint32_t> m_tileRowHeight;                //!< tile row heights in units of CTUs
  std::vector<uint32_t> m_tileColBd;                    //!< tile column left-boundaries in units of CTUs
  std::vector<uint32_t> m_tileRowBd;                    //!< tile row top-boundaries in units of CTUs
  std::vector<uint32_t> m_ctuToTileCol;                 //!< mapping between CTU horizontal address and tile column index
  std::vector<uint32_t> m_ctuToTileRow;                 //!< mapping between CTU vertical address and tile row index
  bool             m_rectSliceFlag;                     //!< rectangular slice flag
  bool             m_singleSlicePerSubPicFlag;          //!< single slice per sub-picture flag
  std::vector<uint32_t> m_ctuToSubPicIdx;               //!< mapping between CTU and Sub-picture index
  uint32_t         m_numSlicesInPic;                    //!< number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool             m_tileIdxDeltaPresentFlag;           //!< tile index delta present flag
  std::vector<RectSlice> m_rectSlices;                  //!< list of rectangular slice signalling parameters
  std::vector<SliceMap>  m_sliceMap;                    //!< list of CTU maps for each slice in the picture
  std::vector<SubPic>      m_subPics;                   //!< list of subpictures in the picture
  bool             m_loopFilterAcrossTilesEnabledFlag;  //!< loop filtering applied across tiles flag
  bool             m_loopFilterAcrossSlicesEnabledFlag; //!< loop filtering applied across slices flag


  bool             m_cabacInitPresentFlag;

  bool             m_pictureHeaderExtensionPresentFlag;   //< picture header extension flags present in picture headers or not
  bool             m_sliceHeaderExtensionPresentFlag;
  bool             m_deblockingFilterControlPresentFlag;
  bool             m_deblockingFilterOverrideEnabledFlag;
  bool             m_ppsDeblockingFilterDisabledFlag;
  int              m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  int              m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  int              m_deblockingFilterCbBetaOffsetDiv2;    //< beta offset for Cb deblocking filter
  int              m_deblockingFilterCbTcOffsetDiv2;      //< tc offset for Cb deblocking filter
  int              m_deblockingFilterCrBetaOffsetDiv2;    //< beta offset for Cr deblocking filter
  int              m_deblockingFilterCrTcOffsetDiv2;      //< tc offset for Cr deblocking filter
  bool             m_listsModificationPresentFlag;

  bool             m_rplInfoInPhFlag;
  bool             m_dbfInfoInPhFlag;
  bool             m_saoInfoInPhFlag;
  bool             m_alfInfoInPhFlag;
  bool             m_wpInfoInPhFlag;
  bool             m_qpDeltaInfoInPhFlag;
  bool             m_mixedNaluTypesInPicFlag;

  bool             m_conformanceWindowFlag;
  uint32_t         m_picWidthInLumaSamples;
  uint32_t         m_picHeightInLumaSamples;
  Window           m_conformanceWindow;
  Window           m_scalingWindow;

  bool             m_wrapAroundEnabledFlag;               //< reference wrap around enabled or not
  unsigned         m_picWidthMinusWrapAroundOffset;          // <pic_width_in_minCbSizeY - wraparound_offset_in_minCbSizeY
  unsigned         m_wrapAroundOffset;                    //< reference wrap around offset in luma samples

public:
  PreCalcValues   *pcv;

public:
                         PPS();
  virtual                ~PPS();

  int                    getPPSId() const                                                 { return m_PPSId;                               }
  void                   setPPSId(int i)                                                  { m_PPSId = i;                                  }
  int                    getSPSId() const                                                 { return m_SPSId;                               }
  void                   setSPSId(int i)                                                  { m_SPSId = i;                                  }

  void                   setTemporalId( int i )                                           { m_temporalId = i;                             }
  int                    getTemporalId()                                            const { return m_temporalId;                          }
  void                   setPuCounter(int i)                                              { m_puCounter = i;                              }
  int                    getPuCounter()                                             const { return m_puCounter;                           }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }

  int                    getPicInitQPMinus26() const                                      { return  m_picInitQPMinus26;                   }
  void                   setPicInitQPMinus26( int i )                                     { m_picInitQPMinus26 = i;                       }
  bool                   getUseDQP() const                                                { return m_useDQP;                              }
  void                   setUseDQP( bool b )                                              { m_useDQP   = b;                               }
  bool                   getPPSChromaToolFlag()                                     const { return  m_usePPSChromaTool;                   }
  void                   setPPSChromaToolFlag(bool b)                                     { m_usePPSChromaTool = b;                       }
  bool                   getSliceChromaQpFlag() const                                     { return  m_bSliceChromaQpFlag;                 }
  void                   setSliceChromaQpFlag( bool b )                                   { m_bSliceChromaQpFlag = b;                     }


  bool                   getJointCbCrQpOffsetPresentFlag() const                          { return m_chromaJointCbCrQpOffsetPresentFlag;   }
  void                   setJointCbCrQpOffsetPresentFlag(bool b)                          { m_chromaJointCbCrQpOffsetPresentFlag = b;      }

  void                   setQpOffset(ComponentID compID, int i )
  {
    if      (compID==COMPONENT_Cb)
    {
      m_chromaCbQpOffset = i;
    }
    else if (compID==COMPONENT_Cr)
    {
      m_chromaCrQpOffset = i;
    }
    else if (compID==JOINT_CbCr)
    {
      m_chromaCbCrQpOffset = i;
    }
    else
    {
      THROW( "Invalid chroma QP offset" );
    }
  }
  int                    getQpOffset(ComponentID compID) const
  {
    return (compID==COMPONENT_Y) ? 0 : (compID==COMPONENT_Cb ? m_chromaCbQpOffset : compID==COMPONENT_Cr ? m_chromaCrQpOffset : m_chromaCbCrQpOffset );
  }

  bool                   getCuChromaQpOffsetListEnabledFlag() const                       { return getChromaQpOffsetListLen()>0;            }
  int                    getChromaQpOffsetListLen() const                                 { return m_chromaQpOffsetListLen;                 }
  void                   clearChromaQpOffsetList()                                        { m_chromaQpOffsetListLen = 0;                    }

  const ChromaQpAdj&     getChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1 ) const
  {
    CHECK_(cuChromaQpOffsetIdxPlus1 >= m_chromaQpOffsetListLen+1, "Invalid chroma QP offset");
    return m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }

  void                   setChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1, int cbOffset, int crOffset, int jointCbCrOffset )
  {
    CHECK_(cuChromaQpOffsetIdxPlus1 == 0 || cuChromaQpOffsetIdxPlus1 > MAX_QP_OFFSET_LIST_SIZE, "Invalid chroma QP offset");
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CbOffset = cbOffset; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CrOffset = crOffset;
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.JointCbCrOffset = jointCbCrOffset;
    m_chromaQpOffsetListLen = std::max(m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
  }

  void                   setNumRefIdxL0DefaultActive(uint32_t ui)                             { m_numRefIdxL0DefaultActive=ui;                }
  uint32_t                   getNumRefIdxL0DefaultActive() const                              { return m_numRefIdxL0DefaultActive;            }
  void                   setNumRefIdxL1DefaultActive(uint32_t ui)                             { m_numRefIdxL1DefaultActive=ui;                }
  uint32_t                   getNumRefIdxL1DefaultActive() const                              { return m_numRefIdxL1DefaultActive;            }

  void                   setRpl1IdxPresentFlag(bool isPresent)                            { m_rpl1IdxPresentFlag = isPresent;             }
  uint32_t               getRpl1IdxPresentFlag() const                                    { return m_rpl1IdxPresentFlag;                  }

  bool                   getUseWP() const                                                 { return m_bUseWeightPred;                      }
  bool                   getWPBiPred() const                                              { return m_useWeightedBiPred;                   }
  void                   setUseWP( bool b )                                               { m_bUseWeightPred = b;                         }
  void                   setWPBiPred( bool b )                                            { m_useWeightedBiPred = b;                      }

  void                   setWrapAroundEnabledFlag(bool b)                                 { m_wrapAroundEnabledFlag = b;                  }
  bool                   getWrapAroundEnabledFlag() const                                 { return m_wrapAroundEnabledFlag;               }  
  void                   setPicWidthMinusWrapAroundOffset(unsigned offset)                { m_picWidthMinusWrapAroundOffset = offset;     }
  unsigned               getPicWidthMinusWrapAroundOffset() const                         { return m_picWidthMinusWrapAroundOffset;       }
  void                   setWrapAroundOffset(unsigned offset)                             { m_wrapAroundOffset = offset;                  }
  unsigned               getWrapAroundOffset() const                                      { return m_wrapAroundOffset;                    }
  void                   setOutputFlagPresentFlag( bool b )                               { m_OutputFlagPresentFlag = b;                  }
  bool                   getOutputFlagPresentFlag() const                                 { return m_OutputFlagPresentFlag;               }
  void                   setNumSubPics(uint32_t u )                                       {
    CHECK_( u >= MAX_NUM_SUB_PICS, "Maximum number of subpictures exceeded" );
                                                                                            m_numSubPics = u;
                                                                                            m_subPicId.resize(m_numSubPics);
                                                                                          }
  uint32_t               getNumSubPics( ) const                                           { return  m_numSubPics;                         }
  void                   setSubPicIdMappingInPpsFlag( bool b )                            { m_subPicIdMappingInPpsFlag = b;               }
  bool                   getSubPicIdMappingInPpsFlag() const                              { return m_subPicIdMappingInPpsFlag;            }
  void                   setSubPicIdLen( uint32_t u )                                     { m_subPicIdLen = u;                            }
  uint32_t               getSubPicIdLen() const                                           { return  m_subPicIdLen;                        }
  void                   setSubPicId( int i, uint16_t u )                                 { m_subPicId[i] = u;     }
  void                   setSubPicId(const std::vector<uint16_t> &v)                      {
    CHECK_(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ; m_subPicId = v; }
  uint16_t               getSubPicId( int i ) const                                       { return  m_subPicId[i]; }
  const std::vector<uint16_t> getSubPicIds() const                                        { return  m_subPicId; }
  uint32_t               getSubPicIdxFromSubPicId( uint32_t subPicId ) const;
  void                   setNoPicPartitionFlag( bool b )                                  { m_noPicPartitionFlag = b;                     }
  bool                   getNoPicPartitionFlag( ) const                                   { return  m_noPicPartitionFlag;                 }
  void                   setLog2CtuSize( uint8_t u )                                      { m_log2CtuSize = u; m_ctuSize = 1 << m_log2CtuSize;
                                                                                            m_picWidthInCtu = (m_picWidthInLumaSamples  + m_ctuSize - 1) / m_ctuSize;
                                                                                            m_picHeightInCtu = (m_picHeightInLumaSamples  + m_ctuSize - 1) / m_ctuSize; }
  uint8_t                getLog2CtuSize( ) const                                          { return  m_log2CtuSize;                        }
  uint8_t                getCtuSize( ) const                                              { return  m_ctuSize;                            }
  uint32_t               getPicWidthInCtu( ) const                                        { return  m_picWidthInCtu;                      }
  uint32_t               getPicHeightInCtu( ) const                                       { return  m_picHeightInCtu;                     }
  void                   setNumExpTileColumns( uint32_t u )                               { m_numExpTileCols = u;                         }
  uint32_t               getNumExpTileColumns( ) const                                    { return  m_numExpTileCols;                     }
  void                   setNumExpTileRows( uint32_t u )                                  { m_numExpTileRows = u;                         }
  uint32_t               getNumExpTileRows( ) const                                       { return  m_numExpTileRows;                     }
  void                   setNumTileColumns( uint32_t u )                                  { m_numTileCols = u;                            }
  uint32_t               getNumTileColumns( ) const                                       { return  m_numTileCols;                        }
  void                   setNumTileRows( uint32_t u )                                     { m_numTileRows = u;                            }
  uint32_t               getNumTileRows( ) const                                          { return  m_numTileRows;                        }
  uint32_t               getNumTiles( ) const                                             { return  m_numTileCols * m_numTileRows;        }
  void                   setTileColumnWidths( std::vector<uint32_t> widths )              { m_tileColWidth = widths;                      }
  void                   setTileRowHeights( std::vector<uint32_t> heights )               { m_tileRowHeight = heights;                    }
  void                   addTileColumnWidth( uint32_t u )                                 {
    CHECK_( m_tileColWidth.size()  >= MAX_TILE_COLS, "Number of tile columns exceeds valid range" ); m_tileColWidth.push_back(u);    }
  void                   addTileRowHeight( uint32_t u )                                   { m_tileRowHeight.push_back(u);   }
  uint32_t               getTileColumnWidth( int idx ) const                              {
    CHECK_( idx >= m_tileColWidth.size(), "Tile column index exceeds valid range" );                 return  m_tileColWidth[idx];    }
  uint32_t               getTileRowHeight( int idx ) const                                {
    CHECK_( idx >= m_tileRowHeight.size(), "Tile row index exceeds valid range" );                   return  m_tileRowHeight[idx];   }
  uint32_t               getTileColumnBd( int idx ) const                                 {
    CHECK_( idx >= m_tileColBd.size(), "Tile column index exceeds valid range" );                    return  m_tileColBd[idx];       }
  uint32_t               getTileRowBd( int idx ) const                                    {
    CHECK_( idx >= m_tileRowBd.size(), "Tile row index exceeds valid range" );                       return  m_tileRowBd[idx];       }
  uint32_t               ctuToTileCol( int ctuX ) const                                   {
    CHECK_( ctuX >= m_ctuToTileCol.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileCol[ctuX];                  }
  uint32_t               ctuToTileRow( int ctuY ) const                                   {
    CHECK_( ctuY >= m_ctuToTileRow.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileRow[ctuY];                  }
  uint32_t               ctuToTileColBd( int ctuX ) const                                 { return  getTileColumnBd(ctuToTileCol( ctuX ));                                                                                  }
  uint32_t               ctuToTileRowBd( int ctuY ) const                                 { return  getTileRowBd(ctuToTileRow( ctuY ));                                                                                     }
  bool                   ctuIsTileColBd( int ctuX ) const                                 { return  ctuX == ctuToTileColBd( ctuX );                                                                                         }
  bool                   ctuIsTileRowBd( int ctuY ) const                                 { return  ctuY == ctuToTileRowBd( ctuY );                                                                                         }
  uint32_t               getTileIdx( uint32_t ctuX, uint32_t ctuY ) const                 { return (ctuToTileRow( ctuY ) * getNumTileColumns()) + ctuToTileCol( ctuX );                                                     }
  uint32_t               getTileIdx( uint32_t ctuRsAddr) const                            { return getTileIdx( ctuRsAddr % m_picWidthInCtu,  ctuRsAddr / m_picWidthInCtu );                                                 }
  uint32_t               getTileIdx( const Position& pos ) const                          { return getTileIdx( pos.x / m_ctuSize, pos.y / m_ctuSize );                                                                      }
  void                   setRectSliceFlag( bool b )                                       { m_rectSliceFlag = b;                                                                                                            }
  bool                   getRectSliceFlag( ) const                                        { return  m_rectSliceFlag;                                                                                                        }
  void                   setSingleSlicePerSubPicFlag( bool b )                            { m_singleSlicePerSubPicFlag = b;                                                                                                 }
  bool                   getSingleSlicePerSubPicFlag( ) const                             { return  m_singleSlicePerSubPicFlag;                                                                                             }
  uint32_t               getCtuToSubPicIdx( int idx ) const                               {
    CHECK_( idx >= m_ctuToSubPicIdx.size(), "CTU address index exceeds valid range" );
    CHECK_( getNumSubPics() < 1, "Number of subpicture cannot be 0" ); return  m_ctuToSubPicIdx[ idx ]; }
  void                   setNumSlicesInPic( uint32_t u )                                  {
    CHECK_( u > MAX_SLICES, "Number of slices in picture exceeds valid range" ); m_numSlicesInPic = u;                               }
  uint32_t               getNumSlicesInPic( ) const                                       { return  m_numSlicesInPic;                                                                                                       }
  void                   setTileIdxDeltaPresentFlag( bool b )                             { m_tileIdxDeltaPresentFlag = b;                                                                                                  }
  bool                   getTileIdxDeltaPresentFlag( ) const                              { return  m_tileIdxDeltaPresentFlag;                                                                                              }
  void                   setSliceWidthInTiles( int idx, uint32_t u )                      {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceWidthInTiles( u );            }
  uint32_t               getSliceWidthInTiles( int idx ) const                            {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceWidthInTiles( );      }
  void                   setSliceHeightInTiles( int idx, uint32_t u )                     {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInTiles( u );           }
  uint32_t               getSliceHeightInTiles( int idx ) const                           {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInTiles( );     }
  void                   setNumSlicesInTile( int idx, uint32_t u )                        {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setNumSlicesInTile( u );              }
  uint32_t               getNumSlicesInTile( int idx ) const                              {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getNumSlicesInTile( );        }
  void                   setSliceHeightInCtu( int idx, uint32_t u )                       {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInCtu( u );             }
  uint32_t               getSliceHeightInCtu( int idx ) const                             {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInCtu( );       }
  void                   setSliceTileIdx(  int idx, uint32_t u )                          {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setTileIdx( u );                      }
  uint32_t               getSliceTileIdx( int idx ) const                                 {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getTileIdx( );                }
  void                   setRectSlices( std::vector<RectSlice> rectSlices )               { m_rectSlices = rectSlices;                                                                                                      }
  void                   setLoopFilterAcrossTilesEnabledFlag( bool b )                    { m_loopFilterAcrossTilesEnabledFlag = b;                                                                                         }
  bool                   getLoopFilterAcrossTilesEnabledFlag( ) const                     { return  m_loopFilterAcrossTilesEnabledFlag;                                                                                     }
  void                   setLoopFilterAcrossSlicesEnabledFlag( bool b )                   { m_loopFilterAcrossSlicesEnabledFlag = b;                                                                                        }
  bool                   getLoopFilterAcrossSlicesEnabledFlag( ) const                    { return  m_loopFilterAcrossSlicesEnabledFlag;                                                                                    }
  void                   resetTileSliceInfo();
  void                   initTiles();
  void                   initRectSlices();
  void                   initRectSliceMap(const SPS  *sps);
  std::vector<SubPic>    getSubPics()  const                                              {return m_subPics;          };
  SubPic                 getSubPic(uint32_t idx) const                                    { return m_subPics[idx]; }
  void                   initSubPic(const SPS &sps);
  const SubPic&          getSubPicFromPos(const Position& pos)  const;
  const SubPic&          getSubPicFromCU (const CodingUnit& cu) const;
  void                   initRasterSliceMap( std::vector<uint32_t> sizes );
  void                   checkSliceMap();
  SliceMap               getSliceMap( int idx ) const                                     {
    CHECK_( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return m_sliceMap[idx];                             }



  void                   setCabacInitPresentFlag( bool flag )                             { m_cabacInitPresentFlag = flag;                }
  bool                   getCabacInitPresentFlag() const                                  { return m_cabacInitPresentFlag;                }
  void                   setDeblockingFilterControlPresentFlag( bool val )                { m_deblockingFilterControlPresentFlag = val;   }
  bool                   getDeblockingFilterControlPresentFlag() const                    { return m_deblockingFilterControlPresentFlag;  }
  void                   setDeblockingFilterOverrideEnabledFlag( bool val )               { m_deblockingFilterOverrideEnabledFlag = val;  }
  bool                   getDeblockingFilterOverrideEnabledFlag() const                   { return m_deblockingFilterOverrideEnabledFlag; }
  void                   setPPSDeblockingFilterDisabledFlag(bool val)                     { m_ppsDeblockingFilterDisabledFlag = val;      } //!< set offset for deblocking filter disabled
  bool                   getPPSDeblockingFilterDisabledFlag() const                       { return m_ppsDeblockingFilterDisabledFlag;     } //!< get offset for deblocking filter disabled
  void                   setDeblockingFilterBetaOffsetDiv2(int val)                       { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  int                    getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  void                   setDeblockingFilterTcOffsetDiv2(int val)                         { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  int                    getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
  void                   setDeblockingFilterCbBetaOffsetDiv2(int val)                     { m_deblockingFilterCbBetaOffsetDiv2 = val;     } //!< set beta offset for Cb deblocking filter
  int                    getDeblockingFilterCbBetaOffsetDiv2() const                      { return m_deblockingFilterCbBetaOffsetDiv2;    } //!< get beta offset for Cb deblocking filter
  void                   setDeblockingFilterCbTcOffsetDiv2(int val)                       { m_deblockingFilterCbTcOffsetDiv2 = val;       } //!< set tc offset for Cb deblocking filter
  int                    getDeblockingFilterCbTcOffsetDiv2() const                        { return m_deblockingFilterCbTcOffsetDiv2;      } //!< get tc offset for Cb deblocking filter
  void                   setDeblockingFilterCrBetaOffsetDiv2(int val)                     { m_deblockingFilterCrBetaOffsetDiv2 = val;     } //!< set beta offset for Cr deblocking filter
  int                    getDeblockingFilterCrBetaOffsetDiv2() const                      { return m_deblockingFilterCrBetaOffsetDiv2;    } //!< get beta offset for Cr deblocking filter
  void                   setDeblockingFilterCrTcOffsetDiv2(int val)                       { m_deblockingFilterCrTcOffsetDiv2 = val;       } //!< set tc offset for Cr deblocking filter
  int                    getDeblockingFilterCrTcOffsetDiv2() const                        { return m_deblockingFilterCrTcOffsetDiv2;      } //!< get tc offset for Cr deblocking filter
  bool                   getListsModificationPresentFlag() const                          { return m_listsModificationPresentFlag;        }
  void                   setListsModificationPresentFlag( bool b )                        { m_listsModificationPresentFlag = b;           }
  bool                   getPictureHeaderExtensionPresentFlag() const                     { return m_pictureHeaderExtensionPresentFlag;     }
  void                   setPictureHeaderExtensionPresentFlag(bool val)                   { m_pictureHeaderExtensionPresentFlag = val;      }
  bool                   getSliceHeaderExtensionPresentFlag() const                       { return m_sliceHeaderExtensionPresentFlag;     }
  void                   setSliceHeaderExtensionPresentFlag(bool val)                     { m_sliceHeaderExtensionPresentFlag = val;      }

  void                   setRplInfoInPhFlag(bool flag)                                    { m_rplInfoInPhFlag = flag;                     }
  bool                   getRplInfoInPhFlag() const                                       { return m_rplInfoInPhFlag;                     }
  void                   setDbfInfoInPhFlag(bool flag)                                    { m_dbfInfoInPhFlag = flag;                     }
  bool                   getDbfInfoInPhFlag() const                                       { return m_dbfInfoInPhFlag;                     }
  void                   setSaoInfoInPhFlag(bool flag)                                    { m_saoInfoInPhFlag = flag;                     }
  bool                   getSaoInfoInPhFlag() const                                       { return m_saoInfoInPhFlag;                     }
  void                   setAlfInfoInPhFlag(bool flag)                                    { m_alfInfoInPhFlag = flag;                     }
  bool                   getAlfInfoInPhFlag() const                                       { return m_alfInfoInPhFlag;                     }
  void                   setWpInfoInPhFlag(bool flag)                                     { m_wpInfoInPhFlag = flag;                      }
  bool                   getWpInfoInPhFlag() const                                        { return m_wpInfoInPhFlag;                      }
  void                   setQpDeltaInfoInPhFlag(bool flag)                                { m_qpDeltaInfoInPhFlag = flag;                 }
  bool                   getQpDeltaInfoInPhFlag() const                                   { return m_qpDeltaInfoInPhFlag; }


  void                    setPicWidthInLumaSamples( uint32_t u )                          { m_picWidthInLumaSamples = u; }
  uint32_t                getPicWidthInLumaSamples() const                                { return  m_picWidthInLumaSamples; }
  void                    setPicHeightInLumaSamples( uint32_t u )                         { m_picHeightInLumaSamples = u; }
  uint32_t                getPicHeightInLumaSamples() const                               { return  m_picHeightInLumaSamples; }

  void                    setConformanceWindowFlag(bool flag)                             { m_conformanceWindowFlag = flag; }
  bool                    getConformanceWindowFlag() const                                { return m_conformanceWindowFlag; }
  Window&                 getConformanceWindow()                                          { return  m_conformanceWindow; }
  const Window&           getConformanceWindow() const                                    { return  m_conformanceWindow; }
  void                    setConformanceWindow( Window& conformanceWindow )               { m_conformanceWindow = conformanceWindow; }

  Window&                 getScalingWindow()                                              { return  m_scalingWindow; }
  const Window&           getScalingWindow()                                        const { return  m_scalingWindow; }
  void                    setScalingWindow( Window& scalingWindow )                       { m_scalingWindow = scalingWindow; }

  int                     getMixedNaluTypesInPicFlag() const                              { return m_mixedNaluTypesInPicFlag; }
  void                    setMixedNaluTypesInPicFlag( const bool flag )                   { m_mixedNaluTypesInPicFlag = flag; }
};

class APS
{
private:
  int                    m_APSId;                    // adaptation_parameter_set_id
  int                    m_temporalId;
  int                    m_puCounter;
  int                    m_layerId;
  ApsType                m_APSType;                  // aps_params_type
  AlfParam               m_alfAPSParam;
  SliceReshapeInfo       m_reshapeAPSInfo;
  ScalingList            m_scalingListApsInfo;
  CcAlfFilterParam       m_ccAlfAPSParam;
  bool                   m_hasPrefixNalUnitType;

public:
  APS();
  virtual                ~APS();

  int                    getAPSId() const                                                 { return m_APSId;                               }
  void                   setAPSId(int i)                                                  { m_APSId = i;                                  }

  ApsType                getAPSType() const                                               { return m_APSType;                             }
  void                   setAPSType( ApsType type )                                       { m_APSType = type;                             }

  void                   setAlfAPSParam(AlfParam& alfAPSParam)                            { m_alfAPSParam = alfAPSParam;                  }
  void                   setTemporalId( int i )                                           { m_temporalId = i;                             }
  int                    getTemporalId()                                            const { return m_temporalId;                          }
  void                   setPuCounter(int i)                                              { m_puCounter = i;                              }
  int                    getPuCounter()                                             const { return m_puCounter;                           }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }
  AlfParam&              getAlfAPSParam()  { return m_alfAPSParam; }

  void                   setReshaperAPSInfo(SliceReshapeInfo& reshapeAPSInfo)             { m_reshapeAPSInfo = reshapeAPSInfo;            }
  SliceReshapeInfo&      getReshaperAPSInfo()                                             { return m_reshapeAPSInfo;                      }
  void                   setScalingList( ScalingList& scalingListAPSInfo )                { m_scalingListApsInfo = scalingListAPSInfo;    }
  ScalingList&           getScalingList()                                                 { return m_scalingListApsInfo;                  }
  void                   setCcAlfAPSParam(CcAlfFilterParam& ccAlfAPSParam)                { m_ccAlfAPSParam = ccAlfAPSParam;              }
  CcAlfFilterParam&      getCcAlfAPSParam()  { return m_ccAlfAPSParam; }
  void                   setHasPrefixNalUnitType( bool b )                                { m_hasPrefixNalUnitType = b;                   }
  bool                   getHasPrefixNalUnitType() const                                  { return m_hasPrefixNalUnitType;                }
  bool chromaPresentFlag;
};

struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  bool     presentFlag;
  uint32_t log2WeightDenom;
  int      codedWeight;
  int      codedOffset;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  int  w;
  int  o;
  int  offset;
  int  shift;
  int  round;

  static bool isWeighted(const WPScalingParam *wp);
};

inline bool WPScalingParam::isWeighted(const WPScalingParam *wp)
{
  return wp != nullptr && (wp[COMPONENT_Y].presentFlag || wp[COMPONENT_Cb].presentFlag || wp[COMPONENT_Cr].presentFlag);
}

struct WPACDCParam
{
  int64_t iAC;
  int64_t iDC;
};

// picture header class
class PicHeader
{
private:
  bool                        m_valid;                                                  //!< picture header is valid yet or not
  Picture*                    m_pcPic;                                                  //!< pointer to picture structure
  int                         m_pocLsb;                                                 //!< least significant bits of picture order count
  bool                        m_nonReferencePictureFlag;                                //!< non-reference picture flag
  bool                        m_gdrOrIrapPicFlag;                                       //!< gdr or irap picture flag
  bool                        m_gdrPicFlag;                                             //!< gradual decoding refresh picture flag
  uint32_t                    m_recoveryPocCnt;                                         //!< recovery POC count
  bool                        m_noOutputBeforeRecoveryFlag;                             //!< NoOutputBeforeRecoveryFlag
  bool                        m_handleCraAsCvsStartFlag;                                //!< HandleCraAsCvsStartFlag
  bool                        m_handleGdrAsCvsStartFlag;                                //!< HandleGdrAsCvsStartFlag
  int                         m_spsId;                                                  //!< sequence parameter set ID
  int                         m_ppsId;                                                  //!< picture parameter set ID
  bool                        m_pocMsbPresentFlag;                                      //!< ph_poc_msb_present_flag
  int                         m_pocMsbVal;                                              //!< poc_msb_val
  bool                        m_virtualBoundariesEnabledFlag;                           //!< loop filtering across virtual boundaries disabled
  bool                        m_virtualBoundariesPresentFlag;                           //!< loop filtering across virtual boundaries disabled
  unsigned                    m_numVerVirtualBoundaries;                                //!< number of vertical virtual boundaries
  unsigned                    m_numHorVirtualBoundaries;                                //!< number of horizontal virtual boundaries
  unsigned                    m_virtualBoundariesPosX[3];                               //!< horizontal virtual boundary positions
  unsigned                    m_virtualBoundariesPosY[3];                               //!< vertical virtual boundary positions
  bool                        m_picOutputFlag;                                          //!< picture output flag
  ReferencePictureList        m_RPL0;                                              //!< RPL for L0 when present in picture header
  ReferencePictureList        m_RPL1;                                              //!< RPL for L1 when present in picture header
  int                         m_rpl0Idx;                                                //!< index of used RPL in the SPS or -1 for local RPL in the picture header
  int                         m_rpl1Idx;                                                //!< index of used RPL in the SPS or -1 for local RPL in the picture header
  bool                        m_picInterSliceAllowedFlag;                               //!< inter slice allowed flag in PH
  bool                        m_picIntraSliceAllowedFlag;                               //!< intra slice allowed flag in PH
  bool                        m_splitConsOverrideFlag;                                  //!< partitioning constraint override flag
  uint32_t                    m_cuQpDeltaSubdivIntra;                                   //!< CU QP delta maximum subdivision for intra slices
  uint32_t                    m_cuQpDeltaSubdivInter;                                   //!< CU QP delta maximum subdivision for inter slices
  uint32_t                    m_cuChromaQpOffsetSubdivIntra;                            //!< CU chroma QP offset maximum subdivision for intra slices
  uint32_t                    m_cuChromaQpOffsetSubdivInter;                            //!< CU chroma QP offset maximum subdivision for inter slices
  bool                        m_enableTMVPFlag;                                         //!< enable temporal motion vector prediction
  bool                        m_picColFromL0Flag;                                       //!< syntax element collocated_from_l0_flag
  uint32_t                    m_colRefIdx;
  bool                        m_mvdL1ZeroFlag;                                          //!< L1 MVD set to zero flag
  uint32_t                    m_maxNumAffineMergeCand;                                  //!< max number of sub-block merge candidates
  bool                        m_disFracMMVD;                                            //!< fractional MMVD offsets disabled flag
  bool                        m_bdofDisabledFlag;                                       //!< picture level BDOF disable flag
  bool                        m_dmvrDisabledFlag;                                       //!< picture level DMVR disable flag
  bool                        m_profDisabledFlag;                                       //!< picture level PROF disable flag
  bool                        m_jointCbCrSignFlag;                                      //!< joint Cb/Cr residual sign flag
  int                         m_qpDelta;                                                //!< value of Qp delta
  bool                        m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];                   //!< sao enabled flags for each channel
  bool                        m_alfEnabledFlag[MAX_NUM_COMPONENT];                      //!< alf enabled flags for each component
  int                         m_numAlfAps;                                              //!< number of alf aps active for the picture
  std::vector<int>            m_alfApsId;                                               //!< list of alf aps for the picture
  int                         m_alfChromaApsId;                                         //!< chroma alf aps ID
  bool m_ccalfEnabledFlag[MAX_NUM_COMPONENT];
  int  m_ccalfCbApsId;
  int  m_ccalfCrApsId;
  bool                        m_deblockingFilterOverrideFlag;                           //!< deblocking filter override controls enabled
  bool                        m_deblockingFilterDisable;                                //!< deblocking filter disabled flag
  int                         m_deblockingFilterBetaOffsetDiv2;                         //!< beta offset for deblocking filter
  int                         m_deblockingFilterTcOffsetDiv2;                           //!< tc offset for deblocking filter
  int                         m_deblockingFilterCbBetaOffsetDiv2;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCbTcOffsetDiv2;                         //!< tc offset for deblocking filter
  int                         m_deblockingFilterCrBetaOffsetDiv2;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCrTcOffsetDiv2;                         //!< tc offset for deblocking filter
  bool                        m_lmcsEnabledFlag;                                        //!< lmcs enabled flag
  int                         m_lmcsApsId;                                              //!< lmcs APS ID
  APS*                        m_lmcsAps;                                                //!< lmcs APS
  bool                        m_lmcsChromaResidualScaleFlag;                            //!< lmcs chroma residual scale flag
  bool                        m_explicitScalingListEnabledFlag;                         //!< explicit quantization scaling list enabled
  int                         m_scalingListApsId;                                       //!< quantization scaling list APS ID
  APS*                        m_scalingListAps;                                         //!< quantization scaling list APS
  unsigned                    m_minQT[3];                                               //!< minimum quad-tree size  0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned                    m_maxMTTHierarchyDepth[3];                                //!< maximum MTT depth
  unsigned                    m_maxBTSize[3];                                           //!< maximum BT size
  unsigned                    m_maxTTSize[3];                                           //!< maximum TT size

  WPScalingParam              m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT];   // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  int                         m_numL0Weights;                                           //!< number of weights for L0 list
  int                         m_numL1Weights;                                           //!< number of weights for L1 list

public:
                              PicHeader();
  virtual                     ~PicHeader();
  void                        initPicHeader();
  bool                        isValid()                                                 { return m_valid;                                                                              }
  void                        setValid()                                                { m_valid = true;                                                                              }
  void                        setPic( Picture* p )                                      { m_pcPic = p;                                                                                 }
  Picture*                    getPic()                                                  { return m_pcPic;                                                                              }
  const Picture*              getPic() const                                            { return m_pcPic;                                                                              }
  void                        setPocLsb(int i)                                          { m_pocLsb = i;                                                                                }
  int                         getPocLsb()                                               { return m_pocLsb;                                                                             }
  void                        setNonReferencePictureFlag( bool b )                      { m_nonReferencePictureFlag = b;                                                               }
  bool                        getNonReferencePictureFlag() const                        { return m_nonReferencePictureFlag;                                                            }
  void                        setGdrOrIrapPicFlag( bool b )                             { m_gdrOrIrapPicFlag = b;                                                                      }
  bool                        getGdrOrIrapPicFlag() const                               { return m_gdrOrIrapPicFlag;                                                                   }
  void                        setGdrPicFlag( bool b )                                   { m_gdrPicFlag = b;                                                                            }
  bool                        getGdrPicFlag() const                                     { return m_gdrPicFlag;                                                                         }
  void                        setRecoveryPocCnt( uint32_t u )                           { m_recoveryPocCnt = u;                                                                        }
  uint32_t                    getRecoveryPocCnt() const                                 { return m_recoveryPocCnt;                                                                     }
  void                        setSPSId( uint32_t u )                                    { m_spsId = u;                                                                                 }
  uint32_t                    getSPSId() const                                          { return m_spsId;                                                                              }
  void                        setPPSId( uint32_t u )                                    { m_ppsId = u;                                                                                 }
  uint32_t                    getPPSId() const                                          { return m_ppsId;                                                                              }
  void                        setPocMsbPresentFlag(bool b)                              { m_pocMsbPresentFlag = b;                                                                     }
  bool                        getPocMsbPresentFlag() const                              { return m_pocMsbPresentFlag;                                                                  }
  void                        setPocMsbVal(int i)                                       { m_pocMsbVal = i;                                                                             }
  int                         getPocMsbVal()                                            { return m_pocMsbVal;                                                                          }
  void                        setVirtualBoundariesPresentFlag( bool b )                 { m_virtualBoundariesPresentFlag = b;                                                          }
  bool                        getVirtualBoundariesPresentFlag() const                   { return m_virtualBoundariesPresentFlag;                                                       }
  void                        setNumVerVirtualBoundaries(unsigned u)                    { m_numVerVirtualBoundaries = u;                                                               }
  unsigned                    getNumVerVirtualBoundaries() const                        { return m_numVerVirtualBoundaries;                                                            }
  void                        setNumHorVirtualBoundaries(unsigned u)                    { m_numHorVirtualBoundaries = u;                                                               }
  unsigned                    getNumHorVirtualBoundaries() const                        { return m_numHorVirtualBoundaries;                                                            }
  void                        setVirtualBoundariesPosX(unsigned u, unsigned idx)        {
    CHECK_( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;   }
  unsigned                    getVirtualBoundariesPosX(unsigned idx) const              {
    CHECK_( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx];}
  void                        setVirtualBoundariesPosY(unsigned u, unsigned idx)        {
    CHECK_( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;   }
  unsigned                    getVirtualBoundariesPosY(unsigned idx) const              {
    CHECK_( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx];}
  void                        setPicOutputFlag( bool b )                                { m_picOutputFlag = b;                                                                         }
  bool                        getPicOutputFlag() const                                  { return m_picOutputFlag;                                                                      }
  ReferencePictureList*       getRPL( bool b )                                          { return (b==1) ? getRPL1() : getRPL0();                                                       }
  void                        setRPLIdx( bool b, int rplIdx)                            { if(b==1) { m_rpl1Idx = rplIdx; } else { m_rpl0Idx = rplIdx; }                                }
  int                         getRPLIdx( bool b ) const                                 { return b==1 ? m_rpl1Idx : m_rpl0Idx;                                                         }
  ReferencePictureList*       getRPL0()                                                 { return &m_RPL0;                                                                              }
  ReferencePictureList*       getRPL1()                                                 { return &m_RPL1;                                                                              }
  void                        setRPL0idx(int rplIdx)                                    { m_rpl0Idx = rplIdx;                                                                          }
  void                        setRPL1idx(int rplIdx)                                    { m_rpl1Idx = rplIdx;                                                                          }
  int                         getRPL0idx() const                                        { return m_rpl0Idx;                                                                            }
  int                         getRPL1idx() const                                        { return m_rpl1Idx;                                                                            }
  void                        setPicInterSliceAllowedFlag(bool b)                       { m_picInterSliceAllowedFlag = b; }
  bool                        getPicInterSliceAllowedFlag() const                       { return m_picInterSliceAllowedFlag; }
  void                        setPicIntraSliceAllowedFlag(bool b)                       { m_picIntraSliceAllowedFlag = b; }
  bool                        getPicIntraSliceAllowedFlag() const                       { return m_picIntraSliceAllowedFlag; }
  void                        setSplitConsOverrideFlag( bool b )                        { m_splitConsOverrideFlag = b;                                                                 }
  bool                        getSplitConsOverrideFlag() const                          { return m_splitConsOverrideFlag;                                                              }
  void                        setCuQpDeltaSubdivIntra( uint32_t u )                     { m_cuQpDeltaSubdivIntra = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivIntra() const                           { return m_cuQpDeltaSubdivIntra;                                                               }
  void                        setCuQpDeltaSubdivInter( uint32_t u )                     { m_cuQpDeltaSubdivInter = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivInter() const                           { return m_cuQpDeltaSubdivInter;                                                               }
  void                        setCuChromaQpOffsetSubdivIntra( uint32_t u )              { m_cuChromaQpOffsetSubdivIntra = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivIntra() const                    { return m_cuChromaQpOffsetSubdivIntra;                                                        }
  void                        setCuChromaQpOffsetSubdivInter( uint32_t u )              { m_cuChromaQpOffsetSubdivInter = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivInter() const                    { return m_cuChromaQpOffsetSubdivInter;                                                        }
  void                        setEnableTMVPFlag( bool b )                               { m_enableTMVPFlag = b;                                                                        }
  bool                        getEnableTMVPFlag() const                                 { return m_enableTMVPFlag;                                                                     }
  void                        setPicColFromL0Flag(bool val)                             { m_picColFromL0Flag = val;                                                                     }
  bool                        getPicColFromL0Flag() const                               { return m_picColFromL0Flag;                                                                    }
  void                        setColRefIdx( uint32_t refIdx)                             { m_colRefIdx = refIdx;                                                                       }
  uint32_t                    getColRefIdx()                                             { return m_colRefIdx;                                                                         }
  void                        setMvdL1ZeroFlag( bool b )                                { m_mvdL1ZeroFlag = b;                                                                         }
  bool                        getMvdL1ZeroFlag() const                                  { return m_mvdL1ZeroFlag;                                                                      }
  void                        setMaxNumAffineMergeCand( uint32_t val )                  { m_maxNumAffineMergeCand = val;                                                               }
  uint32_t                    getMaxNumAffineMergeCand() const                          { return m_maxNumAffineMergeCand;                                                              }
  void                        setDisFracMMVD( bool val )                                { m_disFracMMVD = val;                                                                         }
  bool                        getDisFracMMVD() const                                    { return m_disFracMMVD;                                                                        }
  void                        setBdofDisabledFlag( bool val )                           { m_bdofDisabledFlag = val;                                                                    }
  bool                        getBdofDisabledFlag() const                               { return m_bdofDisabledFlag;                                                                   }
  void                        setDmvrDisabledFlag( bool val )                           { m_dmvrDisabledFlag = val;                                                                    }
  bool                        getDmvrDisabledFlag() const                               { return m_dmvrDisabledFlag;                                                                   }
  void                        setProfDisabledFlag( bool val )                           { m_profDisabledFlag = val;                                                                    }
  bool                        getProfDisabledFlag() const                               { return m_profDisabledFlag;                                                                   }
  void                        setJointCbCrSignFlag( bool b )                            { m_jointCbCrSignFlag = b;                                                                     }
  bool                        getJointCbCrSignFlag() const                              { return m_jointCbCrSignFlag;                                                                  }
  void                        setQpDelta(int b)                                         { m_qpDelta = b;                                                                               }
  int                         getQpDelta() const                                        { return m_qpDelta;                                                                            }
  void                        setSaoEnabledFlag(ChannelType chType, bool b)             { m_saoEnabledFlag[chType] = b;                                                                }
  bool                        getSaoEnabledFlag(ChannelType chType) const               { return m_saoEnabledFlag[chType];                                                             }
  void                        setAlfEnabledFlag(ComponentID compId, bool b)             { m_alfEnabledFlag[compId] = b;                                                                }
  bool                        getAlfEnabledFlag(ComponentID compId) const               { return m_alfEnabledFlag[compId];                                                             }
  void                        setNumAlfAps(int i)                                       { m_numAlfAps = i;                                                                             }
  int                         getNumAlfAps() const                                      { return m_numAlfAps;                                                                          }
  void                        setAlfApsIdChroma(int i)                                  { m_alfChromaApsId = i;                                                                        }
  int                         getAlfApsIdChroma() const                                 { return m_alfChromaApsId;                                                                     }
  void setCcAlfEnabledFlag(ComponentID compId, bool b) { m_ccalfEnabledFlag[compId] = b; }
  bool getCcAlfEnabledFlag(ComponentID compId) const { return m_ccalfEnabledFlag[compId]; }

  void setCcAlfCbApsId(int i) { m_ccalfCbApsId = i; }
  int  getCcAlfCbApsId() const { return m_ccalfCbApsId; }
  void setCcAlfCrApsId(int i) { m_ccalfCrApsId = i; }
  int  getCcAlfCrApsId() const { return m_ccalfCrApsId; }
  void                        setDeblockingFilterOverrideFlag( bool b )                 { m_deblockingFilterOverrideFlag = b;                                                          }
  bool                        getDeblockingFilterOverrideFlag() const                   { return m_deblockingFilterOverrideFlag;                                                       }
  void                        setDeblockingFilterDisable( bool b )                      { m_deblockingFilterDisable= b;                                                                }
  bool                        getDeblockingFilterDisable() const                        { return m_deblockingFilterDisable;                                                            }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )                { m_deblockingFilterBetaOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterBetaOffsetDiv2()const                  { return m_deblockingFilterBetaOffsetDiv2;                                                     }
  void                        setDeblockingFilterTcOffsetDiv2( int i )                  { m_deblockingFilterTcOffsetDiv2 = i;                                                          }
  int                         getDeblockingFilterTcOffsetDiv2() const                   { return m_deblockingFilterTcOffsetDiv2;                                                       }
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )              { m_deblockingFilterCbBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCbBetaOffsetDiv2()const                { return m_deblockingFilterCbBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )                { m_deblockingFilterCbTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCbTcOffsetDiv2() const                 { return m_deblockingFilterCbTcOffsetDiv2;                                                     }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )              { m_deblockingFilterCrBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const                { return m_deblockingFilterCrBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )                { m_deblockingFilterCrTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCrTcOffsetDiv2() const                 { return m_deblockingFilterCrTcOffsetDiv2;                                                     }
  void                        setLmcsEnabledFlag(bool b)                                { m_lmcsEnabledFlag = b;                                                                       }
  bool                        getLmcsEnabledFlag()                                      { return m_lmcsEnabledFlag;                                                                    }
  const bool                  getLmcsEnabledFlag() const                                { return m_lmcsEnabledFlag;                                                                    }
  void                        setLmcsAPS(APS* aps)                                      { m_lmcsAps = aps; m_lmcsApsId = (aps) ? aps->getAPSId() : -1;                                 }
  APS*                        getLmcsAPS() const                                        { return m_lmcsAps;                                                                            }
  void                        setLmcsAPSId(int id)                                      { m_lmcsApsId = id;                                                                            }
  int                         getLmcsAPSId() const                                      { return m_lmcsApsId;                                                                          }
  void                        setLmcsChromaResidualScaleFlag(bool b)                    { m_lmcsChromaResidualScaleFlag = b;                                                           }
  bool                        getLmcsChromaResidualScaleFlag()                          { return m_lmcsChromaResidualScaleFlag;                                                        }
  const bool                  getLmcsChromaResidualScaleFlag() const                    { return m_lmcsChromaResidualScaleFlag;                                                        }
  void                        setScalingListAPS( APS* aps )                             { m_scalingListAps = aps; m_scalingListApsId = ( aps ) ? aps->getAPSId() : -1;                 }
  APS*                        getScalingListAPS() const                                 { return m_scalingListAps;                                                                     }
  void                        setScalingListAPSId( int id )                             { m_scalingListApsId = id;                                                                     }
  int                         getScalingListAPSId() const                               { return m_scalingListApsId;                                                                   }
  void                        setExplicitScalingListEnabledFlag( bool b )               { m_explicitScalingListEnabledFlag = b;                                                        }
  bool                        getExplicitScalingListEnabledFlag()                       { return m_explicitScalingListEnabledFlag;                                                     }
  const bool                  getExplicitScalingListEnabledFlag() const                 { return m_explicitScalingListEnabledFlag;                                                     }

  unsigned*                   getMinQTSizes() const                                     { return (unsigned *)m_minQT;                                                                  }
  unsigned*                   getMaxMTTHierarchyDepths() const                          { return (unsigned *)m_maxMTTHierarchyDepth;                                                   }
  unsigned*                   getMaxBTSizes() const                                     { return (unsigned *)m_maxBTSize;                                                              }
  unsigned*                   getMaxTTSizes() const                                     { return (unsigned *)m_maxTTSize;                                                              }

  void                        setMinQTSize(unsigned idx, unsigned minQT)                { m_minQT[idx] = minQT;                                                                        }
  void                        setMaxMTTHierarchyDepth(unsigned idx, unsigned maxMTT)    { m_maxMTTHierarchyDepth[idx] = maxMTT;                                                        }
  void                        setMaxBTSize(unsigned idx, unsigned maxBT)                { m_maxBTSize[idx] = maxBT;                                                                    }
  void                        setMaxTTSize(unsigned idx, unsigned maxTT)                { m_maxTTSize[idx] = maxTT;                                                                    }

  void                        setMinQTSizes(unsigned*   minQT)                          { m_minQT[0] = minQT[0]; m_minQT[1] = minQT[1]; m_minQT[2] = minQT[2];                                                 }
  void                        setMaxMTTHierarchyDepths(unsigned*   maxMTT)              { m_maxMTTHierarchyDepth[0] = maxMTT[0]; m_maxMTTHierarchyDepth[1] = maxMTT[1]; m_maxMTTHierarchyDepth[2] = maxMTT[2]; }
  void                        setMaxBTSizes(unsigned*   maxBT)                          { m_maxBTSize[0] = maxBT[0]; m_maxBTSize[1] = maxBT[1]; m_maxBTSize[2] = maxBT[2];                                     }
  void                        setMaxTTSizes(unsigned*   maxTT)                          { m_maxTTSize[0] = maxTT[0]; m_maxTTSize[1] = maxTT[1]; m_maxTTSize[2] = maxTT[2];                                     }

  unsigned                    getMinQTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_minQT[0] : m_minQT[2]) : m_minQT[1];                                              }
  unsigned                    getMaxMTTHierarchyDepth(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxMTTHierarchyDepth[0] : m_maxMTTHierarchyDepth[2]) : m_maxMTTHierarchyDepth[1]; }
  unsigned                    getMaxBTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxBTSize[0] : m_maxBTSize[2]) : m_maxBTSize[1];                                  }
  unsigned                    getMaxTTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxTTSize[0] : m_maxTTSize[2]) : m_maxTTSize[1];                                  }

  void                        setAlfAPSs(std::vector<int> apsIDs)                       { m_alfApsId.resize(m_numAlfAps);
                                                                                          for (int i = 0; i < m_numAlfAps; i++)
                                                                                          {
                                                                                            m_alfApsId[i] = apsIDs[i];
                                                                                          }
                                                                                        }

  std::vector<int>            getAlfAPSs() const                                        { return m_alfApsId; }

  void                        setWpScaling(WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT);
  }
  const WPScalingParam *      getWpScaling(const RefPicList refPicList, const int refIdx) const;
  WPScalingParam *            getWpScaling(const RefPicList refPicList, const int refIdx);
  WPScalingParam*             getWpScalingAll()                                        { return (WPScalingParam *) m_weightPredTable; }
  void                        resetWpScaling();
  void                        setNumL0Weights(int b)                                   { m_numL0Weights = b;                          }
  int                         getNumL0Weights()                                        { return m_numL0Weights;                       }
  void                        setNumL1Weights(int b)                                   { m_numL1Weights = b;                          }
  int                         getNumL1Weights()                                        { return m_numL1Weights;                       }

  void                        setNoOutputBeforeRecoveryFlag( bool val )                { m_noOutputBeforeRecoveryFlag = val;  }
  bool                        getNoOutputBeforeRecoveryFlag() const                    { return m_noOutputBeforeRecoveryFlag; }
  void                        setHandleCraAsCvsStartFlag( bool val )                   { m_handleCraAsCvsStartFlag = val;     }
  bool                        getHandleCraAsCvsStartFlag() const                       { return m_handleCraAsCvsStartFlag;    }
  void                        setHandleGdrAsCvsStartFlag( bool val )                   { m_handleGdrAsCvsStartFlag = val;     }
  bool                        getHandleGdrAsCvsStartFlag() const                       { return m_handleGdrAsCvsStartFlag;    }
};

/// slice header class
class Slice
{

private:
  //  Bitstream writing
  bool                       m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];
  int                        m_iPOC;
  int                        m_iLastIDR;
  int                        m_prevGDRInSameLayerPOC;  //< the previous GDR in the same layer
  int                        m_iAssociatedIRAP;
  NalUnitType                m_iAssociatedIRAPType;
  int                        m_prevGDRSubpicPOC;
  int                        m_prevIRAPSubpicPOC;
  NalUnitType                m_prevIRAPSubpicType;
  bool                       m_enableDRAPSEI;
  bool                       m_useLTforDRAP;
  bool                       m_isDRAP;
  int                        m_latestDRAPPOC;
  ReferencePictureList        m_RPL0;            //< RPL for L0 when present in slice header
  ReferencePictureList        m_RPL1;            //< RPL for L1 when present in slice header
  int                         m_rpl0Idx;              //< index of used RPL in the SPS or -1 for local RPL in the slice header
  int                         m_rpl1Idx;              //< index of used RPL in the SPS or -1 for local RPL in the slice header
  NalUnitType                m_eNalUnitType;         ///< Nal unit type for the slice
  bool                       m_pictureHeaderInSliceHeader;
  uint32_t                   m_nuhLayerId;           ///< Nal unit layer id
  SliceType                  m_eSliceType;
  bool                       m_noOutputOfPriorPicsFlag;           //!< no output of prior pictures flag
  int                        m_iSliceQp;
  int                        m_iSliceQpBase;
  bool                       m_ChromaQpAdjEnabled;
  bool                       m_lmcsEnabledFlag;
  bool                       m_explicitScalingListUsed;
  bool                       m_deblockingFilterDisable;
  bool                       m_deblockingFilterOverrideFlag;      //< offsets for deblocking filter inherit from PPS
  int                        m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  int                        m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  int                        m_deblockingFilterCbBetaOffsetDiv2;  //< beta offset for deblocking filter
  int                        m_deblockingFilterCbTcOffsetDiv2;    //< tc offset for deblocking filter
  int                        m_deblockingFilterCrBetaOffsetDiv2;  //< beta offset for deblocking filter
  int                        m_deblockingFilterCrTcOffsetDiv2;    //< tc offset for deblocking filter
  bool                       m_depQuantEnabledFlag;               //!< dependent quantization enabled flag
  bool                       m_signDataHidingEnabledFlag;         //!< sign data hiding enabled flag
  bool                       m_tsResidualCodingDisabledFlag;
  int                        m_list1IdxToList0Idx[MAX_NUM_REF];
  int                        m_aiNumRefIdx   [NUM_REF_PIC_LIST_01];    //  for multiple reference of current slice
  bool                       m_pendingRasInit;

  bool                       m_bCheckLDC;

  bool                       m_biDirPred;
  int                        m_symRefIdx[2];

  //  Data
  int                        m_iSliceQpDelta;
  int                        m_iSliceChromaQpDelta[MAX_NUM_COMPONENT+1];
  Picture*                   m_apcRefPicList [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                        m_aiRefPOCList  [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  bool                       m_bIsUsedAsLongTerm[NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                        m_iDepth;
  Picture*                   m_scaledRefPicList[NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
  Picture*                   m_savedRefPicList[NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
  std::pair<int, int>        m_scalingRatio[NUM_REF_PIC_LIST_01][MAX_NUM_REF_PICS];


  // access channel
  const VPS*                 m_pcVPS;
  const SPS*                 m_pcSPS;
  const PPS*                 m_pcPPS;
  Picture*                   m_pcPic;
  const PicHeader*           m_pcPicHeader;    //!< pointer to picture header structure
  bool                       m_colFromL0Flag;  // collocated picture from List0 flag


  uint32_t                   m_colRefIdx;
  double                     m_lambdas[MAX_NUM_COMPONENT];

  bool                       m_abEqualRef  [NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_REF];
  uint32_t                   m_uiTLayer;
  bool                       m_bTLayerSwitchingFlag;

  SliceMap                   m_sliceMap;                     //!< list of CTUs in current slice - raster scan CTU addresses
  uint32_t                   m_independentSliceIdx;
  bool                       m_nextSlice;
  uint32_t                   m_sliceBits;
  bool                       m_bFinalized;


  bool                       m_bTestWeightPred;
  bool                       m_bTestWeightBiPred;
  WPScalingParam             m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT]; // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  WPACDCParam                m_weightACDCParam[MAX_NUM_COMPONENT];
  ClpRngs                    m_clpRngs;
  std::vector<uint32_t>      m_substreamSizes;
  uint32_t                   m_numEntryPoints;
  uint32_t                   m_numSubstream;

  bool                       m_cabacInitFlag;

  uint32_t                   m_sliceSubPicId;


  SliceType                  m_encCABACTableIdx;           // Used to transmit table selection across slices.

  clock_t                    m_iProcessingStartTime;
  double                     m_dProcessingTime;

  int                        m_rpPicOrderCntVal;
  APS*                       m_alfApss[ALF_CTB_MAX_NUM_APS];
  bool                       m_tileGroupAlfEnabledFlag[MAX_NUM_COMPONENT];
  int                        m_tileGroupNumAps;
  std::vector<int>           m_tileGroupLumaApsId;
  int                        m_tileGroupChromaApsId;
  bool                       m_tileGroupCcAlfCbEnabledFlag;
  bool                       m_tileGroupCcAlfCrEnabledFlag;
  int                        m_tileGroupCcAlfCbApsId;
  int                        m_tileGroupCcAlfCrApsId;
  bool                       m_disableSATDForRd;
  bool                       m_isLossless;
public:
                              Slice();
  virtual                     ~Slice();
  void                        initSlice();
  void                        inheritFromPicHeader( PicHeader *picHeader, const PPS *pps, const SPS *sps );
  void                        setPicHeader( const PicHeader* pcPicHeader )           { m_pcPicHeader = pcPicHeader;                                  }
  const PicHeader*            getPicHeader() const                                   { return m_pcPicHeader;                                         }
  int                         getRefIdx4MVPair( RefPicList eCurRefPicList, int nCurRefIdx );


  void                        setSPS( const SPS* pcSPS )                             { m_pcSPS = pcSPS;                                              }
  const SPS*                  getSPS() const                                         { return m_pcSPS;                                               }
  void                        setVPS( const VPS* pcVPS )                             { m_pcVPS = pcVPS;                                              }
  const VPS*                  getVPS() const                                         { return m_pcVPS;                                               }

  void                        setPPS( const PPS* pcPPS )                             { m_pcPPS = pcPPS;                                              }
  const PPS*                  getPPS() const                                         { return m_pcPPS;                                               }

  void                        setAlfAPSs(APS** apss)                                 { memcpy(m_alfApss, apss, sizeof(m_alfApss));                   }
  APS**                       getAlfAPSs()                                           { return m_alfApss;                                             }
  void                        setSaoEnabledFlag(ChannelType chType, bool s)          {m_saoEnabledFlag[chType] =s;                                   }
  bool                        getSaoEnabledFlag(ChannelType chType) const            { return m_saoEnabledFlag[chType];                              }
  ReferencePictureList*       getRPL0()                                              { return &m_RPL0;                                              }
  ReferencePictureList*       getRPL1()                                              { return &m_RPL1;                                              }
  void                        setRPL0idx(int rplIdx)                                 { m_rpl0Idx = rplIdx;                                          }
  void                        setRPL1idx(int rplIdx)                                 { m_rpl1Idx = rplIdx;                                          }
  int                         getRPL0idx() const                                     { return m_rpl0Idx;                                            }
  int                         getRPL1idx() const                                     { return m_rpl1Idx;                                            }
  void                        setLastIDR(int iIDRPOC)                                { m_iLastIDR = iIDRPOC;                                         }
  int                         getLastIDR() const                                     { return m_iLastIDR;                                            }
  void                        setPrevGDRInSameLayerPOC(int prevGDRInSameLayerPOC)    { m_prevGDRInSameLayerPOC = prevGDRInSameLayerPOC;              }
  int                         getPrevGDRInSameLayerPOC() const                       { return m_prevGDRInSameLayerPOC;                               }
  void                        setAssociatedIRAPPOC(int iAssociatedIRAPPOC)           { m_iAssociatedIRAP = iAssociatedIRAPPOC;                       }
  int                         getAssociatedIRAPPOC() const                           { return m_iAssociatedIRAP;                                     }
  void                        setAssociatedIRAPType(NalUnitType associatedIRAPType)  { m_iAssociatedIRAPType = associatedIRAPType;                   }
  NalUnitType                 getAssociatedIRAPType() const                          { return m_iAssociatedIRAPType;                                 }
  void                        setPrevGDRSubpicPOC(int poc)                           { m_prevGDRSubpicPOC = poc;                                     }
  int                         getPrevGDRSubpicPOC() const                            { return m_prevGDRSubpicPOC;                                    }
  void                        setPrevIRAPSubpicPOC(int poc)                          { m_prevIRAPSubpicPOC = poc;                                    }
  int                         getPrevIRAPSubpicPOC() const                           { return m_prevIRAPSubpicPOC;                                   }
  void                        setPrevIRAPSubpicType(NalUnitType type)                { m_prevIRAPSubpicType = type;                                  }
  NalUnitType                 getPrevIRAPSubpicType() const                          { return m_prevIRAPSubpicType;                                  }
  void                        checkSubpicTypeConstraints(PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int prevIRAPSubpicDecOrderNo);
  SliceType                   getSliceType() const                                   { return m_eSliceType;                                          }
  void                        setNoOutputOfPriorPicsFlag(bool b)                     { m_noOutputOfPriorPicsFlag = b;                                }
  bool                        getNoOutputOfPriorPicsFlag() const                     { return m_noOutputOfPriorPicsFlag;                             }
  int                         getPOC() const                                         { return m_iPOC;                                                }
  int                         getSliceQp() const                                     { return m_iSliceQp;                                            }
  bool                        getUseWeightedPrediction() const                       { return( (m_eSliceType==P_SLICE && testWeightPred()) || (m_eSliceType==B_SLICE && testWeightBiPred()) ); }
  int                         getSliceQpDelta() const                                { return m_iSliceQpDelta;                                       }
  int                         getSliceChromaQpDelta(ComponentID compID) const        { return isLuma(compID) ? 0 : m_iSliceChromaQpDelta[compID];    }
  bool                        getUseChromaQpAdj() const                              { return m_ChromaQpAdjEnabled;                                  }
  bool                        getDeblockingFilterDisable() const                     { return m_deblockingFilterDisable;                             }
  bool                        getDeblockingFilterOverrideFlag() const                { return m_deblockingFilterOverrideFlag;                        }
  int                         getDeblockingFilterBetaOffsetDiv2()const               { return m_deblockingFilterBetaOffsetDiv2;                      }
  int                         getDeblockingFilterTcOffsetDiv2() const                { return m_deblockingFilterTcOffsetDiv2;                        }
  int                         getDeblockingFilterCbBetaOffsetDiv2()const             { return m_deblockingFilterCbBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCbTcOffsetDiv2() const              { return m_deblockingFilterCbTcOffsetDiv2;                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const             { return m_deblockingFilterCrBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCrTcOffsetDiv2() const              { return m_deblockingFilterCrTcOffsetDiv2;                      }
  bool                        getPendingRasInit() const                              { return m_pendingRasInit;                                      }
  void                        setPendingRasInit( bool val )                          { m_pendingRasInit = val;                                       }

  void                        setLmcsEnabledFlag(bool b)                              { m_lmcsEnabledFlag = b;                                       }
  bool                        getLmcsEnabledFlag()                                    { return m_lmcsEnabledFlag;                                    }
  const bool                  getLmcsEnabledFlag() const                              { return m_lmcsEnabledFlag;                                    }

  void                        setExplicitScalingListUsed(bool b)                      { m_explicitScalingListUsed = b;                               }
  bool                        getExplicitScalingListUsed()                            { return m_explicitScalingListUsed;                            }

  int                         getNumRefIdx( RefPicList e ) const                     { return m_aiNumRefIdx[e];                                      }
  Picture*                    getPic()                                               { return m_pcPic;                                               }
  const Picture*              getPic() const                                         { return m_pcPic;                                               }
        Picture*              getRefPic( RefPicList e, int iRefIdx) const            { return m_apcRefPicList[e][iRefIdx];                           }
  int                         getRefPOC( RefPicList e, int iRefIdx) const            { return m_aiRefPOCList[e][iRefIdx];                            }
  int                         getDepth() const                                       { return m_iDepth;                                              }
  bool                        getColFromL0Flag() const                               { return m_colFromL0Flag;                                       }
  uint32_t                    getColRefIdx() const                                   { return m_colRefIdx;                                           }
  void                        checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic);
  bool                        getIsUsedAsLongTerm(int i, int j) const                { return m_bIsUsedAsLongTerm[i][j];                             }
  void                        setIsUsedAsLongTerm(int i, int j, bool value)          { m_bIsUsedAsLongTerm[i][j] = value;                            }
  bool                        getCheckLDC() const                                    { return m_bCheckLDC;                                           }
  int                         getList1IdxToList0Idx( int list1Idx ) const            { return m_list1IdxToList0Idx[list1Idx];                        }
  void                        setPOC( int i )                                        { m_iPOC              = i;                                      }
  bool                        getPictureHeaderInSliceHeader() const                  { return m_pictureHeaderInSliceHeader;                         }
  void                        setPictureHeaderInSliceHeader( bool e )                { m_pictureHeaderInSliceHeader = e;                            }
  void                        setNalUnitType( NalUnitType e )                        { m_eNalUnitType      = e;                                      }
  NalUnitType                 getNalUnitType() const                                 { return m_eNalUnitType;                                        }
  void                        setNalUnitLayerId( uint32_t i )                        { m_nuhLayerId = i;                                             }
  uint32_t                    getNalUnitLayerId() const                              { return m_nuhLayerId;                                          }
  bool                        getRapPicFlag() const;
  bool                        getIdrPicFlag() const                                  { return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  bool                        isIRAP() const { return (getNalUnitType() >= NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA); }
  // CLVSS PU is either an IRAP PU with NoOutputBeforeRecoveryFlag equal to 1 or a GDR PU with NoOutputBeforeRecoveryFlag equal to 1.
  bool                        isClvssPu() const                                      { return m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_GDR && !m_pcPPS->getMixedNaluTypesInPicFlag() && m_pcPicHeader->getNoOutputBeforeRecoveryFlag(); }
  bool                        isIDRorBLA() const { return (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP); }
  void                        checkCRA(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int pocCRA, PicList& rcListPic);
  void                        checkSTSA(PicList& rcListPic);
  void                        checkRPL(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int associatedIRAPDecodingOrderNumber, PicList& rcListPic);
  void                        decodingRefreshMarking(int& pocCRA, bool& bRefreshPending, PicList& rcListPic, const bool bEfficientFieldIRAPEnabled);
  void                        setSliceType( SliceType e )                            { m_eSliceType        = e;                                      }
  void                        setSliceQp( int i )                                    { m_iSliceQp          = i;                                      }
  void                        setSliceQpDelta( int i )                               { m_iSliceQpDelta     = i;                                      }
  void                        setSliceChromaQpDelta( ComponentID compID, int i )     { m_iSliceChromaQpDelta[compID] = isLuma(compID) ? 0 : i;       }
  void                        setUseChromaQpAdj( bool b )                            { m_ChromaQpAdjEnabled = b;                                     }
  void                        setDeblockingFilterDisable( bool b )                   { m_deblockingFilterDisable= b;                                 }
  void                        setDeblockingFilterOverrideFlag( bool b )              { m_deblockingFilterOverrideFlag = b;                           }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )             { m_deblockingFilterBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterTcOffsetDiv2( int i )               { m_deblockingFilterTcOffsetDiv2 = i;                           }
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )           { m_deblockingFilterCbBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )             { m_deblockingFilterCbTcOffsetDiv2 = i;                           }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )           { m_deblockingFilterCrBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )             { m_deblockingFilterCrTcOffsetDiv2 = i;                           }
  void                        setDepQuantEnabledFlag( bool b )                       { m_depQuantEnabledFlag = b;                                                                   }
  bool                        getDepQuantEnabledFlag() const                         { return m_depQuantEnabledFlag;                                                                }  
  void                        setSignDataHidingEnabledFlag( bool b )                 { m_signDataHidingEnabledFlag = b;                                                             }
  bool                        getSignDataHidingEnabledFlag() const                   { return m_signDataHidingEnabledFlag;                                                          }  
  void                        setTSResidualCodingDisabledFlag(bool b) { m_tsResidualCodingDisabledFlag = b; }
  bool                        getTSResidualCodingDisabledFlag() const { return m_tsResidualCodingDisabledFlag; }

  void                        setNumRefIdx( RefPicList e, int i )                    { m_aiNumRefIdx[e]    = i;                                      }
  void                        setPic( Picture* p )                                   { m_pcPic             = p;                                      }
  void                        setDepth( int iDepth )                                 { m_iDepth            = iDepth;                                 }

  void                        constructRefPicList(PicList& rcListPic);
  void                        setRefPOCList();

  void                        setColFromL0Flag( bool colFromL0 )                     { m_colFromL0Flag = colFromL0;                                  }
  void                        setColRefIdx( uint32_t refIdx)                             { m_colRefIdx = refIdx;                                         }
  void                        setCheckLDC( bool b )                                  { m_bCheckLDC = b;                                              }

  void                        setBiDirPred( bool b, int refIdx0, int refIdx1 ) { m_biDirPred = b; m_symRefIdx[0] = refIdx0; m_symRefIdx[1] = refIdx1; }
  bool                        getBiDirPred() const { return m_biDirPred; }
  int                         getSymRefIdx( int refList ) const { return m_symRefIdx[refList]; }

  bool                        isIntra() const                                        { return m_eSliceType == I_SLICE;                               }
  bool                        isInterB() const                                       { return m_eSliceType == B_SLICE;                               }
  bool                        isInterP() const                                       { return m_eSliceType == P_SLICE;                               }

  bool                        getEnableDRAPSEI () const                              { return m_enableDRAPSEI;                                       }
  void                        setEnableDRAPSEI ( bool b )                            { m_enableDRAPSEI = b;                                          }
  bool                        getUseLTforDRAP () const                               { return m_useLTforDRAP;                                        }
  void                        setUseLTforDRAP ( bool b )                             { m_useLTforDRAP = b;                                           }
  bool                        isDRAP () const                                        { return m_isDRAP;                                              }
  void                        setDRAP ( bool b )                                     { m_isDRAP = b;                                                 }
  void                        setLatestDRAPPOC ( int i )                             { m_latestDRAPPOC = i;                                          }
  int                         getLatestDRAPPOC () const                              { return m_latestDRAPPOC;                                       }
  bool                        cvsHasPreviousDRAP() const                             { return m_latestDRAPPOC != MAX_INT;                            }
  bool                        isPocRestrictedByDRAP( int poc, bool precedingDRAPinDecodingOrder );
  bool                        isPOCInRefPicList( const ReferencePictureList *rpl, int poc );
  void                        checkConformanceForDRAP( uint32_t temporalId );

  void                        setLambdas( const double lambdas[MAX_NUM_COMPONENT] )  { for (int component = 0; component < MAX_NUM_COMPONENT; component++) m_lambdas[component] = lambdas[component]; }
  const double*               getLambdas() const                                     { return m_lambdas;                                             }

  void                        setSliceSubPicId(int i)                               { m_sliceSubPicId = i;   }
  uint32_t                    getSliceSubPicId() const                              { return m_sliceSubPicId; }
  uint32_t                    getCuQpDeltaSubdiv() const                             { return this->isIntra() ? m_pcPicHeader->getCuQpDeltaSubdivIntra() : m_pcPicHeader->getCuQpDeltaSubdivInter(); }
  uint32_t                    getCuChromaQpOffsetSubdiv() const                      { return this->isIntra() ? m_pcPicHeader->getCuChromaQpOffsetSubdivIntra() : m_pcPicHeader->getCuChromaQpOffsetSubdivInter(); }
  void                        initEqualRef();
  bool                        isEqualRef( RefPicList e, int iRefIdx1, int iRefIdx2 )
  {
    CHECK_(e>=NUM_REF_PIC_LIST_01, "Invalid reference picture list");
    if (iRefIdx1 < 0 || iRefIdx2 < 0)
    {
      return false;
    }
    else
    {
      return m_abEqualRef[e][iRefIdx1][iRefIdx2];
    }
  }

  void                        setEqualRef( RefPicList e, int iRefIdx1, int iRefIdx2, bool b)
  {
    CHECK_( e >= NUM_REF_PIC_LIST_01, "Invalid reference picture list" );
    m_abEqualRef[e][iRefIdx1][iRefIdx2] = m_abEqualRef[e][iRefIdx2][iRefIdx1] = b;
  }

  static void                 sortPicList( PicList& rcListPic );
  void                        setList1IdxToList0Idx();

  uint32_t                    getTLayer() const                                      { return m_uiTLayer;                                            }
  void                        setTLayer( uint32_t uiTLayer )                             { m_uiTLayer = uiTLayer;                                        }

  void                        checkLeadingPictureRestrictions( PicList& rcListPic, const PPS& pps)                                         const;
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList* pRPL, int rplIdx, bool printErrors, int* refPicIndex, int numActiveRefPics) const;

  void                        applyReferencePictureListBasedMarking( PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, const int layerId, const PPS& pps )  const;
  bool                        isTemporalLayerSwitchingPoint( PicList& rcListPic )                                           const;
  bool                        isStepwiseTemporalLayerSwitchingPointCandidate( PicList& rcListPic )                          const;
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx, bool printErrors)                const;

  void                        setNumTilesInSlice( uint32_t u )                       { m_sliceMap.setNumTilesInSlice( u );                                       }
  uint32_t                    getNumTilesInSlice() const                             { return m_sliceMap.getNumTilesInSlice();                                   }
  void                        setSliceMap( SliceMap map )                            { m_sliceMap = map;                                                         }
  uint32_t                    getFirstCtuRsAddrInSlice() const                       { return m_sliceMap.getCtuAddrInSlice(0);                                   }
  void                        setSliceID( uint32_t u )                               { m_sliceMap.setSliceID( u );                                               }
  uint32_t                    getSliceID() const                                     { return m_sliceMap.getSliceID();                                           }
  uint32_t                    getNumCtuInSlice() const                               { return m_sliceMap.getNumCtuInSlice();                                     }
  uint32_t                    getCtuAddrInSlice( int idx ) const                     { return m_sliceMap.getCtuAddrInSlice( idx );                               }
  void                        initSliceMap()                                         { m_sliceMap.initSliceMap();                                                }
  void                        addCtusToSlice( uint32_t startX, uint32_t stopX,
                                              uint32_t startY, uint32_t stopY,
                                              uint32_t picWidthInCtbsY )             { m_sliceMap.addCtusToSlice(startX, stopX, startY, stopY, picWidthInCtbsY); }
  void                        setIndependentSliceIdx( uint32_t i)                        { m_independentSliceIdx = i;                                    }
  uint32_t                        getIndependentSliceIdx() const                         { return  m_independentSliceIdx;                                }
  void                        copySliceInfo(Slice *pcSliceSrc, bool cpyAlmostAll = true);
  void                        setSliceBits( uint32_t uiVal )                             { m_sliceBits = uiVal;                                          }
  uint32_t                        getSliceBits() const                                   { return m_sliceBits;                                           }
  void                        setFinalized( bool uiVal )                             { m_bFinalized = uiVal;                                         }
  bool                        getFinalized() const                                   { return m_bFinalized;                                          }
  bool                        testWeightPred( ) const                                { return m_bTestWeightPred;                                     }
  void                        setTestWeightPred( bool bValue )                       { m_bTestWeightPred = bValue;                                   }
  bool                        testWeightBiPred( ) const                              { return m_bTestWeightBiPred;                                   }
  void                        setTestWeightBiPred( bool bValue )                     { m_bTestWeightBiPred = bValue;                                 }
  void                        setWpScaling( WPScalingParam  wp[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT] )
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam)*NUM_REF_PIC_LIST_01*MAX_NUM_REF*MAX_NUM_COMPONENT);
  }
  void                        setWpScaling(WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT);
  }
  WPScalingParam *            getWpScalingAll()                                      { return (WPScalingParam *) m_weightPredTable;                  }
  WPScalingParam *            getWpScaling(const RefPicList refPicList, const int refIdx);
  const WPScalingParam *      getWpScaling(const RefPicList refPicList, const int refIdx) const;

  void                        resetWpScaling();
  void                        initWpScaling(const SPS *sps);

  void                        setWpAcDcParam( WPACDCParam wp[MAX_NUM_COMPONENT] )    { memcpy(m_weightACDCParam, wp, sizeof(WPACDCParam)*MAX_NUM_COMPONENT); }

  void                        getWpAcDcParam( const WPACDCParam *&wp ) const;
  void                        initWpAcDcParam();

  void                        clearSubstreamSizes( )                                 { return m_substreamSizes.clear();                              }
  uint32_t                        getNumberOfSubstreamSizes( )                           { return (uint32_t) m_substreamSizes.size();                        }
  void                        addSubstreamSize( uint32_t size )                          { m_substreamSizes.push_back(size);                             }
  uint32_t                        getSubstreamSize( uint32_t idx )                           {
    CHECK_(idx>=getNumberOfSubstreamSizes(),"Invalid index"); return m_substreamSizes[idx]; }
  void                        resetNumberOfSubstream()                               { m_numSubstream = 0;                                           }
  uint32_t                    getNumberOfSubstream()                                 { return (uint32_t) m_numSubstream;                             }
  void                        increaseNumberOfSubstream()                            { m_numSubstream++;                                             }

  void                        setCabacInitFlag( bool val )                           { m_cabacInitFlag = val;                                        } //!< set CABAC initial flag
  bool                        getCabacInitFlag()                               const { return m_cabacInitFlag;                                       } //!< get CABAC initial flag

  void                        setEncCABACTableIdx( SliceType idx )                   { m_encCABACTableIdx = idx;                                     }
  SliceType                   getEncCABACTableIdx() const                            { return m_encCABACTableIdx;                                    }


  void                        setSliceQpBase( int i )                                { m_iSliceQpBase = i;                                           }
  int                         getSliceQpBase()                                 const { return m_iSliceQpBase;                                        }

  void                        setDefaultClpRng( const SPS& sps );
  const ClpRngs&              clpRngs()                                         const { return m_clpRngs;}
  const ClpRng&               clpRng( ComponentID id)                           const { return m_clpRngs.comp[id];}
  ClpRngs&                    getClpRngs()                                            { return m_clpRngs;}
  unsigned                    getMinPictureDistance()                           const ;
  void startProcessingTimer();
  void stopProcessingTimer();
  void resetProcessingTime()       { m_dProcessingTime = m_iProcessingStartTime = 0; }
  double getProcessingTime() const { return m_dProcessingTime; }

  void                        resetTileGroupAlfEnabledFlag() { memset(m_tileGroupAlfEnabledFlag, 0, sizeof(m_tileGroupAlfEnabledFlag)); }
  bool                        getTileGroupAlfEnabledFlag(ComponentID compId) const { return m_tileGroupAlfEnabledFlag[compId]; }
  void                        setTileGroupAlfEnabledFlag(ComponentID compId, bool b) { m_tileGroupAlfEnabledFlag[compId] = b; }
  int                         getTileGroupNumAps() const { return m_tileGroupNumAps; }
  void                        setTileGroupNumAps(int i) { m_tileGroupNumAps = i; }
  int                         getTileGroupApsIdChroma() const { return m_tileGroupChromaApsId; }
  void                        setTileGroupApsIdChroma(int i) { m_tileGroupChromaApsId = i; }
  std::vector<int32_t>        getTileGroupApsIdLuma() const { return m_tileGroupLumaApsId; }
  void                        setAlfAPSs(std::vector<int> ApsIDs)
  {
    m_tileGroupLumaApsId.resize(m_tileGroupNumAps);
    for (int i = 0; i < m_tileGroupNumAps; i++)
    {
      m_tileGroupLumaApsId[i] = ApsIDs[i];
    }
  }
  void resetTileGroupCcAlCbfEnabledFlag() { m_tileGroupCcAlfCbEnabledFlag = 0; }
  void resetTileGroupCcAlCrfEnabledFlag() { m_tileGroupCcAlfCrEnabledFlag = 0; }

  void setTileGroupCcAlfCbEnabledFlag(bool b) { m_tileGroupCcAlfCbEnabledFlag = b; }
  void setTileGroupCcAlfCrEnabledFlag(bool b) { m_tileGroupCcAlfCrEnabledFlag = b; }
  void setTileGroupCcAlfCbApsId(int i) { m_tileGroupCcAlfCbApsId = i; }
  void setTileGroupCcAlfCrApsId(int i) { m_tileGroupCcAlfCrApsId = i; }

  bool getTileGroupCcAlfCbEnabledFlag() { return m_tileGroupCcAlfCbEnabledFlag; }
  bool getTileGroupCcAlfCrEnabledFlag() { return m_tileGroupCcAlfCrEnabledFlag; }
  int  getTileGroupCcAlfCbApsId() { return m_tileGroupCcAlfCbApsId; }
  int  getTileGroupCcAlfCrApsId() { return m_tileGroupCcAlfCrApsId; }
  void                        setDisableSATDForRD(bool b) { m_disableSATDForRd = b; }
  bool                        getDisableSATDForRD() { return m_disableSATDForRd; }
  void                        setLossless(bool b) { m_isLossless = b; }
  bool                        isLossless() const { return m_isLossless; }
  void                        scaleRefPicList( Picture *scaledRefPic[ ], PicHeader *picHeader, APS** apss, APS* lmcsAps, APS* scalingListAps, const bool isDecoder );
  void                        freeScaledRefPicList( Picture *scaledRefPic[] );
  bool                        checkRPR();
  const std::pair<int, int>&  getScalingRatio( const RefPicList refPicList, const int refIdx )  const {
    CHECK_( refIdx < 0, "Invalid reference index" ); return m_scalingRatio[refPicList][refIdx]; }
  void                        setNumSubstream( const SPS *sps, const PPS *pps );
  void                        setNumEntryPoints( const SPS *sps, const PPS *pps );
  uint32_t                    getNumEntryPoints( ) const { return m_numEntryPoints;  }
  bool                        isLastSliceInSubpic();

  CcAlfFilterParam            m_ccAlfFilterParam;
  uint8_t*                    m_ccAlfFilterControl[2];

protected:
  Picture*              xGetRefPic( PicList& rcListPic, const int poc, const int layerId );
  Picture*              xGetLongTermRefPic( PicList& rcListPic, const int poc, const bool pocHasMsb, const int layerId );
  Picture*              xGetLongTermRefPicCandidate( PicList& rcListPic, const int poc, const bool pocHasMsb, const int layerId );
public:
  std::unordered_map< Position, std::unordered_map< Size, double> > m_mapPltCost[2];
private:
};// END CLASS DEFINITION Slice


class PreCalcValues
{
public:
  PreCalcValues( const SPS& sps, const PPS& pps, bool _isEncoder )
    : chrFormat           ( sps.getChromaFormatIdc() )
    , multiBlock422       ( false )
    , maxCUWidth          ( sps.getMaxCUWidth() )
    , maxCUHeight         ( sps.getMaxCUHeight() )
    , maxCUWidthMask      ( maxCUWidth  - 1 )
    , maxCUHeightMask     ( maxCUHeight - 1 )
    , maxCUWidthLog2      ( floorLog2( maxCUWidth  ) )
    , maxCUHeightLog2     ( floorLog2( maxCUHeight ) )
    , minCUWidth          ( 1 << MIN_CU_LOG2 )
    , minCUHeight         ( 1 << MIN_CU_LOG2 )
    , minCUWidthLog2      ( floorLog2( minCUWidth  ) )
    , minCUHeightLog2     ( floorLog2( minCUHeight ) )
    , partsInCtuWidth     ( maxCUWidth >> MIN_CU_LOG2)
    , partsInCtuHeight    ( maxCUHeight >> MIN_CU_LOG2)
    , partsInCtu          ( partsInCtuWidth * partsInCtuHeight )
    , widthInCtus         ( (pps.getPicWidthInLumaSamples () + sps.getMaxCUWidth () - 1) / sps.getMaxCUWidth () )
    , heightInCtus        ( (pps.getPicHeightInLumaSamples() + sps.getMaxCUHeight() - 1) / sps.getMaxCUHeight() )
    , sizeInCtus          ( widthInCtus * heightInCtus )
    , lumaWidth           ( pps.getPicWidthInLumaSamples() )
    , lumaHeight          ( pps.getPicHeightInLumaSamples() )
    , fastDeltaQPCuMaxSize( Clip3(1u << sps.getLog2MinCodingBlockSize(), sps.getMaxCUHeight(), 32u) )
    , noChroma2x2         (  false )
    , isEncoder           ( _isEncoder )
    , ISingleTree         ( !sps.getUseDualITree() )
    , maxBtDepth          { sps.getMaxMTTHierarchyDepthI(), sps.getMaxMTTHierarchyDepth(), sps.getMaxMTTHierarchyDepthIChroma() }
    , minBtSize           { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() }
    , maxBtSize           { sps.getMaxBTSizeI(), sps.getMaxBTSize(), sps.getMaxBTSizeIChroma() }
    , minTtSize           { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() }
    , maxTtSize           { sps.getMaxTTSizeI(), sps.getMaxTTSize(), sps.getMaxTTSizeIChroma() }
    , minQtSize           { sps.getMinQTSize(I_SLICE, CHANNEL_TYPE_LUMA), sps.getMinQTSize(B_SLICE, CHANNEL_TYPE_LUMA), sps.getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA) }
  {}

  const ChromaFormat chrFormat;
  const bool         multiBlock422;
  const unsigned     maxCUWidth;
  const unsigned     maxCUHeight;
  // to get CTU position, use (x & maxCUWidthMask) rather than (x % maxCUWidth)
  const unsigned     maxCUWidthMask;
  const unsigned     maxCUHeightMask;
  const unsigned     maxCUWidthLog2;
  const unsigned     maxCUHeightLog2;
  const unsigned     minCUWidth;
  const unsigned     minCUHeight;
  const unsigned     minCUWidthLog2;
  const unsigned     minCUHeightLog2;
  const unsigned     partsInCtuWidth;
  const unsigned     partsInCtuHeight;
  const unsigned     partsInCtu;
  const unsigned     widthInCtus;
  const unsigned     heightInCtus;
  const unsigned     sizeInCtus;
  const unsigned     lumaWidth;
  const unsigned     lumaHeight;
  const unsigned     fastDeltaQPCuMaxSize;
  const bool         noChroma2x2;
  const bool         isEncoder;
  const bool         ISingleTree;

private:
  const unsigned     maxBtDepth[3];
  const unsigned     minBtSize [3];
  const unsigned     maxBtSize [3];
  const unsigned     minTtSize [3];
  const unsigned     maxTtSize [3];
  const unsigned     minQtSize [3];

  unsigned getValIdx    ( const Slice &slice, const ChannelType chType ) const;

public:
  unsigned getMaxBtDepth( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinQtSize ( const Slice &slice, const ChannelType chType ) const;
};

#if ENABLE_TRACING
void xTraceVPSHeader();
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
void xTraceOPIHeader();
#endif
void xTraceDCIHeader();
void xTraceSPSHeader();
void xTracePPSHeader();
void xTraceAPSHeader();
void xTracePictureHeader();
void xTraceSliceHeader();
void xTraceAccessUnitDelimiter();
void xTraceFillerData();
#endif

#endif // __SLICE__

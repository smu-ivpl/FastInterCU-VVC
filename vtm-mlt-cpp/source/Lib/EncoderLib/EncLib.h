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

/** \file     EncLib.h
    \brief    encoder class (header)
*/

#ifndef __ENCTOP__
#define __ENCTOP__

// Include files
#include "CommonLib/TrQuant.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/NAL.h"

#include "Utilities/VideoIOYuv.h"

#include "EncCfg.h"
#include "EncGOP.h"
#include "EncSlice.h"
#include "EncHRD.h"
#include "VLCWriter.h"
#include "CABACWriter.h"
#include "InterSearch.h"
#include "IntraSearch.h"
#include "EncSampleAdaptiveOffset.h"
#include "EncReshape.h"
#include "EncAdaptiveLoopFilter.h"
#include "RateCtrl.h"

class EncLibCommon;

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class
class EncLib : public EncCfg
{
private:
  // picture
  int                       m_iPOCLast;                           ///< time index (POC)
  int                       m_iNumPicRcvd;                        ///< number of received pictures
  uint32_t                  m_uiNumAllPicCoded;                   ///< number of coded pictures
  PicList&                  m_cListPic;                           ///< dynamic list of pictures
  int                       m_layerId;

  // encoder search
#if ENABLE_SPLIT_PARALLELISM
  InterSearch              *m_cInterSearch;                       ///< encoder search class
  IntraSearch              *m_cIntraSearch;                       ///< encoder search class
#else
  InterSearch               m_cInterSearch;                       ///< encoder search class
  IntraSearch               m_cIntraSearch;                       ///< encoder search class
#endif
  // coding tool
#if ENABLE_SPLIT_PARALLELISM
  TrQuant                  *m_cTrQuant;                           ///< transform & quantization class
#else
  TrQuant                   m_cTrQuant;                           ///< transform & quantization class
#endif
  LoopFilter                m_cLoopFilter;                        ///< deblocking filter class
  EncSampleAdaptiveOffset   m_cEncSAO;                            ///< sample adaptive offset class
  EncAdaptiveLoopFilter     m_cEncALF;
  HLSWriter                 m_HLSWriter;                          ///< CAVLC encoder
#if ENABLE_SPLIT_PARALLELISM
  CABACEncoder             *m_CABACEncoder;
#else
  CABACEncoder              m_CABACEncoder;
#endif

#if ENABLE_SPLIT_PARALLELISM
  EncReshape               *m_cReshaper;                        ///< reshaper class
#else
  EncReshape                m_cReshaper;                        ///< reshaper class
#endif

  // processing unit
  EncGOP                    m_cGOPEncoder;                        ///< GOP encoder
  EncSlice                  m_cSliceEncoder;                      ///< slice encoder
#if ENABLE_SPLIT_PARALLELISM
  EncCu                    *m_cCuEncoder;                         ///< CU encoder
#else
  EncCu                     m_cCuEncoder;                         ///< CU encoder
#endif
  // SPS
  ParameterSetMap<SPS>&     m_spsMap;                             ///< SPS. This is the base value. This is copied to PicSym
  ParameterSetMap<PPS>&     m_ppsMap;                             ///< PPS. This is the base value. This is copied to PicSym
  ParameterSetMap<APS>&     m_apsMap;                             ///< APS. This is the base value. This is copied to PicSym
  PicHeader                 m_picHeader;                          ///< picture header
  // RD cost computation
#if ENABLE_SPLIT_PARALLELISM
  RdCost                   *m_cRdCost;                            ///< RD cost computation class
  CtxCache                 *m_CtxCache;                           ///< buffer for temporarily stored context models
#else
  RdCost                    m_cRdCost;                            ///< RD cost computation class
  CtxCache                  m_CtxCache;                           ///< buffer for temporarily stored context models
#endif
  // quality control
  RateCtrl                  m_cRateCtrl;                          ///< Rate control class

  AUWriterIf*               m_AUWriterIf;

#if ENABLE_SPLIT_PARALLELISM
  int                       m_numCuEncStacks;
#endif

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel                m_cacheModel;
#endif

  APS*                      m_apss[ALF_CTB_MAX_NUM_APS];

  APS*                      m_lmcsAPS;
  APS*                      m_scalinglistAPS;

  EncHRD                    m_encHRD;

  bool                      m_doPlt;
#if JVET_O0756_CALCULATE_HDRMETRICS
  std::chrono::duration<long long, ratio<1, 1000000000>> m_metricTime;
#endif
  int                       m_picIdInGOP;

  VPS*                      m_vps;

  int*                      m_layerDecPicBuffering;

public:
  SPS*                      getSPS( int spsId ) { return m_spsMap.getPS( spsId ); };
  APS**                     getApss() { return m_apss; }
  Ctx                       m_entropyCodingSyncContextState;      ///< leave in addition to vector for compatibility
  PLTBuf                    m_palettePredictorSyncState;

protected:
  void  xGetNewPicBuffer  ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, int ppsId ); ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  void  xInitOPI(OPI& opi); ///< initialize Operating point Information (OPI) from encoder options
#endif
  void  xInitDCI(DCI& dci, const SPS& sps); ///< initialize Decoding Capability Information (DCI) from encoder options
  void  xInitVPS( const SPS& sps ); ///< initialize VPS from encoder options
  void  xInitSPS( SPS& sps );       ///< initialize SPS from encoder options
  void  xInitPPS          (PPS &pps, const SPS &sps); ///< initialize PPS from encoder options
  void  xInitPicHeader    (PicHeader &picHeader, const SPS &sps, const PPS &pps); ///< initialize Picture Header from encoder options
  void  xInitAPS          (APS &aps);                 ///< initialize APS from encoder options
  void  xInitScalingLists ( SPS &sps, APS &aps );     ///< initialize scaling lists
  void  xInitPPSforLT(PPS& pps);
  void  xInitHrdParameters(SPS &sps);                 ///< initialize HRDParameters parameters

  void  xInitRPL(SPS &sps, bool isFieldCoding);           ///< initialize SPS from encoder options

public:
  EncLib( EncLibCommon* encLibCommon );
  virtual ~EncLib();

  void      create          ( const int layerId );
  void      destroy         ();
  void      init            ( bool isFieldCoding, AUWriterIf* auWriterIf );
  void      deletePicBuffer ();

  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------

  AUWriterIf*             getAUWriterIf         ()              { return   m_AUWriterIf;           }
  PicList*                getListPic            ()              { return  &m_cListPic;             }
#if ENABLE_SPLIT_PARALLELISM
  InterSearch*            getInterSearch        ( int jId = 0 ) { return  &m_cInterSearch[jId];    }
  IntraSearch*            getIntraSearch        ( int jId = 0 ) { return  &m_cIntraSearch[jId];    }

  TrQuant*                getTrQuant            ( int jId = 0 ) { return  &m_cTrQuant[jId];        }
#else
  InterSearch*            getInterSearch        ()              { return  &m_cInterSearch;         }
  IntraSearch*            getIntraSearch        ()              { return  &m_cIntraSearch;         }

  TrQuant*                getTrQuant            ()              { return  &m_cTrQuant;             }
#endif
  LoopFilter*             getLoopFilter         ()              { return  &m_cLoopFilter;          }
  EncSampleAdaptiveOffset* getSAO               ()              { return  &m_cEncSAO;              }
  EncAdaptiveLoopFilter*  getALF                ()              { return  &m_cEncALF;              }
  EncGOP*                 getGOPEncoder         ()              { return  &m_cGOPEncoder;          }
  EncSlice*               getSliceEncoder       ()              { return  &m_cSliceEncoder;        }
  EncHRD*                 getHRD                ()              { return  &m_encHRD;               }
#if ENABLE_SPLIT_PARALLELISM
  EncCu*                  getCuEncoder          ( int jId = 0 ) { return  &m_cCuEncoder[jId];      }
#else
  EncCu*                  getCuEncoder          ()              { return  &m_cCuEncoder;           }
#endif
  HLSWriter*              getHLSWriter          ()              { return  &m_HLSWriter;            }
#if ENABLE_SPLIT_PARALLELISM
  CABACEncoder*           getCABACEncoder       ( int jId = 0 ) { return  &m_CABACEncoder[jId];    }

  RdCost*                 getRdCost             ( int jId = 0 ) { return  &m_cRdCost[jId];         }
  CtxCache*               getCtxCache           ( int jId = 0 ) { return  &m_CtxCache[jId];        }
#else
  CABACEncoder*           getCABACEncoder       ()              { return  &m_CABACEncoder;         }

  RdCost*                 getRdCost             ()              { return  &m_cRdCost;              }
  CtxCache*               getCtxCache           ()              { return  &m_CtxCache;             }
#endif
  RateCtrl*               getRateCtrl           ()              { return  &m_cRateCtrl;            }


  void                    getActiveRefPicListNumForPOC(const SPS *sps, int POCCurr, int GOPid, uint32_t *activeL0, uint32_t *activeL1);
  void                    selectReferencePictureList(Slice* slice, int POCCurr, int GOPid, int ltPoc);

  void                   setParamSetChanged(int spsId, int ppsId);
  bool                   APSNeedsWriting(int apsId);
  bool                   PPSNeedsWriting(int ppsId);
  bool                   SPSNeedsWriting(int spsId);
  const PPS* getPPS( int Id ) { return m_ppsMap.getPS( Id); }
  const APS*             getAPS(int Id) { return m_apsMap.getPS(Id); }

#if ENABLE_SPLIT_PARALLELISM
  void                   setNumCuEncStacks( int n )             { m_numCuEncStacks = n; }
  int                    getNumCuEncStacks()              const { return m_numCuEncStacks; }
#endif

#if ENABLE_SPLIT_PARALLELISM
  EncReshape*            getReshaper( int jId = 0 )             { return  &m_cReshaper[jId]; }
#else
  EncReshape*            getReshaper()                          { return  &m_cReshaper; }
#endif

  ParameterSetMap<APS>*  getApsMap() { return &m_apsMap; }

  bool                   getPltEnc()                      const { return   m_doPlt; }
  void                   checkPltStats( Picture* pic );
#if JVET_O0756_CALCULATE_HDRMETRICS
  std::chrono::duration<long long, ratio<1, 1000000000>> getMetricTime()    const { return m_metricTime; };
#endif
  // -------------------------------------------------------------------------------------------------------------------
  // encoder function
  // -------------------------------------------------------------------------------------------------------------------

  /// encode several number of pictures until end-of-sequence
  bool encodePrep( bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg,
               PelStorage* pcPicYuvFilteredOrg,
               const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               int& iNumEncoded );

  bool encode( const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               int& iNumEncoded );

  bool encodePrep( bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg,
               PelStorage* pcPicYuvFilteredOrg,
               const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               int& iNumEncoded, bool isTff );

  bool encode( const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               int& iNumEncoded, bool isTff );


  void printSummary(bool isField) { m_cGOPEncoder.printOutSummary(m_uiNumAllPicCoded, isField, m_printMSEBasedSequencePSNR, 
    m_printSequenceMSE, m_printMSSSIM, m_printHexPsnr, m_resChangeInClvsEnabled, m_spsMap.getFirstPS()->getBitDepths()); }

  int getLayerId() const { return m_layerId; }
  VPS* getVPS()          { return m_vps;     }
};

//! \}

#endif // __ENCTOP__


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

/** \file     EncLib.cpp
    \brief    encoder class
*/
#include "EncLib.h"

#include "EncModeCtrl.h"
#include "AQp.h"
#include "EncCu.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/ChromaFormat.h"
#if ENABLE_SPLIT_PARALLELISM
#include <omp.h>
#endif
#include "EncLibCommon.h"
#include "CommonLib/ProfileLevelTier.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncLib::EncLib( EncLibCommon* encLibCommon )
  : m_cListPic( encLibCommon->getPictureBuffer() )
  , m_cEncALF( encLibCommon->getApsIdStart() )
  , m_spsMap( encLibCommon->getSpsMap() )
  , m_ppsMap( encLibCommon->getPpsMap() )
  , m_apsMap( encLibCommon->getApsMap() )
  , m_AUWriterIf( nullptr )
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
  , m_lmcsAPS(nullptr)
  , m_scalinglistAPS( nullptr )
  , m_doPlt( true )
  , m_vps( encLibCommon->getVPS() )
  , m_layerDecPicBuffering( encLibCommon->getDecPicBuffering() )
{
  m_iPOCLast          = -1;
  m_iNumPicRcvd       =  0;
  m_uiNumAllPicCoded  =  0;

  m_iMaxRefPicNum     = 0;

#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif

  memset(m_apss, 0, sizeof(m_apss));

  m_layerId = NOT_VALID;
  m_picIdInGOP = NOT_VALID;
}

EncLib::~EncLib()
{
}

void EncLib::create( const int layerId )
{
  m_layerId = layerId;
  m_iPOCLast = m_compositeRefEnabled ? -2 : -1;
  // create processing unit classes
  m_cGOPEncoder.        create( );
#if ENABLE_SPLIT_PARALLELISM
#if ENABLE_SPLIT_PARALLELISM
  m_numCuEncStacks  = m_numSplitThreads == 1 ? 1 : NUM_RESERVERD_SPLIT_JOBS;
#else
  m_numCuEncStacks  = 1;
#endif

  m_cCuEncoder      = new EncCu              [m_numCuEncStacks];
  m_cInterSearch    = new InterSearch        [m_numCuEncStacks];
  m_cIntraSearch    = new IntraSearch        [m_numCuEncStacks];
  m_cTrQuant        = new TrQuant            [m_numCuEncStacks];
  m_CABACEncoder    = new CABACEncoder       [m_numCuEncStacks];
  m_cRdCost         = new RdCost             [m_numCuEncStacks];
  m_CtxCache        = new CtxCache           [m_numCuEncStacks];

  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cCuEncoder[jId].         create( this );
  }
#else
  m_cCuEncoder.         create( this );
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cInterSearch.cacheAssign( &m_cacheModel );
#endif

  m_cLoopFilter.create(floorLog2(m_maxCUWidth) - MIN_CU_LOG2);

  if (!m_bLoopFilterDisable && m_encDbOpt)
  {
    m_cLoopFilter.initEncPicYuvBuffer(m_chromaFormatIDC, Size(getSourceWidth(), getSourceHeight()), getMaxCUWidth());
  }

#if ENABLE_SPLIT_PARALLELISM
  m_cReshaper = new EncReshape[m_numCuEncStacks];
#endif
  if (m_lmcsEnabled)
  {
#if ENABLE_SPLIT_PARALLELISM
    for (int jId = 0; jId < m_numCuEncStacks; jId++)
    {
      m_cReshaper[jId].createEnc(getSourceWidth(), getSourceHeight(), m_maxCUWidth, m_maxCUHeight, m_bitDepth[COMPONENT_Y]);
    }
#else
    m_cReshaper.createEnc( getSourceWidth(), getSourceHeight(), m_maxCUWidth, m_maxCUHeight, m_bitDepth[COMPONENT_Y]);
#endif
  }
  if ( m_RCEnableRateControl )
  {
    m_cRateCtrl.init(m_framesToBeEncoded, m_RCTargetBitrate, (int)((double)m_iFrameRate / m_temporalSubsampleRatio + 0.5), m_iGOPSize, m_iSourceWidth, m_iSourceHeight,
      m_maxCUWidth, m_maxCUHeight, getBitDepth(CHANNEL_TYPE_LUMA), m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList);
  }

}

void EncLib::destroy ()
{
  // destroy processing unit classes
  m_cGOPEncoder.        destroy();
  m_cSliceEncoder.      destroy();
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cCuEncoder[jId].destroy();
  }
#else
  m_cCuEncoder.         destroy();
#endif
  if( m_alf )
  {
    m_cEncALF.destroy();
  }
  m_cEncSAO.            destroyEncData();
  m_cEncSAO.            destroy();
  m_cLoopFilter.        destroy();
  m_cRateCtrl.          destroy();
#if ENABLE_SPLIT_PARALLELISM
  for (int jId = 0; jId < m_numCuEncStacks; jId++)
  {
    m_cReshaper[jId].   destroy();
  }
#else
  m_cReshaper.          destroy();
#endif
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cInterSearch[jId].   destroy();
    m_cIntraSearch[jId].   destroy();
  }
#else
  m_cInterSearch.       destroy();
  m_cIntraSearch.       destroy();
#endif

#if ENABLE_SPLIT_PARALLELISM
  delete[] m_cCuEncoder;
  delete[] m_cInterSearch;
  delete[] m_cIntraSearch;
  delete[] m_cTrQuant;
  delete[] m_CABACEncoder;
  delete[] m_cRdCost;
  delete[] m_CtxCache;
#endif

  return;
}

void EncLib::init( bool isFieldCoding, AUWriterIf* auWriterIf )
{
  m_AUWriterIf = auWriterIf;

  SPS &sps0 = *(m_spsMap.allocatePS( m_vps->getGeneralLayerIdx( m_layerId ) )); // NOTE: implementations that use more than 1 SPS need to be aware of activation issues.
  PPS &pps0 = *( m_ppsMap.allocatePS( m_vps->getGeneralLayerIdx( m_layerId ) ) );
  APS &aps0 = *( m_apsMap.allocatePS( SCALING_LIST_APS ) );
  aps0.setAPSId( 0 );
  aps0.setAPSType( SCALING_LIST_APS );

  if (getAvoidIntraInDepLayer() && getNumRefLayers(m_vps->getGeneralLayerIdx( getLayerId())) > 0)
  {
    setIDRRefParamListPresent(true);
  }
  // initialize SPS
  xInitSPS( sps0 );

  for( int i = 0; i < MAX_TLAYER; i++ )
  {
    m_layerDecPicBuffering[m_layerId * MAX_TLAYER + i] = m_maxDecPicBuffering[i];
  }

  xInitVPS( sps0 );
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  xInitOPI(m_opi);
#endif
  xInitDCI(m_dci, sps0);
#if ENABLE_SPLIT_PARALLELISM
  if( omp_get_dynamic() )
  {
    omp_set_dynamic( false );
  }
  omp_set_nested( true );
#endif

  if (getUseCompositeRef() || getDependentRAPIndicationSEIEnabled())
  {
    sps0.setLongTermRefsPresent(true);
  }

#if U0132_TARGET_BITS_SATURATION
  if (m_RCCpbSaturationEnabled)
  {
    m_cRateCtrl.initHrdParam(sps0.getGeneralHrdParameters(), sps0.getOlsHrdParameters(), m_iFrameRate, m_RCInitialCpbFullness);
  }
#endif
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cRdCost[jId].setCostMode ( m_costMode );
  }
#else
  m_cRdCost.setCostMode ( m_costMode );
#endif

  // initialize PPS
  pps0.setPicWidthInLumaSamples( m_iSourceWidth );
  pps0.setPicHeightInLumaSamples( m_iSourceHeight );
  if (pps0.getPicWidthInLumaSamples() == sps0.getMaxPicWidthInLumaSamples() && pps0.getPicHeightInLumaSamples() == sps0.getMaxPicHeightInLumaSamples())
  {
    pps0.setConformanceWindow( sps0.getConformanceWindow() );
    pps0.setConformanceWindowFlag( false );
  }
  else
  {
    pps0.setConformanceWindow( m_conformanceWindow );
    pps0.setConformanceWindowFlag( m_conformanceWindow.getWindowEnabledFlag() );
  }
  xInitPPS(pps0, sps0);
  // initialize APS
  xInitRPL(sps0, isFieldCoding);

  if (m_resChangeInClvsEnabled)
  {
    PPS &pps = *( m_ppsMap.allocatePS( ENC_PPS_ID_RPR ) );
    Window& inputScalingWindow = pps0.getScalingWindow();
    int scaledWidth = int( ( pps0.getPicWidthInLumaSamples() - SPS::getWinUnitX( sps0.getChromaFormatIdc() ) * ( inputScalingWindow.getWindowLeftOffset() + inputScalingWindow.getWindowRightOffset() ) ) / m_scalingRatioHor );
    int minSizeUnit = std::max(8, 1 << sps0.getLog2MinCodingBlockSize());
    int temp = scaledWidth / minSizeUnit;
    int width = ( scaledWidth - ( temp * minSizeUnit) > 0 ? temp + 1 : temp ) * minSizeUnit;

    int scaledHeight = int( ( pps0.getPicHeightInLumaSamples() - SPS::getWinUnitY( sps0.getChromaFormatIdc() ) * ( inputScalingWindow.getWindowTopOffset() + inputScalingWindow.getWindowBottomOffset() ) ) / m_scalingRatioVer );
    temp = scaledHeight / minSizeUnit;
    int height = ( scaledHeight - ( temp * minSizeUnit) > 0 ? temp + 1 : temp ) * minSizeUnit;

    pps.setPicWidthInLumaSamples( width );
    pps.setPicHeightInLumaSamples( height );

    Window conformanceWindow;
    conformanceWindow.setWindow( 0, ( width - scaledWidth ) / SPS::getWinUnitX( sps0.getChromaFormatIdc() ), 0, ( height - scaledHeight ) / SPS::getWinUnitY( sps0.getChromaFormatIdc() ) );
    if (pps.getPicWidthInLumaSamples() == sps0.getMaxPicWidthInLumaSamples() && pps.getPicHeightInLumaSamples() == sps0.getMaxPicHeightInLumaSamples())
    {
      pps.setConformanceWindow( sps0.getConformanceWindow() );
      pps.setConformanceWindowFlag( false );
    }
    else
    {
      pps.setConformanceWindow( conformanceWindow );
      pps.setConformanceWindowFlag( pps.getConformanceWindow().getWindowEnabledFlag() );
    }

    Window scalingWindow;
    scalingWindow.setWindow( 0, ( width - scaledWidth ) / SPS::getWinUnitX( sps0.getChromaFormatIdc() ), 0, ( height - scaledHeight ) / SPS::getWinUnitY( sps0.getChromaFormatIdc() ) );
    pps.setScalingWindow( scalingWindow );

    //register the width/height of the current pic into reference SPS
    if (!sps0.getPPSValidFlag(pps.getPPSId()))
    {
      sps0.setPPSValidFlag(pps.getPPSId(), true);
      sps0.setScalingWindowSizeInPPS(pps.getPPSId(), scaledWidth, scaledHeight);
    }
    int curSeqMaxPicWidthY = sps0.getMaxPicWidthInLumaSamples();    // sps_pic_width_max_in_luma_samples
    int curSeqMaxPicHeightY = sps0.getMaxPicHeightInLumaSamples();  // sps_pic_height_max_in_luma_samples
    int curPicWidthY = width;                                       // pps_pic_width_in_luma_samples
    int curPicHeightY = height;                                     // pps_pic_height_in_luma_samples
    int max8MinCbSizeY = std::max((int)8, (1 << sps0.getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)
    //Warning message of potential scaling window size violation
    for (int i = 0; i < 64; i++)
    {
      if (sps0.getPPSValidFlag(i))
      {
        if ((scaledWidth * curSeqMaxPicWidthY) < sps0.getScalingWindowSizeInPPS(i).width * (curPicWidthY - max8MinCbSizeY))
          printf("Potential violation: (curScaledWIdth * curSeqMaxPicWidthY) should be greater than or equal to refScaledWidth * (curPicWidthY - max(8, MinCbSizeY)\n");
        if ((scaledHeight * curSeqMaxPicHeightY) < sps0.getScalingWindowSizeInPPS(i).height * (curPicHeightY - max8MinCbSizeY))
          printf("Potential violation: (curScaledHeight * curSeqMaxPicHeightY) should be greater than or equal to refScaledHeight * (curPicHeightY - max(8, MinCbSizeY)\n");
      }
    }

    // disable picture partitioning for scaled RPR pictures (slice/tile config only provided for the original resolution)
    m_noPicPartitionFlag = true;

    xInitPPS( pps, sps0 ); // will allocate memory for and initialize pps.pcv inside
    
    if( pps.getWrapAroundEnabledFlag() )
    {
      int minCbSizeY = (1 << sps0.getLog2MinCodingBlockSize());
      pps.setPicWidthMinusWrapAroundOffset      ((pps.getPicWidthInLumaSamples()/minCbSizeY) - (m_wrapAroundOffset * pps.getPicWidthInLumaSamples() / pps0.getPicWidthInLumaSamples() / minCbSizeY) );
      pps.setWrapAroundOffset                   (minCbSizeY * (pps.getPicWidthInLumaSamples() / minCbSizeY - pps.getPicWidthMinusWrapAroundOffset()));

    }
    else 
    {
      pps.setPicWidthMinusWrapAroundOffset      (0);
      pps.setWrapAroundOffset                   ( 0 );       
    }
  }

#if ER_CHROMA_QP_WCG_PPS
  if (m_wcgChromaQpControl.isEnabled())
  {
    PPS &pps1=*(m_ppsMap.allocatePS(1));
    xInitPPS(pps1, sps0);
  }
#endif
  if (getUseCompositeRef())
  {
    PPS &pps2 = *(m_ppsMap.allocatePS(2));
    xInitPPS(pps2, sps0);
    xInitPPSforLT(pps2);
  }
  xInitPicHeader(m_picHeader, sps0, pps0);

  // initialize processing unit classes
  m_cGOPEncoder.  init( this );
  m_cSliceEncoder.init( this, sps0 );
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    // precache a few objects
    for( int i = 0; i < 10; i++ )
    {
      auto x = m_CtxCache[jId].get();
      m_CtxCache[jId].cache( x );
    }

    m_cCuEncoder[jId].init( this, sps0, jId );

    // initialize transform & quantization class
    m_cTrQuant[jId].init( jId == 0 ? nullptr : m_cTrQuant[0].getQuant(),
                          1 << m_log2MaxTbSize,

                          m_useRDOQ,
                          m_useRDOQTS,
#if T0196_SELECTIVE_RDOQ
                          m_useSelectiveRDOQ,
#endif
                          true
    );

    // initialize encoder search class
    CABACWriter* cabacEstimator = m_CABACEncoder[jId].getCABACEstimator( &sps0 );
    m_cIntraSearch[jId].init( this,
                              &m_cTrQuant[jId],
                              &m_cRdCost[jId],
                              cabacEstimator,
                              getCtxCache( jId ), m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize
                            , &m_cReshaper[jId]
                            , sps0.getBitDepth(CHANNEL_TYPE_LUMA)
    );
    m_cInterSearch[jId].init( this,
                              &m_cTrQuant[jId],
                              m_iSearchRange,
                              m_bipredSearchRange,
                              m_motionEstimationSearchMethod,
                              getUseCompositeRef(),
                              m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, &m_cRdCost[jId], cabacEstimator, getCtxCache( jId )
                           , &m_cReshaper[jId]
    );

    // link temporary buffets from intra search with inter search to avoid unnecessary memory overhead
    m_cInterSearch[jId].setTempBuffers( m_cIntraSearch[jId].getSplitCSBuf(), m_cIntraSearch[jId].getFullCSBuf(), m_cIntraSearch[jId].getSaveCSBuf() );
  }
#else  // ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  m_cCuEncoder.   init( this, sps0 );

  // initialize transform & quantization class
  m_cTrQuant.init( nullptr,
                   1 << m_log2MaxTbSize,
                   m_useRDOQ,
                   m_useRDOQTS,
#if T0196_SELECTIVE_RDOQ
                   m_useSelectiveRDOQ,
#endif
                   true
  );

  // initialize encoder search class
  CABACWriter* cabacEstimator = m_CABACEncoder.getCABACEstimator(&sps0);
  m_cIntraSearch.init( this,
                       &m_cTrQuant,
                       &m_cRdCost,
                       cabacEstimator,
                       getCtxCache(), m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize
                     , &m_cReshaper
                     , sps0.getBitDepth(CHANNEL_TYPE_LUMA)
  );
  m_cInterSearch.init( this,
                       &m_cTrQuant,
                       m_iSearchRange,
                       m_bipredSearchRange,
                       m_motionEstimationSearchMethod,
                       getUseCompositeRef(),
    m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, &m_cRdCost, cabacEstimator, getCtxCache()
                     , &m_cReshaper
  );

  // link temporary buffets from intra search with inter search to avoid unneccessary memory overhead
  m_cInterSearch.setTempBuffers( m_cIntraSearch.getSplitCSBuf(), m_cIntraSearch.getFullCSBuf(), m_cIntraSearch.getSaveCSBuf() );
#endif // ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

  m_iMaxRefPicNum = 0;

#if ER_CHROMA_QP_WCG_PPS
  if( m_wcgChromaQpControl.isEnabled() )
  {
    xInitScalingLists( sps0, *m_apsMap.getPS( 1 ) );
    xInitScalingLists( sps0, aps0 );
  }
  else
#endif
  {
    xInitScalingLists( sps0, aps0 );
  }
  if (m_resChangeInClvsEnabled)
  {
    xInitScalingLists( sps0, *m_apsMap.getPS( ENC_PPS_ID_RPR ) );
  }
  if (getUseCompositeRef())
  {
    Picture *picBg = new Picture;
    picBg->create( sps0.getChromaFormatIdc(), Size( pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples() ), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false, m_layerId, m_gopBasedTemporalFilterEnabled );
    picBg->getRecoBuf().fill(0);
    picBg->finalInit( m_vps, sps0, pps0, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );
    picBg->allocateNewSlice();
    picBg->createSpliceIdx(pps0.pcv->sizeInCtus);
    m_cGOPEncoder.setPicBg(picBg);
    Picture *picOrig = new Picture;
    picOrig->create( sps0.getChromaFormatIdc(), Size( pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples() ), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false, m_layerId, m_gopBasedTemporalFilterEnabled );
    picOrig->getOrigBuf().fill(0);
    m_cGOPEncoder.setPicOrig(picOrig);
  }
}

void EncLib::xInitScalingLists( SPS &sps, APS &aps )
{
  // Initialise scaling lists
  // The encoder will only use the SPS scaling lists. The PPS will never be marked present.
  const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE] =
  {
    sps.getMaxLog2TrDynamicRange(CHANNEL_TYPE_LUMA),
    sps.getMaxLog2TrDynamicRange(CHANNEL_TYPE_CHROMA)
  };

  Quant* quant = getTrQuant()->getQuant();

  if(getUseScalingListId() == SCALING_LIST_OFF)
  {
    quant->setFlatScalingList(maxLog2TrDynamicRange, sps.getBitDepths());
    quant->setUseScalingList(false);
#if ENABLE_SPLIT_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setFlatScalingList( maxLog2TrDynamicRange, sps.getBitDepths() );
      getTrQuant( jId )->getQuant()->setUseScalingList( false );
    }
#endif
  }
  else if(getUseScalingListId() == SCALING_LIST_DEFAULT)
  {
    aps.getScalingList().setDefaultScalingList ();
    quant->setScalingList( &( aps.getScalingList() ), maxLog2TrDynamicRange, sps.getBitDepths() );
    quant->setUseScalingList(true);
#if ENABLE_SPLIT_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setUseScalingList( true );
    }
    sps.setDisableScalingMatrixForLfnstBlks(getDisableScalingMatrixForLfnstBlks());
#endif
  }
  else if(getUseScalingListId() == SCALING_LIST_FILE_READ)
  {
    aps.getScalingList().setDefaultScalingList();
    CHECK_( aps.getScalingList().xParseScalingList( getScalingListFileName() ), "Error Parsing Scaling List Input File" );
    aps.getScalingList().checkDcOfMatrix();
    if( aps.getScalingList().isNotDefaultScalingList() == false )
    {
      setUseScalingListId( SCALING_LIST_DEFAULT );
    }
    aps.getScalingList().setChromaScalingListPresentFlag((sps.getChromaFormatIdc()!=CHROMA_400));
    quant->setScalingList( &( aps.getScalingList() ), maxLog2TrDynamicRange, sps.getBitDepths() );
    quant->setUseScalingList(true);
#if ENABLE_SPLIT_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setUseScalingList( true );
    }
#endif

    sps.setDisableScalingMatrixForLfnstBlks(getDisableScalingMatrixForLfnstBlks());
  }
  else
  {
    THROW("error : ScalingList == " << getUseScalingListId() << " not supported\n");
  }

  if( getUseScalingListId() == SCALING_LIST_FILE_READ )
  {
    // Prepare delta's:
    for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
    {
      if (aps.getScalingList().getChromaScalingListPresentFlag()||aps.getScalingList().isLumaScalingList(scalingListId))
      {
        aps.getScalingList().checkPredMode(scalingListId);
      }
    }
  }
}

void EncLib::xInitPPSforLT(PPS& pps)
{
  pps.setOutputFlagPresentFlag(true);
  pps.setDeblockingFilterControlPresentFlag(true);
  pps.setPPSDeblockingFilterDisabledFlag(true);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncLib::deletePicBuffer()
{
  PicList::iterator iterPic = m_cListPic.begin();
  int iSize = int( m_cListPic.size() );

  for ( int i = 0; i < iSize; i++ )
  {
    Picture* pcPic = *(iterPic++);

    pcPic->destroy();

    // get rid of the qpadaption layer
    while( pcPic->aqlayer.size() )
    {
      delete pcPic->aqlayer.back(); pcPic->aqlayer.pop_back();
    }

    delete pcPic;
    pcPic = NULL;
  }

  m_cListPic.clear();
}

bool EncLib::encodePrep( bool flush, PelStorage* pcPicYuvOrg, PelStorage* cPicYuvTrueOrg, PelStorage* pcPicYuvFilteredOrg, const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut, int& iNumEncoded )
{
  if( m_compositeRefEnabled && m_cGOPEncoder.getPicBg()->getSpliceFull() && m_iPOCLast >= 10 && m_iNumPicRcvd == 0 && m_cGOPEncoder.getEncodedLTRef() == false )
  {
    Picture* picCurr = NULL;
    xGetNewPicBuffer( rcListPicYuvRecOut, picCurr, 2 );
    const PPS *pps = m_ppsMap.getPS( 2 );
    const SPS *sps = m_spsMap.getPS( pps->getSPSId() );

    picCurr->M_BUFS( 0, PIC_ORIGINAL ).copyFrom( m_cGOPEncoder.getPicBg()->getRecoBuf() );
    picCurr->finalInit( m_vps, *sps, *pps, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );
    picCurr->poc = m_iPOCLast - 1;
    m_iPOCLast -= 2;
    if( getUseAdaptiveQP() )
    {
      AQpPreanalyzer::preanalyze( picCurr );
    }
    if( m_RCEnableRateControl )
    {
      m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
    }

    m_cGOPEncoder.compressGOP( m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, false, false, 
      snrCSC, m_printFrameMSE, m_printMSSSIM, true, 0 );

#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cGOPEncoder.getMetricTime();
#endif
    m_cGOPEncoder.setEncodedLTRef( true );
    if( m_RCEnableRateControl )
    {
      m_cRateCtrl.destroyRCGOP();
    }

    iNumEncoded = 0;
    m_iNumPicRcvd = 0;
  }

  //PROF_ACCUM_AND_START_NEW_SET( getProfilerPic(), P_GOP_LEVEL );
  if( pcPicYuvOrg != NULL )
  {
    // get original YUV
    Picture* pcPicCurr = NULL;

    int ppsID = -1; // Use default PPS ID
#if ER_CHROMA_QP_WCG_PPS
    if( getWCGChromaQPControl().isEnabled() )
    {
      ppsID = getdQPs()[m_iPOCLast / ( m_compositeRefEnabled ? 2 : 1 ) + 1];
      ppsID += ( getSwitchPOC() != -1 && ( m_iPOCLast + 1 >= getSwitchPOC() ) ? 1 : 0 );
    }
#endif

    if( m_resChangeInClvsEnabled && m_intraPeriod == -1 )
    {
      const int poc = m_iPOCLast + ( m_compositeRefEnabled ? 2 : 1 );

      if( poc / m_switchPocPeriod % 2 )
      {
        ppsID = ENC_PPS_ID_RPR;
      }
      else
      {
        ppsID = 0;
      }
    }

    if( m_vps->getMaxLayers() > 1 )
    {
      ppsID = m_vps->getGeneralLayerIdx( m_layerId );
    }

    xGetNewPicBuffer( rcListPicYuvRecOut, pcPicCurr, ppsID );

    const PPS *pPPS = ( ppsID < 0 ) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS( ppsID );
    const SPS *pSPS = m_spsMap.getPS( pPPS->getSPSId() );

    if (m_resChangeInClvsEnabled)
    {
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Y ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Y ) );
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Cb ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Cb ) );
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Cr ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Cr ) );

      pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT ).getBuf( COMPONENT_Y ).copyFrom( cPicYuvTrueOrg->getBuf( COMPONENT_Y ) );
      pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT ).getBuf( COMPONENT_Cb ).copyFrom( cPicYuvTrueOrg->getBuf( COMPONENT_Cb ) );
      pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT ).getBuf( COMPONENT_Cr ).copyFrom( cPicYuvTrueOrg->getBuf( COMPONENT_Cr ) );

      if(m_gopBasedTemporalFilterEnabled)
      {
        pcPicCurr->M_BUFS( 0, PIC_FILTERED_ORIGINAL_INPUT ).getBuf( COMPONENT_Y ).copyFrom( pcPicYuvFilteredOrg->getBuf( COMPONENT_Y ) );
        pcPicCurr->M_BUFS( 0, PIC_FILTERED_ORIGINAL_INPUT ).getBuf( COMPONENT_Cb ).copyFrom( pcPicYuvFilteredOrg->getBuf( COMPONENT_Cb ) );
        pcPicCurr->M_BUFS( 0, PIC_FILTERED_ORIGINAL_INPUT ).getBuf( COMPONENT_Cr ).copyFrom( pcPicYuvFilteredOrg->getBuf( COMPONENT_Cr ) );
      }

      const ChromaFormat chromaFormatIDC = pSPS->getChromaFormatIdc();

      const PPS *refPPS = m_ppsMap.getPS( 0 );
      const Window& curScalingWindow = pPPS->getScalingWindow();
      int curPicWidth = pPPS->getPicWidthInLumaSamples()   - SPS::getWinUnitX( pSPS->getChromaFormatIdc() ) * ( curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset() );
      int curPicHeight = pPPS->getPicHeightInLumaSamples() - SPS::getWinUnitY( pSPS->getChromaFormatIdc() ) * ( curScalingWindow.getWindowTopOffset()  + curScalingWindow.getWindowBottomOffset() );

      const Window& refScalingWindow = refPPS->getScalingWindow();
      int refPicWidth = refPPS->getPicWidthInLumaSamples()   - SPS::getWinUnitX( pSPS->getChromaFormatIdc() ) * ( refScalingWindow.getWindowLeftOffset() + refScalingWindow.getWindowRightOffset() );
      int refPicHeight = refPPS->getPicHeightInLumaSamples() - SPS::getWinUnitY( pSPS->getChromaFormatIdc() ) * ( refScalingWindow.getWindowTopOffset()  + refScalingWindow.getWindowBottomOffset() );

      int xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
      int yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;
      std::pair<int, int> scalingRatio = std::pair<int, int>( xScale, yScale );

      Picture::rescalePicture( scalingRatio, *pcPicYuvOrg, refPPS->getScalingWindow(), pcPicCurr->getOrigBuf(), pPPS->getScalingWindow(), chromaFormatIDC, pSPS->getBitDepths(), true, true,
        pSPS->getHorCollocatedChromaFlag(), pSPS->getVerCollocatedChromaFlag() );
      Picture::rescalePicture( scalingRatio, *cPicYuvTrueOrg, refPPS->getScalingWindow(), pcPicCurr->getTrueOrigBuf(), pPPS->getScalingWindow(), chromaFormatIDC, pSPS->getBitDepths(), true, true,
        pSPS->getHorCollocatedChromaFlag(), pSPS->getVerCollocatedChromaFlag() );
      if(m_gopBasedTemporalFilterEnabled)
      {
        Picture::rescalePicture( scalingRatio, *pcPicYuvFilteredOrg, refPPS->getScalingWindow(), pcPicCurr->getFilteredOrigBuf(), pPPS->getScalingWindow(), chromaFormatIDC, pSPS->getBitDepths(), true, true,
          pSPS->getHorCollocatedChromaFlag(), pSPS->getVerCollocatedChromaFlag() );
      }
    }
    else
    {
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL ).swap( *pcPicYuvOrg );
      pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL ).swap( *cPicYuvTrueOrg );
      if(m_gopBasedTemporalFilterEnabled)
      {
        pcPicCurr->M_BUFS( 0, PIC_FILTERED_ORIGINAL ).swap( *pcPicYuvFilteredOrg );
      }
    }

    pcPicCurr->finalInit( m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );

    pcPicCurr->poc = m_iPOCLast;

    // compute image characteristics
    if( getUseAdaptiveQP() )
    {
      AQpPreanalyzer::preanalyze( pcPicCurr );
    }
  }

  if( ( m_iNumPicRcvd == 0 ) || ( !flush && ( m_iPOCLast != 0 ) && ( m_iNumPicRcvd != m_iGOPSize ) && ( m_iGOPSize != 0 ) ) )
  {
    iNumEncoded = 0;
    return true;
  }

  if( m_RCEnableRateControl )
  {
    m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
  }

  m_picIdInGOP = 0;

  return false;
}

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   pcPicYuvOrg         original YUV picture
 \param   pcPicYuvTrueOrg
 \param   snrCSC
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  accessUnitsOut      list of output access units
 \retval  iNumEncoded         number of encoded pictures
 */

bool EncLib::encode( const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut, int& iNumEncoded )
{
  // compress GOP
  m_cGOPEncoder.compressGOP( m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut,
    false, false, snrCSC, m_printFrameMSE, m_printMSSSIM, false, m_picIdInGOP );

  m_picIdInGOP++;

  // go over all pictures in a GOP excluding the first IRAP
  if( m_picIdInGOP != m_iGOPSize && m_iPOCLast )
  {
    return true;
  }

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

  if( m_RCEnableRateControl )
  {
    m_cRateCtrl.destroyRCGOP();
  }

  iNumEncoded = m_iNumPicRcvd;
  m_iNumPicRcvd = 0;
  m_uiNumAllPicCoded += iNumEncoded;

  return false;
}

/**------------------------------------------------
 Separate interlaced frame into two fields
 -------------------------------------------------**/
void separateFields(Pel* org, Pel* dstField, uint32_t stride, uint32_t width, uint32_t height, bool isTop)
{
  if (!isTop)
  {
    org += stride;
  }
  for (int y = 0; y < height>>1; y++)
  {
    for (int x = 0; x < width; x++)
    {
      dstField[x] = org[x];
    }

    dstField += stride;
    org += stride*2;
  }

}

bool EncLib::encodePrep( bool flush, PelStorage* pcPicYuvOrg, PelStorage* pcPicYuvTrueOrg, PelStorage* pcPicYuvFilteredOrg, const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut,
  int& iNumEncoded, bool isTff )
{
  iNumEncoded = 0;
  bool keepDoing = true;

  for( int fieldNum = 0; fieldNum < 2; fieldNum++ )
  {
    if( pcPicYuvOrg )
    {
      /* -- field initialization -- */
      const bool isTopField = isTff == ( fieldNum == 0 );

      Picture *pcField;
      xGetNewPicBuffer( rcListPicYuvRecOut, pcField, -1 );

      for( uint32_t comp = 0; comp < ::getNumberValidComponents( pcPicYuvOrg->chromaFormat ); comp++ )
      {
        const ComponentID compID = ComponentID( comp );
        {
          PelBuf compBuf = pcPicYuvOrg->get( compID );
          separateFields( compBuf.buf,
            pcField->getOrigBuf().get( compID ).buf,
            compBuf.stride,
            compBuf.width,
            compBuf.height,
            isTopField );
          // to get fields of true original buffer to avoid wrong PSNR calculation in summary
          compBuf = pcPicYuvTrueOrg->get( compID );
          separateFields( compBuf.buf,
            pcField->getTrueOrigBuf().get(compID).buf,
            compBuf.stride,
            compBuf.width,
            compBuf.height,
            isTopField);
          if(m_gopBasedTemporalFilterEnabled)
          {
            compBuf = pcPicYuvFilteredOrg->get( compID );
            separateFields( compBuf.buf,
              pcField->getTrueOrigBuf().get(compID).buf,
              compBuf.stride,
              compBuf.width,
              compBuf.height,
              isTopField);
          }
        }
      }

      int ppsID = -1; // Use default PPS ID
      const PPS *pPPS = ( ppsID < 0 ) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS( ppsID );
      const SPS *pSPS = m_spsMap.getPS( pPPS->getSPSId() );

      pcField->finalInit( m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );

      pcField->poc = m_iPOCLast;
      pcField->reconstructed = false;

      pcField->setBorderExtension( false );// where is this normally?

      pcField->topField = isTopField;                  // interlaced requirement

      // compute image characteristics
      if( getUseAdaptiveQP() )
      {
        AQpPreanalyzer::preanalyze( pcField );
      }
    }

  }

  if( m_iNumPicRcvd && ( flush || m_iPOCLast == 1 || m_iNumPicRcvd == m_iGOPSize ) )
  {
    m_picIdInGOP = 0;
    keepDoing = false;
  }

  return keepDoing;
}

bool EncLib::encode( const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut, int& iNumEncoded, bool isTff )
{
  iNumEncoded = 0;

  for( int fieldNum = 0; fieldNum < 2; fieldNum++ )
  {
    m_iPOCLast = m_iPOCLast < 2 ? fieldNum : m_iPOCLast;

    // compress GOP
    m_cGOPEncoder.compressGOP( m_iPOCLast, m_iPOCLast < 2 ? m_iPOCLast + 1 : m_iNumPicRcvd, m_cListPic, 
      rcListPicYuvRecOut, true, isTff, snrCSC, m_printFrameMSE, m_printMSSSIM, false, m_picIdInGOP );
#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

    m_picIdInGOP++;
  }

  // go over all pictures in a GOP excluding first top field and first bottom field
  if( m_picIdInGOP != m_iGOPSize && m_iPOCLast > 1 )
  {
    return true;
  }

  iNumEncoded += m_iNumPicRcvd;
  m_uiNumAllPicCoded += m_iNumPicRcvd;
  m_iNumPicRcvd = 0;

  return false;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \retval rpcPic obtained picture buffer
 */
void EncLib::xGetNewPicBuffer ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, int ppsId )
{
  // rotate the output buffer
  rcListPicYuvRecOut.push_back( rcListPicYuvRecOut.front() ); rcListPicYuvRecOut.pop_front();

  rpcPic=0;

  // At this point, the SPS and PPS can be considered activated - they are copied to the new Pic.
  const PPS *pPPS=(ppsId<0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsId);
  CHECK_(!(pPPS!=0), "Unspecified error");
  const PPS &pps=*pPPS;

  const SPS *pSPS=m_spsMap.getPS(pps.getSPSId());
  CHECK_(!(pSPS!=0), "Unspecified error");
  const SPS &sps=*pSPS;

  Slice::sortPicList(m_cListPic);

  // use an entry in the buffered list if the maximum number that need buffering has been reached:
  int maxDecPicBuffering = ( m_vps == nullptr || m_vps->m_numLayersInOls[m_vps->m_targetOlsIdx] == 1 ) ? sps.getMaxDecPicBuffering( MAX_TLAYER - 1 ) : m_vps->getMaxDecPicBuffering( MAX_TLAYER - 1 );

  if( m_cListPic.size() >= (uint32_t)( m_iGOPSize + maxDecPicBuffering + 2 ) )
  {
    PicList::iterator iterPic = m_cListPic.begin();
    int iSize = int( m_cListPic.size() );
    for( int i = 0; i < iSize; i++ )
    {
      rpcPic = *iterPic;
      if( !rpcPic->referenced && rpcPic->layerId == m_layerId )
      {
        break;
      }
      else
      {
        rpcPic = nullptr;
      }
      iterPic++;
    }

    // If PPS ID is the same, we will assume that it has not changed since it was last used
    // and return the old object.
    if( rpcPic && pps.getPPSId() != rpcPic->cs->pps->getPPSId() )
    {
      // the IDs differ - free up an entry in the list, and then create a new one, as with the case where the max buffering state has not been reached.
      rpcPic->destroy();
      delete rpcPic;
      m_cListPic.erase(iterPic);
      rpcPic=0;
    }
  }

  if (rpcPic==0)
  {
    rpcPic = new Picture;
    rpcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, false, m_layerId, m_gopBasedTemporalFilterEnabled );
    if (m_resChangeInClvsEnabled)
    {
      const PPS &pps0 = *m_ppsMap.getPS(0);
      rpcPic->M_BUFS(0, PIC_ORIGINAL_INPUT).create(sps.getChromaFormatIdc(), Area(Position(), Size(pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples())));
      rpcPic->M_BUFS(0, PIC_TRUE_ORIGINAL_INPUT).create(sps.getChromaFormatIdc(), Area(Position(), Size(pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples())));
      if(m_gopBasedTemporalFilterEnabled)
      {
        rpcPic->M_BUFS(0, PIC_FILTERED_ORIGINAL_INPUT).create(sps.getChromaFormatIdc(), Area(Position(), Size(pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples())));
      } 
    }
    if ( getUseAdaptiveQP() )
    {
      const uint32_t iMaxDQPLayer = m_picHeader.getCuQpDeltaSubdivIntra()/2+1;
      rpcPic->aqlayer.resize( iMaxDQPLayer );
      for (uint32_t d = 0; d < iMaxDQPLayer; d++)
      {
        rpcPic->aqlayer[d] = new AQpLayer( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples(), sps.getMaxCUWidth() >> d, sps.getMaxCUHeight() >> d );
      }
    }

    m_cListPic.push_back( rpcPic );
  }

  rpcPic->setBorderExtension( false );
  rpcPic->reconstructed = false;
  rpcPic->referenced = true;
  rpcPic->getHashMap()->clearAll();

  m_iPOCLast += (m_compositeRefEnabled ? 2 : 1);
  m_iNumPicRcvd++;
}

void EncLib::xInitVPS( const SPS& sps )
{
  // The SPS must have already been set up.
  // set the VPS profile information.

  m_vps->m_olsHrdParams.clear();
  m_vps->m_olsHrdParams.resize(m_vps->getNumOlsTimingHrdParamsMinus1(), std::vector<OlsHrdParams>(m_vps->getMaxSubLayers()));
  ProfileLevelTierFeatures profileLevelTierFeatures;
  profileLevelTierFeatures.extractPTLInformation( sps );
#if JVET_R0193
  m_vps->setMaxTidIlRefPicsPlus1(m_cfgVPSParameters.m_maxTidILRefPicsPlus1);
#endif
  m_vps->deriveOutputLayerSets();
  m_vps->deriveTargetOutputLayerSet( m_vps->m_targetOlsIdx );


  // number of the DPB parameters is set equal to the number of OLS containing multi layers
  if( !m_vps->getEachLayerIsAnOlsFlag() )
  {
    m_vps->m_numDpbParams = m_vps->getNumMultiLayeredOlss();
  }

  if( m_vps->m_dpbParameters.size() != m_vps->m_numDpbParams )
  {
    m_vps->m_dpbParameters.resize( m_vps->m_numDpbParams );
  }

  if( m_vps->m_dpbMaxTemporalId.size() != m_vps->m_numDpbParams )
  {
    m_vps->m_dpbMaxTemporalId.resize( m_vps->m_numDpbParams );
  }

  for( int olsIdx = 0, dpbIdx = 0; olsIdx < m_vps->m_numOutputLayersInOls.size(); olsIdx++ )
  {
    if ( m_vps->getNumLayersInOls(olsIdx) > 1 )
    { 
      if( std::find( m_vps->m_layerIdInOls[olsIdx].begin(), m_vps->m_layerIdInOls[olsIdx].end(), m_layerId ) != m_vps->m_layerIdInOls[olsIdx].end() )
      {
        m_vps->setOlsDpbPicWidth( olsIdx, std::max<int>( sps.getMaxPicWidthInLumaSamples(), m_vps->getOlsDpbPicSize( olsIdx ).width ) );
        m_vps->setOlsDpbPicHeight( olsIdx, std::max<int>( sps.getMaxPicHeightInLumaSamples(), m_vps->getOlsDpbPicSize( olsIdx ).height ) );
        m_vps->setOlsDpbChromaFormatIdc( olsIdx, std::max<int>(sps.getChromaFormatIdc(), m_vps->getOlsDpbChromaFormatIdc( olsIdx )));
        m_vps->setOlsDpbBitDepthMinus8( olsIdx, std::max<int>(sps.getBitDepth(CHANNEL_TYPE_LUMA) - 8, m_vps->getOlsDpbBitDepthMinus8( olsIdx )));
      }

      m_vps->setOlsDpbParamsIdx( olsIdx, dpbIdx );
      dpbIdx++;
    }
  }

  //for( int i = 0; i < m_vps->m_numDpbParams; i++ )
  for( int i = 0; i < m_vps->m_numOutputLayersInOls.size(); i++ )
  {
    if ( m_vps->getNumLayersInOls(i) > 1 )
    { 
      int dpbIdx = m_vps->getOlsDpbParamsIdx( i );

      if( m_vps->getMaxSubLayers() == 1 )
      {
        // When vps_max_sublayers_minus1 is equal to 0, the value of vps_dpb_max_tid[ dpbIdx ] is inferred to be equal to 0.
        m_vps->m_dpbMaxTemporalId[dpbIdx] = 0;
      }
      else
      {
        if( m_vps->getDefaultPtlDpbHrdMaxTidFlag() )
        {
          // When vps_max_sublayers_minus1 is greater than 0 and vps_all_layers_same_num_sublayers_flag is equal to 1, the value of vps_dpb_max_tid[ dpbIdx ] is inferred to be equal to vps_max_sublayers_minus1.
          m_vps->m_dpbMaxTemporalId[dpbIdx] = m_vps->getMaxSubLayers() - 1;
        }
        else
        {
          m_vps->m_dpbMaxTemporalId[dpbIdx] = m_vps->getMaxSubLayers() - 1;
        }
      }

      int decPicBuffering[MAX_TLAYER] = { 0 };

      for( int lIdx = 0; lIdx < m_vps->getNumLayersInOls( i ); lIdx++ )
      {
        for( int tId = 0; tId < MAX_TLAYER; tId++ )
        {
          decPicBuffering[tId] += m_layerDecPicBuffering[m_vps->getLayerIdInOls( i, lIdx ) * MAX_TLAYER + tId];
        }
      }
    
      for( int j = ( m_vps->m_sublayerDpbParamsPresentFlag ? 0 : m_vps->m_dpbMaxTemporalId[dpbIdx] ); j <= m_vps->m_dpbMaxTemporalId[dpbIdx]; j++ )
      {
        m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[j] = decPicBuffering[j] > 0 ? decPicBuffering[j] : profileLevelTierFeatures.getMaxDpbSize( m_vps->getOlsDpbPicSize( i ).width * m_vps->getOlsDpbPicSize( i ).height );
        m_vps->m_dpbParameters[dpbIdx].m_maxNumReorderPics[j] = m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[j] - 1;
        m_vps->m_dpbParameters[dpbIdx].m_maxLatencyIncreasePlus1[j] = 0;

        CHECK_( m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[j] > profileLevelTierFeatures.getMaxDpbSize( m_vps->getOlsDpbPicSize( i ).width * m_vps->getOlsDpbPicSize( i ).height ), "DPB size is not sufficient" );
      }

      for( int j = ( m_vps->m_sublayerDpbParamsPresentFlag ? m_vps->m_dpbMaxTemporalId[dpbIdx] : 0 ); j < m_vps->m_dpbMaxTemporalId[dpbIdx]; j++ )
      {
        // When dpb_max_dec_pic_buffering_minus1[ dpbIdx ] is not present for dpbIdx in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to dpb_max_dec_pic_buffering_minus1[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[j] = m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[m_vps->m_dpbMaxTemporalId[dpbIdx]];

        // When dpb_max_num_reorder_pics[ dpbIdx ] is not present for dpbIdx in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to dpb_max_num_reorder_pics[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].m_maxNumReorderPics[j] = m_vps->m_dpbParameters[dpbIdx].m_maxNumReorderPics[m_vps->m_dpbMaxTemporalId[dpbIdx]];

        // When dpb_max_latency_increase_plus1[ dpbIdx ] is not present for dpbIdx in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to dpb_max_latency_increase_plus1[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].m_maxLatencyIncreasePlus1[j] = m_vps->m_dpbParameters[dpbIdx].m_maxLatencyIncreasePlus1[m_vps->m_dpbMaxTemporalId[dpbIdx]];
      }
    }
  }
  for (int i = 0; i < m_vps->getNumOutputLayerSets(); i++)
  {
    m_vps->setHrdMaxTid(i, m_vps->getMaxSubLayers() - 1);
  }

#if !JVET_R0193
  if (m_cfgVPSParameters.m_maxTidILRefPicsPlus1 >= 0)
  {
    for (int i = 0; i < m_vps->getMaxLayers(); i++)
    {
      m_vps->setMaxTidIlRefPicsPlus1(i, m_cfgVPSParameters.m_maxTidILRefPicsPlus1);
    }
  }
#endif
  m_vps->checkVPS();
}

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
void EncLib::xInitOPI(OPI& opi)
{
  if (m_OPIEnabled && m_vps)
  {
    if (!opi.getOlsInfoPresentFlag())
    {
      opi.setOpiOlsIdx(m_vps->deriveTargetOLSIdx());
      opi.setOlsInfoPresentFlag(true);    
    }
    if (!opi.getHtidInfoPresentFlag())
    {
      opi.setOpiHtidPlus1(m_vps->getMaxTidinTOls(opi.getOpiOlsIdx()) + 1);
      opi.setHtidInfoPresentFlag(true);
    }
  }
}
#endif

void EncLib::xInitDCI(DCI& dci, const SPS& sps)
{
  dci.setMaxSubLayersMinus1(sps.getMaxTLayers() - 1);
  std::vector<ProfileTierLevel> ptls;
  ptls.resize(1);
  ptls[0] = *sps.getProfileTierLevel();
  dci.setProfileTierLevel(ptls);
}

void EncLib::xInitSPS( SPS& sps )
{
  ProfileTierLevel* profileTierLevel = sps.getProfileTierLevel();
  ConstraintInfo* cinfo = profileTierLevel->getConstraintInfo();

  cinfo->setGciPresentFlag(m_gciPresentFlag);
  cinfo->setNoRprConstraintFlag(m_noRprConstraintFlag);
  cinfo->setNoResChangeInClvsConstraintFlag(m_noResChangeInClvsConstraintFlag);
  cinfo->setOneTilePerPicConstraintFlag(m_oneTilePerPicConstraintFlag);
  cinfo->setPicHeaderInSliceHeaderConstraintFlag(m_picHeaderInSliceHeaderConstraintFlag);
  cinfo->setOneSlicePerPicConstraintFlag(m_oneSlicePerPicConstraintFlag);
  cinfo->setNoIdrRplConstraintFlag(m_noIdrRplConstraintFlag);
  cinfo->setNoRectSliceConstraintFlag(m_noRectSliceConstraintFlag);
  cinfo->setOneSlicePerSubpicConstraintFlag(m_oneSlicePerSubpicConstraintFlag);
  cinfo->setNoSubpicInfoConstraintFlag(m_noSubpicInfoConstraintFlag);
  cinfo->setOnePictureOnlyConstraintFlag(m_onePictureOnlyConstraintFlag);
  cinfo->setIntraOnlyConstraintFlag         (m_intraOnlyConstraintFlag);
  cinfo->setMaxBitDepthConstraintIdc    (m_maxBitDepthConstraintIdc);
  cinfo->setMaxChromaFormatConstraintIdc((int)m_maxChromaFormatConstraintIdc);
  cinfo->setAllLayersIndependentConstraintFlag (m_allLayersIndependentConstraintFlag);
  cinfo->setNoMrlConstraintFlag (m_noMrlConstraintFlag);
  cinfo->setNoIspConstraintFlag (m_noIspConstraintFlag);
  cinfo->setNoMipConstraintFlag (m_noMipConstraintFlag);
  cinfo->setNoLfnstConstraintFlag (m_noLfnstConstraintFlag);
  cinfo->setNoMmvdConstraintFlag (m_noMmvdConstraintFlag);
  cinfo->setNoSmvdConstraintFlag (m_noSmvdConstraintFlag);
  cinfo->setNoProfConstraintFlag (m_noProfConstraintFlag);
  cinfo->setNoPaletteConstraintFlag (m_noPaletteConstraintFlag);
  cinfo->setNoActConstraintFlag (m_noActConstraintFlag);
  cinfo->setNoLmcsConstraintFlag (m_noLmcsConstraintFlag);
  cinfo->setNoExplicitScaleListConstraintFlag(m_noExplicitScaleListConstraintFlag);
  cinfo->setNoVirtualBoundaryConstraintFlag(m_noVirtualBoundaryConstraintFlag);
  cinfo->setNoMttConstraintFlag(m_noMttConstraintFlag);
  cinfo->setNoChromaQpOffsetConstraintFlag(m_noChromaQpOffsetConstraintFlag);
  cinfo->setNoQtbttDualTreeIntraConstraintFlag(m_noQtbttDualTreeIntraConstraintFlag);
  cinfo->setNoPartitionConstraintsOverrideConstraintFlag(m_noPartitionConstraintsOverrideConstraintFlag);
  cinfo->setNoSaoConstraintFlag(m_noSaoConstraintFlag);
  cinfo->setNoAlfConstraintFlag(m_noAlfConstraintFlag);
  cinfo->setNoCCAlfConstraintFlag(m_noCCAlfConstraintFlag);
  cinfo->setNoWeightedPredictionConstraintFlag(m_noWeightedPredictionConstraintFlag);
  cinfo->setNoRefWraparoundConstraintFlag(m_noRefWraparoundConstraintFlag);
  cinfo->setNoTemporalMvpConstraintFlag(m_noTemporalMvpConstraintFlag);
  cinfo->setNoSbtmvpConstraintFlag(m_noSbtmvpConstraintFlag);
  cinfo->setNoAmvrConstraintFlag(m_noAmvrConstraintFlag);
  cinfo->setNoBdofConstraintFlag(m_noBdofConstraintFlag);
  cinfo->setNoDmvrConstraintFlag(m_noDmvrConstraintFlag);
  cinfo->setNoCclmConstraintFlag(m_noCclmConstraintFlag);
  cinfo->setNoMtsConstraintFlag(m_noMtsConstraintFlag);
  cinfo->setNoSbtConstraintFlag(m_noSbtConstraintFlag);
  cinfo->setNoAffineMotionConstraintFlag(m_noAffineMotionConstraintFlag);
  cinfo->setNoBcwConstraintFlag(m_noBcwConstraintFlag);
  cinfo->setNoIbcConstraintFlag(m_noIbcConstraintFlag);
  cinfo->setNoCiipConstraintFlag(m_noCiipConstraintFlag);
  cinfo->setNoGeoConstraintFlag(m_noGeoConstraintFlag);
  cinfo->setNoLadfConstraintFlag(m_noLadfConstraintFlag);
  cinfo->setNoTransformSkipConstraintFlag(m_noTransformSkipConstraintFlag);
  cinfo->setNoBDPCMConstraintFlag(m_noBDPCMConstraintFlag);
  cinfo->setNoJointCbCrConstraintFlag(m_noJointCbCrConstraintFlag);
  cinfo->setNoCuQpDeltaConstraintFlag(m_noCuQpDeltaConstraintFlag);
  cinfo->setNoDepQuantConstraintFlag(m_noDepQuantConstraintFlag);
  cinfo->setNoSignDataHidingConstraintFlag(m_noSignDataHidingConstraintFlag);
  cinfo->setNoTrailConstraintFlag(m_noTrailConstraintFlag);
  cinfo->setNoStsaConstraintFlag(m_noStsaConstraintFlag);
  cinfo->setNoRaslConstraintFlag(m_noRaslConstraintFlag);
  cinfo->setNoRadlConstraintFlag(m_noRadlConstraintFlag);
  cinfo->setNoIdrConstraintFlag(m_noIdrConstraintFlag);
  cinfo->setNoCraConstraintFlag(m_noCraConstraintFlag);
  cinfo->setNoGdrConstraintFlag(m_noGdrConstraintFlag);
  cinfo->setNoApsConstraintFlag(m_noApsConstraintFlag);

  profileTierLevel->setLevelIdc                    (m_level);
  profileTierLevel->setTierFlag                    (m_levelTier);
  profileTierLevel->setProfileIdc                  (m_profile);
  profileTierLevel->setFrameOnlyConstraintFlag     (m_frameOnlyConstraintFlag);
  profileTierLevel->setMultiLayerEnabledFlag       (m_multiLayerEnabledFlag);
  profileTierLevel->setNumSubProfile(m_numSubProfile);
  for (int k = 0; k < m_numSubProfile; k++)
  {
    profileTierLevel->setSubProfileIdc(k, m_subProfile[k]);
  }
  /* XXX: should Main be marked as compatible with still picture? */
  /* XXX: may be a good idea to refactor the above into a function
   * that chooses the actual compatibility based upon options */
  sps.setVPSId( m_vps->getVPSId() );
  sps.setMaxPicWidthInLumaSamples( m_iSourceWidth );
  sps.setMaxPicHeightInLumaSamples( m_iSourceHeight );
  if (m_resChangeInClvsEnabled)
  {
    int maxPicWidth = std::max(m_iSourceWidth, (int)((double)m_iSourceWidth / m_scalingRatioHor + 0.5));
    int maxPicHeight = std::max(m_iSourceHeight, (int)((double)m_iSourceHeight / m_scalingRatioVer + 0.5));
    const int minCuSize = std::max(8, 1 << m_log2MinCUSize);
    if (maxPicWidth % minCuSize)
    {
      maxPicWidth += ((maxPicWidth / minCuSize) + 1) * minCuSize - maxPicWidth;
    }
    if (maxPicHeight % minCuSize)
    {
      maxPicHeight += ((maxPicHeight / minCuSize) + 1) * minCuSize - maxPicHeight;
    }
    sps.setMaxPicWidthInLumaSamples( maxPicWidth );
    sps.setMaxPicHeightInLumaSamples( maxPicHeight );
  }
  sps.setConformanceWindow( m_conformanceWindow );

  sps.setMaxCUWidth             ( m_maxCUWidth        );
  sps.setMaxCUHeight            ( m_maxCUHeight       );
  sps.setLog2MinCodingBlockSize ( m_log2MinCUSize );
  sps.setChromaFormatIdc        ( m_chromaFormatIDC   );

  sps.setCTUSize                             ( m_CTUSize );
  sps.setSplitConsOverrideEnabledFlag        ( m_useSplitConsOverride );
  sps.setMinQTSizes                          ( m_uiMinQT );
  sps.setMaxMTTHierarchyDepth                ( m_uiMaxMTTHierarchyDepth, m_uiMaxMTTHierarchyDepthI, m_uiMaxMTTHierarchyDepthIChroma );
  sps.setMaxBTSize( m_uiMaxBT[1], m_uiMaxBT[0], m_uiMaxBT[2] );
  sps.setMaxTTSize( m_uiMaxTT[1], m_uiMaxTT[0], m_uiMaxTT[2] );
  sps.setIDRRefParamListPresent              ( m_idrRefParamList );
  sps.setUseDualITree                        ( m_dualITree );
  sps.setUseLFNST                            ( m_LFNST );
  sps.setSbTMVPEnabledFlag(m_sbTmvpEnableFlag);
  sps.setAMVREnabledFlag                ( m_ImvMode != IMV_OFF );
  sps.setBDOFEnabledFlag                    ( m_BIO );
  sps.setMaxNumMergeCand(getMaxNumMergeCand());
  sps.setMaxNumAffineMergeCand(getMaxNumAffineMergeCand());
  sps.setMaxNumIBCMergeCand(getMaxNumIBCMergeCand());
  sps.setMaxNumGeoCand(getMaxNumGeoCand());
  sps.setUseAffine             ( m_Affine );
  sps.setUseAffineType         ( m_AffineType );
  sps.setUsePROF               ( m_PROF );
  sps.setUseLMChroma           ( m_LMChroma ? true : false );
  sps.setHorCollocatedChromaFlag( m_horCollocatedChromaFlag );
  sps.setVerCollocatedChromaFlag( m_verCollocatedChromaFlag );
  sps.setUseMTS                ( m_IntraMTS || m_InterMTS || m_ImplicitMTS );
  sps.setUseIntraMTS           ( m_IntraMTS );
  sps.setUseInterMTS           ( m_InterMTS );
  sps.setUseSBT                             ( m_SBT );
  sps.setUseSMVD                ( m_SMVD );
  sps.setUseBcw                ( m_bcw );
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  sps.setLadfEnabled           ( m_LadfEnabled );
  if ( m_LadfEnabled )
  {
    sps.setLadfNumIntervals    ( m_LadfNumIntervals );
    for ( int k = 0; k < m_LadfNumIntervals; k++ )
    {
      sps.setLadfQpOffset( m_LadfQpOffset[k], k );
      sps.setLadfIntervalLowerBound( m_LadfIntervalLowerBound[k], k );
    }
    CHECK_( m_LadfIntervalLowerBound[0] != 0, "abnormal value set to LadfIntervalLowerBound[0]" );
  }
#endif

  sps.setUseCiip            ( m_ciip );
  sps.setUseGeo                ( m_Geo );
  sps.setUseMMVD               ( m_MMVD );
  sps.setFpelMmvdEnabledFlag   (( m_MMVD ) ? m_allowDisFracMMVD : false);
  sps.setBdofControlPresentInPhFlag(m_BIO);
  sps.setDmvrControlPresentInPhFlag(m_DMVR);
  sps.setProfControlPresentInPhFlag(m_PROF);
  sps.setAffineAmvrEnabledFlag              ( m_AffineAmvr );
  sps.setUseDMVR                            ( m_DMVR );
  sps.setUseColorTrans(m_useColorTrans);
  sps.setPLTMode                            ( m_PLTMode);
  sps.setIBCFlag                            ( m_IBCMode);
  sps.setWrapAroundEnabledFlag                      ( m_wrapAround );
  // ADD_NEW_TOOL : (encoder lib) set tool enabling flags and associated parameters here
  sps.setUseISP                             ( m_ISP );
  sps.setUseLmcs                            ( m_lmcsEnabled );
  sps.setUseMRL                ( m_MRL );
  sps.setUseMIP                ( m_MIP );
  CHECK_(m_log2MinCUSize > std::min(6, floorLog2(sps.getMaxCUWidth())), "sps_log2_min_luma_coding_block_size_minus2 shall be in the range of 0 to min (4, log2_ctu_size - 2)");
  CHECK_(m_uiMaxMTTHierarchyDepth > 2 * (floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize()), "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  CHECK_(m_uiMaxMTTHierarchyDepthI > 2 * (floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize()), "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  CHECK_(m_uiMaxMTTHierarchyDepthIChroma > 2 * (floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize()), "sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  sps.setTransformSkipEnabledFlag(m_useTransformSkip);
  sps.setLog2MaxTransformSkipBlockSize(m_log2MaxTransformSkipBlockSize);
  sps.setBDPCMEnabledFlag(m_useBDPCM);

  sps.setSPSTemporalMVPEnabledFlag((getTMVPModeId() == 2 || getTMVPModeId() == 1));

  sps.setLog2MaxTbSize   ( m_log2MaxTbSize );

  for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    sps.setBitDepth      (ChannelType(channelType), m_bitDepth[channelType] );
    sps.setQpBDOffset  (ChannelType(channelType), (6 * (m_bitDepth[channelType] - 8)));
    sps.setInternalMinusInputBitDepth(ChannelType(channelType), max(0, (m_bitDepth[channelType] - m_inputBitDepth[channelType])));
  }

  sps.setEntropyCodingSyncEnabledFlag( m_entropyCodingSyncEnabledFlag );
  sps.setEntryPointsPresentFlag( m_entryPointPresentFlag );

  sps.setUseWP( m_useWeightedPred );
  sps.setUseWPBiPred( m_useWeightedBiPred );

  sps.setSAOEnabledFlag( m_bUseSAO );
  sps.setJointCbCrEnabledFlag( m_JointCbCrMode );
  sps.setMaxTLayers( m_maxTempLayer );
  sps.setTemporalIdNestingFlag( ( m_maxTempLayer == 1 ) ? true : false );

  for (int i = 0; i < std::min(sps.getMaxTLayers(), (uint32_t) MAX_TLAYER); i++ )
  {
    sps.setMaxDecPicBuffering(m_maxDecPicBuffering[i], i);
    sps.setMaxNumReorderPics(m_maxNumReorderPics[i], i);
  }

  sps.setScalingListFlag ( (m_useScalingListId == SCALING_LIST_OFF) ? 0 : 1 );
  if (sps.getUseColorTrans() && sps.getScalingListFlag())
  {
    sps.setScalingMatrixForAlternativeColourSpaceDisabledFlag( m_disableScalingMatrixForAlternativeColourSpace );
  }
  else
  {
    sps.setScalingMatrixForAlternativeColourSpaceDisabledFlag( false );
  }
  if (sps.getScalingMatrixForAlternativeColourSpaceDisabledFlag())
  {
    sps.setScalingMatrixDesignatedColourSpaceFlag( m_scalingMatrixDesignatedColourSpace );
  }
  else
  {
    sps.setScalingMatrixDesignatedColourSpaceFlag( true );
  }
  sps.setALFEnabledFlag( m_alf );
  sps.setCCALFEnabledFlag( m_ccalf );
  sps.setFieldSeqFlag(false);
  sps.setVuiParametersPresentFlag(getVuiParametersPresentFlag());

  if (sps.getVuiParametersPresentFlag())
  {
    VUI* pcVUI = sps.getVuiParameters();
    pcVUI->setAspectRatioInfoPresentFlag(getAspectRatioInfoPresentFlag());
    pcVUI->setAspectRatioConstantFlag(!getSampleAspectRatioInfoSEIEnabled());
    pcVUI->setAspectRatioIdc(getAspectRatioIdc());
    pcVUI->setSarWidth(getSarWidth());
    pcVUI->setSarHeight(getSarHeight());
    pcVUI->setColourDescriptionPresentFlag(getColourDescriptionPresentFlag());
    pcVUI->setColourPrimaries(getColourPrimaries());
    pcVUI->setTransferCharacteristics(getTransferCharacteristics());
    pcVUI->setMatrixCoefficients(getMatrixCoefficients());
    pcVUI->setProgressiveSourceFlag       (getProgressiveSourceFlag());
    pcVUI->setInterlacedSourceFlag        (getInterlacedSourceFlag());
    pcVUI->setNonPackedFlag               (getNonPackedConstraintFlag());
    pcVUI->setNonProjectedFlag            (getNonProjectedConstraintFlag());
    pcVUI->setChromaLocInfoPresentFlag(getChromaLocInfoPresentFlag());
    pcVUI->setChromaSampleLocTypeTopField(getChromaSampleLocTypeTopField());
    pcVUI->setChromaSampleLocTypeBottomField(getChromaSampleLocTypeBottomField());
    pcVUI->setChromaSampleLocType(getChromaSampleLocType());
    pcVUI->setOverscanInfoPresentFlag(getOverscanInfoPresentFlag());
    pcVUI->setOverscanAppropriateFlag(getOverscanAppropriateFlag());
    pcVUI->setVideoFullRangeFlag(getVideoFullRangeFlag());
  }

  sps.setNumLongTermRefPicSPS(NUM_LONG_TERM_REF_PIC_SPS);
  CHECK_(!(NUM_LONG_TERM_REF_PIC_SPS <= MAX_NUM_LONG_TERM_REF_PICS), "Unspecified error");
  for (int k = 0; k < NUM_LONG_TERM_REF_PIC_SPS; k++)
  {
    sps.setLtRefPicPocLsbSps(k, 0);
    sps.setUsedByCurrPicLtSPSFlag(k, 0);
  }
  int numQpTables = m_chromaQpMappingTableParams.getSameCQPTableForAllChromaFlag() ? 1 : (sps.getJointCbCrEnabledFlag() ? 3 : 2);
  m_chromaQpMappingTableParams.setNumQpTables(numQpTables);
  sps.setChromaQpMappingTableFromParams(m_chromaQpMappingTableParams, sps.getQpBDOffset(CHANNEL_TYPE_CHROMA));
  sps.derivedChromaQPMappingTables();

#if U0132_TARGET_BITS_SATURATION
  if( getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() || getCpbSaturationEnabled() )
#else
  if( getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() )
#endif
  {
    xInitHrdParameters(sps);
  }
  if( getBufferingPeriodSEIEnabled() || getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() )
  {
    sps.setGeneralHrdParametersPresentFlag(true);
  }

  // Set up SPS range extension settings
  sps.getSpsRangeExtension().setTransformSkipRotationEnabledFlag(m_transformSkipRotationEnabledFlag);
  sps.getSpsRangeExtension().setTransformSkipContextEnabledFlag(m_transformSkipContextEnabledFlag);
  sps.getSpsRangeExtension().setExtendedPrecisionProcessingFlag(m_extendedPrecisionProcessingFlag);
  sps.getSpsRangeExtension().setIntraSmoothingDisabledFlag( m_intraSmoothingDisabledFlag );
  sps.getSpsRangeExtension().setHighPrecisionOffsetsEnabledFlag(m_highPrecisionOffsetsEnabledFlag);
  sps.getSpsRangeExtension().setPersistentRiceAdaptationEnabledFlag(m_persistentRiceAdaptationEnabledFlag);
  sps.getSpsRangeExtension().setCabacBypassAlignmentEnabledFlag(m_cabacBypassAlignmentEnabledFlag);

  sps.setSubPicInfoPresentFlag(m_subPicInfoPresentFlag);
  if (m_subPicInfoPresentFlag)
  {
    sps.setNumSubPics(m_numSubPics);
    sps.setSubPicSameSizeFlag(m_subPicSameSizeFlag);
    if (m_subPicSameSizeFlag)
    {
      uint32_t numSubpicCols = (m_iSourceWidth + m_CTUSize - 1) / m_CTUSize / m_subPicWidth[0];
      for (unsigned int i = 0; i < m_numSubPics; i++)
      {
        sps.setSubPicCtuTopLeftX(i, (i % numSubpicCols) * m_subPicWidth[0]);
        sps.setSubPicCtuTopLeftY(i, (i / numSubpicCols) * m_subPicHeight[0]);
        sps.setSubPicWidth(i, m_subPicWidth[0]);
        sps.setSubPicHeight(i, m_subPicHeight[0]);
      }
    }
    else
    {
      sps.setSubPicCtuTopLeftX(m_subPicCtuTopLeftX);
      sps.setSubPicCtuTopLeftY(m_subPicCtuTopLeftY);
      sps.setSubPicWidth(m_subPicWidth);
      sps.setSubPicHeight(m_subPicHeight);
    }
    sps.setSubPicTreatedAsPicFlag(m_subPicTreatedAsPicFlag);
    sps.setLoopFilterAcrossSubpicEnabledFlag(m_loopFilterAcrossSubpicEnabledFlag);
    sps.setSubPicIdLen(m_subPicIdLen);
    sps.setSubPicIdMappingExplicitlySignalledFlag(m_subPicIdMappingExplicitlySignalledFlag);
    if (m_subPicIdMappingExplicitlySignalledFlag)
    {
      sps.setSubPicIdMappingPresentFlag(m_subPicIdMappingInSpsFlag);
      if (m_subPicIdMappingInSpsFlag)
      {
        sps.setSubPicId(m_subPicId);
      }
    }
  }
  else   //In that case, there is only one subpicture that contains the whole picture
  {
    sps.setNumSubPics(1);
    sps.setSubPicCtuTopLeftX(0, 0);
    sps.setSubPicCtuTopLeftY(0, 0);
    sps.setSubPicWidth(0, m_iSourceWidth);
    sps.setSubPicHeight(0, m_iSourceHeight);
    sps.setSubPicTreatedAsPicFlag(0, 1);
    sps.setLoopFilterAcrossSubpicEnabledFlag(0, 0);
    sps.setSubPicIdLen(0);
    sps.setSubPicIdMappingExplicitlySignalledFlag(false);
  }
  sps.setDepQuantEnabledFlag( m_DepQuantEnabledFlag );
  if (!sps.getDepQuantEnabledFlag())
  {
    sps.setSignDataHidingEnabledFlag( m_SignDataHidingEnabledFlag );
  }
  else
  {
    sps.setSignDataHidingEnabledFlag(false);
  }
  sps.setVirtualBoundariesEnabledFlag( m_virtualBoundariesEnabledFlag );
  if( sps.getVirtualBoundariesEnabledFlag() )
  {
    sps.setVirtualBoundariesPresentFlag( m_virtualBoundariesPresentFlag );
    CHECK_( sps.getSubPicInfoPresentFlag() && sps.getVirtualBoundariesPresentFlag() != 1, "When subpicture signalling if present, the signalling of virtual boundaries, is present, shall be in the SPS" );
    sps.setNumVerVirtualBoundaries            ( m_numVerVirtualBoundaries );
    sps.setNumHorVirtualBoundaries            ( m_numHorVirtualBoundaries );
    for( unsigned int i = 0; i < m_numVerVirtualBoundaries; i++ )
    {
      sps.setVirtualBoundariesPosX            ( m_virtualBoundariesPosX[i], i );
    }
    for( unsigned int i = 0; i < m_numHorVirtualBoundaries; i++ )
    {
      sps.setVirtualBoundariesPosY            ( m_virtualBoundariesPosY[i], i );
    }
  }

  sps.setInterLayerPresentFlag( m_layerId > 0 && m_vps->getMaxLayers() > 1 && !m_vps->getAllIndependentLayersFlag() && !m_vps->getIndependentLayerFlag( m_vps->getGeneralLayerIdx( m_layerId ) ) );
  CHECK_( m_vps->getIndependentLayerFlag( m_vps->getGeneralLayerIdx( m_layerId ) ) && sps.getInterLayerPresentFlag(), " When vps_independent_layer_flag[GeneralLayerIdx[nuh_layer_id ]]  is equal to 1, the value of inter_layer_ref_pics_present_flag shall be equal to 0." );

  sps.setResChangeInClvsEnabledFlag(m_resChangeInClvsEnabled);
  sps.setRprEnabledFlag(m_rprEnabledFlag);

  sps.setLog2ParallelMergeLevelMinus2( m_log2ParallelMergeLevelMinus2 );

  CHECK_(sps.getResChangeInClvsEnabledFlag() && sps.getVirtualBoundariesEnabledFlag(), "when the value of sps_res_change_in_clvs_allowed_flag is equal to 1, the value of sps_virtual_boundaries_present_flag shall be equal to 0");
}

void EncLib::xInitHrdParameters(SPS &sps)
{
  m_encHRD.initHRDParameters((EncCfg*) this);

  GeneralHrdParams *generalHrdParams = sps.getGeneralHrdParameters();
  *generalHrdParams = m_encHRD.getGeneralHrdParameters();

  OlsHrdParams *spsOlsHrdParams = sps.getOlsHrdParameters();
  for(int i = 0; i < MAX_TLAYER; i++)
  {
    *spsOlsHrdParams = m_encHRD.getOlsHrdParameters(i);
    spsOlsHrdParams++;
  }
}

void EncLib::xInitPPS(PPS &pps, const SPS &sps)
{
  // pps ID already initialised.
  pps.setSPSId(sps.getSPSId());

  pps.setNumSubPics(sps.getNumSubPics());
  pps.setSubPicIdMappingInPpsFlag(false);
  pps.setSubPicIdLen(sps.getSubPicIdLen());
  for(int picIdx=0; picIdx<pps.getNumSubPics(); picIdx++)
  {
    pps.setSubPicId(picIdx, sps.getSubPicId(picIdx));
  }
  bool bUseDQP = (getCuQpDeltaSubdiv() > 0)? true : false;

  if((getMaxDeltaQP() != 0 )|| getUseAdaptiveQP())
  {
    bUseDQP = true;
  }

#if SHARP_LUMA_DELTA_QP
  if ( getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    bUseDQP = true;
  }
#endif
#if ENABLE_QPA
  if (getUsePerceptQPA() && !bUseDQP)
  {
    CHECK_( m_cuQpDeltaSubdiv != 0, "max. delta-QP subdiv must be zero!" );
    bUseDQP = (getBaseQP() < 38) && (getSourceWidth() > 512 || getSourceHeight() > 320);
  }
#endif

  if (m_costMode==COST_SEQUENCE_LEVEL_LOSSLESS || m_costMode==COST_LOSSLESS_CODING)
  {
    bUseDQP=false;
  }


  if ( m_RCEnableRateControl )
  {
    pps.setUseDQP(true);
  }
  else if(bUseDQP)
  {
    pps.setUseDQP(true);
  }
  else
  {
    pps.setUseDQP(false);
  }

  if ( m_cuChromaQpOffsetSubdiv >= 0 )
  {
    pps.clearChromaQpOffsetList();
    pps.setChromaQpOffsetListEntry(1, 6, 6, 6);
    /* todo, insert table entries from command line (NB, 0 should not be touched) */
  }
  else
  {
    pps.clearChromaQpOffsetList();
  }
  {
    int baseQp = 26;
    if( 16 == getGOPSize() )
    {
      baseQp = getBaseQP()-24;
    }
    else
    {
      baseQp = getBaseQP()-26;
    }
    const int maxDQP = 37;
    const int minDQP = -26 + sps.getQpBDOffset(CHANNEL_TYPE_LUMA);

    pps.setPicInitQPMinus26( std::min( maxDQP, std::max( minDQP, baseQp ) ));
  }

  if( sps.getJointCbCrEnabledFlag() == false || getChromaFormatIdc() == CHROMA_400 || m_chromaCbCrQpOffset == 0 )
  {
    pps.setJointCbCrQpOffsetPresentFlag(false);
  }
  else
  {
    pps.setJointCbCrQpOffsetPresentFlag(true);
  }

#if ER_CHROMA_QP_WCG_PPS
  if (getWCGChromaQPControl().isEnabled())
  {
    const int baseQp=m_iQP+pps.getPPSId();
    const double chromaQp = m_wcgChromaQpControl.chromaQpScale * baseQp + m_wcgChromaQpControl.chromaQpOffset;
    const double dcbQP = m_wcgChromaQpControl.chromaCbQpScale * chromaQp;
    const double dcrQP = m_wcgChromaQpControl.chromaCrQpScale * chromaQp;
    const int cbQP =(int)(dcbQP + ( dcbQP < 0 ? -0.5 : 0.5) );
    const int crQP =(int)(dcrQP + ( dcrQP < 0 ? -0.5 : 0.5) );
    pps.setQpOffset(COMPONENT_Cb, Clip3( -12, 12, min(0, cbQP) + m_chromaCbQpOffset ));
    pps.setQpOffset(COMPONENT_Cr, Clip3( -12, 12, min(0, crQP) + m_chromaCrQpOffset));
    if(pps.getJointCbCrQpOffsetPresentFlag())
      pps.setQpOffset(JOINT_CbCr, Clip3(-12, 12, (min(0, cbQP) + min(0, crQP)) / 2 + m_chromaCbCrQpOffset));
    else
      pps.setQpOffset(JOINT_CbCr, 0);
  }
  else
  {
#endif
  pps.setQpOffset(COMPONENT_Cb, m_chromaCbQpOffset );
  pps.setQpOffset(COMPONENT_Cr, m_chromaCrQpOffset );
  if (pps.getJointCbCrQpOffsetPresentFlag())
    pps.setQpOffset(JOINT_CbCr, m_chromaCbCrQpOffset);
  else
    pps.setQpOffset(JOINT_CbCr, 0);
#if ER_CHROMA_QP_WCG_PPS
  }
#endif
#if W0038_CQP_ADJ
  bool bChromaDeltaQPEnabled = false;
  {
    bChromaDeltaQPEnabled = ( m_sliceChromaQpOffsetIntraOrPeriodic[0] || m_sliceChromaQpOffsetIntraOrPeriodic[1] );
    if( !bChromaDeltaQPEnabled )
    {
      for( int i=0; i<m_iGOPSize; i++ )
      {
        if( m_GOPList[i].m_CbQPoffset || m_GOPList[i].m_CrQPoffset )
        {
          bChromaDeltaQPEnabled = true;
          break;
        }
      }
    }
  }
 #if ENABLE_QPA
  if ((getUsePerceptQPA() || getSliceChromaOffsetQpPeriodicity() > 0) && (getChromaFormatIdc() != CHROMA_400))
  {
    bChromaDeltaQPEnabled = true;
  }
 #endif
  pps.setSliceChromaQpFlag(bChromaDeltaQPEnabled);
#endif
  if (
    !pps.getSliceChromaQpFlag() && sps.getUseDualITree()
    && (getChromaFormatIdc() != CHROMA_400))
  {
    pps.setSliceChromaQpFlag(m_chromaCbQpOffsetDualTree != 0 || m_chromaCrQpOffsetDualTree != 0 || m_chromaCbCrQpOffsetDualTree != 0);
  }

  int minCbSizeY = (1 << sps.getLog2MinCodingBlockSize());
  pps.setWrapAroundEnabledFlag                ( m_wrapAround );
  if( m_wrapAround )
  {
    pps.setPicWidthMinusWrapAroundOffset      ((pps.getPicWidthInLumaSamples()/minCbSizeY) - (m_wrapAroundOffset / minCbSizeY));
    pps.setWrapAroundOffset                   (minCbSizeY *(pps.getPicWidthInLumaSamples() / minCbSizeY- pps.getPicWidthMinusWrapAroundOffset()));
  }
  else 
  {
    pps.setPicWidthMinusWrapAroundOffset      ( 0 );
    pps.setWrapAroundOffset                   ( 0 );       
  }
  CHECK_( !sps.getWrapAroundEnabledFlag() && pps.getWrapAroundEnabledFlag(), "When sps_ref_wraparound_enabled_flag is equal to 0, the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");
  CHECK_( (((sps.getCTUSize() / minCbSizeY) + 1) > ((pps.getPicWidthInLumaSamples() / minCbSizeY) - 1)) && pps.getWrapAroundEnabledFlag(), "When the value of CtbSizeY / MinCbSizeY + 1 is greater than pps_pic_width_in_luma_samples / MinCbSizeY - 1, the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");

  pps.setNoPicPartitionFlag( m_noPicPartitionFlag );
  if( m_noPicPartitionFlag == false )
  {
    pps.setLog2CtuSize( ceilLog2( sps.getCTUSize()) );
    pps.setNumExpTileColumns( (uint32_t) m_tileColumnWidth.size() );
    pps.setNumExpTileRows( (uint32_t) m_tileRowHeight.size() );
    pps.setTileColumnWidths( m_tileColumnWidth );
    pps.setTileRowHeights( m_tileRowHeight );
    pps.initTiles();
    pps.setRectSliceFlag( m_rectSliceFlag );
    if( m_rectSliceFlag )
    {
      pps.setSingleSlicePerSubPicFlag(m_singleSlicePerSubPicFlag);
      pps.setNumSlicesInPic( m_numSlicesInPic );
      pps.setTileIdxDeltaPresentFlag( m_tileIdxDeltaPresentFlag );
      pps.setRectSlices( m_rectSlices );
      pps.initRectSliceMap(&sps);
    }
    else
    {
      pps.initRasterSliceMap( m_rasterSliceSize );
    }
    pps.initSubPic(sps);
    pps.setLoopFilterAcrossTilesEnabledFlag( m_bLFCrossTileBoundaryFlag );
    pps.setLoopFilterAcrossSlicesEnabledFlag( m_bLFCrossSliceBoundaryFlag );
  }
  else
  {
    pps.setLog2CtuSize( ceilLog2( sps.getCTUSize()) );
    pps.setNumExpTileColumns(1);
    pps.setNumExpTileRows(1);
    pps.addTileColumnWidth( pps.getPicWidthInCtu( ) );
    pps.addTileRowHeight( pps.getPicHeightInCtu( ) );
    pps.initTiles();
    pps.setRectSliceFlag( 1 );
    pps.setNumSlicesInPic( 1 );
    pps.initRectSlices( );
    pps.setTileIdxDeltaPresentFlag( 0 );
    pps.setSliceTileIdx( 0, 0 );
    pps.initRectSliceMap( &sps );
    pps.initSubPic(sps);
    pps.setLoopFilterAcrossTilesEnabledFlag( true );
    pps.setLoopFilterAcrossSlicesEnabledFlag( true );
  }

  pps.setUseWP( m_useWeightedPred );
  pps.setWPBiPred( m_useWeightedBiPred );
  pps.setOutputFlagPresentFlag( false );

  if ( getDeblockingFilterMetric() )
  {
    pps.setDeblockingFilterOverrideEnabledFlag(true);
    pps.setPPSDeblockingFilterDisabledFlag(false);
  }
  else
  {
    pps.setDeblockingFilterOverrideEnabledFlag( !getLoopFilterOffsetInPPS() );
    pps.setPPSDeblockingFilterDisabledFlag( getLoopFilterDisable() );
  }

  if (! pps.getPPSDeblockingFilterDisabledFlag())
  {
    pps.setDeblockingFilterBetaOffsetDiv2( getLoopFilterBetaOffset() );
    pps.setDeblockingFilterTcOffsetDiv2( getLoopFilterTcOffset() );
    pps.setDeblockingFilterCbBetaOffsetDiv2( getLoopFilterCbBetaOffset() );
    pps.setDeblockingFilterCbTcOffsetDiv2( getLoopFilterCbTcOffset() );
    pps.setDeblockingFilterCrBetaOffsetDiv2( getLoopFilterCrBetaOffset() );
    pps.setDeblockingFilterCrTcOffsetDiv2( getLoopFilterCrTcOffset() );
  }
  else
  {
    pps.setDeblockingFilterBetaOffsetDiv2(0);
    pps.setDeblockingFilterTcOffsetDiv2(0);
    pps.setDeblockingFilterCbBetaOffsetDiv2(0);
    pps.setDeblockingFilterCbTcOffsetDiv2(0);
    pps.setDeblockingFilterCrBetaOffsetDiv2(0);
    pps.setDeblockingFilterCrTcOffsetDiv2(0);
  }

  // deblockingFilterControlPresentFlag is true if any of the settings differ from the inferred values:
  const bool deblockingFilterControlPresentFlag = pps.getDeblockingFilterOverrideEnabledFlag()   ||
                                                  pps.getPPSDeblockingFilterDisabledFlag()       ||
                                                  pps.getDeblockingFilterBetaOffsetDiv2() != 0   ||
                                                  pps.getDeblockingFilterTcOffsetDiv2() != 0     ||
                                                  pps.getDeblockingFilterCbBetaOffsetDiv2() != 0 ||
                                                  pps.getDeblockingFilterCbTcOffsetDiv2() != 0   ||
                                                  pps.getDeblockingFilterCrBetaOffsetDiv2() != 0 ||
                                                  pps.getDeblockingFilterCrTcOffsetDiv2() != 0;

  pps.setDeblockingFilterControlPresentFlag(deblockingFilterControlPresentFlag);

  pps.setCabacInitPresentFlag(CABAC_INIT_PRESENT_FLAG);
  pps.setLoopFilterAcrossSlicesEnabledFlag( m_bLFCrossSliceBoundaryFlag );

  bool chromaQPOffsetNotZero = false;
  if( pps.getQpOffset(COMPONENT_Cb) != 0 || pps.getQpOffset(COMPONENT_Cr) != 0 || pps.getJointCbCrQpOffsetPresentFlag() || pps.getSliceChromaQpFlag() || pps.getCuChromaQpOffsetListEnabledFlag() )
  {
    chromaQPOffsetNotZero = true;
  }
  bool chromaDbfOffsetNotSameAsLuma = true;
  if( pps.getDeblockingFilterCbBetaOffsetDiv2() == pps.getDeblockingFilterBetaOffsetDiv2() && pps.getDeblockingFilterCrBetaOffsetDiv2() == pps.getDeblockingFilterBetaOffsetDiv2()
     && pps.getDeblockingFilterCbTcOffsetDiv2() == pps.getDeblockingFilterTcOffsetDiv2() && pps.getDeblockingFilterCrTcOffsetDiv2() == pps.getDeblockingFilterTcOffsetDiv2() )
  {
    chromaDbfOffsetNotSameAsLuma = false;
  }
  if ((sps.getChromaFormatIdc() != CHROMA_400) && (chromaQPOffsetNotZero || chromaDbfOffsetNotSameAsLuma))
  {
    pps.setPPSChromaToolFlag(true);
  }
  else
  {
    pps.setPPSChromaToolFlag(false);
  }

  int histogram[MAX_NUM_REF + 1];
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    histogram[i]=0;
  }
  for( int i = 0; i < getGOPSize(); i++)
  {
    CHECK_(!(getRPLEntry(0, i).m_numRefPicsActive >= 0 && getRPLEntry(0, i).m_numRefPicsActive <= MAX_NUM_REF), "Unspecified error");
    histogram[getRPLEntry(0, i).m_numRefPicsActive]++;
  }

  int maxHist=-1;
  int bestPos=0;
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    if(histogram[i]>maxHist)
    {
      maxHist=histogram[i];
      bestPos=i;
    }
  }
  CHECK_(!(bestPos <= 15), "Unspecified error");
    pps.setNumRefIdxL0DefaultActive(bestPos);
  pps.setNumRefIdxL1DefaultActive(bestPos);
  pps.setPictureHeaderExtensionPresentFlag(false);

  pps.setRplInfoInPhFlag(getSliceLevelRpl() ? false : true);
  pps.setDbfInfoInPhFlag(getSliceLevelDblk() ? false : true);
  pps.setSaoInfoInPhFlag(getSliceLevelSao() ? false : true);
  pps.setAlfInfoInPhFlag(getSliceLevelAlf() ? false : true);
  pps.setWpInfoInPhFlag(getSliceLevelWp() ? false : true);
  pps.setQpDeltaInfoInPhFlag(getSliceLevelDeltaQp() ? false : true);

  pps.pcv = new PreCalcValues( sps, pps, true );
  pps.setRpl1IdxPresentFlag(sps.getRPL1IdxPresentFlag());
}

void EncLib::xInitPicHeader(PicHeader &picHeader, const SPS &sps, const PPS &pps)
{
  int i;
  picHeader.initPicHeader();

  // parameter sets
  picHeader.setSPSId( sps.getSPSId() );
  picHeader.setPPSId( pps.getPPSId() );

  // merge list sizes
  picHeader.setMaxNumAffineMergeCand(getMaxNumAffineMergeCand());
  // copy partitioning constraints from SPS
  picHeader.setSplitConsOverrideFlag(false);
  picHeader.setMinQTSizes( sps.getMinQTSizes() );
  picHeader.setMaxMTTHierarchyDepths( sps.getMaxMTTHierarchyDepths() );
  picHeader.setMaxBTSizes( sps.getMaxBTSizes() );
  picHeader.setMaxTTSizes( sps.getMaxTTSizes() );

  bool bUseDQP = (getCuQpDeltaSubdiv() > 0)? true : false;

  if( (getMaxDeltaQP() != 0 )|| getUseAdaptiveQP() )
  {
    bUseDQP = true;
  }

#if SHARP_LUMA_DELTA_QP
  if( getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    bUseDQP = true;
  }
#endif
#if ENABLE_QPA
  if( getUsePerceptQPA() && !bUseDQP )
  {
    CHECK_( m_cuQpDeltaSubdiv != 0, "max. delta-QP subdiv must be zero!" );
    bUseDQP = (getBaseQP() < 38) && (getSourceWidth() > 512 || getSourceHeight() > 320);
  }
#endif

  if( m_costMode==COST_SEQUENCE_LEVEL_LOSSLESS || m_costMode==COST_LOSSLESS_CODING )
  {
    bUseDQP=false;
  }

  if( m_RCEnableRateControl )
  {
    picHeader.setCuQpDeltaSubdivIntra( 0 );
    picHeader.setCuQpDeltaSubdivInter( 0 );
  }
  else if( bUseDQP )
  {
    picHeader.setCuQpDeltaSubdivIntra( m_cuQpDeltaSubdiv );
    picHeader.setCuQpDeltaSubdivInter( m_cuQpDeltaSubdiv );
  }
  else
  {
    picHeader.setCuQpDeltaSubdivIntra( 0 );
    picHeader.setCuQpDeltaSubdivInter( 0 );
  }

  if( m_cuChromaQpOffsetSubdiv >= 0 )
  {
    picHeader.setCuChromaQpOffsetSubdivIntra(m_cuChromaQpOffsetSubdiv);
    picHeader.setCuChromaQpOffsetSubdivInter(m_cuChromaQpOffsetSubdiv);
  }
  else
  {
    picHeader.setCuChromaQpOffsetSubdivIntra(0);
    picHeader.setCuChromaQpOffsetSubdivInter(0);
  }


  // virtual boundaries
  if( sps.getVirtualBoundariesEnabledFlag() )
  {
    picHeader.setVirtualBoundariesPresentFlag( sps.getVirtualBoundariesPresentFlag() );
    picHeader.setNumVerVirtualBoundaries(sps.getNumVerVirtualBoundaries());
    picHeader.setNumHorVirtualBoundaries(sps.getNumHorVirtualBoundaries());
    for(i=0; i<3; i++) {
      picHeader.setVirtualBoundariesPosX(sps.getVirtualBoundariesPosX(i), i);
      picHeader.setVirtualBoundariesPosY(sps.getVirtualBoundariesPosY(i), i);
    }
  }

  // gradual decoder refresh flag
  picHeader.setGdrPicFlag(false);

  // BDOF / DMVR / PROF
  picHeader.setBdofDisabledFlag(false);
  picHeader.setDmvrDisabledFlag(false);
  picHeader.setProfDisabledFlag(false);
}

void EncLib::xInitAPS(APS &aps)
{
  //Do nothing now
}

void EncLib::xInitRPL(SPS &sps, bool isFieldCoding)
{
  ReferencePictureList*      rpl;

  int numRPLCandidates = getRPLCandidateSize(0);
  // To allocate one additional memory for RPL of POC1 (first bottom field) which is not specified in cfg file
  sps.createRPLList0(numRPLCandidates + (isFieldCoding ? 1 : 0));
  sps.createRPLList1(numRPLCandidates + (isFieldCoding ? 1 : 0));
  RPLList* rplList = 0;

  for (int i = 0; i < 2; i++)
  {
    rplList = (i == 0) ? sps.getRPLList0() : sps.getRPLList1();
    for (int j = 0; j < numRPLCandidates; j++)
    {
      const RPLEntry &ge = getRPLEntry(i, j);
      rpl = rplList->getReferencePictureList(j);
      rpl->setNumberOfShorttermPictures(ge.m_numRefPics);
      rpl->setNumberOfLongtermPictures(0);   //Hardcoded as 0 for now. need to update this when implementing LTRP
      rpl->setNumberOfActivePictures(ge.m_numRefPicsActive);
      rpl->setLtrpInSliceHeaderFlag(ge.m_ltrp_in_slice_header_flag);
      rpl->setInterLayerPresentFlag( sps.getInterLayerPresentFlag() );
      // inter-layer reference picture is not signaled in SPS RPL, SPS is shared currently
      rpl->setNumberOfInterLayerPictures( 0 );

      for (int k = 0; k < ge.m_numRefPics; k++)
      {
        rpl->setRefPicIdentifier(k, -ge.m_deltaRefPics[k], 0, false, 0);
      }
    }
  }

  if (isFieldCoding)
  {
    // To set RPL of POC1 (first bottom field) which is not specified in cfg file
    for (int i = 0; i < 2; i++)
    {
      rplList = (i == 0) ? sps.getRPLList0() : sps.getRPLList1();
      rpl = rplList->getReferencePictureList(numRPLCandidates);
      rpl->setNumberOfShorttermPictures(1);
      rpl->setNumberOfLongtermPictures(0);
      rpl->setNumberOfActivePictures(1);
      rpl->setLtrpInSliceHeaderFlag(0);
      rpl->setRefPicIdentifier(0, -1, 0, false, 0);
      rpl->setPOC(0, 0);
    }
  }

  bool isRpl1CopiedFromRpl0 = true;
  for( int i = 0; isRpl1CopiedFromRpl0 && i < numRPLCandidates; i++)
  {
    if( sps.getRPLList0()->getReferencePictureList(i)->getNumRefEntries() == sps.getRPLList1()->getReferencePictureList(i)->getNumRefEntries() )
    {
      for( int j = 0; isRpl1CopiedFromRpl0 && j < sps.getRPLList0()->getReferencePictureList(i)->getNumRefEntries(); j++ )
      {
        if( sps.getRPLList0()->getReferencePictureList(i)->getRefPicIdentifier(j) != sps.getRPLList1()->getReferencePictureList(i)->getRefPicIdentifier(j) )
        {
          isRpl1CopiedFromRpl0 = false;
        }
      }
    }
    else
    {
      isRpl1CopiedFromRpl0 = false;
    }
  }
  sps.setRPL1CopyFromRPL0Flag(isRpl1CopiedFromRpl0);

  //Check if all delta POC of STRP in each RPL has the same sign
  //Check RPLL0 first
  const RPLList* rplList0 = sps.getRPLList0();
  const RPLList* rplList1 = sps.getRPLList1();
  uint32_t numberOfRPL = sps.getNumRPL0();

  bool isAllEntriesinRPLHasSameSignFlag = true;
  bool isFirstEntry = true;
  bool lastSign = true;        //true = positive ; false = negative
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && ii < numberOfRPL; ii++)
  {
    const ReferencePictureList* rpl = rplList0->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->getNumberOfActivePictures(); jj++)
    {
      if (!rpl->isRefPicLongterm(jj) && isFirstEntry)
      {
        lastSign = (rpl->getRefPicIdentifier(jj) >= 0) ? true : false;
        isFirstEntry = false;
      }
      else if (!rpl->isRefPicLongterm(jj) && (((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) >= 0 && lastSign == false) || ((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) < 0 && lastSign == true)))
      {
        isAllEntriesinRPLHasSameSignFlag = false;
      }
    }
  }
  //Check RPLL1. Skip it if it is already found out that this flag is not true for RPL0 or if RPL1 is the same as RPL0
  numberOfRPL = sps.getNumRPL1();
  isFirstEntry = true;
  lastSign = true;
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && !sps.getRPL1CopyFromRPL0Flag() && ii < numberOfRPL; ii++)
  {
    isFirstEntry = true;
    const ReferencePictureList* rpl = rplList1->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->getNumberOfActivePictures(); jj++)
    {
      if (!rpl->isRefPicLongterm(jj) && isFirstEntry)
      {
        lastSign = (rpl->getRefPicIdentifier(jj) >= 0) ? true : false;
        isFirstEntry = false;
      }
      else if (!rpl->isRefPicLongterm(jj) && (((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) >= 0 && lastSign == false) || ((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) < 0 && lastSign == true)))
      {
        isAllEntriesinRPLHasSameSignFlag = false;
      }
    }
  }
  sps.setAllActiveRplEntriesHasSameSignFlag(isAllEntriesinRPLHasSameSignFlag);
}

void EncLib::getActiveRefPicListNumForPOC(const SPS *sps, int POCCurr, int GOPid, uint32_t *activeL0, uint32_t *activeL1)
{
  if (m_intraPeriod < 0)  //Only for RA
  {
    *activeL0 = *activeL1 = 0;
    return;
  }
  uint32_t rpl0Idx = GOPid;
  uint32_t rpl1Idx = GOPid;

  int fullListNum = m_iGOPSize;
  int partialListNum = getRPLCandidateSize(0) - m_iGOPSize;
  int extraNum = fullListNum;
  if (m_intraPeriod < 0)
  {
    if (POCCurr < (2 * m_iGOPSize + 2))
    {
      int candidateIdx = (POCCurr + m_iGOPSize - 1 >= fullListNum + partialListNum) ? GOPid : POCCurr + m_iGOPSize - 1;
      rpl0Idx = candidateIdx;
      rpl1Idx = candidateIdx;
    }
    else
    {
      rpl0Idx = (POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1;
      rpl1Idx = (POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1;
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum<fullListNum + partialListNum; extraNum++)
  {
    if (m_intraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      int POCIndex = POCCurr % m_intraPeriod;
      if (POCIndex == 0)
      {
        POCIndex = m_intraPeriod;
      }
      if (POCIndex == m_RPLList0[extraNum].m_POC)
      {
        rpl0Idx = extraNum;
        rpl1Idx = extraNum;
        extraNum++;
      }
    }
  }

  const ReferencePictureList *rpl0 = sps->getRPLList0()->getReferencePictureList(rpl0Idx);
  *activeL0 = rpl0->getNumberOfActivePictures();
  const ReferencePictureList *rpl1 = sps->getRPLList1()->getReferencePictureList(rpl1Idx);
  *activeL1 = rpl1->getNumberOfActivePictures();
}

void EncLib::selectReferencePictureList(Slice* slice, int POCCurr, int GOPid, int ltPoc)
{
  bool isEncodeLtRef = (POCCurr == ltPoc);
  if (m_compositeRefEnabled && isEncodeLtRef)
  {
    POCCurr++;
  }

  slice->setRPL0idx(GOPid);
  slice->setRPL1idx(GOPid);

  int fullListNum = m_iGOPSize;
  int partialListNum = getRPLCandidateSize(0) - m_iGOPSize;
  int extraNum = fullListNum;

  int rplPeriod = m_intraPeriod;
  if( rplPeriod < 0 )  //Need to check if it is low delay or RA but with no RAP
  {
    if( slice->getSPS()->getRPLList0()->getReferencePictureList(1)->getRefPicIdentifier(0) * slice->getSPS()->getRPLList1()->getReferencePictureList(1)->getRefPicIdentifier(0) < 0)
    {
      rplPeriod = m_iGOPSize * 2;
    }
  }

  if (rplPeriod < 0)
  {
    if (POCCurr < (2 * m_iGOPSize + 2))
    {
      int candidateIdx = (POCCurr + m_iGOPSize - 1 >= fullListNum + partialListNum) ? GOPid : POCCurr + m_iGOPSize - 1;
      slice->setRPL0idx(candidateIdx);
      slice->setRPL1idx(candidateIdx);
    }
    else
    {
      slice->setRPL0idx((POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1);
      slice->setRPL1idx((POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1);
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum < fullListNum + partialListNum; extraNum++)
  {
    if( rplPeriod > 0 )
    {
      int POCIndex = POCCurr % rplPeriod;
      if (POCIndex == 0)
      {
        POCIndex = rplPeriod;
      }
      if (POCIndex == m_RPLList0[extraNum].m_POC)
      {
        slice->setRPL0idx(extraNum);
        slice->setRPL1idx(extraNum);
        extraNum++;
      }
    }
  }

  if (slice->getPic()->fieldPic)
  {
    // To set RPL index of POC1 (first bottom field)
    if (POCCurr == 1)
    {
      slice->setRPL0idx(getRPLCandidateSize(0));
      slice->setRPL1idx(getRPLCandidateSize(0));
    }
    else if( rplPeriod < 0 )
    {
      // To set RPL indexes for LD
      int numRPLCandidates = getRPLCandidateSize(0);
      if (POCCurr < numRPLCandidates - m_iGOPSize + 2)
      {
        slice->setRPL0idx(POCCurr + m_iGOPSize - 2);
        slice->setRPL1idx(POCCurr + m_iGOPSize - 2);
      }
      else
      {
        if (POCCurr%m_iGOPSize == 0)
        {
          slice->setRPL0idx(m_iGOPSize - 2);
          slice->setRPL1idx(m_iGOPSize - 2);
        }
        else if (POCCurr%m_iGOPSize == 1)
        {
          slice->setRPL0idx(m_iGOPSize - 1);
          slice->setRPL1idx(m_iGOPSize - 1);
        }
        else
        {
          slice->setRPL0idx(POCCurr % m_iGOPSize - 2);
          slice->setRPL1idx(POCCurr % m_iGOPSize - 2);
        }
      }
    }
  }

  const ReferencePictureList *rpl0 = (slice->getSPS()->getRPLList0()->getReferencePictureList(slice->getRPL0idx()));
  const ReferencePictureList *rpl1 = (slice->getSPS()->getRPLList1()->getReferencePictureList(slice->getRPL1idx()));
  *slice->getRPL0() = *rpl0;
  *slice->getRPL1() = *rpl1;
}


void EncLib::setParamSetChanged(int spsId, int ppsId)
{
  m_ppsMap.setChangedFlag(ppsId);
  m_spsMap.setChangedFlag(spsId);
}
bool EncLib::APSNeedsWriting(int apsId)
{
  bool isChanged = m_apsMap.getChangedFlag(apsId);
  m_apsMap.clearChangedFlag(apsId);
  return isChanged;
}

bool EncLib::PPSNeedsWriting(int ppsId)
{
  bool bChanged=m_ppsMap.getChangedFlag(ppsId);
  m_ppsMap.clearChangedFlag(ppsId);
  return bChanged;
}

bool EncLib::SPSNeedsWriting(int spsId)
{
  bool bChanged=m_spsMap.getChangedFlag(spsId);
  m_spsMap.clearChangedFlag(spsId);
  return bChanged;
}

void EncLib::checkPltStats( Picture* pic )
{
  int totalArea = 0;
  int pltArea = 0;
  for (auto apu : pic->cs->pus)
  {
    for (int i = 0; i < MAX_NUM_TBLOCKS; ++i)
    {
      int puArea = apu->blocks[i].width * apu->blocks[i].height;
      if (apu->blocks[i].width > 0 && apu->blocks[i].height > 0)
      {
        totalArea += puArea;
        if (CU::isPLT(*apu->cu) || CU::isIBC(*apu->cu))
        {
          pltArea += puArea;
        }
        break;
      }

    }
  }
  if (pltArea * PLT_FAST_RATIO < totalArea)
  {
    m_doPlt = false;
  }
  else
  {
    m_doPlt = true;
  }
}

#if X0038_LAMBDA_FROM_QP_CAPABILITY
int EncCfg::getQPForPicture(const uint32_t gopIndex, const Slice *pSlice) const
{
  const int lumaQpBDOffset = pSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  int qp;

  if (getCostMode()==COST_LOSSLESS_CODING)
  {
    qp = getBaseQP();
  }
  else
  {
    const SliceType sliceType=pSlice->getSliceType();

    qp = getBaseQP();

    // switch at specific qp and keep this qp offset
    static int appliedSwitchDQQ = 0; /* TODO: MT */
    if( pSlice->getPOC() == getSwitchPOC() )
    {
      appliedSwitchDQQ = getSwitchDQP();
    }
    qp += appliedSwitchDQQ;

#if QP_SWITCHING_FOR_PARALLEL
    const int* pdQPs = getdQPs();
    if ( pdQPs )
    {
      qp += pdQPs[pSlice->getPOC() / (m_compositeRefEnabled ? 2 : 1)];
    }
#endif

    if(sliceType==I_SLICE)
    {
      qp += getIntraQPOffset();
    }
    else
    {
        const GOPEntry &gopEntry=getGOPEntry(gopIndex);
        // adjust QP according to the QP offset for the GOP entry.
        qp +=gopEntry.m_QPOffset;

        // adjust QP according to QPOffsetModel for the GOP entry.
        double dqpOffset=qp*gopEntry.m_QPOffsetModelScale+gopEntry.m_QPOffsetModelOffset+0.5;
        int qpOffset = (int)floor(Clip3<double>(0.0, 3.0, dqpOffset));
        qp += qpOffset ;
      }

#if !QP_SWITCHING_FOR_PARALLEL
    // modify QP if a fractional QP was originally specified, cause dQPs to be 0 or 1.
    const int* pdQPs = getdQPs();
    if ( pdQPs )
    {
      qp += pdQPs[ pSlice->getPOC() ];
    }
#endif
  }
  qp = Clip3( -lumaQpBDOffset, MAX_QP, qp );
  return qp;
}
#endif


//! \}

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

/** \file     DecLib.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "DecLib.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/ProfileLevelTier.h"

#include <fstream>
#include <set>
#include <stdio.h>
#include <fcntl.h>
#include "AnnexBread.h"
#include "NALread.h"
#if K0149_BLOCK_STATISTICS
#include "CommonLib/dtrace_blockstatistics.h"
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

bool tryDecodePicture( Picture* pcEncPic, const int expectedPoc, const std::string& bitstreamFileName, ParameterSetMap<APS> *apsMap, bool bDecodeUntilPocFound /* = false */, int debugCTU /* = -1*/, int debugPOC /* = -1*/ )
{
  int      poc;
  PicList* pcListPic = NULL;

  static bool bFirstCall      = true;             /* TODO: MT */
  static bool loopFiltered[MAX_VPS_LAYERS] = { false };            /* TODO: MT */
  static int  iPOCLastDisplay = -MAX_INT;         /* TODO: MT */

  static std::ifstream* bitstreamFile = nullptr;  /* TODO: MT */
  static InputByteStream* bytestream  = nullptr;  /* TODO: MT */
  bool bRet = false;

  // create & initialize internal classes
  static DecLib *pcDecLib = nullptr;              /* TODO: MT */

  if( pcEncPic )
  {
    if( bFirstCall )
    {
      bitstreamFile = new std::ifstream( bitstreamFileName.c_str(), std::ifstream::in | std::ifstream::binary );
      bytestream    = new InputByteStream( *bitstreamFile );

      CHECK_( !*bitstreamFile, "failed to open bitstream file " << bitstreamFileName.c_str() << " for reading" ) ;
      // create decoder class
      pcDecLib = new DecLib;
      pcDecLib->create();

      // initialize decoder class
      pcDecLib->init(
#if  JVET_J0090_MEMORY_BANDWITH_MEASURE
        ""
#endif
      );

      pcDecLib->setDebugCTU( debugCTU );
      pcDecLib->setDebugPOC( debugPOC );
      pcDecLib->setDecodedPictureHashSEIEnabled( true );
      pcDecLib->setAPSMapEnc( apsMap );

      bFirstCall = false;
      msg(INFO_, "start to decode %s \n", bitstreamFileName.c_str() );
    }

    bool goOn = true;

    // main decoder loop
    while( !!*bitstreamFile && goOn )
    {
      InputNALUnit nalu;
      nalu.m_nalUnitType = NAL_UNIT_INVALID;

      // determine if next NAL unit will be the first one from a new picture
      bool bNewPicture = pcDecLib->isNewPicture( bitstreamFile,  bytestream );
      bool bNewAccessUnit = bNewPicture && pcDecLib->isNewAccessUnit( bNewPicture, bitstreamFile,  bytestream );
      bNewPicture = bNewPicture && bNewAccessUnit;

      if( !bNewPicture )
      {
      AnnexBStats stats       = AnnexBStats();
      byteStreamNALUnit( *bytestream, nalu.getBitstream().getFifo(), stats );

      // call actual decoding function
      if( nalu.getBitstream().getFifo().empty() )
      {
        /* this can happen if the following occur:
         *  - empty input file
         *  - two back-to-back start_code_prefixes
         *  - start_code_prefix immediately followed by EOF
         */
        msg(ERROR_, "Warning: Attempt to decode an empty NAL unit\n");
      }
      else
      {
        read( nalu );
        int iSkipFrame = 0;

        pcDecLib->decode(nalu, iSkipFrame, iPOCLastDisplay, 0);
      }
      }

      if ((bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && !pcDecLib->getFirstSliceInSequence(nalu.m_nuhLayerId))
      {
        if (!loopFiltered[nalu.m_nuhLayerId] || *bitstreamFile)
        {
          pcDecLib->finishPictureLight( poc, pcListPic );

          if( pcListPic )
          {
            for( auto & pic : *pcListPic )
            {
              if( pic->poc == poc && (!bDecodeUntilPocFound || expectedPoc == poc ) )
              {
                CHECK_( pcEncPic->slices.size() == 0, "at least one slice should be available" );

                CHECK_( expectedPoc != poc, "mismatch in POC - check encoder configuration" );

                if( debugCTU < 0 || poc != debugPOC )
                {
                for( int i = 0; i < pic->slices.size(); i++ )
                {
                  if( pcEncPic->slices.size() <= i )
                  {
                    pcEncPic->slices.push_back( new Slice );
                    pcEncPic->slices.back()->initSlice();
                    pcEncPic->slices.back()->setPPS( pcEncPic->slices[0]->getPPS() );
                    pcEncPic->slices.back()->setSPS( pcEncPic->slices[0]->getSPS() );
                    pcEncPic->slices.back()->setVPS( pcEncPic->slices[0]->getVPS() );
                    pcEncPic->slices.back()->setPic( pcEncPic->slices[0]->getPic() );
                  }
                  pcEncPic->slices[i]->copySliceInfo( pic->slices[i], false );
                }
                }

                pcEncPic->cs->slice = pcEncPic->slices.back();

                if( debugCTU >= 0 && poc == debugPOC )
                {
                  pcEncPic->cs->initStructData();

                  pcEncPic->cs->copyStructure( *pic->cs, CH_L, true, true );

                  if( CS::isDualITree( *pcEncPic->cs ) )
                  {
                    pcEncPic->cs->copyStructure( *pic->cs, CH_C, true, true );
                  }

                  for( auto &cu : pcEncPic->cs->cus )
                  {
                    cu->slice = pcEncPic->cs->slice;
                  }
                }
                else
                {
                if ( pic->cs->sps->getSAOEnabledFlag() )
                {
                  pcEncPic->copySAO( *pic, 0 );
                }

                if( pic->cs->sps->getALFEnabledFlag() )
                {
                  std::copy(pic->getAlfCtbFilterIndexVec().begin(), pic->getAlfCtbFilterIndexVec().end(), pcEncPic->getAlfCtbFilterIndexVec().begin());
                  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
                  {
                    std::copy( pic->getAlfCtuEnableFlag()[compIdx].begin(), pic->getAlfCtuEnableFlag()[compIdx].end(), pcEncPic->getAlfCtuEnableFlag()[compIdx].begin() );
                  }
                  pcEncPic->resizeAlfCtbFilterIndex(pic->cs->pcv->sizeInCtus);
                  memcpy( pcEncPic->getAlfCtbFilterIndex(), pic->getAlfCtbFilterIndex(), sizeof(short)*pic->cs->pcv->sizeInCtus );

                  std::copy( pic->getAlfCtuAlternative(COMPONENT_Cb).begin(), pic->getAlfCtuAlternative(COMPONENT_Cb).end(), pcEncPic->getAlfCtuAlternative(COMPONENT_Cb).begin() );
                  std::copy( pic->getAlfCtuAlternative(COMPONENT_Cr).begin(), pic->getAlfCtuAlternative(COMPONENT_Cr).end(), pcEncPic->getAlfCtuAlternative(COMPONENT_Cr).begin() );

                  for( int i = 0; i < pic->slices.size(); i++ )
                  {
                    pcEncPic->slices[i]->setTileGroupNumAps(pic->slices[i]->getTileGroupNumAps());
                    pcEncPic->slices[i]->setAlfAPSs(pic->slices[i]->getTileGroupApsIdLuma());
                    pcEncPic->slices[i]->setAlfAPSs(pic->slices[i]->getAlfAPSs());
                    pcEncPic->slices[i]->setTileGroupApsIdChroma(pic->slices[i]->getTileGroupApsIdChroma());
                    pcEncPic->slices[i]->setTileGroupAlfEnabledFlag(COMPONENT_Y,  pic->slices[i]->getTileGroupAlfEnabledFlag(COMPONENT_Y));
                    pcEncPic->slices[i]->setTileGroupAlfEnabledFlag(COMPONENT_Cb, pic->slices[i]->getTileGroupAlfEnabledFlag(COMPONENT_Cb));
                    pcEncPic->slices[i]->setTileGroupAlfEnabledFlag(COMPONENT_Cr, pic->slices[i]->getTileGroupAlfEnabledFlag(COMPONENT_Cr));
                    pcEncPic->slices[i]->setTileGroupCcAlfCbApsId(pic->slices[i]->getTileGroupCcAlfCbApsId());
                    pcEncPic->slices[i]->setTileGroupCcAlfCbEnabledFlag(pic->slices[i]->getTileGroupCcAlfCbEnabledFlag());
                    pcEncPic->slices[i]->setTileGroupCcAlfCrApsId(pic->slices[i]->getTileGroupCcAlfCrApsId());
                    pcEncPic->slices[i]->setTileGroupCcAlfCrEnabledFlag(pic->slices[i]->getTileGroupCcAlfCrEnabledFlag());
                  }
                }

                pcDecLib->executeLoopFilters();
                if ( pic->cs->sps->getSAOEnabledFlag() )
                {
                  pcEncPic->copySAO( *pic, 1 );
                }

                pcEncPic->cs->copyStructure( *pic->cs, CH_L, true, true );

                if( CS::isDualITree( *pcEncPic->cs ) )
                {
                  pcEncPic->cs->copyStructure( *pic->cs, CH_C, true, true );
                }
                }
                goOn = false; // exit the loop return
                bRet = true;
                break;
              }
            }
          }
          // postpone loop filters
          if (!bRet)
          {
            pcDecLib->executeLoopFilters();
          }

          pcDecLib->finishPicture( poc, pcListPic, DETAILS_);

          // write output
          if( ! pcListPic->empty())
          {
            PicList::iterator iterPic   = pcListPic->begin();
            int numPicsNotYetDisplayed = 0;
            int dpbFullness = 0;
            const SPS* activeSPS = (pcListPic->front()->cs->sps);
            uint32_t maxNrSublayers = activeSPS->getMaxTLayers();
            uint32_t maxNumReorderPicsHighestTid = activeSPS->getMaxNumReorderPics(maxNrSublayers-1);
            uint32_t maxDecPicBufferingHighestTid =  activeSPS->getMaxDecPicBuffering(maxNrSublayers-1);
            const VPS* referredVPS = pcListPic->front()->cs->vps;

            if( referredVPS != nullptr && referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] > 1 )
            {
              maxNumReorderPicsHighestTid = referredVPS->getMaxNumReorderPics( maxNrSublayers - 1 );
              maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering( maxNrSublayers - 1 );
            }

            while (iterPic != pcListPic->end())
            {
              Picture* pcCurPic = *(iterPic);
              if(pcCurPic->neededForOutput && pcCurPic->getPOC() > iPOCLastDisplay)
              {
                numPicsNotYetDisplayed++;
                dpbFullness++;
              }
              else if(pcCurPic->referenced)
              {
                dpbFullness++;
              }
              iterPic++;
            }

            iterPic = pcListPic->begin();

            if (numPicsNotYetDisplayed>2)
            {
              iterPic++;
            }

            Picture* pcCurPic = *(iterPic);
            if( numPicsNotYetDisplayed>2 && pcCurPic->fieldPic ) //Field Decoding
            {
              THROW( "no field coding support ");
            }
            else if( !pcCurPic->fieldPic ) //Frame Decoding
            {
              iterPic = pcListPic->begin();

              while (iterPic != pcListPic->end())
              {
                pcCurPic = *(iterPic);

                if(pcCurPic->neededForOutput && pcCurPic->getPOC() > iPOCLastDisplay &&
                  (numPicsNotYetDisplayed >  maxNumReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
                {
                    numPicsNotYetDisplayed--;
                  if( ! pcCurPic->referenced )
                  {
                    dpbFullness--;
                  }
                  // update POC of display order
                  iPOCLastDisplay = pcCurPic->getPOC();

                  // erase non-referenced picture in the reference picture list after display
                  if( ! pcCurPic->referenced && pcCurPic->reconstructed )
                  {
                    pcCurPic->reconstructed = false;
                  }
                  pcCurPic->neededForOutput = false;
                }

                iterPic++;
              }
            }
          }

          pcDecLib->updateAssociatedIRAP();
          // LMCS APS will be assigned later in LMCS initialization step
          pcEncPic->cs->picHeader->setLmcsAPS( nullptr );
          if( bitstreamFile )
          {
            pcDecLib->resetAccessUnitNals();
            pcDecLib->resetAccessUnitApsNals();
          }
        }
        loopFiltered[nalu.m_nuhLayerId] = (nalu.m_nalUnitType == NAL_UNIT_EOS);
        if( nalu.m_nalUnitType == NAL_UNIT_EOS )
        {
          pcDecLib->setFirstSliceInSequence(true, nalu.m_nuhLayerId);
        }

      }
      else if ((bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && pcDecLib->getFirstSliceInSequence(nalu.m_nuhLayerId))
      {
        pcDecLib->setFirstSliceInPicture( true );
      }
    }
  }

  if( !bRet )
  {
    CHECK_( bDecodeUntilPocFound, " decoding failed - check decodeBitstream2 parameter File: " << bitstreamFileName.c_str() );
    if( pcDecLib )
    {
      pcDecLib->destroy();
      pcDecLib->deletePicBuffer();
      delete pcDecLib;
      pcDecLib = nullptr;
    }
    bFirstCall   = true;
    for (int i = 0; i < MAX_VPS_LAYERS; i++)
    {
      loopFiltered[i] = false;
    }
    iPOCLastDisplay = -MAX_INT;

    if( bytestream )
    {
      delete bytestream;
      bytestream = nullptr;
    }

    if( bitstreamFile )
    {
      delete bitstreamFile;
      bitstreamFile = nullptr;
    }
  }

  return bRet;
}


//! \ingroup DecoderLib
//! \{

DecLib::DecLib()
  : m_iMaxRefPicNum(0)
  , m_isFirstGeneralHrd(true)
  , m_prevGeneralHrdParams()
  , m_associatedIRAPDecodingOrderNumber{ 0 }
  , m_decodingOrderCounter(0)
  , m_puCounter(0)
  , m_seiInclusionFlag(false)
  , m_pocRandomAccess(MAX_INT)
  , m_lastRasPoc(MAX_INT)
  , m_cListPic()
  , m_parameterSetManager()
  , m_apcSlicePilot(NULL)
  , m_SEIs()
  , m_cIntraPred()
  , m_cInterPred()
  , m_cTrQuant()
  , m_cSliceDecoder()
  , m_cTrQuantScalingList()
  , m_cCuDecoder()
  , m_HLSReader()
  , m_seiReader()
  , m_cLoopFilter()
  , m_cSAO()
  , m_cReshaper()
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
  , m_pcPic(NULL)
  , m_prevLayerID(MAX_INT)
  , m_prevPOC(MAX_INT)
  , m_prevPicPOC(MAX_INT)
  , m_prevTid0POC(0)
  , m_bFirstSliceInPicture(true)
  , m_firstSliceInSequence{ true }
  , m_firstSliceInBitstream(true)
  , m_isFirstAuInCvs( true )
  , m_prevSliceSkipped(false)
  , m_skippedPOC(0)
  , m_lastPOCNoOutputPriorPics(-1)
  , m_isNoOutputPriorPics(false)
  , m_lastNoOutputBeforeRecoveryFlag{ false }
  , m_sliceLmcsApsId(-1)
  , m_pDecodedSEIOutputStream(NULL)
  , m_audIrapOrGdrAuFlag( false )
#if JVET_S0257_DUMP_360SEI_MESSAGE
  , m_decoded360SeiDumpFileName()
#endif
  , m_decodedPictureHashSEIEnabled(false)
  , m_numberOfChecksumErrorsDetected(0)
  , m_warningMessageSkipPicture(false)
  , m_prefixSEINALUs()
  , m_debugPOC( -1 )
  , m_debugCTU( -1 )
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  , m_opi( nullptr )
  , m_mTidExternalSet(false)
  , m_mTidOpiSet(false)
  , m_tOlsIdxTidExternalSet(false)
  , m_tOlsIdxTidOpiSet(false)
#endif
  , m_vps( nullptr )
  , m_maxDecSubPicIdx(0)
  , m_maxDecSliceAddrInSubPic(-1)
  , m_clsVPSid(0)
  , m_targetSubPicIdx(0)
  , m_dci(NULL)
  , m_apsMapEnc( nullptr )
{
#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif
  memset(m_prevEOS, false, sizeof(m_prevEOS));
  memset(m_accessUnitEos, false, sizeof(m_accessUnitEos));
  std::fill_n(m_prevGDRInSameLayerPOC, MAX_VPS_LAYERS, -MAX_INT);
  std::fill_n(m_pocCRA, MAX_VPS_LAYERS, -MAX_INT);
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_associatedIRAPType[i] = NAL_UNIT_INVALID;
    std::fill_n(m_prevGDRSubpicPOC[i], MAX_NUM_SUB_PICS, -MAX_INT);
    std::fill_n(m_prevIRAPSubpicPOC[i], MAX_NUM_SUB_PICS, -MAX_INT);
    memset(m_prevIRAPSubpicDecOrderNo[i], 0, sizeof(int)*MAX_NUM_SUB_PICS);
    std::fill_n(m_prevIRAPSubpicType[i], MAX_NUM_SUB_PICS, NAL_UNIT_INVALID);
  }
}

DecLib::~DecLib()
{
  while (!m_prefixSEINALUs.empty())
  {
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }

}

void DecLib::create()
{
  m_apcSlicePilot = new Slice;
  m_uiSliceSegmentIdx = 0;
}

void DecLib::destroy()
{
  delete m_apcSlicePilot;
  m_apcSlicePilot = NULL;

  if( m_dci )
  {
    delete m_dci;
    m_dci = NULL;
  }

  m_cSliceDecoder.destroy();
}

void DecLib::init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  const std::string& cacheCfgFileName
#endif
)
{
  m_cSliceDecoder.init( &m_CABACDecoder, &m_cCuDecoder );
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cacheModel.create( cacheCfgFileName );
  m_cacheModel.clear( );
  m_cInterPred.cacheAssign( &m_cacheModel );
#endif
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
}

void DecLib::deletePicBuffer ( )
{
  PicList::iterator  iterPic   = m_cListPic.begin();
  int iSize = int( m_cListPic.size() );

  for (int i = 0; i < iSize; i++ )
  {
    Picture* pcPic = *(iterPic++);
    pcPic->destroy();

    delete pcPic;
    pcPic = NULL;
  }
  m_cALF.destroy();
  m_cSAO.destroy();
  m_cLoopFilter.destroy();
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cacheModel.reportSequence( );
  m_cacheModel.destroy( );
#endif
  m_cCuDecoder.destoryDecCuReshaprBuf();
  m_cReshaper.destroy();
}

Picture* DecLib::xGetNewPicBuffer( const SPS &sps, const PPS &pps, const uint32_t temporalLayer, const int layerId )
{
  Picture * pcPic = nullptr;
  m_iMaxRefPicNum = ( m_vps == nullptr || m_vps->m_numLayersInOls[m_vps->m_targetOlsIdx] == 1 ) ? sps.getMaxDecPicBuffering( temporalLayer ) : m_vps->getMaxDecPicBuffering( temporalLayer );     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
  if (m_cListPic.size() < (uint32_t)m_iMaxRefPicNum)
  {
    pcPic = new Picture();

    pcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true, layerId );

    m_cListPic.push_back( pcPic );

    return pcPic;
  }

  bool bBufferIsAvailable = false;
  for(auto * p: m_cListPic)
  {
    pcPic = p;  // workaround because range-based for-loops don't work with existing variables
    if ( pcPic->reconstructed == false && ! pcPic->neededForOutput )
    {
      pcPic->neededForOutput = false;
      bBufferIsAvailable = true;
      break;
    }

    if( ! pcPic->referenced  && ! pcPic->neededForOutput )
    {
      pcPic->neededForOutput = false;
      pcPic->reconstructed = false;
      bBufferIsAvailable = true;
      break;
    }
  }

  if( ! bBufferIsAvailable )
  {
    //There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    m_iMaxRefPicNum++;

    pcPic = new Picture();

    m_cListPic.push_back( pcPic );

    pcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true, layerId );
  }
  else
  {
    if( !pcPic->Y().Size::operator==( Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ) ) || pps.pcv->maxCUWidth != sps.getMaxCUWidth() || pps.pcv->maxCUHeight != sps.getMaxCUHeight() || pcPic->layerId != layerId )
    {
      pcPic->destroy();
      pcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true, layerId );
    }
  }

  pcPic->setBorderExtension( false );
  pcPic->neededForOutput = false;
  pcPic->reconstructed = false;

  return pcPic;
}


void DecLib::executeLoopFilters()
{
  if( !m_pcPic )
  {
    return; // nothing to deblock
  }

  m_pcPic->cs->slice->startProcessingTimer();

  CodingStructure& cs = *m_pcPic->cs;

  if (cs.sps->getUseLmcs() && cs.picHeader->getLmcsEnabledFlag())
  {
      const PreCalcValues& pcv = *cs.pcv;
      for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
      {
        for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
        {
          const CodingUnit* cu = cs.getCU(Position(xPos, yPos), CHANNEL_TYPE_LUMA);
          if (cu->slice->getLmcsEnabledFlag())
          {
            const uint32_t width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
            const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
            const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
            cs.getRecoBuf(area).get(COMPONENT_Y).rspSignal(m_cReshaper.getInvLUT());
          }
        }
      }
      m_cReshaper.setRecReshaped(false);
      m_cSAO.setReshaper(&m_cReshaper);
  }
  // deblocking filter
  m_cLoopFilter.loopFilterPic( cs );
  CS::setRefinedMotionField(cs);
  if( cs.sps->getSAOEnabledFlag() )
  {
    m_cSAO.SAOProcess( cs, cs.picture->getSAO() );
  }

  if( cs.sps->getALFEnabledFlag() )
  {
    m_cALF.getCcAlfFilterParam() = cs.slice->m_ccAlfFilterParam;
    // ALF decodes the differentially coded coefficients and stores them in the parameters structure.
    // Code could be restructured to do directly after parsing. So far we just pass a fresh non-const
    // copy in case the APS gets used more than once.
    m_cALF.ALFProcess(cs);
  }

  for (int i = 0; i < cs.pps->getNumSubPics() && m_targetSubPicIdx; i++)
  {
    // keep target subpic samples untouched, for other subpics mask their output sample value to 0
    int targetSubPicIdx = m_targetSubPicIdx - 1;
    if (i != targetSubPicIdx)
    {
      SubPic SubPicNoUse = cs.pps->getSubPics()[i];
      uint32_t left  = SubPicNoUse.getSubPicLeft();
      uint32_t right = SubPicNoUse.getSubPicRight();
      uint32_t top   = SubPicNoUse.getSubPicTop();
      uint32_t bottom= SubPicNoUse.getSubPicBottom();
      for (uint32_t row = top; row <= bottom; row++)
      {
        for (uint32_t col = left; col <= right; col++)
        {
          cs.getRecoBuf().Y().at(col, row) = 0;
          // for test only, hard coding using 4:2:0 chroma format
          cs.getRecoBuf().Cb().at(col>>1, row>>1) = 0;
          cs.getRecoBuf().Cr().at(col>>1, row>>1) = 0;
        }
      }
    }
  }

  m_pcPic->cs->slice->stopProcessingTimer();
}

void DecLib::finishPictureLight(int& poc, PicList*& rpcListPic )
{
  Slice*  pcSlice = m_pcPic->cs->slice;

  m_pcPic->neededForOutput = (pcSlice->getPicHeader()->getPicOutputFlag() ? true : false);
  m_pcPic->reconstructed = true;

  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcSlice->getPOC();
  rpcListPic          = &m_cListPic;
  m_puCounter++;
}

void DecLib::finishPicture(int &poc, PicList *&rpcListPic, MsgLevel msgl, bool associatedWithNewClvs)
{
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  CodingStatistics::StatTool& s = CodingStatistics::GetStatisticTool( STATS__TOOL_TOTAL_FRAME );
  s.count++;
  s.pixels = s.count * m_pcPic->Y().width * m_pcPic->Y().height;
#endif

  Slice*  pcSlice = m_pcPic->cs->slice;
  m_prevPicPOC = pcSlice->getPOC();

  char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!m_pcPic->referenced)
  {
    c += 32;  // tolower
  }

  if (pcSlice->isDRAP()) c = 'D';

  //-- For time output for each slice
  msg( msgl, "POC %4d LId: %2d TId: %1d ( %s, %c-SLICE, QP%3d ) ", pcSlice->getPOC(), pcSlice->getPic()->layerId,
         pcSlice->getTLayer(),
         nalUnitTypeToString(pcSlice->getNalUnitType()),
         c,
         pcSlice->getSliceQp() );
  msg( msgl, "[DT %6.3f] ", pcSlice->getProcessingTime() );

  for (int iRefList = 0; iRefList < 2; iRefList++)
  {
    msg( msgl, "[L%d", iRefList);
    for (int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      const std::pair<int, int>& scaleRatio = pcSlice->getScalingRatio( RefPicList( iRefList ), iRefIndex );

      if( pcSlice->getPicHeader()->getEnableTMVPFlag() && pcSlice->getColFromL0Flag() == bool(1 - iRefList) && pcSlice->getColRefIdx() == iRefIndex )
      {
        if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
        {
          msg( msgl, " %dc(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
        }
        else
        {
          msg( msgl, " %dc", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) );
        }
      }
      else
      {
        if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
        {
          msg( msgl, " %d(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
        }
        else
        {
          msg( msgl, " %d", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) );
        }
      }

      if( pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) == pcSlice->getPOC() )
      {
        msg( msgl, ".%d", pcSlice->getRefPic( RefPicList( iRefList ), iRefIndex )->layerId );
      }
    }
    msg( msgl, "] ");
  }
  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages pictureHashes = getSeisByType(m_pcPic->SEIs, SEI::DECODED_PICTURE_HASH );
    const SEIDecodedPictureHash *hash = ( pictureHashes.size() > 0 ) ? (SEIDecodedPictureHash*) *(pictureHashes.begin()) : NULL;
    if (pictureHashes.size() > 1)
    {
      msg(WARNING_, "Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }
    m_numberOfChecksumErrorsDetected += calcAndPrintHashStatus(((const Picture*) m_pcPic)->getRecoBuf(), hash, pcSlice->getSPS()->getBitDepths(), msgl);
  }

  msg( msgl, "\n");

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    m_cacheModel.reportFrame();
    m_cacheModel.accumulateFrame();
    m_cacheModel.clear();
#endif

  m_pcPic->neededForOutput = (pcSlice->getPicHeader()->getPicOutputFlag() ? true : false);
  if (associatedWithNewClvs && m_pcPic->neededForOutput)
  {
    if (!pcSlice->getPPS()->getMixedNaluTypesInPicFlag() && pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL)
    {
      m_pcPic->neededForOutput = false;
    }
    else if (pcSlice->getPPS()->getMixedNaluTypesInPicFlag())
    {
      bool isRaslPic = true;
      for (int i = 0; isRaslPic && i < m_pcPic->numSlices; i++) 
      {
        if (!(pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL))
        {
          isRaslPic = false;
        }
      }
      if (isRaslPic)
      {
        m_pcPic->neededForOutput = false;
      }
    }
  }
  m_pcPic->reconstructed = true;


  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcSlice->getPOC();
  rpcListPic          = &m_cListPic;
  m_bFirstSliceInPicture  = true; // TODO: immer true? hier ist irgendwas faul
  m_maxDecSubPicIdx = 0;
  m_maxDecSliceAddrInSubPic = -1;

  m_pcPic->destroyTempBuffers();
  m_pcPic->cs->destroyCoeffs();
  m_pcPic->cs->releaseIntermediateData();
  m_pcPic->cs->picHeader->initPicHeader();
  m_puCounter++;
}

void DecLib::checkNoOutputPriorPics (PicList* pcListPic)
{
  if (!pcListPic || !m_isNoOutputPriorPics)
  {
    return;
  }

  PicList::iterator  iterPic   = pcListPic->begin();

  while (iterPic != pcListPic->end())
  {
    Picture* pcPicTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != pcPicTmp->getPOC())
    {
      pcPicTmp->neededForOutput = false;
    }
  }
}

void DecLib::xUpdateRasInit(Slice* slice)
{
  slice->setPendingRasInit( false );
  if ( slice->getPOC() > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->setPendingRasInit( true );
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->getPOC();
  }
}

void DecLib::xCreateLostPicture( int iLostPoc, const int layerId )
{
  msg(INFO_, "\ninserting lost poc : %d\n",iLostPoc);
  Picture *cFillPic = xGetNewPicBuffer( *( m_parameterSetManager.getFirstSPS() ), *( m_parameterSetManager.getFirstPPS() ), 0, layerId );

  CHECK_( !cFillPic->slices.size(), "No slices in picture" );

  cFillPic->slices[0]->initSlice();

  PicList::iterator iterPic = m_cListPic.begin();
  int closestPoc = 1000000;
  while ( iterPic != m_cListPic.end())
  {
    Picture * rpcPic = *(iterPic++);
    if(abs(rpcPic->getPOC() -iLostPoc)<closestPoc&&abs(rpcPic->getPOC() -iLostPoc)!=0&&rpcPic->getPOC()!=m_apcSlicePilot->getPOC())
    {
      closestPoc=abs(rpcPic->getPOC() -iLostPoc);
    }
  }
  iterPic = m_cListPic.begin();
  while ( iterPic != m_cListPic.end())
  {
    Picture *rpcPic = *(iterPic++);
    if(abs(rpcPic->getPOC() -iLostPoc)==closestPoc&&rpcPic->getPOC()!=m_apcSlicePilot->getPOC())
    {
      msg(INFO_, "copying picture %d to %d (%d)\n",rpcPic->getPOC() ,iLostPoc,m_apcSlicePilot->getPOC());
      cFillPic->getRecoBuf().copyFrom( rpcPic->getRecoBuf() );
      break;
    }
  }

//  for(int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)  { cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr); }
  cFillPic->referenced = true;
  cFillPic->slices[0]->setPOC(iLostPoc);
  xUpdatePreviousTid0POC(cFillPic->slices[0]);
  cFillPic->reconstructed = true;
  cFillPic->neededForOutput = true;
  if(m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iLostPoc;
  }

}

void  DecLib::xCreateUnavailablePicture( const PPS *pps, const int iUnavailablePoc, const bool longTermFlag, const int temporalId, const int layerId, const bool interLayerRefPicFlag )
{
  msg(INFO_, "\ninserting unavailable poc : %d\n", iUnavailablePoc);
  Picture* cFillPic = xGetNewPicBuffer( *( m_parameterSetManager.getFirstSPS() ), *( m_parameterSetManager.getFirstPPS() ), 0, layerId );

  CHECK_(!cFillPic->slices.size(), "No slices in picture");

  cFillPic->slices[0]->initSlice();

  uint32_t yFill = 1 << (m_parameterSetManager.getFirstSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  uint32_t cFill = 1 << (m_parameterSetManager.getFirstSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
  cFillPic->getRecoBuf().Y().fill(yFill);
  cFillPic->getRecoBuf().Cb().fill(cFill);
  cFillPic->getRecoBuf().Cr().fill(cFill);

  //  for(int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)  { cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr); }
  cFillPic->referenced = true;
  cFillPic->interLayerRefPicFlag = interLayerRefPicFlag;
  cFillPic->longTerm = longTermFlag;
  cFillPic->slices[0]->setPOC(iUnavailablePoc);
  xUpdatePreviousTid0POC(cFillPic->slices[0]);
  cFillPic->reconstructed = true;
  cFillPic->neededForOutput = false;
  // picture header is not derived for generated reference picture
  cFillPic->slices[0]->setPicHeader( nullptr );
  cFillPic->temporalId = temporalId;
  cFillPic->nonReferencePictureFlag = false;
  cFillPic->slices[0]->setPPS( pps );

  if (m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iUnavailablePoc;
  }
}
void DecLib::checkPicTypeAfterEos()
{
  int layerId = m_pcPic->slices[0]->getNalUnitLayerId();
  if (m_prevEOS[layerId])
  {
    bool isIrapOrGdrPu = !m_pcPic->cs->pps->getMixedNaluTypesInPicFlag() && ( m_pcPic->slices[0]->isIRAP() || m_pcPic->slices[0]->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR );
    CHECK_(!isIrapOrGdrPu, "when present, the next PU of a particular layer after an EOS NAL unit that belongs to the same layer shall be an IRAP or GDR PU");

    m_prevEOS[layerId] = false;
  }
}

void DecLib::checkLayerIdIncludedInCvss()
{
  if ((getVPS()->getMaxLayers() == 1 || m_audIrapOrGdrAuFlag) && (m_isFirstAuInCvs || m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP || m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
  {
    // store layerIDs in the first AU
    m_firstAccessUnitPicInfo.assign(m_accessUnitPicInfo.begin(), m_accessUnitPicInfo.end());
  }
  else
  {
    // check whether the layerIDs in an AU are included in the layerIDs of the first AU
    for (auto pic = m_accessUnitPicInfo.begin(); pic != m_accessUnitPicInfo.end(); pic++)
    {
      bool layerIdFind;
      for (auto picFirst = m_firstAccessUnitPicInfo.begin(); picFirst != m_firstAccessUnitPicInfo.end(); picFirst++)
      {
        layerIdFind = pic->m_nuhLayerId == picFirst->m_nuhLayerId ? true : false;
        if (layerIdFind)
        {
          break;
        }
      }
      CHECK_(!layerIdFind, "each picture in an AU in a CVS shall have nuh_layer_id equal to the nuh_layer_id of one of the pictures present in the first AU of the CVS");
    }


    // check whether the layerID of EOS_NUT is included in the layerIDs of the first AU
    for (int i = 0; i < getVPS()->getMaxLayers(); i++)
    {
      int eosLayerId = getVPS()->getLayerId(i);
      if (m_accessUnitEos[eosLayerId])
      {
        bool eosLayerIdFind;
        for (auto picFirst = m_firstAccessUnitPicInfo.begin(); picFirst != m_firstAccessUnitPicInfo.end(); picFirst++)
        {
          eosLayerIdFind = eosLayerId == picFirst->m_nuhLayerId ? true : false;
          if (eosLayerIdFind)
          {
            break;
          }
        }
        CHECK_(!eosLayerIdFind, "When nal_unit_type is equal to EOS_NUT, nuh_layer_id shall be equal to one of the nuh_layer_id values of the layers present in the CVS");
      }
    }
  }

  // update the value of m_isFirstAuInCvs for the next AU according to NAL_UNIT_EOS in each layer
  for (auto pic = m_accessUnitPicInfo.begin(); pic != m_accessUnitPicInfo.end(); pic++)
  {
    m_isFirstAuInCvs = m_accessUnitEos[pic->m_nuhLayerId] ? true : false;
    if(!m_isFirstAuInCvs)
    {
      break;
    }
  }
}

void DecLib::CheckNoOutputPriorPicFlagsInAccessUnit()
{
  if (m_accessUnitNoOutputPriorPicFlags.size() > 1)
  {
    bool anchor = m_accessUnitNoOutputPriorPicFlags[0];
    bool isDiffFlagsInAu = std::find(m_accessUnitNoOutputPriorPicFlags.begin(), m_accessUnitNoOutputPriorPicFlags.end(), !anchor) != m_accessUnitNoOutputPriorPicFlags.end();
    CHECK_(isDiffFlagsInAu, "The value of no_output_of_prior_pics_flag, when present, is required to be the same for all pictures in an AU");
  }
}

void DecLib::checkTidLayerIdInAccessUnit()
{
  int firstPicTid = m_accessUnitPicInfo.begin()->m_temporalId;
  int firstPicLayerId = m_accessUnitPicInfo.begin()->m_nuhLayerId;

  bool isPicTidInAuSame = true;
  bool isSeiTidInAuSameAsAuTid = true;
  bool isFdNaluLayerIdSameAsVclNaluLayerId = true;
  bool isFdTidInAuSameAsAuTid = true;

  for (auto pic = m_accessUnitPicInfo.begin(); pic != m_accessUnitPicInfo.end(); pic++)
  {
    if (pic->m_temporalId != firstPicTid)
    {
      isPicTidInAuSame = false;
      break;
    }
  }
  CHECK_(!isPicTidInAuSame, "All pictures in an AU shall have the same value of TemporalId");

  for (auto tid = m_accessUnitSeiTids.begin(); tid != m_accessUnitSeiTids.end(); tid++)
  {
    if ((*tid) != firstPicTid)
    {
      isSeiTidInAuSameAsAuTid = false;
      break;
    }
  }
  CHECK_(!isSeiTidInAuSameAsAuTid, "The TemporalId of an SEI NAL unit shall be equal to the TemporalId of the AU containing the NAL unit");

  for (auto tempNalu = m_accessUnitNals.begin(); tempNalu != m_accessUnitNals.end(); tempNalu++)
  {
    if ((tempNalu->m_nalUnitType == NAL_UNIT_FD) && (tempNalu->m_nuhLayerId != firstPicLayerId))
    {
      isFdNaluLayerIdSameAsVclNaluLayerId = false;
      break;
    }
  }
  CHECK_(!isFdNaluLayerIdSameAsVclNaluLayerId, "The nuh_layer_id of a filler data NAL unit shall be equal to the nuh_layer_id of associated VCL NAL unit");

  for (auto tempNalu = m_accessUnitNals.begin(); tempNalu != m_accessUnitNals.end(); tempNalu++)
  {
    if ((tempNalu->m_nalUnitType == NAL_UNIT_FD) && (tempNalu->m_temporalId != firstPicTid))
    {
      isFdTidInAuSameAsAuTid = false;
      break;
    }
  }
  CHECK_(!isFdTidInAuSameAsAuTid, "The TemporalId of a filler data NAL unit shall be equal to the TemporalId of the AU containing the NAL unit");
}

void DecLib::checkSEIInAccessUnit()
{
  for (auto &sei : m_accessUnitSeiPayLoadTypes)
  {
    enum NalUnitType         naluType = std::get<0>(sei);
    enum SEI::PayloadType payloadType = std::get<2>(sei);
    if (naluType == NAL_UNIT_PREFIX_SEI && ((payloadType == SEI::BUFFERING_PERIOD || payloadType == SEI::PICTURE_TIMING || payloadType == SEI::DECODING_UNIT_INFO || payloadType == SEI::SUBPICTURE_LEVEL_INFO)))
    {
      bool olsIncludeAllLayersFind = false;
      for (int i = 0; i < m_vps->getNumOutputLayerSets(); i++)
      {
        for (auto pic = m_firstAccessUnitPicInfo.begin(); pic != m_firstAccessUnitPicInfo.end(); pic++)
        {
          int targetLayerId = pic->m_nuhLayerId;
          for (int j = 0; j < m_vps->getNumLayersInOls(i); j++)
          {
            olsIncludeAllLayersFind = m_vps->getLayerIdInOls(i, j) == targetLayerId ? true : false;
            if (olsIncludeAllLayersFind)
            {
              break;
            }
          }
          if (!olsIncludeAllLayersFind)
          {
            break;
          }
        }
        if (olsIncludeAllLayersFind)
        {
          break;
        }
      }
      CHECK_(!olsIncludeAllLayersFind, "When there is no OLS that includes all layers in the current CVS in the entire bitstream, there shall be no non-scalable-nested SEI message with payloadType equal to 0 (BP), 1 (PT), 130 (DUI), or 203 (SLI)");
    }
  }
}

#define SEI_REPETITION_CONSTRAINT_LIST_SIZE  21

/**
 - Count the number of identical SEI messages in the current picture
 */
void DecLib::checkSeiInPictureUnit()
{  
  std::vector<std::tuple<int, uint32_t, uint8_t*>> seiList;

  // payload types subject to constrained SEI repetition
  int picUnitRepConSeiList[SEI_REPETITION_CONSTRAINT_LIST_SIZE] = { 0, 1, 19, 45, 129, 132, 133, 137, 144, 145, 147, 148, 149, 150, 153, 154, 155, 156, 168, 203, 204};

  // extract SEI messages from NAL units
  for (auto &sei : m_pictureSeiNalus)
  {
    InputBitstream bs = sei->getBitstream();

    do
    {  
      int payloadType = 0;
      uint32_t val = 0;

      do
      {
        bs.readByte(val);
        payloadType += val;
      } while (val==0xFF);

      uint32_t payloadSize = 0;
      do
      {
        bs.readByte(val);
        payloadSize += val;
      } while (val==0xFF);
    
      uint8_t *payload = new uint8_t[payloadSize];
      for (uint32_t i = 0; i < payloadSize; i++)
      {
        bs.readByte(val);
        payload[i] = (uint8_t)val;
      }
      seiList.push_back(std::tuple<int, uint32_t, uint8_t*>(payloadType, payloadSize, payload));
    }
    while (bs.getNumBitsLeft() > 8);
  }

  // count repeated messages in list
  for (uint32_t i = 0; i < seiList.size(); i++)
  {
    int      k, count = 1;      
    int      payloadType1 = std::get<0>(seiList[i]);
    uint32_t payloadSize1 = std::get<1>(seiList[i]);
    uint8_t  *payload1    = std::get<2>(seiList[i]);

    // only consider SEI payload types in the PicUnitRepConSeiList
    for(k=0; k<SEI_REPETITION_CONSTRAINT_LIST_SIZE; k++)
    {
      if(payloadType1 == picUnitRepConSeiList[k])
      {
        break;
      }
    }
    if(k >= SEI_REPETITION_CONSTRAINT_LIST_SIZE)
    {
      continue;
    }

    // compare current SEI message with remaining messages in the list
    for (uint32_t j = i+1; j < seiList.size(); j++)
    {
      int      payloadType2 = std::get<0>(seiList[j]);
      uint32_t payloadSize2 = std::get<1>(seiList[j]);
      uint8_t  *payload2    = std::get<2>(seiList[j]);
      
      // check for identical SEI type, size, and payload
      if(payloadType1 == payloadType2 && payloadSize1 == payloadSize2)
      {
        if(memcmp(payload1, payload2, payloadSize1*sizeof(uint8_t)) == 0)
        {
          count++;
        }
      }
    }
    CHECK_(count > 4, "There shall be less than or equal to 4 identical sei_payload( ) syntax structures within a picture unit.");
  }

  // free SEI message list memory
  for (uint32_t i = 0; i < seiList.size(); i++)
  {
    uint8_t *payload = std::get<2>(seiList[i]);
    delete[] payload;
  }
  seiList.clear();
}

/**
 - Reset list of SEI NAL units from the current picture
 */
void DecLib::resetPictureSeiNalus()   
{
  while (!m_pictureSeiNalus.empty())
  {
    delete m_pictureSeiNalus.front();
    m_pictureSeiNalus.pop_front();
  }
}

/**
 - Determine if the first VCL NAL unit of a picture is also the first VCL NAL of an Access Unit
 */
bool DecLib::isSliceNaluFirstInAU( bool newPicture, InputNALUnit &nalu )
{
  // can only be the start of an AU if this is the start of a new picture
  if( newPicture == false )
  {
    return false;
  }

  // should only be called for slice NALU types
  if( nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_TRAIL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_STSA &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RADL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_IDR_W_RADL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_IDR_N_LP &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_CRA &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_GDR )
  {
    return false;
  }


  // check for layer ID less than or equal to previous picture's layer ID
  if( nalu.m_nuhLayerId <= m_prevLayerID )
  {
    return true;
  }

  // get slice POC
  m_apcSlicePilot->setPicHeader( &m_picHeader );
  m_apcSlicePilot->initSlice();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.getSlicePoc( m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC );

  // check for different POC
  return (m_apcSlicePilot->getPOC() != m_prevPOC);
}

void DecLib::checkAPSInPictureUnit()
{
  bool firstVCLFound = false;
  bool suffixAPSFound = false;
  for (auto &nalu : m_pictureUnitNals)
  {
    if (NALUnit::isVclNalUnitType(nalu))
    {
      firstVCLFound = true;
      CHECK_( suffixAPSFound, "When any suffix APS NAL units are present in a PU, they shall follow the last VCL unit of the PU" );
    }
    else if (nalu == NAL_UNIT_PREFIX_APS)
    {
      CHECK_( firstVCLFound, "When any prefix APS NAL units are present in a PU, they shall precede the first VCL unit of the PU");
    }
    else if (nalu == NAL_UNIT_SUFFIX_APS)
    {
      suffixAPSFound = true;
    }
  }
}

void activateAPS(PicHeader* picHeader, Slice* pSlice, ParameterSetManager& parameterSetManager, APS** apss, APS* lmcsAPS, APS* scalingListAPS)
{
  const SPS *sps = parameterSetManager.getSPS(picHeader->getSPSId());
  //luma APSs
  if (pSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y))
  {
    for (int i = 0; i < pSlice->getTileGroupApsIdLuma().size(); i++)
    {
      int apsId = pSlice->getTileGroupApsIdLuma()[i];
      APS* aps = parameterSetManager.getAPS(apsId, ALF_APS);

      if (aps)
      {
        apss[apsId] = aps;
        if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
        {
          THROW("APS activation failed!");
        }

        CHECK_( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
        //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

        CHECK_( sps->getChromaFormatIdc() == CHROMA_400 && aps->chromaPresentFlag, "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an ALF_APS shall be equal to 0" );

        CHECK_(((sps->getCCALFEnabledFlag() == false) && (aps->getCcAlfAPSParam().newCcAlfFilter[0] || aps->getCcAlfAPSParam().newCcAlfFilter[1])), "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag shall be equal to 0");
      }
    }
  }
  if (pSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cb)||pSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) )
  {
    //chroma APS
    int apsId = pSlice->getTileGroupApsIdChroma();
    APS* aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if (aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK_( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

      CHECK_(((sps->getCCALFEnabledFlag() == false) && (aps->getCcAlfAPSParam().newCcAlfFilter[0] || aps->getCcAlfAPSParam().newCcAlfFilter[1])), "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag shall be equal to 0");
    }
  }

  CcAlfFilterParam &filterParam = pSlice->m_ccAlfFilterParam;
  // cleanup before copying
  for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
  {
    memset( filterParam.ccAlfCoeff[COMPONENT_Cb - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMPONENT_Cb - 1][filterIdx]) );
    memset( filterParam.ccAlfCoeff[COMPONENT_Cr - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMPONENT_Cr - 1][filterIdx]) );
  }
  memset( filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cb - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cb - 1]) );
  memset( filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cr - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cr - 1]) );

  if(pSlice->getTileGroupCcAlfCbEnabledFlag())
  {
    int apsId = pSlice->getTileGroupCcAlfCbApsId();
    APS *aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if(aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK_( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

      filterParam.ccAlfFilterCount[COMPONENT_Cb - 1] = aps->getCcAlfAPSParam().ccAlfFilterCount[COMPONENT_Cb - 1];
      for (int filterIdx=0; filterIdx < filterParam.ccAlfFilterCount[COMPONENT_Cb - 1]; filterIdx++ )
      {
        filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cb - 1][filterIdx] = aps->getCcAlfAPSParam().ccAlfFilterIdxEnabled[COMPONENT_Cb - 1][filterIdx];
        memcpy(filterParam.ccAlfCoeff[COMPONENT_Cb - 1][filterIdx], aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cb - 1][filterIdx], sizeof(aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cb - 1][filterIdx]));
      }
    }
    else
    {
      THROW("CC ALF Cb APS not available!");
    }
  }

  if(pSlice->getTileGroupCcAlfCrEnabledFlag())
  {
    int apsId = pSlice->getTileGroupCcAlfCrApsId();
    APS *aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if(aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK_( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

      filterParam.ccAlfFilterCount[COMPONENT_Cr - 1] = aps->getCcAlfAPSParam().ccAlfFilterCount[COMPONENT_Cr - 1];
      for (int filterIdx=0; filterIdx < filterParam.ccAlfFilterCount[COMPONENT_Cr - 1]; filterIdx++ )
      {
        filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cr - 1][filterIdx] = aps->getCcAlfAPSParam().ccAlfFilterIdxEnabled[COMPONENT_Cr - 1][filterIdx];
        memcpy(filterParam.ccAlfCoeff[COMPONENT_Cr - 1][filterIdx], aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cr - 1][filterIdx], sizeof(aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cr - 1][filterIdx]));
      }
    }
    else
    {
      THROW("CC ALF Cr APS not available!");
    }
  }

  if (picHeader->getLmcsEnabledFlag() && lmcsAPS == nullptr)
  {
    lmcsAPS = parameterSetManager.getAPS(picHeader->getLmcsAPSId(), LMCS_APS);
    CHECK_(lmcsAPS == nullptr, "No LMCS APS present");
    if (lmcsAPS)
    {
      parameterSetManager.clearAPSChangedFlag(picHeader->getLmcsAPSId(), LMCS_APS);
      if (false == parameterSetManager.activateAPS(picHeader->getLmcsAPSId(), LMCS_APS))
      {
        THROW("LMCS APS activation failed!");
      }

      CHECK_( sps->getChromaFormatIdc() == CHROMA_400 && lmcsAPS->chromaPresentFlag, "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an LMCS_APS shall be equal to 0");

      CHECK_( lmcsAPS->getReshaperAPSInfo().maxNbitsNeededDeltaCW - 1 < 0 || lmcsAPS->getReshaperAPSInfo().maxNbitsNeededDeltaCW - 1 > sps->getBitDepth(CHANNEL_TYPE_LUMA) - 2, "The value of lmcs_delta_cw_prec_minus1 of an LMCS_APS shall be in the range of 0 to BitDepth 2, inclusive" );

      CHECK_( lmcsAPS->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->setLmcsAPS(lmcsAPS);

  if( picHeader->getExplicitScalingListEnabledFlag() && scalingListAPS == nullptr)
  {
    scalingListAPS = parameterSetManager.getAPS( picHeader->getScalingListAPSId(), SCALING_LIST_APS );
    CHECK_( scalingListAPS == nullptr, "No SCALING LIST APS present" );
    if( scalingListAPS )
    {
      parameterSetManager.clearAPSChangedFlag( picHeader->getScalingListAPSId(), SCALING_LIST_APS );
      if( false == parameterSetManager.activateAPS( picHeader->getScalingListAPSId(), SCALING_LIST_APS ) )
      {
        THROW( "SCALING LIST APS activation failed!" );
      }

      CHECK_( (sps->getChromaFormatIdc() == CHROMA_400 && scalingListAPS->chromaPresentFlag) || (sps->getChromaFormatIdc() != CHROMA_400 && !scalingListAPS->chromaPresentFlag),
        "The value of aps_chroma_present_flag of the APS NAL unit having aps_params_type equal to SCALING_APS and adaptation_parameter_set_id equal to ph_scaling_list_aps_id shall be equal to ChromaArrayType  = =  0 ? 0 : 1" );

      CHECK_( scalingListAPS->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->setScalingListAPS(scalingListAPS);
}

void DecLib::checkParameterSetsInclusionSEIconstraints(const InputNALUnit nalu)
{
  const PPS* pps = m_pcPic->cs->pps;
  const APS* lmcsAPS = m_pcPic->cs->lmcsAps;
  const APS* scalinglistAPS = m_pcPic->cs->scalinglistAps;
  APS** apss = m_parameterSetManager.getAPSs();

  CHECK_(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA &&
        pps->getTemporalId() == nalu.m_temporalId &&
        pps->getPuCounter() > m_puCounter, "Violating Parameter Sets Inclusion Indication SEI constraint");

  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
  {
    if (apss[i] != nullptr)
    {
      CHECK_(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA &&
            apss[i]->getTemporalId() == nalu.m_temporalId &&
            apss[i]->getPuCounter() > m_puCounter, "Violating Parameter Sets Inclusion Indication SEI constraint");
    }
  }
  if (lmcsAPS != nullptr)
  {
    CHECK_(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA &&
          lmcsAPS->getTemporalId() == nalu.m_temporalId &&
          lmcsAPS->getPuCounter() > m_puCounter, "Violating Parameter Sets Inclusion Indication SEI constraint");
  }
  if (scalinglistAPS != nullptr)
  {
    CHECK_(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA &&
          scalinglistAPS->getTemporalId() == nalu.m_temporalId &&
          scalinglistAPS->getPuCounter() > m_puCounter, "Violating Parameter Sets Inclusion Indication SEI constraint");
  }
}

void DecLib::xActivateParameterSets( const InputNALUnit nalu )
{
  const int layerId = nalu.m_nuhLayerId;
  if (m_bFirstSliceInPicture)
  {
    APS** apss = m_parameterSetManager.getAPSs();
    memset(apss, 0, sizeof(*apss) * ALF_CTB_MAX_NUM_APS);
    const PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId()); // this is a temporary PPS object. Do not store this value
    CHECK_(pps == 0, "Referred to PPS not present");

    const SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());             // this is a temporary SPS object. Do not store this value
    CHECK_(sps == 0, "Referred to SPS not present");

    const VPS *vps = m_parameterSetManager.getVPS( sps->getVPSId() );
    CHECK_(vps == 0, "Referred to VPS not present");

    if( nullptr != pps->pcv )
    {
      delete m_parameterSetManager.getPPS( m_picHeader.getPPSId() )->pcv;
    }
    m_parameterSetManager.getPPS( m_picHeader.getPPSId() )->pcv = new PreCalcValues( *sps, *pps, false );
    m_parameterSetManager.clearSPSChangedFlag(sps->getSPSId());
    m_parameterSetManager.clearPPSChangedFlag(pps->getPPSId());

    if (false == m_parameterSetManager.activatePPS(m_picHeader.getPPSId(),m_apcSlicePilot->isIRAP()))
    {
      THROW("Parameter set activation failed!");
    }

    // update the stored VPS to the actually referred to VPS
    m_vps = m_parameterSetManager.getVPS(sps->getVPSId());
    if(sps->getVPSId() == 0)
    {
      //No VPS in bitstream: set defaults values of variables in VPS to the ones signalled in SPS
      m_vps->setMaxSubLayers( sps->getMaxTLayers() );
      m_vps->setLayerId( 0, sps->getLayerId() );
      m_vps->deriveOutputLayerSets(); 
    }
    else
    {
      //VPS in the bitstream: check that SPS and VPS signalling are compatible
      CHECK_(sps->getMaxTLayers() > m_vps->getMaxSubLayers(), "The SPS signals more temporal sub-layers than allowed by the VPS");
    }

    m_parameterSetManager.getApsMap()->clearActive();
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* aps = m_parameterSetManager.getAPS(i, ALF_APS);
      if (aps)
      {
        m_parameterSetManager.clearAPSChangedFlag(i, ALF_APS);
      }
    }
    APS* lmcsAPS = nullptr;
    APS* scalinglistAPS = nullptr;
    activateAPS(&m_picHeader, m_apcSlicePilot, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    xParsePrefixSEImessages();

#if RExt__HIGH_BIT_DEPTH_SUPPORT==0
    if (sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() || sps->getBitDepth(CHANNEL_TYPE_LUMA)>12 || sps->getBitDepth(CHANNEL_TYPE_CHROMA)>12 )
    {
      THROW("High bit depth support must be enabled at compile-time in order to decode this bitstream\n");
    }
#endif

    //  Get a new picture buffer. This will also set up m_pcPic, and therefore give us a SPS and PPS pointer that we can use.
    m_pcPic = xGetNewPicBuffer( *sps, *pps, m_apcSlicePilot->getTLayer(), layerId );

    m_apcSlicePilot->applyReferencePictureListBasedMarking( m_cListPic, m_apcSlicePilot->getRPL0(), m_apcSlicePilot->getRPL1(), layerId, *pps);
    m_pcPic->finalInit( vps, *sps, *pps, &m_picHeader, apss, lmcsAPS, scalinglistAPS );
    m_pcPic->createTempBuffers( m_pcPic->cs->pps->pcv->maxCUWidth );
    m_pcPic->cs->createCoeffs((bool)m_pcPic->cs->sps->getPLTMode());

    m_pcPic->allocateNewSlice();
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    CHECK_(m_pcPic->slices.size() != (m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pcPic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    // we now have a real slice:
    Slice *pSlice = m_pcPic->slices[m_uiSliceSegmentIdx];

    // Update the PPS and SPS pointers with the ones of the picture.
    pps=pSlice->getPPS();
    sps=pSlice->getSPS();

    // fix Parameter Sets, now that we have the real slice
    m_pcPic->cs->slice = pSlice;
    m_pcPic->cs->sps   = sps;
    m_pcPic->cs->pps   = pps;
    m_pcPic->cs->vps = vps;

    memcpy(m_pcPic->cs->alfApss, apss, sizeof(m_pcPic->cs->alfApss));
    m_pcPic->cs->lmcsAps = lmcsAPS;
    m_pcPic->cs->scalinglistAps = scalinglistAPS;

    m_pcPic->cs->pcv   = pps->pcv;

    // Initialise the various objects for the new set of settings
    const int maxDepth = floorLog2(sps->getMaxCUWidth()) - pps->pcv->minCUWidthLog2;
    const uint32_t  log2SaoOffsetScaleLuma   = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_LUMA  ) - MAX_SAO_TRUNCATED_BITDEPTH);
    const uint32_t  log2SaoOffsetScaleChroma = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_CHROMA) - MAX_SAO_TRUNCATED_BITDEPTH);
    m_cSAO.create( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(),
                   sps->getChromaFormatIdc(),
                   sps->getMaxCUWidth(), sps->getMaxCUHeight(),
                   maxDepth,
                   log2SaoOffsetScaleLuma, log2SaoOffsetScaleChroma );
    m_cLoopFilter.create(maxDepth);
    m_cIntraPred.init( sps->getChromaFormatIdc(), sps->getBitDepth( CHANNEL_TYPE_LUMA ) );
    m_cInterPred.init( &m_cRdCost, sps->getChromaFormatIdc(), sps->getMaxCUHeight() );
    if (sps->getUseLmcs())
    {
      m_cReshaper.createDec(sps->getBitDepth(CHANNEL_TYPE_LUMA));
    }

    bool isField = false;
    bool isTopField = false;

    if(!m_SEIs.empty())
    {
      // Check if any new Frame Field Info SEI has arrived
      SEIMessages frameFieldSEIs = getSeisByType(m_SEIs, SEI::FRAME_FIELD_INFO);
      if (frameFieldSEIs.size()>0)
      {
        SEIFrameFieldInfo* ff = (SEIFrameFieldInfo*) *(frameFieldSEIs.begin());
        isField    = ff->m_fieldPicFlag;
        isTopField = isField && (!ff->m_bottomFieldFlag);
      }
      SEIMessages inclusionSEIs = getSeisByType(m_SEIs, SEI::PARAMETER_SETS_INCLUSION_INDICATION);
      const SEIParameterSetsInclusionIndication* inclusion = (inclusionSEIs.size() > 0) ? (SEIParameterSetsInclusionIndication*)*(inclusionSEIs.begin()) : NULL;
      if (inclusion != NULL)
      {
        m_seiInclusionFlag = inclusion->m_selfContainedClvsFlag;
      }
    }
    if (m_seiInclusionFlag)
    {
      checkParameterSetsInclusionSEIconstraints(nalu);
    }

    //Set Field/Frame coding mode
    m_pcPic->fieldPic = isField;
    m_pcPic->topField = isTopField;

    // transfer any SEI messages that have been received to the picture
    m_pcPic->SEIs = m_SEIs;
    m_SEIs.clear();

    // Recursive structure
    m_cCuDecoder.init( &m_cTrQuant, &m_cIntraPred, &m_cInterPred );
    if (sps->getUseLmcs())
    {
      m_cCuDecoder.initDecCuReshaper(&m_cReshaper, sps->getChromaFormatIdc());
    }
    m_cTrQuant.init(m_cTrQuantScalingList.getQuant(), sps->getMaxTbSize(), false, false, false, false);

    // RdCost
    m_cRdCost.setCostMode ( COST_STANDARD_LOSSY ); // not used in decoder side RdCost stuff -> set to default

    m_cSliceDecoder.create();

    if( sps->getALFEnabledFlag() )
    {
      const int maxDepth = floorLog2(sps->getMaxCUWidth()) - sps->getLog2MinCodingBlockSize();
      m_cALF.create( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), maxDepth, sps->getBitDepths().recon);
    }
    pSlice->m_ccAlfFilterControl[0] = m_cALF.getCcAlfControlIdc(COMPONENT_Cb);
    pSlice->m_ccAlfFilterControl[1] = m_cALF.getCcAlfControlIdc(COMPONENT_Cr);
  }
  else
  {
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    m_pcPic->allocateNewSlice();
    CHECK_(m_pcPic->slices.size() != (size_t)(m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pcPic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    Slice *pSlice = m_pcPic->slices[m_uiSliceSegmentIdx]; // we now have a real slice.

    const SPS *sps = pSlice->getSPS();
    const PPS *pps = pSlice->getPPS();
    APS** apss = pSlice->getAlfAPSs();
    APS *lmcsAPS = m_picHeader.getLmcsAPS();
    APS *scalinglistAPS = m_picHeader.getScalingListAPS();

    // fix Parameter Sets, now that we have the real slice
    m_pcPic->cs->slice = pSlice;
    m_pcPic->cs->sps   = sps;
    m_pcPic->cs->pps   = pps;
    memcpy(m_pcPic->cs->alfApss, apss, sizeof(m_pcPic->cs->alfApss));
    m_pcPic->cs->lmcsAps = lmcsAPS;
    m_pcPic->cs->scalinglistAps = scalinglistAPS;

    m_pcPic->cs->pcv   = pps->pcv;

    // check that the current active PPS has not changed...
    if (m_parameterSetManager.getSPSChangedFlag(sps->getSPSId()) )
    {
      EXIT("Error - a new SPS has been decoded while processing a picture");
    }
    if (m_parameterSetManager.getPPSChangedFlag(pps->getPPSId()) )
    {
      EXIT("Error - a new PPS has been decoded while processing a picture");
    }
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* aps = m_parameterSetManager.getAPS(i, ALF_APS);
      if (aps && m_parameterSetManager.getAPSChangedFlag(i, ALF_APS))
      {
        EXIT("Error - a new APS has been decoded while processing a picture");
      }
    }

    if (lmcsAPS && m_parameterSetManager.getAPSChangedFlag(lmcsAPS->getAPSId(), LMCS_APS) )
    {
      EXIT("Error - a new LMCS APS has been decoded while processing a picture");
    }
    if( scalinglistAPS && m_parameterSetManager.getAPSChangedFlag( scalinglistAPS->getAPSId(), SCALING_LIST_APS ) )
    {
      EXIT( "Error - a new SCALING LIST APS has been decoded while processing a picture" );
    }

    activateAPS(&m_picHeader, pSlice, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    m_pcPic->cs->lmcsAps = lmcsAPS;
    m_pcPic->cs->scalinglistAps = scalinglistAPS;

    xParsePrefixSEImessages();

    // Check if any new SEI has arrived
     if(!m_SEIs.empty())
     {
       // Currently only decoding Unit SEI message occurring between VCL NALUs copied
       SEIMessages& picSEI = m_pcPic->SEIs;
       SEIMessages decodingUnitInfos = extractSeisByType( picSEI, SEI::DECODING_UNIT_INFO);
       picSEI.insert(picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end());
       deleteSEIs(m_SEIs);
     }
     if (m_seiInclusionFlag)
     {
       checkParameterSetsInclusionSEIconstraints(nalu);
     }
  }
  xCheckParameterSetConstraints(layerId);
}
void DecLib::xCheckParameterSetConstraints(const int layerId)
{
  // Conformance checks
  Slice *slice = m_pcPic->slices[m_uiSliceSegmentIdx];
  const SPS *sps = slice->getSPS();
  const PPS *pps = slice->getPPS();
  const VPS *vps = slice->getVPS();
  
  if (sps->getVPSId() && (vps != nullptr))
  {
    if ((layerId == vps->getLayerId(0)) && m_firstSliceInSequence[layerId])
    {
      m_clsVPSid = sps->getVPSId();
    }
    CHECK_(m_clsVPSid != sps->getVPSId(), "The value of sps_video_parameter_set_id shall be the same in all SPSs that are referred to by CLVSs in a CVS.");
  }

  if (((vps!=nullptr)&&(vps->getVPSGeneralHrdParamsPresentFlag()))||(sps->getGeneralHrdParametersPresentFlag()))
  {
    if (((vps != nullptr) && (vps->getVPSGeneralHrdParamsPresentFlag())) && (sps->getGeneralHrdParametersPresentFlag()))
    {
      CHECK_(!(*vps->getGeneralHrdParameters() == *sps->getGeneralHrdParameters()), "It is a requirement of bitstream conformance that the content of the general_hrd_parameters( ) syntax structure present in any VPSs or SPSs in the bitstream shall be identical");
    }
    if (!m_isFirstGeneralHrd)
    {
      CHECK_(!(m_prevGeneralHrdParams == (sps->getGeneralHrdParametersPresentFlag() ? *sps->getGeneralHrdParameters() : *vps->getGeneralHrdParameters())), "It is a requirement of bitstream conformance that the content of the general_hrd_parameters( ) syntax structure present in any VPSs or SPSs in the bitstream shall be identical");
    }
    m_prevGeneralHrdParams = (sps->getGeneralHrdParametersPresentFlag() ? *sps->getGeneralHrdParameters() : *vps->getGeneralHrdParameters());
  }
  m_isFirstGeneralHrd = false;
  static std::unordered_map<int, int> m_clvssSPSid;

  if( slice->isClvssPu() && m_bFirstSliceInPicture )
  {
    m_clvssSPSid[layerId] = pps->getSPSId();
  }

  CHECK_( m_clvssSPSid[layerId] != pps->getSPSId(), "The value of pps_seq_parameter_set_id shall be the same in all PPSs that are referred to by coded pictures in a CLVS" );

  CHECK_(sps->getGDREnabledFlag() == false && m_picHeader.getGdrPicFlag(), "When sps_gdr_enabled_flag is equal to 0, the value of ph_gdr_pic_flag shall be equal to 0 ");
  if( !sps->getUseWP() )
  {
    CHECK_( pps->getUseWP(), "When sps_weighted_pred_flag is equal to 0, the value of pps_weighted_pred_flag shall be equal to 0." );
  }

  if( !sps->getUseWPBiPred() )
  {
    CHECK_( pps->getWPBiPred(), "When sps_weighted_bipred_flag is equal to 0, the value of pps_weighted_bipred_flag shall be equal to 0." );
  }

  const int minCuSize = 1 << sps->getLog2MinCodingBlockSize();
  CHECK_( ( pps->getPicWidthInLumaSamples() % ( std::max( 8, minCuSize) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK_( ( pps->getPicHeightInLumaSamples() % ( std::max( 8, minCuSize) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );
  if (!sps->getResChangeInClvsEnabledFlag())
  {
    CHECK_(pps->getPicWidthInLumaSamples() != sps->getMaxPicWidthInLumaSamples(), "When sps_res_change_in_clvs_allowed_flag equal to 0, the value of pps_pic_width_in_luma_samples shall be equal to sps_pic_width_max_in_luma_samples.");
    CHECK_(pps->getPicHeightInLumaSamples() != sps->getMaxPicHeightInLumaSamples(), "When sps_res_change_in_clvs_allowed_flag equal to 0, the value of pps_pic_height_in_luma_samples shall be equal to sps_pic_height_max_in_luma_samples.");
  }
  if (sps->getResChangeInClvsEnabledFlag())
  {
    CHECK_(sps->getSubPicInfoPresentFlag() != 0, "When sps_res_change_in_clvs_allowed_flag is equal to 1, the value of sps_subpic_info_present_flag shall be equal to 0.");
  }
  CHECK_(sps->getResChangeInClvsEnabledFlag() && sps->getVirtualBoundariesEnabledFlag(), "when the value of sps_res_change_in_clvs_allowed_flag is equal to 1, the value of sps_virtual_boundaries_present_flag shall be equal to 0");

  if( sps->getCTUSize() + 2 * ( 1 << sps->getLog2MinCodingBlockSize() ) > pps->getPicWidthInLumaSamples() )
  {
    CHECK_( pps->getWrapAroundEnabledFlag(), "Wraparound shall be disabled when the value of ( CtbSizeY / MinCbSizeY + 1) is greater than or equal to ( pps_pic_width_in_luma_samples / MinCbSizeY - 1 )" );
  }

  if( vps != nullptr && vps->m_numOutputLayersInOls[vps->m_targetOlsIdx] > 1 )
  {
    CHECK_( sps->getMaxPicWidthInLumaSamples() > vps->getOlsDpbPicSize( vps->m_targetOlsIdx ).width, "sps_pic_width_max_in_luma_samples shall be less than or equal to the value of vps_ols_dpb_pic_width[ i ]" );
    CHECK_( sps->getMaxPicHeightInLumaSamples() > vps->getOlsDpbPicSize( vps->m_targetOlsIdx ).height, "sps_pic_height_max_in_luma_samples shall be less than or equal to the value of vps_ols_dpb_pic_height[ i ]" );
    CHECK_( sps->getChromaFormatIdc() > vps->getOlsDpbChromaFormatIdc( vps->m_targetOlsIdx ), "sps_chroma_format_idc shall be less than or equal to the value of vps_ols_dpb_chroma_format[ i ]");
    CHECK_((sps->getBitDepth(CHANNEL_TYPE_LUMA) - 8) > vps->getOlsDpbBitDepthMinus8(vps->m_targetOlsIdx),
          "sps_bitdepth_minus8 shall be less than or equal to the value of vps_ols_dpb_bitdepth_minus8[ i ]");
  }

  static std::unordered_map<int, int> m_layerChromaFormat;
  static std::unordered_map<int, int> m_layerBitDepth;

  if (vps != nullptr && vps->getMaxLayers() > 1)
  {
    int curLayerIdx = vps->getGeneralLayerIdx(layerId);
    int curLayerChromaFormat = sps->getChromaFormatIdc();
    int curLayerBitDepth = sps->getBitDepth(CHANNEL_TYPE_LUMA);

    if( slice->isClvssPu() && m_bFirstSliceInPicture )
    {
      m_layerChromaFormat[curLayerIdx] = curLayerChromaFormat;
      m_layerBitDepth[curLayerIdx] = curLayerBitDepth;
    }
    else
    {
      CHECK_(m_layerChromaFormat[curLayerIdx] != curLayerChromaFormat, "Different chroma format in the same layer.");
      CHECK_(m_layerBitDepth[curLayerIdx] != curLayerBitDepth, "Different bit-depth in the same layer.");
    }

    for (int i = 0; i < curLayerIdx; i++)
    {
      if (vps->getDirectRefLayerFlag(curLayerIdx, i))
      {
        int refLayerChromaFormat = m_layerChromaFormat[i];
        CHECK_(curLayerChromaFormat != refLayerChromaFormat, "The chroma formats of the current layer and the reference layer are different");
        int refLayerBitDepth = m_layerBitDepth[i];
        CHECK_(curLayerBitDepth != refLayerBitDepth, "The bit-depth of the current layer and the reference layer are different");
      }
    }
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getOneTilePerPicConstraintFlag())
  {
    CHECK_(pps->getNumTiles() != 1, "When one_tile_per_pic_constraint_flag is equal to 1, each picture shall contain only one tile");
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getOneSlicePerPicConstraintFlag())
  {
    CHECK_( pps->getRectSliceFlag() && pps->getNumSlicesInPic() != 1, "When one_slice_per_pic_constraint_flag is equal to 1 and if pps_rect_slice_flag is equal to 1, the value of pps_num_slices_in_pic_minus1 shall be equal to 0");
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoRprConstraintFlag())
  {
    CHECK_(sps->getRprEnabledFlag(), "When gci_no_ref_pic_resampling_constraint_flag is equal to 1, the value of sps_ref_pic_resampling_enabled_flag shall be equal to 0");
  }
  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoResChangeInClvsConstraintFlag())
  {
    CHECK_(sps->getResChangeInClvsEnabledFlag(), "When gci_no_res_change_in_clvs_constraint_flag is equal to 1, the value of sps_res_change_in_clvs_allowed_flag shall be equal to 0");
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoIdrRplConstraintFlag())
  {
    CHECK_(sps->getIDRRefParamListPresent(), "When gci_no_idr_rpl_constraint_flag equal to 1 , the value of sps_idr_rpl_present_flag shall be equal to 0")
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoMixedNaluTypesInPicConstraintFlag())
  {
    CHECK_(pps->getMixedNaluTypesInPicFlag(), "When gci_no_mixed_nalu_types_in_pic_constraint_flag equal to 1, the value of pps_mixed_nalu_types_in_pic_flag shall be equal to 0")
  }

#if JVET_R0266_GCI
  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoGdrConstraintFlag())
  {
    CHECK_(sps->getGDREnabledFlag(), "gci_no_gdr_constraint_flag equal to 1 specifies that sps_gdr_enabled_flag for all pictures in OlsInScope shall be equal to 0");
  }
#endif

  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoRectSliceConstraintFlag())
  {
    CHECK_(pps->getRectSliceFlag(), "When gci_no_rectangular_slice_constraint_flag equal to 1, the value of pps_rect_slice_flag shall be equal to 0")
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getOneSlicePerSubpicConstraintFlag())
  {
    CHECK_(!(pps->getSingleSlicePerSubPicFlag()), "When gci_one_slice_per_subpic_constraint_flag equal to 1, the value of pps_single_slice_per_subpic_flag shall be equal to 1")
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoSubpicInfoConstraintFlag())
  {
    CHECK_(sps->getSubPicInfoPresentFlag(), "When gci_no_subpic_info_constraint_flag is equal to 1, the value of sps_subpic_info_present_flag shall be equal to 0")
  }
  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoMttConstraintFlag())
  {
    CHECK_((sps->getMaxMTTHierarchyDepth() || sps->getMaxMTTHierarchyDepthI() || sps->getMaxMTTHierarchyDepthIChroma()), "When gci_no_mtt_constraint_flag is equal to 1, the values of sps_max_mtt_hierarchy_depth_intra_slice_luma, sps_max_mtt_hierarchy_depth_inter_slice and sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be equal to 0");
  }
  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoWeightedPredictionConstraintFlag())
  {
    CHECK_((sps->getUseWP() || sps->getUseWPBiPred()), "When gci_no_weighted_prediction_constraint_flag is equal to 1, the values of sps_weighted_pred_flag and sps_weighted_bipred_flag shall be equal to 0");
  }

  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoChromaQpOffsetConstraintFlag())
  {
    CHECK_((pps->getCuChromaQpOffsetListEnabledFlag()), "When gci_no_ChromaQpOffset_constraint_flag is equal to 1, the values of pps_cu_chroma_qp_offset_list_enabled_flag shall be equal to 0");
  }

  CHECK_(sps->getCTUSize() > (1 << sps->getProfileTierLevel()->getConstraintInfo()->getMaxLog2CtuSizeConstraintIdc()), "The CTU size specified by sps_log2_ctu_size_minus5 shall not exceed the constraint specified by gci_three_minus_max_log2_ctu_size_constraint_idc");

  if (sps->getProfileTierLevel()->getConstraintInfo()->getNoLumaTransformSize64ConstraintFlag())
  {
    CHECK_(sps->getLog2MaxTbSize() != 5, "When gci_no_luma_transform_size_64_constraint_flag is equal to 1, the value of sps_max_luma_transform_size_64_flag shall be equal to 0");
  }

  if (sps->getMaxPicWidthInLumaSamples() == pps->getPicWidthInLumaSamples() &&
      sps->getMaxPicHeightInLumaSamples() == pps->getPicHeightInLumaSamples())
  {
    const Window& spsConfWin = sps->getConformanceWindow();
    const Window& ppsConfWin = pps->getConformanceWindow();
    CHECK_(spsConfWin.getWindowLeftOffset() != ppsConfWin.getWindowLeftOffset(), "When picture size is equal to maximum picutre size, conformance window left offset in SPS and PPS shall be equal");
    CHECK_(spsConfWin.getWindowRightOffset() != ppsConfWin.getWindowRightOffset(), "When picture size is equal to maximum picutre size, conformance window right offset in SPS and PPS shall be equal");
    CHECK_(spsConfWin.getWindowTopOffset() != ppsConfWin.getWindowTopOffset(), "When picture size is equal to maximum picutre size, conformance window top offset in SPS and PPS shall be equal");
    CHECK_(spsConfWin.getWindowBottomOffset() != ppsConfWin.getWindowBottomOffset(), "When picture size is equal to maximum picutre size, conformance window bottom offset in SPS and PPS shall be equal");
  }
  int levelIdcSps = int(sps->getProfileTierLevel()->getLevelIdc());
  int maxLevelIdxDci = 0;
  if (m_dci)
  {
    for (int i = 0; i < m_dci->getNumPTLs(); i++)
    {
      if (maxLevelIdxDci < int(m_dci->getProfileTierLevel(i).getLevelIdc()))
      {
        maxLevelIdxDci = int(m_dci->getProfileTierLevel(i).getLevelIdc());
      }
    }
    CHECK_(levelIdcSps > maxLevelIdxDci, "max level signaled in the DCI shall not be less than the level signaled in the SPS");
  }


  if( slice->getPicHeader()->getGdrOrIrapPicFlag() && !slice->getPicHeader()->getGdrPicFlag() && ( !vps || vps->getIndependentLayerFlag( vps->getGeneralLayerIdx( layerId ) ) ) )
  {
    CHECK_( slice->getPicHeader()->getPicInterSliceAllowedFlag(),
      "When ph_gdr_or_irap_pic_flag is equal to 1 and ph_gdr_pic_flag is equal to 0 and vps_independent_layer_flag[ GeneralLayerIdx[ nuh_layer_id ] ] is equal to 1, ph_inter_slice_allowed_flag shall be equal to 0" );
  }

  if( sps->getVPSId() && vps->m_numLayersInOls[vps->m_targetOlsIdx] == 1 )
  {
    CHECK_( !sps->getPtlDpbHrdParamsPresentFlag(), "When sps_video_parameter_set_id is greater than 0 and there is an OLS that contains only one layer with nuh_layer_id equal to the nuh_layer_id of the SPS, the value of sps_ptl_dpb_hrd_params_present_flag shall be equal to 1" );
  }

  ProfileLevelTierFeatures ptlFeatures;
  ptlFeatures.extractPTLInformation(*sps);
  const ProfileFeatures *profileFeatures = ptlFeatures.getProfileFeatures();
  if (profileFeatures != nullptr)
  {
    CHECK_(sps->getBitDepth(CHANNEL_TYPE_LUMA) > profileFeatures->maxBitDepth, "Bit depth exceeds profile limit");
    CHECK_(sps->getChromaFormatIdc() > profileFeatures->maxChromaFormat, "Chroma format exceeds profile limit");
  }
  else
  {
    CHECK_(sps->getProfileTierLevel()->getProfileIdc() != Profile::NONE, "Unknown profile");
    msg(WARNING_, "Warning: Profile set to none or unknown value\n");
  }
  const LevelTierFeatures *levelTierFeatures = ptlFeatures.getLevelTierFeatures();
  if (levelTierFeatures != nullptr)
  {
    CHECK_(pps->getNumTileColumns() > levelTierFeatures->maxTileCols,
          "Number of tile columns signaled in PPS exceeds level limit");
    CHECK_(pps->getNumTiles() > levelTierFeatures->maxTilesPerAu, "Number of tiles signaled in PPS exceeds level limit");
  }
  else if (profileFeatures != nullptr)
  {
    CHECK_(sps->getProfileTierLevel()->getLevelIdc() == Level::LEVEL15_5, "Cannot use level 15.5 with given profile");
    CHECK_(sps->getProfileTierLevel()->getLevelIdc() != Level::NONE, "Unknown level");
    msg(WARNING_, "Warning: Level set to none, invalid or unknown value\n");
  }
}


void DecLib::xParsePrefixSEIsForUnknownVCLNal()
{
  while (!m_prefixSEINALUs.empty())
  {
    // do nothing?
    msg(NOTICE_, "Discarding Prefix SEI associated with unknown VCL NAL unit.\n");
    delete m_prefixSEINALUs.front();
  }
  // TODO: discard following suffix SEIs as well?
}


void DecLib::xParsePrefixSEImessages()
{
  while (!m_prefixSEINALUs.empty())
  {
    InputNALUnit &nalu=*m_prefixSEINALUs.front();
    m_accessUnitSeiTids.push_back(nalu.m_temporalId);
    const SPS *sps = m_parameterSetManager.getActiveSPS();
    const VPS *vps = m_parameterSetManager.getVPS(sps->getVPSId());
    m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_SEIs, nalu.m_nalUnitType, nalu.m_nuhLayerId, nalu.m_temporalId, vps, sps, m_HRD, m_pDecodedSEIOutputStream );
#if JVET_S0257_DUMP_360SEI_MESSAGE
    m_seiCfgDump.write360SeiDump( m_decoded360SeiDumpFileName, m_SEIs, sps );
#endif
    m_accessUnitSeiPayLoadTypes.push_back(std::tuple<NalUnitType, int, SEI::PayloadType>(nalu.m_nalUnitType, nalu.m_nuhLayerId, m_SEIs.back()->payloadType()));
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
  xCheckPrefixSEIMessages(m_SEIs);
}

void DecLib::xCheckPrefixSEIMessages( SEIMessages& prefixSEIs )
{
  SEIMessages picTimingSEIs  = getSeisByType(prefixSEIs, SEI::PICTURE_TIMING);
  SEIMessages frameFieldSEIs = getSeisByType(prefixSEIs, SEI::FRAME_FIELD_INFO);

  if (!picTimingSEIs.empty() && !frameFieldSEIs.empty())
  {
    SEIPictureTiming  *pt = (SEIPictureTiming*)  picTimingSEIs.front();
    SEIFrameFieldInfo *ff = (SEIFrameFieldInfo*) frameFieldSEIs.front();
    if( pt->m_ptDisplayElementalPeriodsMinus1 != ff->m_displayElementalPeriodsMinus1 )
    {
      msg(WARNING_, "Warning: ffi_display_elemental_periods_minus1 is different in picture timing and frame field information SEI messages!");
    }
  }
}

void DecLib::xDecodePicHeader( InputNALUnit& nalu )
{
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePictureHeader( &m_picHeader, &m_parameterSetManager, true );
  m_picHeader.setValid();
}

bool DecLib::xDecodeSlice(InputNALUnit &nalu, int &iSkipFrame, int iPOCLastDisplay )
{
  m_apcSlicePilot->setPicHeader( &m_picHeader );
  m_apcSlicePilot->initSlice(); // the slice pilot is an object to prepare for a new slice
                                // it is not associated with picture, sps or pps structures.

  Picture* scaledRefPic[MAX_NUM_REF] = {};

  if (m_bFirstSliceInPicture)
  {
    m_uiSliceSegmentIdx = 0;
  }
  else
  {
    CHECK_(nalu.m_nalUnitType != m_pcPic->slices[m_uiSliceSegmentIdx - 1]->getNalUnitType() && !m_pcPic->cs->pps->getMixedNaluTypesInPicFlag(), "If pps_mixed_nalu_types_in_pic_flag is equal to 0, the value of NAL unit type shall be the same for all coded slice NAL units of a picture");
    m_apcSlicePilot->copySliceInfo( m_pcPic->slices[m_uiSliceSegmentIdx-1] );
  }

  m_apcSlicePilot->setNalUnitType(nalu.m_nalUnitType);
  m_apcSlicePilot->setNalUnitLayerId(nalu.m_nuhLayerId);
  m_apcSlicePilot->setTLayer(nalu.m_temporalId);

  for( auto& naluTemporalId : m_accessUnitNals )
  {
    if (
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
      naluTemporalId.m_nalUnitType != NAL_UNIT_OPI &&
#endif
      naluTemporalId.m_nalUnitType != NAL_UNIT_DCI
      && naluTemporalId.m_nalUnitType != NAL_UNIT_VPS
      && naluTemporalId.m_nalUnitType != NAL_UNIT_SPS
      && naluTemporalId.m_nalUnitType != NAL_UNIT_EOS
      && naluTemporalId.m_nalUnitType != NAL_UNIT_EOB)

    {
      CHECK_( naluTemporalId.m_temporalId < nalu.m_temporalId, "TemporalId shall be greater than or equal to the TemporalId of the layer access unit containing the NAL unit" );
    }
  }

  if (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR)
  {
    CHECK_(nalu.m_temporalId != 0, "Current GDR picture has TemporalId not equal to 0");
  }

  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_apcSlicePilot->m_ccAlfFilterParam = m_cALF.getCcAlfFilterParam();
  m_HLSReader.parseSliceHeader( m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC, m_prevPicPOC );

  if (m_picHeader.getGdrOrIrapPicFlag() && m_bFirstSliceInPicture)
  {
    m_accessUnitNoOutputPriorPicFlags.push_back(m_apcSlicePilot->getNoOutputOfPriorPicsFlag());
  }

  PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId());
  CHECK_(pps == 0, "No PPS present");
  SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());
  CHECK_(sps == 0, "No SPS present");
  VPS *vps = m_parameterSetManager.getVPS(sps->getVPSId());


  if (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA && vps != nullptr && (vps->getIndependentLayerFlag(vps->getGeneralLayerIdx(nalu.m_nuhLayerId)) == 1))
  {
    CHECK_(nalu.m_temporalId == 0, "TemporalID of STSA picture shall not be zero in independent layers");
  }

  int currSubPicIdx = pps->getSubPicIdxFromSubPicId( m_apcSlicePilot->getSliceSubPicId() );
  int currSliceAddr = m_apcSlicePilot->getSliceID();
  for(int sp = 0; sp < currSubPicIdx; sp++)
  {
    currSliceAddr -= pps->getSubPic(sp).getNumSlicesInSubPic();
  }
  CHECK_( currSubPicIdx < m_maxDecSubPicIdx, "Error in the order of coded slice NAL units of subpictures" );
  CHECK_( currSubPicIdx == m_maxDecSubPicIdx && currSliceAddr <= m_maxDecSliceAddrInSubPic, "Error in the order of coded slice NAL units within a subpicture" );
  if( currSubPicIdx == m_maxDecSubPicIdx )
  {
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
  if( currSubPicIdx > m_maxDecSubPicIdx )
  {
    m_maxDecSubPicIdx = currSubPicIdx;
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
  if ((sps->getVPSId() == 0) && (m_prevLayerID != MAX_INT))
  {
    CHECK_(m_prevLayerID != nalu.m_nuhLayerId, "All VCL NAL unit in the CVS shall have the same value of nuh_layer_id "
                                              "when sps_video_parameter_set_id is equal to 0");
  }
  CHECK_((sps->getVPSId() > 0) && (vps == 0), "Invalid VPS");

  if ((sps->getProfileTierLevel()->getMultiLayerEnabledFlag() == 0) && (m_prevLayerID != MAX_INT))
  {
    CHECK_(m_prevLayerID != nalu.m_nuhLayerId, "All slices in OlsInScope shall have the same value of nuh_layer_id when ptl_multilayer_enabled_flag is equal to 0" );
  }

  if( vps != nullptr && !vps->getIndependentLayerFlag( vps->getGeneralLayerIdx( nalu.m_nuhLayerId ) ) )
  {
    bool pocIsSet = false;
    for(auto auNALit=m_accessUnitPicInfo.begin(); auNALit != m_accessUnitPicInfo.end();auNALit++)
    {
      for (int iRefIdx = 0; iRefIdx < m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_0) && !pocIsSet; iRefIdx++)
      {
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx) && m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() == (*auNALit).m_POC)
        {
          m_apcSlicePilot->setPOC(m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC());
          pocIsSet = true;
        }
      }
      for (int iRefIdx = 0; iRefIdx < m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_1) && !pocIsSet; iRefIdx++)
      {
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx) && m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() == (*auNALit).m_POC)
        {
          m_apcSlicePilot->setPOC(m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC());
          pocIsSet = true;
        }
      }
    }
  }

  // update independent slice index
  uint32_t uiIndependentSliceIdx = 0;
  if (!m_bFirstSliceInPicture)
  {
    uiIndependentSliceIdx = m_pcPic->slices[m_uiSliceSegmentIdx-1]->getIndependentSliceIdx();
    uiIndependentSliceIdx++;
  }
  m_apcSlicePilot->setIndependentSliceIdx(uiIndependentSliceIdx);

#if K0149_BLOCK_STATISTICS
  writeBlockStatisticsHeader(sps);
#endif

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "poc", m_apcSlicePilot->getPOC() ) );

  if ((m_bFirstSliceInPicture ||
        m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ||
        m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) &&
      getNoOutputPriorPicsFlag())
    {
      checkNoOutputPriorPics(&m_cListPic);
      setNoOutputPriorPicsFlag (false);
    }

  xUpdatePreviousTid0POC(m_apcSlicePilot);

  m_apcSlicePilot->setPrevGDRInSameLayerPOC(m_prevGDRInSameLayerPOC[nalu.m_nuhLayerId]);
  m_apcSlicePilot->setAssociatedIRAPPOC(m_pocCRA[nalu.m_nuhLayerId]);
  m_apcSlicePilot->setAssociatedIRAPType(m_associatedIRAPType[nalu.m_nuhLayerId]);

  if( m_apcSlicePilot->getRapPicFlag() || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR )
  {
    // Derive NoOutputBeforeRecoveryFlag
    if( !pps->getMixedNaluTypesInPicFlag() )
    {
      if( m_firstSliceInSequence[nalu.m_nuhLayerId] )
      {
        m_picHeader.setNoOutputBeforeRecoveryFlag( true );
      }
      else if( m_apcSlicePilot->getIdrPicFlag() )
      {
        m_picHeader.setNoOutputBeforeRecoveryFlag( true );
      }
      else if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )
      {
        m_picHeader.setNoOutputBeforeRecoveryFlag( m_picHeader.getHandleCraAsCvsStartFlag() );
      }
      else if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR )
      {
        m_picHeader.setNoOutputBeforeRecoveryFlag( m_picHeader.getHandleGdrAsCvsStartFlag() );
      }
    }

    //the inference for NoOutputOfPriorPicsFlag
    if( !m_firstSliceInBitstream && m_picHeader.getNoOutputBeforeRecoveryFlag() )
    {
      m_apcSlicePilot->setNoOutputOfPriorPicsFlag(true);
    }
    else
    {
      m_apcSlicePilot->setNoOutputOfPriorPicsFlag(false);
    }

    if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR)
    {
      m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] = m_picHeader.getNoOutputBeforeRecoveryFlag();
    }

    if (m_apcSlicePilot->getNoOutputOfPriorPicsFlag())
    {
      m_lastPOCNoOutputPriorPics = m_apcSlicePilot->getPOC();
      m_isNoOutputPriorPics = true;
    }
    else
    {
      m_isNoOutputPriorPics = false;
    }
  }

  //For inference of PicOutputFlag
  if( !pps->getMixedNaluTypesInPicFlag() && ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL ) )
  {
    if( m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] )
    {
      m_picHeader.setPicOutputFlag(false);
    }
  }

  {
    PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId());
    CHECK_(pps == 0, "No PPS present");
    SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());
    CHECK_(sps == 0, "No SPS present");
    if (sps->getVPSId() > 0)
    {
      VPS *vps = m_parameterSetManager.getVPS(sps->getVPSId());
      CHECK_(vps == 0, "No VPS present");
      bool isCurLayerNotOutput = true;
      for (int i = 0; i < vps->getNumLayersInOls(vps->m_targetOlsIdx); i++)
      {
        if( vps->getLayerIdInOls(vps->m_targetOlsIdx, i) == nalu.m_nuhLayerId )
        {
            isCurLayerNotOutput = false;
            break;
        }
      }

      if(isCurLayerNotOutput)
      {
        m_picHeader.setPicOutputFlag(false);
      }
    }
  }

  //Reset POC MSB when CRA or GDR has NoOutputBeforeRecoveryFlag equal to 1
  if (!pps->getMixedNaluTypesInPicFlag() && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) && m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId])
  {
    int iMaxPOClsb = 1 << sps->getBitsForPOC();
    m_apcSlicePilot->setPOC( m_apcSlicePilot->getPOC() & (iMaxPOClsb - 1) );
    xUpdatePreviousTid0POC(m_apcSlicePilot);
  }

  AccessUnitPicInfo picInfo;
  picInfo.m_nalUnitType = nalu.m_nalUnitType;
  picInfo.m_nuhLayerId  = nalu.m_nuhLayerId;
  picInfo.m_temporalId  = nalu.m_temporalId;
  picInfo.m_POC         = m_apcSlicePilot->getPOC();
  m_accessUnitPicInfo.push_back(picInfo);

  // Skip pictures due to random access

  if (isRandomAccessSkipPicture(iSkipFrame, iPOCLastDisplay))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC = m_apcSlicePilot->getPOC();
    // reset variables for bitstream conformance tests
    resetAccessUnitNals();
    resetAccessUnitApsNals();
    resetAccessUnitPicInfo();
    m_maxDecSubPicIdx = 0;
    m_maxDecSliceAddrInSubPic = -1;
    return false;
  }
  // Skip TFD pictures associated with BLA/BLANT pictures

  // clear previous slice skipped flag
  m_prevSliceSkipped = false;

  //we should only get a different poc for a new picture (with CTU address==0)
  if (m_apcSlicePilot->getPOC() != m_prevPOC && !m_firstSliceInSequence[nalu.m_nuhLayerId] && (m_apcSlicePilot->getFirstCtuRsAddrInSlice() != 0))
  {
    msg(WARNING_, "Warning, the first slice of a picture might have been lost!\n");
  }
  m_prevLayerID = nalu.m_nuhLayerId;

  // leave when a new picture is found
  if(m_apcSlicePilot->getFirstCtuRsAddrInSlice() == 0 && !m_bFirstSliceInPicture)
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 0 ) );
      m_prevPOC = m_apcSlicePilot->getPOC();
      return true;
    }
    m_prevPOC = m_apcSlicePilot->getPOC();
  }
  else
  {
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
  }

  //detect lost reference picture and insert copy of earlier frame.
  {
    int lostPoc;
    int refPicIndex;
    while ((lostPoc = m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPL0(), 0, true, &refPicIndex, m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_0))) > 0)
    {
      if( !pps->getMixedNaluTypesInPicFlag() && ( 
      ( ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ) && ( sps->getIDRRefParamListPresent() || pps->getRplInfoInPhFlag() ) ) ||
        ( ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) && m_picHeader.getNoOutputBeforeRecoveryFlag() ) ) ) 
      {
        if (m_apcSlicePilot->getRPL0()->isInterLayerRefPic(refPicIndex) == 0)
        {
          xCreateUnavailablePicture( m_apcSlicePilot->getPPS(), lostPoc - 1, m_apcSlicePilot->getRPL0()->isRefPicLongterm( refPicIndex ), m_apcSlicePilot->getPic()->temporalId, m_apcSlicePilot->getPic()->layerId, m_apcSlicePilot->getRPL0()->isInterLayerRefPic( refPicIndex ) );
        }
      }
      else
      {
        xCreateLostPicture( lostPoc - 1, m_apcSlicePilot->getPic()->layerId );
      }
    }
    while ((lostPoc = m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPL1(), 0, true, &refPicIndex, m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_1))) > 0)
    {
      if( !pps->getMixedNaluTypesInPicFlag() && ( 
        ( ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ) && ( sps->getIDRRefParamListPresent() || pps->getRplInfoInPhFlag() ) ) ||
        ( ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) && m_picHeader.getNoOutputBeforeRecoveryFlag() ) ) )
      {
        if (m_apcSlicePilot->getRPL1()->isInterLayerRefPic(refPicIndex) == 0)
        {
          xCreateUnavailablePicture( m_apcSlicePilot->getPPS(), lostPoc - 1, m_apcSlicePilot->getRPL1()->isRefPicLongterm( refPicIndex ), m_apcSlicePilot->getPic()->temporalId, m_apcSlicePilot->getPic()->layerId, m_apcSlicePilot->getRPL1()->isInterLayerRefPic( refPicIndex ) );
        }
      }
      else
      {
        xCreateLostPicture( lostPoc - 1, m_apcSlicePilot->getPic()->layerId );
      }
    }
  }

    m_prevPOC = m_apcSlicePilot->getPOC();

  if (m_bFirstSliceInPicture)
  {
    xUpdateRasInit(m_apcSlicePilot);
  }

  // actual decoding starts here
  xActivateParameterSets( nalu );

  m_firstSliceInSequence[nalu.m_nuhLayerId] = false;
  m_firstSliceInBitstream  = false;

  Slice* pcSlice = m_pcPic->slices[m_uiSliceSegmentIdx];
  m_pcPic->numSlices = m_uiSliceSegmentIdx + 1;
  pcSlice->setPic( m_pcPic );
  m_pcPic->poc         = pcSlice->getPOC();
  m_pcPic->referenced  = true;
  m_pcPic->temporalId  = nalu.m_temporalId;
  m_pcPic->layerId     = nalu.m_nuhLayerId;
  m_pcPic->subLayerNonReferencePictureDueToSTSA = false;

  if (pcSlice->getSPS()->getProfileTierLevel()->getConstraintInfo()->getNoApsConstraintFlag())
  {
    bool flag = pcSlice->getSPS()->getCCALFEnabledFlag() || pcSlice->getPicHeader()->getNumAlfAps() || pcSlice->getPicHeader()->getAlfEnabledFlag(COMPONENT_Cb) || pcSlice->getPicHeader()->getAlfEnabledFlag(COMPONENT_Cr);
    CHECK_(flag, "When no_aps_constraint_flag is equal to 1, the values of ph_num_alf_aps_ids_luma, sh_num_alf_aps_ids_luma, ph_alf_cb_flag, ph_alf_cr_flag, sh_alf_cb_flag, sh_alf_cr_flag, and sps_ccalf_enabled_flag shall all be equal to 0")
  }
  if( pcSlice->getNalUnitLayerId() != pcSlice->getSPS()->getLayerId() )
  {
    CHECK_( pcSlice->getSPS()->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of SPS cannot be greater than layer Id of VCL NAL unit the refer to it" );
    CHECK_( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of SPS and layer Id of current slice are different" );
    for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
    {
      bool isCurrLayerInOls = false;
      bool isRefLayerInOls = false;
      int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1;
      for (; j >= 0; j--)
      {
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
        {
          isCurrLayerInOls = true;
        }
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getSPS()->getLayerId() )
        {
          isRefLayerInOls = true;
        }
      }
      CHECK_( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to SPS in layer B, all OLS that contains layer A shall also contains layer B" );
    }
  }
  if( pcSlice->getNalUnitLayerId() != pcSlice->getPPS()->getLayerId() )
  {
    CHECK_( pcSlice->getPPS()->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of PPS cannot be greater than layer Id of VCL NAL unit the refer to it" );
    CHECK_( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of PPS and layer Id of current slice are different" );
    for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
    {
      bool isCurrLayerInOls = false;
      bool isRefLayerInOls = false;
      int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1;
      for (; j >= 0; j--)
      {
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
        {
          isCurrLayerInOls = true;
        }
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getPPS()->getLayerId() )
        {
          isRefLayerInOls = true;
        }
      }
      CHECK_( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to PPS in layer B, all OLS that contains layer A shall also contains layer B" );
    }
  }

  if (m_bFirstSliceInPicture)
  {
    m_pcPic->setDecodingOrderNumber(m_decodingOrderCounter);
    m_decodingOrderCounter++;
    m_pcPic->setPictureType(nalu.m_nalUnitType);
    checkPicTypeAfterEos();
    // store sub-picture numbers, sizes, and locations with a picture
    pcSlice->getPic()->subPictures.clear();

    for( int subPicIdx = 0; subPicIdx < sps->getNumSubPics(); subPicIdx++ )
    {
      pcSlice->getPic()->subPictures.push_back( pps->getSubPic( subPicIdx ) );
    }
    pcSlice->getPic()->numSlices = pps->getNumSlicesInPic();
    pcSlice->getPic()->sliceSubpicIdx.clear();
  }
  pcSlice->getPic()->sliceSubpicIdx.push_back(pps->getSubPicIdxFromSubPicId(pcSlice->getSliceSubPicId()));
  pcSlice->checkCRA(pcSlice->getRPL0(), pcSlice->getRPL1(), m_pocCRA[nalu.m_nuhLayerId], m_cListPic);
  pcSlice->constructRefPicList(m_cListPic);
  pcSlice->setPrevGDRSubpicPOC(m_prevGDRSubpicPOC[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->setPrevIRAPSubpicPOC(m_prevIRAPSubpicPOC[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->setPrevIRAPSubpicType(m_prevIRAPSubpicType[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->checkSubpicTypeConstraints(m_cListPic, pcSlice->getRPL0(), pcSlice->getRPL1(), m_prevIRAPSubpicDecOrderNo[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->checkRPL(pcSlice->getRPL0(), pcSlice->getRPL1(), m_associatedIRAPDecodingOrderNumber[nalu.m_nuhLayerId], m_cListPic);
  pcSlice->checkSTSA(m_cListPic);
  if (m_pcPic->cs->vps && !m_pcPic->cs->vps->getIndependentLayerFlag(m_pcPic->cs->vps->getGeneralLayerIdx(nalu.m_nuhLayerId)) && m_pcPic->cs->pps->getNumSubPics() > 1)
  {
    CU::checkConformanceILRP(pcSlice);
  }

  pcSlice->scaleRefPicList( scaledRefPic, m_pcPic->cs->picHeader, m_parameterSetManager.getAPSs(), m_picHeader.getLmcsAPS(), m_picHeader.getScalingListAPS(), true );


    if (!pcSlice->isIntra())
    {
      bool bLowDelay = true;
      int  iCurrPOC  = pcSlice->getPOC();
      int iRefIdx = 0;

      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      if (pcSlice->isInterB())
      {
        for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
        {
          if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
          {
            bLowDelay = false;
          }
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }

    if (pcSlice->getSPS()->getUseSMVD() && pcSlice->getCheckLDC() == false
      && pcSlice->getPicHeader()->getMvdL1ZeroFlag() == false
      )
    {
      int currPOC = pcSlice->getPOC();

      int forwardPOC = currPOC;
      int backwardPOC = currPOC;
      int ref = 0;
      int refIdx0 = -1;
      int refIdx1 = -1;

      // search nearest forward POC in List 0
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_0, ref)->longTerm;
        if ( poc < currPOC && (poc > forwardPOC || refIdx0 == -1) && !isRefLongTerm )
        {
          forwardPOC = poc;
          refIdx0 = ref;
        }
      }

      // search nearest backward POC in List 1
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_1, ref)->longTerm;
        if ( poc > currPOC && (poc < backwardPOC || refIdx1 == -1) && !isRefLongTerm )
        {
          backwardPOC = poc;
          refIdx1 = ref;
        }
      }

      if ( !(forwardPOC < currPOC && backwardPOC > currPOC) )
      {
        forwardPOC = currPOC;
        backwardPOC = currPOC;
        refIdx0 = -1;
        refIdx1 = -1;

        // search nearest backward POC in List 0
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_0, ref)->longTerm;
          if ( poc > currPOC && (poc < backwardPOC || refIdx0 == -1) && !isRefLongTerm )
          {
            backwardPOC = poc;
            refIdx0 = ref;
          }
        }

        // search nearest forward POC in List 1
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_1, ref)->longTerm;
          if ( poc < currPOC && (poc > forwardPOC || refIdx1 == -1) && !isRefLongTerm )
          {
            forwardPOC = poc;
            refIdx1 = ref;
          }
        }
      }

      if ( forwardPOC < currPOC && backwardPOC > currPOC )
      {
        pcSlice->setBiDirPred( true, refIdx0, refIdx1 );
      }
      else
      {
        pcSlice->setBiDirPred( false, -1, -1 );
      }
    }
    else
    {
      pcSlice->setBiDirPred( false, -1, -1 );
    }

    //---------------
    pcSlice->setRefPOCList();

    NalUnitInfo naluInfo;
    naluInfo.m_nalUnitType = nalu.m_nalUnitType;
    naluInfo.m_nuhLayerId = nalu.m_nuhLayerId;
    naluInfo.m_firstCTUinSlice = pcSlice->getFirstCtuRsAddrInSlice();
    naluInfo.m_POC = pcSlice->getPOC();
    xCheckMixedNalUnit(pcSlice, sps, nalu);
    m_nalUnitInfo[naluInfo.m_nuhLayerId].push_back(naluInfo);
    SEIMessages drapSEIs = getSeisByType(m_pcPic->SEIs, SEI::DEPENDENT_RAP_INDICATION );
    if (!drapSEIs.empty())
    {
      msg(NOTICE_, "Dependent RAP indication SEI decoded\n");
      pcSlice->setDRAP(true);
      pcSlice->setLatestDRAPPOC(pcSlice->getPOC());
    }
    pcSlice->checkConformanceForDRAP(nalu.m_temporalId);

  Quant *quant = m_cTrQuant.getQuant();

  if (pcSlice->getExplicitScalingListUsed())
  {
    APS* scalingListAPS = pcSlice->getPicHeader()->getScalingListAPS();
    if( pcSlice->getNalUnitLayerId() != scalingListAPS->getLayerId() )
    {
      CHECK_( scalingListAPS->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
      CHECK_( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
      for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
      {
        bool isCurrLayerInOls = false;
        bool isRefLayerInOls = false;
        for( int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
        {
          if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
          {
            isCurrLayerInOls = true;
          }
          if( pcSlice->getVPS()->getLayerIdInOls(i, j) == scalingListAPS->getLayerId() )
          {
            isRefLayerInOls = true;
          }
        }
        CHECK_( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
      }
    }
    ScalingList scalingList = scalingListAPS->getScalingList();
    quant->setScalingListDec(scalingList);
    quant->setUseScalingList(true);
  }
  else
  {
    quant->setUseScalingList( false );
  }

  if (pcSlice->getSPS()->getUseLmcs())
  {
    if (m_bFirstSliceInPicture)
      m_sliceLmcsApsId = -1;
    if (pcSlice->getLmcsEnabledFlag())
    {
      APS* lmcsAPS = pcSlice->getPicHeader()->getLmcsAPS();
      if (m_sliceLmcsApsId == -1)
      {
        m_sliceLmcsApsId = lmcsAPS->getAPSId();
      }
      else
      {
        CHECK_(lmcsAPS->getAPSId() != m_sliceLmcsApsId, "same APS ID shall be used for all slices in one picture");
      }
      if( pcSlice->getNalUnitLayerId() != lmcsAPS->getLayerId() )
      {
        CHECK_( lmcsAPS->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
        CHECK_( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
        for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls = false;
          for( int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
          {
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
            {
              isCurrLayerInOls = true;
            }
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == lmcsAPS->getLayerId() )
            {
              isRefLayerInOls = true;
            }
          }
          CHECK_( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
        }
      }
      SliceReshapeInfo& sInfo = lmcsAPS->getReshaperAPSInfo();
      SliceReshapeInfo& tInfo = m_cReshaper.getSliceReshaperInfo();
      tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
      tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
      memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int)*(PIC_CODE_CW_BINS));
      tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
      tInfo.chrResScalingOffset = sInfo.chrResScalingOffset;
      tInfo.setUseSliceReshaper(pcSlice->getLmcsEnabledFlag());
      tInfo.setSliceReshapeChromaAdj(pcSlice->getPicHeader()->getLmcsChromaResidualScaleFlag());
      tInfo.setSliceReshapeModelPresentFlag(true);
    }
    else
    {
      SliceReshapeInfo& tInfo = m_cReshaper.getSliceReshaperInfo();
      tInfo.setUseSliceReshaper(false);
      tInfo.setSliceReshapeChromaAdj(false);
      tInfo.setSliceReshapeModelPresentFlag(false);
    }
    if (pcSlice->getLmcsEnabledFlag())
    {
      m_cReshaper.constructReshaper();
    }
    else
    {
      m_cReshaper.setReshapeFlag(false);
    }
    if ((pcSlice->getSliceType() == I_SLICE) && m_cReshaper.getSliceReshaperInfo().getUseSliceReshaper())
    {
      m_cReshaper.setCTUFlag(false);
      m_cReshaper.setRecReshaped(true);
    }
    else
    {
      if (m_cReshaper.getSliceReshaperInfo().getUseSliceReshaper())
      {
        m_cReshaper.setCTUFlag(true);
        m_cReshaper.setRecReshaped(true);
      }
      else
      {
        m_cReshaper.setCTUFlag(false);
        m_cReshaper.setRecReshaped(false);
      }
    }
    m_cReshaper.setVPDULoc(-1, -1);
  }
  else
  {
    m_cReshaper.setCTUFlag(false);
    m_cReshaper.setRecReshaped(false);
  }

  //  Decode a picture
  m_cSliceDecoder.decompressSlice( pcSlice, &( nalu.getBitstream() ), ( m_pcPic->poc == getDebugPOC() ? getDebugCTU() : -1 ) );

  m_bFirstSliceInPicture = false;
  m_uiSliceSegmentIdx++;

  pcSlice->freeScaledRefPicList( scaledRefPic );

  return false;
}

void DecLib::updatePrevGDRInSameLayer()
{
  const NalUnitType pictureType = m_pcPic->getPictureType();

  if (pictureType == NAL_UNIT_CODED_SLICE_GDR && !m_pcPic->cs->pps->getMixedNaluTypesInPicFlag())
  {
    m_prevGDRInSameLayerPOC[m_pcPic->layerId] = m_pcPic->getPOC();
  }
}

void DecLib::updateAssociatedIRAP()
{
  const NalUnitType pictureType = m_pcPic->getPictureType();

  if ((pictureType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pictureType == NAL_UNIT_CODED_SLICE_IDR_N_LP || pictureType == NAL_UNIT_CODED_SLICE_CRA) && !m_pcPic->cs->pps->getMixedNaluTypesInPicFlag())
  {
    m_associatedIRAPDecodingOrderNumber[m_pcPic->layerId] = m_pcPic->getDecodingOrderNumber();
    m_pocCRA[m_pcPic->layerId] = m_pcPic->getPOC();
    m_associatedIRAPType[m_pcPic->layerId] = pictureType;
  }
}

void DecLib::updatePrevIRAPAndGDRSubpic()
{
  for (int j = 0; j < m_uiSliceSegmentIdx; j++)
  {
    Slice* pcSlice = m_pcPic->slices[j];
    const int subpicIdx = pcSlice->getPPS()->getSubPicIdxFromSubPicId(pcSlice->getSliceSubPicId());
    if (pcSlice->getCtuAddrInSlice(0) == m_pcPic->cs->pps->getSubPic(subpicIdx).getFirstCTUInSubPic())
    {
      const NalUnitType subpicType = pcSlice->getNalUnitType();
      if (subpicType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || subpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP || subpicType == NAL_UNIT_CODED_SLICE_CRA)
      {
        m_prevIRAPSubpicPOC[m_pcPic->layerId][subpicIdx] = m_pcPic->getPOC();
        m_prevIRAPSubpicType[m_pcPic->layerId][subpicIdx] = subpicType;
        m_prevIRAPSubpicDecOrderNo[m_pcPic->layerId][subpicIdx] = m_pcPic->getDecodingOrderNumber();
      }
      else if (subpicType == NAL_UNIT_CODED_SLICE_GDR)
      {
        m_prevGDRSubpicPOC[m_pcPic->layerId][subpicIdx] = m_pcPic->getPOC();
      }
    }
  }
}

#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
void DecLib::xDecodeOPI( InputNALUnit& nalu )
{
  m_opi = new OPI();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK_( nalu.m_temporalId, "The value of TemporalId of OPI NAL units shall be equal to 0" );

  m_HLSReader.parseOPI( m_opi );
}
#endif

void DecLib::xDecodeVPS( InputNALUnit& nalu )
{
  VPS* vps = new VPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK_( nalu.m_temporalId, "The value of TemporalId of VPS NAL units shall be equal to 0" );

  m_HLSReader.parseVPS( vps );

  // storeVPS may directly delete the new VPS in case it is a repetition. Need to retrieve proper initialized memory back
  int vpsID = vps->getVPSId();
  m_parameterSetManager.storeVPS( vps, nalu.getBitstream().getFifo());

  if (m_vps==nullptr)
  {
    // m_vps is used for conformance checks. Unless a VPS is referred to, just set the first one we received
    // repeated parameter sets may be deleted, set a valid VPS pointer back from parameter set manager
    m_vps = m_parameterSetManager.getVPS(vpsID);
  }
}

void DecLib::xDecodeDCI(InputNALUnit& nalu)
{
  m_HLSReader.setBitstream(&nalu.getBitstream());

  CHECK_(nalu.m_temporalId, "The value of TemporalId of DCI NAL units shall be equal to 0");
  if (!m_dci)
  {
    m_dci = new DCI;
    m_HLSReader.parseDCI(m_dci);
  }
  else
  {
    DCI dupDCI;
    m_HLSReader.parseDCI(&dupDCI);
    CHECK_( !m_dci->IsIndenticalDCI(dupDCI), "Two signaled DCIs are different");
  }
}

void DecLib::xDecodeSPS( InputNALUnit& nalu )
{
  SPS* sps = new SPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK_( nalu.m_temporalId, "The value of TemporalId of SPS NAL units shall be equal to 0" );

  m_HLSReader.parseSPS( sps );
  sps->setLayerId( nalu.m_nuhLayerId );
  DTRACE( g_trace_ctx, D_QP_PER_CTU, "CTU Size: %dx%d", sps->getMaxCUWidth(), sps->getMaxCUHeight() );
  m_parameterSetManager.storeSPS( sps, nalu.getBitstream().getFifo() );
}

void DecLib::xDecodePPS( InputNALUnit& nalu )
{
  PPS* pps = new PPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePPS( pps );
  pps->setLayerId( nalu.m_nuhLayerId );
  pps->setTemporalId( nalu.m_temporalId );
  pps->setPuCounter( m_puCounter );
  m_parameterSetManager.storePPS( pps, nalu.getBitstream().getFifo() );
}

void DecLib::xDecodeAPS(InputNALUnit& nalu)
{
  APS* aps = new APS();
  m_HLSReader.setBitstream(&nalu.getBitstream());
  m_HLSReader.parseAPS(aps);
  aps->setTemporalId(nalu.m_temporalId);
  aps->setLayerId( nalu.m_nuhLayerId );
  aps->setHasPrefixNalUnitType( nalu.m_nalUnitType == NAL_UNIT_PREFIX_APS );
  aps->setPuCounter( m_puCounter );
  m_parameterSetManager.checkAuApsContent( aps, m_accessUnitApsNals );
  if( m_apsMapEnc )
  {
    APS* apsEnc = new APS();
    *apsEnc = *aps;
    m_apsMapEnc->storePS( ( apsEnc->getAPSId() << NUM_APS_TYPE_LEN ) + apsEnc->getAPSType(), apsEnc ); 
  }

  // aps will be deleted if it was already stored (and did not changed),
  // thus, storing it must be last action.
  m_parameterSetManager.storeAPS(aps, nalu.getBitstream().getFifo());
}
bool DecLib::decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay, int iTargetOlsIdx)
{
  bool ret;
  // ignore all NAL units of layers > 0

  AccessUnitInfo auInfo;
  auInfo.m_nalUnitType = nalu.m_nalUnitType;
  auInfo.m_nuhLayerId = nalu.m_nuhLayerId;
  auInfo.m_temporalId = nalu.m_temporalId;
  m_accessUnitNals.push_back(auInfo);
  m_pictureUnitNals.push_back( nalu.m_nalUnitType );
  switch (nalu.m_nalUnitType)
  {
    case NAL_UNIT_VPS:
      xDecodeVPS( nalu );
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
      if (getTOlsIdxExternalFlag())
      {
        m_vps->m_targetOlsIdx = iTargetOlsIdx;
      }
      else if (getTOlsIdxOpiFlag())
      {
        m_vps->m_targetOlsIdx = m_opi->getOpiOlsIdx();
      }
      else
      {
        m_vps->m_targetOlsIdx = m_vps->deriveTargetOLSIdx();
      }
#else
      m_vps->m_targetOlsIdx = iTargetOlsIdx;
#endif
      return false;
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
    case NAL_UNIT_OPI:
      xDecodeOPI( nalu );
      return false;
#endif
    case NAL_UNIT_DCI:
      xDecodeDCI( nalu );
      return false;
    case NAL_UNIT_SPS:
      xDecodeSPS( nalu );
      return false;

    case NAL_UNIT_PPS:
      xDecodePPS( nalu );
      return false;

    case NAL_UNIT_PH:
      xDecodePicHeader(nalu);
      return !m_bFirstSliceInPicture;

    case NAL_UNIT_PREFIX_APS:
    case NAL_UNIT_SUFFIX_APS:
      xDecodeAPS(nalu);
      return false;

    case NAL_UNIT_PREFIX_SEI:
      // Buffer up prefix SEI messages until SPS of associated VCL is known.
      m_prefixSEINALUs.push_back(new InputNALUnit(nalu));
      m_pictureSeiNalus.push_back(new InputNALUnit(nalu));
      return false;

    case NAL_UNIT_SUFFIX_SEI:
      if (m_pcPic)
      {
        m_pictureSeiNalus.push_back(new InputNALUnit(nalu));
        m_accessUnitSeiTids.push_back(nalu.m_temporalId);
        const SPS *sps = m_parameterSetManager.getActiveSPS();
        const VPS *vps = m_parameterSetManager.getVPS(sps->getVPSId());
        m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_pcPic->SEIs, nalu.m_nalUnitType, nalu.m_nuhLayerId, nalu.m_temporalId, vps, sps, m_HRD, m_pDecodedSEIOutputStream );
#if JVET_S0257_DUMP_360SEI_MESSAGE
        m_seiCfgDump.write360SeiDump(m_decoded360SeiDumpFileName, m_pcPic->SEIs, sps);
#endif
        m_accessUnitSeiPayLoadTypes.push_back(std::tuple<NalUnitType, int, SEI::PayloadType>(nalu.m_nalUnitType, nalu.m_nuhLayerId, m_pcPic->SEIs.back()->payloadType()));
      }
      else
      {
        msg(NOTICE_, "Note: received suffix SEI but no picture currently active.\n");
      }
      return false;

    case NAL_UNIT_CODED_SLICE_TRAIL:
    case NAL_UNIT_CODED_SLICE_STSA:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_GDR:
    case NAL_UNIT_CODED_SLICE_RADL:
    case NAL_UNIT_CODED_SLICE_RASL:
      ret = xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay);
      return ret;

    case NAL_UNIT_EOS:
      m_associatedIRAPType[nalu.m_nuhLayerId] = NAL_UNIT_INVALID;
      m_pocCRA[nalu.m_nuhLayerId] = -MAX_INT;
      m_prevGDRInSameLayerPOC[nalu.m_nuhLayerId] = -MAX_INT;
      std::fill_n(m_prevGDRSubpicPOC[nalu.m_nuhLayerId], MAX_NUM_SUB_PICS, -MAX_INT);
      std::fill_n(m_prevIRAPSubpicPOC[nalu.m_nuhLayerId], MAX_NUM_SUB_PICS, -MAX_INT);
      memset(m_prevIRAPSubpicDecOrderNo[nalu.m_nuhLayerId], 0, sizeof(int)*MAX_NUM_SUB_PICS);
      std::fill_n(m_prevIRAPSubpicType[nalu.m_nuhLayerId], MAX_NUM_SUB_PICS, NAL_UNIT_INVALID);
      m_pocRandomAccess = MAX_INT;
      m_prevLayerID = MAX_INT;
      m_prevPOC = -MAX_INT;
      m_prevSliceSkipped = false;
      m_skippedPOC = 0;
      m_accessUnitEos[nalu.m_nuhLayerId] = true;
      m_prevEOS[nalu.m_nuhLayerId] = true;
      return false;

    case NAL_UNIT_ACCESS_UNIT_DELIMITER:
      {
        AUDReader audReader;
        uint32_t picType;
        audReader.parseAccessUnitDelimiter(&(nalu.getBitstream()), m_audIrapOrGdrAuFlag, picType);
        return !m_bFirstSliceInPicture;
      }

    case NAL_UNIT_EOB:
      return false;

    case NAL_UNIT_FD:
    {
      FDReader fdReader;
      uint32_t fdSize;
      fdReader.parseFillerData(&(nalu.getBitstream()), fdSize);
      msg(NOTICE_, "Note: found NAL_UNIT_FD with %u bytes payload.\n", fdSize);
      return false;
    }

    case NAL_UNIT_RESERVED_IRAP_VCL_11:
#if !JVET_S0163_ON_TARGETOLS_SUBLAYERS
    case NAL_UNIT_RESERVED_IRAP_VCL_12:
#endif
      msg(NOTICE_, "Note: found reserved VCL NAL unit.\n");
      xParsePrefixSEIsForUnknownVCLNal();
      return false;
    case NAL_UNIT_RESERVED_VCL_4:
    case NAL_UNIT_RESERVED_VCL_5:
    case NAL_UNIT_RESERVED_VCL_6:
    case NAL_UNIT_RESERVED_NVCL_26:
    case NAL_UNIT_RESERVED_NVCL_27:
      msg(NOTICE_, "Note: found reserved NAL unit.\n");
      return false;
    case NAL_UNIT_UNSPECIFIED_28:
    case NAL_UNIT_UNSPECIFIED_29:
    case NAL_UNIT_UNSPECIFIED_30:
    case NAL_UNIT_UNSPECIFIED_31:
      msg(NOTICE_, "Note: found unspecified NAL unit.\n");
      return false;
    default:
      THROW( "Invalid NAL unit type" );
      break;
  }

  return false;
}


/** Function for checking if picture should be skipped because of random access. This function checks the skipping of pictures in the case of -s option random access.
 *  All pictures prior to the random access point indicated by the counter iSkipFrame are skipped.
 *  It also checks the type of Nal unit type at the random access point.
 *  If the random access point is CRA/CRANT/BLA/BLANT, TFD pictures with POC less than the POC of the random access point are skipped.
 *  If the random access point is IDR all pictures after the random access point are decoded.
 *  If the random access point is none of the above, a warning is issues, and decoding of pictures with POC
 *  equal to or greater than the random access point POC is attempted. For non IDR/CRA/BLA random
 *  access point there is no guarantee that the decoder will not crash.
 */
bool DecLib::isRandomAccessSkipPicture( int& iSkipFrame, int& iPOCLastDisplay )
{
  if (iSkipFrame)
  {
    iSkipFrame--;   // decrement the counter
    return true;
  }
  else if ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )
  {
    m_pocRandomAccess = -MAX_INT; // no need to skip the reordered pictures in IDR, they are decodable.
  }
  else if (m_pocRandomAccess == MAX_INT) // start of random access point, m_pocRandomAccess has not been set yet.
  {
    if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR && m_apcSlicePilot->getPicHeader()->getRecoveryPocCnt() == 0 ) )
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->getPOC();
    }
    else
    {
      if(!m_warningMessageSkipPicture)
      {
        msg(WARNING_, "\nWarning: this is not a valid random access point and the data is discarded until the first CRA picture");
        m_warningMessageSkipPicture = true;
      }
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->getPOC() < m_pocRandomAccess && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL))
  {
    iPOCLastDisplay++;
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false;
}

void DecLib::checkNalUnitConstraints( uint32_t naluType )
{
  if (m_parameterSetManager.getActiveSPS() != NULL && m_parameterSetManager.getActiveSPS()->getProfileTierLevel() != NULL)
  {
    const ConstraintInfo *cInfo = m_parameterSetManager.getActiveSPS()->getProfileTierLevel()->getConstraintInfo();
    xCheckNalUnitConstraintFlags( cInfo, naluType );
  }

}
void DecLib::xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType )
{
  if (cInfo != NULL)
  {
    CHECK_(cInfo->getNoTrailConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_TRAIL,
      "Non-conforming bitstream. no_trail_constraint_flag is equal to 1 but bitstream contains NAL unit of type TRAIL_NUT.");
    CHECK_(cInfo->getNoStsaConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_STSA,
      "Non-conforming bitstream. no_stsa_constraint_flag is equal to 1 but bitstream contains NAL unit of type STSA_NUT.");
    CHECK_(cInfo->getNoRaslConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_RASL,
      "Non-conforming bitstream. no_rasl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RASL_NUT.");
    CHECK_(cInfo->getNoRadlConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_RADL,
      "Non-conforming bitstream. no_radl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RADL_NUT.");
    CHECK_(cInfo->getNoIdrConstraintFlag() && (naluType == NAL_UNIT_CODED_SLICE_IDR_W_RADL),
      "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_W_RADL.");
    CHECK_(cInfo->getNoIdrConstraintFlag() && (naluType == NAL_UNIT_CODED_SLICE_IDR_N_LP),
      "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_N_LP.");
    CHECK_(cInfo->getNoCraConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_CRA,
      "Non-conforming bitstream. no_cra_constraint_flag is equal to 1 but bitstream contains NAL unit of type CRA_NUT.");
    CHECK_(cInfo->getNoGdrConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_GDR,
      "Non-conforming bitstream. no_gdr_constraint_flag is equal to 1 but bitstream contains NAL unit of type GDR_NUT.");
    CHECK_(cInfo->getNoApsConstraintFlag() && naluType == NAL_UNIT_PREFIX_APS,
      "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_PREFIX_NUT.");
    CHECK_(cInfo->getNoApsConstraintFlag() && naluType == NAL_UNIT_SUFFIX_APS,
      "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_SUFFIX_NUT.");
  }
}
void DecLib::xCheckMixedNalUnit(Slice* pcSlice, SPS *sps, InputNALUnit &nalu)
{
  if (pcSlice->getPPS()->getMixedNaluTypesInPicFlag())
  {
    CHECK_(pcSlice->getPPS()->getNumSlicesInPic() < 2, "mixed nal unit type picture, but with less than 2 slices");

    CHECK_( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR, "picture with mixed NAL unit type cannot have GDR slice");

    //Check that if current slice is IRAP type, the other type of NAL can only be TRAIL_NUT
    if( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )
    {
      for( int i = 0; i < m_uiSliceSegmentIdx; i++ )
      {
        Slice* PreSlice = m_pcPic->slices[i];
        CHECK_( (pcSlice->getNalUnitType() != PreSlice->getNalUnitType()) && (PreSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_TRAIL), "In a mixed NAL unt type picture, an IRAP slice can be mixed with Trail slice(s) only");
      }
    }

    // if this is the last slice of the picture, check whether that there are at least two different NAL unit types in the picture
    if (pcSlice->getPPS()->getNumSlicesInPic() == (m_uiSliceSegmentIdx + 1))
    {
      bool hasDiffTypes = false;
      for( int i = 1; !hasDiffTypes && i <= m_uiSliceSegmentIdx; i++ )
      {
        Slice* slice1 = m_pcPic->slices[i-1];
        Slice* slice2 = m_pcPic->slices[i];
        if( slice1->getNalUnitType() != slice2->getNalUnitType())
        {
          hasDiffTypes = true;
        }
      }
      CHECK_( !hasDiffTypes, "VCL NAL units of the picture shall have two or more different nal_unit_type values");
    }

  }
  else // all slices shall have the same nal unit type
  {
    bool sameNalUnitType = true;
    for (int i = 0; i < m_uiSliceSegmentIdx; i++)
    {
      Slice *PreSlice = m_pcPic->slices[i];
      if (PreSlice->getNalUnitType() != pcSlice->getNalUnitType())
        sameNalUnitType = false;
    }
    CHECK_(!sameNalUnitType, "pps_mixed_nalu_types_in_pic_flag is zero, but have different nal unit types");
  }
}
/**
- lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new picture
*/
bool DecLib::isNewPicture(std::ifstream *bitstreamFile, class InputByteStream *bytestream)
{
  bool ret = false;
  bool finished = false;

  // cannot be a new picture if there haven't been any slices yet
  if(getFirstSliceInPicture())
  {
    return false;
  }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData* backupStats = new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
  std::streampos location = bitstreamFile->tellg() - std::streampos(bytestream->GetNumBufferedBytes());
#else
  std::streampos location = bitstreamFile->tellg();
#endif

  // look ahead until picture start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg(ERROR_, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu);
      switch( nalu.m_nalUnitType ) {

      // NUT that indicate the start of a new picture
      case NAL_UNIT_ACCESS_UNIT_DELIMITER:
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
      case NAL_UNIT_OPI:
#endif
      case NAL_UNIT_DCI:
      case NAL_UNIT_VPS:
      case NAL_UNIT_SPS:
      case NAL_UNIT_PPS:
      case NAL_UNIT_PH:
        ret = true;
        finished = true;
        break;

      // NUT that may be the start of a new picture - check first bit in slice header
      case NAL_UNIT_CODED_SLICE_TRAIL:
      case NAL_UNIT_CODED_SLICE_STSA:
      case NAL_UNIT_CODED_SLICE_RASL:
      case NAL_UNIT_CODED_SLICE_RADL:
      case NAL_UNIT_RESERVED_VCL_4:
      case NAL_UNIT_RESERVED_VCL_5:
      case NAL_UNIT_RESERVED_VCL_6:
      case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case NAL_UNIT_CODED_SLICE_CRA:
      case NAL_UNIT_CODED_SLICE_GDR:
      case NAL_UNIT_RESERVED_IRAP_VCL_11:
#if !JVET_S0163_ON_TARGETOLS_SUBLAYERS
      case NAL_UNIT_RESERVED_IRAP_VCL_12:
#endif
        ret = checkPictureHeaderInSliceHeaderFlag(nalu);
        finished = true;
        break;

      // NUT that are not the start of a new picture
      case NAL_UNIT_EOS:
      case NAL_UNIT_EOB:
      case NAL_UNIT_SUFFIX_APS:
      case NAL_UNIT_SUFFIX_SEI:
      case NAL_UNIT_FD:
        ret = false;
        finished = true;
        break;

      // NUT that might indicate the start of a new picture - keep looking
      case NAL_UNIT_PREFIX_APS:
      case NAL_UNIT_PREFIX_SEI:
      case NAL_UNIT_RESERVED_NVCL_26:
      case NAL_UNIT_RESERVED_NVCL_27:
      case NAL_UNIT_UNSPECIFIED_28:
      case NAL_UNIT_UNSPECIFIED_29:
      case NAL_UNIT_UNSPECIFIED_30:
      case NAL_UNIT_UNSPECIFIED_31:
      default:
        break;
      }
    }
  }

  // restore previous stream location - minus 3 due to the need for the annexB parser to read three extra bytes
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
  CodingStatistics::SetStatistics(*backupStats);
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg(location-std::streamoff(3));
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

/**
- lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new access unit
*/
bool DecLib::isNewAccessUnit( bool newPicture, std::ifstream *bitstreamFile, class InputByteStream *bytestream )
{
  bool ret = false;
  bool finished = false;

  // can only be the start of an AU if this is the start of a new picture
  if( newPicture == false )
  {
    return false;
  }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData* backupStats = new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
  std::streampos location = bitstreamFile->tellg() - std::streampos(bytestream->GetNumBufferedBytes());
#else
  std::streampos location = bitstreamFile->tellg();
#endif

  // look ahead until access unit start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg(ERROR_, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu);
      switch( nalu.m_nalUnitType ) {

      // AUD always indicates the start of a new access unit
      case NAL_UNIT_ACCESS_UNIT_DELIMITER:
        ret = true;
        finished = true;
        break;

      // slice types - check layer ID and POC
      case NAL_UNIT_CODED_SLICE_TRAIL:
      case NAL_UNIT_CODED_SLICE_STSA:
      case NAL_UNIT_CODED_SLICE_RASL:
      case NAL_UNIT_CODED_SLICE_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case NAL_UNIT_CODED_SLICE_CRA:
      case NAL_UNIT_CODED_SLICE_GDR:
        ret = isSliceNaluFirstInAU( newPicture, nalu );
        finished = true;
        break;

      // NUT that are not the start of a new access unit
      case NAL_UNIT_EOS:
      case NAL_UNIT_EOB:
      case NAL_UNIT_SUFFIX_APS:
      case NAL_UNIT_SUFFIX_SEI:
      case NAL_UNIT_FD:
        ret = false;
        finished = true;
        break;

      // all other NUT - keep looking to find first VCL
      default:
        break;
      }
    }
  }

  // restore previous stream location
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
  CodingStatistics::SetStatistics(*backupStats);
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}
//! \}

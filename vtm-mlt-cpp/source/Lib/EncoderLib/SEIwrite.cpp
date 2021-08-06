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

#include "CommonLib/BitStream.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Slice.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"
#include "SEIwrite.h"

//! \ingroup EncoderLib
//! \{

void SEIWriter::xWriteSEIpayloadData(OutputBitstream &bs, const SEI& sei, HRD &hrd, const uint32_t temporalId)
{
  const SEIBufferingPeriod *bp = NULL;
  switch (sei.payloadType())
  {
  case SEI::USER_DATA_UNREGISTERED:
    xWriteSEIuserDataUnregistered(*static_cast<const SEIuserDataUnregistered*>(&sei));
    break;
  case SEI::DECODING_UNIT_INFO:
    bp = hrd.getBufferingPeriodSEI();
    CHECK_(bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Decoding Unit Information SEI");
    xWriteSEIDecodingUnitInfo(*static_cast<const SEIDecodingUnitInfo*>(& sei), *bp, temporalId);
    break;
  case SEI::SCALABLE_NESTING:
    xWriteSEIScalableNesting(bs, *static_cast<const SEIScalableNesting*>(&sei));
    break;
  case SEI::DECODED_PICTURE_HASH:
    xWriteSEIDecodedPictureHash(*static_cast<const SEIDecodedPictureHash*>(&sei));
    break;
  case SEI::BUFFERING_PERIOD:
    xWriteSEIBufferingPeriod(*static_cast<const SEIBufferingPeriod*>(&sei));
    hrd.setBufferingPeriodSEI(static_cast<const SEIBufferingPeriod*>(&sei));
    break;
  case SEI::PICTURE_TIMING:
    {
      bp = hrd.getBufferingPeriodSEI();
      CHECK_(bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Picture Timing SEI");
      xWriteSEIPictureTiming(*static_cast<const SEIPictureTiming*>(&sei), *bp, temporalId);
    }
    break;
  case SEI::FRAME_FIELD_INFO:
    xWriteSEIFrameFieldInfo(*static_cast<const SEIFrameFieldInfo*>(&sei));
    break;
  case SEI::DEPENDENT_RAP_INDICATION:
    xWriteSEIDependentRAPIndication(*static_cast<const SEIDependentRAPIndication*>(&sei));
    break;
  case SEI::FRAME_PACKING:
    xWriteSEIFramePacking(*static_cast<const SEIFramePacking*>(&sei));
    break;
  case SEI::PARAMETER_SETS_INCLUSION_INDICATION:
    xWriteSEIParameterSetsInclusionIndication(*static_cast<const SEIParameterSetsInclusionIndication*>(&sei));
    break;
  case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
    xWriteSEIMasteringDisplayColourVolume(*static_cast<const SEIMasteringDisplayColourVolume*>(&sei));
    break;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
    xWriteSEIAlternativeTransferCharacteristics(*static_cast<const SEIAlternativeTransferCharacteristics*>(&sei));
    break;
#endif
  case SEI::EQUIRECTANGULAR_PROJECTION:
    xWriteSEIEquirectangularProjection(*static_cast<const SEIEquirectangularProjection*>(&sei));
    break;
  case SEI::SPHERE_ROTATION:
    xWriteSEISphereRotation(*static_cast<const SEISphereRotation*>(&sei));
    break;
  case SEI::OMNI_VIEWPORT:
    xWriteSEIOmniViewport(*static_cast<const SEIOmniViewport*>(&sei));
    break;
  case SEI::REGION_WISE_PACKING:
    xWriteSEIRegionWisePacking(*static_cast<const SEIRegionWisePacking*>(&sei));
    break;
  case SEI::GENERALIZED_CUBEMAP_PROJECTION:
    xWriteSEIGeneralizedCubemapProjection(*static_cast<const SEIGeneralizedCubemapProjection*>(&sei));
    break;
  case SEI::USER_DATA_REGISTERED_ITU_T_T35:
    xWriteSEIUserDataRegistered(*static_cast<const SEIUserDataRegistered*>(&sei));
    break;
  case SEI::FILM_GRAIN_CHARACTERISTICS:
    xWriteSEIFilmGrainCharacteristics(*static_cast<const SEIFilmGrainCharacteristics*>(&sei));
    break;
  case SEI::CONTENT_LIGHT_LEVEL_INFO:
    xWriteSEIContentLightLevelInfo(*static_cast<const SEIContentLightLevelInfo*>(&sei));
    break;
  case SEI::AMBIENT_VIEWING_ENVIRONMENT:
    xWriteSEIAmbientViewingEnvironment(*static_cast<const SEIAmbientViewingEnvironment*>(&sei));
    break;
  case SEI::CONTENT_COLOUR_VOLUME:
    xWriteSEIContentColourVolume(*static_cast<const SEIContentColourVolume*>(&sei));
    break;
  case SEI::SUBPICTURE_LEVEL_INFO:
    xWriteSEISubpictureLevelInfo(*static_cast<const SEISubpicureLevelInfo*>(&sei));
    break;
  case SEI::SAMPLE_ASPECT_RATIO_INFO:
    xWriteSEISampleAspectRatioInfo(*static_cast<const SEISampleAspectRatioInfo*>(&sei));
    break;
  default:
    THROW("Trying to write unhandled SEI message");
    break;
  }
  xWriteByteAlign();
}

/**
 * marshal all SEI messages in provided list into one bitstream bs
 */
void SEIWriter::writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, HRD &hrd, bool isNested, const uint32_t temporalId)
{
#if ENABLE_TRACING
  if (g_HLSTraceEnable)
    xTraceSEIHeader();
#endif

  OutputBitstream bs_count;

  for (SEIMessages::const_iterator sei=seiList.begin(); sei!=seiList.end(); sei++)
  {
    // calculate how large the payload data is
    // TODO: this would be far nicer if it used vectored buffers
    bs_count.clear();
    setBitstream(&bs_count);

#if ENABLE_TRACING
    bool traceEnable = g_HLSTraceEnable;
    g_HLSTraceEnable = false;
#endif
    xWriteSEIpayloadData(bs_count, **sei, hrd, temporalId);
#if ENABLE_TRACING
    g_HLSTraceEnable = traceEnable;
#endif
    uint32_t payload_data_num_bits = bs_count.getNumberOfWrittenBits();
    CHECK_(0 != payload_data_num_bits % 8, "Invalid number of payload data bits");

    setBitstream(&bs);
    uint32_t payloadType = (*sei)->payloadType();
    for (; payloadType >= 0xff; payloadType -= 0xff)
    {
      WRITE_CODE(0xff, 8, "payload_type");
    }
    WRITE_CODE(payloadType, 8, "payload_type");

    uint32_t payloadSize = payload_data_num_bits/8;
    for (; payloadSize >= 0xff; payloadSize -= 0xff)
    {
      WRITE_CODE(0xff, 8, "payload_size");
    }
    WRITE_CODE(payloadSize, 8, "payload_size");

    /* payloadData */
#if ENABLE_TRACING
    if (g_HLSTraceEnable)
      xTraceSEIMessageType((*sei)->payloadType());
#endif

    xWriteSEIpayloadData(bs, **sei, hrd, temporalId);
  }
  if (!isNested)
  {
    xWriteRbspTrailingBits();
  }
}

/**
 * marshal a user_data_unregistered SEI message sei, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIuserDataUnregistered(const SEIuserDataUnregistered &sei)
{
  for (uint32_t i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    WRITE_CODE(sei.uuid_iso_iec_11578[i], 8 , "uuid_iso_iec_11578[i]");
  }

  for (uint32_t i = 0; i < sei.userDataLength; i++)
  {
    WRITE_CODE(sei.userData[i], 8 , "user_data_payload_byte");
  }
}

/**
 * marshal a decoded picture hash SEI message, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei)
{
  const char *traceString="\0";
  switch (sei.method)
  {
    case HASHTYPE_MD5: traceString="picture_md5"; break;
    case HASHTYPE_CRC: traceString="picture_crc"; break;
    case HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: THROW("Unknown hash type"); break;
  }

  if (traceString != 0) //use of this variable is needed to avoid a compiler error with G++ 4.6.1
  {
    WRITE_CODE(sei.method, 8, "dph_sei_hash_type");
#if FIX_TICKET_1405
    WRITE_CODE(sei.singleCompFlag, 1, "dph_sei_single_component_flag");
    WRITE_CODE(0, 7, "dph_sei_reserved_zero_7bits");
#endif
    for(uint32_t i=0; i<uint32_t(sei.m_pictureHash.hash.size()); i++)
    {
      WRITE_CODE(sei.m_pictureHash.hash[i], 8, traceString);
    }
  }
}


void SEIWriter::xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SEIBufferingPeriod& bp, const uint32_t temporalId)
{
  WRITE_UVLC(sei.m_decodingUnitIdx, "decoding_unit_idx");
  if( !bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    for (int i = temporalId; i <= bp.m_bpMaxSubLayers - 1; i++)
    {
      if (i < bp.m_bpMaxSubLayers - 1)
      {
      WRITE_FLAG(sei.m_duiSubLayerDelaysPresentFlag[i], "dui_sub_layer_delays_present_flag[i]");
      }
      if( sei.m_duiSubLayerDelaysPresentFlag[i] )
        WRITE_CODE( sei.m_duSptCpbRemovalDelayIncrement[i], bp.getDuCpbRemovalDelayIncrementLength(), "du_spt_cpb_removal_delay_increment[i]");
    }
  }
  if (bp.m_decodingUnitDpbDuParamsInPicTimingSeiFlag)
  {
    WRITE_FLAG(sei.m_dpbOutputDuDelayPresentFlag, "dpb_output_du_delay_present_flag");
  }
  if(sei.m_dpbOutputDuDelayPresentFlag)
  {
    WRITE_CODE(sei.m_picSptDpbOutputDuDelay, bp.getDpbOutputDelayDuLength(), "pic_spt_dpb_output_du_delay");
  }
}

void SEIWriter::xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei)
{
  WRITE_FLAG( sei.m_bpNalCpbParamsPresentFlag, "bp_nal_hrd_parameters_present_flag");
  WRITE_FLAG( sei.m_bpVclCpbParamsPresentFlag, "bp_vcl_hrd_parameters_present_flag");
  CHECK_(!sei.m_bpNalCpbParamsPresentFlag && !sei.m_bpVclCpbParamsPresentFlag, "bp_nal_hrd_parameters_present_flag and/or bp_vcl_hrd_parameters_present_flag must be true");
  CHECK_(sei.m_initialCpbRemovalDelayLength < 1, "sei.m_initialCpbRemovalDelayLength must be > 0");
  WRITE_CODE( sei.m_initialCpbRemovalDelayLength - 1, 5, "initial_cpb_removal_delay_length_minus1" );
  CHECK_(sei.m_cpbRemovalDelayLength < 1, "sei.m_cpbRemovalDelayLength must be > 0");
  WRITE_CODE( sei.m_cpbRemovalDelayLength - 1,        5, "cpb_removal_delay_length_minus1" );
  CHECK_(sei.m_dpbOutputDelayLength < 1, "sei.m_dpbOutputDelayLength must be > 0");
  WRITE_CODE( sei.m_dpbOutputDelayLength - 1,         5, "dpb_output_delay_length_minus1" );
  WRITE_FLAG( sei.m_bpDecodingUnitHrdParamsPresentFlag, "bp_decoding_unit_hrd_params_present_flag"  );
  if( sei.m_bpDecodingUnitHrdParamsPresentFlag )
  {
    CHECK_(sei.m_duCpbRemovalDelayIncrementLength < 1, "sei.m_duCpbRemovalDelayIncrementLength must be > 0");
    WRITE_CODE( sei.m_duCpbRemovalDelayIncrementLength - 1, 5, "du_cpb_removal_delay_increment_length_minus1" );
    CHECK_(sei.m_dpbOutputDelayDuLength < 1, "sei.m_dpbOutputDelayDuLength must be > 0");
    WRITE_CODE( sei.m_dpbOutputDelayDuLength - 1, 5, "dpb_output_delay_du_length_minus1" );
    WRITE_FLAG( sei.m_decodingUnitCpbParamsInPicTimingSeiFlag, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );
    WRITE_FLAG(sei.m_decodingUnitDpbDuParamsInPicTimingSeiFlag, "decoding_unit_dpb_du_params_in_pic_timing_sei_flag");
  }

  WRITE_FLAG( sei.m_concatenationFlag, "concatenation_flag");
  WRITE_FLAG( sei.m_additionalConcatenationInfoPresentFlag, "additional_concatenation_info_present_flag");
  if (sei.m_additionalConcatenationInfoPresentFlag)
  {
    WRITE_CODE( sei.m_maxInitialRemovalDelayForConcatenation, sei.m_initialCpbRemovalDelayLength, "max_initial_removal_delay_for_concatenation" );
  }

  CHECK_(sei.m_auCpbRemovalDelayDelta < 1, "sei.m_auCpbRemovalDelayDelta must be > 0");
  WRITE_CODE( sei.m_auCpbRemovalDelayDelta - 1, sei.m_cpbRemovalDelayLength, "au_cpb_removal_delay_delta_minus1" );

  CHECK_(sei.m_bpMaxSubLayers < 1, "bp_max_sub_layers_minus1 must be > 0");
  WRITE_CODE(sei.m_bpMaxSubLayers - 1, 3, "bp_max_sub_layers_minus1");
  if (sei.m_bpMaxSubLayers - 1 > 0)
  {
    WRITE_FLAG(sei.m_cpbRemovalDelayDeltasPresentFlag, "cpb_removal_delay_deltas_present_flag");
  }

  if (sei.m_cpbRemovalDelayDeltasPresentFlag)
  {
    CHECK_(sei.m_numCpbRemovalDelayDeltas < 1, "m_numCpbRemovalDelayDeltas must be > 0");
    WRITE_UVLC( sei.m_numCpbRemovalDelayDeltas - 1, "num_cpb_removal_delay_deltas_minus1" );
    for( int i = 0; i < sei.m_numCpbRemovalDelayDeltas; i ++ )
    {
      WRITE_CODE( sei.m_cpbRemovalDelayDelta[i],        sei.m_cpbRemovalDelayLength, "cpb_removal_delay_delta[i]" );
    }
  }
  CHECK_(sei.m_bpCpbCnt < 1, "sei.m_bpCpbCnt must be > 0");
  WRITE_UVLC( sei.m_bpCpbCnt - 1, "bp_cpb_cnt_minus1");
  if (sei.m_bpMaxSubLayers - 1 > 0)
  {
    WRITE_FLAG(sei.m_sublayerInitialCpbRemovalDelayPresentFlag, "bp_sublayer_initial_cpb_removal_delay_present_flag");
  }
  for (int i = (sei.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : sei.m_bpMaxSubLayers - 1); i < sei.m_bpMaxSubLayers; i++)
  {
    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei.m_bpNalCpbParamsPresentFlag ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei.m_bpVclCpbParamsPresentFlag ) ) )
      {
        for( int j = 0; j < sei.m_bpCpbCnt; j ++ )
        {
          WRITE_CODE( sei.m_initialCpbRemovalDelay[j][i][nalOrVcl],  sei.m_initialCpbRemovalDelayLength,           "initial_cpb_removal_delay[j][i][nalOrVcl]" );
          WRITE_CODE( sei.m_initialCpbRemovalOffset[j][i][nalOrVcl], sei.m_initialCpbRemovalDelayLength,           "initial_cpb_removal_delay_offset[j][i][nalOrVcl]" );
        }
      }
    }
  }
  if (sei.m_bpMaxSubLayers-1 > 0) 
  {
    WRITE_FLAG(sei.m_sublayerDpbOutputOffsetsPresentFlag, "bp_sublayer_dpb_output_offsets_present_flag");
  }

  if(sei.m_sublayerDpbOutputOffsetsPresentFlag)
  {
    for(int i = 0; i < sei.m_bpMaxSubLayers - 1; i++)
    {
      WRITE_UVLC( sei.m_dpbOutputTidOffset[i], "dpb_output_tid_offset[i]" );
    }
  }
  WRITE_FLAG(sei.m_altCpbParamsPresentFlag, "bp_alt_cpb_params_present_flag");
  if (sei.m_altCpbParamsPresentFlag)
  {
    WRITE_FLAG(sei.m_useAltCpbParamsFlag, "use_alt_cpb_params_flag");
  }

}

void SEIWriter::xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SEIBufferingPeriod &bp, const uint32_t temporalId)
{

  WRITE_CODE( sei.m_auCpbRemovalDelay[bp.m_bpMaxSubLayers - 1] - 1, bp.m_cpbRemovalDelayLength,               "pt_cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  for (int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i++)
  {
    WRITE_FLAG(sei.m_ptSubLayerDelaysPresentFlag[i], "pt_sub_layer_delays_present_flag[i]");
    if (sei.m_ptSubLayerDelaysPresentFlag[i])
    {
      if (bp.m_cpbRemovalDelayDeltasPresentFlag)
      {
        WRITE_FLAG(sei.m_cpbRemovalDelayDeltaEnabledFlag[i], "pt_cpb_removal_delay_delta_enabled_flag[i]");
      }
      if (sei.m_cpbRemovalDelayDeltaEnabledFlag[i])
      {
        if ((bp.m_numCpbRemovalDelayDeltas - 1) > 0)
        {
          WRITE_CODE(sei.m_cpbRemovalDelayDeltaIdx[i], ceilLog2(bp.m_numCpbRemovalDelayDeltas), "pt_cpb_removal_delay_delta_idx[i]");
        }
      }
      else
      {
        WRITE_CODE(sei.m_auCpbRemovalDelay[i] - 1, bp.m_cpbRemovalDelayLength, "pt_cpb_removal_delay_minus1[i]");
      }
    }
  }
  WRITE_CODE(sei.m_picDpbOutputDelay, bp.m_dpbOutputDelayLength, "pt_dpb_output_delay");
  if( bp.m_altCpbParamsPresentFlag )
  {
    WRITE_FLAG( sei.m_cpbAltTimingInfoPresentFlag, "cpb_alt_timing_info_present_flag" );
    if( sei.m_cpbAltTimingInfoPresentFlag )
    {
      if (bp.m_bpNalCpbParamsPresentFlag)
      {
        for (int i = (bp.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.m_bpMaxSubLayers - 1); i <= bp.m_bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.m_bpCpbCnt; j++)
          {
            WRITE_CODE(sei.m_nalCpbAltInitialRemovalDelayDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "nal_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            WRITE_CODE(sei.m_nalCpbAltInitialRemovalOffsetDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "nal_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
          }
          WRITE_CODE(sei.m_nalCpbDelayOffset[i], bp.m_cpbRemovalDelayLength, "nal_cpb_delay_offset[ i ]");
          WRITE_CODE(sei.m_nalDpbDelayOffset[i], bp.m_dpbOutputDelayLength, "nal_dpb_delay_offset[ i ]");
        }
      }

      if (bp.m_bpVclCpbParamsPresentFlag)
      {
        for (int i = (bp.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.m_bpMaxSubLayers - 1);
             i <= bp.m_bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.m_bpCpbCnt; j++)
          {
            WRITE_CODE(sei.m_vclCpbAltInitialRemovalDelayDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "vcl_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            WRITE_CODE(sei.m_vclCpbAltInitialRemovalOffsetDelta[i][j], bp.m_initialCpbRemovalDelayLength,
                       "vcl_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
          }
          WRITE_CODE(sei.m_vclCpbDelayOffset[i], bp.m_cpbRemovalDelayLength, "vcl_cpb_delay_offset[ i ]");
          WRITE_CODE(sei.m_vclDpbDelayOffset[i], bp.m_dpbOutputDelayLength,  "vcl_dpb_delay_offset[ i ]");
        }
      }
    }
  }
  if (bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitDpbDuParamsInPicTimingSeiFlag)

  {
    WRITE_CODE( sei.m_picDpbOutputDuDelay, bp.m_dpbOutputDelayDuLength, "pic_dpb_output_du_delay" );
  }
  if( bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    WRITE_UVLC( sei.m_numDecodingUnitsMinus1, "num_decoding_units_minus1" );
    if (sei.m_numDecodingUnitsMinus1 > 0)
    {
      WRITE_FLAG( sei.m_duCommonCpbRemovalDelayFlag, "du_commmon_cpb_removal_delay_flag" );
      if( sei.m_duCommonCpbRemovalDelayFlag )
      {
        for( int i = temporalId; i <= bp.m_bpMaxSubLayers - 1; i ++ )
        {
          if( sei.m_ptSubLayerDelaysPresentFlag[i] )
            WRITE_CODE( sei.m_duCommonCpbRemovalDelayMinus1[i], bp.m_duCpbRemovalDelayIncrementLength, "du_common_cpb_removal_delay_increment_minus1[i]" );
        }
      }
      for( int i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
      {
        WRITE_UVLC( sei.m_numNalusInDuMinus1[i], "num_nalus_in_du_minus1[i]" );
        if( !sei.m_duCommonCpbRemovalDelayFlag && i < sei.m_numDecodingUnitsMinus1 )
        {
          for( int j = temporalId; j <= bp.m_bpMaxSubLayers - 1; j ++ )
          {
            if( sei.m_ptSubLayerDelaysPresentFlag[j] )
              WRITE_CODE( sei.m_duCpbRemovalDelayMinus1[i * bp.m_bpMaxSubLayers + j], bp.m_duCpbRemovalDelayIncrementLength, "du_cpb_removal_delay_increment_minus1[i][j]" );
          }
        }
      }
    }
  }
#if !JVET_S0175_ASPECT5
  WRITE_UVLC( sei.m_ptDisplayElementalPeriodsMinus1,          "pt_display_elemental_periods_minus1" );
#else
  WRITE_CODE( sei.m_ptDisplayElementalPeriodsMinus1, 8,       "pt_display_elemental_periods_minus1" );
#endif
}

void SEIWriter::xWriteSEIFrameFieldInfo(const SEIFrameFieldInfo& sei)
{
  WRITE_FLAG( sei.m_fieldPicFlag ? 1 : 0,                    "ffi_field_pic_flag" );
  if (sei.m_fieldPicFlag)
  {
    WRITE_FLAG( sei.m_bottomFieldFlag ? 1 : 0,               "ffi_bottom_field_flag" );
    WRITE_FLAG( sei.m_pairingIndicatedFlag ? 1 : 0,          "ffi_pairing_indicated_flag" );
    if (sei.m_pairingIndicatedFlag)
    {
      WRITE_FLAG( sei.m_pairedWithNextFieldFlag ? 1 : 0,     "ffi_paired_with_next_field_flag" );
    }
  }
  else
  {
    WRITE_FLAG( sei.m_displayFieldsFromFrameFlag ? 1 : 0,     "ffi_display_fields_from_frame_flag" );
    if (sei.m_displayFieldsFromFrameFlag)
    {
      WRITE_FLAG( sei.m_topFieldFirstFlag ? 1 : 0,            "ffi_top_field_first_flag" );
    }
#if !JVET_S0175_ASPECT5
    WRITE_UVLC( sei.m_displayElementalPeriodsMinus1,          "ffi_display_elemental_periods_minus1" );
#else
    WRITE_CODE( sei.m_displayElementalPeriodsMinus1, 8,       "ffi_display_elemental_periods_minus1" );
#endif
  }
  WRITE_CODE( sei.m_sourceScanType, 2,                        "ffi_source_scan_type" );
  WRITE_FLAG( sei.m_duplicateFlag ? 1 : 0,                    "ffi_duplicate_flag" );
}

void SEIWriter::xWriteSEIDependentRAPIndication(const SEIDependentRAPIndication& /*sei*/)
{
  // intentionally empty
}

void SEIWriter::xWriteSEIScalableNesting(OutputBitstream& bs, const SEIScalableNesting& sei)
{
  CHECK_(sei.m_nestedSEIs.size()<1, "There must be at lease one SEI message nested in the scalable nesting SEI.")

  WRITE_FLAG(sei.m_snOlsFlag, "sn_ols_flag");
  WRITE_FLAG(sei.m_snSubpicFlag, "sn_subpic_flag");
  if (sei.m_snOlsFlag)
  {
    WRITE_UVLC(sei.m_snNumOlssMinus1, "sn_num_olss_minus1");
    for (uint32_t i = 0; i <= sei.m_snNumOlssMinus1; i++)
    {
      WRITE_UVLC(sei.m_snOlsIdxDeltaMinus1[i], "sn_ols_idx_delta_minus1[i]");
    }
  }
  else
  {
    WRITE_FLAG(sei.m_snAllLayersFlag, "sn_all_layers_flag");
    if (!sei.m_snAllLayersFlag)
    {
      WRITE_UVLC(sei.m_snNumLayersMinus1, "sn_num_layers");
      for (uint32_t i = 1; i <= sei.m_snNumLayersMinus1; i++)
      {
        WRITE_CODE(sei.m_snLayerId[i], 6, "sn_layer_id");
      }
    }
  }
  if (sei.m_snSubpicFlag)
  {
    WRITE_UVLC( sei.m_snNumSubpics - 1, "sn_num_subpics_minus1");
    CHECK_(sei.m_snSubpicIdLen < 1, "sn_subpic_id_len_minus1 must be >= 0");
    WRITE_UVLC( sei.m_snSubpicIdLen - 1, "sn_subpic_id_len_minus1");
    for (uint32_t i = 0; i < sei.m_snNumSubpics; i++)
    {
      WRITE_CODE(sei.m_snSubpicId[i], sei.m_snSubpicIdLen, "sn_subpic_id[i]");
    }
  }

  WRITE_UVLC( (uint32_t)sei.m_nestedSEIs.size() - 1, "sn_num_seis_minus1");

  // byte alignment
  while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    WRITE_FLAG(0, "sn_zero_bit");
  }

  SEIMessages bufferingPeriod = getSeisByType(sei.m_nestedSEIs, SEI::BUFFERING_PERIOD);
  if (!bufferingPeriod.empty())
  {
    SEIBufferingPeriod *bp = (SEIBufferingPeriod*)bufferingPeriod.front();
    m_nestingHrd.setBufferingPeriodSEI(bp);
  }

  // write nested SEI messages
  writeSEImessages(bs, sei.m_nestedSEIs, m_nestingHrd, true, 0);
}

void SEIWriter::xWriteSEIFramePacking(const SEIFramePacking& sei)
{
  WRITE_UVLC( sei.m_arrangementId,                  "fp_arrangement_id" );
  WRITE_FLAG( sei.m_arrangementCancelFlag,          "fp_arrangement_cancel_flag" );

  if( sei.m_arrangementCancelFlag == 0 )
  {
    WRITE_CODE( sei.m_arrangementType, 7,           "fp_arrangement_type" );

    WRITE_FLAG( sei.m_quincunxSamplingFlag,         "fp_quincunx_sampling_flag" );
    WRITE_CODE( sei.m_contentInterpretationType, 6, "fp_content_interpretation_type" );
    WRITE_FLAG( sei.m_spatialFlippingFlag,          "fp_spatial_flipping_flag" );
    WRITE_FLAG( sei.m_frame0FlippedFlag,            "fp_frame0_flipped_flag" );
    WRITE_FLAG( sei.m_fieldViewsFlag,               "fp_field_views_flag" );
    WRITE_FLAG( sei.m_currentFrameIsFrame0Flag,     "fp_current_frame_is_frame0_flag" );

    WRITE_FLAG( sei.m_frame0SelfContainedFlag,      "fp_frame0_self_contained_flag" );
    WRITE_FLAG( sei.m_frame1SelfContainedFlag,      "fp_frame1_self_contained_flag" );

    if(sei.m_quincunxSamplingFlag == 0 && sei.m_arrangementType != 5)
    {
      WRITE_CODE( sei.m_frame0GridPositionX, 4,     "fp_frame0_grid_position_x" );
      WRITE_CODE( sei.m_frame0GridPositionY, 4,     "fp_frame0_grid_position_y" );
      WRITE_CODE( sei.m_frame1GridPositionX, 4,     "fp_frame1_grid_position_x" );
      WRITE_CODE( sei.m_frame1GridPositionY, 4,     "fp_frame1_grid_position_y" );
    }

    WRITE_CODE( sei.m_arrangementReservedByte, 8,   "fp_arrangement_reserved_byte" );
    WRITE_FLAG( sei.m_arrangementPersistenceFlag,   "fp_arrangement_persistence_flag" );
  }

  WRITE_FLAG( sei.m_upsampledAspectRatio,           "fp_upsampled_aspect_ratio_flag" );
}


void SEIWriter::xWriteSEIParameterSetsInclusionIndication(const SEIParameterSetsInclusionIndication& sei)
{
  WRITE_FLAG(sei.m_selfContainedClvsFlag, "psii_self_contained_clvs_flag");
}

void SEIWriter::xWriteSEIMasteringDisplayColourVolume(const SEIMasteringDisplayColourVolume& sei)
{
  WRITE_CODE( sei.values.primaries[0][0],  16,  "mdcv_display_primaries_x[0]" );
  WRITE_CODE( sei.values.primaries[0][1],  16,  "mdcv_display_primaries_y[0]" );

  WRITE_CODE( sei.values.primaries[1][0],  16,  "mdcv_display_primaries_x[1]" );
  WRITE_CODE( sei.values.primaries[1][1],  16,  "mdcv_display_primaries_y[1]" );

  WRITE_CODE( sei.values.primaries[2][0],  16,  "mdcv_display_primaries_x[2]" );
  WRITE_CODE( sei.values.primaries[2][1],  16,  "mdcv_display_primaries_y[2]" );

  WRITE_CODE( sei.values.whitePoint[0],    16,  "mdcv_white_point_x" );
  WRITE_CODE( sei.values.whitePoint[1],    16,  "mdcv_white_point_y" );

  WRITE_CODE( sei.values.maxLuminance,     32,  "mdcv_max_display_mastering_luminance" );
  WRITE_CODE( sei.values.minLuminance,     32,  "mdcv_min_display_mastering_luminance" );
}

void SEIWriter::xWriteByteAlign()
{
  if( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    WRITE_FLAG( 1, "payload_bit_equal_to_one" );
    while( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0 )
    {
      WRITE_FLAG( 0, "payload_bit_equal_to_zero" );
    }
  }
}

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
void SEIWriter::xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei)
{
  WRITE_CODE(sei.m_preferredTransferCharacteristics, 8, "preferred_transfer_characteristics");
}
#endif

void SEIWriter::xWriteSEIEquirectangularProjection(const SEIEquirectangularProjection &sei)
{
  WRITE_FLAG( sei.m_erpCancelFlag, "erp_cancel_flag" );
  if( !sei.m_erpCancelFlag )
  {
    WRITE_FLAG( sei.m_erpPersistenceFlag, "erp_persistence_flag" );
    WRITE_FLAG( sei.m_erpGuardBandFlag,   "erp_guard_band_flag" );
    WRITE_CODE( 0, 2, "erp_reserved_zero_2bits" );
    if ( sei.m_erpGuardBandFlag == 1)
    {
      WRITE_CODE( sei.m_erpGuardBandType,       3, "erp_guard_band_type" );
      WRITE_CODE( sei.m_erpLeftGuardBandWidth,  8, "erp_left_guard_band_width" );
      WRITE_CODE( sei.m_erpRightGuardBandWidth, 8, "erp_right_guard_band_width" );
    }
  }
}

void SEIWriter::xWriteSEISphereRotation(const SEISphereRotation &sei)
{
  WRITE_FLAG( sei.m_sphereRotationCancelFlag,           "sphere_rotation_cancel_flag" );
  if( !sei.m_sphereRotationCancelFlag )
  {
    WRITE_FLAG( sei.m_sphereRotationPersistenceFlag,    "sphere_rotation_persistence_flag" );
    WRITE_CODE( 0,                                   6, "sphere_rotation_reserved_zero_6bits" );
    WRITE_SCODE(sei.m_sphereRotationYaw,            32, "sphere_rotation_yaw" );
    WRITE_SCODE(sei.m_sphereRotationPitch,          32, "sphere_rotation_pitch" );
    WRITE_SCODE(sei.m_sphereRotationRoll,           32, "sphere_rotation_roll" );
  }
}

void SEIWriter::xWriteSEIOmniViewport(const SEIOmniViewport &sei)
{
  WRITE_CODE( sei.m_omniViewportId,     10,        "omni_viewport_id" );
  WRITE_FLAG( sei.m_omniViewportCancelFlag,        "omni_viewport_cancel_flag" );
  if ( !sei.m_omniViewportCancelFlag )
  {
    WRITE_FLAG( sei.m_omniViewportPersistenceFlag, "omni_viewport_persistence_flag" );
    const uint32_t numRegions = (uint32_t) sei.m_omniViewportRegions.size();
    WRITE_CODE( numRegions - 1, 4,                 "omni_viewport_cnt_minus1" );
    for(uint32_t region=0; region<numRegions; region++)
    {
      const SEIOmniViewport::OmniViewport &viewport=sei.m_omniViewportRegions[region];
      WRITE_SCODE( viewport.azimuthCentre,     32, "omni_viewport_azimuth_centre"   );
      WRITE_SCODE( viewport.elevationCentre,   32, "omni_viewport_elevation_centre" );
      WRITE_SCODE( viewport.tiltCentre,        32, "omni_viewport_tilt_center" );
      WRITE_CODE( viewport.horRange,           32, "omni_viewport_hor_range[i]" );
      WRITE_CODE( viewport.verRange,           32, "omni_viewport_ver_range[i]" );
    }
  }
}

void SEIWriter::xWriteSEIRegionWisePacking(const SEIRegionWisePacking &sei)
{
  WRITE_FLAG( sei.m_rwpCancelFlag,                                           "rwp_cancel_flag" );
  if(!sei.m_rwpCancelFlag)
  {
    WRITE_FLAG( sei.m_rwpPersistenceFlag,                                    "rwp_persistence_flag" );
    WRITE_FLAG( sei.m_constituentPictureMatchingFlag,                        "rwp_constituent_picture_matching_flag" );
    WRITE_CODE( 0, 5,                                                        "rwp_reserved_zero_5bits" );
    WRITE_CODE( (uint32_t)sei.m_numPackedRegions,                 8,         "rwp_num_packed_regions" );
    WRITE_CODE( (uint32_t)sei.m_projPictureWidth,                 32,        "rwp_proj_picture_width" );
    WRITE_CODE( (uint32_t)sei.m_projPictureHeight,                32,        "rwp_proj_picture_height" );
    WRITE_CODE( (uint32_t)sei.m_packedPictureWidth,               16,        "rwp_packed_picture_width" );
    WRITE_CODE( (uint32_t)sei.m_packedPictureHeight,              16,        "rwp_packed_picture_height" );
    for( int i=0; i < sei.m_numPackedRegions; i++ )
    {
      WRITE_CODE( 0, 4,                                                      "rwp_reserved_zero_4bits" );
      WRITE_CODE( (uint32_t)sei.m_rwpTransformType[i],            3,         "rwp_transform_type" );
      WRITE_FLAG( sei.m_rwpGuardBandFlag[i],                                 "rwp_guard_band_flag" );
      WRITE_CODE( (uint32_t)sei.m_projRegionWidth[i],             32,        "rwp_proj_region_width" );
      WRITE_CODE( (uint32_t)sei.m_projRegionHeight[i],            32,        "rwp_proj_region_height" );
      WRITE_CODE( (uint32_t)sei.m_rwpProjRegionTop[i],            32,        "rwp_proj_region_top" );
      WRITE_CODE( (uint32_t)sei.m_projRegionLeft[i],              32,        "rwp_proj_region_left" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionWidth[i],           16,        "rwp_packed_region_width" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionHeight[i],          16,        "rwp_packed_region_height" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionTop[i],             16,        "rwp_packed_region_top" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionLeft[i],            16,        "rwp_packed_region_left" );
      if( sei.m_rwpGuardBandFlag[i] )
      {
        WRITE_CODE( (uint32_t)sei.m_rwpLeftGuardBandWidth[i],     8,         "rwp_left_guard_band_width");
        WRITE_CODE( (uint32_t)sei.m_rwpRightGuardBandWidth[i],    8,         "rwp_right_guard_band_width");
        WRITE_CODE( (uint32_t)sei.m_rwpTopGuardBandHeight[i],     8,         "rwp_top_guard_band_height");
        WRITE_CODE( (uint32_t)sei. m_rwpBottomGuardBandHeight[i], 8,         "rwp_bottom_guard_band_height");
        WRITE_FLAG( sei.m_rwpGuardBandNotUsedForPredFlag[i],                 "rwp_guard_band_not_used_for_pred_flag" );
        for( int j=0; j < 4; j++ )
        {
          WRITE_CODE( (uint32_t)sei.m_rwpGuardBandType[i*4 + j],  3,         "rwp_guard_band_type");
        }
        WRITE_CODE( 0, 3,                                                    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

void SEIWriter::xWriteSEIGeneralizedCubemapProjection(const SEIGeneralizedCubemapProjection &sei)
{
  WRITE_FLAG( sei.m_gcmpCancelFlag,                           "gcmp_cancel_flag" );
  if (!sei.m_gcmpCancelFlag)
  {
    WRITE_FLAG( sei.m_gcmpPersistenceFlag,                    "gcmp_persistence_flag" );
    WRITE_CODE( sei.m_gcmpPackingType,                     3, "gcmp_packing_type" );
    WRITE_CODE( sei.m_gcmpMappingFunctionType,             2, "gcmp_mapping_function_type" );
    int numFace = sei.m_gcmpPackingType == 4 || sei.m_gcmpPackingType == 5 ? 5 : 6;
    for (int i = 0; i < numFace; i++)
    {
      WRITE_CODE( sei.m_gcmpFaceIndex[i],                  3, "gcmp_face_index" );
      WRITE_CODE( sei.m_gcmpFaceRotation[i],               2, "gcmp_face_rotation" );
      if (sei.m_gcmpMappingFunctionType == 2)
      {
        WRITE_CODE( sei.m_gcmpFunctionCoeffU[i],           7, "gcmp_function_coeff_u" );
        WRITE_FLAG( sei.m_gcmpFunctionUAffectedByVFlag[i],    "gcmp_function_u_affected_by_v_flag" );
        WRITE_CODE( sei.m_gcmpFunctionCoeffV[i],           7, "gcmp_function_coeff_v" );
        WRITE_FLAG( sei.m_gcmpFunctionVAffectedByUFlag[i],    "gcmp_function_v_affected_by_u_flag" );
      }
    }
    WRITE_FLAG( sei.m_gcmpGuardBandFlag,                      "gcmp_guard_band_flag" );
    if (sei.m_gcmpGuardBandFlag)
    {
      WRITE_CODE( sei.m_gcmpGuardBandType,                 3, "gcmp_guard_band_type" );
      WRITE_FLAG( sei.m_gcmpGuardBandBoundaryExteriorFlag,    "gcmp_guard_band_boundary_exterior_flag" );
      WRITE_CODE( sei.m_gcmpGuardBandSamplesMinus1,        4, "gcmp_guard_band_samples_minus1" );
    }
  }
}

void SEIWriter::xWriteSEISubpictureLevelInfo(const SEISubpicureLevelInfo &sei)
{
  CHECK_(sei.m_numRefLevels < 1, "SEISubpicureLevelInfo: numRefLevels must be greater than zero");
  CHECK_(sei.m_numRefLevels != (int)sei.m_refLevelIdc.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of levels");
  if (sei.m_explicitFractionPresentFlag)
  {
    CHECK_(sei.m_numRefLevels != (int)sei.m_refLevelFraction.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of fractions");
  }
  WRITE_CODE( (uint32_t)sei.m_numRefLevels - 1, 3,                            "sli_num_ref_levels_minus1");
  WRITE_FLAG(           sei.m_cbrConstraintFlag,                              "sli_cbr_constraint_flag");
  WRITE_FLAG(           sei.m_explicitFractionPresentFlag,                    "sli_explicit_fraction_present_flag");
  if (sei.m_explicitFractionPresentFlag)
  {
    WRITE_UVLC(         sei.m_numSubpics -1 ,                                 "sli_num_subpics_minus1");
    WRITE_CODE( (uint32_t)sei.m_sliMaxSublayers - 1, 3,                       "sli_max_sublayers_minus1");
    WRITE_FLAG(           sei.m_sliSublayerInfoPresentFlag,                   "sli_sublayer_info_present_flag");
    while (!isByteAligned())
    {
      WRITE_FLAG(       0,                                                    "sli_alignment_zero_bit");
    }
  }

  for (int k = sei.m_sliSublayerInfoPresentFlag ? 0 : sei.m_sliMaxSublayers - 1; k < sei.m_sliMaxSublayers; k++)
  {
    for (int i = 0; i < sei.m_numRefLevels; i++)
    {
      WRITE_CODE((uint32_t)sei.m_nonSubpicLayersFraction[i][k], 8, "sli_non_subpic_layers_fraction[i][k]");
      WRITE_CODE((uint32_t)sei.m_refLevelIdc[i][k], 8, "sli_ref_level_idc[i][k]");
      if (sei.m_explicitFractionPresentFlag)
      {
        CHECK_(sei.m_numSubpics != (int)sei.m_refLevelFraction[i].size(), "SEISubpicureLevelInfo: number of fractions differs from number of subpictures");
        for (int j = 0; j < sei.m_numSubpics; j++)
        {
          WRITE_CODE((uint32_t)sei.m_refLevelFraction[i][j][k], 8, "sli_ref_level_fraction_minus1[i][j][k]");
        }
      }
    }
  }
}

void SEIWriter::xWriteSEISampleAspectRatioInfo(const SEISampleAspectRatioInfo &sei)
{
  WRITE_FLAG( sei.m_sariCancelFlag,                                           "sari_cancel_flag" );
  if(!sei.m_sariCancelFlag)
  {
    WRITE_FLAG( sei.m_sariPersistenceFlag,                                    "sari_persistence_flag" );
    WRITE_CODE( (uint32_t)sei.m_sariAspectRatioIdc, 8,                        "sari_aspect_ratio_idc");
    if (sei.m_sariAspectRatioIdc == 255)
    {
      WRITE_CODE( (uint32_t)sei.m_sariSarWidth, 16,                           "sari_sar_width");
      WRITE_CODE( (uint32_t)sei.m_sariSarHeight, 16,                          "sari_sar_height");
    }
  }
}

void SEIWriter::xWriteSEIUserDataRegistered(const SEIUserDataRegistered &sei)
{
  WRITE_CODE((sei.m_ituCountryCode>255) ? 0xff : sei.m_ituCountryCode, 8, "itu_t_t35_country_code");
  if (sei.m_ituCountryCode >= 255)
  {
    assert(sei.m_ituCountryCode < 255 + 256);
    WRITE_CODE(sei.m_ituCountryCode - 255, 8, "itu_t_t35_country_code_extension_byte");
  }
  for (uint32_t i = 0; i<sei.m_userData.size(); i++)
  {
    WRITE_CODE(sei.m_userData[i], 8, "itu_t_t35_payload_byte");
  }
}

void SEIWriter::xWriteSEIFilmGrainCharacteristics(const SEIFilmGrainCharacteristics &sei)
{
  WRITE_FLAG(sei.m_filmGrainCharacteristicsCancelFlag,        "fg_characteristics_cancel_flag");
  if (!sei.m_filmGrainCharacteristicsCancelFlag)
  {
    WRITE_CODE(sei.m_filmGrainModelId, 2,                     "fg_model_id");
    WRITE_FLAG(sei.m_separateColourDescriptionPresentFlag,    "fg_separate_colour_description_present_flag");
    if (sei.m_separateColourDescriptionPresentFlag)
    {
      WRITE_CODE(sei.m_filmGrainBitDepthLumaMinus8, 3,        "fg_bit_depth_luma_minus8");
      WRITE_CODE(sei.m_filmGrainBitDepthChromaMinus8, 3,      "fg_bit_depth_chroma_minus8");
      WRITE_FLAG(sei.m_filmGrainFullRangeFlag,                "fg_full_range_flag");
      WRITE_CODE(sei.m_filmGrainColourPrimaries, 8,           "fg_colour_primaries");
      WRITE_CODE(sei.m_filmGrainTransferCharacteristics, 8,   "fg_transfer_characteristics");
      WRITE_CODE(sei.m_filmGrainMatrixCoeffs, 8,              "fg_matrix_coeffs");
    }
    WRITE_CODE(sei.m_blendingModeId, 2,                       "fg_blending_mode_id");
    WRITE_CODE(sei.m_log2ScaleFactor, 4,                      "fg_log2_scale_factor");
    for (int c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t)cm.intensityValues.size();
      const uint32_t numModelValues = cm.numModelValues;
      WRITE_FLAG(sei.m_compModel[c].presentFlag && numIntensityIntervals>0 && numModelValues>0, "fg_comp_model_present_flag[c]");
    }
    for (uint32_t c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t)cm.intensityValues.size();
      const uint32_t numModelValues = cm.numModelValues;
      if (cm.presentFlag && numIntensityIntervals>0 && numModelValues>0)
      {
        assert(numIntensityIntervals <= 256);
        assert(numModelValues <= 256);
        WRITE_CODE(numIntensityIntervals - 1, 8,              "fg_num_intensity_intervals_minus1[c]");
        WRITE_CODE(numModelValues - 1, 3,                     "fg_num_model_values_minus1[c]");
        for (uint32_t interval = 0; interval<numIntensityIntervals; interval++)
        {
          const SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv = cm.intensityValues[interval];
          WRITE_CODE(cmiv.intensityIntervalLowerBound, 8,     "fg_intensity_interval_lower_bound[c][i]");
          WRITE_CODE(cmiv.intensityIntervalUpperBound, 8,     "fg_intensity_interval_upper_bound[c][i]");
          assert(cmiv.compModelValue.size() == numModelValues);
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            WRITE_SVLC(cmiv.compModelValue[j],                "fg_comp_model_value[c][i]");
          }
        }
      }
    } // for c
    WRITE_FLAG(sei.m_filmGrainCharacteristicsPersistenceFlag, "fg_characteristics_persistence_flag");
  } // cancel flag
}

void SEIWriter::xWriteSEIContentLightLevelInfo(const SEIContentLightLevelInfo& sei)
{
  WRITE_CODE( sei.m_maxContentLightLevel,    16, "clli_max_content_light_level"     );
  WRITE_CODE( sei.m_maxPicAverageLightLevel, 16, "clli_max_pic_average_light_level" );
}

void SEIWriter::xWriteSEIAmbientViewingEnvironment(const SEIAmbientViewingEnvironment& sei)
{
  WRITE_CODE(sei.m_ambientIlluminance, 32, "ambient_illuminance" );
  WRITE_CODE(sei.m_ambientLightX,      16, "ambient_light_x" );
  WRITE_CODE(sei.m_ambientLightY,      16, "ambient_light_y" );
}

void SEIWriter::xWriteSEIContentColourVolume(const SEIContentColourVolume &sei)
{
  WRITE_FLAG(sei.m_ccvCancelFlag, "ccv_cancel_flag");
  if (!sei.m_ccvCancelFlag)
  {
    WRITE_FLAG(sei.m_ccvPersistenceFlag, "ccv_persistence_flag");
    WRITE_FLAG(sei.m_ccvPrimariesPresentFlag, "ccv_primaries_present_flag");
    WRITE_FLAG(sei.m_ccvMinLuminanceValuePresentFlag, "ccv_min_luminance_value_present_flag");
    WRITE_FLAG(sei.m_ccvMaxLuminanceValuePresentFlag, "ccv_max_luminance_value_present_flag");
    WRITE_FLAG(sei.m_ccvAvgLuminanceValuePresentFlag, "ccv_avg_luminance_value_present_flag");

    if (sei.m_ccvPrimariesPresentFlag == true)
    {
      for (int i = 0; i < MAX_NUM_COMPONENT; i++)
      {
        WRITE_SCODE((int32_t)sei.m_ccvPrimariesX[i], 32, "ccv_primaries_x[i]");
        WRITE_SCODE((int32_t)sei.m_ccvPrimariesY[i], 32, "ccv_primaries_y[i]");
      }
    }

    if (sei.m_ccvMinLuminanceValuePresentFlag == true)
    {
      WRITE_CODE((uint32_t)sei.m_ccvMinLuminanceValue, 32, "ccv_min_luminance_value");
    }
    if (sei.m_ccvMinLuminanceValuePresentFlag == true)
    {
      WRITE_CODE((uint32_t)sei.m_ccvMaxLuminanceValue, 32, "ccv_max_luminance_value");
    }
    if (sei.m_ccvMinLuminanceValuePresentFlag == true)
    {
      WRITE_CODE((uint32_t)sei.m_ccvAvgLuminanceValue, 32, "ccv_avg_luminance_value");
    }
  }
}

//! \}

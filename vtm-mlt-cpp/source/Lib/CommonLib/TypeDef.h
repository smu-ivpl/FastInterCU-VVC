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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#ifndef __TYPEDEF__
#define __TYPEDEF__

#ifndef __COMMONDEF__
#error Include CommonDef.h not TypeDef.h
#endif

#include <vector>
#include <utility>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <assert.h>
#include <cassert>

// clang-format off

//########### place macros to be removed in next cycle below this line ###############

#define JVET_S0096_RPL_CONSTRAINT                         1// JVET-S0096 aspect 1: When pps_rpl_info_in_ph_flag is equal to 1 and ph_inter_slice_allowed_flag is equal to 1, the value of num_ref_entries[ 0 ][ RplsIdx[ 0 ] ] shall be greater than 0.
#define JVET_S0078_NOOUTPUTPRIORPICFLAG                   0 // JVET-S0078: Handling of NoOutputOfPriorPicsFlag in output process
#define JVET_S0219_ASPECT1                                1 // JVET-S0219 aspect1 : removal non-referred APS parameter set in the non-output layer.
#define JVET_R0193                                        1 // JVET-R0193: signalling of the number of maximum sublayers used for inter-layer prediction for each layer
#define JVET_R0193_S0141                                  1 // JVET-S0141 item 47 : item 47: In the general sub-bitstream extraction process, specify the conditions under which an output sub-bitstream is required to be a conforming bitstream such that the value of tIdTarget is specified to be in the range of 0 to vps_ptl_max_tid[ vps_ols_ptl_idx[ targetOlsIdx ] ], inclusive (instead of 0 to 6 inclusive). (JVET-S0158 aspect 1)
#define JVET_T0065_LEVEL_6_3                              1 // JVET-T0065: Add level 6.3
#define JVET_T0091_LMCS_ENC_OVERFLOW_FIX                  1 // JVET-T0091: LMCS encoder overflow fix at high bit-depth for SDR
#define JVET_S0163_ON_TARGETOLS_SUBLAYERS                 1 // JVET-S0163: On target OLS and sublayers for decoding (OPI NAL Unit)
#define JVET_R0266_GCI                                    1 // JVET-R0266 #5: Specify that no_gdr_constraint_flag equal to 1 specifies that sps_gdr_enabled_flag shall be equal to 0
#define JVET_S0084_S0110_RADL                             1 // When the current picture is a RADL picture, allow RASL pictures with pps_mixed_nalu_types_in_pic_flag is equal to 1 in active entries in RefPicList[ 0 ] or RefPicList[ 1 ]
#define FIX_TICKET_1405                                   1 // Add dph_sei_single_component_flag and dph_sei_reserved_zero_7bits syntax to decoded picture hash SEI message
#define FIX_SUBPICS_W_RPR                                 1 // Fix handling of RPR with subpictures (via scaling windows with no resolution change)
#define JVET_S0175_ASPECT5                                1 // use u(8) instead of u(4) for (ffi_)display_elemental_periods_minus1 and pt_display_elemental_periods_minus1
#define JVET_S0175_ASPECT6                                1 // The general_nal_hrd_params_present_flag and general_vcl_hrd_params_present_flag are allowed to both be equal to 0
#define JVET_R0046_IRAP_ASPECT2                           1 // Add a constraint on an ILRP being either an IRAP picture or having TemporalId less than or equal to Max (0, vps_max_tid_il_ref_pics_plus1[ refPicVpsLayerId ] - 1 )
#define JVET_T0064                                        1 // JVET-T0064: control of filter strength for ALF

//########### place macros to be be kept below this line ###############
#define JVET_S0257_DUMP_360SEI_MESSAGE                    1 // Software support of 360 SEI messages

#define JVET_R0351_HIGH_BIT_DEPTH_ENABLED                 0 // JVET-R0351: high bit depth coding enabled (increases accuracies of some calculations, e.g. transforms)

#define JVET_R0164_MEAN_SCALED_SATD                       1 // JVET-R0164: Use a mean scaled version of SATD in encoder decisions

#define JVET_M0497_MATRIX_MULT                            0 // 0: Fast method; 1: Matrix multiplication

#define JVET_R0107_BITSTREAM_EXTACTION                    1 // JVET-R0107 Proposal 3:Bitsteam extraction modifications

#define APPLY_SBT_SL_ON_MTS                               1 // apply save & load fast algorithm on inter MTS when SBT is on

typedef std::pair<int, bool> TrMode;
typedef std::pair<int, int>  TrCost;

#define REUSE_CU_RESULTS                                  1
#if REUSE_CU_RESULTS
#define REUSE_CU_RESULTS_WITH_MULTIPLE_TUS                1
#endif

#ifndef JVET_J0090_MEMORY_BANDWITH_MEASURE
#define JVET_J0090_MEMORY_BANDWITH_MEASURE                0
#endif

#ifndef EXTENSION_360_VIDEO
#define EXTENSION_360_VIDEO                               0   ///< extension for 360/spherical video coding support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#ifndef EXTENSION_HDRTOOLS
#define EXTENSION_HDRTOOLS                                0 //< extension for HDRTools/Metrics support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#define JVET_O0756_CONFIG_HDRMETRICS                      1
#if EXTENSION_HDRTOOLS
#define JVET_O0756_CALCULATE_HDRMETRICS                   1
#endif

#ifndef ENABLE_SPLIT_PARALLELISM
#define ENABLE_SPLIT_PARALLELISM                          0
#endif
#if ENABLE_SPLIT_PARALLELISM
#define PARL_SPLIT_MAX_NUM_JOBS                           6                             // number of parallel jobs that can be defined and need memory allocated
#define NUM_RESERVERD_SPLIT_JOBS                        ( PARL_SPLIT_MAX_NUM_JOBS + 1 )  // number of all data structures including the merge thread (0)
#define PARL_SPLIT_MAX_NUM_THREADS                        PARL_SPLIT_MAX_NUM_JOBS
#define NUM_SPLIT_THREADS_IF_MSVC                         4

#endif

// clang-format on

// ====================================================================================================================
// General settings
// ====================================================================================================================

#ifndef ENABLE_TRACING
#define ENABLE_TRACING                                    0 // DISABLE by default (enable only when debugging, requires 15% run-time in decoding) -- see documentation in 'doc/DTrace for NextSoftware.pdf'
#endif

#if ENABLE_TRACING
#define K0149_BLOCK_STATISTICS                            1 // enables block statistics, which can be analysed with YUView (https://github.com/IENT/YUView)
#if K0149_BLOCK_STATISTICS
#define BLOCK_STATS_AS_CSV                                0 // statistics will be written in a comma separated value format. this is not supported by YUView
#endif
#endif

#define WCG_EXT                                           1
#define WCG_WPSNR                                         WCG_EXT

#define KEEP_PRED_AND_RESI_SIGNALS                        0

// ====================================================================================================================
// Debugging
// ====================================================================================================================

// most debugging tools are now bundled within the ENABLE_TRACING macro -- see documentation to see how to use

#define PRINT_MACRO_VALUES                                1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

#define INTRA_FULL_SEARCH                                 0 ///< enables full mode search for intra estimation

// TODO: rename this macro to DECODER_DEBUG_BIT_STATISTICS (may currently cause merge issues with other branches)
// This can be enabled by the makefile
#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS                0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run time by up to ~10%)
#endif

#ifndef RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
#define RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS         (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 (default) = decoder reports as normal, 1 = decoder produces max frame bit usage statistics
#endif

#define TR_ONLY_COEFF_STATS                              (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 combine TS and non-TS decoder debug statistics. 1 = separate TS and non-TS decoder debug statistics.
#define EPBINCOUNT_FIX                                   (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 use count to represent number of calls to decodeBins. 1 = count and bins for EP bins are the same.

#ifndef RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_TOOL_STATISTICS               0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces tool usage statistics
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS || RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_STATISTICS                    1
#endif

// ====================================================================================================================
// Tool Switches - transitory (these macros are likely to be removed in future revisions)
// ====================================================================================================================

#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES  1 ///< TODO: integrate this macro into a broader conformance checking system.
#define T0196_SELECTIVE_RDOQ                              1 ///< selective RDOQ
#define U0040_MODIFIED_WEIGHTEDPREDICTION_WITH_BIPRED_AND_CLIPPING 1
#define U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI    1 ///< Alternative transfer characteristics SEI message (JCTVC-U0033, with syntax naming from V1005)
#define X0038_LAMBDA_FROM_QP_CAPABILITY                   1 ///< This approach derives lambda from QP+QPoffset+QPoffset2. QPoffset2 is derived from QP+QPoffset using a linear model that is clipped between 0 and 3.
                                                            // To use this capability enable config parameter LambdaFromQpEnable

// ====================================================================================================================
// Tool Switches
// ====================================================================================================================


// This can be enabled by the makefile
#ifndef RExt__HIGH_BIT_DEPTH_SUPPORT
#if JVET_R0351_HIGH_BIT_DEPTH_ENABLED
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      1 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#else
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      0 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#endif
#endif

// SIMD optimizations
#define SIMD_ENABLE                                       1
#define ENABLE_SIMD_OPT                                 ( SIMD_ENABLE && !RExt__HIGH_BIT_DEPTH_SUPPORT )    ///< SIMD optimizations, no impact on RD performance
#define ENABLE_SIMD_OPT_MCIF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the interpolation filter, no impact on RD performance
#define ENABLE_SIMD_OPT_BUFFER                          ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the buffer operations, no impact on RD performance
#define ENABLE_SIMD_OPT_DIST                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the distortion calculations(SAD,SSE,HADAMARD), no impact on RD performance
#define ENABLE_SIMD_OPT_AFFINE_ME                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for affine ME, no impact on RD performance
#define ENABLE_SIMD_OPT_ALF                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for ALF
#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_BCW                               1                                                 ///< SIMD optimization for Bcw
#endif

// End of SIMD optimizations


#define ME_ENABLE_ROUNDING_OF_MVS                         1 ///< 0 (default) = disables rounding of motion vectors when right shifted,  1 = enables rounding

#define RDOQ_CHROMA_LAMBDA                                1 ///< F386: weighting of chroma for RDOQ

#define U0132_TARGET_BITS_SATURATION                      1 ///< Rate control with target bits saturation method
#ifdef  U0132_TARGET_BITS_SATURATION
#define V0078_ADAPTIVE_LOWER_BOUND                        1 ///< Target bits saturation with adaptive lower bound
#endif
#define W0038_DB_OPT                                      1 ///< adaptive DB parameter selection, LoopFilterOffsetInPPS and LoopFilterDisable are set to 0 and DeblockingFilterMetric=2;
#define W0038_CQP_ADJ                                     1 ///< chroma QP adjustment based on TL, CQPTLAdjustEnabled is set to 1;

#define SHARP_LUMA_DELTA_QP                               1 ///< include non-normative LCU deltaQP and normative chromaQP change
#define ER_CHROMA_QP_WCG_PPS                              1 ///< Chroma QP model for WCG used in Anchor 3.2
#define ENABLE_QPA                                        1 ///< Non-normative perceptual QP adaptation according to JVET-H0047 and JVET-K0206. Deactivated by default, activated using encoder arguments --PerceptQPA=1 --SliceChromaQPOffsetPeriodicity=1
#define ENABLE_QPA_SUB_CTU                              ( 1 && ENABLE_QPA ) ///< when maximum delta-QP depth is greater than zero, use sub-CTU QPA


#define RDOQ_CHROMA                                       1 ///< use of RDOQ in chroma

#define QP_SWITCHING_FOR_PARALLEL                         1 ///< Replace floating point QP with a source-file frame number. After switching POC, increase base QP instead of frame level QP.

#define LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET         1 /// JVET-L0414 (CE11.2.2) with explicit signalling of num interval, threshold and qpOffset
// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            1 ///< 0 use original 6-bit transform matrices for both forward and inverse transform, 1 (default) = use original matrices for inverse transform and high precision matrices for forward transform
#else
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            0 ///< 0 (default) use original 6-bit transform matrices for both forward and inverse transform, 1 = use original matrices for inverse transform and high precision matrices for forward transform
#endif

#if FULL_NBIT
#define DISTORTION_PRECISION_ADJUSTMENT(x)                0
#else
#define DISTORTION_ESTIMATION_BITS                        8
#define DISTORTION_PRECISION_ADJUSTMENT(x)                ((x>DISTORTION_ESTIMATION_BITS)? ((x)-DISTORTION_ESTIMATION_BITS) : 0)
#endif

// ====================================================================================================================
// Error checks
// ====================================================================================================================

#if ((RExt__HIGH_PRECISION_FORWARD_TRANSFORM != 0) && (RExt__HIGH_BIT_DEPTH_SUPPORT == 0))
#error ERROR_: cannot enable RExt__HIGH_PRECISION_FORWARD_TRANSFORM without RExt__HIGH_BIT_DEPTH_SUPPORT
#endif

// ====================================================================================================================
// Named numerical types
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
typedef       int             Pel;               ///< pixel type
typedef       int64_t           TCoeff;            ///< transform coefficient
typedef       int             TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int64_t           Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint64_t          Intermediate_UInt; ///< used as intermediate value in calculations
#else
typedef       int16_t           Pel;               ///< pixel type
typedef       int             TCoeff;            ///< transform coefficient
typedef       int16_t           TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int             Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint32_t            Intermediate_UInt; ///< used as intermediate value in calculations
#endif

typedef       uint64_t          SplitSeries;       ///< used to encoded the splits that caused a particular CU size
typedef       uint64_t          ModeTypeSeries;    ///< used to encoded the ModeType at different split depth

typedef       uint64_t        Distortion;        ///< distortion measurement

// ====================================================================================================================
// Enumeration
// ====================================================================================================================


enum ApsType
{
  ALF_APS = 0,
  LMCS_APS = 1,
  SCALING_LIST_APS = 2,
};

enum QuantFlags
{
  Q_INIT           = 0x0,
  Q_USE_RDOQ       = 0x1,
  Q_RDOQTS         = 0x2,
  Q_SELECTIVE_RDOQ = 0x4,
};

//EMT transform tags
enum TransType
{
  DCT2 = 0,
  DCT8 = 1,
  DST7 = 2,
  NUM_TRANS_TYPE = 3,
  DCT2_EMT = 4
};

enum MTSIdx
{
  MTS_DCT2_DCT2 = 0,
  MTS_SKIP = 1,
  MTS_DST7_DST7 = 2,
  MTS_DCT8_DST7 = 3,
  MTS_DST7_DCT8 = 4,
  MTS_DCT8_DCT8 = 5
};

enum ISPType
{
  NOT_INTRA_SUBPARTITIONS       = 0,
  HOR_INTRA_SUBPARTITIONS       = 1,
  VER_INTRA_SUBPARTITIONS       = 2,
  NUM_INTRA_SUBPARTITIONS_MODES = 3,
  INTRA_SUBPARTITIONS_RESERVED  = 4
};

enum SbtIdx
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  NUMBER_SBT_IDX,
  SBT_OFF_MTS, //note: must be after all SBT modes, only used in fast algorithm to discern the best mode is inter EMT
};

enum SbtPos
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

enum SbtMode
{
  SBT_VER_H0 = 0,
  SBT_VER_H1 = 1,
  SBT_HOR_H0 = 2,
  SBT_HOR_H1 = 3,
  SBT_VER_Q0 = 4,
  SBT_VER_Q1 = 5,
  SBT_HOR_Q0 = 6,
  SBT_HOR_Q1 = 7,
  NUMBER_SBT_MODE
};

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};

/// chroma formats (according to how the monochrome or the color planes are intended to be coded)
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
};

enum TreeType
{
  TREE_D = 0, //default tree status (for single-tree slice, TREE_D means joint tree; for dual-tree I slice, TREE_D means TREE_L for luma and TREE_C for chroma)
  TREE_L = 1, //separate tree only contains luma (may split)
  TREE_C = 2, //separate tree only contains chroma (not split), to avoid small chroma block
};

enum ModeType
{
  MODE_TYPE_ALL = 0, //all modes can try
  MODE_TYPE_INTER = 1, //can try inter
  MODE_TYPE_INTRA = 2, //can try intra, ibc, palette
};

#define CH_L CHANNEL_TYPE_LUMA
#define CH_C CHANNEL_TYPE_CHROMA

enum ComponentID
{
  COMPONENT_Y         = 0,
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT   = 3,
  JOINT_CbCr          = MAX_NUM_COMPONENT,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
};

#define MAP_CHROMA(c) (ComponentID(c))

enum InputColourSpaceConversion // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1, // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2, // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};

enum MATRIX_COEFFICIENTS // Table E.5 (Matrix coefficients)
{
  MATRIX_COEFFICIENTS_RGB                           = 0,
  MATRIX_COEFFICIENTS_BT709                         = 1,
  MATRIX_COEFFICIENTS_UNSPECIFIED                   = 2,
  MATRIX_COEFFICIENTS_RESERVED_BY_ITUISOIEC         = 3,
  MATRIX_COEFFICIENTS_USFCCT47                      = 4,
  MATRIX_COEFFICIENTS_BT601_625                     = 5,
  MATRIX_COEFFICIENTS_BT601_525                     = 6,
  MATRIX_COEFFICIENTS_SMPTE240                      = 7,
  MATRIX_COEFFICIENTS_YCGCO                         = 8,
  MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE = 9,
  MATRIX_COEFFICIENTS_BT2020_CONSTANT_LUMINANCE     = 10,
};

enum DeblockEdgeDir
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR = 2
};

/// supported prediction type
enum PredMode
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  MODE_IBC                   = 2,     ///< ibc-prediction mode
  MODE_PLT                   = 3,     ///< plt-prediction mode
  NUMBER_OF_PREDICTION_MODES = 4,
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0               = 0,   ///< reference list 0
  REF_PIC_LIST_1               = 1,   ///< reference list 1
  NUM_REF_PIC_LIST_01          = 2,
  REF_PIC_LIST_X               = 100  ///< special mark
};

#define L0 REF_PIC_LIST_0
#define L1 REF_PIC_LIST_1

/// distortion function index
enum DFunc
{
  DF_SSE             = 0,             ///< general size SSE
  DF_SSE2            = DF_SSE+1,      ///<   2xM SSE
  DF_SSE4            = DF_SSE+2,      ///<   4xM SSE
  DF_SSE8            = DF_SSE+3,      ///<   8xM SSE
  DF_SSE16           = DF_SSE+4,      ///<  16xM SSE
  DF_SSE32           = DF_SSE+5,      ///<  32xM SSE
  DF_SSE64           = DF_SSE+6,      ///<  64xM SSE
  DF_SSE16N          = DF_SSE+7,      ///< 16NxM SSE

  DF_SAD             = 8,             ///< general size SAD
  DF_SAD2            = DF_SAD+1,      ///<   2xM SAD
  DF_SAD4            = DF_SAD+2,      ///<   4xM SAD
  DF_SAD8            = DF_SAD+3,      ///<   8xM SAD
  DF_SAD16           = DF_SAD+4,      ///<  16xM SAD
  DF_SAD32           = DF_SAD+5,      ///<  32xM SAD
  DF_SAD64           = DF_SAD+6,      ///<  64xM SAD
  DF_SAD16N          = DF_SAD+7,      ///< 16NxM SAD

  DF_HAD             = 16,            ///< general size Hadamard
  DF_HAD2            = DF_HAD+1,      ///<   2xM HAD
  DF_HAD4            = DF_HAD+2,      ///<   4xM HAD
  DF_HAD8            = DF_HAD+3,      ///<   8xM HAD
  DF_HAD16           = DF_HAD+4,      ///<  16xM HAD
  DF_HAD32           = DF_HAD+5,      ///<  32xM HAD
  DF_HAD64           = DF_HAD+6,      ///<  64xM HAD
  DF_HAD16N          = DF_HAD+7,      ///< 16NxM HAD

  DF_SAD12           = 24,
  DF_SAD24           = 25,
  DF_SAD48           = 26,

  DF_MRSAD           = 27,            ///< general size MR SAD
  DF_MRSAD2          = DF_MRSAD+1,    ///<   2xM MR SAD
  DF_MRSAD4          = DF_MRSAD+2,    ///<   4xM MR SAD
  DF_MRSAD8          = DF_MRSAD+3,    ///<   8xM MR SAD
  DF_MRSAD16         = DF_MRSAD+4,    ///<  16xM MR SAD
  DF_MRSAD32         = DF_MRSAD+5,    ///<  32xM MR SAD
  DF_MRSAD64         = DF_MRSAD+6,    ///<  64xM MR SAD
  DF_MRSAD16N        = DF_MRSAD+7,    ///< 16NxM MR SAD

  DF_MRHAD           = 35,            ///< general size MR Hadamard
  DF_MRHAD2          = DF_MRHAD+1,    ///<   2xM MR HAD
  DF_MRHAD4          = DF_MRHAD+2,    ///<   4xM MR HAD
  DF_MRHAD8          = DF_MRHAD+3,    ///<   8xM MR HAD
  DF_MRHAD16         = DF_MRHAD+4,    ///<  16xM MR HAD
  DF_MRHAD32         = DF_MRHAD+5,    ///<  32xM MR HAD
  DF_MRHAD64         = DF_MRHAD+6,    ///<  64xM MR HAD
  DF_MRHAD16N        = DF_MRHAD+7,    ///< 16NxM MR HAD

  DF_MRSAD12         = 43,
  DF_MRSAD24         = 44,
  DF_MRSAD48         = 45,

  DF_SAD_FULL_NBIT    = 46,
  DF_SAD_FULL_NBIT2   = DF_SAD_FULL_NBIT+1,    ///<   2xM SAD with full bit usage
  DF_SAD_FULL_NBIT4   = DF_SAD_FULL_NBIT+2,    ///<   4xM SAD with full bit usage
  DF_SAD_FULL_NBIT8   = DF_SAD_FULL_NBIT+3,    ///<   8xM SAD with full bit usage
  DF_SAD_FULL_NBIT16  = DF_SAD_FULL_NBIT+4,    ///<  16xM SAD with full bit usage
  DF_SAD_FULL_NBIT32  = DF_SAD_FULL_NBIT+5,    ///<  32xM SAD with full bit usage
  DF_SAD_FULL_NBIT64  = DF_SAD_FULL_NBIT+6,    ///<  64xM SAD with full bit usage
  DF_SAD_FULL_NBIT16N = DF_SAD_FULL_NBIT+7,    ///< 16NxM SAD with full bit usage

  DF_SSE_WTD          = 54,                ///< general size SSE
  DF_SSE2_WTD         = DF_SSE_WTD+1,      ///<   4xM SSE
  DF_SSE4_WTD         = DF_SSE_WTD+2,      ///<   4xM SSE
  DF_SSE8_WTD         = DF_SSE_WTD+3,      ///<   8xM SSE
  DF_SSE16_WTD        = DF_SSE_WTD+4,      ///<  16xM SSE
  DF_SSE32_WTD        = DF_SSE_WTD+5,      ///<  32xM SSE
  DF_SSE64_WTD        = DF_SSE_WTD+6,      ///<  64xM SSE
  DF_SSE16N_WTD       = DF_SSE_WTD+7,      ///< 16NxM SSE
  DF_DEFAULT_ORI      = DF_SSE_WTD+8,

  DF_SAD_INTERMEDIATE_BITDEPTH = 63,

  DF_SAD_WITH_MASK   = 64,
  DF_TOTAL_FUNCTIONS = 65
};

/// motion vector predictor direction used in AMVP
enum MvpDir
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

enum TransformDirection
{
  TRANSFORM_FORWARD              = 0,
  TRANSFORM_INVERSE              = 1,
  TRANSFORM_NUMBER_OF_DIRECTIONS = 2
};

/// supported ME search methods
enum MESearchMethod
{
  MESEARCH_FULL              = 0,
  MESEARCH_DIAMOND           = 1,
  MESEARCH_SELECTIVE         = 2,
  MESEARCH_DIAMOND_ENHANCED  = 3,
  MESEARCH_NUMBER_OF_METHODS = 4
};

/// coefficient scanning type used in ACS
enum CoeffScanType
{
  SCAN_DIAG = 0,        ///< up-right diagonal scan
  SCAN_TRAV_HOR = 1,
  SCAN_TRAV_VER = 2,
  SCAN_NUMBER_OF_TYPES
};

enum CoeffScanGroupType
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum ScalingListMode
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
};

enum ScalingListSize
{
  SCALING_LIST_1x1 = 0,
  SCALING_LIST_2x2,
  SCALING_LIST_4x4,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_64x64,
  SCALING_LIST_128x128,
  SCALING_LIST_SIZE_NUM,
  //for user define matrix
  SCALING_LIST_FIRST_CODED = SCALING_LIST_2x2,
  SCALING_LIST_LAST_CODED = SCALING_LIST_64x64
};

enum ScalingList1dStartIdx
{
  SCALING_LIST_1D_START_2x2    = 0,
  SCALING_LIST_1D_START_4x4    = 2,
  SCALING_LIST_1D_START_8x8    = 8,
  SCALING_LIST_1D_START_16x16  = 14,
  SCALING_LIST_1D_START_32x32  = 20,
  SCALING_LIST_1D_START_64x64  = 26,
};

// For use with decoded picture hash SEI messages, generated by encoder.
enum HashType
{
  HASHTYPE_MD5             = 0,
  HASHTYPE_CRC             = 1,
  HASHTYPE_CHECKSUM        = 2,
  HASHTYPE_NONE            = 3,
  NUMBER_OF_HASHTYPES      = 4
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,

  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};

#define NUM_SAO_BO_CLASSES_LOG2  5
#define NUM_SAO_BO_CLASSES       (1<<NUM_SAO_BO_CLASSES_LOG2)

namespace Profile
{
  enum Name
  {
    NONE                                 = 0,
    STILL_PICTURE                        = 64,
    MAIN_10                              = 1,
    MAIN_10_STILL_PICTURE                = MAIN_10 | STILL_PICTURE,
    MULTILAYER_MAIN_10                   = 17,
    MULTILAYER_MAIN_10_STILL_PICTURE     = MULTILAYER_MAIN_10 | STILL_PICTURE,
    MAIN_10_444                          = 33,
    MAIN_10_444_STILL_PICTURE            = MAIN_10_444 | STILL_PICTURE,
    MULTILAYER_MAIN_10_444               = 49,
    MULTILAYER_MAIN_10_444_STILL_PICTURE = MULTILAYER_MAIN_10_444 | STILL_PICTURE,
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
    NUMBER_OF_TIERS=2
  };

  enum Name
  {
    // code = (major_level * 16 + minor_level * 3)
    NONE     = 0,
    LEVEL1   = 16,
    LEVEL2   = 32,
    LEVEL2_1 = 35,
    LEVEL3   = 48,
    LEVEL3_1 = 51,
    LEVEL4   = 64,
    LEVEL4_1 = 67,
    LEVEL5   = 80,
    LEVEL5_1 = 83,
    LEVEL5_2 = 86,
    LEVEL6   = 96,
    LEVEL6_1 = 99,
    LEVEL6_2 = 102,
#if JVET_T0065_LEVEL_6_3
    LEVEL6_3 = 105,
#endif
    LEVEL15_5 = 255,
  };
}

enum CostMode
{
  COST_STANDARD_LOSSY              = 0,
  COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  COST_LOSSLESS_CODING             = 2,
  COST_MIXED_LOSSLESS_LOSSY_CODING = 3
};

enum WeightedPredictionMethod
{
  WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT                          =0,
  WP_PER_PICTURE_WITH_SIMPLE_DC_PER_COMPONENT                               =1,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT                           =2,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING              =3,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING_AND_EXTENSION=4
};

enum FastInterSearchMode
{
  FASTINTERSEARCH_DISABLED = 0,
  FASTINTERSEARCH_MODE1    = 1, // TODO: assign better names to these.
  FASTINTERSEARCH_MODE2    = 2,
  FASTINTERSEARCH_MODE3    = 3
};

enum SPSExtensionFlagIndex
{
  SPS_EXT__REXT           = 0,
//SPS_EXT__MVHEVC         = 1, //for use in future versions
//SPS_EXT__SHVC           = 2, //for use in future versions
  SPS_EXT__NEXT           = 3,
  NUM_SPS_EXTENSION_FLAGS = 8
};

// TODO: Existing names used for the different NAL unit types can be altered to better reflect the names in the spec.
//       However, the names in the spec are not yet stable at this point. Once the names are stable, a cleanup
//       effort can be done without use of macros to alter the names used to indicate the different NAL unit types.
enum NalUnitType
{
  NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  NAL_UNIT_CODED_SLICE_STSA,        // 1
  NAL_UNIT_CODED_SLICE_RADL,        // 2
  NAL_UNIT_CODED_SLICE_RASL,        // 3

  NAL_UNIT_RESERVED_VCL_4,
  NAL_UNIT_RESERVED_VCL_5,
  NAL_UNIT_RESERVED_VCL_6,

  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  NAL_UNIT_CODED_SLICE_CRA,         // 9
  NAL_UNIT_CODED_SLICE_GDR,         // 10

  NAL_UNIT_RESERVED_IRAP_VCL_11,
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  NAL_UNIT_OPI,                     // 12
#else
  NAL_UNIT_RESERVED_IRAP_VCL_12,
#endif
  NAL_UNIT_DCI,                     // 13
  NAL_UNIT_VPS,                     // 14
  NAL_UNIT_SPS,                     // 15
  NAL_UNIT_PPS,                     // 16
  NAL_UNIT_PREFIX_APS,              // 17
  NAL_UNIT_SUFFIX_APS,              // 18
  NAL_UNIT_PH,                      // 19
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  NAL_UNIT_EOS,                     // 21
  NAL_UNIT_EOB,                     // 22
  NAL_UNIT_PREFIX_SEI,              // 23
  NAL_UNIT_SUFFIX_SEI,              // 24
  NAL_UNIT_FD,                      // 25

  NAL_UNIT_RESERVED_NVCL_26,
  NAL_UNIT_RESERVED_NVCL_27,

  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
};

#if SHARP_LUMA_DELTA_QP
enum LumaLevelToDQPMode
{
  LUMALVL_TO_DQP_DISABLED   = 0,
  LUMALVL_TO_DQP_AVG_METHOD = 1, // use average of CTU to determine luma level
  LUMALVL_TO_DQP_NUM_MODES  = 2
};
#endif

enum MergeType
{
  MRG_TYPE_DEFAULT_N        = 0, // 0
  MRG_TYPE_SUBPU_ATMVP,
  MRG_TYPE_IBC,
  NUM_MRG_TYPE                   // 5
};


//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////

enum EncModeFeature
{
  ENC_FT_FRAC_BITS = 0,
  ENC_FT_DISTORTION,
  ENC_FT_RD_COST,
  ENC_FT_ENC_MODE_TYPE,
  ENC_FT_ENC_MODE_OPTS,
  ENC_FT_ENC_MODE_PART,
  NUM_ENC_FEATURES
};

enum ImvMode
{
  IMV_OFF = 0,
  IMV_FPEL,
  IMV_4PEL,
  IMV_HPEL,
  NUM_IMV_MODES
};


// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for adaptive loop filter
class PicSym;

#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc; // NEW, MERGE, OFF
  int typeIdc;     // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  int typeAuxInfo; // BO: starting band index
  int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  void reset();
  const SAOBlkParam& operator= (const SAOBlkParam& src);
  SAOOffset& operator[](int compIdx){ return offsetParam[compIdx];}
  const SAOOffset& operator[](int compIdx) const { return offsetParam[compIdx];}
private:
  SAOOffset offsetParam[MAX_NUM_COMPONENT];

};



struct BitDepths
{
  int recon[MAX_NUM_CHANNEL_TYPE]; ///< the bit depth as indicated in the SPS
};

enum PLTRunMode
{
  PLT_RUN_INDEX = 0,
  PLT_RUN_COPY  = 1,
  NUM_PLT_RUN   = 2
};
/// parameters for deblocking filter
struct LFCUParam
{
  bool internalEdge;                     ///< indicates internal edge
  bool leftEdge;                         ///< indicates left edge
  bool topEdge;                          ///< indicates top edge
};



struct PictureHash
{
  std::vector<uint8_t> hash;

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }
};

struct SEITimeSet
{
  SEITimeSet() : clockTimeStampFlag(false),
                     numUnitFieldBasedFlag(false),
                     countingType(0),
                     fullTimeStampFlag(false),
                     discontinuityFlag(false),
                     cntDroppedFlag(false),
                     numberOfFrames(0),
                     secondsValue(0),
                     minutesValue(0),
                     hoursValue(0),
                     secondsFlag(false),
                     minutesFlag(false),
                     hoursFlag(false),
                     timeOffsetLength(0),
                     timeOffsetValue(0)
  { }
  bool clockTimeStampFlag;
  bool numUnitFieldBasedFlag;
  int  countingType;
  bool fullTimeStampFlag;
  bool discontinuityFlag;
  bool cntDroppedFlag;
  int  numberOfFrames;
  int  secondsValue;
  int  minutesValue;
  int  hoursValue;
  bool secondsFlag;
  bool minutesFlag;
  bool hoursFlag;
  int  timeOffsetLength;
  int  timeOffsetValue;
};

struct SEIMasteringDisplay
{
  bool      colourVolumeSEIEnabled;
  uint32_t      maxLuminance;
  uint32_t      minLuminance;
  uint16_t    primaries[3][2];
  uint16_t    whitePoint[2];
};

#if SHARP_LUMA_DELTA_QP
struct LumaLevelToDeltaQPMapping
{
  LumaLevelToDQPMode                 mode;             ///< use deltaQP determined by block luma level
  double                             maxMethodWeight;  ///< weight of max luma value when mode = 2
  std::vector< std::pair<int, int> > mapping;          ///< first=luma level, second=delta QP.
#if ENABLE_QPA
  bool isEnabled() const { return (mode != LUMALVL_TO_DQP_DISABLED && mode != LUMALVL_TO_DQP_NUM_MODES); }
#else
  bool isEnabled() const { return mode!=LUMALVL_TO_DQP_DISABLED; }
#endif
};
#endif

#if ER_CHROMA_QP_WCG_PPS
struct WCGChromaQPControl
{
  bool isEnabled() const { return enabled; }
  bool   enabled;         ///< Enabled flag (0:default)
  double chromaCbQpScale; ///< Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale; ///< Chroma Cr QP Scale (1.0:default)
  double chromaQpScale;   ///< Chroma QP Scale (0.0:default)
  double chromaQpOffset;  ///< Chroma QP Offset (0.0:default)
};
#endif

class ChromaCbfs
{
public:
  ChromaCbfs()
    : Cb(true), Cr(true)
  {}
  ChromaCbfs( bool _cbf )
    : Cb( _cbf ), Cr( _cbf )
  {}
public:
  bool sigChroma( ChromaFormat chromaFormat ) const
  {
    if( chromaFormat == CHROMA_400 )
    {
      return false;
    }
    return   ( Cb || Cr );
  }
  bool& cbf( ComponentID compID )
  {
    bool *cbfs[MAX_NUM_TBLOCKS] = { nullptr, &Cb, &Cr };

    return *cbfs[compID];
  }
public:
  bool Cb;
  bool Cr;
};


enum MsgLevel
{
  SILENT_ = 0,
  ERROR_  = 1,
  WARNING_ = 2,
  INFO_    = 3,
  NOTICE_  = 4,
  VERBOSE_ = 5,
  DETAILS_ = 6
};
enum RESHAPE_SIGNAL_TYPE
{
  RESHAPE_SIGNAL_SDR = 0,
  RESHAPE_SIGNAL_PQ  = 1,
  RESHAPE_SIGNAL_HLG = 2,
  RESHAPE_SIGNAL_NULL = 100,
};


// ---------------------------------------------------------------------------
// exception class
// ---------------------------------------------------------------------------

class Exception : public std::exception
{
public:
  Exception( const std::string& _s ) : m_str( _s ) { }
  Exception( const Exception& _e ) : std::exception( _e ), m_str( _e.m_str ) { }
  virtual ~Exception() noexcept { };
  virtual const char* what() const noexcept { return m_str.c_str(); }
  Exception& operator=( const Exception& _e ) { std::exception::operator=( _e ); m_str = _e.m_str; return *this; }
  template<typename T> Exception& operator<<( T t ) { std::ostringstream oss; oss << t; m_str += oss.str(); return *this; }
private:
  std::string m_str;
};

// if a check fails with THROW or CHECK_, please check if ported correctly from assert in revision 1196)
#define THROW(x)            throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define CHECK_(c,x)          if(c){ THROW(x); }
#define EXIT(x)             throw( Exception( "\n" ) << x << "\n" )
#define CHECK_NULLPTR(_ptr) CHECK_( !( _ptr ), "Accessing an empty pointer pointer!" )

#if !NDEBUG  // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in debug
#ifndef _DEBUG
#define _DEBUG 1
#endif
#endif

#if defined( _DEBUG )
#define CHECKD(c,x)         if(c){ THROW(x); }
#else
#define CHECKD(c,x)
#endif // _DEBUG

// ---------------------------------------------------------------------------
// static vector
// ---------------------------------------------------------------------------

template<typename T, size_t N>
class static_vector
{
  T _arr[ N ];
  size_t _size;

public:

  typedef T         value_type;
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T&        reference;
  typedef T const&  const_reference;
  typedef T*        pointer;
  typedef T const*  const_pointer;
  typedef T*        iterator;
  typedef T const*  const_iterator;

  static const size_type max_num_elements = N;

  static_vector() : _size( 0 )                                 { }
  static_vector( size_t N_ ) : _size( N_ )                     { }
  static_vector( size_t N_, const T& _val ) : _size( 0 )       { resize( N_, _val ); }
  template<typename It>
  static_vector( It _it1, It _it2 ) : _size( 0 )               { while( _it1 < _it2 ) _arr[ _size++ ] = *_it1++; }
  static_vector( std::initializer_list<T> _il ) : _size( 0 )
  {
    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }
  static_vector& operator=( std::initializer_list<T> _il )
  {
    _size = 0;

    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }

  void resize( size_t N_ )                      { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = T() ; _size = N_; }
  void resize( size_t N_, const T& _val )       { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = _val; _size = N_; }
  void reserve( size_t N_ )                     { CHECKD( N_ > N, "capacity exceeded" ); }
  void push_back( const T& _val )               { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = _val; }
  void push_back( T&& val )                     { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = std::forward<T>( val ); }
  void pop_back()                               { CHECKD( _size == 0, "calling pop_back on an empty vector" ); _size--; }
  void pop_front()                              { CHECKD( _size == 0, "calling pop_front on an empty vector" ); _size--; for( int i = 0; i < _size; i++ ) _arr[i] = _arr[i + 1]; }
  void clear()                                  { _size = 0; }
  reference       at( size_t _i )               { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference at( size_t _i ) const         { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       operator[]( size_t _i )       { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference operator[]( size_t _i ) const { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       front()                       { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  const_reference front() const                 { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  reference       back()                        { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  const_reference back() const                  { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  pointer         data()                        { return _arr; }
  const_pointer   data() const                  { return _arr; }
  iterator        begin()                       { return _arr; }
  const_iterator  begin() const                 { return _arr; }
  const_iterator  cbegin() const                { return _arr; }
  iterator        end()                         { return _arr + _size; }
  const_iterator  end() const                   { return _arr + _size; };
  const_iterator  cend() const                  { return _arr + _size; };
  size_type       size() const                  { return _size; };
  size_type       byte_size() const             { return _size * sizeof( T ); }
  bool            empty() const                 { return _size == 0; }

  size_type       capacity() const              { return N; }
  size_type       max_size() const              { return N; }
  size_type       byte_capacity() const         { return sizeof(_arr); }

  iterator        insert( const_iterator _pos, const T& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = _val;
                                                  _size++;
                                                  return const_cast<iterator>( _pos ); }

  iterator        insert( const_iterator _pos, T&& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = std::forward<T>( _val );
                                                  _size++; return const_cast<iterator>( _pos ); }
  template<class InputIt>
  iterator        insert( const_iterator _pos, InputIt first, InputIt last )
                                                { const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = const_cast<iterator>( _pos ); _size += numEl;
                                                  while( first != last ) *it++ = *first++;
                                                  return const_cast<iterator>( _pos ); }

  iterator        insert( const_iterator _pos, size_t numEl, const T& val )
                                                { //const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = const_cast<iterator>( _pos ); _size += numEl;
                                                  for ( int k = 0; k < numEl; k++) *it++ = val;
                                                  return const_cast<iterator>( _pos ); }

  void            erase( const_iterator _pos )  { iterator it   = const_cast<iterator>( _pos ) - 1;
                                                  iterator last = end() - 1;
                                                  while( ++it != last ) *it = *( it + 1 );
                                                  _size--; }
};


// ---------------------------------------------------------------------------
// dynamic cache
// ---------------------------------------------------------------------------

template<typename T>
class dynamic_cache
{
  std::vector<T*> m_cache;
#if ENABLE_SPLIT_PARALLELISM
  int64_t         m_cacheId;
#endif

public:

#if ENABLE_SPLIT_PARALLELISM
  dynamic_cache()
  {
    static int cacheId = 0;
    m_cacheId = cacheId++;
  }

#endif
  ~dynamic_cache()
  {
    deleteEntries();
  }

  void deleteEntries()
  {
    for( auto &p : m_cache )
    {
      delete p;
      p = nullptr;
    }

    m_cache.clear();
  }

  T* get()
  {
    T* ret;

    if( !m_cache.empty() )
    {
      ret = m_cache.back();
      m_cache.pop_back();
#if ENABLE_SPLIT_PARALLELISM
      CHECK_( ret->cacheId != m_cacheId, "Putting item into wrong cache!" );
      CHECK_( !ret->cacheUsed,           "Fetched an element that should've been in cache!!" );
#endif
    }
    else
    {
      ret = new T;
    }

#if ENABLE_SPLIT_PARALLELISM
    ret->cacheId   = m_cacheId;
    ret->cacheUsed = false;

#endif
    return ret;
  }

  void cache( T* el )
  {
#if ENABLE_SPLIT_PARALLELISM
    CHECK_( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
    CHECK_( el->cacheUsed,            "Putting cached item back into cache!" );

    el->cacheUsed = true;

#endif
    m_cache.push_back( el );
  }

  void cache( std::vector<T*>& vel )
  {
#if ENABLE_SPLIT_PARALLELISM
    for( auto el : vel )
    {
      CHECK_( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
      CHECK_( el->cacheUsed,            "Putting cached item back into cache!" );

      el->cacheUsed = true;
    }

#endif
    m_cache.insert( m_cache.end(), vel.begin(), vel.end() );
    vel.clear();
  }
};

typedef dynamic_cache<struct CodingUnit    > CUCache;
typedef dynamic_cache<struct PredictionUnit> PUCache;
typedef dynamic_cache<struct TransformUnit > TUCache;

struct XUCache
{
  CUCache cuCache;
  PUCache puCache;
  TUCache tuCache;
};

#define SIGN(x) ( (x) >= 0 ? 1 : -1 )

//! \}

#endif



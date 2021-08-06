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

/** \file     ProfileLevelTier.h
    \brief    Handle profile, level and tier information.
*/

#ifndef __PROFILE_LEVEL_TIER__
#define __PROFILE_LEVEL_TIER__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CommonLib/CommonDef.h"
#include <stdint.h>

class SPS; // Forward declaration.

struct LevelTierFeatures
{
  Level::Name level;
  uint32_t    maxLumaPs;
  uint32_t    maxCpb[Level::NUMBER_OF_TIERS];    // in units of CpbVclFactor or CpbNalFactor bits
  uint32_t    maxSlicesPerAu;
  uint32_t    maxTilesPerAu;
  uint32_t    maxTileCols;
  uint64_t    maxLumaSr;
  uint32_t    maxBr[Level::NUMBER_OF_TIERS];     // in units of BrVclFactor or BrNalFactor bits/s
  uint32_t    minCrBase[Level::NUMBER_OF_TIERS];
  uint32_t    getMaxPicWidthInLumaSamples()  const;
  uint32_t    getMaxPicHeightInLumaSamples() const;
};


struct ProfileFeatures
{
  Profile::Name            profile;
  const char              *pNameString;
  uint32_t                 maxBitDepth;
  ChromaFormat             maxChromaFormat;

  bool                     canUseLevel15p5;
  uint32_t                 cpbVclFactor;
  uint32_t                 cpbNalFactor;
  uint32_t                 formatCapabilityFactorx1000;
  uint32_t                 minCrScaleFactorx100;
  const LevelTierFeatures *pLevelTiersListInfo;
  bool                     onePictureOnlyFlagMustBe1;

  static const ProfileFeatures *getProfileFeatures(const Profile::Name p);
};


class ProfileLevelTierFeatures
{
  private:
    const ProfileFeatures   *m_pProfile;
    const LevelTierFeatures *m_pLevelTier;
    Level::Tier              m_tier;
  public:
    ProfileLevelTierFeatures() : m_pProfile(nullptr), m_pLevelTier(nullptr), m_tier(Level::MAIN) {}

    void extractPTLInformation(const SPS &sps);

    const ProfileFeatures     *getProfileFeatures()   const { return m_pProfile; }
    const LevelTierFeatures   *getLevelTierFeatures() const { return m_pLevelTier; }
    Level::Tier                getTier()              const { return m_tier; }
    uint64_t getCpbSizeInBits()                       const;
    double getMinCr()                                 const;
    uint32_t getMaxDpbSize( uint32_t picSizeMaxInSamplesY ) const;
};


#endif


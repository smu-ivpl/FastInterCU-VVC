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

 /** \file     StreamMergeMain.cpp
     \brief    Stream merge application main
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "StreamMergeApp.h"
#include "program_options_lite.h"

 //! \ingroup DecoderApp
 //! \{

 // ====================================================================================================================
 // Main function
 // ====================================================================================================================

int main(int argc, char* argv[])
{
  int returnCode = EXIT_SUCCESS;

  if (argc < 4)
  {
    printf("usage: %s <bitstream1> <bitstream2> [<bitstream3> ...] <outfile>\n", argv[0]);
    return -1;
  }

  // print information
  fprintf(stdout, "\n");
  fprintf(stdout, "VVCSoftware: VTM Version %s ", VTM_VERSION);
  fprintf(stdout, "\n");

  StreamMergeApp *pStrMergeApp = new StreamMergeApp;
  // parse configuration
  if (!pStrMergeApp->parseCfg(argc, argv))
  {
    returnCode = EXIT_FAILURE;
    return returnCode;
  }

  // starting time
  double dResult;
  clock_t lBefore = clock();

  // call decoding function
  if (0 != pStrMergeApp->mergeStreams())
  {
    printf("\n\n***ERROR_*** A merge error happened\n");
    returnCode = EXIT_FAILURE;
  }

  // ending time
  dResult = (double)(clock() - lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", dResult);

  delete pStrMergeApp;

  return returnCode;
}

//! \}

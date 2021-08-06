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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "CommonLib/CommonDef.h"
#include "BitstreamExtractorApp.h"
#include "program_options_lite.h"

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
  int returnCode = EXIT_SUCCESS;

  // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "VVCSoftware: VTM Bitstream Extractor Version %s ", VTM_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
#if ENABLE_SIMD_OPT
  std::string SIMD;
  df::program_options_lite::Options optsSimd;
  optsSimd.addOptions()( "SIMD", SIMD, std::string( "" ), "" );
  df::program_options_lite::SilentReporter err;
  df::program_options_lite::scanArgv( optsSimd, argc, ( const char** ) argv, err );
  fprintf( stdout, "[SIMD=%s] ", read_x86_extension( SIMD ) );
#endif
#if ENABLE_TRACING
  fprintf( stdout, "[ENABLE_TRACING] " );
#endif
  fprintf( stdout, "\n" );

  BitstreamExtractorApp *extractorApp = new BitstreamExtractorApp;
  // parse configuration
  if(!extractorApp->parseCfg( argc, argv ))
  {
    returnCode = EXIT_FAILURE;
    return returnCode;
  }

  // starting time
  double timeDifference;
  clock_t startTime = clock();

  // call decoding function
#ifndef _DEBUG
  try
  {
#endif // !_DEBUG
    if( 0 != extractorApp->decode() )
    {
      printf( "\n\n***ERROR_*** A decoding mismatch occured: signalled md5sum does not match\n" );
      returnCode = EXIT_FAILURE;
    }
#ifndef _DEBUG
  }
  catch( Exception &e )
  {
    std::cerr << e.what() << std::endl;
    returnCode = EXIT_FAILURE;
  }
  catch( ... )
  {
    std::cerr << "Unspecified error occurred" << std::endl;
    returnCode = EXIT_FAILURE;
  }
#endif

  // ending time
  timeDifference = (double)(clock()-startTime) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", timeDifference);

  delete extractorApp;

  return returnCode;
}


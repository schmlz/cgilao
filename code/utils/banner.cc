#include <iostream>

#include "banner.h"

#ifndef CFLAGS_USED
#define CFLAGS_USED "not defined"
#endif

#ifndef __VERSION__
#define __VERSION__ "UNKNOWN"
#endif

#ifndef GIT_HASH
#define GIT_HASH "UNKNOWN"
#endif

#ifndef HOSTNAME
#define HOSTNAME "UNKNOWN"
#endif



void printBanner(std::ostream& os) {
  os << "This is Replanner-mGPT. Ver 1.0" << std::endl
     << "(Developed by F.W. Trevizan (fwt@cs.cmu.edu) based on mGPT (B. Bonet and H. Geffner)\n"
     << "GIT HASH: " << GIT_HASH << " @ " << HOSTNAME << "\n"
     << "COMPILED ON: " << __DATE__ << " at " << __TIME__ << "\n"
     << "COMPILER: " << __VERSION__ << "\n"
     << "USED CFLAGS: " << CFLAGS_USED << "\n"
     << std::endl << std::endl;
}

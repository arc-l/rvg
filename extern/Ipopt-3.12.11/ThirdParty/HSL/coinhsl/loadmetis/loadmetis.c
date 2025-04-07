/*
 * This file will load symbols dynamically from shared libraries
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stddef.h>
#include <stdio.h>

#ifdef HAVE_WINDOWS_H
#include <windows.h>
#define LOADLIB LoadLibrary
#define LOADSYM GetProcAddress
#define SHLIBEXT "dll"
#else
#include <dlfcn.h>
#define LOADLIB(X) dlopen(X, RTLD_NOW)
#define LOADSYM dlsym
#define SHLIBEXT "so"
#endif

#define METISLIB "libmetis." SHLIBEXT
#define BLASLIB "libblas." SHLIBEXT

typedef void (*METIS_NodeND_t) (int *nvtxs, int *xadj, int *adjncy,
      int *numflag, int *options, int *perm, int *iperm);

void *METIS_handle=NULL;
static METIS_NodeND_t func_METIS_NodeND=NULL;

void metis_nodend_(int *nvtxs, int *xadj, int *adjncy, int *numflag,
      int *options, int *perm, int *iperm) {
   if(!func_METIS_NodeND) {
      if(!METIS_handle) {
         METIS_handle = LOADLIB(METISLIB);
         if(!METIS_handle) {
            printf("Failed to load '%s' - using fallback AMD code\n", METISLIB);
            //printf("%s\n", dlerror());
            perm[0]=-1;
            return;
         }
      }
      func_METIS_NodeND = LOADSYM(METIS_handle, "METIS_NodeND");
      if(!func_METIS_NodeND) {
         printf("Failed to retrieve symbol from '%s' - using fallback AMD code\n", METISLIB);
         perm[0]=-1;
         return;
      }
   }
   func_METIS_NodeND(nvtxs, xadj, adjncy, numflag, options, perm, iperm);
}

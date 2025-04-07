AUTOMAKE_OPTIONS = foreign

ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libcoinhsl.la
include_HEADERS = CoinHslConfig.h

pkgconfigdir 	= $(libdir)/pkgconfig
pkgconfig_DATA = coinhsl.pc

libcoinhsl_la_LDFLAGS = -export-symbols $(srcdir)/libcoinhsl.sym
libcoinhsl_la_SOURCES = common/deps90.f90 \
								common/deps.f \
								common/dump.f90

EXTRA_DIST = LICENCE libcoinhsl.sym

if HAVE_MC19
libcoinhsl_la_SOURCES += mc19/mc19d.f
endif

if HAVE_MA27
libcoinhsl_la_SOURCES += ma27/ma27d.f
endif

if HAVE_MA28
libcoinhsl_la_SOURCES += ma28/ma28d.f
endif

if HAVE_MA57
libcoinhsl_la_SOURCES += ma57/ma57d.f
endif

if HAVE_MA77
include_HEADERS 		 += hsl_ma77/C/hsl_ma77d.h
libcoinhsl_la_SOURCES += hsl_ma77/hsl_ma77d.f90 \
								 hsl_ma77/C/hsl_ma77d_ciface.f90
endif

if HAVE_MA86
include_HEADERS       += hsl_ma86/C/hsl_ma86d.h
libcoinhsl_la_SOURCES += hsl_ma86/hsl_ma86d.f90 \
								 hsl_ma86/C/hsl_ma86d_ciface.f90
endif

if HAVE_MC68
include_HEADERS 		 += hsl_mc68/C/hsl_mc68i.h
#libcoinhsl_la_SOURCES += hsl_mc68/hsl_mc68i.f90 \
#								 hsl_mc68/C/hsl_mc68i_ciface.f90
libcoinhsl_la_SOURCES += hsl_mc68/C/hsl_mc68i_ciface.f90
endif

if HAVE_MA97
include_HEADERS 		 += hsl_ma97/C/hsl_ma97d.h
libcoinhsl_la_SOURCES += hsl_ma97/hsl_ma97d.f90 \
								 hsl_ma97/C/hsl_ma97d_ciface.f90
endif

if TARGET_DLL
libcoinhsl_la_LDFLAGS += -no-undefined
libcoinhsl_la_LIBADD = ${BLAS_LIBS}
endif
libcoinhsl_la_SOURCES += loadmetis/loadmetis.c

if HAVE_METIS
lib_LTLIBRARIES += libmetis.la
libmetis_la_CFLAGS = -I $(srcdir)/metis-4.0.3/Lib $(AM_CFLAGS)
if TARGET_DLL
libmetis_la_CFLAGS += -D__VC__
libmetis_la_LDFLAGS = -no-undefined
endif # TARGET_DLL
nodist_libmetis_la_SOURCES = metis-4.0.3/Lib/balance.c metis-4.0.3/Lib/mfm.c \
							 metis-4.0.3/Lib/bucketsort.c metis-4.0.3/Lib/mincover.c \
							 metis-4.0.3/Lib/ccgraph.c metis-4.0.3/Lib/minitpart2.c \
							 metis-4.0.3/Lib/coarsen.c metis-4.0.3/Lib/minitpart.c \
							 metis-4.0.3/Lib/compress.c metis-4.0.3/Lib/mkmetis.c \
							 metis-4.0.3/Lib/debug.c metis-4.0.3/Lib/mkwayfmh.c \
							 metis-4.0.3/Lib/defs.h metis-4.0.3/Lib/mkwayrefine.c \
							 metis-4.0.3/Lib/estmem.c metis-4.0.3/Lib/mmatch.c \
							 metis-4.0.3/Lib/fm.c metis-4.0.3/Lib/mmd.c \
							 metis-4.0.3/Lib/fortran.c metis-4.0.3/Lib/mpmetis.c \
							 metis-4.0.3/Lib/frename.c metis-4.0.3/Lib/mrefine2.c \
							 metis-4.0.3/Lib/graph.c metis-4.0.3/Lib/mrefine.c \
							 metis-4.0.3/Lib/initpart.c metis-4.0.3/Lib/mutil.c \
							 metis-4.0.3/Lib/kmetis.c metis-4.0.3/Lib/myqsort.c \
							 metis-4.0.3/Lib/kvmetis.c metis-4.0.3/Lib/ometis.c \
							 metis-4.0.3/Lib/kwayfm.c metis-4.0.3/Lib/parmetis.c \
							 metis-4.0.3/Lib/kwayrefine.c metis-4.0.3/Lib/pmetis.c \
							 metis-4.0.3/Lib/kwayvolfm.c metis-4.0.3/Lib/pqueue.c \
							 metis-4.0.3/Lib/kwayvolrefine.c metis-4.0.3/Lib/proto.h \
							 metis-4.0.3/Lib/macros.h metis-4.0.3/Lib/refine.c \
							 metis-4.0.3/Lib/match.c metis-4.0.3/Lib/rename.h \
							 metis-4.0.3/Lib/mbalance2.c metis-4.0.3/Lib/separator.c \
							 metis-4.0.3/Lib/mbalance.c metis-4.0.3/Lib/sfm.c \
							 metis-4.0.3/Lib/mcoarsen.c metis-4.0.3/Lib/srefine.c \
							 metis-4.0.3/Lib/memory.c metis-4.0.3/Lib/stat.c \
							 metis-4.0.3/Lib/mesh.c metis-4.0.3/Lib/struct.h \
							 metis-4.0.3/Lib/meshpart.c metis-4.0.3/Lib/subdomains.c \
							 metis-4.0.3/Lib/metis.h metis-4.0.3/Lib/timing.c \
							 metis-4.0.3/Lib/mfm2.c metis-4.0.3/Lib/util.c
endif # HAVE_METIS

# Automake doesn't know much about modules, so get rid of them ourself
clean-local:
	-rm *.mod

hsl_ma86d_ciface.$(OBJEXT): hsl_ma86d.$(OBJEXT)
hsl_ma97d_ciface.$(OBJEXT): hsl_ma97d.$(OBJEXT)
hsl_mc68i_ciface.$(OBJEXT): hsl_mc68i.$(OBJEXT)

CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

USE_PACKAGE:=1

EXCLUDE_COLD_LIBRARIES:=

IS_LIBRARY:=0
LIBNAME:=libbest
VERSION:=1.0.0
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

-include ./common.mk

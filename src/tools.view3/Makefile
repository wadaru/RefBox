#*****************************************************************************
#             Makefile Build System for Fawkes : Tools
#                            -------------------
#   Created on Tue Feb 12 23:22:15 2013
#   Copyright (C) 2006-2008 by Tim Niemueller, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/clips.mk
include $(BUILDSYSDIR)/boost.mk
# include $(BUILDCONFDIR)/llsf_sps/llsf_sps.mk

CFLAGS += $(CFLAGS_CPP11)

REQ_BOOST_LIBS = thread asio system signals2
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

LIBS_rcll_view3 = stdc++ llsfrbcore llsfrbutils llsfrbconfig llsf_protobuf_comm llsf_msgs
OBJS_rcll_view3 = rcll-view3.o

ifeq ($(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),11)
  OBJS_all += $(OBJS_rcll_view3)
  BINS_all += $(BINDIR)/rcll-view3

  CFLAGS_rcll_view3       += $(CFLAGS_PROTOBUF) \
                             $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS_rcll_view3      += $(LDFLAGS_PROTOBUF) \
                             $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) -lm

  #MANPAGES_all =  $(MANDIR)/man1/refbox-llsf.1
else
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)
.PHONY: warning_libmodbus
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_comm library$(TNORMAL) (Boost library $* not found)"
warning_libmodbus:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build LLSF Puck Tool$(TNORMAL) (libmodbus not found)"
endif

include $(BUILDSYSDIR)/base.mk

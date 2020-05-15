lib_LIBRARIES	 = libloragw.a

libloragw_a_SOURCES  = libloragw/src/loragw_hal.c
libloragw_a_SOURCES += libloragw/src/loragw_debug.c
libloragw_a_SOURCES += libloragw/src/loragw_reg.c
libloragw_a_SOURCES += libloragw/src/loragw_aux.c
libloragw_a_SOURCES += libloragw/src/loragw_gps.c
libloragw_a_SOURCES += libloragw/src/loragw_cal.c
libloragw_a_SOURCES += libloragw/src/loragw_sx125x.c
libloragw_a_SOURCES += libloragw/src/loragw_sx1250.c
libloragw_a_SOURCES += libloragw/src/loragw_sx1302.c
libloragw_a_SOURCES += libloragw/src/loragw_sx1302_timestamp.c
libloragw_a_SOURCES += libloragw/src/loragw_sx1302_rx.c
libloragw_a_SOURCES += libloragw/src/loragw_spi.c
libloragw_a_SOURCES += libloragw/src/loragw_i2c.c
libloragw_a_SOURCES += libloragw/src/loragw_stts751.c
libloragw_a_SOURCES += libloragw/src/agc_fw_sx1250.var
libloragw_a_SOURCES += libloragw/src/agc_fw_sx1257.var
libloragw_a_SOURCES += libloragw/src/arb_fw.var
libloragw_a_SOURCES += libloragw/src/cal_fw.var

libloragw_a_CPPFLAGS  = $(AM_CPPFLAGS)
libloragw_a_CPPFLAGS += -I$(top_srcdir)/libloragw/inc

nodist_include_HEADERS = libloragw/inc/config.h

include_HEADERS	 = libloragw/inc/loragw_agc_params.h
include_HEADERS	+= libloragw/inc/loragw_aux.h
include_HEADERS	+= libloragw/inc/loragw_cal.h
include_HEADERS	+= libloragw/inc/loragw_debug.h
include_HEADERS	+= libloragw/inc/loragw_gps.h
include_HEADERS	+= libloragw/inc/loragw_hal.h
include_HEADERS	+= libloragw/inc/loragw_i2c.h
include_HEADERS	+= libloragw/inc/loragw_reg.h
include_HEADERS	+= libloragw/inc/loragw_spi.h
include_HEADERS	+= libloragw/inc/loragw_stts751.h
include_HEADERS	+= libloragw/inc/loragw_sx1250.h
include_HEADERS	+= libloragw/inc/loragw_sx125x.h
include_HEADERS	+= libloragw/inc/loragw_sx1302.h
include_HEADERS	+= libloragw/inc/loragw_sx1302_rx.h
include_HEADERS	+= libloragw/inc/loragw_sx1302_timestamp.h

pkgconfigdir	 = $(libdir)/pkgconfig
pkgconfig_DATA	 = libloragw.pc

# External copied librairies
noinst_LIBRARIES = libbase64.a libparson.a libtinymt32.a
libbase64_a_SOURCES = libtools/src/base64.c libtools/inc/base64.h
libbase64_a_CPPFLAGS = $(AM_CPPFLAGS) -I$(top_srcdir)/libtools/inc

libparson_a_SOURCES = libtools/src/parson.c libtools/inc/parson.h
libparson_a_CPPFLAGS = $(AM_CPPFLAGS) -I$(top_srcdir)/libtools/inc

libtinymt32_a_SOURCES = libtools/src/tinymt32.c libtools/inc/tinymt32.h
libtinymt32_a_CPPFLAGS = $(AM_CPPFLAGS) -I$(top_srcdir)/libtools/inc

###########################################
# PROGRAMS
###########################################

# include tool libraries for all programs
LDADD = libloragw.a libtinymt32.a libparson.a libbase64.a
AM_CPPFLAGS += -I$(top_srcdir)/libtools/inc

#
# Test Tools
#
noinst_PROGRAMS	 = utils/loragw_spi utils/loragw_reg utils/loragw_hal_tx utils/loragw_hal_rx utils/loragw_cal
utils_loragw_spi_SOURCES =    libloragw/tst/test_loragw_spi.c
utils_loragw_reg_SOURCES =    libloragw/tst/test_loragw_reg.c
utils_loragw_hal_tx_SOURCES = libloragw/tst/test_loragw_hal_tx.c
utils_loragw_hal_rx_SOURCES = libloragw/tst/test_loragw_hal_rx.c
utils_loragw_cal_SOURCES =    libloragw/tst/test_loragw_cal.c

#
# Packet forwarder
#
bin_PROGRAMS = lora_pkt_fwd
lora_pkt_fwd_SOURCES  = packet_forwarder/src/lora_pkt_fwd.c
lora_pkt_fwd_SOURCES += packet_forwarder/src/jitqueue.c
lora_pkt_fwd_CPPFLAGS = $(AM_CPPFLAGS) -I$(top_srcdir)/packet_forwarder/inc
lora_pkt_fwd_LDADD = libloragw.a libtinymt32.a libparson.a libbase64.a

#
# Chip ID
#
bin_PROGRAMS += chip_id
chip_id_SOURCES  = util_chip_id/src/chip_id.c
chip_id_CPPFLAGS = $(AM_CPPFLAGS) -I$(top_srcdir)/util_chip_id/inc
chip_id_LDADD = libloragw.a libtinymt32.a

#
# net downlink
#
bin_PROGRAMS += net_downlink
net_downlink_SOURCES  = util_net_downlink/src/net_downlink.c
net_downlink_CPPFLAGS = $(AM_CPPFLAGS) -I$(top_srcdir)/util_net_downlink/inc
net_downlink_LDADD = libloragw.a libtinymt32.a libparson.a libbase64.a

if HAVE_TAR
UTILSDIR	 = $(PACKAGE_TARNAME)-utils_$(PACKAGE_VERSION)
UTILS		 = $(UTILSDIR).tar.gz

utils_deps_bin	 = utils/loragw_reg
utils_deps_bin	+= utils/loragw_hal_tx
utils_deps_bin	+= utils/loragw_hal_rx
utils_deps_bin	+= utils/loragw_cal
utils_deps_bin	+= lora_pkt_fwd
utils_deps_bin	+= chip_id
utils_deps_bin	+= tools/reset_lgw.sh

utils_deps_data  = packet_forwarder/global_conf.*

$(UTILS): $(utils_deps_bin) $(utils_deps_data)
	$(AM_V_GEN) rm -rf $(UTILSDIR) $@ && \
	$(MKDIR_P) $(UTILSDIR) && \
	$(INSTALL_PROGRAM) $(utils_deps_bin) $(UTILSDIR) && \
	$(INSTALL_DATA) $(utils_deps_data) $(UTILSDIR) && \
	$(TAR) czf $@ --owner=0 --group=0 $(UTILSDIR)

utils-local: $(UTILS)
endif

$(generated): %: $(top_srcdir)/%.in Makefile
	$(AM_V_GEN) $(MKDIR_P) $(@D) && sed \
		-e 's|@PACKAGE_NAME[@]|$(PACKAGE_NAME)|g' \
		-e 's|@PACKAGE_TARNAME[@]|$(PACKAGE_TARNAME)|g' \
		-e 's|@PACKAGE_VERSION[@]|$(PACKAGE_VERSION)|g' \
		-e 's|@bindir[@]|$(bindir)|g' \
		-e 's|@datadir[@]|$(datadir)|g' \
		-e 's|@localstatedir[@]|$(localstatedir)|g' \
		-e 's|@sysconfdir[@]|$(sysconfdir)|g' \
		$< > $@

CLEANFILES	 = $(generated)
EXTRA_DIST	 = $(generated:%=%.in)

include $(top_srcdir)/target.cfg

upload: $(UTILS)
	@echo "---- Copying utils to $(TARGET_IP):$(TARGET_DIR)"
	@ssh $(TARGET_USR)@$(TARGET_IP) "mkdir -p $(TARGET_DIR)"
	@scp -r $(UTILSDIR)/* $(TARGET_USR)@$(TARGET_IP):$(TARGET_DIR)/

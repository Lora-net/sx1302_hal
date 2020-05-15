AC_PREREQ([2.69])
AC_INIT([SX1302 Lora HAL],
        [m4_esyscmd_s([git describe --always --tags --dirty])],
        [https://github.com/Lora-net/sx1302_hal/issues],
        [libloragw1302],
        [https://github.com/Lora-net/sx1302_hal])
AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_SRCDIR([libloragw/src/loragw_hal.c])
AM_INIT_AUTOMAKE([foreign subdir-objects -Wall -Werror])
AC_LANG([C])

# Checks for programs.
AC_PROG_CC
AM_PROG_AR
AC_PROG_RANLIB
AC_PROG_MKDIR_P
AC_PROG_INSTALL
AC_CHECK_PROG([TAR], [tar], [tar])
AM_CONDITIONAL([HAVE_TAR], [test x$TAR != xno])

# Configure programs.
AC_SUBST([AM_CPPFLAGS], ["-Wall -Wextra -std=c99"])
AC_SUBST([ARFLAGS], ["rc"])

# Define libloragw version.
AC_DEFINE_UNQUOTED([LIBLORAGW_VERSION], ["$PACKAGE_VERSION"],
		   [Libloragw version number])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h \
		  linux/spi/spidev.h \
		  math.h \
		  stdbool.h \
		  stdio.h \
		  sys/ioctl.h \
		  termios.h \
		  time.h], [], [AC_MSG_ERROR([header not found])])

# Checks for libraries.
AC_SEARCH_LIBS([clock_nanosleep], [rt], [], [AC_MSG_ERROR([library not found])])
AC_SEARCH_LIBS([pow], [m], [], [AC_MSG_ERROR([library not found])])
AC_CHECK_LIB([pthread], [pthread_create], [], [AC_MSG_ERROR([library not found])])

# Checks for optional features.
AC_ARG_ENABLE([debug],
	      [AS_HELP_STRING([--enable-debug[[=MOD[:MOD]]]],
			      [enable debug modules: \
			       all, aux, gps, hal, lbt, reg, spi])],
	      [for MOD in $(echo $enableval | $AWK '$1=$1' FS=: OFS=' '); do
	       case $MOD in
		       all|yes) enable_debug=all ;;
		       aux) enable_debug_aux=yes ;;
		       gps) enable_debug_gps=yes ;;
		       hal) enable_debug_hal=yes ;;
		       lbt) enable_debug_lbt=yes ;;
		       reg) enable_debug_reg=yes ;;
		       spi) enable_debug_spi=yes ;;
		       i2c) enable_debug_i2c=yes ;;
		       rad) enable_debug_rad=yes ;;
		       cal) enable_debug_cal=yes ;;
		       sx1302) enable_debug_sx1302=yes ;;
	       esac; done], [enable_debug=no])

AC_DEFINE([DEBUG_AUX], [0], [Auxiliary debug disabled])
AC_DEFINE([DEBUG_GPS], [0], [GPS debug disabled])
AC_DEFINE([DEBUG_HAL], [0], [HAL debug disabled])
AC_DEFINE([DEBUG_LBT], [0], [LBT debug disabled])
AC_DEFINE([DEBUG_REG], [0], [Register debug disabled])
AC_DEFINE([DEBUG_SPI], [0], [SPI debug disabled])
AC_DEFINE([DEBUG_I2C], [0], [I2C debug disabled])
AC_DEFINE([DEBUG_RAD], [0], [RAD debug disabled])
AC_DEFINE([DEBUG_CAL], [0], [CAL debug disabled])
AC_DEFINE([DEBUG_SX1302], [0], [SX1302 debug disabled])
AM_SILENT_RULES([no])

AS_IF([test x$enable_debug = xno],
      [AM_SILENT_RULES([yes])],
      [test x$enable_debug = xall],
      [AC_DEFINE([DEBUG_AUX], [1], [Auxiliary debug enabled])
       AC_DEFINE([DEBUG_GPS], [1], [GPS debug enabled])
       AC_DEFINE([DEBUG_HAL], [1], [HAL debug enabled])
       AC_DEFINE([DEBUG_LBT], [1], [LBT debug enabled])
       AC_DEFINE([DEBUG_REG], [1], [Register debug enabled])
       AC_DEFINE([DEBUG_SPI], [1], [SPI debug enabled])]
       AC_DEFINE([DEBUG_I2C], [1], [I2C debug enabled])
       AC_DEFINE([DEBUG_RAD], [1], [RAD debug enabled])
       AC_DEFINE([DEBUG_CAL], [1], [CAL debug enabled])
       AC_DEFINE([DEBUG_SX1302], [1], [SX1302 debug enabled]),
      [AS_IF([test x$enable_debug_aux = xyes],
	     [AC_DEFINE([DEBUG_AUX], [1], [Auxiliary debug enabled])])
       AS_IF([test x$enable_debug_gps = xyes],
	     [AC_DEFINE([DEBUG_GPS], [1], [GPS debug enabled])])
       AS_IF([test x$enable_debug_hal = xyes],
	     [AC_DEFINE([DEBUG_HAL], [1], [HAL debug enabled])])
       AS_IF([test x$enable_debug_lbt = xyes],
	     [AC_DEFINE([DEBUG_LBT], [1], [LBT debug enabled])])
       AS_IF([test x$enable_debug_reg = xyes],
	     [AC_DEFINE([DEBUG_REG], [1], [Register debug enabled])])
       AS_IF([test x$enable_debug_spi = xyes],
	     [AC_DEFINE([DEBUG_SPI], [1], [SPI debug enabled])])
       AS_IF([test x$enable_debug_i2c = xyes],
	     [AC_DEFINE([DEBUG_I2C], [1], [I2C debug enabled])])
       AS_IF([test x$enable_debug_rad = xyes],
	     [AC_DEFINE([DEBUG_RAD], [1], [RAD debug enabled])])
       AS_IF([test x$enable_debug_cal = xyes],
	     [AC_DEFINE([DEBUG_CAL], [1], [CAL debug enabled])])
       AS_IF([test x$enable_debug_sx1302 = xyes],
	     [AC_DEFINE([DEBUG_SX1302], [1], [SX1302 debug enabled])])])

AM_EXTRA_RECURSIVE_TARGETS([utils])
AC_CONFIG_HEADERS([libloragw/inc/config-full.h])
AC_CONFIG_HEADERS([libloragw/inc/config.h])
AC_CONFIG_FILES([libloragw.pc])
AC_CONFIG_FILES([Makefile])
AC_OUTPUT


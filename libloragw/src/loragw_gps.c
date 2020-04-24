/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Library of functions to manage a GNSS module (typically GPS) for accurate
    timestamping of packets and synchronisation of gateways.
    A limited set of module brands/models are supported.

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#define _GNU_SOURCE     /* needed for qsort_r to be defined */
#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcpy */

#include <time.h>       /* struct timespec */
#include <fcntl.h>      /* open */
#include <termios.h>    /* tcflush */
#include <math.h>       /* modf */

#include <assert.h>
#include <stdlib.h>

#include "loragw_gps.h"
#include "cursor/cursor.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_GPS == 1
    #define DEBUG_MSG(args...)  fprintf(stderr, args)
    #define DEBUG_ARRAY(a,b,c)  for(a=0;a<b;++a) fprintf(stderr,"%x.",c[a]);fprintf(stderr,"end\n")
    #define CHECK_NULL(a)       if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_GPS_ERROR;}
#else
    #define DEBUG_MSG(args...)
    #define DEBUG_ARRAY(a,b,c)  for(a=0;a!=0;){}
    #define CHECK_NULL(a)       if(a==NULL){return LGW_GPS_ERROR;}
#endif
#define TRACE()         fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

#define CSR_RES(eXpr_)                                                         \
    do {                                                                       \
        enum cursor_res vAl_ = (eXpr_);                                        \
        if (cursor_res_err_buf_exhausted == (vAl_)) {                          \
            return INCOMPLETE;                                                 \
        }                                                                      \
        assert(vAl_ == cursor_res_ok);                                         \
    } while (0)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define TS_CPS              1E6 /* count-per-second of the timestamp counter */
#define PLUS_10PPM          1.00001
#define MINUS_10PPM         0.99999
#define DEFAULT_BAUDRATE    B9600

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */


/* result of the NMEA parsing */
static short gps_yea = 0; /* year (2 or 4 digits) */
static short gps_mon = 0; /* month (1-12) */
static short gps_day = 0; /* day of the month (1-31) */
static short gps_hou = 0; /* hours (0-23) */
static short gps_min = 0; /* minutes (0-59) */
static short gps_sec = 0; /* seconds (0-60)(60 is for leap second) */
static float gps_fra = 0.0; /* fractions of seconds (<1) */
static bool gps_time_ok = false;
static int16_t gps_week = 0; /* GPS week number of the navigation epoch */
static uint32_t gps_iTOW = 0; /* GPS time of week in milliseconds */
static int32_t gps_fTOW = 0; /* Fractional part of iTOW (+/-500000) in nanosec */
static uint32_t gps_tAcc = 0;   /* Time accuracy estimate */

static short gps_dla = 0; /* degrees of latitude */
static double gps_mla = 0.0; /* minutes of latitude */
static char gps_ola = 0; /* orientation (N-S) of latitude */
static short gps_dlo = 0; /* degrees of longitude */
static double gps_mlo = 0.0; /* minutes of longitude */
static char gps_olo = 0; /* orientation (E-W) of longitude */
static int32_t gps_lat = 0; /* latitude */
static int32_t gps_lon = 0; /* longitude */
static int32_t gps_alt = 0; /* altitude */
static bool gps_pos_ok = false;
static uint32_t gps_hAcc = 0;   /* horizontal accuracy estimate (mm) */
static uint32_t gps_vAcc = 0;   /* vertical accuracy estimate (mm) */
static uint8_t  gps_numSV = 0;  /* number of satellites used in the position fix */

static char gps_mod = 'N'; /* GPS mode (N no fix, A autonomous, D differential) */
static short gps_sat = 0; /* number of satellites used for fix */

static struct termios ttyopt_restore;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static int nmea_checksum(const char *nmea_string, int buff_size, char *checksum);

static char nibble_to_hexchar(uint8_t a);

static bool validate_nmea_checksum(const char *serial_buff, int buff_size);

static bool match_label(const char *s, char *label, int size, char wildcard);

static int str_chop(char *s, int buff_size, char separator, int *idx_ary, int max_idx);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/*
Calculate the checksum for a NMEA string
Skip the first '$' if necessary and calculate checksum until '*' character is
reached (or buff_size exceeded).
Checksum must point to a 2-byte (or more) char array.
Return position of the checksum in the string
*/
static int nmea_checksum(const char *nmea_string, int buff_size, char *checksum) {
    int i = 0;
    uint8_t check_num = 0;

    /* check input parameters */
    if ((nmea_string == NULL) ||  (checksum == NULL) || (buff_size <= 1)) {
        DEBUG_MSG("Invalid parameters for nmea_checksum\n");
        return -1;
    }

    /* skip the first '$' if necessary */
    if (nmea_string[i] == '$') {
        i += 1;
    }

    /* xor until '*' or max length is reached */
    while (nmea_string[i] != '*') {
        check_num ^= nmea_string[i];
        i += 1;
        if (i >= buff_size) {
            DEBUG_MSG("Maximum length reached for nmea_checksum\n");
            return -1;
        }
    }

    /* Convert checksum value to 2 hexadecimal characters */
    checksum[0] = nibble_to_hexchar(check_num / 16); /* upper nibble */
    checksum[1] = nibble_to_hexchar(check_num % 16); /* lower nibble */

    return i + 1;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static char nibble_to_hexchar(uint8_t a) {
    if (a < 10) {
        return '0' + a;
    } else if (a < 16) {
        return 'A' + (a-10);
    } else {
        return '?';
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/*
Calculate the checksum of a NMEA frame and compare it to the checksum that is
present at the end of it.
Return true if it matches
*/
static bool validate_nmea_checksum(const char *serial_buff, int buff_size) {
    int checksum_index;
    char checksum[2]; /* 2 characters to calculate NMEA checksum */

    checksum_index = nmea_checksum(serial_buff, buff_size, checksum);

    /* could we calculate a verification checksum ? */
    if (checksum_index < 0) {
        DEBUG_MSG("ERROR: IMPOSSIBLE TO PARSE NMEA SENTENCE\n");
        return false;
    }

    /* check if there are enough char in the serial buffer to read checksum */
    if (checksum_index >= (buff_size - 2)) {
        DEBUG_MSG("ERROR: IMPOSSIBLE TO READ NMEA SENTENCE CHECKSUM\n");
        return false;
    }

    /* check the checksum per se */
    if ((serial_buff[checksum_index] == checksum[0]) && (serial_buff[checksum_index+1] == checksum[1])) {
        return true;
    } else {
        DEBUG_MSG("ERROR: NMEA CHECKSUM %c%c DOESN'T MATCH VERIFICATION CHECKSUM %c%c\n", serial_buff[checksum_index], serial_buff[checksum_index+1], checksum[0], checksum[1]);
        return false;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/*
Return true if the "label" string (can contain wildcard characters) matches
the begining of the "s" string
*/
static bool match_label(const char *s, char *label, int size, char wildcard) {
    int i;

    for (i=0; i < size; i++) {
        if (label[i] == wildcard) continue;
        if (label[i] != s[i]) return false;
    }
    return true;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/*
Chop a string into smaller strings
Replace every separator in the input character buffer by a null character so
that all s[index] are valid strings.
Populate an array of integer 'idx_ary' representing indexes of token in the
string.
buff_size and max_idx are there to prevent segfaults.
Return the number of token found (number of idx_ary filled).
*/
int str_chop(char *s, int buff_size, char separator, int *idx_ary, int max_idx) {
    int i = 0; /* index in the string */
    int j = 0; /* index in the result array */

    if ((s == NULL) || (buff_size < 0) || (separator == 0) || (idx_ary == NULL) || (max_idx < 0)) {
        /* unsafe to do anything */
        return -1;
    }
    if ((buff_size == 0) || (max_idx == 0)) {
        /* nothing to do */
        return 0;
    }
    s[buff_size - 1] = 0; /* add string terminator at the end of the buffer, just to be sure */
    idx_ary[j] = 0;
    j += 1;
    /* loop until string terminator is reached */
    while (s[i] != 0) {
        if (s[i] == separator) {
            s[i] = 0; /* replace separator by string terminator */
            if (j >= max_idx) { /* no more room in the index array */
                return j;
            }
            idx_ary[j] = i+1; /* next token start after replaced separator */
            ++j;
        }
        ++i;
    }
    return j;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_gps_enable(char *tty_path, char *gps_family, speed_t target_brate, int *fd_ptr) {
    int i;
    struct termios ttyopt; /* serial port options */
    int gps_tty_dev; /* file descriptor to the serial port of the GNSS module */
    /* TODO: implement proper types with proper serializers below. */
    uint8_t const ubx_cmd_cfg_msgs[] = {
                    0xB5, 0x62, /* UBX Sync Chars */
                    0x06, 0x01, /* CFG-MSG Class/ID */
                    0x08, 0x00, /* Payload length */
                    0x01, 0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, /* Enable NAV-PVT output on serial */
                    0x19, 0xe5, /* Checksum */
                    /* --- */
                    0xB5, 0x62, /* UBX Sync Chars */
                    0x06, 0x01, /* CFG-MSG Class/ID */
                    0x08, 0x00, /* Payload length */
                    0x01, 0x20, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, /* Enable NAV-TIMEGPS output on serial */
                    0x32, 0x94, /* Checksum */
    };

    uint8_t const ubx_cmd_prt[] = {
                    0xB5, 0x62, /* UBX Sync Chars */
                    0x06, 0x00, /* CFG-MSG Class/ID */
                    0x14, 0x00, /* Payload length */
                    0x01,       /* UART 1 */
                    0x00,       /* reserved0 */
                    0x00, 0x80, /* Disable TX ready */
                    0xD0, 0x08, 0x00, 0x00, /* 1 stop bit, no parity bits, 8bit char */
                    0x80, 0x25, 0x00, 0x00, /* 9600 baud */
                    0x01, 0x00, /* Inproto mask, UBX only */
                    0x01, 0x00, /* Outproto mask, UBX only */
                    0x00, 0x00, /* flags */
                    0x00, 0x00, /* reserved5 */
                    0x1A, 0xF9  /* checksum */
    };

    ssize_t num_written;

    /* check input parameters */
    CHECK_NULL(tty_path);
    CHECK_NULL(fd_ptr);

    /* open TTY device */
    gps_tty_dev = open(tty_path, O_RDWR | O_NOCTTY);
    if (gps_tty_dev <= 0) {
        DEBUG_MSG("ERROR: TTY PORT FAIL TO OPEN, CHECK PATH AND ACCESS RIGHTS\n");
        return LGW_GPS_ERROR;
    }
    *fd_ptr = gps_tty_dev;

    /* manage the different GPS modules families */
    if (gps_family == NULL) {
        DEBUG_MSG("WARNING: this version of GPS module may not be supported\n");
    } else if (strncmp(gps_family, "ubx7", 4) != 0) {
        /* The current implementation relies on proprietary messages from U-Blox */
        /* GPS modules (UBX, NAV-TIMEGPS...) and has only be tested with a u-blox 7. */
        /* Those messages allow to get NATIVE GPS time (no leap seconds) required */
        /* for class-B handling and GPS synchronization */
        /* see lgw_parse_ubx() function for details */
        DEBUG_MSG("WARNING: this version of GPS module may not be supported\n");
    }

    /* manage the target bitrate */
    if (target_brate != 0) {
        DEBUG_MSG("WARNING: target_brate parameter ignored for now\n"); // TODO
    }

    /* get actual serial port configuration */
    i = tcgetattr(gps_tty_dev, &ttyopt);
    if (i != 0) {
        DEBUG_MSG("ERROR: IMPOSSIBLE TO GET TTY PORT CONFIGURATION\n");
        return LGW_GPS_ERROR;
    }

    /* Save current serial port configuration for restoring later */
    memcpy(&ttyopt_restore, &ttyopt, sizeof ttyopt);

    /* update baudrates */
    cfsetispeed(&ttyopt, DEFAULT_BAUDRATE);
    cfsetospeed(&ttyopt, DEFAULT_BAUDRATE);

    /* update terminal parameters */
    /* The following configuration should allow to:
            - Get ASCII NMEA messages
            - Get UBX binary messages
            - Send UBX binary commands
        Note: as binary data have to be read/written, we need to disable
              various character processing to avoid loosing data */
    /* Control Modes */
    ttyopt.c_cflag |= CLOCAL;  /* local connection, no modem control */
    ttyopt.c_cflag |= CREAD;   /* enable receiving characters */
    ttyopt.c_cflag |= CS8;     /* 8 bit frames */
    ttyopt.c_cflag &= ~PARENB; /* no parity */
    ttyopt.c_cflag &= ~CSTOPB; /* one stop bit */
    /* Input Modes */
    ttyopt.c_iflag |= IGNPAR;  /* ignore bytes with parity errors */
    ttyopt.c_iflag &= ~ICRNL;  /* do not map CR to NL on input*/
    ttyopt.c_iflag &= ~IGNCR;  /* do not ignore carriage return on input */
    ttyopt.c_iflag &= ~IXON;   /* disable Start/Stop output control */
    ttyopt.c_iflag &= ~IXOFF;  /* do not send Start/Stop characters */
    /* Output Modes */
    ttyopt.c_oflag = 0;        /* disable everything on output as we only write binary */
    /* Local Modes */
    ttyopt.c_lflag &= ~ICANON; /* disable canonical input - cannot use with binary input */
    ttyopt.c_lflag &= ~ISIG;   /* disable check for INTR, QUIT, SUSP special characters */
    ttyopt.c_lflag &= ~IEXTEN; /* disable any special control character */
    ttyopt.c_lflag &= ~ECHO;   /* do not echo back every character typed */
    ttyopt.c_lflag &= ~ECHOE;  /* does not erase the last character in current line */
    ttyopt.c_lflag &= ~ECHOK;  /* do not echo NL after KILL character */
    /* `read()`ing the GPS TTY should return when one of these conditions is true: */
    ttyopt.c_cc[VMIN] = LGW_GPS_MIN_MSG_SIZE; /* a minimum number of bytes are received */
    ttyopt.c_cc[VTIME] = 1;                   /* 100 ms after at the most recent byte is received (unit is tenths of a second) */

    /* set new serial ports parameters */
    i = tcsetattr(gps_tty_dev, TCSANOW, &ttyopt);
    if (i != 0){
        DEBUG_MSG("ERROR: IMPOSSIBLE TO UPDATE TTY PORT CONFIGURATION\n");
        return LGW_GPS_ERROR;
    }
    tcflush(gps_tty_dev, TCIOFLUSH);

    /* Configure UART for UBX protocol only */
    /* This is a binary message, serial port has to be properly configured to handle this */
    num_written = write (gps_tty_dev, ubx_cmd_prt, sizeof(ubx_cmd_prt));
    if (num_written != sizeof(ubx_cmd_prt)) {
        DEBUG_MSG("ERROR: Failed to write on serial port (written=%d)\n", (int) num_written);
    }

    sleep(1);
    tcflush(gps_tty_dev, TCIOFLUSH);

    /* Configure GPS to report NAV-TIMEGPS and NAV-PVT messages */
    num_written = write (gps_tty_dev, ubx_cmd_cfg_msgs, sizeof(ubx_cmd_cfg_msgs));
    if (num_written != sizeof(ubx_cmd_cfg_msgs)) {
        DEBUG_MSG("ERROR: Failed to write on serial port (written=%d)\n", (int) num_written);
    }

    /* get timezone info */
    tzset();

    /* initialize global variables */
    gps_time_ok = false;
    gps_pos_ok = false;
    gps_mod = 'N';

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_gps_disable(int fd) {
    int i;

    /* restore serial ports parameters */
    i = tcsetattr(fd, TCSANOW, &ttyopt_restore);
    if (i){
        DEBUG_MSG("ERROR: IMPOSSIBLE TO RESTORE TTY PORT CONFIGURATION\n");
        return LGW_GPS_ERROR;
    }
    tcflush(fd, TCIOFLUSH);

    i = close(fd);
    if (i) {
        DEBUG_MSG("ERROR: TTY PORT FAIL TO CLOSE\n");
        return LGW_GPS_ERROR;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * ## Notes
 *
 * - Important: Users are recommended to check the `gnssFixOK` flag in
 *   the UBX-NAV-PVT pr the NMEA valid flag. Fixes not marked valid
 *   should not normally be used.
 */
struct ubx_nav_pvt {
    /* GPS time of week of the navigation epoch
     * -
     * ms */
    uint32_t iTOW;
    /* Year (UTC)
     * -
     * y */
    uint16_t year;
    /* Month, range 1..12 (UTC)
     * -
     * month */
    uint8_t  month;
    /* Day of month, range 1..31 (UTC)
     * -
     * d */
    uint8_t  day;
    /* Hour of day, range 0..23 (UTC)
     * -
     * h */
    uint8_t  hour;
    /* Minute of hour, range 0..59 (UTC)
     * -
     * min */
    uint8_t  min;
    /* Seconds of minute, range 0..60 (UTC)
     * -
     * s */
    uint8_t  sec;
    /* Validity flags
     * -
     * - */
    uint8_t  valid;
    /* Time accuracy estimate (UTC)
     * -
     * ns */
    uint32_t tAcc;
    /* Fraction of second, range -1e9 .. 1e9 (UTC)
     * -
     * ns */
    int32_t  nano;
    /* GNSSfix Type:
    * - 0: no fix
    * - 1: dead reckoning only
    * - 2: 2D-fix
    * - 3: 3D-fix
    * - 4: GNSS + dead reckoning combined 5: time only fix */
    uint8_t  fixType;
    /* Fix status flags */
    uint8_t  flags;
    uint8_t  reserved1;
    /* Number of satellites used in Nav Solution */
    uint8_t  numSV;
    /* Longitude
     * 1e-7
     * deg */
    int32_t  lon;
    /* Latitude
     * 1e-7
     * deg */
    int32_t  lat;
    /* Height above ellipsoid
     * -
     * mm */
    int32_t  height;
    /* Height above mean sea level
     * -
     * mm */
    int32_t  hMSL;
    /* Horizontal accuracy estimate
     * -
     * mm */
    uint32_t hAcc;
    /* Vertical accuracy estimate
     * -
     * mm */
    uint32_t vAcc;
    /* NED north velocity
     * -
     * mm/s */
    int32_t  velN;
    /* NED east velocity
     * -
     * mm/s */
    int32_t  velE;
    /* NED down velocity
     * -
     * mm/s */
    int32_t  velD;
    /* Ground Speed (2-D)
     * -
     * mm/s */
    int32_t  gSpeed;
    /* Heading of motion (2-D)
     * 1e-5
     * deg */
    int32_t  headMot;
    /* Speed accuracy estimate
     * -
     * mm/s */
    uint32_t sAcc;
    /* Heading accuracy estimate (both motion and vehicle)
     * 1e-5
     * deg */
    uint32_t headAcc;
    /* Position DOP
     * 0.01
     * - */
    uint16_t pDOP;
    uint16_t reserved2;
    uint32_t reserved3;
};
_Static_assert(sizeof(struct ubx_nav_pvt) == 84, "");

enum ubx_nav_pvt_fix_type {
    ubx_nav_pvt_fix_type_none = 0,
    ubx_nav_pvt_fix_type_dead_reckoning,
    ubx_nav_pvt_fix_type_2d,
    ubx_nav_pvt_fix_type_3d,
    ubx_nav_pvt_fix_type_gnss_with_dead_reckoning,
    ubx_nav_pvt_fix_type_time,
};

static bool ubx_nav_pvt_valid_date(struct ubx_nav_pvt * msg) {
    return !!(msg->valid & 1);
}

static bool ubx_nav_pvt_valid_time(struct ubx_nav_pvt * msg) {
    return !!(msg->valid & (1 << 1));
}

static bool ubx_nav_pvt_fully_resolved(struct ubx_nav_pvt * msg) {
    return !!(msg->valid & (1 << 2));
}

static bool ubx_nav_pvt_valid_fix(struct ubx_nav_pvt * msg) {
    return !!(msg->flags & 1);
}

static void lgw_parse_ubx_nav_pvt(struct cursor * csr, struct ubx_nav_pvt * out) {
    assert(cursor_remaining(csr) >= 84);
    cursor_unpack_le_u32(csr, &out->iTOW);
    cursor_unpack_le_u16(csr, &out->year);
    cursor_unpack_le_u8(csr,  &out->month);
    cursor_unpack_le_u8(csr,  &out->day);
    cursor_unpack_le_u8(csr,  &out->hour);
    cursor_unpack_le_u8(csr,  &out->min);
    cursor_unpack_le_u8(csr,  &out->sec);
    cursor_unpack_le_u8(csr,  &out->valid);
    cursor_unpack_le_u32(csr, &out->tAcc);
    cursor_unpack_le_i32(csr, &out->nano);
    cursor_unpack_le_u8(csr,  &out->fixType);
    cursor_unpack_le_u8(csr,  &out->flags);
    cursor_unpack_le_u8(csr,  &out->reserved1);
    cursor_unpack_le_u8(csr,  &out->numSV);
    cursor_unpack_le_i32(csr, &out->lon);
    cursor_unpack_le_i32(csr, &out->lat);
    cursor_unpack_le_i32(csr, &out->height);
    cursor_unpack_le_i32(csr, &out->hMSL);
    cursor_unpack_le_u32(csr, &out->hAcc);
    cursor_unpack_le_u32(csr, &out->vAcc);
    cursor_unpack_le_i32(csr, &out->velN);
    cursor_unpack_le_i32(csr, &out->velE);
    cursor_unpack_le_i32(csr, &out->velD);
    cursor_unpack_le_i32(csr, &out->gSpeed);
    cursor_unpack_le_i32(csr, &out->headMot);
    cursor_unpack_le_u32(csr, &out->sAcc);
    cursor_unpack_le_u32(csr, &out->headAcc);
    cursor_unpack_le_u16(csr, &out->pDOP);
    cursor_unpack_le_u16(csr, &out->reserved2);
    cursor_unpack_le_u32(csr, &out->reserved3);
}

struct ubx_nav_timegps {
    uint32_t iTOW;
    int32_t  fTOW;
    int16_t  week;
    int8_t   leapS;
    uint8_t  valid;
    uint32_t tAcc;
};

static bool ubx_nav_time_valid_week(struct ubx_nav_timegps * msg) {
    return !!(msg->valid & (1 << 1));
}

static bool ubx_nav_time_valid_tow(struct ubx_nav_timegps * msg) {
    return !!(msg->valid & 1);
}

static void lgw_parse_ubx_nav_timegps(struct cursor * csr, struct ubx_nav_timegps * out) {
    assert(cursor_remaining(csr) >= 16);
    cursor_unpack_le_u32(csr, &out->iTOW);
    cursor_unpack_le_i32(csr, &out->fTOW);
    cursor_unpack_le_i16(csr, &out->week);
    cursor_unpack_le_i8(csr,  &out->leapS);
    cursor_unpack_le_u8(csr,  &out->valid);
    cursor_unpack_le_u32(csr, &out->tAcc);
}

enum gps_msg lgw_parse_ubx(const char *serial_buff, size_t buff_size, size_t *msg_size) {
    *msg_size = 0; /* ensure msg_size alway receives a value */

    /* check input parameters */
    if (serial_buff == NULL) {
        return IGNORED;
    }
    if (buff_size < 8) {
        return IGNORED;
    }

    /* display received serial data and checksum */
    if (0) {
        DEBUG_MSG("Note: parsing UBX frame> ");
        for (size_t i=0; i<buff_size; i++) {
            DEBUG_MSG("%02x ", serial_buff[i]);
        }
        DEBUG_MSG("\n");
    }

    struct cursor csr = cursor_new((char *)serial_buff, buff_size);

    uint16_t ubx_sync = 0;
    CSR_RES(cursor_unpack_be_u16(&csr, &ubx_sync));

    /* Check for UBX sync chars 0xB5 0x62 */
    if (ubx_sync != 0xB562) {
        return IGNORED;
    }

    uint8_t msg_class = 0;
    uint8_t msg_id = 0;
    CSR_RES(cursor_unpack_le_u8(&csr, &msg_class));
    CSR_RES(cursor_unpack_le_u8(&csr, &msg_id));

    /* Get payload length to compute message size */
    uint16_t payload_length;
    {
        CSR_RES(cursor_unpack_le_u16(&csr, &payload_length));
        /* header + payload + checksum */
        *msg_size = 6 + payload_length + 2;
        if(*msg_size > buff_size) {
            /* message contains less bytes than indicated by header */
            return INCOMPLETE;
        }
    }

    /* Validate checksum of message */
    {
        uint8_t ck_a, ck_b;
        uint8_t ck_a_rcv = serial_buff[*msg_size-2]; /* received checksum */
        uint8_t ck_b_rcv = serial_buff[*msg_size-1]; /* received checksum */
        /* Use 8-bit Fletcher Algorithm to compute checksum of actual payload */
        ck_a = 0; ck_b = 0;
        for (int i=0; i<(4 + payload_length); i++) {
            ck_a = ck_a + serial_buff[i+2];
            ck_b = ck_b + ck_a;
        }

        /* Compare checksums and exit if not OK */
        if ((ck_a != ck_a_rcv) && (ck_b != ck_b_rcv)) {
            DEBUG_MSG("ERROR: UBX message is corrupted, checksum failed\n");
            return INVALID;
        }
    }

    /* Check for Class 0x01 (NAV) and ID 0x20 (NAV-TIMEGPS) */
    if (msg_class == 0x01 && msg_id == 0x20) {
        DEBUG_MSG("NOTE: NAV-TIMEGPS\n");
        struct ubx_nav_timegps msg;
        lgw_parse_ubx_nav_timegps(&csr, &msg);

        gps_iTOW = msg.iTOW;
        gps_fTOW = msg.fTOW;
        gps_week = msg.week;
        gps_tAcc = msg.tAcc;
        gps_time_ok = ubx_nav_time_valid_tow(&msg);

        /* For debug */
        if (0) {
            short ubx_gps_hou = 0; /* hours (0-23) */
            short ubx_gps_min = 0; /* minutes (0-59) */
            short ubx_gps_sec = 0; /* seconds (0-59) */

            /* Format GPS time in hh:mm:ss based on iTOW */
            ubx_gps_sec = (gps_iTOW / 1000) % 60;
            ubx_gps_min = (gps_iTOW / 1000 / 60) % 60;
            ubx_gps_hou = (gps_iTOW / 1000 / 60 / 60) % 24;
            printf("  GPS time %02d:%02d:%02d\n", ubx_gps_hou, ubx_gps_min, ubx_gps_sec);
            printf("  valid    0x%02x\n", msg.valid);
            printf("    week     %u\n", ubx_nav_time_valid_week(&msg));
            printf("    time     %u\n", ubx_nav_time_valid_tow(&msg));
            printf("  estimated accuracy:\n");
            printf("    time:    %u ns\n", msg.tAcc);
        }
        return UBX_NAV_TIMEGPS;
    } else if (msg_class == 0x01 && msg_id == 0x07) {
        DEBUG_MSG("NOTE: UBX NAV-PVT\n");
        struct ubx_nav_pvt msg;
        lgw_parse_ubx_nav_pvt(&csr, &msg);

        gps_yea = msg.year;
        gps_mon = msg.month;
        gps_day = msg.day;
        gps_hou = msg.hour;
        gps_min = msg.min;
        gps_sec = msg.sec;
        gps_fra = msg.nano * 1e-9f; /* fractions of seconds (<1) */
        gps_lat = msg.lat;
        gps_lon = msg.lon;
        gps_alt = msg.hMSL;
        gps_hAcc = msg.hAcc;
        gps_vAcc = msg.vAcc;
        gps_numSV = msg.numSV;
        gps_pos_ok = ubx_nav_pvt_valid_fix(&msg);

        /* For debug */
        if (0) {
            short ubx_gps_hou = 0; /* hours (0-23) */
            short ubx_gps_min = 0; /* minutes (0-59) */
            short ubx_gps_sec = 0; /* seconds (0-59) */

            /* Format GPS time in hh:mm:ss based on iTOW */
            ubx_gps_sec = (msg.iTOW / 1000) % 60;
            ubx_gps_min = (msg.iTOW / 1000 / 60) % 60;
            ubx_gps_hou = (msg.iTOW / 1000 / 60 / 60) % 24;
            printf("  GPS time %02d:%02d:%02d\n", ubx_gps_hou, ubx_gps_min, ubx_gps_sec);
            printf("  numSV    %u\n", msg.numSV);
            printf("  valid    0x%02x\n", msg.valid);
            printf("    utc date %u\n", ubx_nav_pvt_valid_date(&msg));
            printf("    utc time %u\n", ubx_nav_pvt_valid_time(&msg));
            printf("    resolved %u\n", ubx_nav_pvt_fully_resolved(&msg));
            printf("  flags    0x%02x\n", msg.flags);
            printf("    fix      %u\n", ubx_nav_pvt_valid_fix(&msg));
            printf("  estimated accuracy:\n");
            printf("    horz:    %f m\n", (float)msg.hAcc/1000.0);
            printf("    vert:    %f m\n", (float)msg.vAcc/1000.0);
        }
        return UBX_NAV_PVT;
    } else if ((serial_buff[2] == 0x05) && (serial_buff[3] == 0x00)) {
        DEBUG_MSG("NOTE: UBX ACK-NAK of clsID 0x%02x, msgID 0x%02x received\n", serial_buff[6], serial_buff[7]);
        return IGNORED;
    } else if ((serial_buff[2] == 0x05) && (serial_buff[3] == 0x01)) {
        DEBUG_MSG("NOTE: UBX ACK-ACK of clsID 0x%02x, msgID 0x%02x received\n", serial_buff[6], serial_buff[7]);
        return IGNORED;
    } else { /* not a supported message */
        DEBUG_MSG("ERROR: UBX message is not supported (%02x %02x)\n", serial_buff[2], serial_buff[3]);
        return IGNORED;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

enum gps_msg lgw_parse_nmea(const char *serial_buff, int buff_size) {
    int i, j, k;
    int str_index[30]; /* string index from the string chopping */
    int nb_fields; /* number of strings detected by string chopping */
    char parser_buf[256]; /* parsing modifies buffer so need a local copy */

    /* check input parameters */
    if (serial_buff == NULL) {
        return UNKNOWN;
    }

    if(buff_size > (int)(sizeof(parser_buf) - 1)) {
        DEBUG_MSG("Note: input string to big for parsing\n");
        return INVALID;
    }

    /* look for some NMEA sentences in particular */
    if (buff_size < 8) {
        DEBUG_MSG("ERROR: TOO SHORT TO BE A VALID NMEA SENTENCE\n");
        return UNKNOWN;
    } else if (!validate_nmea_checksum(serial_buff, buff_size)) {
        DEBUG_MSG("Warning: invalid NMEA sentence (bad checksum)\n");
        return INVALID;
    } else if (match_label(serial_buff, "$G?RMC", 6, '?')) {
        /*
        NMEA sentence format: $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode*cs<CR><LF>
        Valid fix: $GPRMC,083559.34,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A*00
        No fix: $GPRMC,,V,,,,,,,,,,N*00
        */
        memcpy(parser_buf, serial_buff, buff_size);
        parser_buf[buff_size] = '\0';
        nb_fields = str_chop(parser_buf, buff_size, ',', str_index, ARRAY_SIZE(str_index));
        if (nb_fields != 13) {
            DEBUG_MSG("Warning: invalid RMC sentence (number of fields)\n");
            return IGNORED;
        }
        /* parse GPS status */
        gps_mod = *(parser_buf + str_index[12]); /* get first character, no need to bother with sscanf */
        if ((gps_mod != 'N') && (gps_mod != 'A') && (gps_mod != 'D')) {
            gps_mod = 'N';
        }
        /* parse complete time */
        i = sscanf(parser_buf + str_index[1], "%2hd%2hd%2hd%4f", &gps_hou, &gps_min, &gps_sec, &gps_fra);
        j = sscanf(parser_buf + str_index[9], "%2hd%2hd%2hd", &gps_day, &gps_mon, &gps_yea);
        if ((i == 4) && (j == 3)) {
            if ((gps_mod == 'A') || (gps_mod == 'D')) {
                gps_time_ok = true;
                DEBUG_MSG("Note: Valid RMC sentence, GPS locked, date: 20%02d-%02d-%02dT%02d:%02d:%06.3fZ\n", gps_yea, gps_mon, gps_day, gps_hou, gps_min, gps_fra + (float)gps_sec);
            } else {
                gps_time_ok = false;
                DEBUG_MSG("Note: Valid RMC sentence, no satellite fix, estimated date: 20%02d-%02d-%02dT%02d:%02d:%06.3fZ\n", gps_yea, gps_mon, gps_day, gps_hou, gps_min, gps_fra + (float)gps_sec);
            }
        } else {
            /* could not get a valid hour AND date */
            gps_time_ok = false;
            DEBUG_MSG("Note: Valid RMC sentence, mode %c, no date\n", gps_mod);
        }
        return NMEA_RMC;
    } else if (match_label(serial_buff, "$G?GGA", 6, '?')) {
        /*
        NMEA sentence format: $xxGGA,time,lat,NS,long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs<CR><LF>
        Valid fix: $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B
        */
        memcpy(parser_buf, serial_buff, buff_size);
        parser_buf[buff_size] = '\0';
        nb_fields = str_chop(parser_buf, buff_size, ',', str_index, ARRAY_SIZE(str_index));
        if (nb_fields != 15) {
            DEBUG_MSG("Warning: invalid GGA sentence (number of fields)\n");
            return IGNORED;
        }
        /* parse number of satellites used for fix */
        sscanf(parser_buf + str_index[7], "%hd", &gps_sat);
        /* parse 3D coordinates */
        i = sscanf(parser_buf + str_index[2], "%2hd%10lf", &gps_dla, &gps_mla);
        gps_ola = *(parser_buf + str_index[3]);
        j = sscanf(parser_buf + str_index[4], "%3hd%10lf", &gps_dlo, &gps_mlo);
        gps_olo = *(parser_buf + str_index[5]);
        k = sscanf(parser_buf + str_index[9], "%d", &gps_alt);
        if ((i == 2) && (j == 2) && (k == 1) && ((gps_ola=='N')||(gps_ola=='S')) && ((gps_olo=='E')||(gps_olo=='W'))) {
            gps_pos_ok = true;
            DEBUG_MSG("Note: Valid GGA sentence, %d sat, lat %02ddeg %06.3fmin %c, lon %03ddeg%06.3fmin %c, alt %d\n", gps_sat, gps_dla, gps_mla, gps_ola, gps_dlo, gps_mlo, gps_olo, gps_alt);
        } else {
            /* could not get a valid latitude, longitude AND altitude */
            gps_pos_ok = false;
            DEBUG_MSG("Note: Valid GGA sentence, %d sat, no coordinates\n", gps_sat);
        }
        return NMEA_GGA;
    } else {
        return IGNORED;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_gps_get(struct timespec *utc, struct timespec *utc_acc, struct timespec *gps_time, struct coord_s *loc) {
    struct tm x;
    time_t y;
    double intpart, fractpart;

    if (utc != NULL) {
        if (!gps_time_ok) {
            DEBUG_MSG("ERROR: NO VALID TIME TO RETURN\n");
            return LGW_GPS_ERROR;
        }
        memset(&x, 0, sizeof(x));
        if (gps_yea < 100) { /* 2-digits year, 20xx */
            x.tm_year = gps_yea + 100; /* 100 years offset to 1900 */
        } else { /* 4-digits year, Gregorian calendar */
            x.tm_year = gps_yea - 1900;
        }
        x.tm_mon = gps_mon - 1; /* tm_mon is [0,11], gps_mon is [1,12] */
        x.tm_mday = gps_day;
        x.tm_hour = gps_hou;
        x.tm_min = gps_min;
        x.tm_sec = gps_sec;
        y = mktime(&x) - timezone; /* need to substract timezone bc mktime assumes time vector is local time */
        if (y == (time_t)(-1)) {
            DEBUG_MSG("ERROR: FAILED TO CONVERT BROKEN-DOWN TIME\n");
            return LGW_GPS_ERROR;
        }
        utc->tv_sec = y;
        utc->tv_nsec = (int32_t)(gps_fra * 1e9);
    }
    if (gps_time != NULL) {
        if (!gps_time_ok) {
            DEBUG_MSG("ERROR: NO VALID TIME TO RETURN\n");
            return LGW_GPS_ERROR;
        }
        fractpart = modf(((double)gps_iTOW / 1E3) + ((double)gps_fTOW / 1E9), &intpart);
        /* Number of seconds since beginning on current GPS week */
        gps_time->tv_sec = (time_t)intpart;
        /* Number of seconds since GPS epoch 06.Jan.1980 */
        gps_time->tv_sec += (time_t)gps_week * 604800; /* day*hours*minutes*secondes: 7*24*60*60; */
        /* Fractional part in nanoseconds */
        gps_time->tv_nsec = (long)(fractpart * 1E9);
    }
    if (utc_acc != NULL) {
        if (!gps_time_ok) {
            DEBUG_MSG("ERROR: NO VALID TIME TO RETURN\n");
            return LGW_GPS_ERROR;
        }
        utc_acc->tv_sec = (time_t)0;
        utc_acc->tv_nsec = (long)gps_tAcc;
    }
    if (loc != NULL) {
        if (!gps_pos_ok) {
            DEBUG_MSG("ERROR: NO VALID POSITION TO RETURN\n");
            return LGW_GPS_ERROR;
        }
        loc->lat = (double)gps_lat * 1e-7d;
        loc->lon = (double)gps_lon * 1e-7d;
        loc->alt = gps_alt/1000;
        loc->eha = gps_hAcc / 1000.0;
        loc->eva = gps_vAcc / 1000.0;
        loc->nsv = gps_numSV;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_gps_sync(struct tref *ref, uint32_t count_us, struct timespec utc, struct timespec utc_acc, struct timespec gps_time) {
    double cnt_diff; /* internal concentrator time difference (in seconds) */
    double utc_diff; /* UTC time difference (in seconds) */
    double slope; /* time slope between new reference and old reference (for sanity check) */

    bool aber_n0; /* is the update value for synchronization aberrant or not ? */
    static bool aber_min1 = false; /* keep track of whether value at sync N-1 was aberrant or not  */
    static bool aber_min2 = false; /* keep track of whether value at sync N-2 was aberrant or not  */

    CHECK_NULL(ref);

    /* calculate the slope */

    cnt_diff = (double)(count_us - ref->count_us) / (double)(TS_CPS); /* uncorrected by xtal_err */
    utc_diff = (double)(utc.tv_sec - (ref->utc).tv_sec) + (1E-9 * (double)(utc.tv_nsec - (ref->utc).tv_nsec));

    /* detect aberrant points by measuring if slope limits are exceeded */
    if (utc_diff != 0) { // prevent divide by zero
        slope = cnt_diff/utc_diff;
        if ((slope > PLUS_10PPM) || (slope < MINUS_10PPM)) {
            DEBUG_MSG("Warning: correction range exceeded\n");
            aber_n0 = true;
        } else {
            aber_n0 = false;
        }
    } else {
        DEBUG_MSG("Warning: aberrant UTC value for synchronization\n");
        aber_n0 = true;
    }

    /* watch if the 3 latest sync point were aberrant or not */
    if (aber_n0 == false) {
        /* value no aberrant -> sync with smoothed slope */
        ref->systime = time(NULL);
        ref->count_us = count_us;
        ref->utc.tv_sec = utc.tv_sec;
        ref->utc.tv_nsec = utc.tv_nsec;
        ref->utc_acc.tv_sec = utc_acc.tv_sec;
        ref->utc_acc.tv_nsec = utc_acc.tv_nsec;
        ref->gps.tv_sec = gps_time.tv_sec;
        ref->gps.tv_nsec = gps_time.tv_nsec;
        ref->xtal_err = slope;
        aber_min2 = aber_min1;
        aber_min1 = aber_n0;
        return LGW_GPS_SUCCESS;
    } else if (aber_n0 && aber_min1 && aber_min2) {
        /* 3 successive aberrant values -> sync reset (keep xtal_err) */
        ref->systime = time(NULL);
        ref->count_us = count_us;
        ref->utc.tv_sec = utc.tv_sec;
        ref->utc.tv_nsec = utc.tv_nsec;
        ref->gps.tv_sec = gps_time.tv_sec;
        ref->gps.tv_nsec = gps_time.tv_nsec;
        /* reset xtal_err only if the present value is out of range */
        if ((ref->xtal_err > PLUS_10PPM) || (ref->xtal_err < MINUS_10PPM)) {
            ref->xtal_err = 1.0;
        }
        DEBUG_MSG("Warning: 3 successive aberrant sync attempts, sync reset\n");
        aber_min2 = aber_min1;
        aber_min1 = aber_n0;
        return LGW_GPS_SUCCESS;
    } else {
        /* only 1 or 2 successive aberrant values -> ignore and return an error */
        aber_min2 = aber_min1;
        aber_min1 = aber_n0;
        return LGW_GPS_ERROR;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_cnt2utc(struct tref ref, uint32_t count_us, struct timespec *utc) {
    double delta_sec;
    double intpart, fractpart;
    long tmp;

    CHECK_NULL(utc);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_10PPM) || (ref.xtal_err < MINUS_10PPM)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR CNT -> UTC CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference count_us and target count_us */
    delta_sec = (double)(count_us - ref.count_us) / (TS_CPS * ref.xtal_err);

    /* now add that delta to reference UTC time */
    fractpart = modf (delta_sec , &intpart);
    tmp = ref.utc.tv_nsec + (long)(fractpart * 1E9);
    if (tmp < (long)1E9) { /* the nanosecond part doesn't overflow */
        utc->tv_sec = ref.utc.tv_sec + (time_t)intpart;
        utc->tv_nsec = tmp;
    } else { /* must carry one second */
        utc->tv_sec = ref.utc.tv_sec + (time_t)intpart + 1;
        utc->tv_nsec = tmp - (long)1E9;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_utc2cnt(struct tref ref, struct timespec utc, uint32_t *count_us) {
    double delta_sec;

    CHECK_NULL(count_us);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_10PPM) || (ref.xtal_err < MINUS_10PPM)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR UTC -> CNT CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference utc and target utc */
    delta_sec = (double)(utc.tv_sec - ref.utc.tv_sec);
    delta_sec += 1E-9 * (double)(utc.tv_nsec - ref.utc.tv_nsec);

    /* now convert that to internal counter tics and add that to reference counter value */
    *count_us = ref.count_us + (uint32_t)(delta_sec * TS_CPS * ref.xtal_err);

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_cnt2gps(struct tref ref, uint32_t count_us, struct timespec *gps_time) {
    double delta_sec;
    double intpart, fractpart;
    long tmp;

    CHECK_NULL(gps_time);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_10PPM) || (ref.xtal_err < MINUS_10PPM)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR CNT -> GPS CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in milliseconds between reference count_us and target count_us */
    delta_sec = (double)(count_us - ref.count_us) / (TS_CPS * ref.xtal_err);

    /* now add that delta to reference GPS time */
    fractpart = modf (delta_sec , &intpart);
    tmp = ref.gps.tv_nsec + (long)(fractpart * 1E9);
    if (tmp < (long)1E9) { /* the nanosecond part doesn't overflow */
        gps_time->tv_sec = ref.gps.tv_sec + (time_t)intpart;
        gps_time->tv_nsec = tmp;
    } else { /* must carry one second */
        gps_time->tv_sec = ref.gps.tv_sec + (time_t)intpart + 1;
        gps_time->tv_nsec = tmp - (long)1E9;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_gps2cnt(struct tref ref, struct timespec gps_time, uint32_t *count_us) {
    double delta_sec;

    CHECK_NULL(count_us);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_10PPM) || (ref.xtal_err < MINUS_10PPM)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR GPS -> CNT CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference gps time and target gps time */
    delta_sec = (double)(gps_time.tv_sec - ref.gps.tv_sec);
    delta_sec += 1E-9 * (double)(gps_time.tv_nsec - ref.gps.tv_nsec);

    /* now convert that to internal counter tics and add that to reference counter value */
    *count_us = ref.count_us + (uint32_t)(delta_sec * TS_CPS * ref.xtal_err);

    return LGW_GPS_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */

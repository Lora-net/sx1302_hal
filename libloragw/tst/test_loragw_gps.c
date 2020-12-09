/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for the loragw_gps module

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf */
#include <string.h>     /* memset */
#include <signal.h>     /* sigaction */
#include <stdlib.h>     /* exit */
#include <unistd.h>     /* read */

#include "loragw_hal.h"
#include "loragw_gps.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define MATCH(a,b) ( ((int32_t)(a-b)<=1) && ((int32_t)(a-b)>=-1) ) /* tolerate 1µs */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define COM_TYPE_DEFAULT LGW_COM_SPI
#define COM_PATH_DEFAULT "/dev/spidev0.0"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

struct tref ppm_ref;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);
static void gps_process_sync(void);
static void gps_process_coords(void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

void usage(void) {
    //printf("Library version information: %s\n", lgw_version_info());
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -u        set COM type as USB (default is SPI)\n");
    printf(" -d <path> COM path to be used to connect the concentrator\n");
    printf("            => default path (SPI): " COM_PATH_DEFAULT "\n");
    printf(" -k <uint> Concentrator clock source (Radio A or Radio B) [0..1]\n");
    printf(" -r <uint> Radio type (1255, 1257, 1250)\n");
}

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = 1;;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

static void gps_process_sync(void) {
    /* variables for PPM pulse GPS synchronization */
    uint32_t ppm_tstamp;
    struct timespec ppm_gps;
    struct timespec ppm_utc;

    /* variables for timestamp <-> GPS time conversions */
    uint32_t x, z;
    struct timespec y;

    /* get GPS time for synchronization */
    int i = lgw_gps_get(&ppm_utc, &ppm_gps, NULL, NULL);
    if (i != LGW_GPS_SUCCESS) {
        printf("    No valid reference GPS time available, synchronization impossible.\n");
        return;
    }

    /* get timestamp for synchronization */
    i = lgw_get_trigcnt(&ppm_tstamp);
    if (i != LGW_HAL_SUCCESS) {
        printf("    Failed to read timestamp, synchronization impossible.\n");
        return;
    }

    /* try to update synchronize time reference with the new GPS & timestamp */
    i = lgw_gps_sync(&ppm_ref, ppm_tstamp, ppm_utc, ppm_gps);
    if (i != LGW_GPS_SUCCESS) {
        printf("    Synchronization error.\n");
        return;
    }

    /* display result */
    printf("    * Synchronization successful *\n");
    printf("    UTC reference time: %lld.%09ld\n", (long long)ppm_ref.utc.tv_sec, ppm_ref.utc.tv_nsec);
    printf("    GPS reference time: %lld.%09ld\n", (long long)ppm_ref.gps.tv_sec, ppm_ref.gps.tv_nsec);
    printf("    Internal counter reference value: %u\n", ppm_ref.count_us);
    printf("    Clock error: %.9f\n", ppm_ref.xtal_err);

    x = ppm_tstamp + 500000;

    /* CNT -> GPS -> CNT */
    printf("\n");
    printf("    * Test of timestamp counter <-> GPS value conversion *\n");
    printf("    Test value: %u\n", x);
    lgw_cnt2gps(ppm_ref, x, &y);
    printf("    Conversion to GPS: %lld.%09ld\n", (long long)y.tv_sec, y.tv_nsec);
    lgw_gps2cnt(ppm_ref, y, &z);
    printf("    Converted back: %u ==> %dµs\n", z, (int32_t)(z-x));
    /* Display test result */
    if (MATCH(x,z)) {
        printf("    ** PASS **: (SX1302 -> GPS -> SX1302) conversion MATCH\n");
    } else {
        printf("    ** FAILED **: (SX1302 -> GPS -> SX1302) conversion MISMATCH\n");
    }

    /* CNT -> UTC -> CNT */
    printf("\n");
    printf("    * Test of timestamp counter <-> UTC value conversion *\n");
    printf("    Test value: %u\n", x);
    lgw_cnt2utc(ppm_ref, x, &y);
    printf("    Conversion to UTC: %lld.%09ld\n", (long long)y.tv_sec, y.tv_nsec);
    lgw_utc2cnt(ppm_ref, y, &z);
    printf("    Converted back: %u ==> %dµs\n", z, (int32_t)(z-x));
    /* Display test result */
    if (MATCH(x,z)) {
        printf("    ** PASS **: (SX1302 -> UTC -> SX1302) conversion MATCH\n");
    } else {
        printf("    ** FAILED **: (SX1302 -> UTC -> SX1302) conversion MISMATCH\n");
    }
}

static void gps_process_coords(void) {
    /* position variable */
    struct coord_s coord;
    struct coord_s gpserr;
    int    i = lgw_gps_get(NULL, NULL, &coord, &gpserr);

    /* update gateway coordinates */
    if (i == LGW_GPS_SUCCESS) {
        printf("\n");
        printf("# GPS coordinates: latitude %.5f, longitude %.5f, altitude %i m\n", coord.lat, coord.lon, coord.alt);
        printf("# GPS err:         latitude %.5f, longitude %.5f, altitude %i m\n", gpserr.lat, gpserr.lon, gpserr.alt);
    }
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    /* SPI interfaces */
    const char com_path_default[] = COM_PATH_DEFAULT;
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = COM_TYPE_DEFAULT;

    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    int i;
    unsigned int arg_u;

    /* concentrator variables */
    uint8_t clocksource = 0;
    lgw_radio_type_t radio_type = LGW_RADIO_TYPE_SX1250;
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;

    /* serial variables */
    char serial_buff[128]; /* buffer to receive GPS data */
    size_t wr_idx = 0;     /* pointer to end of chars in buffer */
    int gps_tty_dev; /* file descriptor to the serial port of the GNSS module */

    /* NMEA/UBX variables */
    enum gps_msg latest_msg; /* keep track of latest NMEA/UBX message parsed */

    /* parse command line options */
    while ((i = getopt (argc, argv, "hk:r:d:u")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'd':
                if (optarg != NULL) {
                    com_path = optarg;
                }
                break;
            case 'u':
                com_type = LGW_COM_USB;
                break;
            case 'r': /* <uint> Radio type */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || ((arg_u != 1255) && (arg_u != 1257) && (arg_u != 1250))) {
                    printf("ERROR: argument parsing of -r argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    switch (arg_u) {
                        case 1255:
                            radio_type = LGW_RADIO_TYPE_SX1255;
                            break;
                        case 1257:
                            radio_type = LGW_RADIO_TYPE_SX1257;
                            break;
                        default: /* 1250 */
                            radio_type = LGW_RADIO_TYPE_SX1250;
                            break;
                    }
                }
                break;
            case 'k': /* <uint> Clock Source */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u > 1)) {
                    printf("ERROR: argument parsing of -k argument. Use -h to print help\n");
                    return EXIT_FAILURE;
                } else {
                    clocksource = (uint8_t)arg_u;
                }
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                exit(EXIT_FAILURE);
        }
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    /* Intro message and library information */
    printf("Beginning of test for loragw_gps.c\n");
    printf("*** Library version information ***\n%s\n***\n", lgw_version_info());

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh start") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    /* Open and configure GPS */
    i = lgw_gps_enable("/dev/ttyS0", "ubx7", 0, &gps_tty_dev);
    if (i != LGW_GPS_SUCCESS) {
        printf("ERROR: Failed to enable GPS\n");
        exit(EXIT_FAILURE);
    }

    /* start concentrator (default conf for IoT SK) */
    /* board config */
    memset(&boardconf, 0, sizeof(boardconf));
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    boardconf.full_duplex = false;
    boardconf.com_type = com_type;
    strncpy(boardconf.com_path, com_path, sizeof boardconf.com_path);
    boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return EXIT_FAILURE;
    }

    /* set configuration for RF chains */
    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 868000000;
    rfconf.rssi_offset = 0.0;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return EXIT_FAILURE;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 868000000;
    rfconf.rssi_offset = 0.0;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return EXIT_FAILURE;
    }

    /* start */
    if (lgw_start() != LGW_HAL_SUCCESS) {
        printf("ERROR: IMPOSSIBLE TO START THE GATEWAY\n");
        exit(EXIT_FAILURE);
    }

    /* initialize some variables before loop */
    memset(serial_buff, 0, sizeof serial_buff);
    memset(&ppm_ref, 0, sizeof ppm_ref);

    /* loop until user action */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        size_t rd_idx = 0;
        size_t frame_end_idx = 0;

        /* blocking non-canonical read on serial port */
        ssize_t nb_char = read(gps_tty_dev, serial_buff + wr_idx, LGW_GPS_MIN_MSG_SIZE);
        if (nb_char <= 0) {
            printf("WARNING: [gps] read() returned value %zd\n", nb_char);
            continue;
        }
        wr_idx += (size_t)nb_char;

        /*******************************************
         * Scan buffer for UBX/NMEA sync chars and *
         * attempt to decode frame if one is found *
         *******************************************/
        while (rd_idx < wr_idx) {
            size_t frame_size = 0;

            /* Scan buffer for UBX sync char */
            if (serial_buff[rd_idx] == (char)LGW_GPS_UBX_SYNC_CHAR) {

                /***********************
                 * Found UBX sync char *
                 ***********************/
                latest_msg = lgw_parse_ubx(&serial_buff[rd_idx], (wr_idx - rd_idx), &frame_size);

                if (frame_size > 0) {
                    if (latest_msg == INCOMPLETE) {
                        /* UBX header found but frame appears to be missing bytes */
                        frame_size = 0;
                    } else if (latest_msg == INVALID) {
                        /* message header received but message appears to be corrupted */
                        printf("WARNING: [gps] could not get a valid message from GPS (no time)\n");
                        frame_size = 0;
                    } else if (latest_msg == UBX_NAV_TIMEGPS) {
                        printf("\n~~ UBX NAV-TIMEGPS sentence, triggering synchronization attempt ~~\n");
                        gps_process_sync();
                    }
                }
            } else if(serial_buff[rd_idx] == (char)LGW_GPS_NMEA_SYNC_CHAR) {
                /************************
                 * Found NMEA sync char *
                 ************************/
                /* scan for NMEA end marker (LF = 0x0a) */
                char* nmea_end_ptr = memchr(&serial_buff[rd_idx],(int)0x0a, (wr_idx - rd_idx));

                if (nmea_end_ptr) {
                    /* found end marker */
                    frame_size = nmea_end_ptr - &serial_buff[rd_idx] + 1;
                    latest_msg = lgw_parse_nmea(&serial_buff[rd_idx], frame_size);

                    if(latest_msg == INVALID || latest_msg == UNKNOWN) {
                        /* checksum failed */
                        frame_size = 0;
                    } else if (latest_msg == NMEA_RMC) { /* Get location from RMC frames */
                        gps_process_coords();
                    }
                }
            }

            if (frame_size > 0) {
                /* At this point message is a checksum verified frame
                   we're processed or ignored. Remove frame from buffer */
                rd_idx += frame_size;
                frame_end_idx = rd_idx;
            } else {
                rd_idx++;
            }
        } /* ...for(rd_idx = 0... */

        if (frame_end_idx) {
          /* Frames have been processed. Remove bytes to end of last processed frame */
          memcpy(serial_buff,&serial_buff[frame_end_idx],wr_idx - frame_end_idx);
          wr_idx -= frame_end_idx;
        } /* ...for(rd_idx = 0... */

        /* Prevent buffer overflow */
        if ((sizeof(serial_buff) - wr_idx) < LGW_GPS_MIN_MSG_SIZE) {
            memcpy(serial_buff,&serial_buff[LGW_GPS_MIN_MSG_SIZE],wr_idx - LGW_GPS_MIN_MSG_SIZE);
            wr_idx -= LGW_GPS_MIN_MSG_SIZE;
        }
    }

    /* clean up before leaving */
    if (exit_sig == 1) {
        lgw_gps_disable(gps_tty_dev);
        lgw_stop();
    }

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh stop") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    printf("\nEnd of test for loragw_gps.c\n");
    exit(EXIT_SUCCESS);
}

/* --- EOF ------------------------------------------------------------------ */

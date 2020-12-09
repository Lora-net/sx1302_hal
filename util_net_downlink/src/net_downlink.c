/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
 (C)2019 Semtech

 Description:
    Network packet sender, sends UDP packets to a running packet forwarder
    Network packet receiver, receives UDP packets from a running packet forwarder.

 License: Revised BSD License, see LICENSE.TXT file include in the project
 */

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* Fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
#define _XOPEN_SOURCE 600
#else
#define _XOPEN_SOURCE 500
#endif

#if defined(__GNUC__) && __GNUC__ >= 7
 #define FALL_THROUGH __attribute__ ((fallthrough))
#else
 #define FALL_THROUGH ((void)0)
#endif /* __GNUC__ >= 7 */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf, fprintf, sprintf, fopen, fputs */
#include <stdlib.h>     /* EXIT_* */
#include <unistd.h>     /* usleep */
#include <stdbool.h>    /* bool type */

#include <string.h>     /* memset */
#include <time.h>       /* time, clock_gettime, strftime, gmtime, clock_nanosleep*/
#include <errno.h>      /* error messages */
#include <sys/time.h>   /* timeval */

#include <sys/socket.h> /* socket specific definitions */
#include <netinet/in.h> /* INET constants and stuff */
#include <arpa/inet.h>  /* IP address conversion stuff */
#include <netdb.h>      /* gai_strerror */

#include <signal.h>     /* sigaction */

#include <pthread.h>

#include "parson.h"
#include "base64.h"

/* -------------------------------------------------------------------------- */
/* --- MACROS & CONSTANTS --------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

#define PROTOCOL_VERSION    2

/* Get a particular bit value from a byte */
/* b: any byte
 p: index >=0
 n: number of bits >=1 */
/**
 @brief Get a particular bit value from a byte
 @param b [in]   Any byte from which we want a bit value
 @param p [in]   Position of the bit in the byte [0..7]
 @param n [in]   Number of bits we want to get
 @return The value corresponding the requested bits
 */
#define TAKE_N_BITS_FROM( b, p, n ) ((b) >> (p)) & ((1 << (n)) - 1)

/* Constants */
#define DEFAULT_LORA_BW             125     /* LoRa modulation bandwidth, kHz */
#define DEFAULT_LORA_SF             7       /* LoRa SF */
#define DEFAULT_LORA_CR             "4/5"   /* LoRa CR */
#define DEFAULT_FSK_FDEV            25      /* FSK frequency deviation */
#define DEFAULT_FSK_BR              50      /* FSK bitrate */
#define DEFAULT_LORA_PREAMBLE_SIZE  8       /* LoRa preamble size */
#define DEFAULT_PAYLOAD_SIZE        4       /* payload size, bytes */
#define PUSH_TIMEOUT_MS             100

/* -------------------------------------------------------------------------- */
/* --- CUSTOM TYPES --------------------------------------------------------- */

typedef enum
{
    PKT_PUSH_DATA = 0,
    PKT_PUSH_ACK = 1,
    PKT_PULL_DATA = 2,
    PKT_PULL_RESP = 3,
    PKT_PULL_ACK = 4,
    PKT_TX_ACK = 5
} pkt_type_t;

typedef struct
{
    uint32_t    nb_loop[2]; /* number of downlinks to be sent on each RF chain */
    uint32_t    delay_ms[2]; /* delay between 2 downlinks on each RF chain */
    int         sock; /* socket file descriptor */
    double      freq_mhz[2];
    double      freq_step;
    uint8_t     freq_nb;
    uint8_t     rf_chain;
    uint16_t    bandwidth_khz;
    char        modulation_rf0[8];
    char        modulation_rf1[8];
    uint8_t     spread_factor[2];
    char        coding_rate[8];
    float       br_kbps;
    uint8_t     fdev_khz;
    int8_t      rf_power[2];
    uint16_t    preamb_size[2];
    uint8_t     pl_size[2];
    bool        ipol;
} thread_params_t;

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* Socket info variables */
static bool sockaddr_valid = false;
static struct sockaddr_storage dist_addr_down;
static socklen_t addr_len_down = sizeof dist_addr_down;

/* Thread variables */
static pthread_mutex_t mx_sockaddr = PTHREAD_MUTEX_INITIALIZER; /* control access to the sockaddr info */

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler( int sigio );
static void usage( void );
static void * thread_down_rf0( const void * arg );
static void * thread_down_rf1( const void * arg );
static void log_csv(FILE * file, uint8_t * buf);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main( int argc, char **argv )
{
    int i, j, x; /* loop variable and temporary variable for return value */
    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
    unsigned arg_u = 0;
    unsigned arg_u2 = 0;
    double arg_f = 0.0;
    double arg_f_step = 0.0;
    double arg_f2 = 0.0;
    int arg_i = 0;
    int arg_i2 = 0;
    char arg_s[8];
    char arg_s2[8];
    bool parse_err = false;

    /* Logging file variables */
    const char * log_fname = NULL; /* pointer to a string we won't touch */
    FILE * log_file = NULL;
    bool is_first = true;

    /* Server socket creation */
    int sock; /* socket file descriptor */
    struct addrinfo hints;
    struct addrinfo * result; /* store result of getaddrinfo */
    struct addrinfo * q; /* pointer to move into *result data */
    char host_name[64];
    char port_name[64];
    const char * port_arg = NULL;
    struct sockaddr_storage dist_addr;
    socklen_t addr_len = sizeof dist_addr;

    /* Uplink forwarder */
    bool fwd_uplink = false;
    int sock_fwd = -1; /* socket file descriptor */
    char serv_addr[64] = "127.0.0.1";
    char serv_port_fwd[8] = "1700";
    struct timeval push_timeout_half = {0, (PUSH_TIMEOUT_MS * 500)};

    /* Variables for receiving and sending packets */
    uint8_t databuf_up[32768];
    uint8_t databuf_ack[4];
    int byte_nb;

    /* Variables for protocol management */
    uint32_t raw_mac_h; /* Most Significant Nibble, network order */
    uint32_t raw_mac_l; /* Least Significant Nibble, network order */
    uint64_t gw_mac; /* MAC address of the client (gateway) */
    uint8_t ack_command;
    bool no_ack;

    /* Downlink variables */
    thread_params_t thread_params = {
        .nb_loop = {0, 0},
        .delay_ms = {1000, 1000},
        .bandwidth_khz = DEFAULT_LORA_BW,
        .spread_factor = {DEFAULT_LORA_SF, DEFAULT_LORA_SF},
        .modulation_rf0 = {"LORA"},
        .modulation_rf1 = {"LORA"},
        .coding_rate = DEFAULT_LORA_CR,
        .br_kbps = DEFAULT_FSK_BR,
        .fdev_khz = DEFAULT_FSK_FDEV,
        .rf_power = {27, 27},
        .preamb_size = {DEFAULT_LORA_PREAMBLE_SIZE, DEFAULT_LORA_PREAMBLE_SIZE},
        .pl_size = {DEFAULT_PAYLOAD_SIZE, DEFAULT_PAYLOAD_SIZE},
        .freq_step = 0.2,
        .freq_nb = 1,
        .ipol = false
    };

    /* Threads ID */
    pthread_t thrid_down_rf0;
    pthread_t thrid_down_rf1;

    /* Parse command line options */
    while( ( i = getopt( argc, argv, "b:c:f:hij:l:p:r:s:t:x:z:A:F:P:m:d:q:" ) ) != -1 )
    {
        switch( i )
        {
            case 'h':
                usage( );
                return EXIT_SUCCESS;
                break;

            case 'l':
                log_fname = optarg;
                break;

            case 'P':
                port_arg = optarg;
                break;

            case 'A':
                fwd_uplink = true;
                strncpy( serv_addr, optarg, sizeof serv_addr );
                serv_addr[sizeof serv_addr - 1] = '\0'; /* ensure string termination */
                break;

            case 'F':
                strncpy( serv_port_fwd, optarg, sizeof serv_port_fwd );
                serv_port_fwd[sizeof serv_port_fwd - 1] = '\0'; /* ensure string termination */
                break;

            case 'f': /* -f <float,float>  target frequency in MHz */
                j = sscanf( optarg, "%lf,%lf", &arg_f, &arg_f2 );
                switch( j )
                {
                    case 2:
                        if( (arg_f2 < 30.0) || (arg_f2 > 3000.0) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.freq_mhz[1] = arg_f2;
                        }
                        FALL_THROUGH;
                    case 1:
                        if( (arg_f < 30.0) || (arg_f > 3000.0) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.freq_mhz[0] = arg_f;
                        }
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -f argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'j':
                j = sscanf( optarg, "%u:%lf", &arg_u, &arg_f_step );
                switch( j )
                {
                    case 2:
                        if( (arg_f_step < 0.05) || (arg_f_step > 20.0) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.freq_step = arg_f_step;
                        }
                          FALL_THROUGH;
                    case 1:
                        if( (arg_u == 0) || (arg_u > 100) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.freq_nb = arg_u;
                        }
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -j argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'b': /* -b <uint>  LoRa modulation bandwidth */
                j = sscanf( optarg, "%u", &arg_u );
                if( (j != 1) || ((arg_u != 125) && (arg_u != 250) && (arg_u != 500)) )
                {
                    printf( "ERROR: argument parsing of -b argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                else
                {
                    thread_params.bandwidth_khz = (uint16_t)arg_u;
                }
                break;

            case 's': /* -s <uint>  LoRa Spreading Factor */
                j = sscanf( optarg, "%u,%u", &arg_u, &arg_u2 );
                switch( j )
                {
                    case 2:
                        if( (arg_u2 < 5) || (arg_u2 > 12) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.spread_factor[1] = (uint8_t)arg_u2;
                        }
                          FALL_THROUGH;
                    case 1:
                        if( (arg_u < 5) || (arg_u > 12) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.spread_factor[0] = (uint8_t)arg_u;
                        }
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -s argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'c': /* -c <string>  LoRa Coding Rate */
                j = sscanf( optarg, "%s", arg_s );
                if( j != 1 )
                {
                    printf( "ERROR: argument parsing of -c argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                else
                {
                    strncpy( thread_params.coding_rate, arg_s, sizeof thread_params.coding_rate );
                    thread_params.coding_rate[sizeof thread_params.coding_rate - 1] = '\0'; /* ensure string termination */
                }
                break;

            case 'm': /* -m <string> Modulation */
                j = sscanf( optarg, "%[^,],%[^,]", arg_s, arg_s2 );
                switch( j )
                {
                    case 2:
                        strncpy( thread_params.modulation_rf1, arg_s2, sizeof thread_params.modulation_rf1 );
                        thread_params.modulation_rf1[sizeof thread_params.modulation_rf1 - 1] = '\0'; /* ensure string termination */
                          FALL_THROUGH;
                    case 1:
                        strncpy( thread_params.modulation_rf0, arg_s, sizeof thread_params.modulation_rf0 );
                        thread_params.modulation_rf0[sizeof thread_params.modulation_rf0 - 1] = '\0'; /* ensure string termination */
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -m argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'd': /* <uint> FSK frequency deviation */
                i = sscanf(optarg, "%u", &arg_u);
                if ((i != 1) || (arg_u < 1) || (arg_u > 250)) {
                    printf("ERROR: invalid FSK frequency deviation\n");
                    usage( );
                    return EXIT_FAILURE;
                } else {
                    thread_params.fdev_khz = (uint8_t)arg_u;
                }
                break;

            case 'q': /* <float> FSK bitrate */
                i = sscanf(optarg, "%lf", &arg_f);
                if ((i != 1) || (arg_f < 0.5) || (arg_f > 250)) {
                    printf("ERROR: invalid FSK bitrate\n");
                    usage( );
                    return EXIT_FAILURE;
                } else {
                    thread_params.br_kbps = arg_f;
                }
                break;

            case 'p': /* -p <int>  RF power (dBm) */
                j = sscanf( optarg, "%i,%i", &arg_i, &arg_i2 );
                switch( j )
                {
                    case 2:
                        if( (arg_i2 < -60) || (arg_i2 > 60) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.rf_power[1] = (int8_t)arg_i2;
                        }
                          FALL_THROUGH;
                    case 1:
                        if( (arg_i < -60) || (arg_i > 60) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.rf_power[0] = (int8_t)arg_i;
                        }
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -p argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'r': /* -r <uint>  preamble size */
                j = sscanf( optarg, "%u,%u", &arg_u, &arg_u2 );
                switch( j )
                {
                    case 2:
                        if( (arg_u2 < 5) || (arg_u2 > 65535) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.preamb_size[1] = (uint16_t)arg_u2;
                        }
                          FALL_THROUGH;
                    case 1:
                        if( (arg_u < 5) || (arg_u > 65535) )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.preamb_size[0] = (uint16_t)arg_u;
                        }
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -r argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'z': /* -z <uint>  payload length (bytes) */
                j = sscanf( optarg, "%u,%u", &arg_u, &arg_u2 );
                switch( j )
                {
                    case 2:
                        if( arg_u2 > 255 )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.pl_size[1] = (uint8_t)arg_u2;
                        }
                          FALL_THROUGH;
                    case 1:
                        if( arg_u > 255 )
                        {
                            parse_err = true;
                        }
                        else
                        {
                            thread_params.pl_size[0] = (uint8_t)arg_u;
                        }
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -z argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'i':
                thread_params.ipol = true;
                break;

            case 't':
                j = sscanf( optarg, "%u,%u", &arg_u, &arg_u2 );
                switch( j )
                {
                    case 2:
                        thread_params.delay_ms[1] = (uint32_t)arg_u2;
                          FALL_THROUGH;
                    case 1:
                        thread_params.delay_ms[0] = (uint32_t)arg_u;
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -t argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            case 'x':
                j = sscanf( optarg, "%u,%u", &arg_u, &arg_u2 );
                switch( j )
                {
                    case 2:
                        thread_params.nb_loop[1] = (uint32_t)arg_u2;
                          FALL_THROUGH;
                    case 1:
                        thread_params.nb_loop[0] = (uint32_t)arg_u;
                        break;
                    default:
                        parse_err = true;
                }
                if( parse_err )
                {
                    printf( "ERROR: argument parsing of -x argument\n" );
                    usage( );
                    return EXIT_FAILURE;
                }
                break;

            default:
                printf( "ERROR: argument parsing options, use -h option for help\n" );
                usage( );
                return EXIT_FAILURE;
        }
    }

    /* Check input arguments */
    if( port_arg == NULL )
    {
        printf( "ERROR: missing argument, use -h option for help\n" );
        usage( );
        return EXIT_FAILURE;
    }

    /* Start message */
    printf( "+++ Start of network uplink logger (30ms delay) +++\n" );

    /* Configure socket for uplink forwarding if required */
    if( fwd_uplink == true )
    {
        /* Prepare hints to open network sockets */
        memset( &hints, 0, sizeof hints );
        hints.ai_family = AF_UNSPEC; /* should handle IP v4 or v6 automatically */
        hints.ai_socktype = SOCK_DGRAM; /* we want UDP sockets */
        hints.ai_protocol = IPPROTO_UDP; /* we want UDP sockets */
        hints.ai_flags = AI_ADDRCONFIG; /* do not return IPv6 results if there is no IPv6 network connection, same with IPv4 */

        /* Look for server address w/ upstream port */
        x = getaddrinfo( serv_addr, serv_port_fwd, &hints, &result );
        if( x != 0 )
        {
            printf( "ERROR: [up] getaddrinfo on address %s (PORT %s) returned %s\n", serv_addr, serv_port_fwd, gai_strerror( x ) );
            return EXIT_FAILURE;
        }

        /* Try to open UDP socket for upstream traffic */
        for( q = result; q != NULL; q = q->ai_next )
        {
            sock_fwd = socket( q->ai_family, q->ai_socktype, q->ai_protocol );
            if( sock_fwd == -1 ) continue; /* try next field */
            else break; /* success, get out of loop */
        }
        if( q == NULL )
        {
            printf( "ERROR: [up] failed to open socket to any of server %s addresses (port %s)\n", serv_addr, serv_port_fwd );
            return EXIT_FAILURE;
        }
        else
        {
            getnameinfo( q->ai_addr, q->ai_addrlen, host_name, sizeof host_name, port_name, sizeof port_name, NI_NUMERICHOST );
            printf( "INFO: socket %i opened for upstream traffic, host: %s, port: %s\n", sock_fwd, host_name, port_name );
        }

        /* Connect the UDP socket so we can send/receive packet with the server only */
        x = connect( sock_fwd, q->ai_addr, q->ai_addrlen );
        if( x != 0 )
        {
            printf( "ERROR: [up] connect returned %s\n", strerror( errno ) );
            return EXIT_FAILURE;
        }

        /* Free the result of getaddrinfo */
        freeaddrinfo( result );

        /* Set upstream socket RX timeout */
        x = setsockopt( sock_fwd, SOL_SOCKET, SO_RCVTIMEO, (void *)&(push_timeout_half), sizeof push_timeout_half );
        if( x != 0 )
        {
            printf( "ERROR: [up] setsockopt returned %s\n", strerror( errno ) );
            exit( EXIT_FAILURE );
        }
    }

    /* Prepare hints to open network sockets */
    memset( &hints, 0, sizeof hints );
    hints.ai_family = AF_UNSPEC; /* should handle IP v4 or v6 automatically */
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; /* will assign local IP automatically */

    /* Look for address */
    x = getaddrinfo( NULL, port_arg, &hints, &result );
    if( x != 0 )
    {
        printf( "ERROR: getaddrinfo returned %s\n", gai_strerror( x ) );
        return EXIT_FAILURE;
    }

    /* Try to open socket and bind it */
    for( q = result; q != NULL; q = q->ai_next )
    {
        sock = socket( q->ai_family, q->ai_socktype, q->ai_protocol );
        if( sock == -1 )
        {
            continue; /* socket failed, try next field */
        }
        else
        {
            x = bind( sock, q->ai_addr, q->ai_addrlen );
            if( x == -1 )
            {
                shutdown( sock, SHUT_RDWR );
                continue; /* bind failed, try next field */
            }
            else
            {
                thread_params.sock = sock;
                break; /* success, get out of loop */
            }
        }
    }
    if( q == NULL )
    {
        printf( "ERROR: failed to open socket or to bind to it\n" );
        i = 1;
        for( q = result; q != NULL; q = q->ai_next )
        {
            getnameinfo( q->ai_addr, q->ai_addrlen, host_name, sizeof host_name, port_name, sizeof port_name, NI_NUMERICHOST );
            printf( "INFO: result %i host:%s service:%s\n", i, host_name, port_name );
            ++i;
        }
        return EXIT_FAILURE;
    }
    printf( "INFO: util_net_downlink listening on port %s\n", port_arg );
    freeaddrinfo( result );

    /* Open log file */
    if( log_fname )
    {
        log_file = fopen( log_fname, "w+" ); /* create log file, overwrite if file already exist */
        if( log_file == NULL )
        {
            printf( "ERROR: impossible to create log file %s\n", log_fname );
            return EXIT_FAILURE;
        }
    }

    /* Configure signal handling */
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction( SIGQUIT, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );

    i = pthread_create( &thrid_down_rf0, NULL, (void * (*)( void * ))thread_down_rf0, (void*)&thread_params );
    if( i != 0 )
    {
        printf( "ERROR: [main] impossible to create downstream thread\n" );
        return EXIT_FAILURE;
    }

    i = pthread_create( &thrid_down_rf1, NULL, (void * (*)( void * ))thread_down_rf1, (void*)&thread_params );
    if( i != 0 )
    {
        printf( "ERROR: [main] impossible to create downstream thread for RF1\n" );
        return EXIT_FAILURE;
    }

    /* Loop until user quits */
    while( ( quit_sig != 1 ) && ( exit_sig != 1 ) )
    {
        /* Wait to receive a packet */
        memset( databuf_up, 0, 4096 );
        byte_nb = recvfrom( sock, databuf_up, sizeof databuf_up, 0, (struct sockaddr *)&dist_addr, &addr_len );
        if( byte_nb == -1 )
        {
            printf( "ERROR: recvfrom returned %s \n", strerror( errno ) );
            continue;
        }

        /* Display info about the sender */
        x = getnameinfo( (struct sockaddr *)&dist_addr, addr_len, host_name, sizeof host_name, port_name, sizeof port_name, NI_NUMERICHOST );
        if( x == -1 )
        {
            printf( "ERROR: getnameinfo returned %s \n", gai_strerror( x ) );
            return EXIT_FAILURE;
        }
        printf( " -> pkt in , host %s (port %s), %i bytes", host_name, port_name, byte_nb );

        /* Check and parse the payload */
        if( byte_nb < 12 )
        {
            /* Not enough bytes for packet from gateway */
            printf( " (too short for GW <-> MAC protocol)\n" );
            continue;
        }
        /* Don't touch the token in position 1-2, it will be sent back "as is" for acknowledgement */

        /* Check protocol version number */
        if( databuf_up[0] != PROTOCOL_VERSION )
        {
            printf( ", invalid version %u\n", databuf_up[0] );
            continue;
        }
        raw_mac_h = *( (uint32_t *)( databuf_up + 4 ) );
        raw_mac_l = *( (uint32_t *)( databuf_up + 8 ) );
        gw_mac = ( (uint64_t)ntohl( raw_mac_h ) << 32 ) + (uint64_t)ntohl( raw_mac_l );

        /* Interpret gateway command and select ACK to be sent */
        switch( databuf_up[3] )
        {
            case PKT_PUSH_DATA:
                printf( ", PUSH_DATA from gateway 0x%08X%08X\n", (uint32_t)( gw_mac >> 32 ), (uint32_t)( gw_mac & 0xFFFFFFFF ) );
                ack_command = PKT_PUSH_ACK;
                no_ack = false;
                if( fwd_uplink == false )
                {
                    printf( "<-  pkt out, PUSH_ACK for host %s (port %s)", host_name, port_name );
                }
                else
                {
                    /* Forward uplink if required */
                    printf( "<-  pkt out, PUSH_ACK for host %s (port %s), FORWARD PUSH_DATA to %s (port %s)", host_name, port_name, serv_addr, serv_port_fwd );
                    x = send( sock_fwd, (void *)databuf_up, byte_nb, 0 );
                    if( x == -1 )
                    {
                        printf( "ERROR: failed to forward uplink packet - %s (%d)\n", strerror(errno), errno);
                    }
                }
                break;

            case PKT_PULL_DATA:
                printf( ", PULL_DATA from gateway 0x%08X%08X\n", (uint32_t)( gw_mac >> 32 ), (uint32_t)( gw_mac & 0xFFFFFFFF ) );
                ack_command = PKT_PULL_ACK;
                no_ack = false;
                printf( "<-  pkt out, PULL_ACK for host %s (port %s)", host_name, port_name );
                /* Record who sent the PULL_DATA for the downlink thread to known where to send PULL_RESP */
                memcpy( &dist_addr_down, &dist_addr, sizeof(struct sockaddr_storage) );
                memcpy( &addr_len_down, &addr_len, sizeof(socklen_t) );
                pthread_mutex_lock( &mx_sockaddr );
                sockaddr_valid = true;
                pthread_mutex_unlock( &mx_sockaddr );
                break;

            case PKT_TX_ACK:
                printf( ", TX_ACK from gateway 0x%08X%08X\n", (uint32_t)( gw_mac >> 32 ), (uint32_t)( gw_mac & 0xFFFFFFFF ) );
                no_ack = true;
                break;

            default:
                printf( ", unexpected command %u\n", databuf_up[3] );
                continue;
        }

        /* Add some artificial latency */
        usleep( 30000 ); /* 30 ms */

        /* Send acknowledge and check return value */
        if( no_ack == false )
        {
            memset( databuf_ack, 0, 4 );
            databuf_ack[0] = PROTOCOL_VERSION;
            databuf_ack[1] = databuf_up[1];
            databuf_ack[2] = databuf_up[2];
            databuf_ack[3] = ack_command;
            byte_nb = sendto( sock, (void *)databuf_ack, 4, 0, (struct sockaddr *)&dist_addr, addr_len );
            if( byte_nb == -1 )
            {
                printf( ", send error:%s\n", strerror( errno ) );
            }
            else
            {
                printf( ", %i bytes sent for ACK\n", byte_nb );
            }
        }

        /* Log uplinks to file */
        if( databuf_up[3] == PKT_PUSH_DATA )
        {
            if( log_fname != NULL )
            {
                if( is_first == true )
                {
                    fprintf(log_file, "tmst,ftime,chan,rfch,freq,mid,stat,modu,datr,bw,codr,rssic,rssis,lsnr,size,data\n");
                    is_first = false;
                }
                log_csv( log_file, &databuf_up[12] );
            }
        }
    }

    /* Wait for downstream thread to finish */
    pthread_join( thrid_down_rf0, NULL );
    pthread_join( thrid_down_rf1, NULL );

    printf( "INFO: Exiting uplink logger\n" );

    /* Close log file */
    if( (log_fname != NULL) && (log_file != NULL) )
    {
        fclose( log_file );
        log_file = NULL;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void log_csv(FILE * file, uint8_t * buf)
{
    JSON_Object * rxpk = NULL;
    JSON_Object * root = NULL;
    JSON_Array * rxpk_array = NULL;
    JSON_Value * root_val = NULL;
    JSON_Value * val = NULL;
    int i, j, rxpk_nb, x;
    const char * str; /* pointer to sub-strings in the JSON data */
    short x0, x1;
    uint8_t payload[255];
    uint8_t size;

    if( file == NULL )
    {
        printf("ERROR: no file opened\n");
        return;
    }

    /* Parse JSON string */
    root_val = json_parse_string( (const char *)buf ); /* JSON offset */
    root = json_value_get_object( root_val );
    if( root == NULL )
    {
        printf( "ERROR: not a valid JSON string\n" );
        json_value_free( root_val );
        return;
    }

    /* Get all packets from array */
    rxpk_array = json_object_get_array( root, "rxpk" );
    if( rxpk_array != NULL)
    {
        rxpk_nb = (int)json_array_get_count( rxpk_array );
        for( i = 0; i < rxpk_nb; i++ )
        {
            rxpk = json_array_get_object( rxpk_array, i );
            if( rxpk == NULL)
            {
                printf("ERROR: failed to get rxpk object\n");
                json_value_free( root_val );
                return;
            }

            /* Parse rxpk fields */
            val = json_object_get_value( rxpk, "tmst" );
            if( json_value_get_type( val ) != JSONNumber )
            {
                printf( "ERROR: wrong type for tmst\n" );
                json_value_free( root_val );
                return;
            }
            fprintf(file, "%u", (uint32_t)json_value_get_number( val ) );

            /* optional field */
            val = json_object_get_value( rxpk, "ftime" );
            if( val != NULL )
            {

                if( json_value_get_type( val ) != JSONNumber )
                {
                    printf( "ERROR: wrong type for tmst\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%u", (uint32_t)json_value_get_number( val ) );
            } else {
                fprintf(file, "," );
            }

            val = json_object_get_value( rxpk, "chan" );
            if( json_value_get_type( val ) != JSONNumber )
            {
                printf( "ERROR: wrong type for chan\n" );
                json_value_free( root_val );
                return;
            }
            fprintf(file, ",%u", (uint8_t)json_value_get_number( val ) );

            val = json_object_get_value( rxpk, "rfch" );
            if( json_value_get_type( val ) != JSONNumber )
            {
                printf( "ERROR: wrong type for rfch\n" );
                json_value_free( root_val );
                return;
            }
            fprintf(file, ",%u", (uint8_t)json_value_get_number( val ) );

            val = json_object_get_value( rxpk, "freq" );
            if( json_value_get_type( val ) != JSONNumber )
            {
                printf( "ERROR: wrong type for rfch\n" );
                json_value_free( root_val );
                return;
            }
            fprintf(file, ",%f", json_value_get_number( val ) );

            val = json_object_get_value( rxpk, "mid" );
            if( json_value_get_type( val ) != JSONNumber )
            {
                printf( "ERROR: wrong type for mid\n" );
                json_value_free( root_val );
                return;
            }
            fprintf(file, ",%u", (uint8_t)json_value_get_number( val ) );

            val = json_object_get_value( rxpk, "stat" );
            if( json_value_get_type( val ) != JSONNumber )
            {
                printf( "ERROR: wrong type for stat\n" );
                json_value_free( root_val );
                return;
            }
            fprintf(file, ",%d", (int8_t)json_value_get_number( val ) );

            val = json_object_get_value( rxpk, "modu" );
            if( json_value_get_type( val ) != JSONString )
            {
                printf( "ERROR: wrong type for stat\n" );
                json_value_free( root_val );
                return;
            }
            str = json_value_get_string( val );
            fprintf(file, ",%s", str );
            if( strcmp( str, "LORA" ) == 0 )
            {
                val = json_object_get_value( rxpk, "datr" );
                if( json_value_get_type( val ) != JSONString )
                {
                    printf( "ERROR: wrong type for datr\n" );
                    json_value_free( root_val );
                    return;
                }
                str = json_value_get_string( val );
                x = sscanf( str, "SF%2hdBW%3hd", &x0, &x1 );
                if( x != 2 )
                {
                    printf( "ERROR: format error in \"rxpk.datr\"\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%d,%d", x0, x1 );

                val = json_object_get_value( rxpk, "codr" );
                if( json_value_get_type( val ) != JSONString )
                {
                    printf( "ERROR: wrong type for codr\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%s", json_value_get_string( val ) );

                val = json_object_get_value( rxpk, "rssi" );
                if( json_value_get_type( val ) != JSONNumber )
                {
                    printf( "ERROR: wrong type for rssic\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%.1f", json_value_get_number( val ) );

                val = json_object_get_value( rxpk, "rssis" );
                if( json_value_get_type( val ) != JSONNumber )
                {
                    printf( "ERROR: wrong type for rssis\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%.1f", json_value_get_number( val ) );

                val = json_object_get_value( rxpk, "lsnr" );
                if( json_value_get_type( val ) != JSONNumber )
                {
                    printf( "ERROR: wrong type for lsnr\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%.1f", json_value_get_number( val ) );
            }
            else if( strcmp( str, "FSK" ) == 0 )
            {
                val = json_object_get_value( rxpk, "datr" );
                if( json_value_get_type( val ) != JSONNumber )
                {
                    printf( "ERROR: wrong type for datr\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%d,,", (uint32_t)json_value_get_number( val ) ); /* bw,codr fields are left empty */

                val = json_object_get_value( rxpk, "rssi" );
                if( json_value_get_type( val ) != JSONNumber )
                {
                    printf( "ERROR: wrong type for rssic\n" );
                    json_value_free( root_val );
                    return;
                }
                fprintf(file, ",%.1f,,", json_value_get_number( val ) ); /* rssis,lsnr fields are left empty */
            }
            else
            {
                printf("ERROR: unknown modulation %s\n", str);
                json_value_free( root_val );
                return;
            }

            val = json_object_get_value( rxpk, "size" );
            if( json_value_get_type( val ) != JSONNumber )
            {
                printf( "ERROR: wrong type for size\n" );
                json_value_free( root_val );
                return;
            }
            size = (uint8_t)json_value_get_number( val );
            fprintf(file, ",%u", size );

            val = json_object_get_value( rxpk, "data" );
            if( json_value_get_type( val ) != JSONString )
            {
                printf( "ERROR: wrong type for data\n" );
                json_value_free( root_val );
                return;
            }
            str = json_value_get_string( val );
            x = b64_to_bin( str, strlen( str ), payload, sizeof payload );
            if( x != size )
            {
                printf( "ERROR: mismatch between .size and .data size once converter to binary\n" );
                json_value_free( root_val );
                return;
            }
            fprintf(file, "," );
            for( j = 0; j < size; j++ )
            {
                fprintf(file, "%02x", payload[j] );
            }

            /* End line */
            fprintf(file, "\n" );
        }
    }

    fflush(file);
    json_value_free( root_val );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void usage( void )
{
    printf( "~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf( " -h  print this help\n" );
    printf( " -f <float,float>   Target frequency in MHz for RF0,RF1\n" );
    printf( " -j <uint>:<float>  Number of channels to jump + explicit offset in MHz between channels\n" );
    printf( " -m <string,string> Modulation [\"LORA\",\"FSK\"] for RF0,RF1\n" );
    printf( " -b <uint>          LoRa bandwidth in kHz [125, 250, 500]\n" );
    printf( " -s <uint,uint>     LoRa Spreading Factor [5-12] for RF0,RF1\n" );
    printf( " -c <string>        LoRa Coding Rate [\"4/5\", \"4/6\", ...]\n" );
    printf( " -d <uint>          FSK frequency deviation in kHz [1:250]\n");
    printf( " -q <float>         FSK bitrate in kbps [0.5:250]\n");
    printf( " -p <int,int>       RF power (dBm) for RF0,RF1\n" );
    printf( " -r <uint,uint>     Preamble size (symbols, [6..65535]) for RF0,RF1\n" );
    printf( " -z <uint,uint>     Payload size (bytes, [0..255]) for RF0,RF1\n" );
    printf( " -i                 Set inverted polarity true\n" );
    printf( " -t <uint,uint>     Number of milliseconds between two downlinks for RF0,RF1\n" );
    printf( " -x <uint,uint>     Number of downlinks to be sent for RF0,RF1\n" );
    printf( " -P <udp port>      UDP port of the Packet Forwarder\n" );
    printf( " -A <ip address>    IP address to be used for uplink forwarding (optional)\n" );
    printf( " -F <udp port>      UDP port to be used for uplink forwarding (optional)\n" );
    printf( " -l <filename>      uplink logging CSV filename (optional)\n" );
    printf( " -B                 Bypass downlink, for uplink logging only (optional)\n" );
    printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf( "~~~ Examples ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
    printf( " Log uplinks into a CSV file, no downlink:\n" );
    printf( "   ./net_downlink -P 1730 -l log.csv\n" );
    printf( " Send downlinks on RF chain 0 only:\n" );
    printf( "   ./net_downlink -f 865.1 -s 7 -b 125 -r 8 -t 500 -x 10 -P 1730\n" );
    printf( " Send downlinks on RF chain 1 only:\n" );
    printf( "   ./net_downlink -f 865.1,865.3 -s 7,8 -t 1000,1000 -x 0,10 -P 1730\n" );
    printf( " Send downlinks on both RF chain 0 and 1:\n" );
    printf( "   ./net_downlink -f 865.1,865.9 -s 10,7 -t 500,1000 -x 5,10 -P 1730\n" );
    printf( " Trigger continuous TX on both RF chain 0 and 1:\n" );
    printf( "   ./net_downlink -f 865.1,865.9 -s 11,12 -x 1,1 -r 65535,65535 -P 1730\n" );
    printf( " Log uplinks into CSV file while continuous TX is running (full_duplex testing):\n" );
    printf( "   ./net_downlink -f 864.5 -s 12 -x 1 -r 65535 -P 1730 -l log.csv\n" );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void sig_handler( int sigio )
{
    if( sigio == SIGQUIT )
    {
        quit_sig = 1;
    }
    else if( ( sigio == SIGINT ) || ( sigio == SIGTERM ) )
    {
        exit_sig = 1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void prepare_downlink_json( const thread_params_t * params, uint8_t rf_chain, uint32_t pkt_sent, JSON_Value * root_val )
{
    int j;

    JSON_Value *val = NULL;
    JSON_Object *root_obj = NULL;
    JSON_Object *obj = NULL;

    char datarate_string[16];
    uint8_t payload[255];
    uint8_t payload_b64[341];
    double freq;
    uint8_t sf;
    int8_t rf_pwr;
    uint16_t pream_sz;
    const char *modulation = (rf_chain == 0) ? params->modulation_rf0 : params->modulation_rf1;

    memset( datarate_string, 0, sizeof datarate_string );
    memset( payload, 0, sizeof payload );
    memset( payload_b64, 0, sizeof payload_b64 );

    root_obj = json_value_get_object( root_val );
    if( root_obj == NULL )
    {
        printf( "ERROR: failed to get JSON root object\n" );
    }
    else
    {
        json_object_set_value( root_obj, "txpk", json_value_init_object( ) );
        obj = json_object_get_object( root_obj, "txpk" );

        /* Set downlink parameters */
        json_object_set_boolean( obj, "imme", true );
        freq = params->freq_mhz[rf_chain] + ((pkt_sent % params->freq_nb) * params->freq_step);
        json_object_set_number( obj, "freq", freq );
        json_object_set_number( obj, "rfch", rf_chain );
        rf_pwr = params->rf_power[rf_chain];
        json_object_set_number( obj, "powe", rf_pwr );
        if( strncmp( modulation, "LORA", 4 ) == 0 )
        {
            json_object_set_string( obj, "modu", "LORA" );
            sf = params->spread_factor[rf_chain];
            sprintf( datarate_string, "SF%uBW%u", sf, params->bandwidth_khz);
            json_object_set_string( obj, "datr", datarate_string );
            json_object_set_string( obj, "codr", params->coding_rate );
        } else if( strncmp( modulation, "FSK", 3 ) == 0 ) {
            json_object_set_string( obj, "modu", "FSK" );
            json_object_set_number( obj, "datr", params->br_kbps * 1E3 );
            json_object_set_number( obj, "fdev", params->fdev_khz * 1E3 );
        } else {
            printf( "ERROR: wrong modulation\n" );
        }
        json_object_set_boolean( obj, "ipol", params->ipol );
        pream_sz = params->preamb_size[rf_chain];
        json_object_set_number( obj, "prea", pream_sz );
        json_object_set_boolean( obj, "ncrc", true );
        json_object_set_boolean( obj, "nhdr", false );
        json_object_set_number( obj, "size", params->pl_size[rf_chain] );

        /* Fill last bytes of payload with downlink counter (32 bits) */
        for( j = 0; j < params->pl_size[rf_chain]; j++ )
        {
            payload[params->pl_size[rf_chain] - ( j + 1 )] = (uint8_t)( (pkt_sent >> (j * 8)) & 0xFF );
        }
        /* Convert payload to base64 */
        j = bin_to_b64( payload, params->pl_size[rf_chain], (char *)(payload_b64), 341 ); /* 255 bytes = 340 chars in b64 + null char */
        if( j >= 0 )
        {
            json_object_set_string( obj, "data", (char *)(payload_b64) );
        }
        else
        {
            printf( "ERROR: failed to convert payload to base64 string\n" );
        }

        /* Free memory */
        json_value_free( val );
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void * thread_down_rf0( const void * arg )
{
    int x;
    int byte_nb;
    const thread_params_t *params = ( (thread_params_t*)arg );

    /* Server socket creation */
    char host_name[64];
    char port_name[64];

    /* JSON variables */
    JSON_Value *root_val = NULL;
    char *serialized_string = NULL;

    /* Downstream data variables */
    uint8_t databuf_down[4096];
    uint32_t nb_loop;
    uint32_t pkt_sent = 0;

    /* Global loop is the max loop defined */
    nb_loop = params->nb_loop[0];
    while( !exit_sig && !quit_sig && (pkt_sent < nb_loop) && (nb_loop > 0) )
    {
        /* Wait for socket address to be valid */
        pthread_mutex_lock( &mx_sockaddr );
        if( sockaddr_valid == false )
        {
            pthread_mutex_unlock( &mx_sockaddr );
            printf( "Waiting for socket to be ready...\n" );
            usleep( 500000 ); /* 500 ms */
            continue;
        }
        pthread_mutex_unlock( &mx_sockaddr );

        /* Display info about the sender */
        x = getnameinfo( (struct sockaddr *)&dist_addr_down, addr_len_down, host_name, sizeof host_name, port_name, sizeof port_name, NI_NUMERICHOST );
        if( x == -1 )
        {
            printf( "ERROR: getnameinfo returned %s \n", gai_strerror( x ) );
            usleep( 10000); /* 10 ms */
            continue;
        }

        /* Prepare JSON object to be sent */
        root_val = json_value_init_object( );
        if( root_val == NULL )
        {
            printf( "ERROR: failed to initialize JSON root object\n" );
        }
        else
        {
            /* Prepare the txpk JSON object */
            prepare_downlink_json( params, 0, pkt_sent, root_val );

            /* Convert JSON object to string */
            serialized_string = json_serialize_to_string( root_val );
            printf( "%s\n", serialized_string );

            /* Send JSON string to socket */
            memset( databuf_down, 0, 4096 );
            databuf_down[0] = PROTOCOL_VERSION;
            databuf_down[1] = 0;
            databuf_down[2] = 0;
            databuf_down[3] = PKT_PULL_RESP;
            memcpy( &databuf_down[4], (uint8_t*)serialized_string, strlen(serialized_string) );
            byte_nb = sendto( params->sock, (void *)databuf_down, strlen(serialized_string) + 4, 0, (struct sockaddr *)&dist_addr_down, addr_len_down );
            if( byte_nb == -1 )
            {
                printf( "ERROR: failed to send downlink to socket - %s\n", strerror( errno ) );
            }
            else
            {
                printf( "<-  pkt out, PULL_RESP for host %s (port %s), %i bytes sent for downlink (%d)\n", host_name, port_name, byte_nb, pkt_sent );
            }

            /* free JSON memory */
            json_free_serialized_string( serialized_string );
            json_value_free( root_val );
        }

        /* One more downlink sent */
        pkt_sent += 1;
        /* Wait before sending next downlink */
        usleep( params->delay_ms[0] * 1E3 );
    }

    /* Exit */
    printf( "\nINFO: End of downstream thread for RF 0\n" );
    return NULL;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void * thread_down_rf1( const void * arg )
{
    int x;
    int byte_nb;
    const thread_params_t *params = ( (thread_params_t*)arg );

    /* Server socket creation */
    char host_name[64];
    char port_name[64];

    /* JSON variables */
    JSON_Value *root_val = NULL;
    char *serialized_string = NULL;

    /* Downstream data variables */
    uint8_t databuf_down[4096];
    uint32_t nb_loop;
    uint32_t pkt_sent = 0;

    /* Global loop is the max loop defined */
    nb_loop = params->nb_loop[1];
    while( !exit_sig && !quit_sig && (pkt_sent < nb_loop) && (nb_loop > 0) )
    {
        /* Wait for socket address to be valid */
        pthread_mutex_lock( &mx_sockaddr );
        if( sockaddr_valid == false )
        {
            pthread_mutex_unlock( &mx_sockaddr );
            printf( "Waiting for socket to be ready...\n" );
            usleep( 500000 ); /* 500 ms */
            continue;
        }
        pthread_mutex_unlock( &mx_sockaddr );

        /* Display info about the sender */
        x = getnameinfo( (struct sockaddr *)&dist_addr_down, addr_len_down, host_name, sizeof host_name, port_name, sizeof port_name, NI_NUMERICHOST );
        if( x == -1 )
        {
            printf( "ERROR: getnameinfo returned %s \n", gai_strerror( x ) );
            usleep( 10000); /* 10 ms */
            continue;
        }

        /* Prepare JSON object to be sent */
        root_val = json_value_init_object( );
        if( root_val == NULL )
        {
            printf( "ERROR: failed to initialize JSON root object\n" );
        }
        else
        {
            /* Prepare the txpk JSON object */
            prepare_downlink_json( params, 1, pkt_sent, root_val );

            /* Convert JSON object to string */
            serialized_string = json_serialize_to_string( root_val );
            printf( "%s\n", serialized_string );

            /* Send JSON string to socket */
            memset( databuf_down, 0, 4096 );
            databuf_down[0] = PROTOCOL_VERSION;
            databuf_down[1] = 0;
            databuf_down[2] = 0;
            databuf_down[3] = PKT_PULL_RESP;
            memcpy( &databuf_down[4], (uint8_t*)serialized_string, strlen(serialized_string) );
            byte_nb = sendto( params->sock, (void *)databuf_down, strlen(serialized_string) + 4, 0, (struct sockaddr *)&dist_addr_down, addr_len_down );
            if( byte_nb == -1 )
            {
                printf( "ERROR: failed to send downlink to socket - %s\n", strerror( errno ) );
            }
            else
            {
                printf( "<-  pkt out, PULL_RESP for host %s (port %s), %i bytes sent for downlink (%d)\n", host_name, port_name, byte_nb, pkt_sent );
            }

            /* free JSON memory */
            json_free_serialized_string( serialized_string );
            json_value_free( root_val );
        }

        /* One more downlink sent */
        pkt_sent += 1;
        /* Wait before sending next downlink */
        usleep( params->delay_ms[1] * 1E3 );
    }

    /* Exit */
    printf( "\nINFO: End of downstream thread for RF1\n" );
    return NULL;
}

/* --- EOF ------------------------------------------------------------------ */

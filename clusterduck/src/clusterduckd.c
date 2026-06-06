/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech
  (C)2026 Taqi Systems (Malaysia)
  (C)2026 Flitnetics (Singapore)

Description:
    Configure LoRaDuck "SuperDuck" - PapaDuck concentrator and forward packets to a internet as a long range sink!
    Use GPS for packet timestamping.
    Send a becon at a regular interval without server intervention

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

#include <stdint.h>         /* C99 types */
#include <stdbool.h>        /* bool type */
#include <stdio.h>          /* printf, fprintf, snprintf, fopen, fputs */
#include <inttypes.h>       /* PRIx64, PRIu64... */

#include <string.h>         /* memset */
#include <strings.h>        /* strcasecmp */
#include <signal.h>         /* sigaction */
#include <time.h>           /* time, clock_gettime, strftime, gmtime */
#include <sys/time.h>       /* timeval */
#include <unistd.h>         /* getopt, access */
#include <stdlib.h>         /* atoi, exit */
#include <errno.h>          /* error messages */
#include <math.h>           /* modf */

#include <sys/socket.h>     /* socket specific definitions */
#include <netinet/in.h>     /* INET constants and stuff */
#include <arpa/inet.h>      /* IP address conversion stuff */
#include <netdb.h>          /* gai_strerror */

#include <pthread.h>

#include "trace.h"
#include "jitqueue.h"
#include "parson.h"
#include "base64.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_gps.h"
#include <MQTTClient.h>
#include "bridge/ClusterDuckBridge.h"   // Unified bridge C API

/* ClusterDuck Protocol initialization */
#ifdef __cplusplus
extern "C" {
#endif
    extern void* hub_init_and_setup(void);  /* Initialize and setup the PapaDuck hub */
    extern void hub_run_loop(void);         /* Run the hub's main processing loop */
    extern int hub_send_data(uint8_t topic, const char* message, int length, const uint8_t* targetDevice);  /* Send data to MamaDucks */
#ifdef __cplusplus
}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#define STRINGIFY(x)    #x
#define STR(x)          STRINGIFY(x)

#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* CDP default radio parameters (from cdpcfg.h).
   If cdpcfg.h is not available in this build context, use these constants. */
#define CDPCFG_RF_LORA_FREQ_HZ 922800000
#define CDPCFG_RF_LORA_BW 125.0f
#define CDPCFG_RF_LORA_SF 7
#define CDPCFG_RF_LORA_TXPOW 14
#define CDPCFG_RF_LORA_GAIN 0

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
    #define VERSION_STRING "undefined"
#endif

#define JSON_CONF_DEFAULT   "global_conf.json"

#define DEFAULT_SERVER      127.0.0.1   /* hostname also supported */
#define DEFAULT_PORT_UP     1780
#define DEFAULT_PORT_DW     1782
#define DEFAULT_KEEPALIVE   5           /* default time interval for downstream keep-alive packet */
#define DEFAULT_STAT        30          /* default time interval for statistics */
#define PUSH_TIMEOUT_MS     100
#define PULL_TIMEOUT_MS     200
#define GPS_REF_MAX_AGE     30          /* maximum admitted delay in seconds of GPS loss before considering latest GPS sync unusable */
#define FETCH_SLEEP_MS      10          /* nb of ms waited when a fetch return no packets */
#define BEACON_POLL_MS      50          /* time in ms between polling of beacon TX status */

#define PROTOCOL_VERSION    2           /* v1.6 */
#define PROTOCOL_JSON_RXPK_FRAME_FORMAT 1

#define XERR_INIT_AVG       16          /* nb of measurements the XTAL correction is averaged on as initial value */
#define XERR_FILT_COEF      256         /* coefficient for low-pass XTAL error tracking */

#define PKT_PUSH_DATA   0
#define PKT_PUSH_ACK    1
#define PKT_PULL_DATA   2
#define PKT_PULL_RESP   3
#define PKT_PULL_ACK    4
#define PKT_TX_ACK      5

#define NB_PKT_MAX      255 /* max number of packets per fetch/send cycle */

#define MIN_LORA_PREAMB 6 /* minimum Lora preamble length for this application */
#define STD_LORA_PREAMB 8
#define MIN_FSK_PREAMB  3 /* minimum FSK preamble length for this application */
#define STD_FSK_PREAMB  5

#define STATUS_SIZE     200
#define TX_BUFF_SIZE    ((540 * NB_PKT_MAX) + 30 + STATUS_SIZE)
#define ACK_BUFF_SIZE   64

#define UNIX_GPS_EPOCH_OFFSET 315964800 /* Number of seconds ellapsed between 01.Jan.1970 00:00:00
                                                                          and 06.Jan.1980 00:00:00 */

#define DEFAULT_BEACON_FREQ_HZ      869525000
#define DEFAULT_BEACON_FREQ_NB      1
#define DEFAULT_BEACON_FREQ_STEP    0
#define DEFAULT_BEACON_DATARATE     9
#define DEFAULT_BEACON_BW_HZ        125000
#define DEFAULT_BEACON_POWER        14
#define DEFAULT_BEACON_INFODESC     0

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* spectral scan */
typedef struct spectral_scan_s {
    bool enable;            /* enable spectral scan thread */
    uint32_t freq_hz_start; /* first channel frequency, in Hz */
    uint8_t nb_chan;        /* number of channels to scan (200kHz between each channel) */
    uint16_t nb_scan;       /* number of scan points for each frequency scan */
    uint32_t pace_s;        /* number of seconds between 2 scans in the thread */
} spectral_scan_t;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
volatile bool exit_sig = false; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
volatile bool quit_sig = false; /* 1 -> application terminates without shutting down the hardware */

/* packets filtering configuration variables */
static bool fwd_valid_pkt = true; /* packets with PAYLOAD CRC OK are forwarded */
static bool fwd_error_pkt = false; /* packets with PAYLOAD CRC ERROR are NOT forwarded */
static bool fwd_nocrc_pkt = false; /* packets with NO PAYLOAD CRC are NOT forwarded */

/* network configuration variables */
static uint64_t lgwm = 0; /* Lora gateway MAC address */
static char serv_addr[64] = STR(DEFAULT_SERVER); /* address of the server (host name or IPv4/IPv6) */
static char serv_port_up[8] = STR(DEFAULT_PORT_UP); /* server port for upstream traffic */
static char serv_port_down[8] = STR(DEFAULT_PORT_DW); /* server port for downstream traffic */
static int keepalive_time = DEFAULT_KEEPALIVE; /* send a PULL_DATA request every X seconds, negative = disabled */

/* statistics collection configuration variables */
static unsigned stat_interval = DEFAULT_STAT; /* time interval (in sec) at which statistics are collected and displayed */

/* MQTT library */
static MQTTClient mqtt_client = NULL;
static volatile bool mqtt_needs_reconnect = false; /* Flag to indicate client needs reconnection */

/* MQTT configuration variables */
static bool mqtt_enabled = false; /* MQTT publishing enabled */
static char mqtt_server[128] = ""; /* MQTT broker address */
static int mqtt_port = 1883; /* MQTT broker port */
static char mqtt_client_id[64] = "papa-duck-gateway-1"; /* MQTT client ID */
static char mqtt_username[64] = ""; /* MQTT username */
static char mqtt_password[64] = ""; /* MQTT password */
static char mqtt_pub_topic[128] = "hub/event"; /* MQTT publish topic */
static char mqtt_sub_topic[128] = "hub/command"; /* MQTT subscribe topic for incoming commands */
static char mqtt_response_topic[128] = "hub/response"; /* MQTT topic for command responses and status */
static int mqtt_keepalive = 60; /* MQTT keepalive interval */
static bool mqtt_use_tls = false; /* Use TLS for MQTT */
static char mqtt_ca_cert_file[256] = ""; /* CA certificate file path */

/* MQTT reconnection state */
static time_t mqtt_last_reconnect_attempt = 0;
static int mqtt_reconnect_delay = 5; /* Initial reconnection delay in seconds */
static int mqtt_max_reconnect_delay = 300; /* Maximum reconnection delay (5 minutes) */
static int mqtt_reconnect_attempts = 0;

/* MQTT message queue for offline buffering */
#define MQTT_QUEUE_SIZE 100 /* Maximum number of messages to queue when disconnected */
typedef struct {
    char topic[128];
    char* message;
    int length;
} mqtt_queued_message_t;

static mqtt_queued_message_t mqtt_message_queue[MQTT_QUEUE_SIZE];
static int mqtt_queue_head = 0;
static int mqtt_queue_tail = 0;
static int mqtt_queue_count = 0;
static pthread_mutex_t mqtt_queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mqtt_reconnect_mutex = PTHREAD_MUTEX_INITIALIZER;  /* prevent parallel reconnection attempts */

/* Command deduplication: rate-limit identical downlink commands */
#define CMD_DEDUP_WINDOW_MS 30000  /* reject duplicate commands within this window (ms) */
static char   cmd_dedup_target[16]  = "";
static uint8_t cmd_dedup_topic      = 0;
static char   cmd_dedup_message[256] = "";
static struct timespec cmd_dedup_last_time = {0, 0};
static pthread_mutex_t cmd_dedup_mutex = PTHREAD_MUTEX_INITIALIZER;

/* gateway <-> MAC protocol variables */
static uint32_t net_mac_h; /* Most Significant Nibble, network order */
static uint32_t net_mac_l; /* Least Significant Nibble, network order */

/* network sockets */
static int sock_up;   /* socket for upstream traffic → LoRaWAN Network Server */
static int sock_down; /* socket for downstream traffic */

/* network protocol variables */
static struct timeval push_timeout_half = {0, (PUSH_TIMEOUT_MS * 500)}; /* cut in half, critical for throughput */
static struct timeval pull_timeout = {0, (PULL_TIMEOUT_MS * 1000)}; /* non critical for throughput */

/* Dual-mode configuration:
 * When lorawan_fwd_enabled is true, received packets are inspected and
 * classified as either LoRaWAN or CDP.  LoRaWAN packets are forwarded to a
 * Network Server (Chirpstack / TTN) via the standard Semtech UDP PUSH_DATA
 * protocol on sock_up.  CDP packets are forwarded to the ClusterDuck bridge.
 * Both paths share the same concentrator hardware (lgw_receive). */
static bool lorawan_fwd_enabled = false; /* set via "lorawan_forward_enabled" in global_conf.json */

/* hardware access control and correction */
pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */
static pthread_mutex_t mx_xcorr = PTHREAD_MUTEX_INITIALIZER; /* control access to the XTAL correction */
static bool xtal_correct_ok = false; /* set true when XTAL correction is stable enough */
static double xtal_correct = 1.0;

/* GPS configuration and synchronization */
static char gps_tty_path[64] = "\0"; /* path of the TTY port GPS is connected on */
static int gps_tty_fd = -1; /* file descriptor of the GPS TTY port */
static bool gps_enabled = false; /* is GPS enabled on that gateway ? */

/* GPS time reference */
static pthread_mutex_t mx_timeref = PTHREAD_MUTEX_INITIALIZER; /* control access to GPS time reference */
static bool gps_ref_valid; /* is GPS reference acceptable (ie. not too old) */
static struct tref time_reference_gps; /* time reference used for GPS <-> timestamp conversion */

/* Reference coordinates, for broadcasting (beacon) */
static struct coord_s reference_coord;

/* Enable faking the GPS coordinates of the gateway */
static bool gps_fake_enable; /* enable the feature */

/* measurements to establish statistics */
static pthread_mutex_t mx_meas_up = PTHREAD_MUTEX_INITIALIZER; /* control access to the upstream measurements */
static uint32_t meas_nb_rx_rcv = 0; /* count packets received */
static uint32_t meas_nb_rx_ok = 0; /* count packets received with PAYLOAD CRC OK */
static uint32_t meas_nb_rx_bad = 0; /* count packets received with PAYLOAD CRC ERROR */
static uint32_t meas_nb_rx_nocrc = 0; /* count packets received with NO PAYLOAD CRC */
static uint32_t meas_up_pkt_fwd = 0; /* number of radio packet forwarded to the server */
static uint32_t meas_up_network_byte = 0; /* sum of UDP bytes sent for upstream traffic */
static uint32_t meas_up_payload_byte = 0; /* sum of radio payload bytes sent for upstream traffic */
static uint32_t meas_up_dgram_sent = 0; /* number of datagrams sent for upstream traffic */
static uint32_t meas_up_ack_rcv = 0; /* number of datagrams acknowledged for upstream traffic */

static pthread_mutex_t mx_meas_dw = PTHREAD_MUTEX_INITIALIZER; /* control access to the downstream measurements */
static uint32_t meas_dw_pull_sent = 0; /* number of PULL requests sent for downstream traffic */
static uint32_t meas_dw_ack_rcv = 0; /* number of PULL requests acknowledged for downstream traffic */
static uint32_t meas_dw_dgram_rcv = 0; /* count PULL response packets received for downstream traffic */
static uint32_t meas_dw_network_byte = 0; /* sum of UDP bytes sent for upstream traffic */
static uint32_t meas_dw_payload_byte = 0; /* sum of radio payload bytes sent for upstream traffic */
static uint32_t meas_nb_tx_ok = 0; /* count packets emitted successfully */
static uint32_t meas_nb_tx_fail = 0; /* count packets were TX failed for other reasons */
static uint32_t meas_nb_tx_requested = 0; /* count TX request from server (downlinks) */
static uint32_t meas_nb_tx_rejected_collision_packet = 0; /* count packets were TX request were rejected due to collision with another packet already programmed */
static uint32_t meas_nb_tx_rejected_collision_beacon = 0; /* count packets were TX request were rejected due to collision with a beacon already programmed */
static uint32_t meas_nb_tx_rejected_too_late = 0; /* count packets were TX request were rejected because it is too late to program it */
static uint32_t meas_nb_tx_rejected_too_early = 0; /* count packets were TX request were rejected because timestamp is too much in advance */
static uint32_t meas_nb_beacon_queued = 0; /* count beacon inserted in jit queue */
static uint32_t meas_nb_beacon_sent = 0; /* count beacon actually sent to concentrator */
static uint32_t meas_nb_beacon_rejected = 0; /* count beacon rejected for queuing */

static pthread_mutex_t mx_meas_gps = PTHREAD_MUTEX_INITIALIZER; /* control access to the GPS statistics */
static bool gps_coord_valid; /* could we get valid GPS coordinates ? */
static struct coord_s meas_gps_coord; /* GPS position of the gateway */
static struct coord_s meas_gps_err; /* GPS position of the gateway */

static pthread_mutex_t mx_stat_rep = PTHREAD_MUTEX_INITIALIZER; /* control access to the status report */
static bool report_ready = false; /* true when there is a new report to send to the server */
static char status_report[STATUS_SIZE]; /* status report as a JSON object */

/* beacon parameters */
static uint32_t beacon_period = 0; /* set beaconing period, must be a sub-multiple of 86400, the nb of sec in a day */
static uint32_t beacon_freq_hz = DEFAULT_BEACON_FREQ_HZ; /* set beacon TX frequency, in Hz */
static uint8_t beacon_freq_nb = DEFAULT_BEACON_FREQ_NB; /* set number of beaconing channels beacon */
static uint32_t beacon_freq_step = DEFAULT_BEACON_FREQ_STEP; /* set frequency step between beacon channels, in Hz */
static uint8_t beacon_datarate = DEFAULT_BEACON_DATARATE; /* set beacon datarate (SF) */
static uint32_t beacon_bw_hz = DEFAULT_BEACON_BW_HZ; /* set beacon bandwidth, in Hz */
static int8_t beacon_power = DEFAULT_BEACON_POWER; /* set beacon TX power, in dBm */
static uint8_t beacon_infodesc = DEFAULT_BEACON_INFODESC; /* set beacon information descriptor */

/* auto-quit function */
static uint32_t autoquit_threshold = 0; /* enable auto-quit after a number of non-acknowledged PULL_DATA (0 = disabled)*/

/* Just In Time TX scheduling */
static struct jit_queue_s jit_queue[LGW_RF_CHAIN_NB];

/* Gateway specificities */
static int8_t antenna_gain = 0;

/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut[LGW_RF_CHAIN_NB]; /* TX gain table */
static uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
static uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */
static bool tx_enable[LGW_RF_CHAIN_NB] = {false}; /* Is TX enabled for a given RF chain ? */

static uint32_t nb_pkt_log[LGW_IF_CHAIN_NB][8]; /* [CH][SF] */
static uint32_t nb_pkt_received_lora = 0;
static uint32_t nb_pkt_received_fsk = 0;

static struct lgw_conf_debug_s debugconf;
static uint32_t nb_pkt_received_ref[16];

/* Interface type */
static lgw_com_type_t com_type = LGW_COM_SPI;

/* Spectral Scan */
static spectral_scan_t spectral_scan_params = {
    .enable = false,
    .freq_hz_start = 0,
    .nb_chan = 0,
    .nb_scan = 0,
    .pace_s = 10
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void usage(void);

static void sig_handler(int sigio);

static int parse_SX130x_configuration(const char * conf_file);

static int parse_gateway_configuration(const char * conf_file);

static int parse_mqtt_configuration(const char * conf_file);

static int parse_debug_configuration(const char * conf_file);

static uint16_t crc16(const uint8_t * data, unsigned size);

static double difftimespec(struct timespec end, struct timespec beginning);

static void gps_process_sync(void);

static void gps_process_coords(void);

static int get_tx_gain_lut_index(uint8_t rf_chain, int8_t rf_power, uint8_t * lut_index);
static int send_tx_ack(uint8_t token_h, uint8_t token_l, enum jit_error_e error, int32_t error_value);

/* MQTT function declarations */
static int mqtt_queue_message(const char* topic, const char* message, int length);
static int mqtt_publish_queued_messages(void);
static void mqtt_publish_response(const char* message, int length);

/* MQTT Message Arrival Callback - handles incoming commands */
int mqtt_message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    (void)context;
    (void)topicLen;
    
    MSG("[MQTT] Received command on topic: %s\n", topicName);
    MSG("[MQTT] Message length: %d bytes\n", message->payloadlen);
    
    if (message->payloadlen > 0 && message->payload != NULL) {
        // Parse JSON command format: {"target":"MAMADUCK","topic":20,"message":"LED_ON"}
        // For simplicity, we'll use parson library which is already included
        
        char *payload_str = malloc(message->payloadlen + 1);
        if (payload_str == NULL) {
            MSG("ERROR: [MQTT] Failed to allocate memory for payload\n");
            MQTTClient_freeMessage(&message);
            MQTTClient_free(topicName);
            return 1;
        }
        
        memcpy(payload_str, message->payload, message->payloadlen);
        payload_str[message->payloadlen] = '\0';
        
        MSG("[MQTT] Command payload: %s\n", payload_str);
        
        // Parse JSON
        JSON_Value *root_value = json_parse_string(payload_str);
        if (root_value == NULL) {
            MSG("WARNING: [MQTT] Failed to parse JSON command\n");
            free(payload_str);
            MQTTClient_freeMessage(&message);
            MQTTClient_free(topicName);
            return 1;
        }
        
        JSON_Object *root_obj = json_value_get_object(root_value);
        
        // Extract command parameters
        const char *target_str = json_object_get_string(root_obj, "target");
        double topic_num = json_object_get_number(root_obj, "topic");
        const char *cmd_message = json_object_get_string(root_obj, "message");
        
        if (target_str == NULL || topic_num < 20 || cmd_message == NULL) {
            MSG("WARNING: [MQTT] Invalid command format. Required: {\"target\":\"BROADCAST|DEVICEID\",\"topic\":20-255,\"message\":\"text\"}\n");
            json_value_free(root_value);
            free(payload_str);
            MQTTClient_freeMessage(&message);
            MQTTClient_free(topicName);
            return 1;
        }
        
        // Parse target device ID
        uint8_t target_device[8];
        if (strcasecmp(target_str, "BROADCAST") == 0 || strcasecmp(target_str, "ALL") == 0) {
            // Broadcast to all MamaDucks
            memset(target_device, 0xFF, 8);
            MSG("[MQTT] Target: BROADCAST\n");
        } else {
            // Specific device ID (pad with spaces if shorter than 8 chars)
            memset(target_device, 0x20, 8); // Fill with spaces
            size_t len = strlen(target_str);
            if (len > 8) len = 8;
            memcpy(target_device, target_str, len);
            MSG("[MQTT] Target device: %.8s\n", target_device);
        }
        
        // Send command to MamaDucks
        uint8_t topic = (uint8_t)topic_num;
        int cmd_len = strlen(cmd_message);
        
        MSG("[MQTT] Sending command: topic=%d, message='%s', length=%d\n", topic, cmd_message, cmd_len);

        /* --- Command deduplication: drop identical commands within the rate-limit window --- */
        //pthread_mutex_lock(&cmd_dedup_mutex);
        //struct timespec now_ts;
        //clock_gettime(CLOCK_MONOTONIC, &now_ts);
        //long elapsed_ms = (now_ts.tv_sec  - cmd_dedup_last_time.tv_sec)  * 1000L
        //                + (now_ts.tv_nsec - cmd_dedup_last_time.tv_nsec) / 1000000L;
        //bool is_duplicate = (elapsed_ms < CMD_DEDUP_WINDOW_MS)
        //                 && (cmd_dedup_topic == topic)
        //                 && (strncmp(cmd_dedup_target, target_str, sizeof(cmd_dedup_target) - 1) == 0)
        //                 && (strncmp(cmd_dedup_message, cmd_message, sizeof(cmd_dedup_message) - 1) == 0); 
        //if (!is_duplicate) {
        //    /* Record this command as the last seen */
        //    strncpy(cmd_dedup_target,  target_str,  sizeof(cmd_dedup_target)  - 1);
        //    cmd_dedup_target[sizeof(cmd_dedup_target) - 1] = '\0';
        //    strncpy(cmd_dedup_message, cmd_message, sizeof(cmd_dedup_message) - 1);
        //    cmd_dedup_message[sizeof(cmd_dedup_message) - 1] = '\0';
        //    cmd_dedup_topic     = topic;
        //    cmd_dedup_last_time = now_ts;
        //}
        //pthread_mutex_unlock(&cmd_dedup_mutex);

        //if (is_duplicate) {
        //    MSG("WARNING: [MQTT] Duplicate command dropped (same target/topic/message within %d ms)\n", CMD_DEDUP_WINDOW_MS);
        //    json_value_free(root_value);
        //    free(payload_str);
        //    MQTTClient_freeMessage(&message);
        //    MQTTClient_free(topicName);
        //    return 1;
        //}
        /* --- End deduplication --- */

        int result = hub_send_data(topic, cmd_message, cmd_len, target_device);
        
        // Send acknowledgment response
        char response[512];
        int response_len;
        if (result == 0) {
            MSG("[MQTT] Command forwarded to ClusterDuck network successfully\n");
            response_len = snprintf(response, sizeof(response),
                "{\"status\":\"success\",\"target\":\"%.8s\",\"topic\":%d,\"message\":\"%s\"}",
                target_device, topic, cmd_message);
        } else {
            MSG("ERROR: [MQTT] Failed to forward command to ClusterDuck network: %d\n", result);
            response_len = snprintf(response, sizeof(response),
                "{\"status\":\"error\",\"code\":%d,\"target\":\"%.8s\",\"topic\":%d,\"message\":\"%s\"}",
                result, target_device, topic, cmd_message);
        }
        
        // Queue acknowledgment for async publishing (don't publish from within callback)
        // Publishing from MQTT callback can cause timeouts/deadlocks
        if (response_len > 0 && response_len < (int)sizeof(response)) {
            mqtt_queue_message(mqtt_response_topic, response, response_len);
            MSG("[MQTT] Response queued for async publishing\n");
        }
        
        // Cleanup
        json_value_free(root_value);
        free(payload_str);
    }
    
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

/* MQTT Connection Lost Callback */
void mqtt_connection_lost(void *context, char *cause) {
    (void)context;
    MSG("WARNING: [MQTT] Connection lost: %s\n", cause ? cause : "unknown reason");
    // The health check in thread_duck will handle reconnection
}

/* threads */
void thread_up(void);
void thread_down(void);
void thread_jit(void);
void thread_gps(void);
void thread_valid(void);
void thread_spectral_scan(void);
void thread_duck(void);  /* ClusterDuck processing */

/* MQTT Initialization and Connection */
int mqtt_init_and_connect(void) {
    if (!mqtt_enabled) {
        return 0; // MQTT disabled, skip
    }
    
    // If client already exists, clean it up first
    if (mqtt_client != NULL) {
        // Mark as needing reconnect to prevent use during cleanup
        mqtt_needs_reconnect = true;
        
        // Only disconnect if actually connected
        if (MQTTClient_isConnected(mqtt_client)) {
            MQTTClient_disconnect(mqtt_client, 100);
        }
        MQTTClient_destroy(&mqtt_client);
        mqtt_client = NULL;
    }
    
    // Create MQTT broker address string
    char broker_address[256];
    snprintf(broker_address, sizeof(broker_address), "tcp://%s:%d", mqtt_server, mqtt_port);
    
    MSG("[MQTT] Creating MQTT client for broker: %s (attempt %d)\n", 
        broker_address, mqtt_reconnect_attempts + 1);
    
    // Create MQTT client
    int rc = MQTTClient_create(&mqtt_client, broker_address, mqtt_client_id,
                                MQTTCLIENT_PERSISTENCE_NONE, NULL);
    if (rc != MQTTCLIENT_SUCCESS) {
        MSG("ERROR: [MQTT] Failed to create MQTT client, return code %d\n", rc);
        mqtt_reconnect_attempts++;
        return -1;
    }
    
    // Set up callbacks for incoming messages and connection loss
    MQTTClient_setCallbacks(mqtt_client, NULL, mqtt_connection_lost, mqtt_message_arrived, NULL);
    MSG("[MQTT] Callbacks registered for incoming messages\n");
    
    // Set up connection options
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.keepAliveInterval = mqtt_keepalive;
    conn_opts.cleansession = 1;
    
    // Set username and password if provided
    if (strlen(mqtt_username) > 0) {
        conn_opts.username = mqtt_username;
        conn_opts.password = mqtt_password;
        MSG("[MQTT] Using username authentication\n");
    }
    
    // TODO: Add TLS/SSL support
    // if (mqtt_use_tls) {
    //     MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
    //     ssl_opts.trustStore = mqtt_ca_cert_file;
    //     conn_opts.ssl = &ssl_opts;
    // }
    
    MSG("[MQTT] Connecting to broker...\n");
    rc = MQTTClient_connect(mqtt_client, &conn_opts);
    if (rc != MQTTCLIENT_SUCCESS) {
        MSG("ERROR: [MQTT] Failed to connect to broker, return code %d\n", rc);
        MQTTClient_destroy(&mqtt_client);
        mqtt_client = NULL;
        mqtt_reconnect_attempts++;
        return -1;
    }
    
    MSG("[MQTT] Successfully connected to broker\n");
    
    // Reset reconnection state on successful connection
    mqtt_reconnect_attempts = 0;
    mqtt_reconnect_delay = 5; // Reset to initial delay
    
    // Subscribe to incoming topic if configured
    if (strlen(mqtt_sub_topic) > 0) {
        MSG("[MQTT] Subscribing to topic: %s\n", mqtt_sub_topic);
        rc = MQTTClient_subscribe(mqtt_client, mqtt_sub_topic, 0);
        if (rc != MQTTCLIENT_SUCCESS) {
            MSG("WARNING: [MQTT] Failed to subscribe to topic %s, return code %d\n", 
                mqtt_sub_topic, rc);
        }
    }
    
    // Publish any queued messages from previous disconnections
    MSG("[MQTT] Checking for queued messages: count=%d\n", mqtt_queue_count);
    if (mqtt_queue_count > 0) {
        MSG("[MQTT] Publishing %d queued message(s) after reconnection\n", mqtt_queue_count);
        mqtt_publish_queued_messages();
    }
    
    return 0;
}

/* MQTT Reconnection with Exponential Backoff */
int mqtt_reconnect_with_backoff(void) {
    pthread_mutex_lock(&mqtt_reconnect_mutex);
    
    time_t now = time(NULL);
    
    // Check if enough time has passed since last reconnection attempt
    if (now - mqtt_last_reconnect_attempt < mqtt_reconnect_delay) {
        pthread_mutex_unlock(&mqtt_reconnect_mutex);
        return -1; // Too soon to retry
    }
    
    // Update last attempt time
    mqtt_last_reconnect_attempt = now;
    
    MSG("[MQTT] Attempting reconnection (delay was %d seconds)...\n", mqtt_reconnect_delay);
    
    // Try to reconnect
    int result = mqtt_init_and_connect();
    
    if (result != 0) {
        // Connection failed, increase backoff delay exponentially
        mqtt_reconnect_delay = mqtt_reconnect_delay * 2;
        if (mqtt_reconnect_delay > mqtt_max_reconnect_delay) {
            mqtt_reconnect_delay = mqtt_max_reconnect_delay;
        }
        MSG("[MQTT] Reconnection failed. Next attempt in %d seconds.\n", mqtt_reconnect_delay);
    } else {
        MSG("[MQTT] Reconnection successful!\n");
        
        // Clear the reconnect flag
        mqtt_needs_reconnect = false;
        
        // Publish any queued messages
        if (mqtt_queue_count > 0) {
            mqtt_publish_queued_messages();
        }
    }
    
    pthread_mutex_unlock(&mqtt_reconnect_mutex);
    
    return result;
}

/* Queue an MQTT message when disconnected */
static int mqtt_queue_message(const char* topic, const char* message, int length) {
    pthread_mutex_lock(&mqtt_queue_mutex);
    
    if (mqtt_queue_count >= MQTT_QUEUE_SIZE) {
        MSG("WARNING: [MQTT] Queue full (%d messages), dropping oldest message\n", MQTT_QUEUE_SIZE);
        
        // Free the oldest message
        if (mqtt_message_queue[mqtt_queue_tail].message != NULL) {
            free(mqtt_message_queue[mqtt_queue_tail].message);
            mqtt_message_queue[mqtt_queue_tail].message = NULL;
        }
        
        // Move tail forward (drop oldest)
        mqtt_queue_tail = (mqtt_queue_tail + 1) % MQTT_QUEUE_SIZE;
        mqtt_queue_count--;
    }
    
    // Add message to queue
    strncpy(mqtt_message_queue[mqtt_queue_head].topic, topic, sizeof(mqtt_message_queue[mqtt_queue_head].topic) - 1);
    mqtt_message_queue[mqtt_queue_head].topic[sizeof(mqtt_message_queue[mqtt_queue_head].topic) - 1] = '\0';
    
    mqtt_message_queue[mqtt_queue_head].message = malloc(length + 1);
    if (mqtt_message_queue[mqtt_queue_head].message == NULL) {
        MSG("ERROR: [MQTT] Failed to allocate memory for queued message\n");
        pthread_mutex_unlock(&mqtt_queue_mutex);
        return -1;
    }
    
    memcpy(mqtt_message_queue[mqtt_queue_head].message, message, length);
    mqtt_message_queue[mqtt_queue_head].message[length] = '\0';
    mqtt_message_queue[mqtt_queue_head].length = length;
    
    mqtt_queue_head = (mqtt_queue_head + 1) % MQTT_QUEUE_SIZE;
    mqtt_queue_count++;
    
    MSG("[MQTT] Message queued (%d/%d in queue)\n", mqtt_queue_count, MQTT_QUEUE_SIZE);
    
    pthread_mutex_unlock(&mqtt_queue_mutex);
    return 0;
}

/* Publish all queued messages */
static int mqtt_publish_queued_messages(void) {
    int published = 0;
    
    MSG("[MQTT] Publishing %d queued message(s)\n", mqtt_queue_count);
    
    while (1) {
        /* Lock only long enough to dequeue one message */
        pthread_mutex_lock(&mqtt_queue_mutex);
        if (mqtt_queue_count == 0) {
            pthread_mutex_unlock(&mqtt_queue_mutex);
            break;
        }

        /* Copy message out of queue */
        mqtt_queued_message_t* msg = &mqtt_message_queue[mqtt_queue_tail];
        char* topic_copy = strdup(msg->topic);
        char* payload_copy = (char*)malloc(msg->length);
        int length_copy = msg->length;
        if (payload_copy) {
            memcpy(payload_copy, msg->message, msg->length);
        }

        /* Free original and advance queue */
        free(msg->message);
        msg->message = NULL;
        mqtt_queue_tail = (mqtt_queue_tail + 1) % MQTT_QUEUE_SIZE;
        mqtt_queue_count--;
        pthread_mutex_unlock(&mqtt_queue_mutex);

        if (!payload_copy || !topic_copy) {
            free(topic_copy);
            free(payload_copy);
            continue;
        }

        /* Publish without holding the queue mutex */
        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        pubmsg.payload = payload_copy;
        pubmsg.payloadlen = length_copy;
        pubmsg.qos = 1;
        pubmsg.retained = 0;

        MQTTClient_deliveryToken token;
        int rc = MQTTClient_publishMessage(mqtt_client, topic_copy, &pubmsg, &token);

        if (rc != MQTTCLIENT_SUCCESS) {
            MSG("ERROR: [MQTT] Failed to publish queued message, return code %d\n", rc);
            free(topic_copy);
            free(payload_copy);
            return published;
        }

        /* Wait for delivery with timeout - mutex is NOT held here */
        rc = MQTTClient_waitForCompletion(mqtt_client, token, 5000L);
        if (rc != MQTTCLIENT_SUCCESS) {
            MSG("WARNING: [MQTT] Queued message delivery timeout after 5 seconds\n");
        }

        free(topic_copy);
        free(payload_copy);
        published++;
    }
    
    MSG("[MQTT] Successfully published %d queued message(s)\n", published);
    return published;
}


/* MQTT Publishing function */
void mqtt_publish_message(const char* topic, const char* message, int length) {
    if (!mqtt_enabled) {
        return; // MQTT disabled, skip
    }
    
    // Use configured topic if empty topic provided
    const char* pub_topic = (topic && strlen(topic) > 0) ? topic : mqtt_pub_topic;
    
    // Lock for thread-safe access to mqtt_client
    pthread_mutex_lock(&mqtt_reconnect_mutex);
    
    // Check if client is connected and doesn't need reconnection
    if (mqtt_client == NULL || mqtt_needs_reconnect || !MQTTClient_isConnected(mqtt_client)) {
        MSG("WARNING: [MQTT] Client disconnected, queueing message\n");
        
        // Queue the message for later
        mqtt_queue_message(pub_topic, message, length);
        
        pthread_mutex_unlock(&mqtt_reconnect_mutex);
        
        // Try to reconnect with backoff (this will acquire the mutex again)
        mqtt_reconnect_with_backoff();
        
        return;
    }
    
    MSG("[MQTT] Publishing to topic: %s\n", pub_topic);
    MSG("[MQTT] Message (%d bytes): %s\n", length, message);
    
    // Publish message
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = (void*)message;
    pubmsg.payloadlen = length;
    pubmsg.qos = 1;
    pubmsg.retained = 0;
    
    MQTTClient_deliveryToken token;
    int rc = MQTTClient_publishMessage(mqtt_client, pub_topic, &pubmsg, &token);
    if (rc != MQTTCLIENT_SUCCESS) {
        MSG("ERROR: [MQTT] Failed to publish message, return code %d. Queueing message.\n", rc);
        mqtt_queue_message(pub_topic, message, length);
        // Mark client as needing reconnection
        mqtt_needs_reconnect = true;
        pthread_mutex_unlock(&mqtt_reconnect_mutex);
        return;
    }
    
    // Wait for message to be delivered (5 second timeout)
    rc = MQTTClient_waitForCompletion(mqtt_client, token, 5000L);
    if (rc != MQTTCLIENT_SUCCESS) {
        MSG("WARNING: [MQTT] Message delivery timeout after 5 seconds. Queueing message and reconnecting.\n");
        mqtt_queue_message(pub_topic, message, length);
        // Mark client as needing reconnection - don't touch the client object
        mqtt_needs_reconnect = true;
        pthread_mutex_unlock(&mqtt_reconnect_mutex);
        // Note: reconnection will be handled by health check in thread_duck
        return;
    } else {
        MSG("[MQTT] Message published successfully\n");
        pthread_mutex_unlock(&mqtt_reconnect_mutex);
    }
}

/* MQTT Response Publishing function - publishes to response topic */
void mqtt_publish_response(const char* message, int length) {
    mqtt_publish_message(mqtt_response_topic, message, length);
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void usage( void )
{
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -h  print this help\n");
    printf(" -c <filename>  use config file other than 'global_conf.json'\n");
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = true;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = true;
    }
    return;
}

static int parse_SX130x_configuration(const char * conf_file) {
    int i, j, number;
    char param_name[32]; /* used to generate variable parameter names */
    const char *str; /* used to store string value from JSON object */
    const char conf_obj_name[] = "SX130x_conf";
    JSON_Value *root_val = NULL;
    JSON_Value *val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_txgain_obj;
    JSON_Object *conf_ts_obj;
    JSON_Object *conf_sx1261_obj = NULL;
    JSON_Object *conf_scan_obj = NULL;
    JSON_Object *conf_lbt_obj = NULL;
    JSON_Object *conf_lbtchan_obj = NULL;
    JSON_Array *conf_txlut_array = NULL;
    JSON_Array *conf_lbtchan_array = NULL;
    JSON_Array *conf_demod_array = NULL;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    struct lgw_conf_demod_s demodconf;
    struct lgw_conf_ftime_s tsconf;
    struct lgw_conf_sx1261_s sx1261conf;
    uint32_t sf, bw, fdev;
    bool sx1250_tx_lut;
    size_t size;

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing SX1302 parameters\n", conf_file, conf_obj_name);
    }

    /* set board configuration */
    memset(&boardconf, 0, sizeof boardconf); /* initialize configuration structure */
    str = json_object_get_string(conf_obj, "com_type");
    if (str == NULL) {
        MSG("ERROR: com_type must be configured in %s\n", conf_file);
        return -1;
    } else if (!strncmp(str, "SPI", 3) || !strncmp(str, "spi", 3)) {
        boardconf.com_type = LGW_COM_SPI;
    } else if (!strncmp(str, "USB", 3) || !strncmp(str, "usb", 3)) {
        boardconf.com_type = LGW_COM_USB;
    } else {
        MSG("ERROR: invalid com type: %s (should be SPI or USB)\n", str);
        return -1;
    }
    com_type = boardconf.com_type;
    str = json_object_get_string(conf_obj, "com_path");
    if (str != NULL) {
        strncpy(boardconf.com_path, str, sizeof boardconf.com_path);
        boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    } else {
        MSG("ERROR: com_path must be configured in %s\n", conf_file);
        return -1;
    }
    val = json_object_get_value(conf_obj, "lorawan_public"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    } else {
        MSG("WARNING: Data type for lorawan_public seems wrong, please check\n");
        boardconf.lorawan_public = false;
    }
    val = json_object_get_value(conf_obj, "clksrc"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONNumber) {
        boardconf.clksrc = (uint8_t)json_value_get_number(val);
    } else {
        MSG("WARNING: Data type for clksrc seems wrong, please check\n");
        boardconf.clksrc = 0;
    }
    val = json_object_get_value(conf_obj, "full_duplex"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.full_duplex = (bool)json_value_get_boolean(val);
    } else {
        MSG("WARNING: Data type for full_duplex seems wrong, please check\n");
        boardconf.full_duplex = false;
    }
    MSG("INFO: com_type %s, com_path %s, lorawan_public %d, clksrc %d, full_duplex %d\n", (boardconf.com_type == LGW_COM_SPI) ? "SPI" : "USB", boardconf.com_path, boardconf.lorawan_public, boardconf.clksrc, boardconf.full_duplex);
    /* all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        MSG("ERROR: Failed to configure board\n");
        return -1;
    }

    /* set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); /* fetch value (if possible) */
    if (val != NULL) {
        if (json_value_get_type(val) == JSONNumber) {
            antenna_gain = (int8_t)json_value_get_number(val);
        } else {
            MSG("WARNING: Data type for antenna_gain seems wrong, please check\n");
            antenna_gain = 0;
        }
    }
    MSG("INFO: antenna_gain %d dBi\n", antenna_gain);

    /* set timestamp configuration */
    conf_ts_obj = json_object_get_object(conf_obj, "fine_timestamp");
    if (conf_ts_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object for fine timestamp\n", conf_file);
    } else {
        val = json_object_get_value(conf_ts_obj, "enable"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONBoolean) {
            tsconf.enable = (bool)json_value_get_boolean(val);
        } else {
            MSG("WARNING: Data type for fine_timestamp.enable seems wrong, please check\n");
            tsconf.enable = false;
        }
        if (tsconf.enable == true) {
            str = json_object_get_string(conf_ts_obj, "mode");
            if (str == NULL) {
                MSG("ERROR: fine_timestamp.mode must be configured in %s\n", conf_file);
                return -1;
            } else if (!strncmp(str, "high_capacity", 13) || !strncmp(str, "HIGH_CAPACITY", 13)) {
                tsconf.mode = LGW_FTIME_MODE_HIGH_CAPACITY;
            } else if (!strncmp(str, "all_sf", 6) || !strncmp(str, "ALL_SF", 6)) {
                tsconf.mode = LGW_FTIME_MODE_ALL_SF;
            } else {
                MSG("ERROR: invalid fine timestamp mode: %s (should be high_capacity or all_sf)\n", str);
                return -1;
            }
            MSG("INFO: Configuring precision timestamp with %s mode\n", str);

            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_ftime_setconf(&tsconf) != LGW_HAL_SUCCESS) {
                MSG("ERROR: Failed to configure fine timestamp\n");
                return -1;
            }
        } else {
            MSG("INFO: Configuring legacy timestamp\n");
        }
    }

    /* set SX1261 configuration */
    memset(&sx1261conf, 0, sizeof sx1261conf); /* initialize configuration structure */
    conf_sx1261_obj = json_object_get_object(conf_obj, "sx1261_conf"); /* fetch value (if possible) */
    if (conf_sx1261_obj == NULL) {
        MSG("INFO: no configuration for SX1261\n");
    } else {
        /* Global SX1261 configuration */
        str = json_object_get_string(conf_sx1261_obj, "spi_path");
        if (str != NULL) {
            strncpy(sx1261conf.spi_path, str, sizeof sx1261conf.spi_path);
            sx1261conf.spi_path[sizeof sx1261conf.spi_path - 1] = '\0'; /* ensure string termination */
        } else {
            MSG("INFO: SX1261 spi_path is not configured in %s\n", conf_file);
        }
        val = json_object_get_value(conf_sx1261_obj, "rssi_offset"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONNumber) {
            sx1261conf.rssi_offset = (int8_t)json_value_get_number(val);
        } else {
            MSG("WARNING: Data type for sx1261_conf.rssi_offset seems wrong, please check\n");
            sx1261conf.rssi_offset = 0;
        }

        /* Spectral Scan configuration */
        conf_scan_obj = json_object_get_object(conf_sx1261_obj, "spectral_scan"); /* fetch value (if possible) */
        if (conf_scan_obj == NULL) {
            MSG("INFO: no configuration for Spectral Scan\n");
        } else {
            val = json_object_get_value(conf_scan_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                /* Enable background spectral scan thread in packet forwarder */
                spectral_scan_params.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("WARNING: Data type for spectral_scan.enable seems wrong, please check\n");
            }
            if (spectral_scan_params.enable == true) {
                /* Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG("INFO: Spectral Scan with SX1261 is enabled\n");

                /* Get Spectral Scan Parameters */
                val = json_object_get_value(conf_scan_obj, "freq_start"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.freq_hz_start = (uint32_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.freq_start seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_chan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_chan = (uint8_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.nb_chan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_scan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_scan = (uint16_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.nb_scan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "pace_s"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.pace_s = (uint32_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.pace_s seems wrong, please check\n");
                }
            }
        }

        /* LBT configuration */
        conf_lbt_obj = json_object_get_object(conf_sx1261_obj, "lbt"); /* fetch value (if possible) */
        if (conf_lbt_obj == NULL) {
            MSG("INFO: no configuration for LBT\n");
        } else {
            val = json_object_get_value(conf_lbt_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                sx1261conf.lbt_conf.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("WARNING: Data type for lbt.enable seems wrong, please check\n");
            }
            if (sx1261conf.lbt_conf.enable == true) {
                /* Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG("INFO: Listen-Before-Talk with SX1261 is enabled\n");

                val = json_object_get_value(conf_lbt_obj, "rssi_target"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    sx1261conf.lbt_conf.rssi_target = (int8_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for lbt.rssi_target seems wrong, please check\n");
                    sx1261conf.lbt_conf.rssi_target = 0;
                }
                /* set LBT channels configuration */
                conf_lbtchan_array = json_object_get_array(conf_lbt_obj, "channels");
                if (conf_lbtchan_array != NULL) {
                    sx1261conf.lbt_conf.nb_channel = json_array_get_count(conf_lbtchan_array);
                    MSG("INFO: %u LBT channels configured\n", sx1261conf.lbt_conf.nb_channel);
                }
                for (i = 0; i < (int)sx1261conf.lbt_conf.nb_channel; i++) {
                    /* Sanity check */
                    if (i >= LGW_LBT_CHANNEL_NB_MAX) {
                        MSG("ERROR: LBT channel %d not supported, skip it\n", i);
                        break;
                    }
                    /* Get LBT channel configuration object from array */
                    conf_lbtchan_obj = json_array_get_object(conf_lbtchan_array, i);

                    /* Channel frequency */
                    val = json_object_dotget_value(conf_lbtchan_obj, "freq_hz"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            sx1261conf.lbt_conf.channels[i].freq_hz = (uint32_t)json_value_get_number(val);
                        } else {
                            MSG("WARNING: Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].freq_hz = 0;
                        }
                    } else {
                        MSG("ERROR: no frequency defined for LBT channel %d\n", i);
                        return -1;
                    }

                    /* Channel bandiwdth */
                    val = json_object_dotget_value(conf_lbtchan_obj, "bandwidth"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            bw = (uint32_t)json_value_get_number(val);
                            switch(bw) {
                                case 500000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_500KHZ; break;
                                case 250000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_250KHZ; break;
                                case 125000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_125KHZ; break;
                                default: sx1261conf.lbt_conf.channels[i].bandwidth = BW_UNDEFINED;
                            }
                        } else {
                            MSG("WARNING: Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].bandwidth = BW_UNDEFINED;
                        }
                    } else {
                        MSG("ERROR: no bandiwdth defined for LBT channel %d\n", i);
                        return -1;
                    }

                    /* Channel scan time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "scan_time_us"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            if ((uint16_t)json_value_get_number(val) == 128) {
                                sx1261conf.lbt_conf.channels[i].scan_time_us = LGW_LBT_SCAN_TIME_128_US;
                            } else if ((uint16_t)json_value_get_number(val) == 5000) {
                                sx1261conf.lbt_conf.channels[i].scan_time_us = LGW_LBT_SCAN_TIME_5000_US;
                            } else {
                                MSG("ERROR: scan time not supported for LBT channel %d, must be 128 or 5000\n", i);
                                return -1;
                            }
                        } else {
                            MSG("WARNING: Data type for lbt.channels[%d].scan_time_us seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].scan_time_us = 0;
                        }
                    } else {
                        MSG("ERROR: no scan_time_us defined for LBT channel %d\n", i);
                        return -1;
                    }

                    /* Channel transmit time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "transmit_time_ms"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = (uint16_t)json_value_get_number(val);
                        } else {
                            MSG("WARNING: Data type for lbt.channels[%d].transmit_time_ms seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = 0;
                        }
                    } else {
                        MSG("ERROR: no transmit_time_ms defined for LBT channel %d\n", i);
                        return -1;
                    }
                }
            }
        }

        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_sx1261_setconf(&sx1261conf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: Failed to configure the SX1261 radio\n");
            return -1;
        }
    }

    /* set configuration for RF chains */
    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        memset(&rfconf, 0, sizeof rfconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for radio %i\n", i);
            continue;
        }
        /* there is an object to configure that radio, let's parse it */
        snprintf(param_name, sizeof param_name, "radio_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            rfconf.enable = (bool)json_value_get_boolean(val);
        } else {
            rfconf.enable = false;
        }
        if (rfconf.enable == false) { /* radio disabled, nothing else to parse */
            MSG("INFO: radio %i disabled\n", i);
        } else  { /* radio enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "radio_%i.freq", i);
            rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_offset", i);
            rfconf.rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_a", i);
            rfconf.rssi_tcomp.coeff_a = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_b", i);
            rfconf.rssi_tcomp.coeff_b = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_c", i);
            rfconf.rssi_tcomp.coeff_c = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_d", i);
            rfconf.rssi_tcomp.coeff_d = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_e", i);
            rfconf.rssi_tcomp.coeff_e = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.type", i);
            str = json_object_dotget_string(conf_obj, param_name);
            if (!strncmp(str, "SX1255", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1255;
            } else if (!strncmp(str, "SX1257", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1257;
            } else if (!strncmp(str, "SX1250", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1250;
            } else {
                MSG("WARNING: invalid radio type: %s (should be SX1255 or SX1257 or SX1250)\n", str);
            }
            snprintf(param_name, sizeof param_name, "radio_%i.single_input_mode", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.single_input_mode = (bool)json_value_get_boolean(val);
            } else {
                rfconf.single_input_mode = false;
            }

            snprintf(param_name, sizeof param_name, "radio_%i.tx_enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.tx_enable = (bool)json_value_get_boolean(val);
                tx_enable[i] = rfconf.tx_enable; /* update global context for later check */
                if (rfconf.tx_enable == true) {
                    /* tx is enabled on this rf chain, we need its frequency range */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_min", i);
                    tx_freq_min[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_max", i);
                    tx_freq_max[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    if ((tx_freq_min[i] == 0) || (tx_freq_max[i] == 0)) {
                        MSG("WARNING: no frequency range specified for TX rf chain %d\n", i);
                    }

                    /* set configuration for tx gains */
                    memset(&txlut[i], 0, sizeof txlut[i]); /* initialize configuration structure */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_gain_lut", i);
                    conf_txlut_array = json_object_dotget_array(conf_obj, param_name);
                    if (conf_txlut_array != NULL) {
                        txlut[i].size = json_array_get_count(conf_txlut_array);
                        /* Detect if we have a sx125x or sx1250 configuration */
                        conf_txgain_obj = json_array_get_object(conf_txlut_array, 0);
                        val = json_object_dotget_value(conf_txgain_obj, "pwr_idx");
                        if (val != NULL) {
                            printf("INFO: Configuring Tx Gain LUT for rf_chain %u with %u indexes for sx1250\n", i, txlut[i].size);
                            sx1250_tx_lut = true;
                        } else {
                            printf("INFO: Configuring Tx Gain LUT for rf_chain %u with %u indexes for sx125x\n", i, txlut[i].size);
                            sx1250_tx_lut = false;
                        }
                        /* Parse the table */
                        for (j = 0; j < (int)txlut[i].size; j++) {
                             /* Sanity check */
                            if (j >= TX_GAIN_LUT_SIZE_MAX) {
                                printf("ERROR: TX Gain LUT [%u] index %d not supported, skip it\n", i, j);
                                break;
                            }
                            /* Get TX gain object from LUT */
                            conf_txgain_obj = json_array_get_object(conf_txlut_array, j);
                            /* rf power */
                            val = json_object_dotget_value(conf_txgain_obj, "rf_power");
                            if (json_value_get_type(val) == JSONNumber) {
                                txlut[i].lut[j].rf_power = (int8_t)json_value_get_number(val);
                            } else {
                                printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "rf_power", j);
                                txlut[i].lut[j].rf_power = 0;
                            }
                            /* PA gain */
                            val = json_object_dotget_value(conf_txgain_obj, "pa_gain");
                            if (json_value_get_type(val) == JSONNumber) {
                                txlut[i].lut[j].pa_gain = (uint8_t)json_value_get_number(val);
                            } else {
                                printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "pa_gain", j);
                                txlut[i].lut[j].pa_gain = 0;
                            }
                            if (sx1250_tx_lut == false) {
                                /* DIG gain */
                                val = json_object_dotget_value(conf_txgain_obj, "dig_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].dig_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "dig_gain", j);
                                    txlut[i].lut[j].dig_gain = 0;
                                }
                                /* DAC gain */
                                val = json_object_dotget_value(conf_txgain_obj, "dac_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].dac_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "dac_gain", j);
                                    txlut[i].lut[j].dac_gain = 3; /* This is the only dac_gain supported for now */
                                }
                                /* MIX gain */
                                val = json_object_dotget_value(conf_txgain_obj, "mix_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].mix_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "mix_gain", j);
                                    txlut[i].lut[j].mix_gain = 0;
                                }
                            } else {
                                /* TODO: rework this, should not be needed for sx1250 */
                                txlut[i].lut[j].mix_gain = 5;

                                /* power index */
                                val = json_object_dotget_value(conf_txgain_obj, "pwr_idx");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].pwr_idx = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "pwr_idx", j);
                                    txlut[i].lut[j].pwr_idx = 0;
                                }
                            }
                        }
                        /* all parameters parsed, submitting configuration to the HAL */
                        if (txlut[i].size > 0) {
                            if (lgw_txgain_setconf(i, &txlut[i]) != LGW_HAL_SUCCESS) {
                                MSG("ERROR: Failed to configure concentrator TX Gain LUT for rf_chain %u\n", i);
                                return -1;
                            }
                        } else {
                            MSG("WARNING: No TX gain LUT defined for rf_chain %u\n", i);
                        }
                    } else {
                        MSG("WARNING: No TX gain LUT defined for rf_chain %u\n", i);
                    }
                }
            } else {
                rfconf.tx_enable = false;
            }
            MSG("INFO: radio %i enabled (type %s), center frequency %u, RSSI offset %f, tx enabled %d, single input mode %d\n", i, str, rfconf.freq_hz, rfconf.rssi_offset, rfconf.tx_enable, rfconf.single_input_mode);
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxrf_setconf(i, &rfconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for radio %i\n", i);
            return -1;
        }
    }

    /* set configuration for demodulators */
    memset(&demodconf, 0, sizeof demodconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_multiSF_All"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for LoRa multi-SF spreading factors enabling\n");
    } else {
        conf_demod_array = json_object_dotget_array(conf_obj, "chan_multiSF_All.spreading_factor_enable");
        if ((conf_demod_array != NULL) && ((size = json_array_get_count(conf_demod_array)) <= LGW_MULTI_NB)) {
            for (i = 0; i < (int)size; i++) {
                number = json_array_get_number(conf_demod_array, i);
                if (number < 5 || number > 12) {
                    MSG("WARNING: failed to parse chan_multiSF_All.spreading_factor_enable (wrong value at idx %d)\n", i);
                    demodconf.multisf_datarate = 0xFF; /* enable all SFs */
                    break;
                } else {
                    /* set corresponding bit in the bitmask SF5 is LSB -> SF12 is MSB */
                    demodconf.multisf_datarate |= (1 << (number - 5));
                }
            }
        } else {
            MSG("WARNING: failed to parse chan_multiSF_All.spreading_factor_enable\n");
            demodconf.multisf_datarate = 0xFF; /* enable all SFs */
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_demod_setconf(&demodconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for demodulation parameters\n");
            return -1;
        }
    }

    /* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for Lora multi-SF channel %i\n", i);
            continue;
        }
        /* there is an object to configure that Lora multi-SF channel, let's parse it */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) { /* Lora multi-SF channel disabled, nothing else to parse */
            MSG("INFO: Lora multi-SF channel %i disabled\n", i);
        } else  { /* Lora multi-SF channel enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
            // TODO: handle individual SF enabling and disabling (spread_factor)
            MSG("INFO: Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 5 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxif_setconf(i, &ifconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for Lora multi-SF channel %i\n", i);
            return -1;
        }
    }

    /* set configuration for Lora standard channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_Lora_std"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for Lora standard channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG("INFO: Lora standard channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.bandwidth");
            switch(bw) {
                case 500000: ifconf.bandwidth = BW_500KHZ; break;
                case 250000: ifconf.bandwidth = BW_250KHZ; break;
                case 125000: ifconf.bandwidth = BW_125KHZ; break;
                default: ifconf.bandwidth = BW_UNDEFINED;
            }
            sf = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.spread_factor");
            switch(sf) {
                case  5: ifconf.datarate = DR_LORA_SF5;  break;
                case  6: ifconf.datarate = DR_LORA_SF6;  break;
                case  7: ifconf.datarate = DR_LORA_SF7;  break;
                case  8: ifconf.datarate = DR_LORA_SF8;  break;
                case  9: ifconf.datarate = DR_LORA_SF9;  break;
                case 10: ifconf.datarate = DR_LORA_SF10; break;
                case 11: ifconf.datarate = DR_LORA_SF11; break;
                case 12: ifconf.datarate = DR_LORA_SF12; break;
                default: ifconf.datarate = DR_UNDEFINED;
            }
            val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_hdr");
            if (json_value_get_type(val) == JSONBoolean) {
                ifconf.implicit_hdr = (bool)json_value_get_boolean(val);
            } else {
                ifconf.implicit_hdr = false;
            }
            if (ifconf.implicit_hdr == true) {
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_payload_length");
                if (json_value_get_type(val) == JSONNumber) {
                    ifconf.implicit_payload_length = (uint8_t)json_value_get_number(val);
                } else {
                    MSG("ERROR: payload length setting is mandatory for implicit header mode\n");
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_crc_en");
                if (json_value_get_type(val) == JSONBoolean) {
                    ifconf.implicit_crc_en = (bool)json_value_get_boolean(val);
                } else {
                    MSG("ERROR: CRC enable setting is mandatory for implicit header mode\n");
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_coderate");
                if (json_value_get_type(val) == JSONNumber) {
                    ifconf.implicit_coderate = (uint8_t)json_value_get_number(val);
                } else {
                    MSG("ERROR: coding rate setting is mandatory for implicit header mode\n");
                    return -1;
                }
            }

            MSG("INFO: Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u, %s\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf, (ifconf.implicit_hdr == true) ? "Implicit header" : "Explicit header");
        }
        if (lgw_rxif_setconf(8, &ifconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for Lora standard channel\n");
            return -1;
        }
    }

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_FSK"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for FSK channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG("INFO: FSK channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_FSK.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.bandwidth");
            fdev = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.freq_deviation");
            ifconf.datarate = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.datarate");

            /* if chan_FSK.bandwidth is set, it has priority over chan_FSK.freq_deviation */
            if ((bw == 0) && (fdev != 0)) {
                bw = 2 * fdev + ifconf.datarate;
            }
            if      (bw == 0)      ifconf.bandwidth = BW_UNDEFINED;
#if 0 /* TODO */
            else if (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
            else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
            else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
            else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
#endif
            else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
            else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
            else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
            else ifconf.bandwidth = BW_UNDEFINED;

            MSG("INFO: FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
        }
        if (lgw_rxif_setconf(9, &ifconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for FSK channel\n");
            return -1;
        }
    }
    json_value_free(root_val);

    return 0;
}

static int parse_gateway_configuration(const char * conf_file) {
    const char conf_obj_name[] = "gateway_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */
    unsigned long long ull = 0;

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj_name);
    }

    /* gateway unique identifier (aka MAC address) (optional)
     * If omitted, main() will fall back to the hardware EUI from lgw_get_eui().
     * When lorawan_forward_enabled is true this value must match the gateway
     * registered in your Network Server (Chirpstack / TTN). */
    str = json_object_get_string(conf_obj, "gateway_id");
    if (str != NULL) {
        sscanf(str, "%llx", &ull);
        lgwm = ull;
        MSG("INFO: gateway MAC address is configured to %016llX\n", ull);
    } else {
        MSG("INFO: gateway_id not set in config, will use concentrator hardware EUI\n");
    }

    /* server hostname or IP address (optional) */
    str = json_object_get_string(conf_obj, "server_address");
    if (str != NULL) {
        strncpy(serv_addr, str, sizeof serv_addr);
        serv_addr[sizeof serv_addr - 1] = '\0'; /* ensure string termination */
        MSG("INFO: server hostname or IP address is configured to \"%s\"\n", serv_addr);
    }

    /* get up and down ports (optional) */
    val = json_object_get_value(conf_obj, "serv_port_up");
    if (val != NULL) {
        snprintf(serv_port_up, sizeof serv_port_up, "%u", (uint16_t)json_value_get_number(val));
        MSG("INFO: upstream port is configured to \"%s\"\n", serv_port_up);
    }
    val = json_object_get_value(conf_obj, "serv_port_down");
    if (val != NULL) {
        snprintf(serv_port_down, sizeof serv_port_down, "%u", (uint16_t)json_value_get_number(val));
        MSG("INFO: downstream port is configured to \"%s\"\n", serv_port_down);
    }

    /* get keep-alive interval (in seconds) for downstream (optional) */
    val = json_object_get_value(conf_obj, "keepalive_interval");
    if (val != NULL) {
        keepalive_time = (int)json_value_get_number(val);
        MSG("INFO: downstream keep-alive interval is configured to %u seconds\n", keepalive_time);
    }

    /* get interval (in seconds) for statistics display (optional) */
    val = json_object_get_value(conf_obj, "stat_interval");
    if (val != NULL) {
        stat_interval = (unsigned)json_value_get_number(val);
        MSG("INFO: statistics display interval is configured to %u seconds\n", stat_interval);
    }

    /* get time-out value (in ms) for upstream datagrams (optional) */
    val = json_object_get_value(conf_obj, "push_timeout_ms");
    if (val != NULL) {
        push_timeout_half.tv_usec = 500 * (long int)json_value_get_number(val);
        MSG("INFO: upstream PUSH_DATA time-out is configured to %u ms\n", (unsigned)(push_timeout_half.tv_usec / 500));
    }

    /* dual-mode: enable LoRaWAN upstream forwarding to a Network Server (optional) */
    val = json_object_get_value(conf_obj, "lorawan_forward_enabled");
    if (json_value_get_type(val) == JSONBoolean) {
        lorawan_fwd_enabled = (bool)json_value_get_boolean(val);
        MSG("INFO: LoRaWAN upstream forwarding is %s\n", lorawan_fwd_enabled ? "ENABLED" : "DISABLED");
    }

    /* packet filtering parameters */
    val = json_object_get_value(conf_obj, "forward_crc_valid");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_valid_pkt = (bool)json_value_get_boolean(val);
    }
    MSG("INFO: packets received with a valid CRC will%s be forwarded\n", (fwd_valid_pkt ? "" : " NOT"));
    val = json_object_get_value(conf_obj, "forward_crc_error");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_error_pkt = (bool)json_value_get_boolean(val);
    }
    MSG("INFO: packets received with a CRC error will%s be forwarded\n", (fwd_error_pkt ? "" : " NOT"));
    val = json_object_get_value(conf_obj, "forward_crc_disabled");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_nocrc_pkt = (bool)json_value_get_boolean(val);
    }
    MSG("INFO: packets received with no CRC will%s be forwarded\n", (fwd_nocrc_pkt ? "" : " NOT"));

    /* GPS module TTY path (optional) */
    str = json_object_get_string(conf_obj, "gps_tty_path");
    if (str != NULL) {
        strncpy(gps_tty_path, str, sizeof gps_tty_path);
        gps_tty_path[sizeof gps_tty_path - 1] = '\0'; /* ensure string termination */
        MSG("INFO: GPS serial port path is configured to \"%s\"\n", gps_tty_path);
    }

    /* get reference coordinates */
    val = json_object_get_value(conf_obj, "ref_latitude");
    if (val != NULL) {
        reference_coord.lat = (double)json_value_get_number(val);
        MSG("INFO: Reference latitude is configured to %f deg\n", reference_coord.lat);
    }
    val = json_object_get_value(conf_obj, "ref_longitude");
    if (val != NULL) {
        reference_coord.lon = (double)json_value_get_number(val);
        MSG("INFO: Reference longitude is configured to %f deg\n", reference_coord.lon);
    }
    val = json_object_get_value(conf_obj, "ref_altitude");
    if (val != NULL) {
        reference_coord.alt = (short)json_value_get_number(val);
        MSG("INFO: Reference altitude is configured to %i meters\n", reference_coord.alt);
    }

    /* Gateway GPS coordinates hardcoding (aka. faking) option */
    val = json_object_get_value(conf_obj, "fake_gps");
    if (json_value_get_type(val) == JSONBoolean) {
        gps_fake_enable = (bool)json_value_get_boolean(val);
        if (gps_fake_enable == true) {
            MSG("INFO: fake GPS is enabled\n");
        } else {
            MSG("INFO: fake GPS is disabled\n");
        }
    }

    /* Beacon signal period (optional) */
    val = json_object_get_value(conf_obj, "beacon_period");
    if (val != NULL) {
        beacon_period = (uint32_t)json_value_get_number(val);
        if ((beacon_period > 0) && (beacon_period < 6)) {
            MSG("ERROR: invalid configuration for Beacon period, must be >= 6s\n");
            return -1;
        } else {
            MSG("INFO: Beaconing period is configured to %u seconds\n", beacon_period);
        }
    }

    /* Beacon TX frequency (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_hz");
    if (val != NULL) {
        beacon_freq_hz = (uint32_t)json_value_get_number(val);
        MSG("INFO: Beaconing signal will be emitted at %u Hz\n", beacon_freq_hz);
    }

    /* Number of beacon channels (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_nb");
    if (val != NULL) {
        beacon_freq_nb = (uint8_t)json_value_get_number(val);
        MSG("INFO: Beaconing channel number is set to %u\n", beacon_freq_nb);
    }

    /* Frequency step between beacon channels (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_step");
    if (val != NULL) {
        beacon_freq_step = (uint32_t)json_value_get_number(val);
        MSG("INFO: Beaconing channel frequency step is set to %uHz\n", beacon_freq_step);
    }

    /* Beacon datarate (optional) */
    val = json_object_get_value(conf_obj, "beacon_datarate");
    if (val != NULL) {
        beacon_datarate = (uint8_t)json_value_get_number(val);
        MSG("INFO: Beaconing datarate is set to SF%d\n", beacon_datarate);
    }

    /* Beacon modulation bandwidth (optional) */
    val = json_object_get_value(conf_obj, "beacon_bw_hz");
    if (val != NULL) {
        beacon_bw_hz = (uint32_t)json_value_get_number(val);
        MSG("INFO: Beaconing modulation bandwidth is set to %dHz\n", beacon_bw_hz);
    }

    /* Beacon TX power (optional) */
    val = json_object_get_value(conf_obj, "beacon_power");
    if (val != NULL) {
        beacon_power = (int8_t)json_value_get_number(val);
        MSG("INFO: Beaconing TX power is set to %ddBm\n", beacon_power);
    }

    /* Beacon information descriptor (optional) */
    val = json_object_get_value(conf_obj, "beacon_infodesc");
    if (val != NULL) {
        beacon_infodesc = (uint8_t)json_value_get_number(val);
        MSG("INFO: Beaconing information descriptor is set to %u\n", beacon_infodesc);
    }

    /* Auto-quit threshold (optional) */
    val = json_object_get_value(conf_obj, "autoquit_threshold");
    if (val != NULL) {
        autoquit_threshold = (uint32_t)json_value_get_number(val);
        MSG("INFO: Auto-quit after %u non-acknowledged PULL_DATA\n", autoquit_threshold);
    }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

static int parse_mqtt_configuration(const char * conf_file) {
    const char conf_obj_name[] = "mqtt_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Value *val = NULL;
    const char *str;

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the MQTT configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s, MQTT disabled\n", conf_file, conf_obj_name);
        json_value_free(root_val);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing MQTT parameters\n", conf_file, conf_obj_name);
    }

    /* MQTT enabled (optional) */
    val = json_object_get_value(conf_obj, "enabled");
    if (json_value_get_type(val) == JSONBoolean) {
        mqtt_enabled = (bool)json_value_get_boolean(val);
        if (mqtt_enabled == true) {
            MSG("INFO: MQTT is enabled\n");
        } else {
            MSG("INFO: MQTT is disabled\n");
            json_value_free(root_val);
            return 0;
        }
    }

    /* MQTT server address (required if enabled) */
    str = json_object_get_string(conf_obj, "server");
    if (str != NULL) {
        strncpy(mqtt_server, str, sizeof mqtt_server);
        mqtt_server[sizeof mqtt_server - 1] = '\0';
        MSG("INFO: MQTT server is configured to \"%s\"\n", mqtt_server);
    }

    /* MQTT port (optional) */
    val = json_object_get_value(conf_obj, "port");
    if (val != NULL) {
        mqtt_port = (int)json_value_get_number(val);
        MSG("INFO: MQTT port is configured to %d\n", mqtt_port);
    }

    /* MQTT client ID (optional) */
    str = json_object_get_string(conf_obj, "client_id");
    if (str != NULL) {
        strncpy(mqtt_client_id, str, sizeof mqtt_client_id);
        mqtt_client_id[sizeof mqtt_client_id - 1] = '\0';
        MSG("INFO: MQTT client ID is configured to \"%s\"\n", mqtt_client_id);
    }

    /* MQTT username (optional) */
    str = json_object_get_string(conf_obj, "username");
    if (str != NULL && strlen(str) > 0) {
        strncpy(mqtt_username, str, sizeof mqtt_username);
        mqtt_username[sizeof mqtt_username - 1] = '\0';
        MSG("INFO: MQTT username is configured\n");
    }

    /* MQTT password (optional) */
    str = json_object_get_string(conf_obj, "password");
    if (str != NULL && strlen(str) > 0) {
        strncpy(mqtt_password, str, sizeof mqtt_password);
        mqtt_password[sizeof mqtt_password - 1] = '\0';
        MSG("INFO: MQTT password is configured\n");
    }

    /* MQTT topics (using new nested format) */
    JSON_Object *topics_obj = json_object_get_object(conf_obj, "topics");
    if (topics_obj != NULL) {
        /* Publish topic */
        str = json_object_get_string(topics_obj, "publish");
        if (str != NULL) {
            strncpy(mqtt_pub_topic, str, sizeof mqtt_pub_topic);
            mqtt_pub_topic[sizeof mqtt_pub_topic - 1] = '\0';
            MSG("INFO: MQTT publish topic is configured to \"%s\"\n", mqtt_pub_topic);
        }
        
        /* Subscribe topic */
        str = json_object_get_string(topics_obj, "subscribe");
        if (str != NULL) {
            strncpy(mqtt_sub_topic, str, sizeof mqtt_sub_topic);
            mqtt_sub_topic[sizeof mqtt_sub_topic - 1] = '\0';
            MSG("INFO: MQTT subscribe topic is configured to \"%s\"\n", mqtt_sub_topic);
        }
        
        /* Response topic */
        str = json_object_get_string(topics_obj, "response");
        if (str != NULL) {
            strncpy(mqtt_response_topic, str, sizeof mqtt_response_topic);
            mqtt_response_topic[sizeof mqtt_response_topic - 1] = '\0';
            MSG("INFO: MQTT response topic is configured to \"%s\"\n", mqtt_response_topic);
        }
    } else {
        MSG("WARNING: No 'topics' object found in mqtt_conf. Using default topics.\n");
    }

    /* MQTT keepalive (optional) */
    val = json_object_get_value(conf_obj, "keepalive");
    if (val != NULL) {
        mqtt_keepalive = (int)json_value_get_number(val);
        MSG("INFO: MQTT keepalive is configured to %d seconds\n", mqtt_keepalive);
    }

    /* MQTT TLS (optional) */
    val = json_object_get_value(conf_obj, "use_tls");
    if (json_value_get_type(val) == JSONBoolean) {
        mqtt_use_tls = (bool)json_value_get_boolean(val);
        MSG("INFO: MQTT TLS is %s\n", mqtt_use_tls ? "enabled" : "disabled");
    }

    /* MQTT CA certificate file (optional) */
    str = json_object_get_string(conf_obj, "ca_cert_file");
    if (str != NULL && strlen(str) > 0) {
        strncpy(mqtt_ca_cert_file, str, sizeof mqtt_ca_cert_file);
        mqtt_ca_cert_file[sizeof mqtt_ca_cert_file - 1] = '\0';
        MSG("INFO: MQTT CA certificate file is configured to \"%s\"\n", mqtt_ca_cert_file);
    }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

static int parse_debug_configuration(const char * conf_file) {
    int i;
    const char conf_obj_name[] = "debug_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Array *conf_array = NULL;
    JSON_Object *conf_obj_array = NULL;
    const char *str; /* pointer to sub-strings in the JSON data */

    /* Initialize structure */
    memset(&debugconf, 0, sizeof debugconf);

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        json_value_free(root_val);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing debug parameters\n", conf_file, conf_obj_name);
    }

    /* Get reference payload configuration */
    conf_array = json_object_get_array (conf_obj, "ref_payload");
    if (conf_array != NULL) {
        debugconf.nb_ref_payload = json_array_get_count(conf_array);
        MSG("INFO: got %u debug reference payload\n", debugconf.nb_ref_payload);

        for (i = 0; i < (int)debugconf.nb_ref_payload; i++) {
            conf_obj_array = json_array_get_object(conf_array, i);
            /* id */
            str = json_object_get_string(conf_obj_array, "id");
            if (str != NULL) {
                sscanf(str, "0x%08X", &(debugconf.ref_payload[i].id));
                MSG("INFO: reference payload ID %d is 0x%08X\n", i, debugconf.ref_payload[i].id);
            }

            /* global count */
            nb_pkt_received_ref[i] = 0;
        }
    }

    /* Get log file configuration */
    str = json_object_get_string(conf_obj, "log_file");
    if (str != NULL) {
        strncpy(debugconf.log_file_name, str, sizeof debugconf.log_file_name);
        debugconf.log_file_name[sizeof debugconf.log_file_name - 1] = '\0'; /* ensure string termination */
        MSG("INFO: setting debug log file name to %s\n", debugconf.log_file_name);
    }

    /* Commit configuration */
    if (lgw_debug_setconf(&debugconf) != LGW_HAL_SUCCESS) {
        MSG("ERROR: Failed to configure debug\n");
        json_value_free(root_val);
        return -1;
    }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

static uint16_t crc16(const uint8_t * data, unsigned size) {
    const uint16_t crc_poly = 0x1021;
    const uint16_t init_val = 0x0000;
    uint16_t x = init_val;
    unsigned i, j;

    if (data == NULL)  {
        return 0;
    }

    for (i=0; i<size; ++i) {
        x ^= (uint16_t)data[i] << 8;
        for (j=0; j<8; ++j) {
            x = (x & 0x8000) ? (x<<1) ^ crc_poly : (x<<1);
        }
    }

    return x;
}

static double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
    int i; /* loop variable and temporary variable for return value */
    int x;
    int l, m;

    /* configuration file related */
    const char defaut_conf_fname[] = JSON_CONF_DEFAULT;
    const char * conf_fname = defaut_conf_fname; /* pointer to a string we won't touch */

    /* threads */
    pthread_t thrid_up;
    pthread_t thrid_down;
    pthread_t thrid_gps;
    pthread_t thrid_valid;
    pthread_t thrid_jit;
    pthread_t thrid_ss;
    pthread_t thrid_duck;  /* ClusterDuck processing thread */

    /* variables to get local copies of measurements */
    uint32_t cp_nb_rx_rcv;
    uint32_t cp_nb_rx_ok;
    uint32_t cp_nb_rx_bad;
    uint32_t cp_nb_rx_nocrc;
    uint32_t cp_up_pkt_fwd;
    uint32_t cp_up_payload_byte;
    uint32_t cp_dw_dgram_rcv;
    uint32_t cp_dw_payload_byte;
    uint32_t cp_nb_tx_ok;
    uint32_t cp_nb_tx_fail;
    uint32_t cp_nb_tx_requested = 0;
    uint32_t cp_nb_tx_rejected_collision_packet = 0;
    uint32_t cp_nb_tx_rejected_collision_beacon = 0;
    uint32_t cp_nb_tx_rejected_too_late = 0;
    uint32_t cp_nb_tx_rejected_too_early = 0;
    uint32_t cp_nb_beacon_queued = 0;
    uint32_t cp_nb_beacon_sent = 0;
    uint32_t cp_nb_beacon_rejected = 0;

    /* GPS coordinates variables */
    bool coord_ok = false;
    struct coord_s cp_gps_coord = {0.0, 0.0, 0};

    /* SX1302 data variables */
    uint32_t trig_tstamp;
    uint32_t inst_tstamp;
    uint64_t eui;
    float temperature;

    /* statistics variable */
    time_t t;
    char stat_timestamp[24];
    float rx_ok_ratio;
    float rx_bad_ratio;
    float rx_nocrc_ratio;

    /* Parse command line options */
    while( (i = getopt( argc, argv, "hc:" )) != -1 )
    {
        switch( i )
        {
        case 'h':
            usage( );
            return EXIT_SUCCESS;
            break;

        case 'c':
            conf_fname = optarg;
            break;

        default:
            printf( "ERROR: argument parsing options, use -h option for help\n" );
            usage( );
            return EXIT_FAILURE;
        }
    }

    /* display version informations */
    MSG("*** ClusterDuck SuperDuck (PapaDuck) ***\nVersion: " VERSION_STRING "\n");
    MSG("*** SX1302 HAL library version info ***\n%s\n***\n", lgw_version_info());

    /* display host endianness */
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        MSG("INFO: Little endian host\n");
    #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        MSG("INFO: Big endian host\n");
    #else
        MSG("INFO: Host endianness unknown\n");
    #endif

    /* load configuration files */
    if (access(conf_fname, R_OK) == 0) { /* if there is a global conf, parse it  */
        MSG("INFO: found configuration file %s, parsing it\n", conf_fname);
        x = parse_SX130x_configuration(conf_fname);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        x = parse_gateway_configuration(conf_fname);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        x = parse_mqtt_configuration(conf_fname);
        if (x != 0) {
            MSG("INFO: no MQTT configuration or MQTT disabled\n");
        } else {
            // Initialize and connect MQTT client if enabled
            x = mqtt_init_and_connect();
            if (x != 0) {
                MSG("WARNING: [main] MQTT initialization failed, continuing without MQTT\n");
                mqtt_enabled = false;
            }
        }
        x = parse_debug_configuration(conf_fname);
        if (x != 0) {
            MSG("INFO: no debug configuration\n");
        }
    } else {
        MSG("ERROR: [main] failed to find any configuration file named %s\n", conf_fname);
        exit(EXIT_FAILURE);
    }

    /* Start GPS a.s.a.p., to allow it to lock */
    if (gps_tty_path[0] != '\0') { /* do not try to open GPS device if no path set */
        i = lgw_gps_enable(gps_tty_path, "ubx7", 0, &gps_tty_fd); /* HAL only supports u-blox 7 for now */
        if (i != LGW_GPS_SUCCESS) {
            printf("WARNING: [main] impossible to open %s for GPS sync (check permissions)\n", gps_tty_path);
            gps_enabled = false;
            gps_ref_valid = false;
        } else {
            printf("INFO: [main] TTY port %s open for GPS synchronization\n", gps_tty_path);
            gps_enabled = true;
            gps_ref_valid = false;
        }
    }

    /* get timezone info */
    tzset();

    /* sanity check on configuration variables */
    // TODO

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh start") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    for (l = 0; l < LGW_IF_CHAIN_NB; l++) {
        for (m = 0; m < 8; m++) {
            nb_pkt_log[l][m] = 0;
        }
    }

    /* starting the concentrator */
    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: [main] concentrator started, packet can now be received\n");
    } else {
        MSG("ERROR: [main] failed to start the concentrator\n");
        exit(EXIT_FAILURE);
    }

    /* -----------------------------------------------------------------------
     * LoRaWAN upstream socket setup (dual-mode only)
     * Open a UDP socket to the configured Network Server so that LoRaWAN
     * packets identified by the upstream classifier can be forwarded via the
     * standard Semtech UDP protocol (PUSH_DATA / PUSH_ACK).
     * lorawan_fwd_enabled is set by parse_gateway_configuration above.
     * --------------------------------------------------------------------- */
    MSG("INFO: [main] LoRaWAN upstream forwarding is %s\n", lorawan_fwd_enabled ? "ENABLED" : "DISABLED");

    if (lorawan_fwd_enabled) {
        struct addrinfo hints_lw, *result_lw = NULL, *q_lw = NULL;
        memset(&hints_lw, 0, sizeof hints_lw);
        hints_lw.ai_family   = AF_INET;
        hints_lw.ai_socktype = SOCK_DGRAM;

        i = getaddrinfo(serv_addr, serv_port_up, &hints_lw, &result_lw);
        if (i != 0) {
            MSG("ERROR: [main] getaddrinfo for LoRaWAN NS %s:%s failed: %s\n", serv_addr, serv_port_up, gai_strerror(i));
            MSG("WARNING: [main] Disabling LoRaWAN forwarding\n");
            lorawan_fwd_enabled = false;
        } else {
            for (q_lw = result_lw; q_lw != NULL; q_lw = q_lw->ai_next) {
                sock_up = socket(q_lw->ai_family, q_lw->ai_socktype, q_lw->ai_protocol);
                if (sock_up == -1) continue;
                else break;
            }
            if (q_lw == NULL) {
                MSG("ERROR: [main] failed to open upstream socket to LoRaWAN NS %s:%s\n", serv_addr, serv_port_up);
                lorawan_fwd_enabled = false;
            } else {
                i = connect(sock_up, q_lw->ai_addr, q_lw->ai_addrlen);
                if (i != 0) {
                    MSG("ERROR: [main] connect to LoRaWAN NS failed: %s\n", strerror(errno));
                    lorawan_fwd_enabled = false;
                } else {
                    MSG("INFO: [main] upstream socket open to LoRaWAN NS %s:%s\n", serv_addr, serv_port_up);
                }
            }
            freeaddrinfo(result_lw);
        }

        /* open downstream socket (PULL_DATA keepalive + PULL_RESP downlinks) */
        if (lorawan_fwd_enabled) {
            struct addrinfo hints_dw, *result_dw = NULL, *q_dw = NULL;
            memset(&hints_dw, 0, sizeof hints_dw);
            hints_dw.ai_family   = AF_INET;
            hints_dw.ai_socktype = SOCK_DGRAM;

            i = getaddrinfo(serv_addr, serv_port_down, &hints_dw, &result_dw);
            if (i != 0) {
                MSG("ERROR: [main] getaddrinfo for downstream NS %s:%s failed: %s\n", serv_addr, serv_port_down, gai_strerror(i));
                MSG("WARNING: [main] Disabling LoRaWAN forwarding (no downstream socket)\n");
                lorawan_fwd_enabled = false;
            } else {
                for (q_dw = result_dw; q_dw != NULL; q_dw = q_dw->ai_next) {
                    sock_down = socket(q_dw->ai_family, q_dw->ai_socktype, q_dw->ai_protocol);
                    if (sock_down == -1) continue;
                    else break;
                }
                if (q_dw == NULL) {
                    MSG("ERROR: [main] failed to open downstream socket to LoRaWAN NS %s:%s\n", serv_addr, serv_port_down);
                    lorawan_fwd_enabled = false;
                } else {
                    i = connect(sock_down, q_dw->ai_addr, q_dw->ai_addrlen);
                    if (i != 0) {
                        MSG("ERROR: [main] connect downstream to LoRaWAN NS failed: %s\n", strerror(errno));
                        lorawan_fwd_enabled = false;
                    } else {
                        MSG("INFO: [main] downstream socket open to LoRaWAN NS %s:%s\n", serv_addr, serv_port_down);
                    }
                }
                freeaddrinfo(result_dw);
            }
        }
    }

    /* Final confirmation: lorawan_fwd_enabled may have been cleared above
     * if DNS resolution or socket connect failed — log the actual state. */
    MSG("INFO: [main] LoRaWAN forwarding FINAL state: %s\n",
        lorawan_fwd_enabled ? "ACTIVE (sockets open)" : "DISABLED (check DNS/socket errors above)");

    /* get the concentrator EUI */
    i = lgw_get_eui(&eui);
    if (i != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to get concentrator EUI\n");
    } else {
        printf("INFO: concentrator EUI: 0x%016" PRIx64 "\n", eui);
        /* If gateway_id was not set in the config (lgwm == 0), fall back to
         * the hardware EUI burned into the SX1302 concentrator.  This ensures
         * PUSH_DATA datagrams always carry a valid, unique gateway identifier
         * without requiring the user to copy the EUI into global_conf.json. */
        if (lgwm == 0) {
            lgwm = eui;
            MSG("INFO: gateway_id not set in config, using concentrator EUI: %016" PRIx64 "\n", lgwm);
        }
    }

    /* Derive the network-order MAC words used in every PUSH_DATA header.
     * Must be computed after lgwm is finalised (config + EUI fallback above). */
    net_mac_h = htonl((uint32_t)(0xFFFFFFFF & (lgwm >> 32)));
    net_mac_l = htonl((uint32_t)(0xFFFFFFFF &  lgwm       ));

    /* Initialize ClusterDuck Protocol */
    MSG("INFO: [main] Initializing ClusterDuck Protocol\n");
    void* hub_ptr = hub_init_and_setup();
    if (hub_ptr == NULL) {
        MSG("ERROR: [main] failed to initialize ClusterDuck Protocol\n");
        lgw_stop();
        exit(EXIT_FAILURE);
    }
    MSG("INFO: [main] ClusterDuck Protocol initialized successfully\n");

    /* spawn threads to manage upstream and downstream */
    i = pthread_create(&thrid_up, NULL, (void * (*)(void *))thread_up, NULL);
    if (i != 0) {
        MSG("ERROR: [main] impossible to create upstream thread\n");
        lgw_stop();
        exit(EXIT_FAILURE);
    }
    /* thread_down and thread_jit are only spawned when LoRaWAN forwarding is
     * enabled.  Without this guard, thread_down's inner recv() loop runs on
     * an uninitialised sock_down (EBADF) and returns immediately every call,
     * burning 100% CPU in a tight busy-loop. */
    if (lorawan_fwd_enabled) {
        i = pthread_create(&thrid_down, NULL, (void * (*)(void *))thread_down, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create downstream thread\n");
            lgw_stop();
            exit(EXIT_FAILURE);
        }
        i = pthread_create(&thrid_jit, NULL, (void * (*)(void *))thread_jit, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create JIT thread\n");
            lgw_stop();
            exit(EXIT_FAILURE);
        }
    }
    i = pthread_create(&thrid_duck, NULL, (void * (*)(void *))thread_duck, NULL);
    if (i != 0) {
        MSG("ERROR: [main] impossible to create ClusterDuck processing thread\n");
        lgw_stop();
        exit(EXIT_FAILURE);
    }

    /* spawn thread for background spectral scan */
    if (spectral_scan_params.enable == true) {
        i = pthread_create(&thrid_ss, NULL, (void * (*)(void *))thread_spectral_scan, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create Spectral Scan thread\n");
            lgw_stop();
            exit(EXIT_FAILURE);
        }
    }

    /* spawn thread to manage GPS */
    if (gps_enabled == true) {
        i = pthread_create(&thrid_gps, NULL, (void * (*)(void *))thread_gps, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create GPS thread\n");
            lgw_stop();
            exit(EXIT_FAILURE);
        }
        i = pthread_create(&thrid_valid, NULL, (void * (*)(void *))thread_valid, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create validation thread\n");
            lgw_stop();
            exit(EXIT_FAILURE);
        }
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL); /* Ctrl-\ */
    sigaction(SIGINT, &sigact, NULL); /* Ctrl-C */
    sigaction(SIGTERM, &sigact, NULL); /* default "kill" command */

    /* main loop task : statistics collection */
    while (!exit_sig && !quit_sig) {
        /* wait for next reporting interval */
        wait_ms(1000 * stat_interval);

        /* get timestamp for statistics */
        t = time(NULL);
        strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));

        /* access upstream statistics, copy and reset them */
        pthread_mutex_lock(&mx_meas_up);
        cp_nb_rx_rcv       = meas_nb_rx_rcv;
        cp_nb_rx_ok        = meas_nb_rx_ok;
        cp_nb_rx_bad       = meas_nb_rx_bad;
        cp_nb_rx_nocrc     = meas_nb_rx_nocrc;
        cp_up_pkt_fwd      = meas_up_pkt_fwd;
        cp_up_payload_byte = meas_up_payload_byte;
        meas_nb_rx_rcv = 0;
        meas_nb_rx_ok = 0;
        meas_nb_rx_bad = 0;
        meas_nb_rx_nocrc = 0;
        meas_up_pkt_fwd = 0;
        meas_up_network_byte = 0;
        meas_up_payload_byte = 0;
        meas_up_dgram_sent = 0;
        meas_up_ack_rcv = 0;
        pthread_mutex_unlock(&mx_meas_up);
        if (cp_nb_rx_rcv > 0) {
            rx_ok_ratio = (float)cp_nb_rx_ok / (float)cp_nb_rx_rcv;
            rx_bad_ratio = (float)cp_nb_rx_bad / (float)cp_nb_rx_rcv;
            rx_nocrc_ratio = (float)cp_nb_rx_nocrc / (float)cp_nb_rx_rcv;
        } else {
            rx_ok_ratio = 0.0;
            rx_bad_ratio = 0.0;
            rx_nocrc_ratio = 0.0;
        }

        /* access downstream statistics, copy and reset them */
        pthread_mutex_lock(&mx_meas_dw);
        cp_dw_dgram_rcv    =  meas_dw_dgram_rcv;
        cp_dw_payload_byte =  meas_dw_payload_byte;
        cp_nb_tx_ok        =  meas_nb_tx_ok;
        cp_nb_tx_fail      =  meas_nb_tx_fail;
        cp_nb_tx_requested                 +=  meas_nb_tx_requested;
        cp_nb_tx_rejected_collision_packet +=  meas_nb_tx_rejected_collision_packet;
        cp_nb_tx_rejected_collision_beacon +=  meas_nb_tx_rejected_collision_beacon;
        cp_nb_tx_rejected_too_late         +=  meas_nb_tx_rejected_too_late;
        cp_nb_tx_rejected_too_early        +=  meas_nb_tx_rejected_too_early;
        cp_nb_beacon_queued   +=  meas_nb_beacon_queued;
        cp_nb_beacon_sent     +=  meas_nb_beacon_sent;
        cp_nb_beacon_rejected +=  meas_nb_beacon_rejected;
        meas_dw_pull_sent = 0;
        meas_dw_ack_rcv = 0;
        meas_dw_dgram_rcv = 0;
        meas_dw_network_byte = 0;
        meas_dw_payload_byte = 0;
        meas_nb_tx_ok = 0;
        meas_nb_tx_fail = 0;
        meas_nb_tx_requested = 0;
        meas_nb_tx_rejected_collision_packet = 0;
        meas_nb_tx_rejected_collision_beacon = 0;
        meas_nb_tx_rejected_too_late = 0;
        meas_nb_tx_rejected_too_early = 0;
        meas_nb_beacon_queued = 0;
        meas_nb_beacon_sent = 0;
        meas_nb_beacon_rejected = 0;
        pthread_mutex_unlock(&mx_meas_dw);

        /* access GPS statistics, copy them */
        if (gps_enabled == true) {
            pthread_mutex_lock(&mx_meas_gps);
            coord_ok = gps_coord_valid;
            cp_gps_coord = meas_gps_coord;
            pthread_mutex_unlock(&mx_meas_gps);
        }

        /* overwrite with reference coordinates if function is enabled */
        if (gps_fake_enable == true) {
            cp_gps_coord = reference_coord;
        }

        /* print periodic status report to console */
        printf("##### %s #####\n", stat_timestamp);
        printf("### [UPSTREAM] ###\n");
        printf("# RF packets received by concentrator: %u\n", cp_nb_rx_rcv);
        printf("# CRC_OK: %.2f%%, CRC_FAIL: %.2f%%, NO_CRC: %.2f%%\n",
               100.0 * rx_ok_ratio, 100.0 * rx_bad_ratio, 100.0 * rx_nocrc_ratio);
        if (lorawan_fwd_enabled) {
            printf("# RF packets forwarded to NS: %u (%u bytes)\n", cp_up_pkt_fwd, cp_up_payload_byte);
        }
        printf("### [DOWNSTREAM] ###\n");
        printf("# Downlinks received from NS: %u\n", cp_dw_dgram_rcv);
        printf("# Packets emitted successfully: %u\n", cp_nb_tx_ok);
        printf("# Packets failed to emit: %u\n", cp_nb_tx_fail);
        if (cp_nb_tx_requested > 0) {
            printf("# TX requested: %u, rejected (collision): %u, too late: %u, too early: %u\n",
                   cp_nb_tx_requested,
                   cp_nb_tx_rejected_collision_packet + cp_nb_tx_rejected_collision_beacon,
                   cp_nb_tx_rejected_too_late,
                   cp_nb_tx_rejected_too_early);
        }
        if ((gps_enabled == true) && (coord_ok == true)) {
            printf("### [GPS] lat=%.5f lon=%.5f alt=%d\n",
                   cp_gps_coord.lat, cp_gps_coord.lon, cp_gps_coord.alt);
        }
        printf("##### END #####\n");
        fflush(stdout);
    }

    /* wait for all threads with a COM with the concentrator board to finish (1 fetch cycle max) */
    i = pthread_join(thrid_up, NULL);
    if (i != 0) {
        printf("ERROR: failed to join upstream thread with %d - %s\n", i, strerror(errno));
    }
    i = pthread_join(thrid_down, NULL);
    if (i != 0) {
        printf("ERROR: failed to join downstream thread with %d - %s\n", i, strerror(errno));
    }
    i = pthread_join(thrid_jit, NULL);
    if (i != 0) {
        printf("ERROR: failed to join JIT thread with %d - %s\n", i, strerror(errno));
    }
    if (spectral_scan_params.enable == true) {
        i = pthread_join(thrid_ss, NULL);
        if (i != 0) {
            printf("ERROR: failed to join Spectral Scan thread with %d - %s\n", i, strerror(errno));
        }
    }

    if (gps_enabled == true) {
        pthread_cancel(thrid_gps); /* don't wait for GPS thread, no access to concentrator board */
        pthread_cancel(thrid_valid); /* don't wait for validation thread, no access to concentrator board */

        i = lgw_gps_disable(gps_tty_fd);
        if (i == LGW_HAL_SUCCESS) {
            MSG("INFO: GPS closed successfully\n");
        } else {
            MSG("WARNING: failed to close GPS successfully\n");
        }
    }

    /* if an exit signal was received, try to quit properly */
    if (exit_sig) {
        /* disconnect MQTT client */
        if (mqtt_enabled && mqtt_client != NULL) {
            MSG("INFO: Disconnecting MQTT client...\n");
            
            // Clean up message queue
            pthread_mutex_lock(&mqtt_queue_mutex);
            for (int i = 0; i < MQTT_QUEUE_SIZE; i++) {
                if (mqtt_message_queue[i].message != NULL) {
                    free(mqtt_message_queue[i].message);
                    mqtt_message_queue[i].message = NULL;
                }
            }
            mqtt_queue_count = 0;
            pthread_mutex_unlock(&mqtt_queue_mutex);
            
            MQTTClient_disconnect(mqtt_client, 1000);
            MQTTClient_destroy(&mqtt_client);
            MSG("INFO: MQTT client disconnected\n");
        }
        
        /* stop the hardware */
        i = lgw_stop();
        if (i == LGW_HAL_SUCCESS) {
            MSG("INFO: concentrator stopped successfully\n");
        } else {
            MSG("WARNING: failed to stop concentrator successfully\n");
        }
    }

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh stop") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    MSG("INFO: Exiting clusterduckd program\n");
    exit(EXIT_SUCCESS);
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 1: RECEIVING PACKETS AND FORWARDING THEM ---------------------- */

/* ---------------------------------------------------------------------------
 * is_lorawan_packet - packet classifier
 *
 * Returns true if the packet should be forwarded as LoRaWAN to the NS.
 * Returns false if it looks like a CDP (ClusterDuck Protocol) packet.
 *
 * Strategy: positively identify CDP first, then assume LoRaWAN for anything
 * that doesn't match.  This is more robust than the inverse because:
 *
 *   - The old approach checked the LoRaWAN MHDR byte (byte 0).  But CDP
 *     byte 0 is SDUID[0] — a hardware-derived device ID byte that can
 *     coincidentally satisfy the LoRaWAN MHDR bit-field constraints
 *     (bits[4:0]==0, MType!=RFU), causing CDP packets to be misrouted to
 *     the Network Server.
 *
 *   - CDP has tight, independently verifiable structural invariants:
 *       · total size  >=  MIN_PACKET_LENGTH (28 bytes = 27-byte header + 1)
 *       · byte 21 (Duck Type) in range [UNKNOWN=0 .. MAX_TYPE-1=4]
 *       · byte 22 (Hop Count) in range [0 .. MAX_HOPS=6]
 *     A LoRaWAN frame is extremely unlikely to satisfy all three
 *     simultaneously at the same byte offsets.
 *
 *   - LoRaWAN JoinRequest (23 bytes) fails the CDP size floor → LoRaWAN ✓
 *   - LoRaWAN data frames are >= 13 bytes but have FHDR/FCnt at bytes 21-22
 *     which rarely both land inside duck-type and hop-count ranges together.
 * --------------------------------------------------------------------------- */
static bool is_lorawan_packet(const uint8_t *payload, uint16_t size) {
    /* Must be at least the CDP minimum to even consider it CDP.
     * MIN_PACKET_LENGTH = HEADER_LENGTH(27) + 1 = 28  (CdpPacket.h) */
    if (size >= 28) {
        uint8_t duck_type = payload[21]; /* DUCK_TYPE_POS (CdpPacket.h) */
        uint8_t hop_count = payload[22]; /* HOP_COUNT_POS (CdpPacket.h) */
        /* DuckType: UNKNOWN=0 .. MAX_TYPE-1=4  (DuckTypes.h)
         * MAX_HOPS: 6                           (CdpPacket.h) */
        if (duck_type <= 4 && hop_count <= 6) {
            return false; /* looks like CDP — route to ClusterDuck bridge */
        }
    }
    /* Everything else: treat as LoRaWAN */
    if (size < 8) return false;  /* too short for any valid LoRaWAN frame */
    return true;
}

void thread_up(void) {
    int i, j, k;
    char stat_timestamp[24];
    time_t t;

    /* allocate memory for packet fetching and processing */
    struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX];
    struct lgw_pkt_rx_s *p;
    int nb_pkt;

    /* data buffer for Semtech UDP PUSH_DATA (LoRaWAN path) */
    uint8_t buff_up[TX_BUFF_SIZE];
    int buff_index;

    /* PUSH_DATA framing */
    uint8_t token_h, token_l;
    int pkt_in_dgram;

    /* report management */
    bool send_report = false;

    /* GPS time reference (copy taken once per fetch to avoid per-packet mutex) */
    bool ref_ok = false;
    struct tref local_ref;
    struct timespec pkt_utc_time;
    struct tm *x;
    struct timespec pkt_gps_time;
    uint64_t pkt_gps_time_ms;

    /* mote info (LoRaWAN DevAddr + FCnt extracted from FHDR) */
    uint32_t mote_addr = 0;
    uint16_t mote_fcnt  = 0;

    /* pre-fill the fixed header fields of the PUSH_DATA buffer */
    buff_up[0] = PROTOCOL_VERSION;
    buff_up[3] = PKT_PUSH_DATA;
    *(uint32_t *)(buff_up + 4) = net_mac_h;
    *(uint32_t *)(buff_up + 8) = net_mac_l;

    while (!exit_sig && !quit_sig) {

        /* ----------------------------------------------------------------
         * Fetch all available packets from the concentrator in one call.
         * Both LoRaWAN and CDP packets arrive here — the classifier below
         * routes each one to the correct protocol path.
         * -------------------------------------------------------------- */
        pthread_mutex_lock(&mx_concent);
        nb_pkt = lgw_receive(NB_PKT_MAX, rxpkt);
        pthread_mutex_unlock(&mx_concent);
        if (nb_pkt == LGW_HAL_ERROR) {
            MSG("ERROR: [up] failed packet fetch, exiting\n");
            exit_sig = true; /* let the main thread perform a clean lgw_stop() shutdown */
            return;
        }

        /* RAW PACKET DIAGNOSTIC — print every received packet before any filtering */
        if (nb_pkt > 0) {
            int _d;
            for (_d = 0; _d < nb_pkt; _d++) {
                struct lgw_pkt_rx_s *_p = &rxpkt[_d];
                MSG("RAW[%d/%d] freq=%u SF%u BW%ukHz CR%u size=%u status=%u rssi=%.0f snr=%.1f payload[0..3]=%02X %02X %02X %02X\n",
                    _d+1, nb_pkt,
                    _p->freq_hz,
                    _p->datarate,
                    (_p->bandwidth == BW_125KHZ ? 125 : _p->bandwidth == BW_250KHZ ? 250 : 500),
                    _p->coderate,
                    _p->size, _p->status,
                    _p->rssic, _p->snr,
                    (_p->size > 0 ? _p->payload[0] : 0xFF),
                    (_p->size > 1 ? _p->payload[1] : 0xFF),
                    (_p->size > 2 ? _p->payload[2] : 0xFF),
                    (_p->size > 3 ? _p->payload[3] : 0xFF));
            }
        }

        /* check if there are status report to send */
        send_report = report_ready;

        /* wait a short time if no packets, nor status report */
        if ((nb_pkt == 0) && (send_report == false)) {
            wait_ms(FETCH_SLEEP_MS);
            continue;
        }

        /* get GPS time reference once per fetch cycle */
        if ((nb_pkt > 0) && (gps_enabled == true)) {
            pthread_mutex_lock(&mx_timeref);
            ref_ok    = gps_ref_valid;
            local_ref = time_reference_gps;
            pthread_mutex_unlock(&mx_timeref);
        } else {
            ref_ok = false;
        }

        /* timestamp for statistics */
        t = time(NULL);
        strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));
        MSG_DEBUG(DEBUG_PKT_FWD, "\nCurrent time: %s \n", stat_timestamp);

        /* DEBUG channel/SF counters */
        {
            int l, m;
            MSG_PRINTF(DEBUG_PKT_FWD, "\n");
            for (l = 0; l < (LGW_IF_CHAIN_NB - 1); l++) {
                MSG_PRINTF(DEBUG_PKT_FWD, "CH%d: ", l);
                for (m = 0; m < 8; m++) {
                    MSG_PRINTF(DEBUG_PKT_FWD, "\t%d", nb_pkt_log[l][m]);
                }
                MSG_PRINTF(DEBUG_PKT_FWD, "\n");
            }
            MSG_PRINTF(DEBUG_PKT_FWD, "FSK: \t%d\n", nb_pkt_log[9][0]);
            MSG_PRINTF(DEBUG_PKT_FWD, "Total LoRa pkts received: %u\n", nb_pkt_received_lora);
            MSG_PRINTF(DEBUG_PKT_FWD, "Total FSK  pkts received: %u\n", nb_pkt_received_fsk);
            for (l = 0; l < debugconf.nb_ref_payload; l++) {
                MSG_PRINTF(DEBUG_PKT_FWD, "Total pkts from 0x%08X: %u\n",
                           debugconf.ref_payload[l].id, nb_pkt_received_ref[l]);
            }
        }

        if (nb_pkt > 0) {
            MSG("INFO: [up] fetched %d packet(s) from concentrator\n", nb_pkt);
        }

        /* ----------------------------------------------------------------
         * Begin the PUSH_DATA datagram for LoRaWAN packets.
         * We build one datagram per fetch cycle, appending each LoRaWAN
         * packet as a JSON rxpk object.  CDP packets bypass this buffer
         * and go directly to the ClusterDuck bridge.
         * -------------------------------------------------------------- */
        token_h = (uint8_t)rand();
        token_l = (uint8_t)rand();
        buff_up[1] = token_h;
        buff_up[2] = token_l;
        buff_index  = 12;           /* 12-byte header */
        pkt_in_dgram = 0;

        /* open the rxpk JSON array */
        memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
        buff_index += 9;

        /* ----------------------------------------------------------------
         * Per-packet loop: classify → route
         * -------------------------------------------------------------- */
        for (i = 0; i < nb_pkt; ++i) {
            p = &rxpkt[i];

            /* ---- basic validation ---- */
            if (p->size == 0 || p->size > 255) {
                MSG("WARNING: [up] [%d/%d] invalid size=%u, skipping\n", i+1, nb_pkt, p->size);
                continue;
            }
            if (p->payload == NULL) {
                MSG("WARNING: [up] [%d/%d] NULL payload, skipping\n", i+1, nb_pkt);
                continue;
            }

            /* ---- CRC filter (applies to both paths) ---- */
            pthread_mutex_lock(&mx_meas_up);
            meas_nb_rx_rcv += 1;
            switch (p->status) {
                case STAT_CRC_OK:
                    meas_nb_rx_ok += 1;
                    if (!fwd_valid_pkt) { pthread_mutex_unlock(&mx_meas_up); continue; }
                    break;
                case STAT_CRC_BAD:
                    meas_nb_rx_bad += 1;
                    if (!fwd_error_pkt) { pthread_mutex_unlock(&mx_meas_up); continue; }
                    break;
                case STAT_NO_CRC:
                    meas_nb_rx_nocrc += 1;
                    if (!fwd_nocrc_pkt) { pthread_mutex_unlock(&mx_meas_up); continue; }
                    break;
                default:
                    MSG("WARNING: [up] unknown status 0x%02X, skipping\n", p->status);
                    pthread_mutex_unlock(&mx_meas_up);
                    continue;
            }
            meas_up_pkt_fwd    += 1;
            meas_up_payload_byte += p->size;
            pthread_mutex_unlock(&mx_meas_up);

            /* ---- PACKET CLASSIFIER ---- */
            if (lorawan_fwd_enabled) {
                bool is_lw = is_lorawan_packet(p->payload, p->size);
                MSG("DEBUG: [up] pkt[%d/%d] MHDR=0x%02X size=%u → %s\n",
                    i+1, nb_pkt, (p->size > 0 ? p->payload[0] : 0xFF), p->size,
                    is_lw ? "LoRaWAN" : "CDP");
            }
            if (lorawan_fwd_enabled && is_lorawan_packet(p->payload, p->size)) {
                /* ============================================================
                 * PATH A: LoRaWAN → Semtech UDP PUSH_DATA → Network Server
                 * ============================================================ */

                /* extract DevAddr + FCnt from FHDR for logging */
                if (p->size >= 8) {
                    mote_addr  = p->payload[1];
                    mote_addr |= p->payload[2] << 8;
                    mote_addr |= p->payload[3] << 16;
                    mote_addr |= p->payload[4] << 24;
                    mote_fcnt  = p->payload[6];
                    mote_fcnt |= p->payload[7] << 8;
                } else {
                    mote_addr = mote_fcnt = 0;
                }
                MSG("INFO: [up] [%d/%d] LoRaWAN pkt → NS (DevAddr=%08X fcnt=%u size=%u rssi=%.0f snr=%.1f)\n",
                    i+1, nb_pkt, mote_addr, mote_fcnt, p->size, p->rssic, p->snr);

                /* inter-packet separator */
                if (pkt_in_dgram == 0) {
                    buff_up[buff_index++] = '{';
                } else {
                    buff_up[buff_index++] = ',';
                    buff_up[buff_index++] = '{';
                }

                /* jver */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                             "\"jver\":%d", PROTOCOL_JSON_RXPK_FRAME_FORMAT);
                if (j > 0) buff_index += j; else { MSG("ERROR: [up] snprintf jver\n"); exit(EXIT_FAILURE); }

                /* tmst */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                             ",\"tmst\":%u", p->count_us);
                if (j > 0) buff_index += j; else { MSG("ERROR: [up] snprintf tmst\n"); exit(EXIT_FAILURE); }

                /* GPS-based UTC + GPS time (optional) */
                if (ref_ok == true) {
                    j = lgw_cnt2utc(local_ref, p->count_us, &pkt_utc_time);
                    if (j == LGW_GPS_SUCCESS) {
                        x = gmtime(&(pkt_utc_time.tv_sec));
                        j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                                     ",\"time\":\"%04i-%02i-%02iT%02i:%02i:%02i.%06liZ\"",
                                     (x->tm_year)+1900, (x->tm_mon)+1, x->tm_mday,
                                     x->tm_hour, x->tm_min, x->tm_sec,
                                     (pkt_utc_time.tv_nsec)/1000);
                        if (j > 0) buff_index += j;
                    }
                    j = lgw_cnt2gps(local_ref, p->count_us, &pkt_gps_time);
                    if (j == LGW_GPS_SUCCESS) {
                        pkt_gps_time_ms = pkt_gps_time.tv_sec * 1000ULL +
                                          pkt_gps_time.tv_nsec / 1000000ULL;
                        j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                                     ",\"tmms\":%" PRIu64, pkt_gps_time_ms);
                        if (j > 0) buff_index += j;
                    }
                }

                /* fine timestamp */
                if (p->ftime_received == true) {
                    j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                                 ",\"ftime\":%u", p->ftime);
                    if (j > 0) buff_index += j;
                }

                /* channel, rf chain, freq, modem id */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                             ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf,\"mid\":%2u",
                             p->if_chain, p->rf_chain, ((double)p->freq_hz / 1e6), p->modem_id);
                if (j > 0) buff_index += j; else { MSG("ERROR: [up] snprintf chan\n"); exit(EXIT_FAILURE); }

                /* CRC status */
                switch (p->status) {
                    case STAT_CRC_OK:  memcpy(buff_up + buff_index, ",\"stat\":1",  9); buff_index += 9;  break;
                    case STAT_CRC_BAD: memcpy(buff_up + buff_index, ",\"stat\":-1", 10); buff_index += 10; break;
                    case STAT_NO_CRC:  memcpy(buff_up + buff_index, ",\"stat\":0",  9); buff_index += 9;  break;
                    default: break;
                }

                /* modulation, datarate, bandwidth, coderate */
                if (p->modulation == MOD_LORA) {
                    memcpy(buff_up + buff_index, ",\"modu\":\"LORA\"", 14); buff_index += 14;
                    switch (p->datarate) {
                        case DR_LORA_SF5:  memcpy(buff_up+buff_index, ",\"datr\":\"SF5",  12); buff_index+=12; break;
                        case DR_LORA_SF6:  memcpy(buff_up+buff_index, ",\"datr\":\"SF6",  12); buff_index+=12; break;
                        case DR_LORA_SF7:  memcpy(buff_up+buff_index, ",\"datr\":\"SF7",  12); buff_index+=12; break;
                        case DR_LORA_SF8:  memcpy(buff_up+buff_index, ",\"datr\":\"SF8",  12); buff_index+=12; break;
                        case DR_LORA_SF9:  memcpy(buff_up+buff_index, ",\"datr\":\"SF9",  12); buff_index+=12; break;
                        case DR_LORA_SF10: memcpy(buff_up+buff_index, ",\"datr\":\"SF10", 13); buff_index+=13; break;
                        case DR_LORA_SF11: memcpy(buff_up+buff_index, ",\"datr\":\"SF11", 13); buff_index+=13; break;
                        case DR_LORA_SF12: memcpy(buff_up+buff_index, ",\"datr\":\"SF12", 13); buff_index+=13; break;
                        default: memcpy(buff_up+buff_index, ",\"datr\":\"SF?",  12); buff_index+=12; break;
                    }
                    switch (p->bandwidth) {
                        case BW_125KHZ: memcpy(buff_up+buff_index, "BW125\"", 6); buff_index+=6; break;
                        case BW_250KHZ: memcpy(buff_up+buff_index, "BW250\"", 6); buff_index+=6; break;
                        case BW_500KHZ: memcpy(buff_up+buff_index, "BW500\"", 6); buff_index+=6; break;
                        default:        memcpy(buff_up+buff_index, "BW?\"",   4); buff_index+=4; break;
                    }
                    switch (p->coderate) {
                        case CR_LORA_4_5: memcpy(buff_up+buff_index, ",\"codr\":\"4/5\"", 13); buff_index+=13; break;
                        case CR_LORA_4_6: memcpy(buff_up+buff_index, ",\"codr\":\"4/6\"", 13); buff_index+=13; break;
                        case CR_LORA_4_7: memcpy(buff_up+buff_index, ",\"codr\":\"4/7\"", 13); buff_index+=13; break;
                        case CR_LORA_4_8: memcpy(buff_up+buff_index, ",\"codr\":\"4/8\"", 13); buff_index+=13; break;
                        case 0:           memcpy(buff_up+buff_index, ",\"codr\":\"OFF\"", 13); buff_index+=13; break;
                        default:          memcpy(buff_up+buff_index, ",\"codr\":\"?\"",  11); buff_index+=11; break;
                    }
                    j = snprintf((char *)(buff_up+buff_index), TX_BUFF_SIZE-buff_index,
                                 ",\"rssis\":%.0f", roundf(p->rssis));
                    if (j > 0) buff_index += j;
                    j = snprintf((char *)(buff_up+buff_index), TX_BUFF_SIZE-buff_index,
                                 ",\"lsnr\":%.1f", p->snr);
                    if (j > 0) buff_index += j;
                    j = snprintf((char *)(buff_up+buff_index), TX_BUFF_SIZE-buff_index,
                                 ",\"foff\":%d", p->freq_offset);
                    if (j > 0) buff_index += j;
                    nb_pkt_log[p->if_chain][p->datarate - 5] += 1;
                    nb_pkt_received_lora += 1;
                    for (k = 0; k < debugconf.nb_ref_payload; k++) {
                        if ((p->payload[0] == (uint8_t)(debugconf.ref_payload[k].id >> 24)) &&
                            (p->payload[1] == (uint8_t)(debugconf.ref_payload[k].id >> 16)) &&
                            (p->payload[2] == (uint8_t)(debugconf.ref_payload[k].id >>  8)) &&
                            (p->payload[3] == (uint8_t)(debugconf.ref_payload[k].id >>  0))) {
                            nb_pkt_received_ref[k] += 1;
                        }
                    }
                } else if (p->modulation == MOD_FSK) {
                    memcpy(buff_up+buff_index, ",\"modu\":\"FSK\"", 13); buff_index += 13;
                    j = snprintf((char *)(buff_up+buff_index), TX_BUFF_SIZE-buff_index,
                                 ",\"datr\":%u", p->datarate);
                    if (j > 0) buff_index += j;
                    nb_pkt_log[p->if_chain][0] += 1;
                    nb_pkt_received_fsk += 1;
                }

                /* RSSI + size */
                j = snprintf((char *)(buff_up+buff_index), TX_BUFF_SIZE-buff_index,
                             ",\"rssi\":%.0f,\"size\":%u", roundf(p->rssic), p->size);
                if (j > 0) buff_index += j;

                /* base64 payload */
                memcpy(buff_up+buff_index, ",\"data\":\"", 9); buff_index += 9;
                j = bin_to_b64(p->payload, p->size, (char *)(buff_up+buff_index), 341);
                if (j >= 0) buff_index += j;
                buff_up[buff_index++] = '"';

                /* close rxpk object */
                buff_up[buff_index++] = '}';
                ++pkt_in_dgram;

            } else {
                /* ============================================================
                 * PATH B: CDP (ClusterDuck Protocol) → ClusterDuck bridge
                 * ============================================================ */
                MSG("INFO: [up] [%d/%d] CDP pkt → bridge (size=%u rssi=%.0f snr=%.1f)\n",
                    i+1, nb_pkt, p->size, p->rssic, p->snr);

                /* Reject obviously bad coding rates before handing to CDP */
                if (p->coderate == 0 || p->coderate > CR_LORA_4_8) {
                    MSG("WARNING: [up] invalid coderate (%u), skipping\n", p->coderate);
                    continue;
                }

                cdp_bridge_handle_uplink(
                    p->payload, (uint16_t)p->size,
                    (int16_t)p->rssic, p->snr,
                    p->freq_hz, p->count_us, (uint8_t)p->if_chain,
                    p->bandwidth, p->datarate, p->coderate);
            }
        } /* end per-packet loop */

        /* --------------------------------------------------------------------
         * Send accumulated LoRaWAN PUSH_DATA datagram (if any) to the NS
         * ------------------------------------------------------------------ */
        if (lorawan_fwd_enabled && pkt_in_dgram > 0) {
            /* close rxpk array + JSON object */
            buff_up[buff_index++] = ']';

            /* optionally append status report */
            if (send_report == true) {
                buff_up[buff_index++] = ',';
                pthread_mutex_lock(&mx_stat_rep);
                report_ready = false;
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                             "%s", status_report);
                pthread_mutex_unlock(&mx_stat_rep);
                if (j > 0) buff_index += j;
            }

            buff_up[buff_index++] = '}';
            buff_up[buff_index]   = 0; /* null-terminate for safety */

            MSG_DEBUG(DEBUG_PKT_FWD, "\nJSON up: %s\n", (char *)(buff_up + 12));

            MSG("INFO: [up] sending PUSH_DATA to NS (%d pkt(s), %d bytes)\n", pkt_in_dgram, buff_index);
            j = send(sock_up, (void *)buff_up, buff_index, 0);
            if (j < 0) {
                MSG("ERROR: [up] send() to NS failed: %s\n", strerror(errno));
            }
            pthread_mutex_lock(&mx_meas_up);
            meas_up_dgram_sent   += 1;
            meas_up_network_byte += buff_index;
            pthread_mutex_unlock(&mx_meas_up);
            /* PUSH_ACK is not awaited here: blocking on recv() for up to
             * 100 ms (2 × push_timeout_half) stalls the fetch loop and
             * delays CDP packet delivery.  The NS does not require an ACK
             * confirmation from the gateway to process uplink packets, so
             * dropping the wait is safe.  The sock_up socket is non-blocking
             * for reads; any stray PUSH_ACK arriving later is silently
             * discarded by the OS. */
        } else if (lorawan_fwd_enabled && send_report == true) {
            /* LoRaWAN enabled, no packets this cycle but stat report due:
             * send a PUSH_DATA with empty rxpk array + stat object so the
             * NS sees regular gateway status heartbeats. */
            buff_index = 12;
            memcpy(buff_up + buff_index, "{\"rxpk\":[],", 11); buff_index += 11;
            pthread_mutex_lock(&mx_stat_rep);
            report_ready = false;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index,
                         "%s", status_report);
            pthread_mutex_unlock(&mx_stat_rep);
            if (j > 0) buff_index += j;
            buff_up[buff_index++] = '}';
            buff_up[buff_index]   = 0;
            send(sock_up, (void *)buff_up, buff_index, 0);
            pthread_mutex_lock(&mx_meas_up);
            meas_up_dgram_sent   += 1;
            meas_up_network_byte += buff_index;
            pthread_mutex_unlock(&mx_meas_up);
            MSG("INFO: [up] stat-only PUSH_DATA sent to NS\n");
        }

    } /* end main loop */
    MSG("\nINFO: End of upstream thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 2: POLLING SERVER AND ENQUEUING PACKETS IN JIT QUEUE ---------- */

/* Send TX acknowledgement datagram back to the Network Server.
 * Only called when lorawan_fwd_enabled is true (sock_down is valid). */
static int send_tx_ack(uint8_t token_h, uint8_t token_l, enum jit_error_e error, int32_t error_value) {
    uint8_t buff_ack[64];
    int buff_index = 0;
    memset(buff_ack, 0, sizeof buff_ack);
    buff_ack[buff_index++] = PROTOCOL_VERSION;
    buff_ack[buff_index++] = token_h;
    buff_ack[buff_index++] = token_l;
    buff_ack[buff_index++] = PKT_TX_ACK;
    *(uint32_t *)(buff_ack + buff_index) = net_mac_h; buff_index += 4;
    *(uint32_t *)(buff_ack + buff_index) = net_mac_l; buff_index += 4;
    if (error != JIT_ERROR_OK) {
        memcpy(buff_ack + buff_index, "{\"txpk_ack\":{\"error\":", 21); buff_index += 21;
        switch (error) {
            case JIT_ERROR_FULL:
            case JIT_ERROR_COLLISION_PACKET:
                memcpy(buff_ack + buff_index, "\"COLLISION_PACKET\"}}", 20); buff_index += 20; break;
            case JIT_ERROR_TOO_LATE:
                memcpy(buff_ack + buff_index, "\"TOO_LATE\"}}", 13); buff_index += 13; break;
            case JIT_ERROR_TOO_EARLY:
                memcpy(buff_ack + buff_index, "\"TOO_EARLY\"}}", 14); buff_index += 14; break;
            case JIT_ERROR_COLLISION_BEACON:
                memcpy(buff_ack + buff_index, "\"COLLISION_BEACON\"}}", 21); buff_index += 21; break;
            case JIT_ERROR_TX_FREQ:
                memcpy(buff_ack + buff_index, "\"TX_FREQ\"}}", 12); buff_index += 12; break;
            case JIT_ERROR_TX_POWER:
                memcpy(buff_ack + buff_index, "\"TX_POWER\"}}", 13); buff_index += 13; break;
            case JIT_ERROR_GPS_UNLOCKED:
                memcpy(buff_ack + buff_index, "\"GPS_UNLOCKED\"}}", 17); buff_index += 17; break;
            default:
                memcpy(buff_ack + buff_index, "\"UNKNOWN\"}}", 12); buff_index += 12; break;
        }
    } else {
        memcpy(buff_ack + buff_index, "{\"txpk_ack\":{\"error\":\"NONE\"}}", 31); buff_index += 31;
    }
    (void)error_value; /* reserved for future extended error reporting */
    return send(sock_down, (void *)buff_ack, buff_index, 0);
}

static int get_tx_gain_lut_index(uint8_t rf_chain, int8_t rf_power, uint8_t * lut_index) {
    uint8_t pow_index;
    int current_best_index = -1;
    uint8_t current_best_match = 0xFF;
    int diff;

    /* Check input parameters */
    if (lut_index == NULL) {
        MSG("ERROR: %s - wrong parameter\n", __FUNCTION__);
        return -1;
    }

    /* Search requested power in TX gain LUT */
    for (pow_index = 0; pow_index < txlut[rf_chain].size; pow_index++) {
        diff = rf_power - txlut[rf_chain].lut[pow_index].rf_power;
        if (diff < 0) {
            /* The selected power must be lower or equal to requested one */
            continue;
        } else {
            /* Record the index corresponding to the closest rf_power available in LUT */
            if ((current_best_index == -1) || (diff < current_best_match)) {
                current_best_match = diff;
                current_best_index = pow_index;
            }
        }
    }

    /* Return corresponding index */
    if (current_best_index > -1) {
        *lut_index = (uint8_t)current_best_index;
    } else {
        *lut_index = 0;
        MSG("ERROR: %s - failed to find tx gain lut index\n", __FUNCTION__);
        return -1;
    }

    return 0;
}

void thread_down(void) {
    int i; /* loop variables */

    /* configuration and metadata for an outbound packet */
    struct lgw_pkt_tx_s txpkt;

    /* local timekeeping variables */
    struct timespec send_time; /* time of the pull request */
    struct timespec recv_time; /* time of return from recv socket call */

    /* data buffers */
    uint8_t buff_down[1000]; /* buffer to receive downstream packets */
    uint8_t buff_req[12]; /* buffer to compose pull requests */
    int msg_len;

    /* protocol variables */
    uint8_t token_h; /* random token for acknowledgement matching */
    uint8_t token_l; /* random token for acknowledgement matching */

    /* beacon variables */
    struct lgw_pkt_tx_s beacon_pkt;
    uint8_t beacon_chan;
    uint8_t beacon_loop;
    size_t beacon_RFU1_size = 0;
    size_t beacon_RFU2_size = 0;
    uint8_t beacon_pyld_idx = 0;
    time_t diff_beacon_time;
    struct timespec next_beacon_gps_time; /* gps time of next beacon packet */
    struct timespec last_beacon_gps_time; /* gps time of last enqueued beacon packet */
    int retry;

    /* beacon data fields, byte 0 is Least Significant Byte */
    int32_t field_latitude; /* 3 bytes, derived from reference latitude */
    int32_t field_longitude; /* 3 bytes, derived from reference longitude */
    uint16_t field_crc1, field_crc2;

    /* auto-quit variable */
    uint32_t autoquit_cnt = 0; /* count the number of PULL_DATA sent since the latest PULL_ACK */

    /* Just In Time downlink */
    uint32_t current_concentrator_time;
    enum jit_error_e jit_result = JIT_ERROR_OK;
    enum jit_pkt_type_e downlink_type;
    enum jit_error_e warning_result = JIT_ERROR_OK;
    int32_t warning_value = 0;
    uint8_t tx_lut_idx = 0;

    /* Zaihan's code */
    /* Buffer for a possible downlink from ClusterDuck */
    uint8_t duck_payload_buf[256];
    uint16_t buf_capacity = sizeof(duck_payload_buf);
    uint32_t dl_freq_hz = 0;
    uint32_t dl_tmst = 0;
    int16_t dl_tx_power_dbm = 0;
    uint32_t dl_bw_hz = 0;
    uint8_t dl_sf = 0;
    uint8_t dl_cr = 0;
    uint8_t dl_rf_chain = 0;
    /* End of Zaihan's code */

    /* LoRaWAN PULL_DATA / PULL_RESP state */
    bool req_ack = false;       /* true once PULL_ACK received for current token */
    bool sent_immediate = false;
    JSON_Value *root_val = NULL;
    JSON_Object *txpk_obj = NULL;
    JSON_Value *val = NULL;
    const char *str = NULL;
    short x0, x1;
    double x2, x3, x4;
    struct timespec gps_tx;
    struct tref local_ref;

    /* beacon variables initialization */
    last_beacon_gps_time.tv_sec = 0;
    last_beacon_gps_time.tv_nsec = 0;

    /* beacon packet parameters */
    beacon_pkt.tx_mode = ON_GPS; /* send on PPS pulse */
    beacon_pkt.rf_chain = 0; /* antenna A */
    beacon_pkt.rf_power = beacon_power;
    beacon_pkt.modulation = MOD_LORA;
    switch (beacon_bw_hz) {
        case 125000:
            beacon_pkt.bandwidth = BW_125KHZ;
            break;
        case 500000:
            beacon_pkt.bandwidth = BW_500KHZ;
            break;
        default:
            /* should not happen */
            MSG("ERROR: unsupported bandwidth for beacon\n");
            exit(EXIT_FAILURE);
    }
    switch (beacon_datarate) {
        case 8:
            beacon_pkt.datarate = DR_LORA_SF8;
            beacon_RFU1_size = 1;
            beacon_RFU2_size = 3;
            break;
        case 9:
            beacon_pkt.datarate = DR_LORA_SF9;
            beacon_RFU1_size = 2;
            beacon_RFU2_size = 0;
            break;
        case 10:
            beacon_pkt.datarate = DR_LORA_SF10;
            beacon_RFU1_size = 3;
            beacon_RFU2_size = 1;
            break;
        case 12:
            beacon_pkt.datarate = DR_LORA_SF12;
            beacon_RFU1_size = 5;
            beacon_RFU2_size = 3;
            break;
        default:
            /* should not happen */
            MSG("ERROR: unsupported datarate for beacon\n");
            exit(EXIT_FAILURE);
    }
    beacon_pkt.size = beacon_RFU1_size + 4 + 2 + 7 + beacon_RFU2_size + 2;
    beacon_pkt.coderate = CR_LORA_4_5;
    beacon_pkt.invert_pol = false;
    beacon_pkt.preamble = 10;
    beacon_pkt.no_crc = true;
    beacon_pkt.no_header = true;

    /* network common part beacon fields (little endian) */
    for (i = 0; i < (int)beacon_RFU1_size; i++) {
        beacon_pkt.payload[beacon_pyld_idx++] = 0x0;
    }

    /* network common part beacon fields (little endian) */
    beacon_pyld_idx += 4; /* time (variable), filled later */
    beacon_pyld_idx += 2; /* crc1 (variable), filled later */

    /* calculate the latitude and longitude that must be publicly reported */
    field_latitude = (int32_t)((reference_coord.lat / 90.0) * (double)(1<<23));
    if (field_latitude > (int32_t)0x007FFFFF) {
        field_latitude = (int32_t)0x007FFFFF; /* +90 N is represented as 89.99999 N */
    } else if (field_latitude < (int32_t)0xFF800000) {
        field_latitude = (int32_t)0xFF800000;
    }
    field_longitude = (int32_t)((reference_coord.lon / 180.0) * (double)(1<<23));
    if (field_longitude > (int32_t)0x007FFFFF) {
        field_longitude = (int32_t)0x007FFFFF; /* +180 E is represented as 179.99999 E */
    } else if (field_longitude < (int32_t)0xFF800000) {
        field_longitude = (int32_t)0xFF800000;
    }

    /* gateway specific beacon fields */
    beacon_pkt.payload[beacon_pyld_idx++] = beacon_infodesc;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_latitude;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_latitude >>  8);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_latitude >> 16);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_longitude;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_longitude >>  8);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_longitude >> 16);

    /* RFU */
    for (i = 0; i < (int)beacon_RFU2_size; i++) {
        beacon_pkt.payload[beacon_pyld_idx++] = 0x0;
    }

    /* CRC of the beacon gateway specific part fields */
    field_crc2 = crc16((beacon_pkt.payload + 6 + beacon_RFU1_size), 7 + beacon_RFU2_size);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_crc2;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_crc2 >> 8);

    /* JIT queue initialization */
    jit_queue_init(&jit_queue[0]);
    jit_queue_init(&jit_queue[1]);

    /* LoRaWAN downstream socket setup */
    if (lorawan_fwd_enabled) {
        i = setsockopt(sock_down, SOL_SOCKET, SO_RCVTIMEO, (void *)&pull_timeout, sizeof pull_timeout);
        if (i != 0) {
            MSG("ERROR: [down] setsockopt returned %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        /* pre-fill the PULL_DATA buffer with fixed fields */
        buff_req[0] = PROTOCOL_VERSION;
        buff_req[3] = PKT_PULL_DATA;
        *(uint32_t *)(buff_req + 4) = net_mac_h;
        *(uint32_t *)(buff_req + 8) = net_mac_l;
    }

    while (!exit_sig && !quit_sig) {


            /* Try to pop a downlink from ClusterDuck */
            int duck_rc = cdp_bridge_pop_downlink(duck_payload_buf, &buf_capacity,
                                                   &dl_freq_hz, &dl_tmst, &dl_tx_power_dbm,
                                                   &dl_bw_hz, &dl_sf, &dl_cr, &dl_rf_chain);

        if (duck_rc != 1) {
            MSG("DEBUG: [thread_down] cdp_bridge_pop_downlink returned %d, buf_capacity=%u\n", duck_rc, buf_capacity);
        }

        if (duck_rc == 0 && buf_capacity > 0) {

            MSG("INFO: ClusterDuck downlink received (outer loop), size=%u\n", buf_capacity);

                    /* Build lgw_pkt_tx_s */
                    memset(&txpkt, 0, sizeof(txpkt));

                    /* Copy payload */
                    memcpy(txpkt.payload, duck_payload_buf, buf_capacity);
                    txpkt.size = buf_capacity;

                    /* Frequency: use ext metadata if provided, else CDP default */
                    txpkt.freq_hz = (dl_freq_hz > 0) ? dl_freq_hz : CDPCFG_RF_LORA_FREQ_HZ;
                    MSG("INFO: ClusterDuck outer downlink TX freq=%u Hz\n", txpkt.freq_hz);

                    /* For ClusterDuck packets, use immediate transmission */
                    txpkt.count_us = 0;
                    txpkt.tx_mode = IMMEDIATE;

                /* RF chain and power */
                txpkt.rf_chain = (dl_rf_chain < LGW_RF_CHAIN_NB) ? dl_rf_chain : 0;
                txpkt.rf_power = CDPCFG_RF_LORA_TXPOW;

                /* Modulation params */
                txpkt.modulation  = MOD_LORA;
                txpkt.bandwidth   = BW_125KHZ;
                txpkt.datarate    = DR_LORA_SF7;
                txpkt.coderate    = CR_LORA_4_5;
                txpkt.invert_pol  = false; /* CDP is not LoRaWAN — use normal (non-inverted) IQ */
                txpkt.preamble    = 8;     /* Standard LoRa preamble */
                txpkt.no_crc      = false;
                txpkt.no_header   = false;

	        MSG("INFO: txpkt size is %u\n", txpkt.size);
                downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_C;

                /* reset error/warning results */
                jit_result = warning_result = JIT_ERROR_OK;
                warning_value = 0;

                /* check TX frequency before trying to queue packet */
                if ((txpkt.freq_hz < tx_freq_min[txpkt.rf_chain]) || (txpkt.freq_hz > tx_freq_max[txpkt.rf_chain])) {
                    jit_result = JIT_ERROR_TX_FREQ;
                    MSG("ERROR: Packet REJECTED, unsupported frequency - %u (min:%u,max:%u)\n", txpkt.freq_hz, tx_freq_min[txpkt.rf_chain], tx_freq_max[txpkt.rf_chain]);
                }

                /* check TX power before trying to queue packet, send a warning if not supported */
                if (jit_result == JIT_ERROR_OK) {
                    i = get_tx_gain_lut_index(txpkt.rf_chain, txpkt.rf_power, &tx_lut_idx);
                    if ((i < 0) || (txlut[txpkt.rf_chain].lut[tx_lut_idx].rf_power != txpkt.rf_power)) {
                        /* this RF power is not supported, throw a warning, and use the closest lower power supported */
                        warning_result = JIT_ERROR_TX_POWER;
                        warning_value = (int32_t)txlut[txpkt.rf_chain].lut[tx_lut_idx].rf_power;
                        printf("WARNING: Requested TX power is not supported (%ddBm), actual power used: %ddBm\n", txpkt.rf_power, warning_value);
                        txpkt.rf_power = txlut[txpkt.rf_chain].lut[tx_lut_idx].rf_power;
                    }
                }

                /* insert packet to be sent into JIT queue */
                if (jit_result == JIT_ERROR_OK) {
                    /* Get fresh concentrator time for CDP downlink scheduling */
                    lgw_get_instcnt(&current_concentrator_time);
                    jit_result = jit_enqueue(&jit_queue[txpkt.rf_chain], current_concentrator_time, &txpkt, downlink_type);
                    if (jit_result != JIT_ERROR_OK) {
                        printf("ERROR: Packet REJECTED (jit error=%d)\n", jit_result);
                    } else {
                        /* In case of a warning having been raised before, we notify it */
                        jit_result = warning_result;
                    }
                    pthread_mutex_lock(&mx_meas_dw);
                    meas_nb_tx_requested += 1;
                    pthread_mutex_unlock(&mx_meas_dw);
                }

            /* Reset buffer capacity for next call */
            buf_capacity = sizeof(duck_payload_buf);
	    }

        /* auto-quit if the threshold is crossed */
        if ((autoquit_threshold > 0) && (autoquit_cnt >= autoquit_threshold)) {
            exit_sig = true;
            MSG("INFO: [down] the last %u PULL_DATA were not ACKed, exiting application\n", autoquit_threshold);
            break;
        }

        /* generate random token for request */
        token_h = (uint8_t)rand(); /* random token */
        token_l = (uint8_t)rand(); /* random token */
        buff_req[1] = token_h;
        buff_req[2] = token_l;

        /* send PULL_DATA request to NS and record time */
        if (lorawan_fwd_enabled) {
            send(sock_down, (void *)buff_req, sizeof buff_req, 0);
            clock_gettime(CLOCK_MONOTONIC, &send_time);
            pthread_mutex_lock(&mx_meas_dw);
            meas_dw_pull_sent += 1;
            pthread_mutex_unlock(&mx_meas_dw);
            req_ack = false;
            autoquit_cnt++;
        }

        /* listen to packets and process them until a new PULL request must be sent */
        clock_gettime(CLOCK_MONOTONIC, &send_time); /* anchor: marks when this PULL cycle started */
        recv_time = send_time;
        while (((int)difftimespec(recv_time, send_time) < keepalive_time) && !exit_sig && !quit_sig) {

            /* receive datagram from NS (blocks up to pull_timeout) */
            if (lorawan_fwd_enabled) {
                msg_len = recv(sock_down, (void *)buff_down, (sizeof buff_down)-1, 0);
                clock_gettime(CLOCK_MONOTONIC, &recv_time);
            } else {
                /* CDP-only mode: sleep briefly to yield the CPU */
                struct timespec slp = {0, 10000000L}; /* 10 ms */
                nanosleep(&slp, NULL);
                clock_gettime(CLOCK_MONOTONIC, &recv_time);
                msg_len = -1;
                errno = EAGAIN;
            }

            /* Pre-allocate beacon slots in JiT queue, to check downlink collisions */
            beacon_loop = JIT_NUM_BEACON_IN_QUEUE - jit_queue[0].num_beacon;
            retry = 0;
            while (beacon_loop && (beacon_period != 0)) {
                pthread_mutex_lock(&mx_timeref);
                /* Wait for GPS to be ready before inserting beacons in JiT queue */
                if ((gps_ref_valid == true) && (xtal_correct_ok == true)) {

                    /* compute GPS time for next beacon to come      */
                    /*   LoRaWAN: T = k*beacon_period + TBeaconDelay */
                    /*            with TBeaconDelay = [1.5ms +/- 1µs]*/
                    if (last_beacon_gps_time.tv_sec == 0) {
                        /* if no beacon has been queued, get next slot from current GPS time */
                        diff_beacon_time = time_reference_gps.gps.tv_sec % ((time_t)beacon_period);
                        next_beacon_gps_time.tv_sec = time_reference_gps.gps.tv_sec +
                                                        ((time_t)beacon_period - diff_beacon_time);
                    } else {
                        /* if there is already a beacon, take it as reference */
                        next_beacon_gps_time.tv_sec = last_beacon_gps_time.tv_sec + beacon_period;
                    }
                    /* now we can add a beacon_period to the reference to get next beacon GPS time */
                    next_beacon_gps_time.tv_sec += (retry * beacon_period);
                    next_beacon_gps_time.tv_nsec = 0;

#if DEBUG_BEACON
                    {
                    time_t time_unix;

                    time_unix = time_reference_gps.gps.tv_sec + UNIX_GPS_EPOCH_OFFSET;
                    MSG_DEBUG(DEBUG_BEACON, "GPS-now : %s", ctime(&time_unix));
                    time_unix = last_beacon_gps_time.tv_sec + UNIX_GPS_EPOCH_OFFSET;
                    MSG_DEBUG(DEBUG_BEACON, "GPS-last: %s", ctime(&time_unix));
                    time_unix = next_beacon_gps_time.tv_sec + UNIX_GPS_EPOCH_OFFSET;
                    MSG_DEBUG(DEBUG_BEACON, "GPS-next: %s", ctime(&time_unix));
                    }
#endif

                    /* convert GPS time to concentrator time, and set packet counter for JiT trigger */
                    lgw_gps2cnt(time_reference_gps, next_beacon_gps_time, &(beacon_pkt.count_us));
                    pthread_mutex_unlock(&mx_timeref);

                    /* apply frequency correction to beacon TX frequency */
                    if (beacon_freq_nb > 1) {
                        beacon_chan = (next_beacon_gps_time.tv_sec / beacon_period) % beacon_freq_nb; /* floor rounding */
                    } else {
                        beacon_chan = 0;
                    }
                    /* Compute beacon frequency */
                    beacon_pkt.freq_hz = beacon_freq_hz + (beacon_chan * beacon_freq_step);

                    /* load time in beacon payload */
                    beacon_pyld_idx = beacon_RFU1_size;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  next_beacon_gps_time.tv_sec;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (next_beacon_gps_time.tv_sec >>  8);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (next_beacon_gps_time.tv_sec >> 16);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (next_beacon_gps_time.tv_sec >> 24);

                    /* calculate CRC */
                    field_crc1 = crc16(beacon_pkt.payload, 4 + beacon_RFU1_size); /* CRC for the network common part */
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & field_crc1;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_crc1 >> 8);

                    /* Insert beacon packet in JiT queue */
                    pthread_mutex_lock(&mx_concent);
                    lgw_get_instcnt(&current_concentrator_time);
                    pthread_mutex_unlock(&mx_concent);
                    jit_result = jit_enqueue(&jit_queue[0], current_concentrator_time, &beacon_pkt, JIT_PKT_TYPE_BEACON);
                    if (jit_result == JIT_ERROR_OK) {
                        /* update stats */
                        pthread_mutex_lock(&mx_meas_dw);
                        meas_nb_beacon_queued += 1;
                        pthread_mutex_unlock(&mx_meas_dw);

                        /* One more beacon in the queue */
                        beacon_loop--;
                        retry = 0;
                        last_beacon_gps_time.tv_sec = next_beacon_gps_time.tv_sec; /* keep this beacon time as reference for next one to be programmed */

                        /* display beacon payload */
                        MSG("INFO: Beacon queued (count_us=%u, freq_hz=%u, size=%u):\n", beacon_pkt.count_us, beacon_pkt.freq_hz, beacon_pkt.size);
                        printf( "   => " );
                        for (i = 0; i < beacon_pkt.size; ++i) {
                            MSG("%02X ", beacon_pkt.payload[i]);
                        }
                        MSG("\n");
                    } else {
                        MSG_DEBUG(DEBUG_BEACON, "--> beacon queuing failed with %d\n", jit_result);
                        /* update stats */
                        pthread_mutex_lock(&mx_meas_dw);
                        if (jit_result != JIT_ERROR_COLLISION_BEACON) {
                            meas_nb_beacon_rejected += 1;
                        }
                        pthread_mutex_unlock(&mx_meas_dw);
                        /* In case previous enqueue failed, we retry one period later until it succeeds */
                        /* Note: In case the GPS has been unlocked for a while, there can be lots of retries */
                        /*       to be done from last beacon time to a new valid one */
                        retry++;
                        MSG_DEBUG(DEBUG_BEACON, "--> beacon queuing retry=%d\n", retry);
                    }
                } else {
                    pthread_mutex_unlock(&mx_timeref);
                    break;
                }
            }

            /* Handle LoRaWAN datagram received from NS (PULL_ACK or PULL_RESP) */
            if (lorawan_fwd_enabled) {
                if (msg_len == -1) {
                    /* timeout or no packet — continue to CDP downlink check */
                } else if ((msg_len < 4) || (buff_down[0] != PROTOCOL_VERSION) ||
                           ((buff_down[3] != PKT_PULL_RESP) && (buff_down[3] != PKT_PULL_ACK))) {
                    MSG("WARNING: [down] ignoring invalid packet len=%d, protocol_version=%d, id=%d\n",
                        msg_len, buff_down[0], buff_down[3]);
                } else if (buff_down[3] == PKT_PULL_ACK) {
                    if ((buff_down[1] == token_h) && (buff_down[2] == token_l)) {
                        if (req_ack) {
                            MSG("INFO: [down] duplicate ACK received :)\n");
                        } else {
                            req_ack = true;
                            autoquit_cnt = 0;
                            pthread_mutex_lock(&mx_meas_dw);
                            meas_dw_ack_rcv += 1;
                            pthread_mutex_unlock(&mx_meas_dw);
                            MSG("DEBUG: [down] PULL_ACK received in %i ms\n",
                                (int)(1000 * difftimespec(recv_time, send_time)));
                        }
                    } else {
                        MSG("INFO: [down] received out-of-sync ACK\n");
                    }
                } else {
                    /* PULL_RESP: parse and enqueue downlink packet */
                    buff_down[msg_len] = 0;
                    MSG("INFO: [down] PULL_RESP received - token[%d:%d]\n", buff_down[1], buff_down[2]);

                    memset(&txpkt, 0, sizeof txpkt);
                    root_val = json_parse_string_with_comments((const char *)(buff_down + 4));
                    if (root_val == NULL) {
                        MSG("WARNING: [down] invalid JSON, TX aborted\n");
                        goto pull_resp_done;
                    }
                    txpk_obj = json_object_get_object(json_value_get_object(root_val), "txpk");
                    if (txpk_obj == NULL) {
                        MSG("WARNING: [down] no \"txpk\" in JSON, TX aborted\n");
                        json_value_free(root_val); root_val = NULL;
                        goto pull_resp_done;
                    }
                    /* transmission mode */
                    i = json_object_get_boolean(txpk_obj, "imme");
                    if (i == 1) {
                        sent_immediate = true;
                        downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_C;
                    } else {
                        sent_immediate = false;
                        val = json_object_get_value(txpk_obj, "tmst");
                        if (val != NULL) {
                            txpkt.count_us = (uint32_t)json_value_get_number(val);
                            downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_A;
                        } else {
                            MSG("WARNING: [down] no tmst or imme in PULL_RESP, TX aborted\n");
                            json_value_free(root_val); root_val = NULL;
                            send_tx_ack(buff_down[1], buff_down[2], JIT_ERROR_GPS_UNLOCKED, 0);
                            goto pull_resp_done;
                        }
                    }
                    /* optional fields */
                    val = json_object_get_value(txpk_obj, "ncrc");
                    if (val != NULL) txpkt.no_crc = (bool)json_value_get_boolean(val);
                    val = json_object_get_value(txpk_obj, "nhdr");
                    if (val != NULL) txpkt.no_header = (bool)json_value_get_boolean(val);
                    /* frequency */
                    val = json_object_get_value(txpk_obj, "freq");
                    if (val == NULL) { MSG("WARNING: [down] no freq, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done; }
                    txpkt.freq_hz = (uint32_t)((double)(1.0e6) * json_value_get_number(val));
                    /* rf chain */
                    val = json_object_get_value(txpk_obj, "rfch");
                    if (val == NULL) { MSG("WARNING: [down] no rfch, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done; }
                    txpkt.rf_chain = (uint8_t)json_value_get_number(val);
                    /* power */
                    val = json_object_get_value(txpk_obj, "powe");
                    if (val != NULL) txpkt.rf_power = (int8_t)json_value_get_number(val) - antenna_gain;
                    /* modulation */
                    str = json_object_get_string(txpk_obj, "modu");
                    if (str == NULL) { MSG("WARNING: [down] no modu, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done; }
                    if (strcmp(str, "LORA") == 0) {
                        txpkt.modulation = MOD_LORA;
                        str = json_object_get_string(txpk_obj, "datr");
                        if (str == NULL || sscanf(str, "SF%2hdBW%3hd", &x0, &x1) != 2) {
                            MSG("WARNING: [down] bad datr, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done;
                        }
                        switch (x0) {
                            case  5: txpkt.datarate = DR_LORA_SF5;  break;
                            case  6: txpkt.datarate = DR_LORA_SF6;  break;
                            case  7: txpkt.datarate = DR_LORA_SF7;  break;
                            case  8: txpkt.datarate = DR_LORA_SF8;  break;
                            case  9: txpkt.datarate = DR_LORA_SF9;  break;
                            case 10: txpkt.datarate = DR_LORA_SF10; break;
                            case 11: txpkt.datarate = DR_LORA_SF11; break;
                            case 12: txpkt.datarate = DR_LORA_SF12; break;
                            default: MSG("WARNING: [down] bad SF, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done;
                        }
                        switch (x1) {
                            case 125: txpkt.bandwidth = BW_125KHZ; break;
                            case 250: txpkt.bandwidth = BW_250KHZ; break;
                            case 500: txpkt.bandwidth = BW_500KHZ; break;
                            default:  MSG("WARNING: [down] bad BW, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done;
                        }
                        str = json_object_get_string(txpk_obj, "codr");
                        if (str == NULL) { MSG("WARNING: [down] no codr, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done; }
                        if      (strcmp(str,"4/5")==0) txpkt.coderate = CR_LORA_4_5;
                        else if (strcmp(str,"4/6")==0 || strcmp(str,"2/3")==0) txpkt.coderate = CR_LORA_4_6;
                        else if (strcmp(str,"4/7")==0) txpkt.coderate = CR_LORA_4_7;
                        else if (strcmp(str,"4/8")==0 || strcmp(str,"1/2")==0) txpkt.coderate = CR_LORA_4_8;
                        else { MSG("WARNING: [down] bad codr, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done; }
                        val = json_object_get_value(txpk_obj, "ipol");
                        if (val != NULL) txpkt.invert_pol = (bool)json_value_get_boolean(val);
                        val = json_object_get_value(txpk_obj, "prea");
                        txpkt.preamble = (val != NULL && (int)json_value_get_number(val) >= MIN_LORA_PREAMB)
                                       ? (uint16_t)json_value_get_number(val) : (uint16_t)STD_LORA_PREAMB;
                    } else {
                        MSG("WARNING: [down] unsupported modulation \"%s\", TX aborted\n", str);
                        json_value_free(root_val); root_val = NULL; goto pull_resp_done;
                    }
                    /* size + payload */
                    val = json_object_get_value(txpk_obj, "size");
                    if (val == NULL) { MSG("WARNING: [down] no size, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done; }
                    txpkt.size = (uint16_t)json_value_get_number(val);
                    str = json_object_get_string(txpk_obj, "data");
                    if (str == NULL) { MSG("WARNING: [down] no data, TX aborted\n"); json_value_free(root_val); root_val = NULL; goto pull_resp_done; }
                    b64_to_bin(str, strlen(str), txpkt.payload, sizeof txpkt.payload);
                    json_value_free(root_val); root_val = NULL;

                    txpkt.tx_mode = sent_immediate ? IMMEDIATE : TIMESTAMPED;

                    pthread_mutex_lock(&mx_meas_dw);
                    meas_dw_dgram_rcv    += 1;
                    meas_dw_network_byte += msg_len;
                    meas_dw_payload_byte += txpkt.size;
                    pthread_mutex_unlock(&mx_meas_dw);

                    jit_result = warning_result = JIT_ERROR_OK;
                    warning_value = 0;
                    if ((txpkt.freq_hz < tx_freq_min[txpkt.rf_chain]) || (txpkt.freq_hz > tx_freq_max[txpkt.rf_chain])) {
                        jit_result = JIT_ERROR_TX_FREQ;
                        MSG("ERROR: [down] PULL_RESP REJECTED: bad TX freq %u\n", txpkt.freq_hz);
                    }
                    if (jit_result == JIT_ERROR_OK) {
                        i = get_tx_gain_lut_index(txpkt.rf_chain, txpkt.rf_power, &tx_lut_idx);
                        if ((i < 0) || (txlut[txpkt.rf_chain].lut[tx_lut_idx].rf_power != txpkt.rf_power)) {
                            warning_result = JIT_ERROR_TX_POWER;
                            warning_value  = (int32_t)txlut[txpkt.rf_chain].lut[tx_lut_idx].rf_power;
                            txpkt.rf_power = txlut[txpkt.rf_chain].lut[tx_lut_idx].rf_power;
                        }
                    }
                    if (jit_result == JIT_ERROR_OK) {
                        pthread_mutex_lock(&mx_concent);
                        lgw_get_instcnt(&current_concentrator_time);
                        pthread_mutex_unlock(&mx_concent);
                        jit_result = jit_enqueue(&jit_queue[txpkt.rf_chain], current_concentrator_time, &txpkt, downlink_type);
                        if (jit_result != JIT_ERROR_OK) {
                            MSG("ERROR: [down] PULL_RESP enqueue failed (jit=%d)\n", jit_result);
                        } else {
                            jit_result = warning_result;
                        }
                        pthread_mutex_lock(&mx_meas_dw);
                        meas_nb_tx_requested += 1;
                        pthread_mutex_unlock(&mx_meas_dw);
                    }
                    send_tx_ack(buff_down[1], buff_down[2], jit_result, warning_value);
                    pull_resp_done:;
                }
            } /* end lorawan_fwd_enabled datagram handling */

            /* Check for ClusterDuck downlinks inside inner loop for faster response */
            int duck_rc_inner = cdp_bridge_pop_downlink(duck_payload_buf, &buf_capacity,
                                                         &dl_freq_hz, &dl_tmst, &dl_tx_power_dbm,
                                                         &dl_bw_hz, &dl_sf, &dl_cr, &dl_rf_chain);
            
            if (duck_rc_inner == 0 && buf_capacity > 0) {
                MSG("INFO: ClusterDuck downlink received (inner loop), size=%u\n", buf_capacity);
                /* No nanosleep needed: JIT CLASS_C schedules TX at current_time + 2*TX_JIT_DELAY (~80ms),
                 * which is well beyond the MamaDuck SX1262 RX turnaround (~5-10ms). Sleeping here
                 * would block LoRaWAN PULL_RESP handling in this same loop. */
                
                /* Build and enqueue packet - same code as outer loop */
                memset(&txpkt, 0, sizeof(txpkt));
                memcpy(txpkt.payload, duck_payload_buf, buf_capacity);
                txpkt.size = buf_capacity;
                /* Reply on the same frequency the RREQ was received on.
                 * last_rx_freq_hz is stored in DuckLoRa.cpp and passed via enqueue_downlink_ext. */
                txpkt.freq_hz = (dl_freq_hz > 0) ? dl_freq_hz : CDPCFG_RF_LORA_FREQ_HZ;
                MSG("INFO: ClusterDuck downlink TX freq=%u Hz\n", txpkt.freq_hz);
                
                /* Always get fresh timestamp - ClusterDuck packets are always IMMEDIATE */
                pthread_mutex_lock(&mx_concent);
                lgw_get_instcnt(&current_concentrator_time);
                pthread_mutex_unlock(&mx_concent);
                
                /* For ClusterDuck packets, use immediate transmission */
                /* Let JIT queue handle collision avoidance automatically */
                txpkt.count_us = 0;  /* 0 means ASAP - JIT will schedule avoiding collisions */
                txpkt.tx_mode = IMMEDIATE;  /* Use IMMEDIATE mode - JIT will find next available slot */
                downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_C;  /* CLASS_C calculates ASAP timing */
                
                /* Configure radio parameters */
                txpkt.rf_chain = 0;
                txpkt.rf_power = CDPCFG_RF_LORA_TXPOW;
                txpkt.modulation = MOD_LORA;
                txpkt.bandwidth = BW_125KHZ;
                txpkt.datarate = DR_LORA_SF7;
                txpkt.coderate = CR_LORA_4_5;
                txpkt.invert_pol = false; /* CDP is not LoRaWAN — MamaDuck uses normal (non-inverted) IQ */
                txpkt.preamble = 8;
                txpkt.no_crc = false;
                txpkt.no_header = false;
                
                MSG("DEBUG: bandwidth=%u (BW_125KHZ=%u), datarate=%u, coderate=%u, tx_mode=%s, count_us=%u\n", 
                    txpkt.bandwidth, BW_125KHZ, txpkt.datarate, txpkt.coderate,
                    (txpkt.tx_mode == IMMEDIATE) ? "IMMEDIATE" : "TIMESTAMPED", txpkt.count_us);
                
                jit_result = jit_enqueue(&jit_queue[txpkt.rf_chain], current_concentrator_time, &txpkt, downlink_type);
                if (jit_result == JIT_ERROR_OK) {
                    MSG("INFO: ClusterDuck packet enqueued to JIT queue\n");
                    pthread_mutex_lock(&mx_meas_dw);
                    meas_nb_tx_requested += 1;
                    pthread_mutex_unlock(&mx_meas_dw);
                } else {
                    MSG("ERROR: ClusterDuck packet rejected by JIT queue: %d\n", jit_result);
                }
                
                /* Reset buffer capacity */
                buf_capacity = sizeof(duck_payload_buf);
            }

            /* Pace the inner poll loop: the original Semtech thread_down was paced by
             * recv(sock_down) which blocks up to PULL_TIMEOUT_MS per call.  That recv
             * was removed in the CDP-only port (no NS socket exists), so without an
             * explicit sleep here the loop busy-spins at millions of iterations/second,
             * pegging one CPU core at 100%.  Also advance recv_time so the while
             * condition (difftimespec < keepalive_time) can eventually become false
             * and we re-send the keepalive (PULL_DATA) at the right interval. */
            wait_ms(PULL_TIMEOUT_MS);
            clock_gettime(CLOCK_MONOTONIC, &recv_time);
        }
    }
    MSG("\nINFO: End of downstream thread\n");
}

void print_tx_status(uint8_t tx_status) {
    switch (tx_status) {
        case TX_OFF:
            MSG("INFO: [jit] lgw_status returned TX_OFF\n");
            break;
        case TX_FREE:
            MSG("INFO: [jit] lgw_status returned TX_FREE\n");
            break;
        case TX_EMITTING:
            MSG("INFO: [jit] lgw_status returned TX_EMITTING\n");
            break;
        case TX_SCHEDULED:
            MSG("INFO: [jit] lgw_status returned TX_SCHEDULED\n");
            break;
        default:
            MSG("INFO: [jit] lgw_status returned UNKNOWN (%d)\n", tx_status);
            break;
    }
}


/* -------------------------------------------------------------------------- */
/* --- THREAD 3: CHECKING PACKETS TO BE SENT FROM JIT QUEUE AND SEND THEM --- */

void thread_jit(void) {
    int result = LGW_HAL_SUCCESS;
    struct lgw_pkt_tx_s pkt;
    int pkt_index = -1;
    uint32_t current_concentrator_time;
    enum jit_error_e jit_result;
    enum jit_pkt_type_e pkt_type;
    uint8_t tx_status;
    int i;

    while (!exit_sig && !quit_sig) {
        wait_ms(10);

        for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
            /* transfer data and metadata to the concentrator, and schedule TX */
            pthread_mutex_lock(&mx_concent);
            lgw_get_instcnt(&current_concentrator_time);
            pthread_mutex_unlock(&mx_concent);
            jit_result = jit_peek(&jit_queue[i], current_concentrator_time, &pkt_index);
            if (jit_result == JIT_ERROR_OK) {
                if (pkt_index > -1) {
                    jit_result = jit_dequeue(&jit_queue[i], pkt_index, &pkt, &pkt_type);
                    if (jit_result == JIT_ERROR_OK) {
                        /* update beacon stats */
                        if (pkt_type == JIT_PKT_TYPE_BEACON) {
                            /* Compensate breacon frequency with xtal error */
                            pthread_mutex_lock(&mx_xcorr);
                            pkt.freq_hz = (uint32_t)(xtal_correct * (double)pkt.freq_hz);
                            MSG_DEBUG(DEBUG_BEACON, "beacon_pkt.freq_hz=%u (xtal_correct=%.15lf)\n", pkt.freq_hz, xtal_correct);
                            pthread_mutex_unlock(&mx_xcorr);

                            /* Update statistics */
                            pthread_mutex_lock(&mx_meas_dw);
                            meas_nb_beacon_sent += 1;
                            pthread_mutex_unlock(&mx_meas_dw);
                            MSG("INFO: Beacon dequeued (count_us=%u)\n", pkt.count_us);
                        }

                        /* check if concentrator is free for sending new packet */
                        pthread_mutex_lock(&mx_concent); /* may have to wait for a fetch to finish */
                        result = lgw_status(pkt.rf_chain, TX_STATUS, &tx_status);
                        pthread_mutex_unlock(&mx_concent); /* free concentrator ASAP */
                        if (result == LGW_HAL_ERROR) {
                            MSG("WARNING: [jit%d] lgw_status failed\n", i);
                        } else {
                            if (tx_status == TX_EMITTING) {
                                MSG("ERROR: concentrator is currently emitting on rf_chain %d\n", i);
                                print_tx_status(tx_status);
                                continue;
                            } else if (tx_status == TX_SCHEDULED) {
                                MSG("WARNING: a downlink was already scheduled on rf_chain %d, overwritting it...\n", i);
                                print_tx_status(tx_status);
                            } else {
                                /* Nothing to do */
                            }
                        }

                        /* send packet to concentrator */
                        pthread_mutex_lock(&mx_concent); /* may have to wait for a fetch to finish */
                        if (spectral_scan_params.enable == true) {
                            result = lgw_spectral_scan_abort();
                            if (result != LGW_HAL_SUCCESS) {
                                MSG("WARNING: [jit%d] lgw_spectral_scan_abort failed\n", i);
                            }
                        }
                        result = lgw_send(&pkt);
                        pthread_mutex_unlock(&mx_concent); /* free concentrator ASAP */
                        if (result != LGW_HAL_SUCCESS) {
                            pthread_mutex_lock(&mx_meas_dw);
                            meas_nb_tx_fail += 1;
                            pthread_mutex_unlock(&mx_meas_dw);
                            MSG("WARNING: [jit] lgw_send failed on rf_chain %d\n", i);
                            continue;
                        } else {
                            pthread_mutex_lock(&mx_meas_dw);
                            meas_nb_tx_ok += 1;
                            pthread_mutex_unlock(&mx_meas_dw);
                            MSG_DEBUG(DEBUG_PKT_FWD, "lgw_send done on rf_chain %d: count_us=%u\n", i, pkt.count_us);
                        }
                    } else {
                        MSG("ERROR: jit_dequeue failed on rf_chain %d with %d\n", i, jit_result);
                    }
                }
            } else if (jit_result == JIT_ERROR_EMPTY) {
                /* Do nothing, it can happen */
            } else {
                MSG("ERROR: jit_peek failed on rf_chain %d with %d\n", i, jit_result);
            }
        }
    }

    MSG("\nINFO: End of JIT thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 4: PARSE GPS MESSAGE AND KEEP GATEWAY IN SYNC ----------------- */

static void gps_process_sync(void) {
    struct timespec gps_time;
    struct timespec utc;
    uint32_t trig_tstamp; /* concentrator timestamp associated with PPM pulse */
    int i = lgw_gps_get(&utc, &gps_time, NULL, NULL);

    /* get GPS time for synchronization */
    if (i != LGW_GPS_SUCCESS) {
        MSG("WARNING: [gps] could not get GPS time from GPS\n");
        return;
    }

    /* get timestamp captured on PPM pulse  */
    pthread_mutex_lock(&mx_concent);
    i = lgw_get_trigcnt(&trig_tstamp);
    pthread_mutex_unlock(&mx_concent);
    if (i != LGW_HAL_SUCCESS) {
        MSG("WARNING: [gps] failed to read concentrator timestamp\n");
        return;
    }

    /* try to update time reference with the new GPS time & timestamp */
    pthread_mutex_lock(&mx_timeref);
    i = lgw_gps_sync(&time_reference_gps, trig_tstamp, utc, gps_time);
    pthread_mutex_unlock(&mx_timeref);
    if (i != LGW_GPS_SUCCESS) {
        MSG("WARNING: [gps] GPS out of sync, keeping previous time reference\n");
    }
}

static void gps_process_coords(void) {
    /* position variable */
    struct coord_s coord;
    struct coord_s gpserr;
    int    i = lgw_gps_get(NULL, NULL, &coord, &gpserr);

    /* update gateway coordinates */
    pthread_mutex_lock(&mx_meas_gps);
    if (i == LGW_GPS_SUCCESS) {
        gps_coord_valid = true;
        meas_gps_coord = coord;
        meas_gps_err = gpserr;
        // TODO: report other GPS statistics (typ. signal quality & integrity)
    } else {
        gps_coord_valid = false;
    }
    pthread_mutex_unlock(&mx_meas_gps);
}

void thread_gps(void) {
    /* serial variables */
    char serial_buff[128]; /* buffer to receive GPS data */
    size_t wr_idx = 0;     /* pointer to end of chars in buffer */

    /* variables for PPM pulse GPS synchronization */
    enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */

    /* initialize some variables before loop */
    memset(serial_buff, 0, sizeof serial_buff);

    while (!exit_sig && !quit_sig) {
        size_t rd_idx = 0;
        size_t frame_end_idx = 0;

        /* blocking non-canonical read on serial port */
        ssize_t nb_char = read(gps_tty_fd, serial_buff + wr_idx, LGW_GPS_MIN_MSG_SIZE);
        if (nb_char <= 0) {
            MSG("WARNING: [gps] read() returned value %zd\n", nb_char);
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
                        MSG("WARNING: [gps] could not get a valid message from GPS (no time)\n");
                        frame_size = 0;
                    } else if (latest_msg == UBX_NAV_TIMEGPS) {
                        gps_process_sync();
                    }
                }
            } else if (serial_buff[rd_idx] == (char)LGW_GPS_NMEA_SYNC_CHAR) {
                /************************
                 * Found NMEA sync char *
                 ************************/
                /* scan for NMEA end marker (LF = 0x0a) */
                char* nmea_end_ptr = memchr(&serial_buff[rd_idx],(int)0x0a, (wr_idx - rd_idx));

                if(nmea_end_ptr) {
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
          memcpy(serial_buff, &serial_buff[frame_end_idx], wr_idx - frame_end_idx);
          wr_idx -= frame_end_idx;
        } /* ...for(rd_idx = 0... */

        /* Prevent buffer overflow */
        if ((sizeof(serial_buff) - wr_idx) < LGW_GPS_MIN_MSG_SIZE) {
            memcpy(serial_buff, &serial_buff[LGW_GPS_MIN_MSG_SIZE], wr_idx - LGW_GPS_MIN_MSG_SIZE);
            wr_idx -= LGW_GPS_MIN_MSG_SIZE;
        }
    }
    MSG("\nINFO: End of GPS thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 5: CHECK TIME REFERENCE AND CALCULATE XTAL CORRECTION --------- */

void thread_valid(void) {

    /* GPS reference validation variables */
    long gps_ref_age = 0;
    bool ref_valid_local = false;
    double xtal_err_cpy;

    /* variables for XTAL correction averaging */
    unsigned init_cpt = 0;
    double init_acc = 0.0;
    double x;

    /* correction debug */
    FILE * log_file = NULL;
    time_t now_time;
    char log_name[64];

    /* initialization */
    time(&now_time);
    strftime(log_name,sizeof log_name,"xtal_err_%Y%m%dT%H%M%SZ.csv",localtime(&now_time));
    log_file = fopen(log_name, "w");
    setbuf(log_file, NULL);
    fprintf(log_file,"\"xtal_correct\",\"XERR_INIT_AVG %u XERR_FILT_COEF %u\"\n", XERR_INIT_AVG, XERR_FILT_COEF); // DEBUG

    /* main loop task */
    while (!exit_sig && !quit_sig) {
        wait_ms(1000);

        /* calculate when the time reference was last updated */
        pthread_mutex_lock(&mx_timeref);
        gps_ref_age = (long)difftime(time(NULL), time_reference_gps.systime);
        if ((gps_ref_age >= 0) && (gps_ref_age <= GPS_REF_MAX_AGE)) {
            /* time ref is ok, validate and  */
            gps_ref_valid = true;
            ref_valid_local = true;
            xtal_err_cpy = time_reference_gps.xtal_err;
            //printf("XTAL err: %.15lf (1/XTAL_err:%.15lf)\n", xtal_err_cpy, 1/xtal_err_cpy); // DEBUG
        } else {
            /* time ref is too old, invalidate */
            gps_ref_valid = false;
            ref_valid_local = false;
        }
        pthread_mutex_unlock(&mx_timeref);

        /* manage XTAL correction */
        if (ref_valid_local == false) {
            /* couldn't sync, or sync too old -> invalidate XTAL correction */
            pthread_mutex_lock(&mx_xcorr);
            xtal_correct_ok = false;
            xtal_correct = 1.0;
            pthread_mutex_unlock(&mx_xcorr);
            init_cpt = 0;
            init_acc = 0.0;
        } else {
            if (init_cpt < XERR_INIT_AVG) {
                /* initial accumulation */
                init_acc += xtal_err_cpy;
                ++init_cpt;
            } else if (init_cpt == XERR_INIT_AVG) {
                /* initial average calculation */
                pthread_mutex_lock(&mx_xcorr);
                xtal_correct = (double)(XERR_INIT_AVG) / init_acc;
                //printf("XERR_INIT_AVG=%d, init_acc=%.15lf\n", XERR_INIT_AVG, init_acc);
                xtal_correct_ok = true;
                pthread_mutex_unlock(&mx_xcorr);
                ++init_cpt;
                // fprintf(log_file,"%.18lf,\"average\"\n", xtal_correct); // DEBUG
            } else {
                /* tracking with low-pass filter */
                x = 1 / xtal_err_cpy;
                pthread_mutex_lock(&mx_xcorr);
                xtal_correct = xtal_correct - xtal_correct/XERR_FILT_COEF + x/XERR_FILT_COEF;
                pthread_mutex_unlock(&mx_xcorr);
                // fprintf(log_file,"%.18lf,\"track\"\n", xtal_correct); // DEBUG
            }
        }

        //printf("Time ref: %s, XTAL correct: %s (%.15lf)\n", ref_valid_local?"valid":"invalid", xtal_correct_ok?"valid":"invalid", xtal_correct); // DEBUG
    }
    MSG("\nINFO: End of validation thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 6: BACKGROUND SPECTRAL SCAN                           --------- */

void thread_spectral_scan(void) {
    int i, x;
    uint32_t freq_hz = spectral_scan_params.freq_hz_start;
    uint32_t freq_hz_stop = spectral_scan_params.freq_hz_start + spectral_scan_params.nb_chan * 200E3;
    int16_t levels[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    uint16_t results[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    struct timeval tm_start;
    lgw_spectral_scan_status_t status;
    uint8_t tx_status = TX_FREE;
    bool spectral_scan_started;
    bool exit_thread = false;

    /* main loop task */
    while (!exit_sig && !quit_sig) {
        /* Pace the scan thread (1 sec min), and avoid waiting several seconds when exit */
        for (i = 0; i < (int)(spectral_scan_params.pace_s ? spectral_scan_params.pace_s : 1); i++) {
            if (exit_sig || quit_sig) {
                exit_thread = true;
                break;
            }
            wait_ms(1000);
        }
        if (exit_thread == true) {
            break;
        }

        spectral_scan_started = false;

        /* Start spectral scan (if no downlink programmed) */
        pthread_mutex_lock(&mx_concent);
        /* -- Check if there is a downlink programmed */
        for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
            if (tx_enable[i] == true) {
                x = lgw_status((uint8_t)i, TX_STATUS, &tx_status);
                if (x != LGW_HAL_SUCCESS) {
                    printf("ERROR: failed to get TX status on chain %d\n", i);
                } else {
                    if (tx_status == TX_SCHEDULED || tx_status == TX_EMITTING) {
                        printf("INFO: skip spectral scan (downlink programmed on RF chain %d)\n", i);
                        break; /* exit for loop */
                    }
                }
            }
        }
        if (tx_status != TX_SCHEDULED && tx_status != TX_EMITTING) {
            x = lgw_spectral_scan_start(freq_hz, spectral_scan_params.nb_scan);
            if (x != 0) {
                printf("ERROR: spectral scan start failed\n");
                pthread_mutex_unlock(&mx_concent);
                continue; /* main while loop */
            }
            spectral_scan_started = true;
        }
        pthread_mutex_unlock(&mx_concent);

        if (spectral_scan_started == true) {
            /* Wait for scan to be completed */
            status = LGW_SPECTRAL_SCAN_STATUS_UNKNOWN;
            timeout_start(&tm_start);
            do {
                /* handle timeout */
                if (timeout_check(tm_start, 2000) != 0) {
                    printf("ERROR: %s: TIMEOUT on Spectral Scan\n", __FUNCTION__);
                    break;  /* do while */
                }

                /* get spectral scan status */
                pthread_mutex_lock(&mx_concent);
                x = lgw_spectral_scan_get_status(&status);
                pthread_mutex_unlock(&mx_concent);
                if (x != 0) {
                    printf("ERROR: spectral scan status failed\n");
                    break; /* do while */
                }

                /* wait a bit before checking status again */
                wait_ms(10);
            } while (status != LGW_SPECTRAL_SCAN_STATUS_COMPLETED && status != LGW_SPECTRAL_SCAN_STATUS_ABORTED);

            if (status == LGW_SPECTRAL_SCAN_STATUS_COMPLETED) {
                /* Get spectral scan results */
                memset(levels, 0, sizeof levels);
                memset(results, 0, sizeof results);
                pthread_mutex_lock(&mx_concent);
                x = lgw_spectral_scan_get_results(levels, results);
                pthread_mutex_unlock(&mx_concent);
                if (x != 0) {
                    printf("ERROR: spectral scan get results failed\n");
                    continue; /* main while loop */
                }

                /* print results */
                printf("SPECTRAL SCAN - %u Hz: ", freq_hz);
                for (i = 0; i < LGW_SPECTRAL_SCAN_RESULT_SIZE; i++) {
                    printf("%u ", results[i]);
                }
                printf("\n");

                /* Next frequency to scan */
                freq_hz += 200000; /* 200kHz channels */
                if (freq_hz >= freq_hz_stop) {
                    freq_hz = spectral_scan_params.freq_hz_start;
                }
            } else if (status == LGW_SPECTRAL_SCAN_STATUS_ABORTED) {
                printf("INFO: %s: spectral scan has been aborted\n", __FUNCTION__);
            } else {
                printf("ERROR: %s: spectral scan status us unexpected 0x%02X\n", __FUNCTION__, status);
            }
        }
    }
    printf("\nINFO: End of Spectral Scan thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD: CLUSTERDUCK PROTOCOL PROCESSING ------------------------------ */

void thread_duck(void) {
    MSG("INFO: ClusterDuck processing thread started\n");
    
    /* Give other threads time to initialize */
    wait_ms(1000);
    MSG("INFO: ClusterDuck thread starting main loop\n");
    
    /* Counter for periodic MQTT health check (every 30 seconds) */
    int mqtt_check_counter = 0;
    
    /* Main loop: call hub.main() to process ClusterDuck packets */
    while (!exit_sig && !quit_sig) {
        /* Block until a CDP packet is queued or 100 ms elapses (whichever
         * comes first).  Using a condition variable here means packets are
         * processed with sub-millisecond latency instead of waiting up to
         * 100 ms for the old fixed sleep to expire. */
        cdp_bridge_wait_for_uplink(100);

        /* Call the ClusterDuck main loop to process received packets */
        hub_run_loop();

        /* Periodic MQTT connection health check */
        if (mqtt_enabled) {
            mqtt_check_counter++;
            
            // Check more frequently if we have queued messages (every 0.5 seconds)
            // Otherwise check every 30 seconds
            int check_interval = (mqtt_queue_count > 0) ? 5 : 300;  // 5 * 100ms = 0.5s, 300 * 100ms = 30s
            
            if (mqtt_check_counter >= check_interval) {
                mqtt_check_counter = 0;
                
                // Check if connection is still alive or needs reconnection
                if (mqtt_needs_reconnect || (mqtt_client != NULL && !MQTTClient_isConnected(mqtt_client))) {
                    MSG("WARNING: [MQTT] Connection lost or needs reconnect (queue count=%d), attempting reconnection...\n", mqtt_queue_count);
                    int result = mqtt_reconnect_with_backoff();
                    if (result == 0) {
                        MSG("[MQTT] Reconnection successful from health check\n");
                    }
                } else if (mqtt_queue_count > 0 && mqtt_client != NULL && MQTTClient_isConnected(mqtt_client)) {
                    // Connection is healthy but we have queued messages - publish them
                    MSG("INFO: [MQTT] Publishing %d queued message(s)...\n", mqtt_queue_count);
                    mqtt_publish_queued_messages();
                }
            }
        }
    }
    
    MSG("\nINFO: End of ClusterDuck processing thread\n");
}

/* --- EOF ------------------------------------------------------------------ */

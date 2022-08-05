/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator : Packet Forwarder trace helpers

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORA_PKTFWD_TRACE_H
#define _LORA_PKTFWD_TRACE_H

#define DEBUG_PKT_FWD   0
#define DEBUG_JIT       0
#define DEBUG_JIT_ERROR 1
#define DEBUG_TIMERSYNC 0
#define DEBUG_BEACON    0
#define DEBUG_LOG       0

#define MSG(args...) printf(args); fflush(stdout) /* message that is destined to the user */
#define MSG_DEBUG(FLAG, fmt, ...)                                                                         \
            do  {                                                                                         \
                if (FLAG) {                                                                               \
                    fprintf(stdout, "%s:%d:%s(): " fmt, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__); \
                    fflush(stdout);                                                                       \
                }                                                                                         \
            } while (0)
#define MSG_PRINTF(FLAG, fmt, ...)                                                                         \
            do  {                                                                                         \
                if (FLAG) {                                                                               \
                    fprintf(stdout, fmt, ##__VA_ARGS__); \
                    fflush(stdout);                                                                       \
                }                                                                                         \
            } while (0)

#endif
/* --- EOF ------------------------------------------------------------------ */

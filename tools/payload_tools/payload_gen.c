#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "tinymt32.h"

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void usage(void);
void remove_spaces(char *str);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    int j;
    uint8_t dev_id[4];
    uint8_t payload[255];
    uint8_t payload_size;
    unsigned int packet_cnt;
    tinymt32_t tinymt;
    char hexstr[32];

    if (argc < 4) {
        usage();
        return -1;
    }

    /* Get dev_id hex string from command line */
    memcpy(hexstr, argv[1], strlen(argv[1]));
    hexstr[strlen(argv[1])] = '\0';

    /* Remove spaces from the string if any */
    remove_spaces(hexstr);
    hexstr[strlen(hexstr)] = '\0';
    printf("Dev_id: %s\n", hexstr);

    /* Convert hex string to byte array */
    payload_size = strlen(hexstr) / 2;
    for (j = 0; j < 4; j++) {
        sscanf(hexstr + 2*j, "%02hhx", &dev_id[j]);
    }

    /* Get packet count from which generate the random payload */
    packet_cnt = atoi(argv[2]);

    /* Get packet payload size */
    payload_size = (uint8_t)atoi(argv[3]);

    /* Initialize the pseudo-random generator */
    tinymt.mat1 = 0x8f7011ee;
    tinymt.mat2 = 0xfc78ff1f;
    tinymt.tmat = 0x3793fdff;
    tinymt32_init(&tinymt, packet_cnt);

    /* Construct packet */
    payload[0] = dev_id[0];
    payload[1] = dev_id[1];
    payload[2] = dev_id[2];
    payload[3] = dev_id[3];
    payload[4] = (uint8_t)(packet_cnt >> 24);
    payload[5] = (uint8_t)(packet_cnt >> 16);
    payload[6] = (uint8_t)(packet_cnt >> 8);
    payload[7] = (uint8_t)(packet_cnt >> 0);
    for (j = 8; j < payload_size; j++) {
        payload[j] = (uint8_t)tinymt32_generate_uint32(&tinymt);
    }
    for (j = 0; j < payload_size; j++) {
        printf("%02X ", payload[j]);
    }
    printf("\n");

#if 0
    for (packet_cnt = 0; packet_cnt < 10; packet_cnt++) {
        tinymt32_init(&tinymt, (int)packet_cnt);
        payload[0] = 0xCA;
        payload[1] = 0xFE;
        payload[2] = 0x12;
        payload[3] = 0x34;
        payload[4] = (uint8_t)(packet_cnt >> 24);
        payload[5] = (uint8_t)(packet_cnt >> 16);
        payload[6] = (uint8_t)(packet_cnt >> 8);
        payload[7] = (uint8_t)(packet_cnt >> 0);
        for (j = 8; j < 16; j++) {
            payload[j] = (uint8_t)tinymt32_generate_uint32(&tinymt);
        }
        for (j = 0; j < 16; j++) {
            printf("%02X ", payload[j]);
        }
        printf("\n");
    }
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void usage(void) {
    printf("Missing parameters: ./payload_gen dev_id pkt_cnt pkt_size\n");
    printf("       dev_id: hex string for 4-bytes dev_id\n");
    printf("       pkt_cnt: unsigned int used to initialize the pseudo-random generator\n");
    printf("       pkt_size: paylaod size in bytes [0..255]\n");
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void remove_spaces(char *str)
{
    /* To keep track of non-space character count */
    int count = 0;

    /* Traverse the given string. If current character
        is not space, then place it at index 'count++' */
    for (int i = 0; str[i]; i++) {
        if (str[i] != ' ') {
            str[count++] = str[i]; /* here count is incremented */
        }
    }
    str[count] = '\0';
}

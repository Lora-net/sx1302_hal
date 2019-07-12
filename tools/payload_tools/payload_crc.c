#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void usage(void);
uint16_t sx1302_lora_payload_crc(const uint8_t * data, uint8_t size);
void remove_spaces(char *str);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    int j;
    uint8_t payload[255];
    uint8_t payload_size;
    uint16_t crc;
    char hexstr[1024];

    if (argc < 2) {
        usage();
        return -1;
    }

    /* Get payload hex string from command line */
    memcpy(hexstr, argv[1], strlen(argv[1]));
    hexstr[strlen(argv[1])] = '\0';
    printf("Input hex string: %s\n", hexstr);

    /* Remove spaces from the string if any */
    remove_spaces(hexstr);
    hexstr[strlen(hexstr)] = '\0';
    printf("Removing spaces: %s\n", hexstr);

    /* Convert hex string to byte array */
    payload_size = strlen(hexstr) / 2;
    for (j = 0; j < payload_size; j++) {
        sscanf(hexstr + 2*j, "%02hhx", &payload[j]);
    }

    /* Compute CRC */
    crc = sx1302_lora_payload_crc(payload, payload_size);
    printf("Payload CRC_16: %04X\n", crc);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void usage(void) {
    printf("Missing payload hex string\n");
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lora_crc16(const char data, int *crc) {
    int next = 0;
    next  =  (((data>>0)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1)                 )      ;
    next += ((((data>>1)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1)                 )<<1 ) ;
    next += ((((data>>2)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1)                 )<<2 ) ;
    next += ((((data>>3)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<3 ) ;
    next += ((((data>>4)&1) ^ ((*crc>>12)&1)                                  )<<4 ) ;
    next += ((((data>>5)&1) ^ ((*crc>>13)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1))<<5 ) ;
    next += ((((data>>6)&1) ^ ((*crc>>14)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1))<<6 ) ;
    next += ((((data>>7)&1) ^ ((*crc>>15)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1))<<7 ) ;
    next += ((((*crc>>0)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<8 ) ;
    next += ((((*crc>>1)&1) ^ ((*crc>>12)&1)                                  )<<9 ) ;
    next += ((((*crc>>2)&1) ^ ((*crc>>13)&1)                                  )<<10) ;
    next += ((((*crc>>3)&1) ^ ((*crc>>14)&1)                                  )<<11) ;
    next += ((((*crc>>4)&1) ^ ((*crc>>15)&1) ^ ((*crc>>12)&1) ^ ((*crc>> 8)&1))<<12) ;
    next += ((((*crc>>5)&1) ^ ((*crc>>13)&1) ^ ((*crc>> 9)&1)                 )<<13) ;
    next += ((((*crc>>6)&1) ^ ((*crc>>14)&1) ^ ((*crc>>10)&1)                 )<<14) ;
    next += ((((*crc>>7)&1) ^ ((*crc>>15)&1) ^ ((*crc>>11)&1)                 )<<15) ;
    (*crc) = next;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t sx1302_lora_payload_crc(const uint8_t * data, uint8_t size) {
    int i;
    int crc = 0;

    for (i = 0; i < size; i++) {
        lora_crc16(data[i], &crc);
    }

    //printf("CRC16: 0x%02X 0x%02X (%X)\n", (uint8_t)(crc >> 8), (uint8_t)crc, crc);
    return (uint16_t)crc;
}

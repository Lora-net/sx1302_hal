#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* --- MACROS --------------------------------------------------------------- */

#define TAKE_N_BITS_FROM(b, p, n) (((b) >> (p)) & ((1 << (n)) - 1))

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void usage(void);
void remove_spaces(char *str);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    int i, j;
    uint8_t payload_a[255];
    uint8_t payload_b[255];
    uint8_t payload_diff[255];
    uint8_t payload_size;
    char hexstr[1024];
    uint16_t nb_bits_diff = 0;

    if (argc < 3) {
        usage();
        return -1;
    }

    if (strlen(argv[1]) != strlen(argv[2])) {
        printf("ERROR: payloads A & B must have same size\n");
        return -1;
    }

    /* Get payload A hex string from command line */
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
        sscanf(hexstr + 2*j, "%02hhx", &payload_a[j]);
    }

    /* Get payload B hex string from command line */
    memcpy(hexstr, argv[2], strlen(argv[2]));
    hexstr[strlen(argv[2])] = '\0';
    printf("Input hex string: %s\n", hexstr);

    /* Remove spaces from the string if any */
    remove_spaces(hexstr);
    hexstr[strlen(hexstr)] = '\0';
    printf("Removing spaces: %s\n", hexstr);

    /* Convert hex string to byte array */
    for (j = 0; j < payload_size; j++) {
        sscanf(hexstr + 2*j, "%02hhx", &payload_b[j]);
    }

    /* Count how many bits differs */
    printf("Diff: ");
    for (j = 0; j < payload_size; j++) {
        payload_diff[j] = payload_a[j] ^ payload_b[j];
        printf("%02X ", payload_diff[j]);
    }
    printf("\n");

    for (j = 0; j < payload_size; j++) {
        for (i = 7; i >= 0; i--) {
            printf("%u", TAKE_N_BITS_FROM(payload_diff[j], i, 1));
            if (TAKE_N_BITS_FROM(payload_diff[j], i, 1) == 1) {
                nb_bits_diff += 1;
            }
        }
        printf(" ");
    }
    printf("\n");
    printf("%u bits flipped\n", nb_bits_diff);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void usage(void) {
    printf("Missing payload hex strings for a & b\n");
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


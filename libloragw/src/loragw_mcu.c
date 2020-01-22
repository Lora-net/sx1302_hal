/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa 2.4GHz concentrator MCU interface functions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcpy */
#include <stdlib.h>     /* rand */
#include <fcntl.h>      /* open/close */
#include <errno.h>      /* perror */
#include <unistd.h>     /* read, write */
#include <termios.h>    /* POSIX terminal control definitions */

#include "loragw_mcu.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_MCU == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr, fmt, args)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a<b;++a) fprintf(stderr,"%x.",c[a]);fprintf(stderr,"end\n")
    #define CHECK_NULL(a)                 if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return -1;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a!=0;){}
    #define CHECK_NULL(a)                 if(a==NULL){return -1;}
#endif

#define DEBUG_VERBOSE 0

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

/* Commands */
#define HEADER_CMD_SIZE  4
#define WRITE_SIZE_MAX 280
#define READ_SIZE_MAX 500

/*!
* \brief Represents the ramping time for radio power amplifier
*/
typedef enum {
    RADIO_RAMP_02_US = 0x00,
    RADIO_RAMP_04_US = 0x20,
    RADIO_RAMP_06_US = 0x40,
    RADIO_RAMP_08_US = 0x60,
    RADIO_RAMP_10_US = 0x80,
    RADIO_RAMP_12_US = 0xA0,
    RADIO_RAMP_16_US = 0xC0,
    RADIO_RAMP_20_US = 0xE0,
} RampTimes_t;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* Hardware */
static uint8_t nb_radio_rx = 0;
static uint8_t nb_radio_tx = 0;

static uint8_t buf_req[WRITE_SIZE_MAX];
static uint8_t buf_ack[READ_SIZE_MAX];

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

const char * cmd_get_str(const uint8_t cmd) {
    switch (cmd) {
        case ORDER_ID__REQ_PING:
            return "REQ_PING";
            
        case ORDER_ID__REQ_SPI:
            return "REQ_SPI";/*
        case ORDER_ID__REQ_PREPARE_TX:
            return "REQ_PREPARE_TX";
        case ORDER_ID__REQ_GET_STATUS:
            return "REQ_GET_STATUS";
        case ORDER_ID__REQ_BOOTLOADER_MODE:
            return "REQ_BOOTLOADER_MODE";
        case ORDER_ID__REQ_GET_RX_MSG:
            return "REQ_GET_RX_MSG";
        case ORDER_ID__REQ_GET_TX_STATUS:
            return "REQ_GET_TX_STATUS";
        case ORDER_ID__REQ_RESET:
            return "REQ_RESET";
        case ORDER_ID__REQ_SET_COEF_TEMP_RSSI:
            return "REQ_SET_COEF_TEMP_RSSI";
        case ORDER_ID__REQ_READ_REGS:
            return "ORDER_ID__REQ_READ_REGS";
        case ORDER_ID__REQ_WRITE_REGS:
            return "ORDER_ID__REQ_WRITE_REGS";*/
        default:
            return "UNKNOWN";
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t cmd_get_id(const uint8_t * bytes) {
    return bytes[0];
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t cmd_get_size(const uint8_t * bytes) {
    return (uint16_t)(bytes[1] << 8) | bytes[2];
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t cmd_get_type(const uint8_t * bytes) {
    return bytes[3];
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int write_req(int fd, e_order_cmd cmd, uint16_t size, const uint8_t * payload) {
    uint8_t buf_w[HEADER_CMD_SIZE];
    int n;

    /* Write command header */
    buf_w[0] = rand() % 255;
    buf_w[1] = (uint8_t)(size >> 8); /* MSB */
    buf_w[2] = (uint8_t)(size >> 0); /* LSB */
    buf_w[3] = cmd;
    n = write(fd, buf_w, HEADER_CMD_SIZE);
    if (n < 0) {
        printf("ERROR: failed to write command header to com port\n");
        return -1;
    }

    /* Write command payload */
    if (size > 0) {
        if (payload == NULL) {
            printf("ERROR: invalid payload\n");
            return -1;
        }
        n = write(fd, payload, size);
        if (n < 0) {
            printf("ERROR: failed to write command payload to com port\n");
            return -1;
        }
    }

    DEBUG_PRINTF("\nINFO: write_req 0x%02X (%s) done, id:0x%02X\n", cmd, cmd_get_str(cmd), buf_w[0]);

#if 0
    int i;
    printf("write_buf : ");
    for (i = 0; i < 4; i++) {
        printf("%02X ", buf_w[i]);
    }
    printf("write_payload : ");
    for (i = 0; i < size; i++) {
        printf("%02X ", payload[i]);
    }
    printf("\n");
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int read_ack(int fd, uint8_t * buf, size_t buf_size) {
    int i, n;
    size_t size;
    int nb_read = 0;

    /* Read message header first */
    n = read(fd, &buf[0], (size_t)HEADER_CMD_SIZE);
    if (errno == EINTR) {
        printf("INFO: syscall was interrupted, continue...\n");
        /* TODO: check what to do here */
        return -1;
    } else if (n == -1) {
        perror("ERROR: Unable to read /dev/ttyACMx - ");
        return -1;
    } else {
        DEBUG_PRINTF("INFO: read %d bytes for header from gateway\n", n);
        nb_read += n;
    }

    /* debug print */
#if 0
    printf("read header : ");
    for (i = 0; i < (int)(HEADER_CMD_SIZE); i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n");

#endif
    /* Get remaining payload size (metadata + pkt payload) */
    size  = (size_t)buf[CMD_OFFSET__SIZE_MSB] << 8;
    size |= (size_t)buf[CMD_OFFSET__SIZE_LSB] << 0;
    if (((size_t)HEADER_CMD_SIZE + size) > buf_size) {
        printf("ERROR: not enough memory to store all data (%zd) (buf_size : %d)\n", (size_t)HEADER_CMD_SIZE + size, buf_size);
        return -1;
    }

    /* Read payload if any */
    if (size > 0) {
        do {
            n = read(fd, &buf[nb_read], size - (nb_read - HEADER_CMD_SIZE));
            if (errno == EINTR) {
                printf("INFO: syscall was interrupted, continue...\n");
                /* TODO: check what to do here */
                return -1;
            } else if (n == -1) {
                perror("ERROR: Unable to read /dev/ttyACMx - ");
                return -1;
            } else {
                DEBUG_PRINTF("INFO: read %d bytes from gateway\n", n);
                nb_read += n;
            }
        } while ((nb_read - HEADER_CMD_SIZE) < (int)size); /* we want to read only the expected payload, not more */

        /* debug print */
#if 0
        printf("read buffer : ");
        for (i = HEADER_CMD_SIZE; i < (int)(HEADER_CMD_SIZE + size); i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
#endif
    }

    return nb_read;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_get_status(const uint8_t * payload, float * temperature) {//}, s_status * status) {
    int i;
    //int16_t temperature;

    /* sanity checks */
    if ((payload == NULL) ) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_GET_STATUS) {
        printf("ERROR: wrong ACK type for GET_STATUS (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_GET_STATUS, cmd_get_type(payload));
        return -1;
    }
    /*printf("ACK_GET_STATUS : \n");
    for (i = 0; i < 10; i++) {
        printf("    payload[%d]:  %x\n", i, payload[i]);
    }*/

    *temperature = (float) (payload[8]*256 + payload[9])/100;
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_ping(const uint8_t * payload, s_ping_info * info) {
    /* sanity checks */
    if ((payload == NULL) || (info == NULL)) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_PING) {
        printf("ERROR: wrong ACK type for PING (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_PING, cmd_get_type(payload));
        return -1;
    }

    /* payload info */
    info->unique_id_high = bytes_be_to_uint32_le(&payload[HEADER_CMD_SIZE + ACK_PING__UNIQUE_ID_0]);
    info->unique_id_mid  = bytes_be_to_uint32_le(&payload[HEADER_CMD_SIZE + ACK_PING__UNIQUE_ID_4]);
    info->unique_id_low  = bytes_be_to_uint32_le(&payload[HEADER_CMD_SIZE + ACK_PING__UNIQUE_ID_8]);

    memcpy(info->version, &payload[HEADER_CMD_SIZE + ACK_PING__VERSION_0], (sizeof info->version) - 1);
    info->version[(sizeof info->version) - 1] = '\0'; /* terminate string */

    info->nb_radio_tx = payload[HEADER_CMD_SIZE + ACK_PING__NB_RADIO_TX];

    info->nb_radio_rx = payload[HEADER_CMD_SIZE + ACK_PING__NB_RADIO_RX];

    /* store local context */
    nb_radio_rx = info->nb_radio_rx;
    nb_radio_tx = info->nb_radio_tx;

#if 1
    printf("## ACK_PING\n");
    printf("   id:           0x%02X\n", cmd_get_id(payload));
    printf("   size:         %u\n", cmd_get_size(payload));
    printf("   unique_id:    0x%08X%08X%08X\n", info->unique_id_high, info->unique_id_mid, info->unique_id_low);
    printf("   FW version:   %s\n", info->version);
    printf("   nb_radio_tx:  %u\n", info->nb_radio_tx);
    printf("   nb_radio_rx:  %u\n", info->nb_radio_rx);
#endif

    return 0;
}


int decode_ack_gpio_access(const uint8_t * payload) {
    uint8_t status;
    if ((payload == NULL)) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_WRITE_GPIO) {
        printf("ERROR: wrong ACK type for WRITE_GPIO (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_WRITE_GPIO, cmd_get_type(payload));
        return -1;
    }
    status = payload[HEADER_CMD_SIZE + ACK_GPIO_WRITE__STATUS];
    if (status == 0) {
        return 0;
    }
    else 
    {
        return -1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_reset(const uint8_t * payload, uint8_t * reset_status) {
     /* sanity checks */
    if (payload == NULL) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_RESET) {
        printf("ERROR: wrong ACK type for ACK_RESET (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_RESET, cmd_get_type(payload));
        return -1;
    }

    /* payload info */
    *reset_status = payload[HEADER_CMD_SIZE + ACK_RESET__STATUS];

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_RESET\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(payload));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(payload));
    DEBUG_PRINTF("   status:       %u\n", *reset_status);
#endif

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int decode_ack_bootloader_mode(const uint8_t * payload) {
     /* sanity checks */
    if (payload == NULL) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_BOOTLOADER_MODE) {
        printf("ERROR: wrong ACK type for ACK_BOOTLOADER_MODE (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_BOOTLOADER_MODE, cmd_get_type(payload));
        return -1;
    }

#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_BOOTLOADER_MODE\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(payload));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(payload));
#endif

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int mcu_open(const char * tty_path) {
    int fd;
    struct termios tty;

    fd = open(tty_path, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("ERROR: Unable to open tty_path - ");
    } else {
        memset(&tty, 0, sizeof tty);

        /* Get current attributes */
        if (tcgetattr(fd, &tty) != 0) {
            printf("ERROR: tcgetattr failed with %d - %s", errno, strerror(errno));
            return -1;
        }

        cfsetospeed(&tty, 115200);
        cfsetispeed(&tty, 115200);

        /* Control Modes */
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* set 8-bit characters */
        tty.c_cflag |= CLOCAL;                      /* local connection, no modem control */
        tty.c_cflag |= CREAD;                       /* enable receiving characters */
        tty.c_cflag &= ~PARENB;                     /* no parity */
        tty.c_cflag &= ~CSTOPB;                     /* one stop bit */
        /* Input Modes */
        tty.c_iflag &= ~IGNBRK;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        /* Output Modes */
        tty.c_oflag &= ~IGNBRK;
        tty.c_oflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        /* Local Modes */
        tty.c_lflag = 0;
        /* Settings for non-canonical mode */
        /* set blocking mode, need at least n char to return */
        tty.c_cc[VMIN] = HEADER_CMD_SIZE;   /* x bytes minimum for full message header */
        tty.c_cc[VTIME] = 1;                /* 100ms */

        /* set attributes */
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            DEBUG_PRINTF("ERROR: tcsetattr(TCSANOW) failed with %d - %s", errno, strerror(errno));
            return -1;
        }

        /* flush input/ouput queues */
        wait_ms(100);
        if (tcflush(fd, TCIOFLUSH) != 0) {
            DEBUG_PRINTF("ERROR: tcflush failed with %d - %s", errno, strerror(errno));
            return -1;
        }
    }

    /* Initialize pseudo-randoml generator for request ID */
    srand(0);

    return fd;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_close(int fd) {
    return close(fd);
}




/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_ping(int fd, s_ping_info * info) {
    CHECK_NULL(info);
    if (write_req(fd, ORDER_ID__REQ_PING, 0, NULL) != 0) {
        printf("ERROR: failed to write PING request\n");
        return -1;
    }

    if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read PING ack\n");
        return -1;
    }

    if (decode_ack_ping(buf_ack, info) != 0) {
        printf("ERROR: invalid PING ack\n");
        return -1;
    }

    return 0;
}


int mcu_get_status(int fd, float * temperature) { //}, s_status * status) {

    if (write_req(fd, ORDER_ID__REQ_GET_STATUS, 0, NULL) != 0) {
        printf("ERROR: failed to write GET_STATUS request\n");
        return -1;
    }

    printf("NOTE: GET_STATUS read ack\n");
    if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read GET_STATUS ack\n");
        return -1;
    }

    printf("NOTE: GET_STATUS decode ack\n");
    if (decode_ack_get_status(buf_ack, temperature) != 0) { //, status) != 0) {
        printf("ERROR: invalid GET_STATUS ack\n");
        return -1;
    }

    return 0;
}


int decode_ack_spi_access(const uint8_t * payload, uint8_t * reg_value) { 
    /* sanity checks */
    if (payload == NULL) {
        printf("ERROR: invalid parameter\n");
        return -1;
    }

    if (cmd_get_type(payload) != ORDER_ID__ACK_SPI) {
        printf("ERROR: wrong ACK type for ACK_SPI (expected:0x%02X, got 0x%02X)\n", ORDER_ID__ACK_SPI, cmd_get_type(payload));
        return -1;
    }

    /* payload info */
    *reg_value = payload[HEADER_CMD_SIZE + ACK_SPI__VALUE];

    
#if DEBUG_VERBOSE
    DEBUG_MSG   ("## ACK_SPI_ACCESS\n");
    DEBUG_PRINTF("   id:           0x%02X\n", cmd_get_id(payload));
    DEBUG_PRINTF("   size:         %u\n", cmd_get_size(payload));
#endif

    return 0;
}


int mcu_spi_access(int fd, uint8_t * buf, size_t buf_size,uint8_t * buf_ack) {


    if (write_req(fd, ORDER_ID__REQ_SPI, buf_size, buf) != 0) {
        printf("ERROR: failed to write REQ_READ_REGS request\n");
        return -1;
    }

    if (read_ack(fd, buf_ack, buf_size+4) < 0) {
        printf("ERROR: failed to read REQ_READ_REGS ack\n");
        return -1;
    }

    if (decode_ack_spi_access(buf_ack, buf) != 0) {
        printf("ERROR: invalid REQ_READ_REGS ack\n");
        return -1;
    }

    return 0;
}


int mcu_gpio_write(int fd,  uint8_t gpio_port ,uint8_t gpio_id, uint8_t gpio_value) {
    uint8_t int_buf [4];
    int_buf[0] = gpio_port;
    int_buf[1] = gpio_id;
    int_buf[2] = gpio_value;


    if (write_req(fd, ORDER_ID__REQ_WRITE_GPIO, 3, int_buf) != 0) {
        printf("ERROR: failed to write REQ_READ_REGS request\n");
        return -1;
    }

    if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read PING ack\n");
        return -1;
    }

    if (decode_ack_gpio_access(buf_ack) != 0) {
        printf("ERROR: invalid REQ_READ_REGS ack\n");
        return -1;
    }

    return 0;
}


int mcu_reset(int fd, bool reset_mcu) {
    uint8_t status;


    /* Reset MCU */
    if (reset_mcu == true) {
        buf_req[REQ_RESET__TYPE] = RESET_TYPE__GTW;
        if (write_req(fd, ORDER_ID__REQ_RESET, REQ_RESET_SIZE, buf_req) != 0) {
            printf("ERROR: failed to write RESET request\n");
            return -1;
        }

        if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
            printf("ERROR: failed to read RESET ack\n");
            return -1;
        }

        if (decode_ack_reset(buf_ack, &status) != 0) {
            printf("ERROR: invalid RESET ack\n");
            return -1;
        }

        if (status != 0) {
            printf("ERROR: Failed to reset concentrator MCU\n");
            return -1;
        }
    }

    /* Wait for MCU to get ready after reset */
    wait_ms(500);

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mcu_boot(int fd) {
    if (write_req(fd, ORDER_ID__REQ_BOOTLOADER_MODE, 0, NULL) != 0) {
        printf("ERROR: failed to write BOOTLOADER_MODE request\n");
        return -1;
    }

    if (read_ack(fd, buf_ack, sizeof buf_ack) < 0) {
        printf("ERROR: failed to read BOOTLOADER_MODE ack\n");
        return -1;
    }

    if (decode_ack_bootloader_mode(buf_ack) != 0) {
        printf("ERROR: invalid BOOTLOADER_MODE ack\n");
        return -1;
    }

    return 0;
}



/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t mcu_get_nb_rx_radio(void) {
    return nb_radio_rx;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t mcu_get_nb_tx_radio(void) {
    return nb_radio_tx;
}






/* Simple write */
int lgw_usb_w(int usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    //int spi_device;
    uint8_t out_buf[5];
    uint8_t in_buf[5+4];
    uint8_t command_size;
    int a;


    /* prepare frame to be sent */
    out_buf[0] = 0;
    out_buf[1] = spi_mux_target;
    out_buf[2] = 0x80 | ((address >> 8) & 0x7F);
    out_buf[3] =        ((address >> 0) & 0xFF);
    out_buf[4] = data;
    command_size = 5;
    a = mcu_spi_access(usb_target,out_buf,command_size,in_buf);

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB WRITE FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB write success\n");
        return 0;
    }
}


int lgw_usb_r(int usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    //int spi_device;
    uint8_t out_buf[6];
    uint8_t in_buf[6+4];
    uint8_t command_size;
    int a;

   

    /* prepare frame to be sent */
    out_buf[0] = 0;
    out_buf[1] = spi_mux_target;
    out_buf[2] = 0x00 | ((address >> 8) & 0x7F);
    out_buf[3] =        ((address >> 0) & 0xFF);
    out_buf[4] = 0x00;
    out_buf[5] = 0x00;
    command_size = 6;
    a = mcu_spi_access(usb_target,out_buf,command_size,in_buf);

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB READ FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB read success\n");
        *data = in_buf[command_size+4 - 1];
        //printf("TARGET : %d USB_READ @ 0x%x : 0x%x\n",spi_mux_target,   address, *data);
        return 0;
    }

}


/* Burst (multiple-byte) write */
int lgw_usb_wb(int usb_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size) {
    //int spi_device;
    uint8_t out_buf[4096];
    uint8_t in_buf[4096+4];
    uint8_t command_size;
    //int size_to_do, chunk_size, offset;
    //int byte_transfered = 0;
    int i;
    int a;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return -1;
    }


    /* prepare command byte */
    out_buf[0] = 0;
    out_buf[1] = spi_mux_target;
    out_buf[2] = 0x80 | ((address >> 8) & 0x7F);
    out_buf[3] =        ((address >> 0) & 0xFF);
    for (i=0;i<size; i++) {
        out_buf[i+4] = data[i];
    }
    command_size = 4 + size;
    //printf("USB write burst @ 0x%4x size : %x\n", address, size);
    a = mcu_spi_access(usb_target,out_buf,command_size,in_buf);

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB WRITE FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB write success\n");
        return 0;
    }

}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_usb_rb(int usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    //int spi_device;
    uint8_t out_buf[4096];
    uint8_t in_buf[4096+4];
    uint8_t command_size;
    //int size_to_do, chunk_size, offset;
    //int byte_transfered = 0;
    int i;
    int a;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return -1;
    }


    /* prepare command byte */
    out_buf[0] = 0;
    out_buf[1] = spi_mux_target;
    out_buf[2] = 0x00 | ((address >> 8) & 0x7F);
    out_buf[3] =        ((address >> 0) & 0xFF);
    out_buf[4] = 0x00; 
    for (i=0;i<size; i++) {
        out_buf[i+5] =0;
    }

    command_size = 5+size;
    //printf("USB read burst @ 0x%4x size : %x\n", address, size);
    a = mcu_spi_access(usb_target,out_buf,command_size,in_buf);

    //data = &in_buf[command_size+4-size];
    memcpy(data, in_buf + 4+5, size);
    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB WRITE FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB write success\n");
        return 0;
    }
}




/* --- EOF ------------------------------------------------------------------ */
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2018 Semtech

Description:
    TODO

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "loragw_spi.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_sx1261.h"
#include "pram_lbt_scan.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_RAD == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif

#define SX1261_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define WAIT_BUSY_SX1261_MS  0.1
#define WAIT_BUSY_SX1261_US 100

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

extern void *sx1261_lgw_spi_target; /*! generic pointer to the SPI device */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx1261_write_command( sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int spi_device;
    int cmd_size = 1; /*  op_code */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a, i;

    /* wait BUSY */
    //wait_ms(WAIT_BUSY_SX1261_MS);
    wait_us(WAIT_BUSY_SX1261_US);

    /* check input variables */
    CHECK_NULL(sx1261_lgw_spi_target);

    spi_device = *(int *)sx1261_lgw_spi_target; /* must check that spi_target is not null beforehand */

    /* prepare frame to be sent */
    
    out_buf[0] = (uint8_t)op_code;
    for(i = 0; i < (int)size; i++) {
        out_buf[cmd_size + i] = data[i];
    }
    command_size = cmd_size + size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len) {
        printf("ERROR: SPI WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI write success\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_read_command( sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int spi_device;
    int cmd_size = 1; /* header + op_code + NOP */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    uint8_t in_buf[ARRAY_SIZE(out_buf)];
    struct spi_ioc_transfer k;
    int a, i;

    /* wait BUSY */
    //wait_ms(WAIT_BUSY_SX1261_MS);
    wait_us(WAIT_BUSY_SX1261_US);

    /* check input variables */
    CHECK_NULL(sx1261_lgw_spi_target);
    CHECK_NULL(data);

    spi_device = *(int *)sx1261_lgw_spi_target; /* must check that spi_target is not null beforehand */

    /* prepare frame to be sent */
    out_buf[0] = (uint8_t)op_code;
    for(i = 0; i < (int)size; i++) {
        out_buf[cmd_size + i] = data[i];
    }
    command_size = cmd_size + size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.rx_buf = (unsigned long) in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len) {
        printf("ERROR: SPI READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI read success\n");
        //*data = in_buf[command_size - 1];
        memcpy(data, in_buf + cmd_size, size);
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


int sx1261_calibrate(uint32_t freq_hz) {
    uint8_t buff[16];

    buff[0] = 0x00;
    sx1261_read_command(SX1261_GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);

    /* Run calibration */
    if ((freq_hz > 430E6) && (freq_hz < 440E6)) {
        buff[0] = 0x6B;
        buff[1] = 0x6F;
    } else if ((freq_hz > 470E6) && (freq_hz < 510E6)) {
        buff[0] = 0x75;
        buff[1] = 0x81;
    } else if ((freq_hz > 779E6) && (freq_hz < 787E6)) {
        buff[0] = 0xC1;
        buff[1] = 0xC5;
    } else if ((freq_hz > 863E6) && (freq_hz < 870E6)) {
        buff[0] = 0xD7;
        buff[1] = 0xDB;
    } else if ((freq_hz > 902E6) && (freq_hz < 928E6)) {
        buff[0] = 0xE1;
        buff[1] = 0xE9;
    } else {
        printf("ERROR: failed to calibrate sx1261 radio, frequency range not supported (%u)\n", freq_hz);
        return -1;
    }
    sx1261_write_command(SX1261_CALIBRATE_IMAGE, buff, 2);

    /* Wait for calibration to complete */
    wait_ms(10);

    buff[0] = 0x00;
    buff[1] = 0x00;
    buff[2] = 0x00;
    sx1261_read_command( SX1261_GET_DEVICE_ERRORS, buff, 3);
    printf("%s: get_device_errors: 0x%02X 0x%02X 0x%02X\n", __FUNCTION__, buff[0], buff[1], buff[2]);
    if (TAKE_N_BITS_FROM(buff[2], 4, 1) != 0) {
        printf("ERROR: sx1261 Image Calibration Error\n");
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
int sx1261_load_pram (void) {
    uint8_t buff[32];
    uint16_t k;

    /* Set Radio in Standby mode */
    buff[0] = (uint8_t)SX1261_STDBY_RC;
    sx1261_write_command( SX1261_SET_STANDBY, buff, 1);
    wait_ms(10);

    buff[0] = 0x00;
    sx1261_read_command( SX1261_GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);


    buff[0] = 0x03;
    buff[1] = 0x20;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    buff[5] = 0x00;
    buff[6] = 0x00;
    buff[7] = 0x00;
    buff[8] = 0x00;
    buff[9] = 0x00;
    buff[10] = 0x00;
    buff[11] = 0x00;
    buff[12] = 0x00;
    buff[13] = 0x00;
    buff[14] = 0x00;
    buff[15] = 0x00;
    buff[16] = 0x00;
    buff[17] = 0x00;
    sx1261_read_command( SX1261_READ_REGISTER, buff, 18);
    printf(" @0x320 ");
    
    for(k=0; k< 18; k++) {
        printf("%c", (char)buff[k]);
    }

    printf("\n");
    
    buff[0] = 0x08;
    buff[1] = 0xAC;
    buff[2] = 0x00;
    buff[3] = 0x00;
    sx1261_read_command( SX1261_READ_REGISTER, buff, 4);
    printf(" @0x8AC d: 0x%x 0x%x \n", buff[2], buff[3]);

    
    buff[0] = 0x08;
    buff[1] = 0xAC;
    buff[2] = 0x96;//0x77;
    sx1261_write_command( SX1261_WRITE_REGISTER, buff, 3);
    buff[0] = 0x08;
    buff[1] = 0xAC;
    buff[2] = 0x00;
    buff[3] = 0x00;
    sx1261_read_command( SX1261_READ_REGISTER, buff, 4);
    printf(" @0x8AC d: 0x%x 0x%x \n", buff[2], buff[3]);





    /* Load Patch RAM*/
    
    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x00;
    buff[3] = 0x00;
    sx1261_read_command( SX1261_READ_REGISTER, buff, 4);
    printf(" @0x610 d: 0x%x 0x%x \n", buff[2], buff[3]);

    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x10;
    sx1261_write_command( SX1261_WRITE_REGISTER, buff, 3);
    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x00;
    buff[3] = 0x00;
    sx1261_read_command( SX1261_READ_REGISTER, buff, 4);
    printf(" @0x610 d: 0x%x 0x%x \n", buff[2], buff[3]);

    
    for(uint16_t i=0; i< PRAM_COUNT; i++)
    {
        uint32_t val = pram[i];
        uint32_t addr;
        addr = 0x8000+ 4*i;

        buff[0] = (addr >> 8 ) & 0xFF;
        buff[1] = addr  & 0xFF;
        buff[2] = (val >> 24) & 0xff;
        buff[3] = (val >> 16) & 0xff;
        buff[4] = (val >> 8) & 0xff;
        buff[5] = (val >> 0) & 0xff;
        sx1261_write_command( SX1261_WRITE_REGISTER, buff, 6);
       
    }
    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x00;
    sx1261_write_command( SX1261_WRITE_REGISTER, buff, 3);

    buff[0] = 0;
    sx1261_write_command( 0xd9, buff, 0);

    buff[0] = 0x03;
    buff[1] = 0x20;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    buff[5] = 0x00;
    buff[6] = 0x00;
    buff[7] = 0x00;
    buff[8] = 0x00;
    buff[9] = 0x00;
    buff[10] = 0x00;
    buff[11] = 0x00;
    buff[12] = 0x00;
    buff[13] = 0x00;
    buff[14] = 0x00;
    buff[15] = 0x00;
    buff[16] = 0x00;
    buff[17] = 0x00;
    sx1261_read_command( SX1261_READ_REGISTER, buff, 18);
    printf(" @0x320 ");
    
    for(k=0; k< 18; k++) {
        printf("%c", (char)buff[k]);
    }

    printf("\n");
    return 0;

}

int sx1261_start_lbt( int scan_time_us, int threshold_dbm) {

    uint8_t buff[16];
    int nb_scan;
    uint8_t threshold_reg;
    threshold_reg = -2*threshold_dbm;
    nb_scan = scan_time_us >>1;
    buff[0] = 32; //intervall8us => 
    buff[1] = (nb_scan>>8) & 0xFF;  //nbScans MSB
    buff[2] = nb_scan & 0xFF; //nbScans LSB
    buff[3] = threshold_reg; //threshold
    buff[4] = 3;  //gpioId
    sx1261_write_command( 0x9a, buff, 5);
    printf("START LBT\n");
    return 0;

}
int sx1261_spectral_scan(int nb_scan) {
    
    uint16_t k;
    uint8_t buff[20];
    buff[0] = (nb_scan>>8) & 0xFF;  //nbScans MSB
    buff[1] = nb_scan & 0xFF; //nbScans LSB
    buff[2] = 240; //-80
    buff[3] = 120;  //-60
    buff[4] = 80;  //-40
    sx1261_write_command( 0x9b, buff, 5);
    printf("START SCAN\n");

    wait_ms(10);

    buff[0] = 0x04;
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    buff[5] = 0x00;
    buff[6] = 0x00;
    buff[7] = 0x00;
    buff[8] = 0x00;
    buff[9] = 0x00;
    buff[10] = 0x00;
    buff[11] = 0x00;
    buff[12] = 0x00;
    buff[13] = 0x00;
    buff[14] = 0x00;
    buff[15] = 0x00;
    buff[16] = 0x00;
    buff[17] = 0x00;
    sx1261_read_command( SX1261_READ_REGISTER, buff, 18);
    printf(" @0x400 ");
    
    for(k=0; k< 18; k++) {
        printf("0x%x ", buff[k]);
    }

    printf("\n");

    return 0;
}


int sx1261_setup( uint32_t freq_hz) {
    int32_t freq_reg;
    uint8_t buff[32];
       
    /* Set Radio in Standby mode */
    buff[0] = (uint8_t)SX1261_STDBY_XOSC;
    sx1261_write_command( SX1261_SET_STANDBY, buff, 1);
    wait_ms(10);

    buff[0] = 0x00;
    sx1261_read_command( SX1261_GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);



    buff[0] = 0x00; /* FSK */
    sx1261_write_command( SX1261_SET_PACKET_TYPE, buff, 1); 
    /* Set frequency */
    freq_reg = SX1261_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    sx1261_write_command( SX1261_SET_RF_FREQUENCY, buff, 4);
    /* Set PacketType */

    /* Set modulation param LoRa */
    buff[0] = 0;    // BR
    buff[1] = 0x14; // BR  
    buff[2] = 0x00; // BR
    buff[3] = 0x00; // Gaussian BT disabled
    buff[4] = 0x0A; // BW_234300 (232.3 kHz DSB) 
    buff[5] = 0x02; // FDEV
    buff[6] = 0xE9; // FDEV
    buff[7] = 0x0F; // FDEV
    sx1261_write_command( SX1261_SET_MODULATION_PARAMS, buff, 8); 

    /* Set packet param LoRa */    
    buff[0] = 0x00; /* Preamble length MSB */
    buff[1] = 0x20; /* Preamble length LSB 32 bits*/
    buff[2] = 0x05; /* Preamble detector lenght 16 bits */
    buff[3] = 0x20; /* SyncWordLength 32 bits*/
    buff[4] = 0x00; /* AddrComp disabled */ 
    buff[5] = 0x01; /* PacketType variable size */
    buff[6] = 0xff; /* PayloadLength 255 bytes */
    buff[7] = 0x00; /* CRCType 1 Byte */
    buff[8] = 0x00; /* Whitening disabled*/
    sx1261_write_command( SX1261_SET_PACKET_PARAMS, buff, 9); 

    /* Set frequency offset to 0 */
/*     buff[0] = 0x08;
    buff[1] = 0x8F;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    sx1261_write_command( WRITE_REGISTER, buff, 5); */

    /* Configure RSSI averaging window */
    buff[0] = 0x08;
    buff[1] = 0x9B;
    buff[2] = 0x03 << 2;
    sx1261_write_command( SX1261_WRITE_REGISTER, buff, 3); /* Rx Continuous */


    /* Set Radio in Rx mode, necessary to give a clock to SX1302 */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    sx1261_write_command( SX1261_SET_RX, buff, 3); /* Rx Continuous */
    sx1261_read_command( SX1261_GET_STATUS, buff, 1);
    printf("%s: get_status: 0x%02X\n", __FUNCTION__, buff[0]);

    /*buff[0] = 0x05;
    buff[1] = 0x87;
    buff[2] = 0x0E;
    sx1261_write_command( WRITE_REGISTER, buff, 3); // 0x0E => HOST DRIVE DIO through register   
*/
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

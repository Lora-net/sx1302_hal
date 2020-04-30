/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Host specific functions to address the LoRa concentrator registers through
    a USB interface.
    Single-byte read/write and burst read/write.

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

#include "loragw_usb.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_USB_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_USB_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_usb_open(const char * com_path, void **com_target_ptr) {
    int *usb_device = NULL;
    int dev;
    int a=0, b=0;
    int i;

    /* check input variables */
    CHECK_NULL(com_target_ptr);

    /* allocate memory for the device descriptor */
    usb_device = malloc(sizeof(int));
    if (usb_device == NULL) {
        DEBUG_MSG("ERROR: MALLOC FAIL\n");
        return LGW_USB_ERROR;
    }

    /* open USB device */
    dev = open(com_path, O_RDWR);
    if (dev < 0) {
        DEBUG_PRINTF("ERROR: failed to open USB device %s\n", com_path);
        return LGW_USB_ERROR;
    }

    /* TODO */

    *usb_device = dev;
    *com_target_ptr = (void *)usb_device;
    DEBUG_MSG("Note: USB port opened and configured ok\n");
    return LGW_USB_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_usb_close(void *usb_target) {
    int usb_device;
    int a;

    /* check input variables */
    CHECK_NULL(usb_target);

    /* close file & deallocate file descriptor */
    usb_device = *(int *)usb_target;
    a = close(usb_device);
    free(usb_target);

    /* determine return code */
    if (a < 0) {
        DEBUG_MSG("ERROR: USB PORT FAILED TO CLOSE\n");
        return LGW_USB_ERROR;
    } else {
        DEBUG_MSG("Note: USB port closed\n");
        return LGW_USB_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_usb_w(void *usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    /* TODO */
    return LGW_USB_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_usb_r(void *usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    /* TODO */
    return LGW_USB_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_usb_wb(void *usb_target, uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size) {
    /* TODO */
    return LGW_USB_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_usb_rb(void *usb_target, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    /* TODO */
    return LGW_USB_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */

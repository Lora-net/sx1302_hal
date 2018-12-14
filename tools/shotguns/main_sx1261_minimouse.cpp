/*!
 * \file      Main4certification.c
 *
 * \brief     Description : Lorawan Stack example
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
  __  __ _       _
 |  \/  (_)     (_)
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___|


 * \endcode

Maintainer        : Fabien Holin (SEMTECH)
*/

#include "Define.h"
#include "utilities.h"
#include "UserDefine.h"
#include "appli.h"
#include "SX126x.h"
#include "ApiMcu.h"
#include "utilities.h"
#include "main.h"
#include "UserDefine.h"
#include "ApiMcu.h"

#include "tinymt32.h"

#define RAND_RANGE(min, max) ((rand() % (max - min + 1)) + min)

#define TX_START_FERQ 865100000
#define TX_STEP_FREQ 200000

uint32_t Channels[10] = {
    TX_START_FERQ + 0 * TX_STEP_FREQ,
    TX_START_FERQ + 1 * TX_STEP_FREQ,
    TX_START_FERQ + 2 * TX_STEP_FREQ,
    TX_START_FERQ + 3 * TX_STEP_FREQ,
    TX_START_FERQ + 4 * TX_STEP_FREQ,
    TX_START_FERQ + 5 * TX_STEP_FREQ,
    TX_START_FERQ + 6 * TX_STEP_FREQ,
    TX_START_FERQ + 7 * TX_STEP_FREQ,
    868300000,
    868800000
};

McuXX<McuSTM32L4> mcu ( LORA_SPI_MOSI, LORA_SPI_MISO, LORA_SPI_SCLK ) ;
SX126x * RadioUser;

uint32_t nb_pkt_sent = 0;
uint32_t nb_loop = 360000;

uint32_t TimeOnAir( uint8_t pktLen, eBandWidth bandwidth, uint8_t Datarate, uint16_t PreambleLen, bool CrcOn, bool HeaderOn, uint8_t Coderate );

/* ISR routine specific of this example */
void UserIsr ( void ) {
    IrqFlags_t RegIrqFlag;

    RegIrqFlag = RadioUser->GetIrqFlagsLora( );
    RadioUser->ClearIrqFlagsLora( );

    switch ( RegIrqFlag ) {
        case SENT_PACKET_IRQ_FLAG :
            DEBUG_MSG ("packet sent\n");
            break;

        case RECEIVE_PACKET_IRQ_FLAG :
            DEBUG_MSG ("packet received\n");
            break;

        case RXTIMEOUT_IRQ_FLAG :
            DEBUG_MSG ("RX timeout\n");
            break;

        case BAD_PACKET_IRQ_FLAG :
            DEBUG_MSG ("Bad packet\n");
            break;

        default :
            DEBUG_PRINTF ("ERROR: radio ISR %x\n", RegIrqFlag);
            break;
    }
}

int main( ) {
    uint8_t UserRxPayloadSize = 32;
    uint8_t UserRxPayload[255];
    int j;
    uint32_t toa;

    uint8_t currentSF;
    eBandWidth currentBW;
    uint32_t currentFreq;

    mcu.InitMcu ( );
    RadioUser = new SX126x( LORA_BUSY, LORA_CS, LORA_RESET,TX_RX_IT );

    //mcu.WatchDogStart ( );

    /*!
    * \brief Restore the LoraWan Context
    */
    DEBUG_MSG("sx1261_shotgun is starting ...\n");

    mcu.mwait(2);

    mcu.AttachInterruptIn( &UserIsr ); // attach ISR

    tinymt32_t tinymt;
    tinymt.mat1 = 0x8f7011ee;
    tinymt.mat2 = 0xfc78ff1f;
    tinymt.tmat = 0x3793fdff;

    RadioUser->Reset();

    while (1) {
        if ((nb_loop == 0) || (nb_loop > nb_pkt_sent)) {
            tinymt32_init(&tinymt, (int)nb_pkt_sent);
            UserRxPayload[0] = 0xCA;
            UserRxPayload[1] = 0xFE;
            UserRxPayload[2] = 0x23;
            UserRxPayload[3] = 0x45;
            UserRxPayload[4] = (uint8_t)(nb_pkt_sent >> 24);
            UserRxPayload[5] = (uint8_t)(nb_pkt_sent >> 16);
            UserRxPayload[6] = (uint8_t)(nb_pkt_sent >> 8);
            UserRxPayload[7] = (uint8_t)(nb_pkt_sent >> 0);
            UserRxPayloadSize = (uint8_t)tinymt32_generate_uint32(&tinymt);
            if (UserRxPayloadSize < 8) {
                UserRxPayloadSize = 8; /* minimum size */
            }
            for (j = 8; j < UserRxPayloadSize; j++) {
                UserRxPayload[j] = (uint8_t)tinymt32_generate_uint32(&tinymt);
            }
            
            currentSF = nb_pkt_sent%8 + 5;
            currentBW = BW125;
            currentFreq = Channels[nb_pkt_sent%4]; /* on 4 channels only */

            RadioUser->SendLora( &UserRxPayload[0], UserRxPayloadSize, currentSF, currentBW, currentFreq, 0 );
            toa = TimeOnAir( UserRxPayloadSize, currentBW, currentSF, 8, true, true, 1);
            mcu.mwait_ms(toa);
            if (currentSF == 12) {
                mcu.mwait_ms(100); /* add some time before sending next packet, to avoid the GW to receive the next SF5 before this SF12 (processing time) */
            }
            nb_pkt_sent += 1;
            DEBUG_PRINTF("%u packet sent (SF=%u, toa=%u ms)\n", nb_pkt_sent, currentSF, toa);
        } else {
            mcu.mwait_ms(100);
        }
    }
}

uint32_t TimeOnAir( uint8_t pktLen, eBandWidth bandwidth, uint8_t Datarate, uint16_t PreambleLen, bool CrcOn, bool HeaderOn, uint8_t Coderate )
{
    uint32_t airTime = 0;
    double bw = 0.0;

    if (Datarate < 7) {
        Datarate = 7; /* TODO */
    }

    switch (bandwidth) {
        case BW125:
            bw = 125 * 1e3;
            break;
        case BW250:
            bw = 250 * 1e3;
            break;
        case BW500:
            bw = 500 * 1e3;
            break;
         default:
            break;
    }

    // Symbol rate : time for one symbol (secs)
    double rs = bw / ( 1 << Datarate );
    double ts = 1 / rs;
    // time of preamble
    double tPreamble = ( PreambleLen + 4.25 ) * ts;
    // Symbol length of payload and time
    double tmp = ceil( ( 8 * pktLen - 4 * Datarate +
                            28 + 16 * CrcOn -
                            ( ( HeaderOn == false ) ? 20 : 0 ) ) /
                            ( double )( 4 * ( Datarate -
                            ( ( Datarate > 10 ) ? 2 : 0 ) ) ) ) *
                            ( Coderate + 4 );
    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return ms secs
    airTime = floor( tOnAir * 1e3 + 0.999 );

    return airTime;
}

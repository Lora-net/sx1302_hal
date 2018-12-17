#include "mbed.h"

#include "tinymt32.h"

#include "sx1272-hal.h"
#include "radio.h"
#include "main.h"

#define BUFFER_SIZE 32

#define TX_START_FERQ 865100000
#define TX_STEP_FREQ 200000

#define RAND_RANGE(min, max) ((rand() % (max - min + 1)) + min)

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

typedef enum Modulation
{
    MOD_LORA,
    MOD_FSK
} Modulation_t;

typedef enum LoRaBandwidth
{
    LORA_BW_125K,
    LORA_BW_250K,
    LORA_BW_500K,
    LORA_BW_NA
} LoRaBandwidth_t;

/*!
 * Serial communication for debug logs
 */
Serial pc(USBTX, USBRX); // tx, rx

/*!
 * Interrupt handler for nucleo user button
 */
InterruptIn DatarateButton( USER_BUTTON );

/*
 *  Global variables declarations
 */
typedef enum
{
    CONFIGURE = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
}AppStates_t;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * RX buffer and variables
 */
uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

/*!
 * TX buffer and variables
 */
static uint8_t LoRaWANBuffer[255];
static uint16_t FCnt = 0;

/*
 *  Global variables declarations
 */
SX1272MB2xAS Radio( NULL );

/* -------------- */

uint32_t SendPacket( uint32_t freq_hz, RadioModems_t modulation, LoRaBandwidth_t bw, uint8_t pkt_size, uint8_t datarate, uint8_t coderate, bool implicit_header )
{
    uint32_t TimeOnAir;

    Radio.SetChannel( freq_hz );

    switch( modulation )
    {
        case MODEM_LORA:
            Radio.SetTxConfig( MODEM_LORA, 0, 0, bw, datarate, coderate, 8, implicit_header, true, 0, 0, false, 10e3 );
            TimeOnAir = Radio.TimeOnAir( MODEM_LORA, pkt_size );
            break;
        case MODEM_FSK:
            Radio.SetTxConfig( MODEM_FSK, 0, 25e3, 0, 50e3, 0, 5, false, true, 0, 0, false, 3e3 );
            TimeOnAir = Radio.TimeOnAir( MODEM_FSK, pkt_size );
            break;
        default:
            TimeOnAir = 0;
            break;
    }
    
    Radio.Send( LoRaWANBuffer, pkt_size );
    
    return TimeOnAir;
}

/* -------------- */

int main() {
    Timer t;
    double PktToA;
    uint32_t freq_hz;
    uint8_t tx_datr;
    uint16_t tx_delay;
    uint16_t tx_nbpkt;
    int pkt_count = 0;
    int j;
    uint8_t random_size;

    pc.printf( "\n > Initializing... < \n" );
    
    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );
 
    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {
        pc.printf( "Radio could not be detected!\n", NULL );
        wait( 1 );
    }
 
    pc.printf( "\n > Board Type: SX1272MB2xAS < \n" );
    
    Radio.SetPublicNetwork( true );

    // initialize pseudo-random generator
    tinymt32_t tinymt;
    tinymt.mat1 = 0x8f7011ee;
    tinymt.mat2 = 0xfc78ff1f;
    tinymt.tmat = 0x3793fdff;
  
    //
    // TX LOOP
    //
    pkt_count = 0;
    FCnt = 0;
    tx_delay = 0;
    tx_nbpkt = 50000;
    while ( (tx_nbpkt == 0) || (pkt_count < tx_nbpkt) )
    {
        tinymt32_init(&tinymt, (int)pkt_count);
        LoRaWANBuffer[0] = 0xCA;
        LoRaWANBuffer[1] = 0xFE;
        LoRaWANBuffer[2] = 0x12;
        LoRaWANBuffer[3] = 0x34;
        LoRaWANBuffer[4] = (uint8_t)(FCnt >> 24);
        LoRaWANBuffer[5] = (uint8_t)(FCnt >> 16);
        LoRaWANBuffer[6] = (uint8_t)(FCnt >> 8);
        LoRaWANBuffer[7] = (uint8_t)(FCnt >> 0);
        random_size = (uint8_t)tinymt32_generate_uint32(&tinymt);
        random_size = random_size % 100; /* maximum size */
        if (random_size < 8) {
            random_size = 8; /* minimum size */
        }
        for (j = 8; j < random_size; j++) {
            LoRaWANBuffer[j] = (uint8_t)tinymt32_generate_uint32(&tinymt);
        }
     
        // Send LoRa packet
        //freq_hz = Channels[FCnt%8];
        freq_hz = Channels[FCnt%4 + 4]; /* Send on 4 last channels */
        tx_datr = RAND_RANGE( 7, 7 );

#if 0
        if ((FCnt % 10) < 8) /* LoRa Multi-SF */
        {
            PktToA = SendPacket( freq_hz, MODEM_LORA, LORA_BW_125K, random_size, tx_datr, 1, false );
        }
        else if ((FCnt % 10) == 8) /* LoRa Service */
        {
            PktToA = SendPacket( 866300000, MODEM_LORA, LORA_BW_250K, random_size, tx_datr, 1, false );
        }
        else /* FSK */
        {
            PktToA = SendPacket( 866800000, MODEM_FSK, LORA_BW_NA, random_size, 0, 0, false );
        }
#else
        if (FCnt % 9)
        {
            PktToA = SendPacket( freq_hz, MODEM_LORA, LORA_BW_125K, random_size, tx_datr, 1, false );
        }
        else
        {
            PktToA = SendPacket( 866300000, MODEM_LORA, LORA_BW_250K, random_size, tx_datr, 1, false );
        }
#endif

        //pc.printf( "Sending LoRa packet: Freq=%u, SF%u (%.3fms), FCnt=%u...\n", freq_hz, tx_datr, PktToA, FCnt );
        wait_ms( PktToA );
        Radio.Sleep( );
        
        FCnt += 1;
        if (tx_nbpkt > 0) /* else infinite loop */
        {
            pkt_count++;
        }
        
        wait_ms( tx_delay );
    }
}

void OnTxDone( void )
{
    //Radio.Sleep( );
    //pc.printf( "> OnTxDone\n\r" );
}
 
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    pc.printf( "> OnRxDone (size:%u)\n\r",size );
}
 
void OnTxTimeout( void )
{
    Radio.Sleep( );
    pc.printf( "> OnTxTimeout\n\r" );
}
 
void OnRxTimeout( void )
{
    Radio.Sleep( );
    pc.printf( "> OnRxTimeout\n\r" );
}
 
void OnRxError( void )
{
    Radio.Sleep( );
    pc.printf( "> OnRxError\n\r" );
}

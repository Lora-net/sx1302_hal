#include "DuckLoRa.h"
#include "../bridge/ClusterDuckBridge.h"
#include "../utils/DuckUtils.h"

#ifdef CDPCFG_RADIO_SX1262
#define DUCK_RADIO_IRQ_TIMEOUT RADIOLIB_SX126X_IRQ_TIMEstd::optional<std::vector<uint8_t>> DuckLoRa::readReceivedData() {
    std::vector<uint8_t> packetBytes;

    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup %s\n", DUCKLORA_ERR_NOT_INITIALIZED);
        return std::nullopt;
    }

    // Check if the forwarder provided a packet via the unified bridge
    auto forwarderPkt = cdp_bridge::pop_uplink_packet();
    if (forwarderPkt.has_value()) {
        packetBytes = std::move(forwarderPkt.value());
        loginfo_ln("readReceivedData(): got packet from forwarder bridge size=%d", (int)packetBytes.size());
    } else {_RADIO_IRQ_TX_DONE RADIOLIB_SX126X_IRQ_TX_DONE
#define DUCK_RADIO_IRQ_RX_DONE RADIOLIB_SX126X_IRQ_RX_DONE
#define DUCK_RADIO_IRQ_CRC_ERROR RADIOLIB_SX126X_IRQ_CRC_ERR
#define DUCK_RADIO_IRQ_HEADER_ERR RADIOLIB_SX126X_IRQ_HEADER_ERR
#else
#define DUCK_RADIO_IRQ_TIMEOUT RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_TIMEOUT
#define DUCK_RADIO_IRQ_TX_DONE RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE
#define DUCK_RADIO_IRQ_RX_DONE RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_DONE
#define DUCK_RADIO_IRQ_CRC_ERROR RADIOLIB_SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR
#endif
/*
#if defined(CDPCFG_RADIO_SX1262)
CDPCFG_LORA_CLASS lora =
        new Module(CDPCFG_PIN_LORA_CS, CDPCFG_PIN_LORA_DIO1, CDPCFG_PIN_LORA_RST,
                   CDPCFG_PIN_LORA_BUSY);
#else
CDPCFG_LORA_CLASS lora = new Module(CDPCFG_PIN_LORA_CS, CDPCFG_PIN_LORA_DIO0,
                  CDPCFG_PIN_LORA_RST, CDPCFG_PIN_LORA_DIO1);
#endif
*/

volatile uint16_t DuckLoRa::interruptFlags = 0;
volatile bool DuckLoRa::receivedFlag = false;

/* RX radio metadata — updated on every received packet (callback and polling paths) */
static uint32_t last_rx_freq_hz = CDPCFG_RF_LORA_FREQ_HZ;
static uint8_t  last_rx_sf      = CDPCFG_RF_LORA_SF;
static int16_t  last_rx_rssi    = 0;
static float    last_rx_snr     = 0.0f;

const LoRaConfigParams DuckLoRa::defaultRadioParams = {
    /* band     = */ CDPCFG_RF_LORA_FREQ,
    /* txPower  = */ CDPCFG_RF_LORA_TXPOW,
    /* bw       = */ CDPCFG_RF_LORA_BW,
    /* sf       = */ CDPCFG_RF_LORA_SF,
    /* gain     = */ CDPCFG_RF_LORA_GAIN,
    /* func     = */ onInterrupt
};

// Forward declaration for bridge callback (non-static to match friend declaration)
void duck_rx_from_forwarder_cb(const uint8_t* payload, uint16_t size,
                               int16_t rssi, float snr,
                               uint32_t freq_hz, uint32_t tmst, uint8_t rf_chain,
                               uint32_t bandwidth_hz, uint8_t datarate_sf, uint8_t coderate);

// Helper: Build a test downlink payload from an RX payload.
// Replace this with LoRaWAN PHYPayload construction when needed.
static std::vector<uint8_t> build_echo_downlink(const uint8_t* rx_payload, uint16_t rx_size) {
    const char prefix[] = "ECHO:";
    std::vector<uint8_t> dl;
    dl.reserve(sizeof(prefix) - 1 + rx_size);
    dl.insert(dl.end(), prefix, prefix + (sizeof(prefix) - 1));
    dl.insert(dl.end(), rx_payload, rx_payload + rx_size);
    return dl;
}

int DuckLoRa::checkLoRaParameters(LoRaConfigParams config) { //this can be improved
    int rc = DUCK_ERR_NONE;
    if (config.func == NULL) {
        logerr_ln("ERROR  interrupt function is NULL");
        return DUCK_ERR_INVALID_ARGUMENT;
    }
    if (config.sf < 6 || config.sf > 12) {
        logerr_ln("ERROR  spreading factor is invalid");
        return DUCK_ERR_INVALID_ARGUMENT;
    }
    if (config.band < 150.0 || config.band > 960.0) {
        logerr_ln("ERROR  frequency is invalid");
        return DUCK_ERR_INVALID_ARGUMENT;
    }
    if (config.txPower < -9 || config.txPower > 22) {
        logerr_ln("ERROR  tx power is invalid");
        return DUCK_ERR_INVALID_ARGUMENT;
    }
    if (config.bw < 7.8 || config.bw > 500.0) {
        logerr_ln("ERROR  bandwidth is invalid");
        return DUCK_ERR_INVALID_ARGUMENT;
    }
    // Allow gain of 0 (no antenna gain correction)
    if (config.gain < 0 || config.gain > 6) {
        logerr_ln("ERROR  gain is invalid (must be 0-6)");
        return DUCK_ERR_INVALID_ARGUMENT;
    }
    return rc;
}

int DuckLoRa::setupRadio(const LoRaConfigParams &config) {
    loginfo_ln("Setting up RADIOLIB LoRa radio...");
    printf("[DuckLoRa] ===== SETUPRADIO CALLED =====\n");
    int rc;
    rc = checkLoRaParameters(config);
    if (rc != DUCK_ERR_NONE) {
        return rc;
    }

    // Register the forwarder RX callback FIRST (even if already setup)
    // This ensures ClusterDuck can receive lgw_pkt_rx_s data from the packet forwarder
    printf("[DuckLoRa] Registering RX callback from forwarder...\n");
    cdp_bridge_register_rx_callback(duck_rx_from_forwarder_cb);
    printf("[DuckLoRa] RX callback registered\n");

    if (isSetup) {
        loginfo_ln("LoRa radio already setup");
        return DUCK_ERR_NONE;
    }

    /*rc = lora.begin();
    if (rc != RADIOLIB_ERR_NONE) {
        logerr_ln("ERROR  initializing LoRa driver. state = %d", rc);
        return DUCKLORA_ERR_BEGIN;
    }
    
    loginfo_ln("Setting up LoRa radio parameters...");
    rc = lora.setFrequency(config.band);
    if (rc == RADIOLIB_ERR_INVALID_FREQUENCY) {
        logerr_ln("ERROR  frequency is invalid");
        return DUCKLORA_ERR_SETUP;
    }

    rc = lora.setBandwidth(config.bw);
    if (rc == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        logerr_ln("ERROR  bandwidth is invalid");
        return DUCKLORA_ERR_SETUP;
    }

    rc = lora.setSpreadingFactor(config.sf);
    if (rc == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        logerr_ln("ERROR  spreading factor is invalid");
        return DUCKLORA_ERR_SETUP;
    }

    rc = lora.setOutputPower(config.txPower);
    if (rc == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        logerr_ln("ERROR  output power is invalid");
        return DUCKLORA_ERR_SETUP;
    }
#ifdef CDPCFG_RADIO_SX1262
    // set the interrupt handler to execute when packet tx or rx is done.
    lora.setDio1Action(config.func);
#else
    rc = lora.setGain(CDPCFG_RF_LORA_GAIN);
    if (rc == RADIOLIB_ERR_INVALID_GAIN) {
        logerr_ln("ERROR  gain is invalid");
        return DUCKLORA_ERR_SETUP;
    }
    // set the interrupt handler to execute when packet tx or rx is done.
    lora.setDio0Action(config.func,RISING);
#endif

    // set sync word to public (0x34) — dual-protocol requires both CDP and
    // LoRaWAN to share the same sync word so the SX1302 concentrator receives
    // packets from both protocols on the same channel configuration.
    rc = lora.setSyncWord(0x34);

    if (rc != RADIOLIB_ERR_NONE) {
        logerr_ln("ERROR  sync word is invalid");
        return DUCKLORA_ERR_SETUP;
    }   

    rc = lora.startReceive();

    if (rc != RADIOLIB_ERR_NONE) {
        logerr_ln("ERROR Failed to start receive");
        return DUCKLORA_ERR_RECEIVE;
    } */

    loginfo_ln("LoRa radio setup complete");
    isSetup = true;
    return DUCK_ERR_NONE;
}

/*int DuckLoRa::goToReceiveMode(bool clearReceiveFlag) {
    if (clearReceiveFlag) {
      DuckLoRa::setReceiveFlag(false);
    }
    return startReceive(); 
    return true;
} */

std::optional<std::vector<uint8_t>> DuckLoRa::readReceivedData() { //return a std optional
    std::vector<uint8_t> packetBytes;
    //int err = RADIOLIB_ERR_NONE;
    //int rxState = RADIOLIB_ERR_NONE;

    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup %s\n", DUCKLORA_ERR_NOT_INITIALIZED);
        return std::nullopt;
    }

    // Check if the forwarder provided a packet via the unified bridge
    auto forwarderPkt = cdp_bridge::pop_uplink_packet();
    if (forwarderPkt.has_value()) {
        // Store radio metadata so getSignalScore() / getFreqHz() / getRxSF() return real values
        last_rx_rssi    = forwarderPkt->rssi;
        last_rx_snr     = forwarderPkt->snr;
        last_rx_freq_hz = forwarderPkt->freq_hz;
        last_rx_sf      = forwarderPkt->datarate_sf;
        packetBytes = std::move(forwarderPkt->payload);
        loginfo_ln("readReceivedData(): got packet from forwarder bridge size=%d freq=%u SF=%u",
                   (int)packetBytes.size(), last_rx_freq_hz, last_rx_sf);
        
        // Clear the receive flag only if the queue is now empty
        // This allows hub.main() to continue processing remaining packets
        if (!cdp_bridge::has_uplink_packet()) {
            setReceiveFlag(false);
            loginfo_ln("readReceivedData(): queue empty, clearing receive flag");
        } else {
            loginfo_ln("readReceivedData(): queue has more packets, keeping receive flag set");
        }
    } /* else {
        // Fallback: read from radio as before
        int packet_length = lora.getPacketLength();
        if (packet_length < MIN_PACKET_LENGTH) {
            logerr_ln("ERROR  handlePacket rx data size invalid: %d", packet_length);
            return std::nullopt;
        }
        }

        packetBytes.resize(packet_length);
        err = lora.readData(packetBytes.data(), packet_length);
        loginfo_ln("readReceivedData() - lora.readData returns: err = %d", err);

        // restore RX mode regardless
        rxState = goToReceiveMode(true);

        if (err != RADIOLIB_ERR_NONE) {
            logerr_ln("ERROR  readReceivedData failed. err = %d", err);
            lastReceiveTime = millis();
            return std::nullopt;
        }
    } */

    loginfo_ln("Rx packet: %s", duckutils::toString(packetBytes.data(), packetBytes.size()).c_str());
    loginfo_ln("readReceivedData: checking path offset integrity");

    // Bounds checks before reading offsets
    if (packetBytes.size() <= DATA_CRC_POS) {
        lastReceiveTime = millis();
        logerr_ln("ERROR  packet too small (size=%d) for DATA_CRC_POS=%d", (int)packetBytes.size(), DATA_CRC_POS);
        return std::nullopt;
    }
    if (packetBytes.size() < DATA_POS) {
        lastReceiveTime = millis();
        logerr_ln("ERROR  packet too small (size=%d) for DATA_POS=%d", (int)packetBytes.size(), DATA_POS);
        return std::nullopt;
    }

    // Validate data section CRC
    loginfo_ln("readReceivedData: checking data section CRC");
    const uint8_t* data = packetBytes.data();
    const size_t data_section_len = packetBytes.size() - DATA_POS;
    uint32_t packet_data_crc = duckutils::toUint32(&data[DATA_CRC_POS]);
    uint32_t computed_data_crc = CRC32::calculate(&data[DATA_POS], data_section_len);
    lastReceiveTime = millis(); // always update last-receive timestamp when a packet is present

    if (computed_data_crc != packet_data_crc) {
        logerr_ln("ERROR data crc mismatch: received: 0x%X, calculated: 0x%X", packet_data_crc, computed_data_crc);
        return std::nullopt;
    }

    // RSSI and SNR are already provided by the SX1302 HAL via the bridge
    // No need to read from lora object since we're using the gateway hardware
    loginfo_ln("RX: packet size: %d", (int)packetBytes.size());

    /*if (rxState != RADIOLIB_ERR_NONE) {
        return std::nullopt;
    }*/

    return packetBytes;

    /* std::vector<uint8_t> packetBytes;
    int packet_length = 0;
    int err = DUCK_ERR_NONE;
    int rxState = DUCK_ERR_NONE;

    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup %s\n", DUCKLORA_ERR_NOT_INITIALIZED);
        return std::nullopt;
    } 

    packet_length = length; */

    /*
    if (packet_length < MIN_PACKET_LENGTH) {
        logerr_ln("ERROR  handlePacket rx data size invalid: %d", packet_length);

        rxState = goToReceiveMode(true); // go back to receive mode and reset the receive flag
    }

    loginfo_ln("readReceivedData() - packet length returns: %d", packet_length);

    packetBytes.resize(packet_length);
    err = lora.readData(packetBytes.data(), packet_length);
    loginfo_ln("readReceivedData() - lora.readData returns: err = %d", err);

    rxState = goToReceiveMode(true);

    if (err != RADIOLIB_ERR_NONE) {
        logerr_ln("ERROR  readReceivedData failed. err = %d", DUCKLORA_ERR_HANDLE_PACKET);
    }

    loginfo_ln("Rx packet: %s", duckutils::toString(packetBytes.data(), packetBytes.size()).c_str());

    loginfo_ln("readReceivedData: checking path offset integrity"); 

    uint8_t* data = packetBytes.data() */

    loginfo_ln("readReceivedData: checking data section CRC");

    lastReceiveTime = millis();
    /*if (rxState != RADIOLIB_ERR_NONE) {
        lastReceiveTime = millis(); //even if rxState is bad, we need to know when we last received
        return std::nullopt;
    }
    lastReceiveTime = millis(); // always update the last receive time 
    std::vector<uint8_t> packetVector(data, data + packet_length);
    return packetVector; */ 
}

int DuckLoRa::sendData(uint8_t* data, int length)
{
    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup");
        return DUCKLORA_ERR_NOT_INITIALIZED;
    }
    /* Gateway mode: skip embedded collision-avoidance delay.
     * The SX1302 JIT queue handles TX timing. Adding a random
     * sleep here would block thread_duck for up to 3 seconds,
     * delaying RREP delivery past the MamaDuck's RX window. */
    return startTransmitData(data, length);
}

void DuckLoRa::delay(size_t size) {
    // Delay the transmission if we have received within the last 5 seconds
    if ((millis() - this->lastReceiveTime) < 5000L) {
        std::uniform_int_distribution<> distrib(0, 3000L);
        std::chrono::milliseconds txdelay(distrib(gen));
        
        loginfo_ln("Last receive was %ld ms ago, delaying transmission by %ld ms", 
                   millis() - this->lastReceiveTime, txdelay.count());
    }
}

int DuckLoRa::sendData(std::vector<uint8_t> data)
{
    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup");
        return DUCKLORA_ERR_NOT_INITIALIZED;
    }
    delay(data.size());
    return startTransmitData(data.data(), data.size());
}

int DuckLoRa::startReceive()
{
    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup");
        return DUCKLORA_ERR_NOT_INITIALIZED;
    }
    
    // In gateway mode (SX1302 HAL), receiving is handled by the concentrator hardware
    // No need to call lora.startReceive() since clusterduckd.c manages reception
    loginfo_ln("Gateway mode: receive handled by SX1302 HAL");
    return DUCK_ERR_NONE;
}

float DuckLoRa::getRSSI()
{ 
    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup");
        return DUCKLORA_ERR_NOT_INITIALIZED;
    }
    return static_cast<float>(last_rx_rssi);
}

float DuckLoRa::getSNR()
{
    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup");
        return DUCKLORA_ERR_NOT_INITIALIZED;
    }
    return last_rx_snr;
}

uint32_t DuckLoRa::getFreqHz()
{
    return last_rx_freq_hz;
}

uint8_t DuckLoRa::getRxSF()
{
    return last_rx_sf;
}


int DuckLoRa::standBy()
{ 
    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup");
        return DUCKLORA_ERR_NOT_INITIALIZED;
    }
    // In gateway mode, standby is managed by the SX1302 HAL
    loginfo_ln("Gateway mode: standby handled by SX1302 HAL");
    return DUCK_ERR_NONE;
}

int DuckLoRa::sleep()
{ 
    if (!isSetup) {
        logerr_ln("ERROR  LoRa radio not setup");
        return DUCKLORA_ERR_NOT_INITIALIZED;
    }    
    // In gateway mode, sleep is managed by the SX1302 HAL
    loginfo_ln("Gateway mode: sleep handled by SX1302 HAL");
    return DUCK_ERR_NONE;
}

void DuckLoRa::serviceInterruptFlags() {
/*    if (DuckLoRa::interruptFlags != 0) {

#ifdef CDPCFG_RADIO_SX1262
        // SX1262 flags
        if (DuckLoRa::interruptFlags & RADIOLIB_SX126X_CMD_CLEAR_IRQ_STATUS) {
            loginfo_ln("SX1262 Interrupt flag was set: clear IRQ status");
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX126X_CMD_CLEAR_DEVICE_ERRORS) {
            loginfo_ln("SX1262 Interrupt flag was set: clear device errors");
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX126X_IRQ_CRC_ERR ) {
            loginfo_ln("SX1262 Interrupt flag was set: payload CRC error");
            goToReceiveMode(false);
            lora.standby();
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX126X_IRQ_HEADER_ERR ) {
            loginfo_ln("SX1262 Interrupt flag was set: header CRC error");
            goToReceiveMode(false);
            lora.standby();
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX126X_IRQ_RX_DONE ) {
            loginfo_ln("SX1262 Interrupt flag was set: packet reception complete");
            setReceiveFlag(true);
            lora.standby(); // we are done receiving, go to standby. We can't sleep because read buffer is not empty
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX126X_IRQ_TX_DONE ) {
            loginfo_ln("SX1262 Interrupt flag was set: payload transmission complete");
            lora.finishTransmit();
            goToReceiveMode(false);
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX126X_IRQ_TIMEOUT ) {
            loginfo_ln("SX1262 Interrupt flag was set: timeout");
            goToReceiveMode(false);
        }
#else
        // SX127X flags
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_TIMEOUT) {
            goToReceiveMode(true); // go back to receive mode and reset the receive flag
            loginfo_ln("SX127x Interrupt flag was set: timeout");
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_DONE) {
            loginfo_ln("SX127x Interrupt flag was set: packet reception complete");
            setReceiveFlag(true); // set the receive flag and we stay in receive mode
            lora.standby(); // we are done receiving, go to standby. We can't sleep because read buffer is not empty
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR) {
            goToReceiveMode(true); // go back to receive mode and reset the receive flag
            loginfo_ln("SX127x Interrupt flag was set: payload CRC error");
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_VALID_HEADER) {
            loginfo_ln("SX127x Interrupt flag was set: valid header received");
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE) {
            loginfo_ln("SX127x Interrupt flag was set: payload transmission complete");
            goToReceiveMode(false); // go back to receive mode and reset the receive flag
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_CAD_DONE) {
            loginfo_ln("SX127x Interrupt flag was set: CAD complete");
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_FHSS_CHANGE_CHANNEL) {
            loginfo_ln("SX127x Interrupt flag was set: FHSS change channel");
        }
        if (DuckLoRa::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_CAD_DETECTED) {
            loginfo_ln("SX127x Interrupt flag was set: valid LoRa signal detected during CAD operation");
        }
#endif
        DuckLoRa::interruptFlags = 0;
    } */
}

void DuckLoRa::onInterrupt(void) {
    // Interrupt handling not needed in gateway mode (SX1302 HAL)
}

/* RX radio metadata moved to top of file — see file-scope statics above */

// Callback invoked when packet_forwarder receives packets via the bridge
// This function needs extern linkage (non-static) to match the friend declaration in DuckLoRa.h
void duck_rx_from_forwarder_cb(const uint8_t* payload, uint16_t size,
                               int16_t rssi, float snr,
                               uint32_t freq_hz, uint32_t tmst, uint8_t rf_chain,
                               uint32_t bandwidth_hz, uint8_t datarate_sf, uint8_t coderate)
{
    // Log the topic from the CDP packet (layout: SDUID[0-7] DDUID[8-15] MUID[16-19] TOPIC[20])
    if (size >= TOPIC_POS + 1) {
        uint8_t topic = payload[TOPIC_POS];
        // Print SDUID (source)
        printf("[DUCK_RX_CB] RX CDP packet: topic=%d, size=%d, rssi=%d, snr=%.2f, freq=%u\n",
                topic, size, rssi, snr, freq_hz);
        printf("[DUCK_RX_CB]   SDUID(src) : %02X%02X%02X%02X%02X%02X%02X%02X\n",
                payload[0],payload[1],payload[2],payload[3],
                payload[4],payload[5],payload[6],payload[7]);
        printf("[DUCK_RX_CB]   DDUID(dst) : %02X%02X%02X%02X%02X%02X%02X%02X\n",
                payload[8],payload[9],payload[10],payload[11],
                payload[12],payload[13],payload[14],payload[15]);
        printf("[DUCK_RX_CB]   MUID       : %02X%02X%02X%02X\n",
                payload[16],payload[17],payload[18],payload[19]);
    } else {
        printf("[DUCK_RX_CB] RX data: size=%d (too small for CDP packet), rssi=%d, snr=%.2f\n",
                size, rssi, snr);
    }    // Store the packet in the bridge for readReceivedData() to pick up
    // Use the C++ API to push directly to the polling buffer
    /* Store all RX metadata: frequency, SF, RSSI, SNR for signal scoring and reply routing */
    last_rx_freq_hz = freq_hz;
    last_rx_sf      = datarate_sf;
    last_rx_rssi    = rssi;
    last_rx_snr     = snr;
    printf("[DUCK_RX_CB] Stored rx metadata: freq=%u Hz, SF=%u, rssi=%d, snr=%.2f\n",
           last_rx_freq_hz, last_rx_sf, last_rx_rssi, last_rx_snr);
    cdp_bridge::UplinkPacket pkt;
    pkt.payload.assign(payload, payload + size);
    pkt.freq_hz     = freq_hz;
    pkt.datarate_sf = datarate_sf;
    pkt.rf_chain    = rf_chain;
    pkt.rssi        = rssi;
    pkt.snr         = snr;
    cdp_bridge::push_uplink_packet(pkt);
    
    // CRITICAL: Set the receive flag so hub.processPackets() knows to process packets
    // This mimics the behavior of SX127x/SX1262 interrupt handlers
    DuckLoRa::setReceiveFlag(true);
}

int DuckLoRa::startTransmitData(uint8_t* data, int length) {
    int err = DUCK_ERR_NONE;

    // Log the topic from the serialized CDP packet (layout: SDUID[0-7] DDUID[8-15] MUID[16-19] TOPIC[20])
    if (length >= TOPIC_POS + 1) {
        uint8_t topic = data[TOPIC_POS];
        printf("[DUCK_TX_CB] TX CDP packet: topic=%d, length=%d\n", topic, length);
        printf("[DUCK_TX_CB]   SDUID(src) : %02X%02X%02X%02X%02X%02X%02X%02X\n",
                data[0],data[1],data[2],data[3],
                data[4],data[5],data[6],data[7]);
        printf("[DUCK_TX_CB]   DDUID(dst) : %02X%02X%02X%02X%02X%02X%02X%02X\n",
                data[8],data[9],data[10],data[11],
                data[12],data[13],data[14],data[15]);
        printf("[DUCK_TX_CB]   MUID       : %02X%02X%02X%02X\n",
                data[16],data[17],data[18],data[19]);
    } else {
        printf("[DUCK_TX_CB] TX data: length=%d (too small for CDP packet)\n", length);
    }

    // Enqueue downlink for transmission via the gateway
    // Note: data is already a serialized CdpPacket from prepareForSending()
    // Packet structure: [SDUID(8)][DDUID(8)][MUID(4)][TOPIC(1)][TYPE(1)][HOP(1)][CRC(4)][DATA...]
    /* Reply on the same frequency the RREQ arrived on */
    printf("[DUCK_TX_CB] Enqueuing downlink on freq=%u Hz\n", last_rx_freq_hz);
    cdp_bridge_enqueue_downlink_ext(data, (int)length,
        last_rx_freq_hz, /* reply on same RX channel */
        0,               /* tmst unused for IMMEDIATE */
        CDPCFG_RF_LORA_TXPOW,
        125000,          /* bw_hz: 125 kHz */
        7,               /* sf: SF7 */
        1,               /* cr: 4/5 */
        0                /* rf_chain */
    );
    return err;
}

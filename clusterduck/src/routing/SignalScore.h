//
// Created by brent on 10/21/2025.
//
/**
 * @file SignalScore.h
 * @brief Contains the SignalScore structure for signal quality evaluation
 */
#ifndef CLUSTERDUCK_PROTOCOL_SIGNALSCORE_H
#define CLUSTERDUCK_PROTOCOL_SIGNALSCORE_H
/**
 * @struct SignalScore
 * @brief Structure to store signal quality metrics
 *
 * This struct contains signal quality measurements including RSSI (Received Signal Strength Indicator),
 * SNR (Signal-to-Noise Ratio), and a calculated overall signal score.
 */
struct SignalScore {
    /**
    * @brief Received Signal Strength Indicator in dBm
    *
    * Measures the power of the received radio signal. Typically ranges from
    * -120 (very weak) to -30 (very strong) for wireless communications.
    */
    float rssi;

    /**
     * @brief Signal-to-Noise Ratio in dB
     *
     * Measures the ratio of signal power to noise power. Higher values
     * indicate a cleaner signal with less noise interference.
     */
    float snr;

    /**
     * @brief Calculated overall signal quality score
     *
     * A derived metric combining RSSI and SNR to provide a single value
     * representing the overall quality of the signal connection.
     */
    float signalScore;

    /**
     * @brief Receive frequency in Hz (e.g. 922800000 for 922.8 MHz).
     *
     * Populated from the SX1302 HAL uplink metadata so the router can
     * record which channel a neighbor was heard on and reply on the same
     * frequency.  Defaults to 0 when not available (e.g. on Arduino nodes).
     */
    uint32_t freq_hz = 0;

    /**
     * @brief LoRa Spreading Factor used by the sender (5–12).
     *
     * Stored alongside freq_hz so that downlink replies can match the
     * sender's SF.  Defaults to 0 when not available.
     */
    uint8_t sf = 0;
};
#endif //CLUSTERDUCK_PROTOCOL_SIGNALSCORE_H

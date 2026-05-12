/**
 * @file ClusterDuckBridge.h
 * @brief Unified bridge between SX1302 HAL (C) and ClusterDuck Protocol (C++)
 * 
 * This bridge provides:
 * 1. Uplink path: SX1302 concentrator -> ClusterDuck Protocol
 * 2. Downlink path: ClusterDuck Protocol -> SX1302 concentrator
 * 
 * Thread-safe design allows concurrent access from multiple threads.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
#include <vector>
#include <optional>
#endif

/* ========================================================================== */
/* === C INTERFACE (for clusterduckd.c and other C code) =================== */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* --- UPLINK (RX): Concentrator -> ClusterDuck ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Callback function type for received packets
 * 
 * Called by the bridge when a packet is received from the concentrator.
 * The ClusterDuck Protocol registers this callback to receive packets.
 */
typedef void (*cdp_rx_callback_t)(
    const uint8_t* payload, uint16_t size,
    int16_t rssi, float snr,
    uint32_t freq_hz, uint32_t tmst, uint8_t rf_chain,
    uint32_t bandwidth_hz, uint8_t datarate_sf, uint8_t coderate
);

/**
 * @brief Register a callback for received packets
 * 
 * ClusterDuck Protocol calls this during initialization to register
 * its packet handler.
 * 
 * @param cb Callback function pointer (NULL to unregister)
 */
void cdp_bridge_register_rx_callback(cdp_rx_callback_t cb);

/**
 * @brief Push a received packet to the bridge
 * 
 * Called by clusterduckd.c when a packet is received from the concentrator.
 * This will either:
 * 1. Call the registered callback immediately, OR
 * 2. Queue the packet for later retrieval (depending on mode)
 * 
 * @param payload Packet payload data
 * @param size Payload size in bytes
 * @param rssi Received signal strength indicator (dBm)
 * @param snr Signal-to-noise ratio (dB)
 * @param freq_hz Center frequency (Hz)
 * @param tmst Internal timestamp
 * @param rf_chain RF chain that received the packet
 * @param bandwidth_hz Modulation bandwidth (Hz)
 * @param datarate_sf Spreading factor
 * @param coderate Coding rate
 */
void cdp_bridge_handle_uplink(
    const uint8_t* payload, uint16_t size,
    int16_t rssi, float snr,
    uint32_t freq_hz, uint32_t tmst, uint8_t rf_chain,
    uint32_t bandwidth_hz, uint8_t datarate_sf, uint8_t coderate
);

/**
 * @brief Block until an uplink packet is queued or the timeout expires.
 *
 * Called by thread_duck instead of a fixed sleep so that CDP packets are
 * processed immediately rather than waiting up to timeout_ms milliseconds.
 *
 * @param timeout_ms Maximum time to wait in milliseconds.
 */
void cdp_bridge_wait_for_uplink(int timeout_ms);

/* -------------------------------------------------------------------------- */
/* --- DOWNLINK (TX): ClusterDuck -> Concentrator --------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enqueue a packet for downlink transmission (simple version)
 * 
 * ClusterDuck Protocol calls this to send a packet through the concentrator.
 * Uses default radio parameters.
 * 
 * @param payload Packet payload data
 * @param size Payload size in bytes
 * @return 0 on success, negative on error
 */
int cdp_bridge_enqueue_downlink(const uint8_t* payload, uint16_t size);

/**
 * @brief Enqueue a packet for downlink transmission (extended version)
 * 
 * Allows specifying custom radio parameters. Any parameter set to 0
 * will use default values.
 * 
 * @param payload Packet payload data
 * @param size Payload size in bytes
 * @param freq_hz Transmission frequency (Hz), 0 = default
 * @param tmst Transmission timestamp, 0 = ASAP
 * @param tx_power_dbm Transmission power (dBm), 0 = default
 * @param bw_hz Bandwidth (Hz), 0 = default
 * @param sf Spreading factor, 0 = default
 * @param cr Coding rate, 0 = default
 * @param rf_chain RF chain to use, 0 = default
 * @return 0 on success, negative on error
 */
int cdp_bridge_enqueue_downlink_ext(
    const uint8_t* payload, uint16_t size,
    uint32_t freq_hz, uint32_t tmst, int16_t tx_power_dbm,
    uint32_t bw_hz, uint8_t sf, uint8_t cr, uint8_t rf_chain
);

/**
 * @brief Pop a packet from the downlink queue
 * 
 * Called by clusterduckd.c to retrieve packets for transmission.
 * 
 * @param out_buf Buffer to store packet payload
 * @param inout_capacity Input: buffer size, Output: actual payload size
 * @param freq_hz Output: transmission frequency (may be 0 if default)
 * @param tmst Output: transmission timestamp
 * @param tx_power_dbm Output: transmission power
 * @param bw_hz Output: bandwidth
 * @param sf Output: spreading factor
 * @param cr Output: coding rate
 * @param rf_chain Output: RF chain
 * @return 0 on success, 1 if queue empty, negative on error
 */
int cdp_bridge_pop_downlink(
    uint8_t* out_buf, uint16_t* inout_capacity,
    uint32_t* freq_hz, uint32_t* tmst, int16_t* tx_power_dbm,
    uint32_t* bw_hz, uint8_t* sf, uint8_t* cr, uint8_t* rf_chain
);

/* -------------------------------------------------------------------------- */
/* --- DIAGNOSTICS ---------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get bridge statistics
 * 
 * @param uplink_count Total uplink packets processed
 * @param downlink_count Total downlink packets queued
 * @param downlink_queue_size Current downlink queue size
 */
void cdp_bridge_get_stats(
    uint32_t* uplink_count,
    uint32_t* downlink_count,
    uint32_t* downlink_queue_size
);

/**
 * @brief Reset bridge statistics
 */
void cdp_bridge_reset_stats(void);

#ifdef __cplusplus
}
#endif

/* ========================================================================== */
/* === C++ INTERFACE (for ClusterDuck Protocol) ============================ */
/* ========================================================================== */

#ifdef __cplusplus

namespace cdp_bridge {

/**
 * @brief Check if an uplink packet is available
 * 
 * Alternative to callback-based reception. ClusterDuck can poll
 * for packets instead of using callbacks.
 * 
 * @return true if a packet is waiting
 */
bool has_uplink_packet();

/**
 * @brief Pop an uplink packet from the queue
 * 
 * @return Packet data if available, std::nullopt otherwise
 */
std::optional<std::vector<uint8_t>> pop_uplink_packet();

/**
 * @brief Push an uplink packet (for testing/simulation)
 * 
 * Allows injecting packets directly into the bridge for testing.
 * 
 * @param payload Packet data
 */
void push_uplink_packet(const std::vector<uint8_t>& payload);

} // namespace cdp_bridge

#endif // __cplusplus

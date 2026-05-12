/**
 * @file ClusterDuckBridge.cpp
 * @brief Implementation of unified bridge between SX1302 HAL and ClusterDuck Protocol
 */

#include "ClusterDuckBridge.h"
#include <deque>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <optional>
#include <cstring>
#include <cstdio>

/* ========================================================================== */
/* === INTERNAL DATA STRUCTURES ============================================= */
/* ========================================================================== */

/**
 * @brief Downlink packet with metadata
 */
struct DownlinkPacket {
    std::vector<uint8_t> payload;
    uint32_t freq_hz = 0;
    uint32_t tmst = 0;
    int16_t  tx_power_dbm = 0;
    uint32_t bw_hz = 0;
    uint8_t  sf = 0;
    uint8_t  cr = 0;
    uint8_t  rf_chain = 0;
};

/**
 * @brief Bridge state and queues
 */
static struct {
    // Uplink state
    std::mutex uplink_mutex;
    std::condition_variable uplink_cv;  /* signalled when a packet is queued */
    cdp_rx_callback_t rx_callback = nullptr;
    std::deque<std::vector<uint8_t>> uplink_queue;  // Queue to handle multiple concurrent packets
    
    // Downlink state
    std::mutex downlink_mutex;
    std::deque<DownlinkPacket> downlink_queue;
    
    // Statistics
    std::mutex stats_mutex;
    uint32_t uplink_count = 0;
    uint32_t downlink_count = 0;
} g_bridge;

/* ========================================================================== */
/* === UPLINK (RX) IMPLEMENTATION =========================================== */
/* ========================================================================== */

extern "C" {

void cdp_bridge_register_rx_callback(cdp_rx_callback_t cb) {
    std::lock_guard<std::mutex> lock(g_bridge.uplink_mutex);
    g_bridge.rx_callback = cb;
    printf("[CDP_BRIDGE] RX callback %s (cb=%p)\n", 
           cb ? "REGISTERED" : "UNREGISTERED", (void*)cb);
}

void cdp_bridge_handle_uplink(
    const uint8_t* payload, uint16_t size,
    int16_t rssi, float snr,
    uint32_t freq_hz, uint32_t tmst, uint8_t rf_chain,
    uint32_t bandwidth_hz, uint8_t datarate_sf, uint8_t coderate)
{
    if (!payload || size == 0) {
        printf("[CDP_BRIDGE] WARNING: Invalid uplink packet (payload=%p, size=%u)\n", 
               (void*)payload, size);
        return;
    }
    
    printf("[CDP_BRIDGE] Uplink: size=%u, rssi=%d, snr=%.1f, freq=%u Hz\n", 
           size, rssi, snr, freq_hz);
    
    // Update statistics
    {
        std::lock_guard<std::mutex> lock(g_bridge.stats_mutex);
        g_bridge.uplink_count++;
    }
    
    // Callback mode: call registered callback immediately
    if (g_bridge.rx_callback) {
        printf("[CDP_BRIDGE] Calling registered callback\n");
        g_bridge.rx_callback(payload, size, rssi, snr, freq_hz, tmst, rf_chain,
                            bandwidth_hz, datarate_sf, coderate);
    } else {
        // Polling mode: store packet in queue for later retrieval
        printf("[CDP_BRIDGE] No callback registered, storing in queue for polling\n");
        {
            std::lock_guard<std::mutex> lock(g_bridge.uplink_mutex);
            const size_t MAX_QUEUE_SIZE = 50;
            if (g_bridge.uplink_queue.size() >= MAX_QUEUE_SIZE) {
                printf("[CDP_BRIDGE] WARNING: Polling queue full, dropping oldest\n");
                g_bridge.uplink_queue.pop_front();
            }
            g_bridge.uplink_queue.emplace_back(payload, payload + size);
        }
        /* Wake thread_duck immediately so it doesn't wait for its sleep to expire */
        g_bridge.uplink_cv.notify_one();
    }
}

void cdp_bridge_wait_for_uplink(int timeout_ms) {
    std::unique_lock<std::mutex> lock(g_bridge.uplink_mutex);
    if (!g_bridge.uplink_queue.empty()) return; /* packet already waiting */
    g_bridge.uplink_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms));
}

} // extern "C"

/* ========================================================================== */
/* === DOWNLINK (TX) IMPLEMENTATION ========================================= */
/* ========================================================================== */

extern "C" {

int cdp_bridge_enqueue_downlink(const uint8_t* payload, uint16_t size) {
    if (!payload || size == 0) {
        return -1;
    }
    
    DownlinkPacket pkt;
    pkt.payload.assign(payload, payload + size);
    
    std::lock_guard<std::mutex> lock(g_bridge.downlink_mutex);
    g_bridge.downlink_queue.emplace_back(std::move(pkt));
    
    // Update statistics
    {
        std::lock_guard<std::mutex> stats_lock(g_bridge.stats_mutex);
        g_bridge.downlink_count++;
    }
    
    printf("[CDP_BRIDGE] Enqueued downlink packet: size=%u, queue_size=%zu\n", 
           size, g_bridge.downlink_queue.size());
    
    return 0;
}

int cdp_bridge_enqueue_downlink_ext(
    const uint8_t* payload, uint16_t size,
    uint32_t freq_hz, uint32_t tmst, int16_t tx_power_dbm,
    uint32_t bw_hz, uint8_t sf, uint8_t cr, uint8_t rf_chain)
{
    if (!payload || size == 0) {
        return -1;
    }
    
    DownlinkPacket pkt;
    pkt.payload.assign(payload, payload + size);
    pkt.freq_hz = freq_hz;
    pkt.tmst = tmst;
    pkt.tx_power_dbm = tx_power_dbm;
    pkt.bw_hz = bw_hz;
    pkt.sf = sf;
    pkt.cr = cr;
    pkt.rf_chain = rf_chain;
    
    std::lock_guard<std::mutex> lock(g_bridge.downlink_mutex);
    g_bridge.downlink_queue.emplace_back(std::move(pkt));
    
    // Update statistics
    {
        std::lock_guard<std::mutex> stats_lock(g_bridge.stats_mutex);
        g_bridge.downlink_count++;
    }
    
    printf("[CDP_BRIDGE] Enqueued downlink packet (ext): size=%u, freq=%u Hz, queue_size=%zu\n", 
           size, freq_hz, g_bridge.downlink_queue.size());
    
    return 0;
}

int cdp_bridge_pop_downlink(
    uint8_t* out_buf, uint16_t* inout_capacity,
    uint32_t* freq_hz, uint32_t* tmst, int16_t* tx_power_dbm,
    uint32_t* bw_hz, uint8_t* sf, uint8_t* cr, uint8_t* rf_chain)
{
    if (!inout_capacity) {
        return -1;
    }
    
    std::lock_guard<std::mutex> lock(g_bridge.downlink_mutex);
    
    // Queue empty
    if (g_bridge.downlink_queue.empty()) {
        return 1;
    }
    
    // Get packet from front of queue
    DownlinkPacket pkt = std::move(g_bridge.downlink_queue.front());
    g_bridge.downlink_queue.pop_front();
    
    // Check buffer capacity
    if (!out_buf || *inout_capacity < pkt.payload.size()) {
        *inout_capacity = 0;
        printf("[CDP_BRIDGE] ERROR: Buffer too small for downlink packet\n");
        return -2;
    }
    
    // Copy payload
    std::memcpy(out_buf, pkt.payload.data(), pkt.payload.size());
    *inout_capacity = static_cast<uint16_t>(pkt.payload.size());
    
    // Copy metadata
    if (freq_hz)      *freq_hz = pkt.freq_hz;
    if (tmst)         *tmst = pkt.tmst;
    if (tx_power_dbm) *tx_power_dbm = pkt.tx_power_dbm;
    if (bw_hz)        *bw_hz = pkt.bw_hz;
    if (sf)           *sf = pkt.sf;
    if (cr)           *cr = pkt.cr;
    if (rf_chain)     *rf_chain = pkt.rf_chain;
    
    printf("[CDP_BRIDGE] Popped downlink packet: size=%u, remaining=%zu\n", 
           *inout_capacity, g_bridge.downlink_queue.size());
    
    return 0;
}

} // extern "C"

/* ========================================================================== */
/* === DIAGNOSTICS ========================================================== */
/* ========================================================================== */

extern "C" {

void cdp_bridge_get_stats(
    uint32_t* uplink_count,
    uint32_t* downlink_count,
    uint32_t* downlink_queue_size)
{
    std::lock_guard<std::mutex> stats_lock(g_bridge.stats_mutex);
    std::lock_guard<std::mutex> downlink_lock(g_bridge.downlink_mutex);
    
    if (uplink_count)        *uplink_count = g_bridge.uplink_count;
    if (downlink_count)      *downlink_count = g_bridge.downlink_count;
    if (downlink_queue_size) *downlink_queue_size = static_cast<uint32_t>(g_bridge.downlink_queue.size());
}

void cdp_bridge_reset_stats(void) {
    std::lock_guard<std::mutex> lock(g_bridge.stats_mutex);
    g_bridge.uplink_count = 0;
    g_bridge.downlink_count = 0;
}

} // extern "C"

/* ========================================================================== */
/* === C++ INTERFACE (for ClusterDuck Protocol) ============================ */
/* ========================================================================== */

namespace cdp_bridge {

bool has_uplink_packet() {
    std::lock_guard<std::mutex> lock(g_bridge.uplink_mutex);
    return !g_bridge.uplink_queue.empty();
}

std::optional<std::vector<uint8_t>> pop_uplink_packet() {
    std::lock_guard<std::mutex> lock(g_bridge.uplink_mutex);
    if (g_bridge.uplink_queue.empty()) {
        return std::nullopt;
    }
    
    auto packet = std::move(g_bridge.uplink_queue.front());
    g_bridge.uplink_queue.pop_front();
    
    printf("[CDP_BRIDGE] Popped uplink packet from queue (remaining=%zu): size=%zu\n", 
           g_bridge.uplink_queue.size(), packet.size());
    return packet;
}

void push_uplink_packet(const std::vector<uint8_t>& payload) {
    std::lock_guard<std::mutex> lock(g_bridge.uplink_mutex);
    
    // Limit queue size to prevent memory exhaustion
    const size_t MAX_QUEUE_SIZE = 50;
    if (g_bridge.uplink_queue.size() >= MAX_QUEUE_SIZE) {
        printf("[CDP_BRIDGE] WARNING: Uplink queue full (%zu packets), dropping oldest\n", 
               g_bridge.uplink_queue.size());
        g_bridge.uplink_queue.pop_front();
    }
    
    g_bridge.uplink_queue.push_back(payload);
    printf("[CDP_BRIDGE] Pushed uplink packet to queue (total=%zu): size=%zu\n", 
           g_bridge.uplink_queue.size(), payload.size());
}

} // namespace cdp_bridge

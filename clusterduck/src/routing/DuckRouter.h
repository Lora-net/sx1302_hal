/**
 * @file DuckRouter.h
 * @brief Internal CDP mesh routing table and network-state FSM.
 *
 * ## Routing table design
 *
 * The routing table is a hash map keyed on **destination device UID** (Duid).
 * Each key maps to a list of @ref Neighbor records — one per observed next-hop
 * path to that destination.  On every lookup the list is sorted by signal
 * quality (descending) and the best candidate is returned.
 *
 * ```
 * routingTable:
 *   "MAMA_A" -> [ Neighbor(nextHop=LINK_1, score=90, lastSeen=T),
 *                 Neighbor(nextHop=LINK_2, score=72, lastSeen=T-30s) ]
 *   "MAMA_B" -> [ Neighbor(nextHop=MAMA_A, score=85, lastSeen=T) ]
 *   ...
 * ```
 *
 * ## TTL-based eviction
 *
 * Entries are passively evicted on each call to @ref insertIntoRoutingTable.
 * Any @ref Neighbor record whose `lastSeen` timestamp is older than the
 * configured TTL (default 5 minutes / 300 000 ms) is removed.  When all
 * records for a destination are removed the destination key is also dropped,
 * keeping the table bounded without an artificial entry-count cap.
 *
 * ## Network-state FSM
 *
 * ```
 *  SEARCHING ──── network found ───► PUBLIC
 *  PUBLIC    ──── signal lost   ───► SEARCHING
 *  PUBLIC    ──── explicit disc ───► DISCONNECTED
 *  DISCONNECTED ─ retry         ───► SEARCHING
 * ```
 *
 * @date 2025-07-24 (created)
 * @date 2026-05-10 (TTL eviction enhancement)
 */
#ifndef DUCKROUTER_H_
#define DUCKROUTER_H_

#include <map>
#include <list>
#include <optional>
#include "bloomfilter.h"
#include "Neighbor.h"

/**
 * @brief Operational state of the CDP network connection.
 *
 * | State        | Meaning                                              |
 * |--------------|------------------------------------------------------|
 * | SEARCHING    | No CDP network found yet; broadcasting RREQ probes. |
 * | PUBLIC       | Joined a CDP network; normal packet forwarding.      |
 * | DISCONNECTED | Explicitly disconnected; not attempting to rejoin.   |
 */
enum class NetworkState {SEARCHING, PUBLIC, DISCONNECTED};

class DuckRouter {
    public:
        DuckRouter() = default;
        ~DuckRouter() = default;
        BloomFilter& getFilter();
        NetworkState getNetworkState(){ return networkState; };

                /**
         * @brief Insert or update a routing entry and evict stale records.
         *
         * Behaviour:
         * - Calls @ref evictStaleEntries() first to prune any expired records.
         * - If no entry exists for @p deviceID, creates a new destination key
         *   and adds the neighbor record.
         * - If an entry already exists, removes any older record for the same
         *   next-hop before appending the fresh one (prevents duplicates while
         *   preserving alternative paths via different next-hops).
         *
         * @param deviceID   Ultimate destination device UID.
         * @param nextHop    Immediate next-hop UID to reach @p deviceID.
         * @param signalInfo Signal quality metrics (RSSI, SNR, composite score).
         */
        void insertIntoRoutingTable(Duid deviceID, Duid nextHop, SignalScore signalInfo);

        /**
         * @brief Return the best next-hop UID to reach a destination.
         *
         * Sorts the neighbor list for @p targetDeviceId by descending signal
         * score and returns the front element's device ID.
         *
         * @param targetDeviceId  Destination device UID to look up.
         * @return The next-hop Duid to forward to, or std::nullopt if the
         *         destination is not in the routing table.
         */
        std::optional<Duid> getBestNextHop(Duid targetDeviceId);

        /**
         * @brief NetworkState if the Duck joins or disconnects from a CDP network
         * @param newState The new NetworkState to join
         */
        void setNetworkState(NetworkState newState){
            if (networkState != newState) {
                NetworkState oldState = networkState;
                networkTransition(oldState, newState);
            }
        }
    protected:
        /**
         * @brief Evict stale neighbor records from the routing table.
         *
         * Iterates over every destination in the routing table and removes
         * individual @ref Neighbor records whose `lastSeen` age exceeds
         * @p ttl_ms.  Destinations with no remaining records are also removed.
         *
         * This replaces the original fixed-size `CullRoutingTable(maxSize=3)`
         * approach, which arbitrarily discarded routes once a 3-entry limit was
         * reached (with no regard for entry age or signal quality).
         *
         * ### Why passive eviction?
         * `evictStaleEntries()` is called from @ref insertIntoRoutingTable on
         * every new packet, so the table is cleaned continuously without needing
         * a dedicated background thread or timer.
         *
         * ### Tuning the TTL
         * - Dense, high-traffic networks: lower TTL (e.g. 60 000 ms) to keep
         *   only recently active routes.
         * - Sparse or low-duty-cycle networks: raise TTL (e.g. 600 000 ms) to
         *   retain routes across long quiet periods.
         *
         * @param ttl_ms  Maximum age of a neighbor record in milliseconds.
         *                Default is 300 000 ms (5 minutes).
         */
        void evictStaleEntries(unsigned long ttl_ms = 300000UL);

    private:
        std::unordered_map<std::string, std::list<Neighbor>> routingTable;
        BloomFilter filter;
        NetworkState networkState = NetworkState::SEARCHING;

        /**
         * @brief NetworkState transition for NetworkState FSM
         * @param oldState NetworkState to transition out of
         * @param newState NetworkState to transition in to
         */
        void networkTransition(NetworkState oldState, NetworkState newState){
            if (oldState == NetworkState::SEARCHING && newState == NetworkState::PUBLIC) {
                loginfo_ln("[ROUTER] Public network joined.");
                networkState = newState;
            } else if (oldState == NetworkState::PUBLIC && newState == NetworkState::DISCONNECTED){
                networkState = newState;
                loginfo_ln("[ROUTER] Successfully disconnected from CDP network.");
            } else if(oldState == NetworkState::PUBLIC && newState == NetworkState::SEARCHING){
                networkState = newState;
                loginfo_ln("[ROUTER] Lost connection to CDP Network.");
            } else if(oldState == NetworkState::DISCONNECTED && newState == NetworkState::SEARCHING){
                networkState = newState;
                logdbg_ln("[ROUTER] Leaving disconnected state, looking for CDP networks.");
            } else {
                logdbg_ln("[ROUTER] Invalid network state transition!");
            }
        }
        
};
  #endif
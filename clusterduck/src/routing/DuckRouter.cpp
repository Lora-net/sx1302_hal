#include "DuckRouter.h"

void DuckRouter::insertIntoRoutingTable(Duid deviceID, Duid nextHop, SignalScore signalInfo) {

    // Passively evict stale entries on every insert so the table stays bounded
    // without a separate cleanup thread.  Uses the default 5-minute TTL.
    evictStaleEntries();

    Neighbor neighborRecord(deviceID, nextHop, signalInfo, millis());
    auto index = routingTable.find(duckutils::toString(deviceID));

    if (index == routingTable.end()) {
        // First time we see this destination — create a new entry.
        // Guard: do not add a self-loop (e.g. Link1276 -> Link1276 via Mama1262).
        std::list<Neighbor> neighborList;
        neighborList.push_back(neighborRecord);
        routingTable.insert(std::make_pair(neighborRecord.getDeviceId(), neighborList));
    } else {
        // Destination already known.  Remove any older record that uses the same
        // next-hop so we don't accumulate duplicates, then append the fresh one.
        // Records via *different* next-hops are kept to preserve alternative paths.
        index->second.remove_if([neighborRecord](const Neighbor& n) {
            return n.getLastSeen() < neighborRecord.getLastSeen() && n.getDeviceId() == neighborRecord.getDeviceId();
        });
        index->second.push_back(neighborRecord);
    }
};

std::optional<Duid> DuckRouter::getBestNextHop(Duid targetDeviceId){
    // Look up all known paths to the requested destination.
    auto nextHopRecord = routingTable.find(duckutils::toString(targetDeviceId));
    if (nextHopRecord == routingTable.end()) {
        return std::nullopt; // Destination not in table — caller should flood or drop.
    }

    // Sort candidates by descending signal score so the best path is at front.
    // Neighbor::operator> compares routingScore, which is a composite of RSSI and SNR.
    nextHopRecord->second.sort(std::greater<>());

    std::string nextHopStr = nextHopRecord->second.front().getDeviceId();
    Duid nextHopId;
    std::copy(nextHopStr.begin(), nextHopStr.end(), nextHopId.begin());

    // TODO: if the best candidate's TTL is about to expire, pre-emptively send
    //       a new RREQ to refresh the route before it goes stale:
    // if (nextHop.ttl > 0) { sendRREQ(targetDeviceId); }

    return nextHopId;
};

void DuckRouter::evictStaleEntries(unsigned long ttl_ms) {
    unsigned long now = millis();

    for (auto it = routingTable.begin(); it != routingTable.end(); ) {
        // Step 1 — remove individual Neighbor records that are too old.
        // A record is stale when:  now - lastSeen > ttl_ms
        // Using unsigned arithmetic avoids sign issues on platforms where
        // millis() wraps around (e.g. after ~49 days on 32-bit systems).
        it->second.remove_if([now, ttl_ms](const Neighbor& n) {
            return (now - n.getLastSeen()) > ttl_ms;
        });

        // Step 2 — if all next-hop candidates for this destination were evicted,
        // remove the destination key itself so the table doesn't hold empty lists.
        if (it->second.empty()) {
            loginfo_ln("[ROUTER] Evicted stale routing entry: %s", it->first.c_str());
            it = routingTable.erase(it); // erase() returns the next valid iterator.
        } else {
            ++it;
        }
    }
};

BloomFilter& DuckRouter::getFilter(){
    return filter; //just call the bloomfilter function here?
};

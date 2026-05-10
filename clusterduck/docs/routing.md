# Mesh Routing: TTL-Based Eviction

## Overview

The CDP routing table (`DuckRouter`) maps **destination device UIDs** to a list
of candidate **next-hop** neighbors. Entries are now evicted based on age (TTL)
rather than table size, allowing the gateway to maintain routes to an arbitrary
number of MamaDucks.

---

## Background: original limitation

The original implementation included a `CullRoutingTable(maxSize = 3)` function
that discarded routing entries once the table exceeded 3 destinations:

```cpp
// OLD — removed
void CullRoutingTable(size_t maxSize = 3) {
    while (routingTable.size() > maxSize) {
        auto it = std::prev(routingTable.end(), 1); // last-inserted = arbitrary
        routingTable.erase(it);
    }
}
```

Problems:
- Hard ceiling of 3 reachable devices — unusable for real deployments
- Eviction policy was insertion-order (hash map), not LRU or signal-quality
- `CullRoutingTable` was `protected` and **never actually called**, so the
  table grew unboundedly with stale entries accumulating over time

---

## Current design: passive TTL eviction

`evictStaleEntries(ttl_ms)` is called at the top of every
`insertIntoRoutingTable()` call. No background thread or timer is needed.

```
insertIntoRoutingTable()
        │
        ├─ evictStaleEntries(300 000 ms)
        │       │
        │       └─ for each destination key:
        │               remove Neighbor records where (now - lastSeen) > ttl_ms
        │               if list is empty → erase destination key
        │
        └─ insert / update new Neighbor record
```

### Routing table data structure

```
routingTable (unordered_map):
  "MAMA_A" → [ Neighbor(nextHop=LINK_1, score=90, lastSeen=T),
               Neighbor(nextHop=LINK_2, score=72, lastSeen=T-30s) ]
  "MAMA_B" → [ Neighbor(nextHop=MAMA_A, score=85, lastSeen=T) ]
  "MAMA_C" → [ Neighbor(nextHop=LINK_3, score=60, lastSeen=T-4min) ]
```

Each **destination key** holds multiple `Neighbor` records — one per observed
next-hop path. Alternative paths (via different relays) are preserved as long
as they are within TTL. `getBestNextHop()` sorts the list by descending signal
score on each lookup.

---

## API

### `insertIntoRoutingTable(deviceID, nextHop, signalInfo)`

Inserts or updates a route. Eviction runs automatically before insert.

```cpp
router.insertIntoRoutingTable(mama_a_duid, link_1_duid, {rssi, snr, score});
```

### `getBestNextHop(targetDeviceId)`

Returns the next-hop with the highest signal score for a given destination,
or `std::nullopt` if no route exists (caller should flood or drop).

```cpp
std::optional<Duid> hop = router.getBestNextHop(target);
if (!hop.has_value()) {
    // flood or send RREQ
}
```

### `evictStaleEntries(ttl_ms = 300 000)`

Protected method, called automatically. Can be called explicitly with a custom
TTL if needed:

```cpp
router.evictStaleEntries(60000UL);   // 1-minute TTL for dense networks
router.evictStaleEntries(600000UL);  // 10-minute TTL for sparse networks
```

---

## TTL tuning guide

| Deployment scenario | Recommended TTL | Rationale |
|---|---|---|
| Dense urban, high duty cycle | 60 000 ms (1 min) | Frequent updates; stale routes purged quickly |
| Default / mixed | 300 000 ms (5 min) | Balances freshness vs. sparse-traffic retention |
| Rural / low duty cycle | 600 000 ms (10 min) | Devices transmit infrequently; retain routes longer |
| Disaster / mobile nodes | 120 000 ms (2 min) | Topology changes rapidly; discard stale routes fast |

The TTL default (5 minutes) is defined in `DuckRouter.h`:

```cpp
void evictStaleEntries(unsigned long ttl_ms = 300000UL);
```

---

## Network-state FSM

`DuckRouter` also manages the logical connection state of the duck:

```
  SEARCHING ──── network found ───► PUBLIC
  PUBLIC    ──── signal lost   ───► SEARCHING
  PUBLIC    ──── explicit disc ───► DISCONNECTED
  DISCONNECTED ─ retry         ───► SEARCHING
```

| State | Meaning |
|---|---|
| `SEARCHING` | No CDP network found; broadcasting RREQ probes |
| `PUBLIC` | Joined a CDP network; normal packet forwarding active |
| `DISCONNECTED` | Explicitly disconnected; not attempting to rejoin |

A `PapaDuck` (SX1302 gateway) skips the `SEARCHING` phase entirely and is set
directly to `PUBLIC` on startup — it is always the sink/root of the mesh.

---

## Bloom filter deduplication

Each `DuckRouter` instance holds a `BloomFilter` used to detect already-seen
packet MUIDs (Message UIDs). This prevents relay loops without maintaining a
full seen-packet list:

```cpp
BloomFilter& filter = router.getFilter();
if (filter.contains(muid)) { /* drop duplicate */ }
else { filter.add(muid); /* relay */ }
```

The Bloom filter has a small false-positive rate — a packet may occasionally be
dropped as a spurious duplicate. This is an acceptable trade-off for the
significant memory savings over an exact set on resource-constrained hardware.

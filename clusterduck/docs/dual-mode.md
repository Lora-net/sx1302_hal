# Dual-Mode Operation: LoRaWAN + ClusterDuck Protocol

## Overview

`clusterduckd` can simultaneously forward **LoRaWAN** packets to a Network Server
(Chirpstack, TTN, etc.) and process **CDP (ClusterDuck Protocol)** mesh packets —
all from a single SX1302 concentrator hardware, running as one process.

This eliminates the need for two separate gateways or two competing processes
trying to open the same concentrator.

---

## Architecture

```
                        SX1302 Concentrator
                        lgw_receive() [single call, all channels]
                                │
                     ┌──────────▼──────────┐
                     │  Packet Classifier   │
                     │  is_lorawan_packet() │
                     └──────────┬──────────┘
                                │
               ┌────────────────┴────────────────┐
               │                                  │
               ▼                                  ▼
    LoRaWAN MHDR valid                   CDP (anything else)
               │                                  │
               ▼                                  ▼
   Semtech PUSH_DATA (UDP)           cdp_bridge_handle_uplink()
  ──────────────────────►                        │
   Network Server                                ▼
   (Chirpstack / TTN)               ClusterDuck Protocol stack
                                    PapaDuck → MQTT → mesh
```

### Packet Classification

The classifier `is_lorawan_packet()` in `clusterduckd.c` inspects **byte 0
(MHDR)** of every received packet per LoRaWAN spec §4.2:

| MHDR bits | Field  | Valid LoRaWAN value |
|-----------|--------|---------------------|
| `[7:5]`   | MType  | `000`–`101` (not `110` = RFU) |
| `[4:2]`   | RFU    | `000` |
| `[1:0]`   | Major  | `00`  |

A CDP packet's first byte is part of a `SDUID` (Source Device UID) and carries
no such structured bit fields, so the separation is clean in practice.

```c
static bool is_lorawan_packet(const uint8_t *payload, uint16_t size) {
    if (size < 8) return false;
    uint8_t mhdr  = payload[0];
    if ((mhdr & 0x1F) != 0x00) return false; /* RFU + Major must be 0 */
    uint8_t mtype = (mhdr >> 5) & 0x07;
    if (mtype == 0x06) return false;          /* MType=RFU — discard */
    return true;
}
```

---

## Configuration

Dual-mode is **off by default**. To enable it, set the following in your
`global_conf.json` under `gateway_conf`:

```json
"gateway_conf": {
    "server_address": "chirpstack.example.com",
    "serv_port_up":   1700,
    "serv_port_down": 1700,

    "lorawan_forward_enabled": true
}
```

> **Note:** `serv_port_up` / `serv_port_down` and `server_address` are the same
> fields used by the standard `lora_pkt_fwd`. Port `1700` is the default
> Semtech UDP protocol port used by Chirpstack and TTN.

When `lorawan_forward_enabled` is `false` (default), all packets are sent to the
CDP bridge. No upstream socket is opened and no Network Server communication
takes place.

### Regional config files

All bundled regional config files already contain the key set to `false`:

```
global_conf.json.sx1250.EU868
global_conf.json.sx1250.US915
global_conf.json.sx1250.AS923.USB
global_conf.json.sx1250.CN490
... (all variants)
```

---

## Upstream path: LoRaWAN → Network Server

When a packet is classified as LoRaWAN, `thread_up()` builds a standard
**Semtech UDP PUSH_DATA** datagram (`rxpk` JSON array) and sends it to the
configured Network Server via `sock_up`.

Fields serialized per packet:

| JSON key   | Source field              | Notes                             |
|------------|---------------------------|-----------------------------------|
| `tmst`     | `p->count_us`             | Concentrator internal timestamp   |
| `time`     | GPS UTC (if locked)       | ISO 8601                          |
| `tmms`     | GPS milliseconds          | GPS epoch (if locked)             |
| `ftime`    | `p->ftime`                | Fine timestamp (if available)     |
| `chan`      | `p->if_chain`             | IF channel index                  |
| `rfch`     | `p->rf_chain`             | RF chain                          |
| `freq`     | `p->freq_hz / 1e6`        | MHz                               |
| `stat`     | `p->status`               | CRC status                        |
| `modu`     | `p->modulation`           | `"LORA"` or `"FSK"`              |
| `datr`     | `p->datarate`             | e.g. `"SF7BW125"`                |
| `codr`     | `p->coderate`             | e.g. `"4/5"`                     |
| `rssis`    | `p->rssis`                | Signal RSSI (dBm)                 |
| `rssi`     | `p->rssic`                | Channel RSSI (dBm)                |
| `lsnr`     | `p->snr`                  | LoRa SNR (dB)                    |
| `foff`     | `p->freq_offset`          | Frequency offset (Hz)             |
| `size`     | `p->size`                 | Payload size (bytes)              |
| `data`     | `p->payload`              | Base64-encoded payload            |

After sending, `thread_up()` waits for a **PUSH_ACK** (two attempts, each
bounded by `push_timeout_ms / 2` from config).

---

## Downstream path: Network Server → concentrator

The downstream path (`thread_down()`) remains **unchanged**. It connects to the
NS via `sock_down` and handles `PULL_RESP` / `TX_ACK` as usual. LoRaWAN
downlinks from the NS are scheduled through the JIT queue and transmitted via
`lgw_send()`.

CDP downlinks from the MQTT broker continue to be injected via
`cdp_bridge_pop_downlink()` → JIT queue on the separate `thread_duck()`.

---

## Deployment examples

### Normal operations (sensors only, no disaster)

```json
"lorawan_forward_enabled": true,
"server_address": "chirpstack.internal",
"serv_port_up": 1700
```

All LoRaWAN sensors forward to Chirpstack normally. CDP mesh is active but
idle (no MamaDucks transmitting).

### Disaster mode only

```json
"lorawan_forward_enabled": false
```

All radio bandwidth available to CDP mesh. No NS forwarding overhead.

### Full dual-mode (sensors + disaster response)

```json
"lorawan_forward_enabled": true,
"server_address": "chirpstack.internal",
"serv_port_up": 1700
```

Both LoRaWAN sensors and CDP MamaDucks share the same concentrator channels.
Classification is done per-packet with no additional radio overhead.

---

## Comparison: single process vs two processes

| | Two processes | Dual-mode (this feature) |
|---|---|---|
| Hardware | Two SX1302 units required | One SX1302 |
| Processes | `lora_pkt_fwd` + `clusterduckd` | `clusterduckd` only |
| Channel sharing | Conflict — both try `lgw_start()` | Single `lgw_receive()` call |
| Configuration | Two config files | One `global_conf.json` |
| Switchover | Manual restart | Runtime flag, no restart needed |

---

## Known limitations

- **IQ polarity**: LoRaWAN downlinks use inverted IQ (`invert_pol = true`).
  CDP downlinks do not (`invert_pol = false`, set at line ~3006 in
  `clusterduckd.c`). The concentrator handles this per-packet in TX — no
  conflict on RX.
- **Channel plan**: Both LoRaWAN devices and CDP MamaDucks must be configured
  to use frequencies within the concentrator's active channel plan
  (`SX130x_conf`). They share the same 8-channel LoRa modem.
- **Classifier false positives**: A CDP packet whose SDUID byte 0 happens to
  satisfy the MHDR check (RFU=0, Major=0) will be misclassified as LoRaWAN
  and forwarded to the NS (where it will fail MIC verification and be silently
  dropped). This is expected and harmless.

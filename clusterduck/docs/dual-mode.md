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
    "gateway_id": "AC1F09FFFE0XXXXX",
    "server_address": "chirpstack.example.com",
    "serv_port_up":   1700,
    "serv_port_down": 1700,

    "lorawan_forward_enabled": true
}
```

### `gateway_id`

The **EUI-64** identifier for this gateway. This is what the Network Server
(Chirpstack, TTN) uses to identify the gateway and route downlinks back to it.

**How to find your EUI:**  
Run `clusterduckd` and look for this line in the startup log:
```
INFO: concentrator EUI: 0xAC1F09FFFE0XXXXX
```
Use those 16 hex characters as your `gateway_id` value. Register the same EUI
in Chirpstack → Gateways → Add Gateway, or in TTN Console → Gateways → Register.

**If `gateway_id` is omitted from config:**  
`clusterduckd` automatically detects the hardware EUI from the SX1302 concentrator
via `lgw_get_eui()` and uses it. The log will confirm:
```
INFO: gateway_id not set in config, using concentrator EUI: AC1F09FFFE0XXXXX
```
This means `gateway_id` is **optional** — omitting it is safe and recommended
if you only have one gateway, since the hardware EUI is always unique.

**The placeholder `AA555A0000000000`** is Semtech's default and must be replaced
(or removed) before connecting to a real Network Server, otherwise the NS will
reject PUSH_DATA from a gateway it does not recognise.

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

`thread_down()` handles **all communication from the Network Server back to the
concentrator**. It runs only when `lorawan_forward_enabled` is `true`
(otherwise `thread_down` and `thread_jit` are not spawned at all).

### PULL_DATA keepalive

The Semtech UDP protocol requires the gateway to send periodic `PULL_DATA`
datagrams to the NS. The NS uses these to:

1. Maintain NAT hole-punching so it can reach the gateway through firewalls
2. Track gateway liveness — if no `PULL_DATA` arrives within ~60 s the NS marks
   the gateway **Disconnected**

`thread_down()` sends `PULL_DATA` once per `keepalive_time` seconds (configurable
via `keepalive_interval` in config, default **5 seconds**).

```
thread_down loop:
  ┌─ every keepalive_time seconds ───────────────────────────── ┐
  │  send PULL_DATA(token_h, token_l) → sock_down               │
  │                                                               │
  │  inner receive loop (duration = keepalive_time):             │
  │    recv(sock_down):                                           │
  │      PKT_PULL_ACK  → record ack, reset autoquit counter      │
  │      PKT_PULL_RESP → parse JSON, build lgw_pkt_tx_s          │
  │                      → jit_enqueue() → send_tx_ack() to NS   │
  │      timeout/error → continue (non-fatal)                    │
  └──────────────────────────────────────────────────────────────┘
```

Log lines to look for:
```
INFO: [down] PULL_ACK received in X ms
INFO: [down] PULL_RESP received - token[N:N]
```

### PULL_RESP → downlink TX

When the NS wants to send a downlink (Class A/B/C), it returns a `PULL_RESP`
datagram containing a `txpk` JSON object. `thread_down()` parses the full
Semtech UDP protocol fields:

| JSON key | Meaning |
|----------|---------|
| `imme`   | If `true`, transmit immediately (Class C) |
| `tmst`   | Concentrator timestamp to send at (Class A) |
| `freq`   | TX frequency in MHz |
| `rfch`   | RF chain (0 or 1) |
| `powe`   | TX power in dBm (minus `antenna_gain`) |
| `modu`   | `"LORA"` or `"FSK"` |
| `datr`   | Data rate string e.g. `"SF7BW125"` |
| `codr`   | Coding rate e.g. `"4/5"` |
| `ipol`   | Invert IQ polarity (true for LoRaWAN downlinks) |
| `prea`   | Preamble length (optional, min `MIN_LORA_PREAMB`) |
| `size`   | Payload size in bytes |
| `data`   | Base64-encoded payload |

The packet is then passed to `jit_enqueue()` for collision-aware scheduling,
and a `PKT_TX_ACK` datagram is sent back to the NS via `send_tx_ack()` to
confirm the scheduling result.

### TX acknowledgement (`send_tx_ack`)

After every `PULL_RESP`, `clusterduckd` sends a `PKT_TX_ACK` datagram back to
the NS on `sock_down`. The `error` field reflects the JIT queue result:

| Error | Meaning |
|-------|---------|
| `NONE` | Packet accepted and scheduled |
| `COLLISION_PACKET` | JIT queue collision with another downlink |
| `TOO_LATE` | TX timestamp already passed |
| `TOO_EARLY` | TX timestamp too far in future |
| `TX_FREQ` | Frequency out of allowed TX range |
| `TX_POWER` | Requested RF power not in TX LUT (nearest used instead) |
| `GPS_UNLOCKED` | GPS-based timestamp requested but GPS not locked |

### CDP downlinks

CDP downlinks arrive via `cdp_bridge_pop_downlink()` and are processed in
both the outer loop and the inner recv loop of `thread_down()`. This gives
CDP downlinks fast response (< 10 ms latency in the idle case) without
requiring a separate thread.

CDP downlinks always use:
- `tx_mode = IMMEDIATE`
- `invert_pol = false` (CDP is not LoRaWAN — MamaDucks expect normal IQ)
- `preamble = 8`
- 500 ms delay before TX to allow MamaDuck radio turnaround after its RREQ TX

---

## Gateway status heartbeat

The NS displays gateway statistics (RX count, TX count, uptime) from periodic
`PUSH_DATA` datagrams containing a `stat` JSON object.

`thread_up()` sends the stat object:

- **When packets were received**: the `stat` object is appended inside the
  `rxpk` datagram at the end of each `stat_interval` cycle
- **When no packets arrived**: a stat-only `PUSH_DATA` is sent with an empty
  `rxpk` array:
  ```json
  {"rxpk":[], "stat":{"time":"...","rxnb":0,"rxok":0,...}}
  ```

This ensures the NS always receives regular heartbeats regardless of RF
activity, so the gateway stays **Connected** in the console.

The stat interval is controlled by `stat_interval` in `gateway_conf`
(default: **30 seconds**):

```json
"stat_interval": 30
```

If the NS shows **"no status"**, wait 30–60 s after connect. If it persists,
check the log for `PUSH_ACK received` — absence means UDP replies from the NS
are not reaching the gateway (firewall/NAT issue on port 1700).

---

## Troubleshooting

### Gateway shows "Disconnected" on NS console

Cause: `PULL_DATA` keepalives not reaching the NS.

Check log:
```
INFO: [down] PULL_ACK received in X ms
```
If absent:
- Confirm `lorawan_forward_enabled` is `true` in config (check startup log)
- Confirm `sock_down` connected — look for:
  ```
  INFO: [main] downstream socket open to LoRaWAN NS host:port
  ```
- Check firewall allows outbound UDP to NS port 1700

### Gateway shows "no status" on NS console

Cause: `PUSH_DATA` (containing `stat` object) not being acknowledged.

Check log:
```
INFO: [up] PUSH_ACK received in X ms
```
or:
```
INFO: [up] stat-only PUSH_DATA sent to NS
```
If absent, wait one more `stat_interval` (30 s). If still absent, check
that `sock_up` is open and UDP replies on port 1700 are not blocked.

### Gateway EUI mismatch

The gateway EUI reported in the startup log must exactly match what is
registered on the NS:
```
INFO: concentrator EUI: 0x0016C001F1000FB7
```
Or if `gateway_id` is set in config:
```
INFO: gateway_id configured to 0016C001F1000FB7
```
Register this 16-character hex EUI on the NS. If the EUI does not match,
the NS accepts the UDP datagrams but marks the gateway as unauthorised and
does not deliver downlinks.

### `thread_down` / `thread_jit` not running

These threads are only spawned when `lorawan_forward_enabled` is `true`.
If CDP downlinks from MQTT are not being transmitted, confirm the flag is
enabled — CDP downlinks are also handled in `thread_down`.

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

# LoRaWAN Setup & Troubleshooting

## Gateway Hardware Info

| Field | Value |
|-------|-------|
| Concentrator | SX1302 via USB (`/dev/ttyACM0`) |
| Chip version | `0x10` (v1.0) |
| **Gateway EUI** | **`0016C001F1000FB7`** |
| Frequency plan | AS923 |

The EUI is read from the concentrator via:
```bash
./util_chip_id/chip_id -u -d /dev/ttyACM0 2>&1 | grep -i eui
```

It is printed at `clusterduckd` startup:
```
INFO: concentrator EUI: 0x0016c001f1000fb7
```

This EUI **must match** the Gateway EUI registered in The Things Stack (TTS). If `gateway_id` is not set in `global_conf.json`, the hardware EUI is used automatically.

---

## Frequency Configuration (AS923)

`global_conf.json` radio center frequencies:

| Radio | Center Frequency | Notes |
|-------|-----------------|-------|
| radio_0 | 923.0 MHz | Anchors CH0/1/4/5/6 and std-SF channel |
| radio_1 | 922.1 MHz | Anchors CH2/3/7 |

Resulting 8 multi-SF channels:

| CH | Frequency |
|----|-----------|
| 0  | 923.2 MHz |
| 1  | 923.4 MHz |
| 2  | 922.2 MHz |
| 3  | 922.4 MHz |
| 4  | 922.6 MHz |
| 5  | 922.8 MHz |
| 6  | 923.0 MHz |
| 7  | 922.0 MHz |

> **Common mistake:** Using radio_1 = 923.3 MHz (copied from wrong template) causes CH2/3/7 to land on wrong frequencies, causing ~half of AS923 uplinks to be missed.

`lorawan_public: true` must be set to use sync word `0x34`. Devices using the private sync word (`0x12`) will be hardware-filtered by the concentrator.

---

## Dual-Protocol (CDP + LoRaWAN) Mode

### Sync Word

`DuckLoRa.cpp` → `lora.setSyncWord(0x34)` — must match `lorawan_public: true` in config so CDP and LoRaWAN devices share the same sync word.

### Packet Classifier (`is_lorawan_packet()`)

Located in `clusterduckd.c`. Logic: **positively identify CDP first**, treat everything else as LoRaWAN.

A packet is CDP if bytes 21–22 satisfy:
- `duck_type` (high nibble of byte 21) ≤ 4
- `hop_count` (low nibble of byte 21) ≤ 6
- payload size ≥ 28

Everything else is routed to the LoRaWAN network server.

### LoRaWAN Forwarding

Configured in `global_conf.json`:
```json
"lorawan_forward_enabled": true,
"server_address": "thethingstack.service.consul",
"serv_port_up": 1700,
"serv_port_down": 1700
```

TX is enabled on RF chain 0 only (`radio_0`, `tx_enable: true`).

---

## Join Troubleshooting

If uplinks are visible in logs but the end device cannot join:

### 1. Verify gateway EUI in TTS
The EUI in TTS must match `0016C001F1000FB7`. If it doesn't, TTS receives uplinks but won't open a downlink socket — no `PULL_RESP` will ever arrive.

### 2. Check downlink socket is open
On the server, look for periodic PULL_ACK in the logs:
```
DEBUG: [down] PULL_ACK received in X ms
```
If absent, `thread_down` is not maintaining the keepalive to TTS — check network/DNS for `thethingstack.service.consul:1700`.

### 3. Check TTS Console → Gateway → Live Data
- Does TTS show the gateway as **Connected**?
- Does the join request appear in TTS Live Data?
- Does TTS show a **join accept** being scheduled?

If the join request appears in TTS but no join accept is sent, the device is not registered in TTS (wrong DevEUI, AppEUI, or AppKey).

### 4. Check for PULL_RESP in clusterduckd logs
```
INFO: [down] PULL_RESP received - token[X:X]
```
Followed by:
```
lgw_send done on rf_chain 0: count_us=XXXXXXXX
```
If `PULL_RESP` arrives but `lgw_send` does not follow, check for:
- `ERROR: [down] PULL_RESP REJECTED: bad TX freq` — downlink frequency outside `tx_freq_min`/`tx_freq_max`
- `ERROR: [down] PULL_RESP enqueue failed (jit=N)` — JIT queue timing issue

### TX frequency range (AS923 downlinks)

AS923 RX1 replies on the same channel as the uplink; RX2 is 923.2 MHz / SF10.
`radio_0.tx_freq_min: 920000000`, `radio_0.tx_freq_max: 923400000` — covers all AS923 downlink frequencies.

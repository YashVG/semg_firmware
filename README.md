# sEMG BLE Firmware

Zephyr RTOS firmware for the nRF5340 DK that samples 4-channel sEMG data at
2000 Hz and streams it over BLE as 240-byte notification packets.

Currently uses **fake (incrementing) sample data** in place of a real ADC
driver. The data path, timing, packet format, and BLE plumbing are all
production-shaped so that swapping in real ADC reads later is a one-function
change.

## How it works (end to end)

```
boot
 |
 v
main()
 |- bt_enable()                        initialise the BLE stack
 |- register auth + connection cbs     track connect / disconnect
 |- k_work_init(&semg_send_work)       prep the BLE-send work item
 |- k_timer_start(500 us period)       start the 2000 Hz sample timer
 |- bt_le_adv_start()                  begin advertising as "sEMG Sensor"
 |
 v
 while (1)  ── sleeps 1 s, handles reconnect + battery sim ──
```

Once a BLE central connects and enables notifications on the sEMG
characteristic, the real-time data path kicks in:

```
k_timer ISR  (every 500 us)
 |
 |  write 4 x int16 fake sample into sample_bufs[active_buf]
 |  increment sample_idx
 |
 |  sample_idx == 29?
 |    yes ──> swap active/ready buffers
 |            k_work_submit(semg_send_work)
 |
 v
system workqueue thread  (semg_send_handler)
 |
 |  pack 240-byte packet (header + 29 samples)
 |  bt_gatt_notify() ──> over-the-air to central
 |  print "sEMG pkt #N  dt=XX ms (expect ~14)"
```

### Why two execution contexts?

| Context | What runs there | Why |
|---|---|---|
| **Timer ISR** | `sample_timer_handler` | Deterministic 500 us timing; must not block |
| **System workqueue** | `semg_send_handler` | `bt_gatt_notify` can block briefly; needs thread context |

The timer ISR only does array writes and a `k_work_submit` (both ISR-safe).
The actual BLE call happens later in thread context on the system workqueue.

## Packet format (240 bytes)

Every packet carries exactly 29 samples from 4 channels:

```
Offset  Size   Type     Field
------  -----  -------  ---------------------------
0       2      uint16   sequence number (LE)
2       4      uint32   timestamp in ms since boot (LE)
6       1      uint8    sample count (always 29)
7       1      uint8    channel mask  (0x0F = ch 0-3)
8       232    int16[]  sample data: 29 samples x 4 channels (LE)
------  -----
Total:  240 bytes
```

Samples are laid out as `[s0_ch0, s0_ch1, s0_ch2, s0_ch3, s1_ch0, ...]`
in little-endian int16.

At 2000 samples/sec with 29 samples per packet, a new packet is sent
roughly every **14.5 ms (~69 packets/sec)**.

## BLE service definition

| Item | UUID |
|---|---|
| sEMG Service | `00001800-b5a3-f393-e0a9-e50e24dcca9e` |
| sEMG Data Characteristic | `00001801-b5a3-f393-e0a9-e50e24dcca9e` |

These are **custom 128-bit UUIDs** (not the standard Bluetooth SIG 0x1800
Generic Access). The characteristic supports **notify only** (no read/write).

A client enables notifications by writing `0x0001` to the characteristic's
**CCCD** (Client Characteristic Configuration Descriptor). The firmware logs
`sEMG notifications enabled` when this happens.

Two standard 16-bit services are also present (inherited from the original
Zephyr sample):

- **BAS** (Battery Service, 0x180F) -- fake battery level, decrements once/sec
- **DIS** (Device Information Service, 0x180A)

## Double-buffer and overrun protection

Two sample buffers (`sample_bufs[0]` and `sample_bufs[1]`) are used in a
ping-pong scheme:

1. The timer ISR fills `sample_bufs[active_buf]`.
2. When 29 samples are collected, the ISR swaps `active_buf` and `ready_buf`,
   then submits a work item.
3. The work handler reads from `sample_bufs[ready_buf]` and sends via BLE.
4. While the work handler sends, the ISR is already filling the other buffer.

An `atomic_t send_pending` flag prevents a race condition: if the work handler
hasn't finished by the time the next 29 samples are ready, the ISR **drops
that batch** rather than corrupting the buffer the work handler is reading.
This is the safe trade-off -- you lose one packet (~14.5 ms of data) instead
of sending garbled data.

## File overview

```
semg_firmware/
  CMakeLists.txt          Zephyr build file; pulls in all src/*.c
  prj.conf                Kconfig: BLE, MTU sizes, workqueue stack
  src/
    main.c                everything (service, timer, BLE, LED blink)
```

### prj.conf key settings

| Setting | Value | Why |
|---|---|---|
| `CONFIG_BT_L2CAP_TX_MTU` | 247 | ATT MTU must be >= 243 (240 payload + 3 header) |
| `CONFIG_BT_BUF_ACL_TX_SIZE` | 251 | L2CAP frame = MTU + 4 byte L2CAP header |
| `CONFIG_BT_BUF_ACL_RX_SIZE` | 251 | Symmetric with TX for MTU exchange |
| `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE` | 2048 | `bt_gatt_notify` call chain needs headroom |
| `CONFIG_BT_SMP` | y | Security Manager Protocol (pairing support) |

The central **must** request an ATT MTU of at least 243 during the MTU
exchange, or the 240-byte notification will fail. Most modern BLE centrals
(phones, nRF Connect, etc.) request 247+ automatically.

## Building and flashing

Requires the nRF Connect SDK (NCS) with Zephyr. Assuming the SDK is
installed and `ZEPHYR_BASE` is set:

```bash
west build -b nrf5340dk/nrf5340/cpuapp
west flash
```

Open a serial terminal (115200 baud) to see log output:

```
Bluetooth initialized
Starting Legacy Advertising (connectable and scannable)
Advertising successfully started
```

## Testing with nRF Connect (phone app)

1. Open nRF Connect, scan for **"sEMG Sensor"**, and connect.
2. The app will auto-negotiate MTU (verify it's >= 243 in the connection info).
3. Find the **Unknown Service** with UUID `00001800-b5a3-f393-...`.
4. Tap the triple-down-arrow on the characteristic to enable notifications.
5. You should see 240-byte payloads arriving every ~14-15 ms.
6. On the serial console you'll see timing logs:
   ```
   sEMG pkt #1  dt=15 ms (expect ~14)
   sEMG pkt #2  dt=14 ms (expect ~14)
   ```

## Where to plug in real ADC data

Replace the body of `sample_timer_handler()` (line 158). Currently it does:

```c
for (int ch = 0; ch < SEMG_CHANNEL_COUNT; ch++) {
    dst[ch] = fake_sample++;    // <-- replace this
}
```

Swap `fake_sample++` with your actual ADC read for each channel.
Everything downstream (buffering, packing, BLE send) stays the same.

**Important:** this function runs in ISR context. Your ADC read must be
non-blocking. If your ADC driver requires a blocking call, move the read
into a dedicated thread or DMA callback and write to `sample_bufs` from
there, keeping the same double-buffer protocol.

## Sequence number and dropped-packet detection

`semg_seq` is a uint16 that increments once per packet. On the receiving
end, check for gaps: if you see seq jump from N to N+2, one packet was
either dropped by the overrun guard or lost over the air. The `dt=` log on
the serial console helps you distinguish the two -- a large `dt` means the
workqueue was backed up; normal `dt` with a seq gap means RF loss.

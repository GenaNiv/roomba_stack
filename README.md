# roomba_stack
Layered software stack for controlling iRobot Roomba (600 series OI).  
Dev on Ubuntu laptop, deploy to Raspberry Pi 5.

SW architecture breakdown:
[External User]
     ↑ WiFi / Web / BLE / CLI
     │
 [L5 UI Layer]  ←  FastAPI, WebSocket, CLI, etc.
     │
 [L4 App Layer]  ← event bus, command bus, scheduler
     │
 [L3 Domain]    ← behaviors, safety, state machine
     │
 [L2 OI]        ← opcodes, sensor parsing, OI service
     │
 [L1 Serial]    ← (CURRENT MODULE) → abstract serial port
     │
 [Roomba OI Port] ← physical UART / USB-TTL only

                                    ┌──────────────────────────────────────────────────────────────────────┐
                                   │                           HOST (Ubuntu)                              │
                                   │                                                                      │
        Roomba OI UART             │  USB Bridge (CP210x/FTDI/CH340)     Linux USB/TTY driver + buffers   │
    TX──▶ 115200 8N1 ─────────────▶│────────────── USB packets ─────────▶ /dev/ttyUSBx (/dev/ttyACMx)      │
                                   │                                                                      │
                                   └──────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                             l1_drivers.PySerialPort (thread-safe facade; Story 2)                           │
│                                                                                                                              │
│   [Reader Thread]                                      [Writer Thread - future]                                              │
│   ───────────────                                      ─────────────────────────                                              │
│   - reads chunks via pyserial.read()                   - drains TxFrameQueue (timed get)                                     │
│   - invokes set_reader(cb)(data)                       - pyserial.write(frame)                                               │
│                                                                                                                              │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
                                     │
                                     │ data: bytes (arbitrary chunking; may contain partial/whole/multiple frames)
                                     ▼

┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                             app.OIService  (process boundary)                                                │
│                                                                                                                              │
│  (A) ENQUEUE-ONLY RX CALLBACK                                                                                                 │
│  ──────────────────────────                                                                                                   │
│  _on_serial_bytes(data):                                                                                                      │
│    if not data: return                                                                                                        │
│    ok = RxByteQueue.put(data, timeout=Q_PUT_TIMEOUT)                                                                          │
│    if not ok: WARN "overflow: dropped {len(data)}"                                                                            │
│                                                                                                                              │
│  (B) BOUNDED QUEUES (Monitor semantics)                                                                                       │
│  ─────────────────────────────────────────────────────────────────────────────                                                │
│   RxByteQueue (max=RX_QUEUE_MAX)        TxFrameQueue (max=TX_QUEUE_MAX)  …future                                              │
│     • timed put/get (no infinite waits)  • timed put/get (no infinite waits)                                                  │
│     • overflow policy: drop-newest       • overflow policy: drop-newest                                                       │
│                                                                                                                              │
│  (C) DISPATCHER THREAD (Half-Sync side)                                                                                       │
│  ─────────────────────────────────────────────────────────────────────────────                                                │
│  _dispatcher_loop():                                                                                                          │
│    while _running:                                                                                                            │
│      ok, chunk = RxByteQueue.get(timeout=Q_GET_TIMEOUT)  # timed; no busy spin                                                │
│      if not ok: continue                                                                                                      │
│      _rx_buf.extend(chunk)                    # reassembly buffer                                                             │
│      _decode_available_frames()               # decode 0..N frames; consume bytes exactly                                      │
│                                                                                                                              │
│  (D) DECODER / DEMUX (single event dispatcher)                                                                               │
│  ─────────────────────────────────────────────────────────────────────────────                                                │
│  _decode_available_frames():                                                                                                │
│    while True:                                                                                                               │
│      if not _rx_buf: break                                                                                                    │
│      lead = _rx_buf[0]                                                                                                        │
│                                                                                                                              │
│      Case A: STREAM FRAME (opcode 148; header 0x13)                                                                           │
│        - need ≥ 3 bytes to read len N                                                                                         │
│        - sanity-cap N (e.g., ≤128)                                                                                            │
│        - total = 2 + N + 1 (hdr,len,chk)                                                                                      │
│        - if buffer has < total: break (await more)                                                                            │
│        - verify checksum (sum==0 mod 256); if bad → WARN + drop 1 byte (resync)                                               │
│        - decode payload → {pid→parsed}                                                                                         │
│        - for each (pid, parsed): _deliver(pid, parsed)                                                                        │
│        - del _rx_buf[:total] and continue                                                                                     │
│                                                                                                                              │
│      Case B: PENDING SINGLE REPLY (opcode 142; raw payload only)                                                              │
│        - _pending_request_id = pid                                                                                            │
│        - expected_len = packet_length(pid)                                                                                    │
│        - if buffer has < expected_len: break                                                                                  │
│        - raw = _rx_buf[:expected_len]; parse → parsed                                                                         │
│        - _deliver(pid, parsed); clear _pending_request_id                                                                     │
│        - del _rx_buf[:expected_len]; continue                                                                                 │
│                                                                                                                              │
│      Case C: UNKNOWN / GARBAGE                                                                                                │
│        - WARN "RX resync: dropping 1 byte (buf=..)"                                                                           │
│        - del _rx_buf[0]; continue                                                                                             │
│                                                                                                                              │
│  (E) DELIVERY (single place, deterministic)                                                                                   │
│  ─────────────────────────────────────────────────────────────────────────────                                                │
│  _deliver(pid, parsed):                                                                                                       │
│    latest_packets[pid] = parsed                                                                                               │
│    if _on_sensor:                                                                                                             │
│       try: _on_sensor(pid, parsed)                                                                                            │
│       except: log.exception("on_sensor callback error")                                                                       │
│                                                                                                                              │
│  (F) SHUTDOWN                                                                                                                 │
│  ─────────────────────────────────────────────────────────────────────────────                                                │
│  close():                                                                                                                     │
│    _running=False; port.set_reader(None)                                                                                      │
│    join dispatcher (timeout) if alive & not self                                                                              │
│    port.close()                                                                                                               │
│                                                                                                                              │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
                                     │
                                     │ events: (pid, value)   +   cache: latest_packets[pid]
                                     ▼

┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                        Upper Consumers (current & near-future)                                               │
│                                                                                                                              │
│  • CLI / UI (today)                                                                                                          │
│     - subscribes via _on_sensor callback; prints packet 7, 25, etc.                                                          │
│                                                                                                                              │
│  • App logic (near-future)                                                                                                   │
│     - Pub/Sub bus: topics sensor.<pid>, mode.changed, rx.raw, tx.sent                                                         │
│     - OI mode State Machine (OFF/PASSIVE/SAFE/FULL) gates allowed commands                                                    │
│     - Safety/Watchdog: timeouts, RX stall monitor, overflow counters                                                          │
│     - Structured logging/telemetry                                                                                            │
│                                                                                                                              │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

…TX PATH (next iteration; shown for completeness)
───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
Commands (Start/Safe/Drive/Sensors/StreamOn/Off)
    → validate against State Machine
    → encode via codec
    → enqueue TxFrameQueue (timed put; bounded)
    → writer thread drains and pyserial.write(frame)

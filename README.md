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
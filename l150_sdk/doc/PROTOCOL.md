# L150 serial protocol summary

- Port: 115200 baud, 8 data bits, no parity, 1 stop bit.
- Command frame: 11 bytes, `0x7B` header, BCC, `0x7D` tail.
- Feedback frame: 24 bytes, `0x7B` header, BCC, `0x7D` tail.
- Multi-byte signed values use big-endian `int16_t` encoding.
- Linear and angular command values are multiplied by 1000.
- The real chassis uses clockwise-positive yaw. The SDK negates command and
  feedback yaw so ROS messages follow REP-103 (counter-clockwise positive).

See `src/l150_serial_node.cpp` for the authoritative parser and encoder.


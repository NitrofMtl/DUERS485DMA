- 0.9.8 : 2026/02/24
### Changed
- Restored updateBuffer() call in polling paths (available() / readBytes()) to ensure proper RX synchronization.
- Improved IRQ handling to provide more consistent timing behavior.
- Default postDelay set to 0 (transmission completion already guaranteed by flush()).

### Added
- Added optional readFrame() helper function for frame-based access (not used by default Stream/Modbus flow).

### Fixed
- Resolved RX timing issue that could cause premature frame detection and Modbus timeouts.
- Improved buffer handling consistency between interrupt and polling modes.


- 0.9.7 : 
### Added
- Hardware RX silence detection using USART Receiver Timeout (RTOR).
- Software RX frame idle detection based on elapsed time since last RX activity.
- `isRxIdle()` API to query protocol-level RX completion.
- RX stall guard timeout to prevent blocking indefinitely while waiting for RX to complete.
- Helper functions to compute character and frame timing from USART configuration.

### Changed
- RX DMA buffer is now flushed to the Arduino ring buffer on RX timeout events instead of relying on polling.
- RX idle detection and RX stall protection are now clearly separated concepts.
- TX no longer waits internally for RX to become idle; RX/TX sequencing is now handled explicitly via `noReceive()` on RX side and `endTransmission()` on TX side.
- `setIdleTime()` now takes microseconds instead of character counts.

### Fixed
- DMA RX rollover issue causing duplicated or missing bytes.
- Race condition between DMA RX progress and `Serial::available()` / `read()`.
- Inconsistent RX behavior under continuous or bursty traffic.

### Removed
- RX wait guard logic from transmission start path to avoid duplicated timeout policies.

- 0.9.6 : fix: correct ring buffer + DMA rearm handling

- 0.9.5 : docs(readme): clarify Arduino IDE (DUE) support and recommend PlatformIO

- 0.9.4 : put examples into named folders

- 0.9.3 : Fix miss spelling of example folder

- 0.9.2 : update modbus lib compatible link

- 0.9.1 :  Removed USE_DMA requirement, simplified usage

- 0.9.0 : initial commit.

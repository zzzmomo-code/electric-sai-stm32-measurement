# Hardware Baseline

This file separates firmware facts from measurements that still require the
physical board. Do not fill a hardware row from software assumptions alone.

## Configuration

| Item | Observed value | Evidence |
|---|---|---|
| MCU top marking | Pending hardware verification | Board photo or package marking |
| Debug probe | Pending hardware verification | ST-Link or J-Link connection log |
| Firmware target | STM32F407ZGTx | `1.ioc`, startup file, and linker script |
| AD7606 range pin | Pending hardware verification | Schematic or measured pin level |
| AD7606 oversampling | OS x64 | Firmware GPIO configuration |
| UART frame | 402 bytes: `AA 55` + 400-byte payload | Compile-time assertions in `main.c` |

If the physical MCU is STM32F407ZE rather than STM32F407ZG, create a separate
CubeMX/linker migration plan before changing target files.

## Logic Analyzer

Status: **待硬件验证**

Capture CONVST A, BUSY, CS, and SCLK together.

| Metric | Acceptance | Observed | Evidence |
|---|---|---|---|
| CONVST trigger frequency | About 1281.738 Hz | Pending | Logic-analyzer capture |
| BUSY high time at OS x64 | At or below 315 us nominal maximum | Pending | Logic-analyzer capture |
| BUSY sequence | High observed before accepted falling edge | Pending | Logic-analyzer capture |
| SCLK clocks per conversion | 128 clocks on DOUTA | Pending | Logic-analyzer capture |
| Readout context | Readout starts after EXTI returns | Pending | Trace or GPIO instrumentation |

## UART Continuity

Status: **待硬件验证**

Run for 10 minutes at USART3 115200 8N1. Validate every received frame as 402
bytes beginning with `AA 55`, and compare expected and received frame counts.
Protocol v1 has no sequence number or CRC, so this cannot prove that every
payload byte is uncorrupted. After a DMA error, the receiver must resynchronize
on the next `AA 55` header because a partial frame may remain on the wire.

| Counter | Start | End | Acceptance |
|---|---:|---:|---|
| `conversion_timeout` | Pending | Pending | No increase |
| `spurious_busy_edge` | Pending | Pending | No increase |
| `sample_read_failed` | Pending | Pending | No increase |
| `frame_dropped` | Pending | Pending | No increase |
| `uart_start_failed` | Pending | Pending | No increase |
| `uart_dma_error` | Pending | Pending | No increase |

Expected frame rate from the current timer and decimation settings is about
25.635 frames/s. Record the measured rate and instrument tolerance here.

## Channel Accuracy

Status: **待硬件验证**

Confirm the RANGE pin electrically or from the schematic before converting
codes to volts. Apply a stable reference and collect at least 10,000 samples per
channel. Keep reference values and measured results in separate columns.

`voltage = signed_code * (2 * range_volts) / 65536`

`mean_error = measured_mean - reference_voltage`

| Channel | Reference V | Mean V | Std dev V | Min V | Max V | Mean error V |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | Pending | Pending | Pending | Pending | Pending | Pending |
| 2 | Pending | Pending | Pending | Pending | Pending | Pending |
| 3 | Pending | Pending | Pending | Pending | Pending | Pending |
| 4 | Pending | Pending | Pending | Pending | Pending | Pending |
| 5 | Pending | Pending | Pending | Pending | Pending | Pending |
| 6 | Pending | Pending | Pending | Pending | Pending | Pending |
| 7 | Pending | Pending | Pending | Pending | Pending | Pending |
| 8 | Pending | Pending | Pending | Pending | Pending | Pending |

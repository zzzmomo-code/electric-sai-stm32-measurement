# AD7606 Reliability and Observability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make AD7606 conversion completion, timeout recovery, sampling statistics, and UART-DMA failure handling deterministic without changing confirmed pins, CubeMX peripheral configuration, or the existing 402-byte binary frame.

**Architecture:** Add a small HAL-independent AD7606 BUSY state machine that can be tested on the host, then let the existing GPIO driver own that state and keep EXTI callbacks limited to event marking. The main loop remains responsible for bit-banged reads, recovery, buffering, and DMA submission. Existing `AA 55 + 400-byte payload` output remains byte-for-byte compatible.

**Tech Stack:** STM32F407ZGTx, STM32 HAL F4, C11/GNU11, STM32CubeIDE GCC 14.3, PowerShell, MinGW host GCC, Git.

---

## Review Baseline

- Current branch: `main`; reviewed baseline commit: `5999565 fix: stabilize ad7606 sampling pipeline`.
- Full rebuild command `make -C Debug -B all` passes. Image size is 37,588 bytes text, 476 bytes data, and 3,404 bytes BSS.
- Remaining build messages are CubeIDE-generated `*.cyclo/*.su peer target` warnings, not C source warnings.
- TIM6 runs from an 84 MHz timer clock with `PSC=0`, `ARR=65535`: trigger rate is about 1281.738 Hz.
- Every second conversion is stored: effective stored rate is about 640.869 sample sets/s.
- One frame carries 25 sets x 8 channels x 2 bytes plus `AA 55`: 402 bytes, about 25.635 frames/s.
- USART3 at 115200 8N1 needs about 34.896 ms per frame; frame period is about 39.010 ms. Wire utilization is about 89.45%, leaving only about 4.11 ms margin.
- AD7606 is configured for OS x64. Datasheet conversion time is 257-315 us; the current 780 us trigger period is sufficient, but BUSY fault handling is incomplete.
- Hardware pins are user-confirmed. Do not alter GPIO mapping or `1.ioc` in this plan.
- The repository currently targets `STM32F407ZGTx` and CubeIDE, while project rules mention `STM32F407ZET6` and Keil. This is a confirmation gate, not an automatic configuration change.

## File Map

- Create `Core/USER/ad7606_state.h`: pure state-machine types and API; no HAL includes.
- Create `Core/USER/ad7606_state.c`: BUSY-high/BUSY-low sequencing and wrap-safe timeout logic.
- Create `tests/host/test_ad7606_state.c`: dependency-free host unit tests.
- Create `tests/host/Makefile`: repeatable host build and test command.
- Modify `Core/USER/ad7606.h`: typed status/API cleanup while preserving all confirmed pin macros.
- Modify `Core/USER/ad7606.c`: integrate the state machine, validate OS mode, and add recovery.
- Modify `Core/Src/main.c`: distinct counters, timeout recovery, PRIMASK-safe critical sections, UART retry backoff, and compile-time frame checks. All edits remain inside CubeMX user sections.
- Modify `Core/USER/user_usart.h` and `Core/USER/user_usart.c`: return HAL status and remove `size_t` to `uint16_t` truncation.
- Create `docs/hardware-baseline.md`: record target marking, measured timings, and long-run results without guessing.
- Do not modify `1.ioc`, generated `MX_*_Init()` bodies, linker scripts, startup files, or the UART frame layout.

### Task 1: Add a host-tested AD7606 BUSY state machine

**Files:**
- Create: `Core/USER/ad7606_state.h`
- Create: `Core/USER/ad7606_state.c`
- Create: `tests/host/test_ad7606_state.c`
- Create: `tests/host/Makefile`

- [ ] **Step 1: Write the state-machine tests first**

Create `tests/host/test_ad7606_state.c` with four explicit cases:

```c
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "ad7606_state.h"

static void test_low_without_seen_high_is_not_complete(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, 10U, false));
    AD7606_StateObserveBusy(&ctx, false, 10U, 2U);
    AD7606_StateOnFallingEdge(&ctx, false);
    assert(ctx.state == AD7606_STATE_WAIT_BUSY_HIGH);
    assert(!AD7606_StateTakeReady(&ctx));
}

static void test_high_then_falling_edge_completes(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, 20U, true));
    assert(ctx.state == AD7606_STATE_WAIT_BUSY_LOW);
    AD7606_StateOnFallingEdge(&ctx, false);
    assert(AD7606_StateTakeReady(&ctx));
    assert(ctx.state == AD7606_STATE_READING);
    AD7606_StateFinishRead(&ctx);
    assert(ctx.state == AD7606_STATE_IDLE);
}

static void test_timeout_is_latched_once(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, UINT32_MAX - 1U, true));
    AD7606_StateObserveBusy(&ctx, true, 0U, 2U);
    assert(AD7606_StateTakeTimeout(&ctx));
    assert(!AD7606_StateTakeTimeout(&ctx));
    assert(ctx.state == AD7606_STATE_FAULT);
}

static void test_idle_edge_is_spurious(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    AD7606_StateOnFallingEdge(&ctx, false);
    assert(ctx.spurious_edge_count == 1U);
    assert(!AD7606_StateTakeReady(&ctx));
}

int main(void)
{
    test_low_without_seen_high_is_not_complete();
    test_high_then_falling_edge_completes();
    test_timeout_is_latched_once();
    test_idle_edge_is_spurious();
    return 0;
}
```

- [ ] **Step 2: Add the host Makefile and confirm RED**

Create `tests/host/Makefile`:

```make
CC ?= gcc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -I../../Core/USER
TARGET := test_ad7606_state.exe

.PHONY: all test clean

all: $(TARGET)

$(TARGET): test_ad7606_state.c ../../Core/USER/ad7606_state.c ../../Core/USER/ad7606_state.h
	$(CC) $(CFLAGS) test_ad7606_state.c ../../Core/USER/ad7606_state.c -o $(TARGET)

test: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)
```

Run: `make -C tests/host test`

Expected: FAIL because `ad7606_state.h` and `ad7606_state.c` do not exist yet.

- [ ] **Step 3: Add the minimal state-machine interface**

Create `Core/USER/ad7606_state.h`:

```c
#ifndef AD7606_STATE_H
#define AD7606_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AD7606_STATE_IDLE = 0,
    AD7606_STATE_WAIT_BUSY_HIGH,
    AD7606_STATE_WAIT_BUSY_LOW,
    AD7606_STATE_DATA_READY,
    AD7606_STATE_READING,
    AD7606_STATE_FAULT
} ad7606_state_t;

typedef struct {
    ad7606_state_t state;
    uint32_t started_ms;
    uint32_t spurious_edge_count;
    bool timeout_pending;
} ad7606_state_ctx_t;

void AD7606_StateInit(ad7606_state_ctx_t *ctx);
bool AD7606_StateStart(ad7606_state_ctx_t *ctx,
                      uint32_t now_ms,
                      bool busy_is_high);
void AD7606_StateObserveBusy(ad7606_state_ctx_t *ctx,
                             bool busy_is_high,
                             uint32_t now_ms,
                             uint32_t timeout_ms);
void AD7606_StateOnFallingEdge(ad7606_state_ctx_t *ctx, bool busy_is_high);
bool AD7606_StateTakeReady(ad7606_state_ctx_t *ctx);
void AD7606_StateFinishRead(ad7606_state_ctx_t *ctx);
bool AD7606_StateTakeTimeout(ad7606_state_ctx_t *ctx);
void AD7606_StateRecover(ad7606_state_ctx_t *ctx);

#endif
```

- [ ] **Step 4: Implement wrap-safe sequencing and timeout**

Create `Core/USER/ad7606_state.c`:

```c
#include "ad7606_state.h"

#include <stddef.h>

void AD7606_StateInit(ad7606_state_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->state = AD7606_STATE_IDLE;
    ctx->started_ms = 0U;
    ctx->spurious_edge_count = 0U;
    ctx->timeout_pending = false;
}

bool AD7606_StateStart(ad7606_state_ctx_t *ctx,
                      uint32_t now_ms,
                      bool busy_is_high)
{
    if ((ctx == NULL) || (ctx->state != AD7606_STATE_IDLE)) {
        return false;
    }
    ctx->started_ms = now_ms;
    ctx->timeout_pending = false;
    ctx->state = busy_is_high ? AD7606_STATE_WAIT_BUSY_LOW
                              : AD7606_STATE_WAIT_BUSY_HIGH;
    return true;
}

void AD7606_StateObserveBusy(ad7606_state_ctx_t *ctx,
                             bool busy_is_high,
                             uint32_t now_ms,
                             uint32_t timeout_ms)
{
    if (ctx == NULL) {
        return;
    }
    if ((ctx->state == AD7606_STATE_WAIT_BUSY_HIGH) && busy_is_high) {
        ctx->state = AD7606_STATE_WAIT_BUSY_LOW;
    } else if ((ctx->state == AD7606_STATE_WAIT_BUSY_LOW) && !busy_is_high) {
        ctx->state = AD7606_STATE_DATA_READY;
    }
    if (((ctx->state == AD7606_STATE_WAIT_BUSY_HIGH) ||
         (ctx->state == AD7606_STATE_WAIT_BUSY_LOW)) &&
        ((uint32_t)(now_ms - ctx->started_ms) >= timeout_ms)) {
        ctx->state = AD7606_STATE_FAULT;
        ctx->timeout_pending = true;
    }
}

void AD7606_StateOnFallingEdge(ad7606_state_ctx_t *ctx, bool busy_is_high)
{
    if (ctx == NULL) {
        return;
    }
    if ((ctx->state == AD7606_STATE_WAIT_BUSY_LOW) && !busy_is_high) {
        ctx->state = AD7606_STATE_DATA_READY;
    } else {
        ctx->spurious_edge_count++;
    }
}

bool AD7606_StateTakeReady(ad7606_state_ctx_t *ctx)
{
    if ((ctx == NULL) || (ctx->state != AD7606_STATE_DATA_READY)) {
        return false;
    }
    ctx->state = AD7606_STATE_READING;
    return true;
}

void AD7606_StateFinishRead(ad7606_state_ctx_t *ctx)
{
    if ((ctx != NULL) && (ctx->state == AD7606_STATE_READING)) {
        ctx->state = AD7606_STATE_IDLE;
    }
}

bool AD7606_StateTakeTimeout(ad7606_state_ctx_t *ctx)
{
    bool pending;

    if (ctx == NULL) {
        return false;
    }
    pending = ctx->timeout_pending;
    ctx->timeout_pending = false;
    return pending;
}

void AD7606_StateRecover(ad7606_state_ctx_t *ctx)
{
    if (ctx != NULL) {
        ctx->state = AD7606_STATE_IDLE;
        ctx->timeout_pending = false;
    }
}
```

- [ ] **Step 5: Run GREEN tests and commit**

Run: `make -C tests/host clean test`

Expected: executable exits with code 0 and GCC emits no warnings.

The tests must assert `READING` after `AD7606_StateTakeReady()`, call
`AD7606_StateFinishRead()`, and then assert `IDLE`. This locks the rule that
another conversion cannot start during the 128-clock data read.

```powershell
git add Core/USER/ad7606_state.h Core/USER/ad7606_state.c tests/host/test_ad7606_state.c tests/host/Makefile
git commit -m "test: add ad7606 busy state machine"
```

### Task 2: Integrate deterministic BUSY handling and recovery

**Files:**
- Modify: `Core/USER/ad7606.h:68-108`
- Modify: `Core/USER/ad7606.c:94-228`
- Modify: `Core/Src/main.c:183-221,268-288`

- [ ] **Step 1: Add driver-level timeout tests to the host state suite**

Extend `tests/host/test_ad7606_state.c` with these assertions before firmware integration:

```c
static void test_polling_fallback_completes_after_high(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, 100U, false));
    AD7606_StateObserveBusy(&ctx, true, 100U, 2U);
    AD7606_StateObserveBusy(&ctx, false, 100U, 2U);
    assert(AD7606_StateTakeReady(&ctx));
    assert(ctx.state == AD7606_STATE_READING);
    AD7606_StateFinishRead(&ctx);
    assert(ctx.state == AD7606_STATE_IDLE);
}

static void test_recovery_returns_fault_to_idle(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, 5U, true));
    AD7606_StateObserveBusy(&ctx, true, 7U, 2U);
    assert(ctx.state == AD7606_STATE_FAULT);
    AD7606_StateRecover(&ctx);
    assert(ctx.state == AD7606_STATE_IDLE);
}
```

Call both tests from `main()`, then run `make -C tests/host test`.

Expected before integration: PASS; this locks the state contract used by the HAL wrapper.

- [ ] **Step 2: Replace public flag coupling with a typed driver API**

In `Core/USER/ad7606.h`, keep all GPIO mappings unchanged, include `ad7606_state.h`, remove the duplicate `AD7606_CH_NUM`, and expose this API:

```c
#define AD7606_CH_NUM            8U
#define AD7606_BUSY_TIMEOUT_MS   2U

typedef enum {
    AD7606_OK = 0,
    AD7606_NOT_READY,
    AD7606_NULL_POINTER,
    AD7606_BUSY,
    AD7606_TIMEOUT,
    AD7606_INVALID_ARGUMENT
} ad7606_result_t;

ad7606_result_t AD7606_SetOS(uint8_t os_mode);
void AD7606_HW_Init(void);
void AD7606_Prog_Init(void);
ad7606_result_t AD7606_ConvStart(uint32_t now_ms);
void AD7606_Service(uint32_t now_ms);
void AD7606_Busy_IRQHandler(void);
bool AD7606_IsIdle(void);
bool AD7606_IsDataReady(void);
bool AD7606_TakeTimeout(void);
uint32_t AD7606_GetSpuriousEdgeCount(void);
void AD7606_Recover(void);
ad7606_result_t AD7606_ReadData(int16_t data[AD7606_CH_NUM]);
```

Remove public `ad7606_complete_flag` and `ad7606_conv_busy_flag`; `main.c` must use the API rather than editing driver state.

- [ ] **Step 3: Integrate the state machine into the GPIO driver**

Add a private `ad7606_state_ctx_t` in `Core/USER/ad7606.c`. Start the state only after generating CONVST, sample BUSY immediately after the pulse, and require `WAIT_BUSY_LOW` before accepting EXTI:

```c
static ad7606_state_ctx_t ad7606_state;

static uint32_t AD7606_Lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void AD7606_Unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

ad7606_result_t AD7606_ConvStart(uint32_t now_ms)
{
    uint32_t primask;
    bool started;
    bool busy_is_high;

    primask = AD7606_Lock();
    started = AD7606_StateStart(&ad7606_state, now_ms, false);
    AD7606_Unlock(primask);
    if (!started) {
        return AD7606_BUSY;
    }
    AD7606_ConvstLow();
    AD7606_DelayShort();
    AD7606_ConvstHigh();
    AD7606_DelayShort();
    AD7606_ConvstLow();
    busy_is_high = (AD7606_BUSY_READ() == GPIO_PIN_SET);
    primask = AD7606_Lock();
    AD7606_StateObserveBusy(&ad7606_state,
                            busy_is_high,
                            now_ms,
                            AD7606_BUSY_TIMEOUT_MS);
    AD7606_Unlock(primask);
    return AD7606_OK;
}

void AD7606_Service(uint32_t now_ms)
{
    uint32_t primask;
    bool busy_is_high = (AD7606_BUSY_READ() == GPIO_PIN_SET);

    primask = AD7606_Lock();
    AD7606_StateObserveBusy(&ad7606_state,
                            busy_is_high,
                            now_ms,
                            AD7606_BUSY_TIMEOUT_MS);
    AD7606_Unlock(primask);
}

void AD7606_Busy_IRQHandler(void)
{
    uint32_t primask = AD7606_Lock();

    AD7606_StateOnFallingEdge(&ad7606_state,
                              AD7606_BUSY_READ() == GPIO_PIN_SET);
    AD7606_Unlock(primask);
}
```

Wrap every state-machine check/transition with the same short lock, including
`AD7606_IsIdle()`, `AD7606_ConvStart()`, timeout consumption, and recovery.
Never hold the lock over GPIO pulse generation or serial data shifting.

`AD7606_ReadData()` must return `AD7606_NOT_READY` unless
`AD7606_StateTakeReady()` succeeds under the lock. That transition enters
`READING`; after CS returns high, call `AD7606_StateFinishRead()` under a second
short lock. It must still perform all 128 DOUTA clocks in main context, never in
EXTI. Thus `AD7606_IsIdle()` stays false for the entire physical read and TIM6
cannot start a new conversion midway through it.

- [ ] **Step 4: Validate configuration and implement main-context recovery**

Make `AD7606_SetOS()` reject values above `AD7606_OS_64X`. Add this recovery body:

```c
void AD7606_Recover(void)
{
    AD7606_CS_H;
    AD7606_SCLK_H;
    AD7606_ConvstLow();
    AD7606_ResetPulse();
    (void)AD7606_SetOS(AD7606_OS_64X);
    AD7606_StateRecover(&ad7606_state);
}
```

In `APP_ProcessAd7606()`, call `AD7606_Service(HAL_GetTick())`. If `AD7606_TakeTimeout()` is true, increment the timeout counter and call `AD7606_Recover()` in the main loop. Do not reset the device from TIM6 or EXTI callbacks.

- [ ] **Step 5: Keep callbacks bounded and verify firmware build**

TIM6 callback may only update counters/state and call `AD7606_ConvStart(HAL_GetTick())`. EXTI callback may only validate the pin and call `AD7606_Busy_IRQHandler()`.

Run:

```powershell
make -C tests/host clean test
make -C Debug -B all
```

Expected: host tests pass; firmware links; no real C warnings; EXTI contains no channel-read loop, `memcpy`, UART call, delay, or recovery pulse.

```powershell
git add Core/USER/ad7606.h Core/USER/ad7606.c Core/Src/main.c
git commit -m "fix: harden ad7606 busy sequencing"
```

### Task 3: Separate statistics and bound UART retry behavior

**Files:**
- Modify: `Core/Src/main.c:46-79,183-303`

- [ ] **Step 1: Define unambiguous counters and frame invariants**

Replace mixed-unit counters with this debugger-visible structure inside a user-code section:

```c
typedef struct {
    uint32_t conversion_started;
    uint32_t conversion_completed;
    uint32_t trigger_skipped;
    uint32_t conversion_timeout;
    uint32_t spurious_busy_edge;
    uint32_t sample_read_failed;
    uint32_t stored_sample_sets;
    uint32_t frame_dropped;
    uint32_t uart_start_failed;
    uint32_t uart_dma_error;
    uint32_t recovery_count;
} app_stats_t;

volatile app_stats_t g_app_stats;

_Static_assert(BUF_BYTES == 400U, "AD7606 payload must remain 400 bytes");
_Static_assert(sizeof(tx_buf) == 402U, "UART frame must remain 402 bytes");
```

Use `frame_dropped++` for one rejected 25-set buffer; never add `BUF_SAMPLES` to a counter named `frame_dropped`.

- [ ] **Step 2: Add a consistent snapshot API**

Add this prototype and implementation in `main.c` user sections:

```c
void APP_GetStats(app_stats_t *out);

void APP_GetStats(app_stats_t *out)
{
    uint32_t primask;

    if (out == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_app_stats;
    __set_PRIMASK(primask);
}
```

Do not print ASCII statistics on USART3 while the binary stream is active. Inspect `g_app_stats` through the debugger in this phase; RTT output is a separate plan after the local RTT library/tooling is confirmed.

- [ ] **Step 3: Add a 1 ms retry backoff for DMA start failures**

Keep `send_ready` set until DMA actually starts, and add wrap-safe retry timing:

```c
static uint32_t uart_retry_at_ms;

static bool APP_TimeReached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static void APP_ProcessUartTx(void)
{
    HAL_StatusTypeDef status;
    uint32_t now = HAL_GetTick();

    if ((send_ready == 0U) || (uart_tx_idle == 0U) ||
        !APP_TimeReached(now, uart_retry_at_ms)) {
        return;
    }
    tx_buf[0] = 0xAAU;
    tx_buf[1] = 0x55U;
    memcpy(&tx_buf[2], (const uint8_t *)send_buf, BUF_BYTES);
    status = HAL_UART_Transmit_DMA(&huart3, tx_buf, (uint16_t)sizeof(tx_buf));
    if (status == HAL_OK) {
        uart_tx_idle = 0U;
        send_ready = 0U;
    } else {
        g_app_stats.uart_start_failed++;
        uart_retry_at_ms = now + 1U;
    }
}
```

On `HAL_UART_ErrorCallback`, increment `uart_dma_error`, set `uart_tx_idle = 1U`, and retain/requeue the current immutable frame for a full-frame retry. Document that the receiver must resynchronize on `AA 55` because a DMA error may leave a partial frame on the wire.

- [ ] **Step 4: Restore prior interrupt state instead of enabling blindly**

Replace each `__disable_irq()` / `__enable_irq()` pair in application code with:

```c
uint32_t primask = __get_PRIMASK();
__disable_irq();
/* Copy or update only the shared flags here. */
__set_PRIMASK(primask);
```

Keep each critical section limited to flags/pointers; GPIO reads, AD7606 shifts, `memcpy`, and HAL UART calls stay outside.

- [ ] **Step 5: Build, inspect the callbacks, and commit**

Run:

```powershell
make -C Debug -B all
rg -n "HAL_TIM_PeriodElapsedCallback|HAL_GPIO_EXTI_Callback|AD7606_ReadData|memcpy|HAL_UART_Transmit" Core/Src/main.c
```

Expected: build passes with no real C warnings; EXTI has no read/copy/transmit work; frame assertions compile; `AA 55` and 400-byte payload are unchanged.

```powershell
git add Core/Src/main.c
git commit -m "fix: make sampling statistics and uart retries deterministic"
```

### Task 4: Remove latent C and configuration hazards

**Files:**
- Modify: `Core/USER/ad7606.h:64-108`
- Modify: `Core/USER/ad7606.c:122-289`
- Modify: `Core/USER/user_usart.h:1-9`
- Modify: `Core/USER/user_usart.c:1-6`
- Modify: `Core/Src/main.c:118-121,311-321`

- [ ] **Step 1: Correct header constants without touching pins**

Apply these exact semantic changes:

```c
#define AD7606_DB15_H() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DB15_L() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define AD7606_LSB_V ((2.0f * AD7606_V_RANGE) / 65536.0f)
```

Update call sites to `AD7606_DB15_H()` / `AD7606_DB15_L()`, retain one `AD7606_CH_NUM`, and use it in every array declaration. Replace garbled comments with concise ASCII or valid UTF-8 comments. No GPIO port or pin number changes are allowed.

- [ ] **Step 2: Make the UART helper checked and length-safe**

Use this API:

```c
HAL_StatusTypeDef Usart_Send_Computer(UART_HandleTypeDef *huart,
                                      const char *msg);
```

Implement chunked transmission so `strlen()` cannot narrow silently:

```c
HAL_StatusTypeDef Usart_Send_Computer(UART_HandleTypeDef *huart,
                                      const char *msg)
{
    size_t remaining;

    if ((huart == NULL) || (msg == NULL)) {
        return HAL_ERROR;
    }
    remaining = strlen(msg);
    while (remaining > 0U) {
        uint16_t chunk = (remaining > UINT16_MAX)
                             ? UINT16_MAX
                             : (uint16_t)remaining;
        HAL_StatusTypeDef status = HAL_UART_Transmit(
            huart, (uint8_t *)(void *)msg, chunk, 1000U);
        if (status != HAL_OK) {
            return status;
        }
        msg += chunk;
        remaining -= chunk;
    }
    return HAL_OK;
}
```

Include `<stdint.h>`, `<stddef.h>`, and `<string.h>` directly where needed instead of relying on circular `system.h` includes.

- [ ] **Step 3: Check timer startup and preserve binary-UART behavior**

Change startup to:

```c
System_Init();
if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
}
```

Keep fatal ASCII output limited to the stopped/error state. Cast the return explicitly when no recovery is possible:

```c
(void)Usart_Send_Computer(&huart3, "error");
```

- [ ] **Step 4: Run strict compile checks and commit**

Run the normal build, then compile the user modules with the project include/define list plus:

```text
-Wall -Wextra -Wconversion -Wshadow -Wdouble-promotion
-Wformat=2 -Wundef -Wstrict-prototypes -Wmissing-prototypes
```

Expected: no warning in `Core/USER/*.c` or the user sections of `Core/Src/main.c`.

```powershell
git add Core/USER/ad7606.h Core/USER/ad7606.c Core/USER/user_usart.h Core/USER/user_usart.c Core/Src/main.c
git commit -m "fix: remove latent ad7606 and uart c hazards"
```

### Task 5: Verify timing, continuity, and measurement quality on hardware

**Files:**
- Create: `docs/hardware-baseline.md`
- Do not modify firmware configuration during measurement.

- [ ] **Step 1: Record the physical target before changing platform files**

Create `docs/hardware-baseline.md` with this table and fill it only from board markings/tool output:

```markdown
# Hardware Baseline

| Item | Observed value | Evidence |
|---|---|---|
| MCU top marking | Pending hardware verification | Board photo/marking |
| Debug probe | Pending hardware verification | ST-Link or J-Link log |
| Firmware target | STM32F407ZGTx | Current `.ioc` and linker script |
| AD7606 range pin | Pending hardware verification | Schematic or measured level |
| AD7606 oversampling | OS x64 | Firmware GPIO configuration |
```

If the physical MCU is ZE rather than ZG, stop and create a separate CubeMX/linker migration plan before allowing image size to approach 512 KiB.

- [ ] **Step 2: Measure GPIO timing with a logic analyzer**

Capture CONVST A, BUSY, CS, and SCLK together. Record:

```markdown
| Metric | Acceptance |
|---|---|
| CONVST trigger frequency | 1281.738 Hz within instrument tolerance |
| BUSY high time at OS x64 | <= 315 us nominal datasheet maximum |
| BUSY sequence | High observed before accepted falling edge |
| SCLK clocks per conversion | 128 clocks on single DOUTA |
| Readout context | Readout occurs after EXTI return |
```

Mark the whole section `待硬件验证` until traces are captured.

- [ ] **Step 3: Run a 10-minute UART continuity test**

Receive USART3 at 115200 8N1 and validate every frame as exactly 402 bytes beginning with `AA 55`. During the run, watch `g_app_stats` and record start/end values.

Acceptance:

- `conversion_timeout == 0`
- `spurious_busy_edge == 0`
- `sample_read_failed == 0`
- `frame_dropped == 0`
- `uart_start_failed == 0`
- `uart_dma_error == 0`
- observed frame rate is about 25.635 frames/s

Because protocol v1 has no sequence number or CRC, also compare expected frame count with received frame count; do not claim undetectable payload corruption is ruled out.

- [ ] **Step 4: Measure per-channel error statistics**

Apply a known stable DC source within the confirmed input range. For each of 8 channels, collect at least 10,000 samples and report mean, standard deviation, minimum, maximum, and mean error against the reference meter. Keep reference values and measured values in separate columns.

Use the conversion formula:

```text
voltage = signed_code * (2 * range_volts) / 65536
mean_error = measured_mean - reference_voltage
```

Do not infer the RANGE pin state from software alone; confirm it electrically or from the schematic before calculating volts.

- [ ] **Step 5: Final build, documentation commit, and tag candidate**

Run:

```powershell
make -C tests/host clean test
make -C Debug -B all
git status --short
```

Expected: tests/build pass and only intended documentation/result files are changed.

```powershell
git add docs/hardware-baseline.md
git commit -m "docs: record ad7606 hardware baseline"
```

Do not create a release tag until all hardware acceptance rows are filled with evidence.

## Deferred Separate Plans

1. **Target/toolchain alignment:** confirm ZG versus ZE and decide whether CubeIDE remains authoritative or a maintained Keil MDK v5 project is required. This may affect linker memory, startup files, and CI, so it must not be mixed into driver reliability work.
2. **Protocol v2:** add version, payload length, sequence number, and CRC while preserving a selectable v1 mode for the current receiver. The current UART budget is already 89.45%, so baud rate and receiver compatibility must be designed together.
3. **Dual-platform port:** move pin mappings behind board-specific wrappers and port the tested state machine to STM32F103C8T6 only after the F407 hardware baseline passes.
4. **RTT diagnostics:** add nonintrusive periodic statistics only after the local J-Link RTT source/library and probe workflow are confirmed. Do not mix ASCII diagnostics into the active USART3 binary stream.

## Final Acceptance Checklist

- [ ] Host state-machine tests pass with `-Wall -Wextra -Werror -pedantic`.
- [ ] `make -C Debug -B all` succeeds with no real C source warnings.
- [ ] EXTI does no bit-bang read, memory copy, UART operation, delay, or reset.
- [ ] BUSY completion requires a validated conversion state and has a 2 ms timeout/recovery path.
- [ ] Interrupt state is restored with PRIMASK rather than enabled unconditionally.
- [ ] Every statistic has one unit and one meaning.
- [ ] UART retries are rate-limited and cannot spin at main-loop speed.
- [ ] Frame remains `AA 55 + 25 x 8 x int16_t`, exactly 402 bytes.
- [ ] Confirmed pin mapping and `.ioc` configuration are unchanged.
- [ ] Hardware timing, frame continuity, and per-channel error remain explicitly `待硬件验证` until measured.

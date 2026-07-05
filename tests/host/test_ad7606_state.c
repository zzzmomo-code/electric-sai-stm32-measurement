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

static void test_polling_fallback_completes_after_high(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, 100U, false));
    AD7606_StateObserveBusy(&ctx, true, 100U, 2U);
    AD7606_StateObserveBusy(&ctx, false, 100U, 2U);
    assert(AD7606_StateTakeReady(&ctx));
    AD7606_StateFinishRead(&ctx);
    assert(ctx.state == AD7606_STATE_IDLE);
}

static void test_timeout_is_wrap_safe_and_latched_once(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, UINT32_MAX - 1U, true));
    AD7606_StateObserveBusy(&ctx, true, 0U, 2U);
    assert(ctx.state == AD7606_STATE_FAULT);
    assert(AD7606_StateTakeTimeout(&ctx));
    assert(!AD7606_StateTakeTimeout(&ctx));
}

static void test_idle_edge_is_spurious(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    AD7606_StateOnFallingEdge(&ctx, false);
    assert(ctx.spurious_edge_count == 1U);
    assert(!AD7606_StateTakeReady(&ctx));
}

static void test_reading_state_rejects_new_conversion(void)
{
    ad7606_state_ctx_t ctx;

    AD7606_StateInit(&ctx);
    assert(AD7606_StateStart(&ctx, 30U, true));
    AD7606_StateOnFallingEdge(&ctx, false);
    assert(AD7606_StateTakeReady(&ctx));
    assert(!AD7606_StateStart(&ctx, 31U, false));
    AD7606_StateFinishRead(&ctx);
    assert(AD7606_StateStart(&ctx, 32U, false));
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
    assert(!AD7606_StateTakeTimeout(&ctx));
}

int main(void)
{
    test_low_without_seen_high_is_not_complete();
    test_high_then_falling_edge_completes();
    test_polling_fallback_completes_after_high();
    test_timeout_is_wrap_safe_and_latched_once();
    test_idle_edge_is_spurious();
    test_reading_state_rejects_new_conversion();
    test_recovery_returns_fault_to_idle();
    return 0;
}

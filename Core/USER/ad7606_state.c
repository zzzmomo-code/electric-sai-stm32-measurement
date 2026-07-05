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

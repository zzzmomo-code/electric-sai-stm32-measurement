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

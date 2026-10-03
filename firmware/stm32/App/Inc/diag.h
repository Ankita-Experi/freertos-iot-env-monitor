#ifndef DIAG_H
#define DIAG_H

/*
 * On-board diagnostics: UDS (ISO 14229-1) over ISO-TP (ISO 15765-2) on CAN.
 *
 *   0x7E0  physical request   (tester -> this node)
 *   0x7DF  functional request (tester -> all nodes, single frames only)
 *   0x7E8  response           (this node -> tester)
 *
 * Runs entirely inside the CAN task, which is the single owner of CAN TX,
 * so no extra locking is needed around the ISO-TP or UDS state.
 */

#include <stdint.h>

#include "can_bus.h"

void diag_init(void);

/* Returns 1 if the frame is addressed to the diagnostic server. */
int diag_accepts(uint32_t std_id);

/* Hand a received diagnostic frame to the transport layer. */
void diag_on_frame(const can_frame_t *f);

/* Send due Consecutive Frames, run timers, refresh DTCs.
 * Returns how many ms the CAN task may block before calling again. */
uint32_t diag_poll(void);

#endif /* DIAG_H */

/**
 * @file gsp_snapshot.h
 *
 * @brief Tear-safe snapshot capture from garudaData for GSP telemetry.
 *
 * Component: GSP
 */

#ifndef GSP_SNAPSHOT_H
#define GSP_SNAPSHOT_H

#include "gsp_commands.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Capture a tear-safe snapshot of current ESC state.
 * Most fields are 8/16-bit (atomic). 32-bit fields may tear
 * but this is acceptable for telemetry.
 * Prio-7 fields use seqlock/double-read for consistency.
 *
 * @param dst  Destination snapshot struct (caller provides storage)
 */
void GSP_CaptureSnapshot(GSP_SNAPSHOT_T *dst);

#ifdef __cplusplus
}
#endif

#endif /* GSP_SNAPSHOT_H */

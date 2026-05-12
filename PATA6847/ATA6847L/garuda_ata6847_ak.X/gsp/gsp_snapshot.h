/**
 * @file gsp_snapshot.h
 * @brief CK board snapshot capture from gData for GSP telemetry.
 */

#ifndef GSP_SNAPSHOT_H
#define GSP_SNAPSHOT_H

#include "gsp_commands.h"

#ifdef __cplusplus
extern "C" {
#endif

void GSP_CaptureSnapshot(GSP_CK_SNAPSHOT_T *dst);

#ifdef __cplusplus
}
#endif

#endif /* GSP_SNAPSHOT_H */

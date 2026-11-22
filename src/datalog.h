/*
 * Long-term measurement: log in flash (partition slot1_partition, "image-1").
 *
 * The secondary image slot is only needed during a firmware update. As long as
 * no update runs, it holds the measurement log. A Bluetooth update overwrites
 * it: the firmware stops logging as soon as an upload begins, and the app warns
 * beforehand. The last DL_TAIL bytes (MCUboot trailer) are never touched.
 *
 * Layout: 16-byte entries (little endian), contiguous from index 0, followed
 * by erased flash (0xFF). Entry 0 is the log header:
 *   char[4] "FEOL", uint8 version (1), uint8 0, uint16 16, uint32 capacity, uint32 0
 * Then, in any order (byte 0 = type):
 *
 * DL_T_SESSION: start of a measurement
 *   uint8 type, uint8 side (0 unknown, 1 left, 2 right), uint16 threshold (0.01 kg),
 *   uint32 start (Unix s, 0 = time unknown), char[8] pseudonym
 * DL_T_STRIDE: one stride
 *   uint8 type, uint8 flags (GAIT_F_*), uint16 stride_ms, uint32 t_ic (ms uptime),
 *   uint16 stance_ms, uint16 f_peak, uint16 f_mean (0.01 kg), uint16 f_imp (0.01 kg*s)
 * DL_T_MINUTE: one minute (uptime), written at its end
 *   uint8 type, uint8 seconds with movement, uint16 strides, uint32 t_end (ms uptime),
 *   uint16 seconds with heel load > threshold while not moving (in 5 s steps
 *   during rest mode), uint16 max heel load while not moving (0.01 kg),
 *   uint16 battery (mV), uint16 strides with peak > threshold
 * DL_T_EVENT: event
 *   uint8 type, uint8 code (DL_E_*), uint16 arg16, uint32 t (ms uptime),
 *   uint32 Unix s (0 = unknown), uint32 arg32
 *
 * Time: every t is ms since the last power-up (uptime). DL_E_BOOT separates
 * the segments; DL_E_START and DL_E_TIME link uptime to wall-clock time
 * (Unix time of an entry with t_x = unix + (t_x - t) / 1000, same segment).
 */
#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DL_ENTRY      16
#define DL_TAIL       (8 * 1024)

#define DL_T_SESSION  1
#define DL_T_STRIDE   2
#define DL_T_MINUTE   3
#define DL_T_EVENT    4

#define DL_E_START    1   /* arg16 threshold */
#define DL_E_STOP     2
#define DL_E_BOOT     3   /* arg32 reset cause (hwinfo) */
#define DL_E_TIME     4   /* time set by the app */
#define DL_E_REST     5   /* rest mode entered */
#define DL_E_WAKE     6   /* active again */
#define DL_E_ZERO     7   /* power-on zero, arg16 = VREF_0 in 0.1 mV (0 = implausible) */
#define DL_E_MARKER   8   /* marker from the app, arg16 = number */
#define DL_E_BATLOW   9   /* arg16 = battery (mV) */
#define DL_E_FULL     10  /* log full, measurement ended */

/* Control commands (byte 0 of the written value) */
#define DL_C_TIME     1   /* uint32 Unix s */
#define DL_C_START    2   /* uint16 threshold (0.01 kg), uint8 side, uint8 options (bit 0: erase first), char[8] pseudonym */
#define DL_C_STOP     3
#define DL_C_ERASE    4
#define DL_C_READ     5   /* uint32 from index (0xFFFFFFFF = abort) */
#define DL_C_MARKER   6   /* uint16 number */

/* Status bits, see dl_status() */
#define DL_ST_RUNNING 0x01
#define DL_ST_VALID   0x02   /* log present (header valid) */
#define DL_ST_ERASING 0x04
#define DL_ST_TIME    0x08   /* wall-clock time known */
#define DL_ST_LOCKED  0x10   /* logging impossible (image unconfirmed or update running) */
#define DL_ST_FULL    0x20
#define DL_STATUS_LEN 36

int dl_init(void);
/* logging allowed (image confirmed); false locks */
void dl_allow(bool ok);
/* firmware upload started: stop logging, the log counts as overwritten */
void dl_dfu_started(void);

/* control command from the BT thread (executed in dl_poll()) */
int dl_command(const uint8_t *b, size_t len);
/* Status: uint8 version, uint8 DL_ST_*, uint16 (stream mask, filled in by main.c),
 * uint32 entries, uint32 capacity, uint32 Unix s now, uint16 threshold,
 * uint8 side, uint8 erase progress (%), char[8] pseudonym,
 * uint32 index of the session header, uint32 uptime (ms) */
void dl_status(uint8_t b[DL_STATUS_LEN]);

bool dl_running(void);
uint32_t dl_now(void);
float dl_thr_kg(void);
void dl_event(uint8_t code, uint16_t arg16, uint32_t arg32);

/* measurements (main loop) */
void dl_stride(uint8_t flags, uint16_t stride_ms, uint32_t t_ic, uint16_t stance_ms,
	       float peak_kg, float mean_kg, float imp_kgs);
void dl_second(bool moving, float still_max_kg, uint16_t bat_mv);
void dl_rest_sample(float kg, uint16_t secs, uint16_t bat_mv);

/* main loop: commands, erasing, end of minute. Returns true if it must be
 * called again soon (erase or readout in progress). */
bool dl_poll(void);

/* Readout: packet = uint32 index + n entries; n = 0 marks the end.
 * Returns the length (0 = nothing to send); call dl_read_done() after the
 * packet was sent. */
size_t dl_read_fill(uint8_t *buf, size_t max);
void dl_read_done(void);
/* uptime of the next minute boundary (for rest mode), INT64_MAX if not logging */
int64_t dl_minute_end(void);

#endif

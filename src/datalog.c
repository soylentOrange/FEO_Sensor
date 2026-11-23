/*
 * Long-term measurement: log in flash, format see datalog.h.
 * Everything except dl_command(), dl_dfu_started() and dl_status() runs in the
 * main loop; commands from the Bluetooth thread go through a queue.
 */
#include "datalog.h"
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/hwinfo.h>
#include <string.h>
#include <math.h>

#define ERASE_CHUNK  4096
#define MINUTE_MS    60000
#define CMD_LEN      20

static const struct flash_area *fa;
static uint32_t cap;             /* entries */
static uint32_t head;            /* next free entry */
static bool valid;               /* header (entry 0) valid */
static bool running, full;
static atomic_t allowed = ATOMIC_INIT(1);
static atomic_t dfu;

/* session */
static uint16_t thr_cg = 200;
static uint8_t side;
static char pseudo[8];
static uint32_t sess_idx;
static uint8_t ver = DL_VERSION;      /* format of the log in flash */

/* wall clock: Unix ms at uptime 0, 0 = unknown */
static int64_t unix_ref_ms;

/* erasing */
static uint32_t erase_off, erase_end;   /* erase_end 0 = not erasing */
static uint8_t pending_start[CMD_LEN];  /* start after erasing, [0] = 0: none */

/* readout */
static int64_t rd_idx = -1;
static uint32_t rd_n;

/* current minute */
static struct {
	int64_t end;
	uint16_t strides, over, rest_s, rest_max, bat, lie_s, up_s;
	uint8_t move_s;
} mn;
static bool bat_low_logged;

K_MSGQ_DEFINE(cmd_q, CMD_LEN, 8, 4);

static bool erased(const uint8_t e[DL_ENTRY])
{
	for (int i = 0; i < DL_ENTRY; i++) {
		if (e[i] != 0xff) {
			return false;
		}
	}
	return true;
}

static int rd(uint32_t idx, uint8_t e[DL_ENTRY])
{
	return flash_area_read(fa, (off_t)idx * DL_ENTRY, e, DL_ENTRY);
}

static uint16_t cg(float kg)
{
	if (!(kg >= 0)) {
		return 0;
	}
	return (uint16_t)MIN(lroundf(fminf(kg, 1000.0f) * 100.0f), UINT16_MAX);
}

uint32_t dl_now(void)
{
	return unix_ref_ms ? (uint32_t)((unix_ref_ms + k_uptime_get()) / 1000) : 0;
}

bool dl_running(void)
{
	return running;
}

float dl_thr_kg(void)
{
	return thr_cg / 100.0f;
}

static bool can_write(void)
{
	return fa && valid && !atomic_get(&dfu);
}

/* append an entry; the last free slot is kept for DL_E_FULL */
static int put(const uint8_t e[DL_ENTRY])
{
	bool full_ev = e[0] == DL_T_EVENT && e[1] == DL_E_FULL;

	if (!can_write() || head >= cap || (head >= cap - 1 && !full_ev)) {
		return -ENOSPC;
	}
	int err = flash_area_write(fa, (off_t)head * DL_ENTRY, e, DL_ENTRY);

	if (err) {
		printk("# Dauermessung: Schreibfehler %d\n", err);
		return err;
	}
	head++;
	if (running && head >= cap - 1 && !full_ev) {
		full = true;
		running = false;
		dl_event(DL_E_FULL, 0, 0);
		printk("# Dauermessung: Speicher voll, beendet\n");
	}
	return 0;
}

static void ev_write(uint8_t code, uint16_t arg16, uint32_t arg32)
{
	uint8_t e[DL_ENTRY] = { DL_T_EVENT, code };

	sys_put_le16(arg16, e + 2);
	sys_put_le32((uint32_t)k_uptime_get(), e + 4);
	sys_put_le32(dl_now(), e + 8);
	sys_put_le32(arg32, e + 12);
	put(e);
}

void dl_event(uint8_t code, uint16_t arg16, uint32_t arg32)
{
	if (running || code == DL_E_STOP || code == DL_E_FULL) {
		ev_write(code, arg16, arg32);
	}
}

static void minute_reset(int64_t end)
{
	uint16_t bat = mn.bat;

	memset(&mn, 0, sizeof(mn));
	mn.end = end;
	mn.bat = bat;
}

static void minute_write(void)
{
	uint8_t e[DL_ENTRY] = { DL_T_MINUTE, mn.move_s };

	sys_put_le16(mn.strides, e + 2);
	sys_put_le32((uint32_t)k_uptime_get(), e + 4);
	e[8] = (uint8_t)MIN(mn.rest_s, UINT8_MAX);
	e[9] = (uint8_t)MIN(mn.lie_s, UINT8_MAX);
	sys_put_le16(mn.rest_max, e + 10);
	sys_put_le16(mn.bat, e + 12);
	e[14] = (uint8_t)MIN(mn.over, UINT8_MAX);
	e[15] = (uint8_t)MIN(mn.up_s, UINT8_MAX);
	put(e);
}

static void write_magic(void)
{
	uint8_t e[DL_ENTRY] = { 'F', 'E', 'O', 'L', DL_VERSION, 0 };

	sys_put_le16(DL_ENTRY, e + 6);
	sys_put_le32(cap, e + 8);
	sys_put_le32(0, e + 12);
	valid = true;
	ver = DL_VERSION;
	head = 0;
	if (put(e)) {
		valid = false;
	}
}

static void start_now(const uint8_t *c)
{
	uint8_t e[DL_ENTRY] = { DL_T_SESSION };

	thr_cg = sys_get_le16(c + 1);
	side = c[3];
	memcpy(pseudo, c + 5, sizeof(pseudo));
	e[1] = side;
	sys_put_le16(thr_cg, e + 2);
	sys_put_le32(dl_now(), e + 4);
	memcpy(e + 8, pseudo, sizeof(pseudo));
	sess_idx = head;
	if (put(e)) {
		printk("# Dauermessung: Start fehlgeschlagen\n");
		return;
	}
	running = true;
	full = false;
	bat_low_logged = false;
	ev_write(DL_E_START, thr_cg, 0);
	minute_reset(k_uptime_get() + MINUTE_MS);
	printk("# Dauermessung gestartet (Schwelle %u.%02u kg, %u von %u Einträgen belegt)\n",
	       thr_cg / 100, thr_cg % 100, head, cap);
}

static void stop_now(void)
{
	if (!running) {
		return;
	}
	minute_write();
	ev_write(DL_E_STOP, 0, 0);
	running = false;
	printk("# Dauermessung gestoppt (%u Einträge)\n", head);
}

static void erase_begin(void)
{
	/* erase only the used part if the log is valid, otherwise everything */
	erase_end = valid ? ROUND_UP(head * DL_ENTRY, ERASE_CHUNK) : cap * DL_ENTRY;
	erase_end = MAX(erase_end, ERASE_CHUNK);
	erase_off = 0;
	valid = false;
	running = false;
	rd_idx = -1;
	printk("# Dauermessung: lösche %u KiB\n", erase_end / 1024);
}

static void erase_step(void)
{
	if (flash_area_erase(fa, erase_off, ERASE_CHUNK)) {
		printk("# Dauermessung: Löschfehler bei %u\n", erase_off);
	}
	erase_off += ERASE_CHUNK;
	if (erase_off >= erase_end) {
		erase_end = 0;
		write_magic();
		full = false;
		printk("# Dauermessung: Speicher gelöscht\n");
		if (pending_start[0]) {
			start_now(pending_start);
			pending_start[0] = 0;
		}
	}
}

static void exec(const uint8_t *c)
{
	switch (c[0]) {
	case DL_C_TIME: {
		uint32_t u = sys_get_le32(c + 1);

		unix_ref_ms = u ? (int64_t)u * 1000 - k_uptime_get() : 0;
		dl_event(DL_E_TIME, 0, 0);
		break;
	}
	case DL_C_START:
		if (running || erase_end || !atomic_get(&allowed) || atomic_get(&dfu)) {
			break;
		}
		/* a log in the old format is erased before a new measurement */
		if (!valid || ver != DL_VERSION || (c[4] & 1) || head >= cap - 16) {
			memcpy(pending_start, c, CMD_LEN);
			erase_begin();
		} else {
			start_now(c);
		}
		break;
	case DL_C_STOP:
		stop_now();
		break;
	case DL_C_ERASE:
		if (!erase_end && !atomic_get(&dfu)) {
			stop_now();
			pending_start[0] = 0;
			erase_begin();
		}
		break;
	case DL_C_READ: {
		uint32_t from = sys_get_le32(c + 1);

		rd_idx = from == UINT32_MAX ? -1 : from;
		break;
	}
	case DL_C_MARKER:
		dl_event(DL_E_MARKER, sys_get_le16(c + 1), 0);
		break;
	default:
		break;
	}
}

int dl_command(const uint8_t *b, size_t len)
{
	uint8_t c[CMD_LEN] = { 0 };

	if (len < 1 || len > CMD_LEN) {
		return -EINVAL;
	}
	memcpy(c, b, len);
	return k_msgq_put(&cmd_q, c, K_NO_WAIT);
}

void dl_allow(bool ok)
{
	atomic_set(&allowed, ok);
}

void dl_dfu_started(void)
{
	atomic_set(&dfu, 1);
}

int dl_init(void)
{
	uint8_t e[DL_ENTRY];
	int err = flash_area_open(PARTITION_ID(slot1_partition), &fa);

	if (err) {
		fa = NULL;
		printk("# Dauermessung: kein Speicherbereich (%d)\n", err);
		return err;
	}
	cap = (uint32_t)((fa->fa_size - DL_TAIL) / DL_ENTRY);
	valid = rd(0, e) == 0 && !memcmp(e, "FEOL", 4) && (e[4] == 1 || e[4] == DL_VERSION) &&
		sys_get_le32(e + 8) == cap;
	ver = valid ? e[4] : DL_VERSION;
	if (!valid) {
		printk("# Dauermessung: Speicher leer oder überschrieben (vor dem Start löschen)\n");
		return 0;
	}
	/* find the first free entry (entries are contiguous from 0) */
	uint32_t lo = 1, hi = cap;

	while (lo < hi) {
		uint32_t mid = lo + (hi - lo) / 2;

		if (rd(mid, e) == 0 && erased(e)) {
			hi = mid;
		} else {
			lo = mid + 1;
		}
	}
	head = lo;
	full = head >= cap - 1;

	/* was a measurement running at power-off? Search back to session header or stop
	 * (an old-format log is only kept for reading out) */
	for (uint32_t i = head; ver == DL_VERSION && i-- > 1;) {
		if (rd(i, e)) {
			break;
		}
		if (e[0] == DL_T_EVENT && (e[1] == DL_E_STOP || e[1] == DL_E_FULL)) {
			break;
		}
		if (e[0] == DL_T_SESSION) {
			side = e[1];
			thr_cg = sys_get_le16(e + 2);
			memcpy(pseudo, e + 8, sizeof(pseudo));
			sess_idx = i;
			running = !full;
			break;
		}
	}
	printk("# Dauermessung: %u von %u Einträgen belegt%s\n", head, cap,
	       running ? ", Messung läuft weiter" : "");
	if (running) {
		uint32_t cause = 0;

		if (IS_ENABLED(CONFIG_HWINFO)) {
			hwinfo_get_reset_cause(&cause);
			hwinfo_clear_reset_cause();
		}
		ev_write(DL_E_BOOT, 0, cause);
		minute_reset(k_uptime_get() + MINUTE_MS);
	}
	return 0;
}

void dl_status(uint8_t b[DL_STATUS_LEN])
{
	uint8_t f = (running ? DL_ST_RUNNING : 0) | (valid ? DL_ST_VALID : 0) |
		    (erase_end ? DL_ST_ERASING : 0) | (unix_ref_ms ? DL_ST_TIME : 0) |
		    (!atomic_get(&allowed) || atomic_get(&dfu) ? DL_ST_LOCKED : 0) |
		    (full ? DL_ST_FULL : 0);

	memset(b, 0, DL_STATUS_LEN);
	b[0] = 1;
	b[1] = f;
	sys_put_le32(valid ? head : 0, b + 4);
	sys_put_le32(cap, b + 8);
	sys_put_le32(dl_now(), b + 12);
	sys_put_le16(thr_cg, b + 16);
	b[18] = side;
	b[19] = erase_end ? (uint8_t)(100ULL * erase_off / erase_end) : 0;
	memcpy(b + 20, pseudo, sizeof(pseudo));
	sys_put_le32(sess_idx, b + 28);
	sys_put_le32((uint32_t)k_uptime_get(), b + 32);
}

void dl_stride(const struct gait_stride *s)
{
	if (!running) {
		return;
	}
	uint8_t e[DL_ENTRY] = { DL_T_STRIDE, (uint8_t)s->flags };
	uint16_t pk = cg(s->f_peak_kg);

	sys_put_le16(s->stride_ms, e + 2);
	sys_put_le32(s->t_ic_ms, e + 4);
	sys_put_le16(s->stance_ms, e + 8);
	sys_put_le16(pk, e + 10);
	sys_put_le16(cg(s->f_mean_kg), e + 12);
	sys_put_le16(cg(s->f_imp_kgs), e + 14);
	if (put(e)) {
		return;
	}
	uint8_t x[DL_ENTRY] = { DL_T_STRIDE_X, s->heel_pct };

	sys_put_le16(s->t_fpeak_ms, x + 2);
	sys_put_le32(s->t_ic_ms, x + 4);
	sys_put_le16((uint16_t)CLAMP(lroundf(s->ms_dps * 10.0f), 0, UINT16_MAX), x + 8);
	sys_put_le16((uint16_t)CLAMP(lroundf(s->rom_deg * 10.0f), 0, UINT16_MAX), x + 10);
	sys_put_le16((uint16_t)CLAMP(lroundf(s->f_rate_kgps * 10.0f), 0, UINT16_MAX), x + 12);
	sys_put_le16((uint16_t)(int16_t)CLAMP(lroundf(s->f_swing_kg * 100.0f), INT16_MIN, INT16_MAX), x + 14);
	put(x);
	mn.strides++;
	if (pk > thr_cg) {
		mn.over++;
	}
}

static void bat_note(uint16_t bat_mv)
{
	if (!bat_mv) {
		return;
	}
	mn.bat = bat_mv;
	if (bat_mv < 3500 && !bat_low_logged) {
		bat_low_logged = true;
		dl_event(DL_E_BATLOW, bat_mv, 0);
	}
}

/* not moving for secs seconds: heel load, else lying / leg up or upright by the shank tilt */
static void still_note(uint16_t f, float tilt_deg, uint16_t secs)
{
	if (f > thr_cg) {
		mn.rest_s += secs;
	} else if (tilt_deg > DL_LIE_DEG) {
		mn.lie_s += secs;
	} else if (tilt_deg >= 0) {          /* not NAN */
		mn.up_s += secs;
	}
	mn.rest_max = MAX(mn.rest_max, f);
}

void dl_second(bool moving, float still_max_kg, float tilt_deg, uint16_t bat_mv)
{
	if (!running) {
		return;
	}
	if (moving) {
		mn.move_s = MIN(mn.move_s + 1, 255);
	} else {
		still_note(cg(still_max_kg), tilt_deg, 1);
	}
	bat_note(bat_mv);
}

void dl_rest_sample(float kg, float tilt_deg, uint16_t secs, uint16_t bat_mv)
{
	if (!running) {
		return;
	}
	still_note(cg(kg), tilt_deg, secs);
	bat_note(bat_mv);
}

bool dl_poll(void)
{
	uint8_t c[CMD_LEN];

	if (!fa) {
		while (k_msgq_get(&cmd_q, c, K_NO_WAIT) == 0) {
		}
		return false;
	}
	if (atomic_get(&dfu) && (running || valid)) {
		running = false;
		valid = false;
		erase_end = 0;
		rd_idx = -1;
		printk("# Dauermessung: Firmware-Update, Messung beendet\n");
	}
	if (erase_end) {
		erase_step();
	} else if (k_msgq_get(&cmd_q, c, K_NO_WAIT) == 0) {
		exec(c);
	}
	if (running) {
		int64_t now = k_uptime_get();

		if (now >= mn.end) {
			minute_write();
			/* after a long sleep, do not write an entry for every missed minute */
			minute_reset(now - mn.end > MINUTE_MS ? now + MINUTE_MS : mn.end + MINUTE_MS);
		}
	}
	return erase_end || rd_idx >= 0 || k_msgq_num_used_get(&cmd_q);
}

int64_t dl_minute_end(void)
{
	return running ? mn.end : INT64_MAX;
}

size_t dl_read_fill(uint8_t *buf, size_t max)
{
	if (rd_idx < 0 || max < 4 || erase_end) {
		return 0;
	}
	uint32_t idx = (uint32_t)rd_idx;
	uint32_t end = valid ? head : 0;
	uint32_t n = 0;

	sys_put_le32(idx, buf);
	while (idx + n < end && 4 + (n + 1) * DL_ENTRY <= max) {
		if (rd(idx + n, buf + 4 + n * DL_ENTRY)) {
			break;
		}
		n++;
	}
	rd_n = n;
	return 4 + n * DL_ENTRY;
}

void dl_read_done(void)
{
	if (rd_idx < 0) {
		return;
	}
	rd_idx = rd_n ? rd_idx + rd_n : -1;    /* empty packet = end */
}

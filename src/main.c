/*
 * Fersenentlastungsorthese, Sensor-Firmware für den XIAO nRF54L15 Sense:
 * IMU (LSM6DS3TR-C), Kraftsensor (FSR mit Verstärker an A0) und Akkuspannung
 * mit 100 Hz erfassen und
 *   1. als CSV über USB-Seriell ausgeben (115200 Baud, zum Aufzeichnen) und
 *   2. per Bluetooth LE live streamen (für die Web-App).
 *
 * Kraftsensor: Der MCP602 wird über D1 versorgt (High = an), sein Ausgang
 * liegt an A0 (AIN0). Der SAADC (interne Referenz 0,9 V, 12 bit, 8-fach
 * Oversampling) schaltet die Verstärkung automatisch um:
 *   Gain 1   -> Messbereich 0..0,9 V,  hoch bei > 800 mV
 *   Gain 1/2 -> Messbereich 0..1,8 V,  hoch bei > 1600 mV, runter bei < 350 mV
 *   Gain 1/4 -> Messbereich 0..3,6 V,                     runter bei < 700 mV
 * Die erste Messung nach einem Wechsel wird verworfen.
 * Akku: alle 1 s über den Teiler 1:2 an AIN7 (Gain 1/4); der Teiler wird nur
 * für die Messung zugeschaltet (Lastschalter vbat_pwr).
 *
 * Seriell, eine Zeile pro Messung:
 *   t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,u_fsr_mv,fsr_range_mv,u_bat_mv
 * (fsr_range_mv = Messbereich, mit dem u_fsr_mv gemessen wurde: 900, 1800 oder 3600)
 * Zeilen, die mit '#' beginnen, sind Kommentare.
 * Die Zeilen werden in Schüben zu SERIAL_BATCH Zeilen gesendet: Die USB-Brücke
 * des XIAO leitet Daten nur dann sofort weiter, wenn die Leitung zwischendurch
 * > ca. 10 ms ruht. Bei gleichmäßigen 100 Zeilen/s sammelt sie sonst 16 KiB
 * (ca. 4 s) bevor etwas am Mac ankommt.
 *
 * BLE: Gerätename "XIAO-FEO", Service FEO_SVC_UUID.
 * Characteristic FEO_DATA_UUID (notify), ein Paket (little endian):
 *   uint32 t_ms        Zeitstempel der ersten Messung im Paket
 *   uint8  n           Anzahl Messungen (Abstand jeweils 1000/SAMPLE_RATE_HZ ms)
 *   n x {
 *     6 x int16        ax,ay,az in mg; gx,gy,gz in 0,1 Grad/s
 *     uint16           Bit 0..13: U_FSR in mV, Bit 14..15: Messbereich
 *                      (0 = 0,9 V / Gain 1, 1 = 1,8 V / Gain 1/2, 2 = 3,6 V / Gain 1/4)
 *   }
 * Characteristic FEO_BAT_UUID (read, notify 1x pro Sekunde):
 *   uint16 U_Akku in mV (0 = noch nicht gemessen)
 * Characteristic FEO_CAL_UUID (read, write): Kalibrierung des Kraftsensors,
 * im Flash gespeichert (Settings), die App rechnet daraus kg:
 *   float32 VREF (mV), RF (Ohm), RS (Ohm), K (Ohm*kg^N), N; uint32 Zeitpunkt (Unix-s, 0 = Werkswert)
 *   Lesen liefert zusätzlich float32 VREF_0: beim Einschalten gemessene Ruhespannung
 *   (2 s nach dem Start 1 s gemittelt, 0 = noch nicht gemessen oder unplausibel).
 *   Geschrieben werden nur die ersten 24 Byte.
 *   Modell: R_FSR = RS + K * kg^-N, U = VREF * (1 + RF / R_FSR)
 *
 * Characteristic FEO_FRAME_UUID (read, write): Ausrichtung des Sensors am
 * Unterschenkel (Segment-Koordinatensystem, siehe src/gait.h), im Flash:
 *   float32 R[3][3] (Zeilen x vorn, y rechts = Sagittalachse, z nach unten, je in
 *   Sensorkoordinaten), uint32 Zeitpunkt (Unix-s; 0 = keine Kalibrierung -> automatisch)
 *   Lesen liefert zusätzlich float32 Achse[3] (aktuell benutzte Sagittalachse),
 *   float32 Gyro-Offset[3] (Grad/s), uint32 Anzahl Doppelschritte seit dem Einschalten.
 *   Geschrieben werden 40 Byte.
 * Characteristic FEO_STEP_UUID (notify, write): ein Datensatz pro Doppelschritt
 * (Fersenauftritt bis Fersenauftritt, Schritterkennung nach Fadillioglu et al. 2020):
 *   uint32 seq, uint32 t_ic_ms, uint16 stride_ms, uint16 stance_ms,
 *   int16 ms, ic, to (Drehrate in 0,1 Grad/s), uint16 f_peak, f_mean (0,01 kg),
 *   uint16 f_imp (0,01 kg*s), uint16 t_fpeak_ms, int16 f_swing (0,01 kg), uint16 flags,
 *   20 x uint16 Kraft über den Gangzyklus (Mittel über je 5 %, IC = 0 %, 0,01 kg),
 *   8 x uint16 Mittel, 8 x uint16 Maximum der Kraft je Gangphase nach Rancho Los Amigos
 *   (IC, LR, MSt, TSt, PSw, ISw, MSw, TSw; Grenzen siehe src/gait.h; 0,01 kg) (102 Byte). Die letzten STEP_LOG Datensätze bleiben im RAM; uint32 seq schreiben
 *   schickt alle ab dieser Nummer noch einmal (z. B. nach einer Unterbrechung).
 * Seriell erscheint jeder Datensatz als Kommentarzeile
 *   #S,seq,t_ic_ms,stride_ms,stance_ms,ms_dps,ic_dps,to_dps,f_peak_kg,f_mean_kg,f_imp_kgs,t_fpeak_ms,f_swing_kg,flags,
 *     f_0..f_19 (kg je 5 % des Gangzyklus), 8 x Mittel und 8 x Maximum je Gangphase (kg)
 *
 * Characteristic FEO_CTRL_UUID (read, write, notify about once per second): status of
 *   the long-term log plus activity, and control commands (see src/datalog.h, README).
 *   Command 7 sets the output mask OUT_* (raw data, stride records, serial CSV, serial #S),
 *   stored in flash. Battery and this status are always on while connected.
 * Characteristic FEO_LOG_UUID (notify): long-term log readout, uint32 index + n x 16 bytes.
 *
 * Energiesparen: Ist das Bein REST_AFTER_S lang ruhig und streamt keine App die
 * Rohdaten, geht der Sensor in den Ruhemodus: Gyro aus, Beschleunigungssensor im
 * Low-Power-Modus (26 Hz, ±2 g) mit Aufwach-Interrupt (INT1 an P0.02), MCP602 aus,
 * keine seriellen Daten; der nRF schläft bis zur nächsten Bewegung (Akku alle 60 s).
 * Bluetooth wirbt nach 30 s nur noch langsam (ca. 1 s). Die Rohdaten (FEO_DATA_UUID)
 * schickt die Firmware nur, wenn die App sie abonniert.
 * LED: off at rest; dimly on while movement is detected, a bright flash at every
 * initial contact (IC). Dim and bright level (% PWM duty) and flash length are set
 * from the app (control command 9, stored in flash).
 *
 * Firmware-Update (nur Build mit MCUboot, zephyr/ota.conf): MCUmgr/SMP-Service
 * über BLE. Ein neues Image läuft nach dem Neustart zunächst zur Probe; die
 * Firmware bestätigt es selbst, sobald IMU, ADC und Bluetooth laufen. Startet
 * es nicht bis dahin, stellt MCUboot beim nächsten Reset die alte Version wieder her.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include "gait.h"
#include "datalog.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/settings/settings.h>
#include <string.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#ifdef CONFIG_MCUBOOT_IMG_MANAGER
#include <zephyr/dfu/mcuboot.h>
#endif
#ifdef CONFIG_MCUMGR_GRP_IMG_STATUS_HOOKS
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt_callbacks.h>
#endif

#define SAMPLE_RATE_HZ   100   /* Ausgaberate */
#define IMU_ODR_HZ       208   /* interne Messrate des Sensors (>= 2x Ausgaberate) */
#define ACCEL_RANGE_G    16    /* +-2, 4, 8 oder 16 g; Fersenaufprall kann > 8 g sein */
#define GYRO_RANGE_DPS   2000  /* +-245, 500, 1000 oder 2000 Grad/s */
#define BLE_BATCH        5     /* Messungen pro BLE-Paket -> 20 Pakete/s */
#define SERIAL_BATCH     2     /* Zeilen pro seriellem Schub, siehe unten */
#define BAT_PERIOD       SAMPLE_RATE_HZ   /* Akku alle 100 Messungen = 1 s messen */
#define BAT_DIVIDER      2     /* Spannungsteiler 1:2 auf dem XIAO */

#define REST_AFTER_S     20    /* so lange ruhig -> Ruhemodus */
#define REST_BAT_S       60    /* Akku im Ruhemodus messen */
#define REST_FSR_S       5     /* rest mode while logging: sample heel load every 5 s */
#define STEP_LOG         1024  /* Schritt-Datensätze im RAM */
#define STEP_LEN         (30 + 2 * GAIT_BINS + 4 * GAIT_PHASES)
#define ADV_FAST_S       30    /* nach Start/Trennen so lange schnell werben */

#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)
#define G_TO_MS2         9.80665
#define RAD_TO_DEG       57.29577951308232
#define PKT_HDR          5
#define PKT_SAMPLE       14

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));
static const struct gpio_dt_spec fsr_pwr = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fsr_pwr_gpios);
static const struct adc_dt_spec adc_fsr = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), fsr);
static const struct adc_dt_spec adc_bat = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vbat);
static const struct device *const vbat_sw = DEVICE_DT_GET(DT_NODELABEL(vbat_pwr));
static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(DT_ALIAS(imu0));
static const struct gpio_dt_spec imu_int = GPIO_DT_SPEC_GET(DT_ALIAS(imu0), irq_gpios);

/* ---------------- Bluetooth ---------------- */

/* 7a1e0011-5c3b-4f0a-9d2e-3c8f1b6a4d01 (eigene UUIDs, damit die alte IMU-App das Gerät nicht anzeigt) */
#define FEO_SVC_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0011, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
#define FEO_DATA_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0012, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
#define FEO_BAT_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0013, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
static const struct bt_uuid_128 feo_svc_uuid = BT_UUID_INIT_128(FEO_SVC_UUID_VAL);
static const struct bt_uuid_128 feo_data_uuid = BT_UUID_INIT_128(FEO_DATA_UUID_VAL);
static const struct bt_uuid_128 feo_bat_uuid = BT_UUID_INIT_128(FEO_BAT_UUID_VAL);
#define FEO_CAL_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0014, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
static const struct bt_uuid_128 feo_cal_uuid = BT_UUID_INIT_128(FEO_CAL_UUID_VAL);
#define FEO_FRAME_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0015, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
#define FEO_STEP_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0016, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
static const struct bt_uuid_128 feo_frame_uuid = BT_UUID_INIT_128(FEO_FRAME_UUID_VAL);
static const struct bt_uuid_128 feo_step_uuid = BT_UUID_INIT_128(FEO_STEP_UUID_VAL);
#define FEO_CTRL_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0017, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
#define FEO_LOG_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0018, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
static const struct bt_uuid_128 feo_ctrl_uuid = BT_UUID_INIT_128(FEO_CTRL_UUID_VAL);
static const struct bt_uuid_128 feo_log_uuid = BT_UUID_INIT_128(FEO_LOG_UUID_VAL);

/* Switchable outputs (bit mask, stored in flash). Battery and the status/activity
 * notification on FEO_CTRL_UUID are always on while connected. */
#define OUT_BLE_RAW      BIT(0)   /* raw data on FEO_DATA_UUID */
#define OUT_BLE_STEP     BIT(1)   /* stride records on FEO_STEP_UUID */
#define OUT_SER_CSV      BIT(2)   /* serial CSV lines */
#define OUT_SER_STEP     BIT(3)   /* serial #S lines */
#define OUT_ALL          0x000f
#define CTRL_OUTPUTS     7        /* control command: uint16 output mask */
#define CTRL_LED         9        /* control command: uint8 dim %, uint8 bright %, uint16 flash ms */
#define CTRL_PASSKEY     8        /* control command: uint32 passkey 0..999999, 0xFFFFFFFF = none */
#define PK_NONE          0xFFFFFFFFu
#define PK_RESET_MS      5000     /* hold the user button this long at power-up: clear passkey + bonds */
#define PK_PAIR_S        60       /* passkey set: disconnect if the link is not paired by then */
#define CTRL_STATUS_LEN  (DL_STATUS_LEN + 12)
static atomic_t outputs = ATOMIC_INIT(OUT_ALL);
static atomic_t passkey = ATOMIC_INIT((atomic_val_t)PK_NONE);
/* LED (stored in flash): dimly on while moving, a bright flash at every initial
 * contact. Brightness in % duty cycle of a software PWM (the LED pin P2.0 has
 * no hardware PWM), 0 = off. */
#define LED_PWM_US       4000     /* PWM period (250 Hz) */
#define LED_FLASH_MAX_MS 2000
static atomic_t led_dim = ATOMIC_INIT(3);        /* % while moving */
static atomic_t led_bright = ATOMIC_INIT(100);   /* % at initial contact */
static atomic_t led_flash_ms = ATOMIC_INIT(300);
static atomic_t stride_total;     /* strides since power-up */
static atomic_t activity;         /* 0 rest mode, 1 still, 2 moving */
static atomic_t still_secs;

/* ---------------- Kalibrierung (im Flash) ---------------- */

struct fsr_cal {
	float vref_mv, rf_ohm, rs_ohm, k, n;
	uint32_t stamp;         /* Unix-Zeit der Kalibrierung, 0 = Werkswert */
};
#define CAL_LEN 24
/* Werkswerte: Kalibrierung vom 22.11.2026 (Rf = 12k, runder 30-mm-FSR) */
static struct fsr_cal cal = { 94.9f, 12000.0f, 324.7f, 4704.9f, 1.102f, 0 };
static float vref_zero;  /* beim Einschalten gemessene Ruhespannung (mV), 0 = keine */
#define ZERO_START   (2 * SAMPLE_RATE_HZ)   /* Filter/OPV einschwingen lassen */
#define ZERO_LEN     SAMPLE_RATE_HZ         /* 1 s mitteln */
#define ZERO_MIN_MV  60                     /* Plausibilitätsfenster für V_ref */
#define ZERO_MAX_MV  150

static void cal_pack(const struct fsr_cal *c, uint8_t *b)
{
	const float f[5] = { c->vref_mv, c->rf_ohm, c->rs_ohm, c->k, c->n };

	for (int i = 0; i < 5; i++) {
		uint32_t u;

		memcpy(&u, &f[i], 4);
		sys_put_le32(u, b + 4 * i);
	}
	sys_put_le32(c->stamp, b + 20);
}

static bool cal_unpack(const uint8_t *b, struct fsr_cal *c)
{
	float f[5];

	for (int i = 0; i < 5; i++) {
		uint32_t u = sys_get_le32(b + 4 * i);

		memcpy(&f[i], &u, 4);
		if (!(f[i] == f[i])) {      /* NaN */
			return false;
		}
	}
	/* Plausibilität: positive Werte, Exponent in sinnvollem Bereich */
	if (f[0] <= 0 || f[1] <= 0 || f[2] < 0 || f[3] <= 0 || f[4] < 0.1f || f[4] > 10.0f) {
		return false;
	}
	*c = (struct fsr_cal){ f[0], f[1], f[2], f[3], f[4], sys_get_le32(b + 20) };
	return true;
}

/* ---------------- Ausrichtung (im Flash) ---------------- */

#define FRAME_LEN 40
static struct gait_frame frame;           /* stamp 0 = automatisch */
static struct k_spinlock frame_lock;
static atomic_t frame_dirty;              /* neue Ausrichtung für die Schritterkennung */

static void frame_pack(const struct gait_frame *f, uint8_t *b)
{
	for (int i = 0; i < 9; i++) {
		uint32_t u;

		memcpy(&u, &f->r[i / 3][i % 3], 4);
		sys_put_le32(u, b + 4 * i);
	}
	sys_put_le32(f->stamp, b + 36);
}

static bool frame_unpack(const uint8_t *b, struct gait_frame *f)
{
	for (int i = 0; i < 9; i++) {
		uint32_t u = sys_get_le32(b + 4 * i);

		memcpy(&f->r[i / 3][i % 3], &u, 4);
	}
	f->stamp = sys_get_le32(b + 36);
	return f->stamp == 0 || gait_frame_valid(f);
}

static int frame_settings_set(size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t b[FRAME_LEN];
	struct gait_frame f;

	if (len == FRAME_LEN && read_cb(cb_arg, b, FRAME_LEN) == FRAME_LEN && frame_unpack(b, &f)) {
		frame = f;
		atomic_set(&frame_dirty, 1);
	}
	return 0;
}

static int cal_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t b[CAL_LEN];

	if (!strcmp(name, "frame")) {
		return frame_settings_set(len, read_cb, cb_arg);
	}
	if (!strcmp(name, "led")) {
		uint8_t l[4];

		if (len == 4 && read_cb(cb_arg, l, 4) == 4 && l[0] <= 100 && l[1] <= 100 &&
		    sys_get_le16(l + 2) <= LED_FLASH_MAX_MS) {
			atomic_set(&led_dim, l[0]);
			atomic_set(&led_bright, l[1]);
			atomic_set(&led_flash_ms, sys_get_le16(l + 2));
		}
		return 0;
	}
	if (!strcmp(name, "pk")) {
		uint8_t k[4];

		if (len == 4 && read_cb(cb_arg, k, 4) == 4) {
			atomic_set(&passkey, (atomic_val_t)sys_get_le32(k));
		}
		return 0;
	}
	if (!strcmp(name, "outputs")) {
		uint8_t m[2];

		if (len == 2 && read_cb(cb_arg, m, 2) == 2) {
			atomic_set(&outputs, sys_get_le16(m) & OUT_ALL);
		}
		return 0;
	}
	if (strcmp(name, "cal") || len != CAL_LEN || read_cb(cb_arg, b, CAL_LEN) != CAL_LEN) {
		return 0;
	}
	if (!cal_unpack(b, &cal)) {
		printk("# Kalibrierung im Flash ungültig, nehme Werkswerte\n");
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(feo, "feo", NULL, cal_settings_set, NULL, NULL);

static void cal_print(const char *what)
{
	printk("# Kalibrierung %s: VREF=%.1f mV RF=%.0f RS=%.1f K=%.1f N=%.4f (%s)\n", what,
	       (double)cal.vref_mv, (double)cal.rf_ohm, (double)cal.rs_ohm, (double)cal.k,
	       (double)cal.n, cal.stamp ? "gespeichert" : "Werkswert");
}

K_SEM_DEFINE(wake_sem, 0, 1);              /* weckt die Hauptschleife im Ruhemodus */
static struct bt_conn *cur_conn;
static atomic_t notify_on;
static atomic_t step_notify_on;
static atomic_t step_resend = ATOMIC_INIT(-1);   /* Datensätze ab dieser seq noch einmal senden */
static uint32_t gait_axis_bias_count[7];         /* Achse, Offset (float-Bits), Anzahl */
static atomic_t bat_notify_on;
static uint16_t bat_mv;           /* letzte Akkuspannung (mV), 0 = noch keine Messung */
static atomic_t need_adv = ATOMIC_INIT(1);
static atomic_t ctrl_notify_on;
static atomic_t log_notify_on;

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&notify_on, value == BT_GATT_CCC_NOTIFY);
	k_sem_give(&wake_sem);                 /* Streaming angefordert -> aus dem Ruhemodus */
}

static void step_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&step_notify_on, value == BT_GATT_CCC_NOTIFY);
	k_sem_give(&wake_sem);
}

static ssize_t frame_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
	uint8_t b[FRAME_LEN + 28];
	k_spinlock_key_t key = k_spin_lock(&frame_lock);

	frame_pack(&frame, b);
	for (int i = 0; i < 7; i++) {
		sys_put_le32(gait_axis_bias_count[i], b + FRAME_LEN + 4 * i);
	}
	k_spin_unlock(&frame_lock, key);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, b, sizeof(b));
}

static ssize_t frame_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct gait_frame f;

	if (offset != 0 || len != FRAME_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (!frame_unpack(buf, &f)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	k_spinlock_key_t key = k_spin_lock(&frame_lock);

	frame = f;
	k_spin_unlock(&frame_lock, key);
	atomic_set(&frame_dirty, 1);
	int err = settings_save_one("feo/frame", buf, FRAME_LEN);

	printk("# Ausrichtung %s%s\n", f.stamp ? "kalibriert" : "automatisch",
	       err ? ", Speichern FEHLGESCHLAGEN" : ", gespeichert");
	return err ? BT_GATT_ERR(BT_ATT_ERR_UNLIKELY) : len;
}

static ssize_t step_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != 4) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	atomic_set(&step_resend, (atomic_val_t)MIN(sys_get_le32(buf), (uint32_t)INT32_MAX));
	k_sem_give(&wake_sem);
	return len;
}

static void bat_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&bat_notify_on, value == BT_GATT_CCC_NOTIFY);
}

static ssize_t bat_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint8_t v[2];

	sys_put_le16(bat_mv, v);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, v, sizeof(v));
}

static ssize_t cal_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint8_t b[CAL_LEN + 4];
	uint32_t u;

	cal_pack(&cal, b);
	memcpy(&u, &vref_zero, 4);
	sys_put_le32(u, b + CAL_LEN);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, b, sizeof(b));
}

static ssize_t cal_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct fsr_cal c;

	if (offset != 0 || len != CAL_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (!cal_unpack(buf, &c)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	cal = c;
	int err = settings_save_one("feo/cal", buf, CAL_LEN);

	cal_print(err ? "übernommen, Speichern FEHLGESCHLAGEN" : "gespeichert");
	return err ? BT_GATT_ERR(BT_ATT_ERR_UNLIKELY) : len;
}

static void ctrl_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&ctrl_notify_on, value == BT_GATT_CCC_NOTIFY);
}

static void log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&log_notify_on, value == BT_GATT_CCC_NOTIFY);
}

static void led_set(bool on);

/* ---------------- Pairing code (passkey) ----------------
 * Without a passkey everything is open (as before). With a passkey, the FEO and
 * MCUmgr (SMP) services need an authenticated (paired with the code) link: the
 * board asks for pairing on connect, the phone or computer shows its pairing
 * dialog, and until then reads and writes are refused. Bonds are stored, so
 * a paired phone does not ask again. Reset: hold the user button at power-up. */
static bool pk_set(void)
{
	return (uint32_t)atomic_get(&passkey) != PK_NONE;
}

static void passkey_store(uint32_t k)
{
	uint8_t v[4];

	atomic_set(&passkey, (atomic_val_t)k);
	sys_put_le32(k, v);
	settings_save_one("feo/pk", v, 4);
}

static void unpair_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	bt_unpair(BT_ID_DEFAULT, NULL);
}
static K_WORK_DEFINE(unpair_work, unpair_handler);

static uint32_t app_passkey(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	return pk_set() ? (uint32_t)atomic_get(&passkey) : BT_PASSKEY_RAND;
}

static void passkey_display(struct bt_conn *conn, unsigned int key)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(key);   /* no display: the user knows the code */
}

static void auth_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
}

static struct bt_conn_auth_cb auth_cb = {
	.app_passkey = app_passkey,
	.passkey_display = passkey_display,
	.cancel = auth_cancel,
};

static bool link_ok(struct bt_conn *conn)
{
	return !pk_set() || bt_conn_get_security(conn) >= BT_SECURITY_L3;
}

static void pair_timeout_handler(struct k_work *w)
{
	ARG_UNUSED(w);
	if (cur_conn && !link_ok(cur_conn)) {
		bt_conn_disconnect(cur_conn, BT_HCI_ERR_AUTH_FAIL);
	}
}
static K_WORK_DELAYABLE_DEFINE(pair_timeout, pair_timeout_handler);

/* user button held at power-up: clear the passkey and all bonds (LED fast blink) */
static void passkey_reset_check(void)
{
	if (!gpio_is_ready_dt(&btn) || gpio_pin_configure_dt(&btn, GPIO_INPUT) ||
	    gpio_pin_get_dt(&btn) <= 0) {
		return;
	}
	for (int t = 0; t < PK_RESET_MS; t += 100) {
		if (gpio_pin_get_dt(&btn) <= 0) {
			return;
		}
		led_set(t % 1000 < 100);
		k_msleep(100);
	}
	passkey_store(PK_NONE);
	bt_unpair(BT_ID_DEFAULT, NULL);
	for (int i = 0; i < 10; i++) {
		led_set(i & 1);
		k_msleep(80);
	}
	led_set(false);
	printk("# Pairing code and bonds cleared\n");
}

/* Status: dl_status() (output mask at offset 2) + uint32 strides since power-up,
 * uint8 activity (0 rest mode, 1 still, 2 moving), uint8 0, uint16 seconds still */
static void ctrl_status(uint8_t b[CTRL_STATUS_LEN])
{
	dl_status(b);
	sys_put_le16((uint16_t)atomic_get(&outputs), b + 2);
	sys_put_le32((uint32_t)atomic_get(&stride_total), b + DL_STATUS_LEN);
	b[DL_STATUS_LEN + 4] = (uint8_t)atomic_get(&activity);
	b[DL_STATUS_LEN + 5] = (pk_set() ? 1 : 0) |
		(cur_conn && bt_conn_get_security(cur_conn) >= BT_SECURITY_L3 ? 2 : 0);
	sys_put_le16((uint16_t)MIN(atomic_get(&still_secs), UINT16_MAX), b + DL_STATUS_LEN + 6);
	b[DL_STATUS_LEN + 8] = (uint8_t)atomic_get(&led_dim);
	b[DL_STATUS_LEN + 9] = (uint8_t)atomic_get(&led_bright);
	sys_put_le16((uint16_t)atomic_get(&led_flash_ms), b + DL_STATUS_LEN + 10);
}

static ssize_t ctrl_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t b[CTRL_STATUS_LEN];

	ctrl_status(b);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, b, sizeof(b));
}

static ssize_t ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *b = buf;

	if (offset != 0 || len < 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (b[0] == CTRL_OUTPUTS) {
		if (len != 3) {
			return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
		}
		uint8_t m[2];

		sys_put_le16(sys_get_le16(b + 1) & OUT_ALL, m);
		atomic_set(&outputs, sys_get_le16(m));
		settings_save_one("feo/outputs", m, 2);
	} else if (b[0] == CTRL_LED) {
		if (len != 5 || b[1] > 100 || b[2] > 100 || sys_get_le16(b + 3) > LED_FLASH_MAX_MS) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
		atomic_set(&led_dim, b[1]);
		atomic_set(&led_bright, b[2]);
		atomic_set(&led_flash_ms, sys_get_le16(b + 3));
		settings_save_one("feo/led", b + 1, 4);
	} else if (b[0] == CTRL_PASSKEY) {
		if (len != 5) {
			return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
		}
		uint32_t k = sys_get_le32(b + 1);

		if (k != PK_NONE && k > 999999) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
		passkey_store(k);
		/* old bonds were made with the old code: drop them (a bonded link
		 * is disconnected and pairs again with the new code) */
		k_work_submit(&unpair_work);
	} else if (dl_command(b, len)) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	k_sem_give(&wake_sem);
	return len;
}

/* Attribute: [0] Service, [1] Daten-Decl, [2] Daten-Wert, [3] CCC,
 *            [4] Akku-Decl, [5] Akku-Wert, [6] CCC, [7] Kal-Decl, [8] Kal-Wert,
 *            [9] Ausrichtung-Decl, [10] Ausrichtung-Wert,
 *            [11] Schritt-Decl, [12] Schritt-Wert, [13] CCC,
 *            [14] control decl, [15] control value, [16] CCC,
 *            [17] log decl, [18] log value, [19] CCC */
BT_GATT_SERVICE_DEFINE(feo_svc,
	BT_GATT_PRIMARY_SERVICE(&feo_svc_uuid),
	BT_GATT_CHARACTERISTIC(&feo_data_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&feo_bat_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, bat_read, NULL, NULL),
	BT_GATT_CCC(bat_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&feo_cal_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, cal_read, cal_write, NULL),
	BT_GATT_CHARACTERISTIC(&feo_frame_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, frame_read, frame_write, NULL),
	BT_GATT_CHARACTERISTIC(&feo_step_uuid.uuid, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, step_write, NULL),
	BT_GATT_CCC(step_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&feo_ctrl_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, ctrl_read, ctrl_write, NULL),
	BT_GATT_CCC(ctrl_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&feo_log_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* GAP/GATT services and all declarations stay readable (discovery); the FEO
 * service and SMP come after them in the attribute table */
static bool attr_open(const struct bt_gatt_attr *attr)
{
	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_INCLUDE)) {
		return true;
	}
	return bt_gatt_attr_get_handle(attr) < bt_gatt_attr_get_handle(&feo_svc.attrs[0]);
}

static bool authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	return attr_open(attr) || link_ok(conn);
}

static const struct bt_gatt_authorization_cb authz_cb = {
	.read_authorize = authorize,
	.write_authorize = authorize,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, FEO_SVC_UUID_VAL),
};
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("# BLE: Verbindung fehlgeschlagen (0x%02x)\n", err);
		atomic_set(&need_adv, 1);
		return;
	}
	cur_conn = bt_conn_ref(conn);
	printk("# BLE: verbunden\n");
	if (pk_set()) {
		/* ask the central to pair (passkey entry); refused attributes until then */
		bt_conn_set_security(conn, BT_SECURITY_L3);
		k_work_reschedule(&pair_timeout, K_SECONDS(PK_PAIR_S));
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("# BLE: getrennt (0x%02x)\n", reason);
	atomic_set(&notify_on, 0);
	atomic_set(&bat_notify_on, 0);
	atomic_set(&step_notify_on, 0);
	atomic_set(&step_resend, -1);
	atomic_set(&ctrl_notify_on, 0);
	atomic_set(&log_notify_on, 0);
	const uint8_t abort_rd[5] = { DL_C_READ, 0xff, 0xff, 0xff, 0xff };

	dl_command(abort_rd, sizeof(abort_rd));
	k_work_cancel_delayable(&pair_timeout);
	if (cur_conn) {
		bt_conn_unref(cur_conn);
		cur_conn = NULL;
	}
}

static void recycled(void)
{
	atomic_set(&need_adv, 1);   /* Verbindung freigegeben -> wieder werben */
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
};

/* Werben: nach Start oder Trennen ADV_FAST_S lang alle 100..150 ms (schnell
 * gefunden), danach alle 1..1,2 s (spart Strom, Verbinden dauert etwas länger) */
static int64_t adv_fast_until;    /* 0 = wirbt langsam oder gar nicht */

static void adv_start_if_needed(void)
{
	int64_t now = k_uptime_get();

	if (adv_fast_until && now > adv_fast_until && !cur_conn) {
		adv_fast_until = 0;
		bt_le_adv_stop();
		int err = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, BT_GAP_ADV_SLOW_INT_MIN,
							  BT_GAP_ADV_SLOW_INT_MAX, NULL),
					  ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			atomic_set(&need_adv, 1);
		}
		return;
	}
	if (!atomic_cas(&need_adv, 1, 0)) {
		return;
	}
	int err = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, BT_GAP_ADV_FAST_INT_MIN_2,
						  BT_GAP_ADV_FAST_INT_MAX_2, NULL),
				  ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		printk("# BLE: Advertising-Fehler %d\n", err);
		atomic_set(&need_adv, 1);   /* beim nächsten Durchlauf erneut */
	} else {
		adv_fast_until = now + ADV_FAST_S * 1000;
		printk("# BLE: sichtbar als \"%s\"\n", CONFIG_BT_DEVICE_NAME);
	}
}

/* Messungen sammeln und als ein Paket verschicken */
static uint8_t pkt[PKT_HDR + BLE_BATCH * PKT_SAMPLE];
static uint8_t pkt_n;

static void ble_push(uint32_t t_ms, const int16_t v[6], uint16_t fsr_word)
{
	if (!cur_conn || !atomic_get(&notify_on) || !(atomic_get(&outputs) & OUT_BLE_RAW)) {
		pkt_n = 0;
		return;
	}
	/* Wie viele Messungen passen bei der ausgehandelten MTU in ein Paket? */
	uint16_t room = bt_gatt_get_mtu(cur_conn) - 3;
	uint8_t max_n = MIN(BLE_BATCH, (room - PKT_HDR) / PKT_SAMPLE);

	if (max_n == 0) {
		return;
	}
	if (pkt_n == 0) {
		sys_put_le32(t_ms, &pkt[0]);
	}
	uint8_t *p = &pkt[PKT_HDR + pkt_n * PKT_SAMPLE];

	for (int i = 0; i < 6; i++) {
		sys_put_le16((uint16_t)v[i], p + 2 * i);
	}
	sys_put_le16(fsr_word, p + 12);
	if (++pkt_n >= max_n) {
		pkt[4] = pkt_n;
		int err = bt_gatt_notify(cur_conn, &feo_svc.attrs[1], pkt,
					 PKT_HDR + pkt_n * PKT_SAMPLE);
		if (err && err != -ENOMEM) {
			printk("# BLE: notify-Fehler %d\n", err);
		}
		pkt_n = 0;
	}
}

/* ---------------- Schritte ---------------- */

static uint8_t step_log[STEP_LOG][STEP_LEN];
static uint32_t step_last;        /* seq des neuesten Datensatzes, 0 = keiner */

static int16_t i16(float x, float scale)
{
	return (int16_t)CLAMP(lroundf(x * scale), INT16_MIN, INT16_MAX);
}

static uint16_t u16(float x, float scale)
{
	return (uint16_t)CLAMP(lroundf(x * scale), 0, UINT16_MAX);
}

static void step_store(const struct gait_stride *s)
{
	uint8_t *b = step_log[s->seq % STEP_LOG];

	sys_put_le32(s->seq, b);
	sys_put_le32(s->t_ic_ms, b + 4);
	sys_put_le16(s->stride_ms, b + 8);
	sys_put_le16(s->stance_ms, b + 10);
	sys_put_le16(i16(s->ms_dps, 10), b + 12);
	sys_put_le16(i16(s->ic_dps, 10), b + 14);
	sys_put_le16(i16(s->to_dps, 10), b + 16);
	sys_put_le16(u16(s->f_peak_kg, 100), b + 18);
	sys_put_le16(u16(s->f_mean_kg, 100), b + 20);
	sys_put_le16(u16(s->f_imp_kgs, 100), b + 22);
	sys_put_le16(s->t_fpeak_ms, b + 24);
	sys_put_le16(i16(s->f_swing_kg, 100), b + 26);
	sys_put_le16(s->flags, b + 28);
	for (int i = 0; i < GAIT_BINS; i++) {
		sys_put_le16(u16(s->f_bins[i], 100), b + 30 + 2 * i);
	}
	for (int i = 0; i < GAIT_PHASES; i++) {
		sys_put_le16(u16(s->f_ph_mean[i], 100), b + 30 + 2 * GAIT_BINS + 2 * i);
		sys_put_le16(u16(s->f_ph_peak[i], 100), b + 30 + 2 * GAIT_BINS + 2 * GAIT_PHASES + 2 * i);
	}
	step_last = s->seq;
}

static void step_print(const struct gait_stride *s)
{
	if (!IS_ENABLED(CONFIG_PRINTK) || !(atomic_get(&outputs) & OUT_SER_STEP)) {
		return;
	}
	char line[100 + 7 * (GAIT_BINS + 2 * GAIT_PHASES)];
	int n = snprintk(line, sizeof(line), "#S,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%u,%.2f,%u",
			 s->seq, s->t_ic_ms, s->stride_ms, s->stance_ms, (double)s->ms_dps,
			 (double)s->ic_dps, (double)s->to_dps, (double)s->f_peak_kg,
			 (double)s->f_mean_kg, (double)s->f_imp_kgs, s->t_fpeak_ms,
			 (double)s->f_swing_kg, s->flags);

	for (int i = 0; i < GAIT_BINS && n > 0 && n < (int)sizeof(line); i++) {
		n += snprintk(line + n, sizeof(line) - n, ",%.2f", (double)s->f_bins[i]);
	}
	for (int i = 0; i < 2 * GAIT_PHASES && n > 0 && n < (int)sizeof(line); i++) {
		n += snprintk(line + n, sizeof(line) - n, ",%.2f", (double)(i < GAIT_PHASES ?
			      s->f_ph_mean[i] : s->f_ph_peak[i - GAIT_PHASES]));
	}
	printk("%s\n", line);
}

static bool step_notify(uint32_t seq)
{
	return bt_gatt_notify(cur_conn, &feo_svc.attrs[11], step_log[seq % STEP_LOG], STEP_LEN) == 0;
}

/* neue oder angeforderte ältere Datensätze verschicken (ein paar pro Durchlauf) */
static void step_send_pending(void)
{
	if (!cur_conn || !atomic_get(&step_notify_on) || !(atomic_get(&outputs) & OUT_BLE_STEP)) {
		return;
	}
	atomic_val_t from = atomic_get(&step_resend);

	if (from < 0) {
		return;
	}
	uint32_t seq = MAX((uint32_t)from, step_last >= STEP_LOG ? step_last - STEP_LOG + 1 : 1);

	for (int i = 0; i < 6 && seq <= step_last; i++, seq++) {
		if (!step_notify(seq)) {
			break;              /* Puffer voll: später weiter */
		}
	}
	atomic_cas(&step_resend, from, seq <= step_last ? (atomic_val_t)seq : -1);
}

static void ble_push_bat(void)
{
	if (!cur_conn || !atomic_get(&bat_notify_on)) {
		return;
	}
	uint8_t v[2];

	sys_put_le16(bat_mv, v);
	bt_gatt_notify(cur_conn, &feo_svc.attrs[5], v, sizeof(v));
}

/* status/activity notification (about once per second, always on while connected) */
static void ble_push_status(void)
{
	if (!cur_conn || !atomic_get(&ctrl_notify_on)) {
		return;
	}
	uint8_t b[CTRL_STATUS_LEN];

	ctrl_status(b);
	bt_gatt_notify(cur_conn, &feo_svc.attrs[15], b, sizeof(b));
}

/* log readout: a few packets per call, as many as the buffers take */
static void log_send_pending(void)
{
	static uint8_t b[4 + 15 * DL_ENTRY];

	for (int i = 0; i < 8; i++) {
		if (!cur_conn || !atomic_get(&log_notify_on)) {
			return;
		}
		size_t room = MIN(sizeof(b), (size_t)bt_gatt_get_mtu(cur_conn) - 3);
		size_t n = dl_read_fill(b, room);

		if (n == 0 || bt_gatt_notify(cur_conn, &feo_svc.attrs[18], b, n)) {
			return;
		}
		dl_read_done();
		if (n == 4) {
			return;             /* end marker sent */
		}
	}
}

/* main-loop housekeeping for the log; returns true if it needs to run again soon */
static bool log_service(void)
{
	static uint8_t last_flags, last_pct;
	static uint32_t last_count;
	static int64_t last_push;
	bool busy = dl_poll();
	uint8_t st[DL_STATUS_LEN];

	log_send_pending();
	/* tell the app right away when the log state changes (start, stop, erase
	 * progress), also in rest mode where the status is otherwise sent rarely */
	dl_status(st);
	uint32_t count = sys_get_le32(st + 4);
	int64_t now = k_uptime_get();

	if (st[1] != last_flags || ((st[19] != last_pct || count != last_count) && now - last_push >= 250)) {
		last_flags = st[1];
		last_pct = st[19];
		last_count = count;
		last_push = now;
		ble_push_status();
	}
	return busy;
}

/* ---------------- ADC: Kraftsensor und Akku ---------------- */

/* Messbereiche des FSR-Kanals (interne Referenz 0,9 V) mit Umschaltschwellen */
static const enum adc_gain fsr_gain[] = { ADC_GAIN_1, ADC_GAIN_1_2, ADC_GAIN_1_4 };
static const uint16_t fsr_range_mv[] = { 900, 1800, 3600 };
static const int32_t fsr_up_mv[] = { 800, 1600, INT32_MAX };   /* größeren Bereich wählen über */
static const int32_t fsr_down_mv[] = { INT32_MIN, 350, 700 };  /* kleineren Bereich wählen unter */
#define FSR_START_RANGE 2       /* mit dem größten Bereich anfangen */

static struct adc_channel_cfg fsr_cfg;
static uint8_t fsr_idx = FSR_START_RANGE;
static bool adc_ok;

/* Einen Kanal einmal wandeln, Ergebnis in mV (am ADC-Pin) */
static int adc_read_mv(const struct adc_dt_spec *spec, enum adc_gain gain, int32_t *mv)
{
	int16_t raw;
	struct adc_sequence seq = {
		.channels = BIT(spec->channel_id),
		.buffer = &raw,
		.buffer_size = sizeof(raw),
		.resolution = spec->resolution,
		.oversampling = spec->oversampling,
	};
	int err = adc_read_dt(spec, &seq);

	if (err) {
		return err;
	}
	*mv = MAX(raw, 0);   /* single-ended: kleine negative Werte um 0 V abschneiden */
	return adc_raw_to_millivolts(adc_ref_internal(spec->dev), gain, spec->resolution, mv);
}

static int fsr_set_range(uint8_t idx)
{
	fsr_cfg.gain = fsr_gain[idx];
	fsr_idx = idx;
	return adc_channel_setup(adc_fsr.dev, &fsr_cfg);
}

/* FSR-Spannung messen, dabei den Messbereich automatisch anpassen.
 * Gibt das Paketwort zurück: Bit 0..13 mV, Bit 14..15 Messbereich. */
static uint16_t fsr_measure(int32_t *mv_out)
{
	int32_t mv = 0;
	bool discard = false;

	*mv_out = 0;
	if (!adc_ok) {
		return 0;
	}
	/* höchstens zwei Wechsel (je Wechsel eine verworfene Messung) */
	for (int tries = 0; tries < 6; tries++) {
		if (adc_read_mv(&adc_fsr, fsr_gain[fsr_idx], &mv)) {
			return 0;
		}
		if (discard) {          /* erste Messung nach dem Umschalten verwerfen */
			discard = false;
			continue;
		}
		uint8_t idx = fsr_idx;

		if (mv > fsr_up_mv[idx] && idx < ARRAY_SIZE(fsr_gain) - 1) {
			idx++;
		} else if (mv < fsr_down_mv[idx] && idx > 0) {
			idx--;
		}
		if (idx == fsr_idx) {
			break;
		}
		if (fsr_set_range(idx)) {
			adc_ok = false;
			return 0;
		}
		discard = true;
	}
	mv = CLAMP(mv, 0, 0x3fff);
	*mv_out = mv;
	return (uint16_t)mv | ((uint16_t)fsr_idx << 14);
}

static void bat_measure(void)
{
	int32_t mv;

	if (adc_ok && adc_read_mv(&adc_bat, adc_bat.channel_cfg.gain, &mv) == 0) {
		bat_mv = (uint16_t)CLAMP(mv * BAT_DIVIDER, 1, UINT16_MAX);
	}
}

static int sensors_analog_init(void)
{
	/* MCP602 einschalten */
	if (!gpio_is_ready_dt(&fsr_pwr) ||
	    gpio_pin_configure_dt(&fsr_pwr, GPIO_OUTPUT_ACTIVE)) {
		printk("# Fehler: D1 (FSR-Versorgung) nicht verfügbar\n");
	}
	if (!adc_is_ready_dt(&adc_fsr) || !adc_is_ready_dt(&adc_bat)) {
		printk("# Fehler: ADC nicht bereit\n");
		return -ENODEV;
	}
	fsr_cfg = adc_fsr.channel_cfg;
	if (fsr_set_range(FSR_START_RANGE) || adc_channel_setup_dt(&adc_bat)) {
		printk("# Fehler: ADC-Kanal-Setup\n");
		return -EIO;
	}
	if (!device_is_ready(vbat_sw)) {
		printk("# Fehler: Akku-Messschalter nicht bereit\n");
	}
	adc_ok = true;
	return 0;
}

/* Kraft in kg aus der FSR-Spannung (wie in der App), INFINITY = Überlast */
static float fsr_kg(int32_t mv)
{
	const float vr = vref_zero > 0 ? vref_zero : cal.vref_mv;

	if (!((float)mv > vr + 2.0f)) {           /* ~2 mV Totband gegen Rauschen */
		return 0;
	}
	const float den = vr * cal.rf_ohm / ((float)mv - vr) - cal.rs_ohm;

	return den > 0 ? powf(cal.k / den, 1.0f / cal.n) : INFINITY;
}

/* ---------------- IMU ---------------- */

/* serielle Zeilen sammeln und als Schub ausgeben */
static char ser_buf[SERIAL_BATCH * 96];
static size_t ser_len;
static uint8_t ser_n;

static void serial_push(uint32_t t, const double a[3], const double g[3],
			int32_t fsr_mv, uint16_t range_mv)
{
	if (!IS_ENABLED(CONFIG_PRINTK) || !(atomic_get(&outputs) & OUT_SER_CSV)) {
		ser_len = 0;
		ser_n = 0;
		return;
	}
	int len = snprintk(&ser_buf[ser_len], sizeof(ser_buf) - ser_len,
			   "%u,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%u,%u\n", t,
			   a[0], a[1], a[2], g[0], g[1], g[2],
			   fsr_mv, range_mv, bat_mv);

	if (len > 0 && ser_len + len < sizeof(ser_buf)) {
		ser_len += len;
	}
	if (++ser_n >= SERIAL_BATCH) {
		printk("%s", ser_buf);
		ser_len = 0;
		ser_n = 0;
		ser_buf[0] = '\0';
	}
}

static int imu_configure(void)
{
	struct sensor_value v;
	int ret;

	/* Messbereiche (Treiber erwartet m/s^2 bzw. rad/s) */
	sensor_g_to_ms2(ACCEL_RANGE_G, &v);
	ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	if (ret) {
		printk("# Fehler: Accel-Messbereich (%d)\n", ret);
		return ret;
	}
	sensor_degrees_to_rad(GYRO_RANGE_DPS, &v);
	ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	if (ret) {
		printk("# Fehler: Gyro-Messbereich (%d)\n", ret);
		return ret;
	}

	/* Messrate: erst damit schaltet der Sensor ein */
	v.val1 = IMU_ODR_HZ;
	v.val2 = 0;
	ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (ret) {
		printk("# Fehler: Accel-Messrate (%d)\n", ret);
		return ret;
	}
	ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (ret) {
		printk("# Fehler: Gyro-Messrate (%d)\n", ret);
		return ret;
	}
	return 0;
}

/* ---------------- Ruhemodus (Aktivitätserkennung der IMU) ---------------- */

/* Register des LSM6DS3TR-C, die der Zephyr-Treiber nicht anbietet */
#define LSM_CTRL6_C      0x15
#define LSM_XL_HM_MODE   BIT(4)     /* 1 = Beschleunigungssensor im Low-Power-Modus */
#define LSM_WAKE_UP_SRC  0x1B
#define LSM_WU_IA        BIT(3)
#define LSM_TAP_CFG      0x58
#define LSM_WAKE_UP_THS  0x5B
#define LSM_WAKE_UP_DUR  0x5C
#define LSM_MD1_CFG      0x5E
#define LSM_INT1_WU      BIT(5)
#define WAKE_THS         2          /* x FS/64 = 2 x 31 mg bei ±2 g */

static struct gpio_callback imu_int_cb;
static atomic_t imu_woke;

static void imu_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	atomic_set(&imu_woke, 1);
	k_sem_give(&wake_sem);
}

static int imu_set(enum sensor_channel ch, enum sensor_attribute attr, int32_t v1)
{
	struct sensor_value v = { .val1 = v1 };

	return sensor_attr_set(imu, ch, attr, &v);
}

/* Gyro aus, Beschleunigung 26 Hz Low-Power, ±2 g, Aufwach-Interrupt an INT1 */
static int imu_sleep(void)
{
	struct sensor_value v;
	uint8_t src;
	int err = imu_set(SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, 0);

	sensor_g_to_ms2(2, &v);
	err |= sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	err |= imu_set(SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, 26);
	err |= i2c_reg_update_byte_dt(&imu_i2c, LSM_CTRL6_C, LSM_XL_HM_MODE, LSM_XL_HM_MODE);
	err |= i2c_reg_write_byte_dt(&imu_i2c, LSM_WAKE_UP_DUR, 0);
	err |= i2c_reg_write_byte_dt(&imu_i2c, LSM_WAKE_UP_THS, WAKE_THS);
	err |= i2c_reg_write_byte_dt(&imu_i2c, LSM_TAP_CFG, 0x81);   /* Interrupts an, gelatcht */
	k_msleep(100);              /* Filter einschwingen lassen, sonst sofortiges Aufwachen */
	err |= i2c_reg_read_byte_dt(&imu_i2c, LSM_WAKE_UP_SRC, &src);
	atomic_set(&imu_woke, 0);
	err |= i2c_reg_write_byte_dt(&imu_i2c, LSM_MD1_CFG, LSM_INT1_WU);
	err |= gpio_pin_interrupt_configure_dt(&imu_int, GPIO_INT_EDGE_TO_ACTIVE);
	if (gpio_pin_get_dt(&imu_int) > 0) {
		atomic_set(&imu_woke, 1);
	}
	return err;
}

static void imu_wake(void)
{
	uint8_t src;

	gpio_pin_interrupt_configure_dt(&imu_int, GPIO_INT_DISABLE);
	i2c_reg_write_byte_dt(&imu_i2c, LSM_MD1_CFG, 0);
	i2c_reg_write_byte_dt(&imu_i2c, LSM_TAP_CFG, 0);
	i2c_reg_read_byte_dt(&imu_i2c, LSM_WAKE_UP_SRC, &src);
	i2c_reg_update_byte_dt(&imu_i2c, LSM_CTRL6_C, LSM_XL_HM_MODE, 0);
	imu_configure();
}

static bool rest_ok;              /* Aufwach-Interrupt verfügbar */

static void rest_init(void)
{
	rest_ok = device_is_ready(imu_i2c.bus) && gpio_is_ready_dt(&imu_int) &&
		  gpio_pin_configure_dt(&imu_int, GPIO_INPUT) == 0;
	if (rest_ok) {
		gpio_init_callback(&imu_int_cb, imu_int_handler, BIT(imu_int.pin));
		rest_ok = gpio_add_callback(imu_int.port, &imu_int_cb) == 0;
	}
	if (!rest_ok) {
		printk("# Warnung: kein IMU-Interrupt, Ruhemodus aus\n");
	}
}

/* software PWM: one timer toggles the pin, only while 0 < duty < 100 % */
static uint32_t led_on_us, led_off_us;
static bool led_phase;

static void led_pwm_fn(struct k_timer *t)
{
	led_phase = !led_phase;
	gpio_pin_set_dt(&led, led_phase);
	k_timer_start(t, K_USEC(led_phase ? led_on_us : led_off_us), K_NO_WAIT);
}
static K_TIMER_DEFINE(led_pwm, led_pwm_fn, NULL);

static void led_pct(uint8_t pct)
{
	static int8_t cur = -1;

	if (!gpio_is_ready_dt(&led) || pct == cur) {
		return;
	}
	cur = pct;
	k_timer_stop(&led_pwm);
	if (pct == 0 || pct >= 100) {
		led_phase = pct != 0;
		gpio_pin_set_dt(&led, led_phase);
		return;
	}
	led_on_us = LED_PWM_US * pct / 100;
	led_off_us = LED_PWM_US - led_on_us;
	led_phase = true;
	gpio_pin_set_dt(&led, 1);
	k_timer_start(&led_pwm, K_USEC(led_on_us), K_NO_WAIT);
}

static void led_set(bool on)
{
	led_pct(on ? 100 : 0);
}

static void bat_once(void)
{
	regulator_enable(vbat_sw);
	k_msleep(10);
	bat_measure();
	regulator_disable(vbat_sw);
	ble_push_bat();
}

static bool raw_requested(void)
{
	return atomic_get(&notify_on) && (atomic_get(&outputs) & OUT_BLE_RAW);
}

/* rest mode while logging: switch the amplifier on briefly and sample the heel load */
static void rest_fsr_sample(void)
{
	int32_t mv;

	gpio_pin_set_dt(&fsr_pwr, 1);
	k_msleep(50);                              /* amplifier and low-pass settle */
	fsr_measure(&mv);
	gpio_pin_set_dt(&fsr_pwr, 0);
	dl_rest_sample(adc_ok ? fsr_kg(mv) : 0, REST_FSR_S, bat_mv);
}

/* Schlafen, bis sich das Bein bewegt oder die App Rohdaten anfordert */
static void rest(void)
{
	printk("# Ruhemodus (%d s ohne Bewegung)\n", REST_AFTER_S);
	dl_event(DL_E_REST, 0, 0);
	atomic_set(&activity, 0);
	ble_push_status();
	led_set(false);
	gait_pause();
	pkt_n = 0;
	gpio_pin_set_dt(&fsr_pwr, 0);              /* MCP602 aus */
	if (imu_sleep()) {
		printk("# Fehler: IMU-Ruhemodus\n");
	}
	int64_t bat_next = k_uptime_get() + REST_BAT_S * 1000;
	int64_t fsr_next = k_uptime_get() + REST_FSR_S * 1000;

	while (!atomic_get(&imu_woke) && !raw_requested()) {
		adv_start_if_needed();
		step_send_pending();
		bool busy = log_service();
		int64_t now = k_uptime_get();

		if (now >= bat_next) {
			bat_once();
			bat_next = now + REST_BAT_S * 1000;
			ble_push_status();
		}
		if (now >= fsr_next) {
			if (dl_running()) {
				rest_fsr_sample();
				ble_push_status();
			}
			fsr_next = now + REST_FSR_S * 1000;
		}
		int64_t until = bat_next;

		if (dl_running()) {
			until = MIN(until, MIN(fsr_next, dl_minute_end()));
		}
		if (busy) {
			until = now + 10;                   /* erasing or reading out the log */
		}

		if (adv_fast_until && adv_fast_until + 10 < until) {
			until = adv_fast_until + 10;
		}
		if (atomic_get(&step_resend) >= 0) {
			until = now + 20;                   /* Datensätze nachschicken */
		}
		k_sem_take(&wake_sem, K_TIMEOUT_ABS_MS(until));
	}
	imu_wake();
	gpio_pin_set_dt(&fsr_pwr, 1);              /* MCP602 an, ... */
	k_msleep(50);                              /* ... Verstärker und Tiefpass einschwingen */
	printk("# aktiv (%s)\n", raw_requested() ? "Streaming" : "Bewegung");
	dl_event(DL_E_WAKE, 0, 0);
	atomic_set(&activity, 2);
	atomic_set(&still_secs, 0);
}

#ifdef CONFIG_MCUMGR_GRP_IMG_STATUS_HOOKS
/* a firmware upload overwrites slot 1: stop logging before the first write */
static enum mgmt_cb_return dfu_cb(uint32_t event, enum mgmt_cb_return prev_status, int32_t *rc,
				  uint16_t *group, bool *abort_more, void *data, size_t data_size)
{
	dl_dfu_started();
	k_sem_give(&wake_sem);
	return MGMT_CB_OK;
}

static struct mgmt_callback dfu_mgmt_cb = {
	.callback = dfu_cb,
	.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STARTED,
};
#endif

int main(void)
{
	struct sensor_value acc[3], gyr[3];
	uint32_t n = 0;

	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	k_msleep(500); /* kurz warten, damit der Mac den Port geöffnet hat */

	if (!device_is_ready(imu)) {
		while (1) {
			printk("# Fehler: IMU nicht bereit\n");
			k_msleep(1000);
		}
	}
	if (imu_configure()) {
		return 0;
	}
	sensors_analog_init();
	gait_init();
	rest_init();

	if (settings_subsys_init() == 0) {
		settings_load_subtree("feo");   /* "bt" (bonds) after bt_enable() */
	} else {
		printk("# Fehler: Settings (Kalibrierung) nicht verfügbar\n");
	}
#ifdef CONFIG_MCUBOOT_IMG_MANAGER
	dl_allow(false);              /* no logging into slot 1 before the image is confirmed */
#endif
	dl_init();
#ifdef CONFIG_MCUMGR_GRP_IMG_STATUS_HOOKS
	mgmt_callback_register(&dfu_mgmt_cb);
#endif

	int err = bt_enable(NULL);

	if (!err) {
		settings_load_subtree("bt");     /* bonds */
		bt_conn_auth_cb_register(&auth_cb);
		bt_gatt_authorization_cb_register(&authz_cb);
		passkey_reset_check();
	}
	if (err) {
		printk("# Fehler: Bluetooth-Start (%d), nur seriell\n", err);
		atomic_set(&need_adv, 0);
	}

#ifdef CONFIG_MCUBOOT_IMG_MANAGER
	/* neues Image nach erfolgreichem Start dauerhaft übernehmen */
	if (!err && !boot_is_img_confirmed()) {
		int cerr = boot_write_img_confirmed();

		printk("# Update: neue Firmware %s\n", cerr ? "NICHT bestätigt" : "bestätigt");
	}
	dl_allow(boot_is_img_confirmed());
	printk("# Firmware %s (MCUboot, Update per Bluetooth möglich)\n",
	       CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION);
#endif
	printk("# XIAO nRF54L15 FEO-Sensor, %d Hz, +-%d g, +-%d dps, FSR + Akku, Schritterkennung\n",
	       SAMPLE_RATE_HZ, ACCEL_RANGE_G, GYRO_RANGE_DPS);
	cal_print("aktiv");
	printk("# t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,u_fsr_mv,fsr_range_mv,u_bat_mv\n");

	int64_t next = k_uptime_get();
	int64_t led_off = 0;          /* LED ausschalten zu diesem Zeitpunkt, 0 = ist aus */
	uint32_t still_s = 0;         /* Sekunden ohne Bewegung */
	bool moving = true;           /* Bewegung in der letzten Sekunde */
	float blk_max = 0;            /* größte Abweichung in der laufenden Sekunde */
	float sec_kg = 0;             /* max heel load in the current second */

	if (dl_running() && (atomic_get(&outputs) & OUT_SER_CSV)) {
		printk("# Hinweis: Dauermessung läuft, serielle CSV-Ausgabe kostet Strom\n");
	}

	while (1) {
		adv_start_if_needed();
		step_send_pending();
		log_service();

		/* neue Ausrichtung aus dem Flash oder von der App übernehmen */
		if (atomic_cas(&frame_dirty, 1, 0)) {
			k_spinlock_key_t key = k_spin_lock(&frame_lock);
			struct gait_frame f = frame;

			k_spin_unlock(&frame_lock, key);
			gait_set_frame(&f);
		}

		uint32_t t = (uint32_t)k_uptime_get();

		/* Akku: Teiler zuschalten, eine Messung (10 ms) später messen, wieder abschalten */
		if (n % BAT_PERIOD == 0) {
			regulator_enable(vbat_sw);
		} else if (n % BAT_PERIOD == 1) {
			bat_measure();
			regulator_disable(vbat_sw);
			ble_push_bat();
		}

		int32_t fsr_mv;
		uint16_t fsr_word = fsr_measure(&fsr_mv);

		/* Nullpunkt beim Einschalten: dann liegt sicher keine Last an */
		static int32_t zero_sum;

		if (n >= ZERO_START && n < ZERO_START + ZERO_LEN) {
			zero_sum += fsr_mv;
		} else if (n == ZERO_START + ZERO_LEN && adc_ok) {
			float z = (float)zero_sum / ZERO_LEN;

			bool ok = z >= ZERO_MIN_MV && z <= ZERO_MAX_MV;

			dl_event(DL_E_ZERO, ok ? (uint16_t)lroundf(z * 10.0f) : 0, 0);
			if (ok) {
				vref_zero = z;
				printk("# Nullpunkt beim Start: VREF_0 = %.1f mV\n", (double)z);
			} else {
				printk("# Warnung: Nullpunkt %.1f mV unplausibel (Last beim Start?), "
				       "nehme VREF aus der Kalibrierung\n", (double)z);
			}
		}

		if (sensor_sample_fetch(imu) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr) == 0) {
			double a[3], g[3];
			float af[3], gf[3];
			int16_t raw[6];
			struct gait_stride st;

			for (int i = 0; i < 3; i++) {
				a[i] = sensor_value_to_double(&acc[i]) / G_TO_MS2;
				g[i] = sensor_value_to_double(&gyr[i]) * RAD_TO_DEG;
				af[i] = (float)a[i];
				gf[i] = (float)g[i];
				raw[i] = (int16_t)CLAMP(a[i] * 1000.0, INT16_MIN, INT16_MAX);
				raw[3 + i] = (int16_t)CLAMP(g[i] * 10.0, INT16_MIN, INT16_MAX);
			}
			ble_push(t, raw, fsr_word);
			serial_push(t, a, g, fsr_mv, adc_ok ? fsr_range_mv[fsr_word >> 14] : 0);

			/* Bewegung: Drehrate oder Abweichung der Beschleunigung von 1 g */
			float ga = sqrtf(gf[0] * gf[0] + gf[1] * gf[1] + gf[2] * gf[2]);
			float aa = fabsf(sqrtf(af[0] * af[0] + af[1] * af[1] + af[2] * af[2]) - 1.0f);

			blk_max = fmaxf(blk_max, fmaxf(ga / 20.0f, aa / 0.1f));

			float kg = adc_ok ? fsr_kg(fsr_mv) : 0;

			sec_kg = fmaxf(sec_kg, kg);
			if (gait_update(t, gf, af, kg, &st)) {
				step_store(&st);
				step_print(&st);
				dl_stride((uint8_t)st.flags, st.stride_ms, st.t_ic_ms, st.stance_ms,
					  st.f_peak_kg, st.f_mean_kg, st.f_imp_kgs);
				atomic_inc(&stride_total);
				ble_push_status();         /* lets the app flash its activity badge */
				if (cur_conn && atomic_get(&step_notify_on) && atomic_get(&step_resend) < 0 &&
				    (atomic_get(&outputs) & OUT_BLE_STEP)) {
					step_notify(st.seq);
				}
				if (atomic_get(&led_bright) && atomic_get(&led_flash_ms)) {
					/* initial contact: bright flash */
					led_pct((uint8_t)atomic_get(&led_bright));
					led_off = k_uptime_get() + atomic_get(&led_flash_ms);
				}
			}
		} else {
			printk("# Lesefehler\n");
		}

		/* einmal pro Sekunde: Bewegung? Akku-Status für die App */
		if (++n % SAMPLE_RATE_HZ == 0) {
			moving = blk_max > 1.0f;
			blk_max = 0;
			still_s = moving ? 0 : still_s + 1;
			dl_second(moving, sec_kg, bat_mv);
			sec_kg = 0;
			atomic_set(&activity, moving ? 2 : 1);
			atomic_set(&still_secs, still_s);
			ble_push_status();
			float ax[3], bias[3];
			uint32_t cnt;

			gait_status(ax, bias, &cnt);
			k_spinlock_key_t key = k_spin_lock(&frame_lock);

			for (int i = 0; i < 3; i++) {
				memcpy(&gait_axis_bias_count[i], &ax[i], 4);
				memcpy(&gait_axis_bias_count[3 + i], &bias[i], 4);
			}
			gait_axis_bias_count[6] = cnt;
			k_spin_unlock(&frame_lock, key);
		}

		/* LED: dim while moving, bright flash after initial contact */
		int64_t now = k_uptime_get();

		if (led_off && now >= led_off) {
			led_off = 0;
		}
		if (!led_off) {
			/* dimly on while moving, otherwise off */
			led_pct(moving ? (uint8_t)atomic_get(&led_dim) : 0);
		}

		if (rest_ok && still_s >= REST_AFTER_S && !raw_requested()) {
			rest();
			still_s = 0;
			moving = true;
			next = k_uptime_get();
			continue;
		}

		/* festen Takt halten, unabhängig von der Laufzeit der Schleife */
		next += SAMPLE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
	return 0;
}

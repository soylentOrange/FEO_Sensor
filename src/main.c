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

#define SAMPLE_RATE_HZ   100   /* Ausgaberate */
#define IMU_ODR_HZ       208   /* interne Messrate des Sensors (>= 2x Ausgaberate) */
#define ACCEL_RANGE_G    16    /* +-2, 4, 8 oder 16 g; Fersenaufprall kann > 8 g sein */
#define GYRO_RANGE_DPS   2000  /* +-245, 500, 1000 oder 2000 Grad/s */
#define BLE_BATCH        5     /* Messungen pro BLE-Paket -> 20 Pakete/s */
#define SERIAL_BATCH     2     /* Zeilen pro seriellem Schub, siehe unten */
#define BAT_PERIOD       SAMPLE_RATE_HZ   /* Akku alle 100 Messungen = 1 s messen */
#define BAT_DIVIDER      2     /* Spannungsteiler 1:2 auf dem XIAO */

#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)
#define G_TO_MS2         9.80665
#define RAD_TO_DEG       57.29577951308232
#define PKT_HDR          5
#define PKT_SAMPLE       14

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));
static const struct gpio_dt_spec fsr_pwr = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fsr_pwr_gpios);
static const struct adc_dt_spec adc_fsr = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), fsr);
static const struct adc_dt_spec adc_bat = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vbat);
static const struct device *const vbat_sw = DEVICE_DT_GET(DT_NODELABEL(vbat_pwr));

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

static int cal_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	uint8_t b[CAL_LEN];

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

static struct bt_conn *cur_conn;
static atomic_t notify_on;
static atomic_t bat_notify_on;
static uint16_t bat_mv;           /* letzte Akkuspannung (mV), 0 = noch keine Messung */
static atomic_t need_adv = ATOMIC_INIT(1);

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&notify_on, value == BT_GATT_CCC_NOTIFY);
	printk("# BLE: Streaming %s\n", value == BT_GATT_CCC_NOTIFY ? "an" : "aus");
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

/* Attribute: [0] Service, [1] Daten-Decl, [2] Daten-Wert, [3] CCC,
 *            [4] Akku-Decl, [5] Akku-Wert, [6] CCC, [7] Kal-Decl, [8] Kal-Wert */
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
);

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
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("# BLE: getrennt (0x%02x)\n", reason);
	atomic_set(&notify_on, 0);
	atomic_set(&bat_notify_on, 0);
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

static void adv_start_if_needed(void)
{
	if (!atomic_cas(&need_adv, 1, 0)) {
		return;
	}
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		printk("# BLE: Advertising-Fehler %d\n", err);
		atomic_set(&need_adv, 1);   /* beim nächsten Durchlauf erneut */
	} else {
		printk("# BLE: sichtbar als \"%s\"\n", CONFIG_BT_DEVICE_NAME);
	}
}

/* Messungen sammeln und als ein Paket verschicken */
static uint8_t pkt[PKT_HDR + BLE_BATCH * PKT_SAMPLE];
static uint8_t pkt_n;

static void ble_push(uint32_t t_ms, const int16_t v[6], uint16_t fsr_word)
{
	if (!cur_conn || !atomic_get(&notify_on)) {
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

static void ble_push_bat(void)
{
	if (!cur_conn || !atomic_get(&bat_notify_on)) {
		return;
	}
	uint8_t v[2];

	sys_put_le16(bat_mv, v);
	bt_gatt_notify(cur_conn, &feo_svc.attrs[5], v, sizeof(v));
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

/* ---------------- IMU ---------------- */

/* serielle Zeilen sammeln und als Schub ausgeben */
static char ser_buf[SERIAL_BATCH * 96];
static size_t ser_len;
static uint8_t ser_n;

static void serial_push(uint32_t t, const double a[3], const double g[3],
			int32_t fsr_mv, uint16_t range_mv)
{
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

	if (settings_subsys_init() == 0) {
		settings_load();
	} else {
		printk("# Fehler: Settings (Kalibrierung) nicht verfügbar\n");
	}

	int err = bt_enable(NULL);

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
	printk("# Firmware %s (MCUboot, Update per Bluetooth möglich)\n",
	       CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION);
#endif
	printk("# XIAO nRF54L15 FEO-Sensor, %d Hz, +-%d g, +-%d dps, FSR + Akku\n",
	       SAMPLE_RATE_HZ, ACCEL_RANGE_G, GYRO_RANGE_DPS);
	cal_print("aktiv");
	printk("# t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,u_fsr_mv,fsr_range_mv,u_bat_mv\n");

	int64_t next = k_uptime_get();

	while (1) {
		adv_start_if_needed();

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

			if (z >= ZERO_MIN_MV && z <= ZERO_MAX_MV) {
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
			int16_t raw[6];

			for (int i = 0; i < 3; i++) {
				a[i] = sensor_value_to_double(&acc[i]) / G_TO_MS2;
				g[i] = sensor_value_to_double(&gyr[i]) * RAD_TO_DEG;
				raw[i] = (int16_t)CLAMP(a[i] * 1000.0, INT16_MIN, INT16_MAX);
				raw[3 + i] = (int16_t)CLAMP(g[i] * 10.0, INT16_MIN, INT16_MAX);
			}
			ble_push(t, raw, fsr_word);
			serial_push(t, a, g, fsr_mv, adc_ok ? fsr_range_mv[fsr_word >> 14] : 0);
		} else {
			printk("# Lesefehler\n");
		}

		/* LED: langsam blinken = wartet auf BLE, schnell = streamt */
		uint32_t blink = atomic_get(&notify_on) ? SAMPLE_RATE_HZ / 10 : SAMPLE_RATE_HZ / 2;

		if (++n % blink == 0) {
			gpio_pin_toggle_dt(&led);
		}

		/* festen Takt halten, unabhängig von der Laufzeit der Schleife */
		next += SAMPLE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
	return 0;
}

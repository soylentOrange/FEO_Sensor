/*
 * Schritterkennung über die sagittale Drehrate des Unterschenkels nach
 * Fadillioglu, Stetter et al. (2020): Automated gait event detection for a
 * variety of locomotion tasks using a novel gyroscope-based algorithm.
 * Gait & Posture, doi:10.1016/j.gaitpost.2020.06.019
 * Hier als Echtzeit-Variante mit einem Ringpuffer der letzten 5 s:
 *
 *   x  = Drehrate um die Sagittalachse (positiv = Fuß schwingt nach vorn),
 *        Gyro-Offset abgezogen (in Ruhe nachgeführt)
 *   w  = x, Butterworth-Tiefpass 15 Hz
 *   c  = Komplementärsignal x - w, Tiefpass 10 Hz
 *   MS Mitte der Schwungphase: Maximum von w über der Schwelle, frühestens
 *      333 ms nach dem vorigen MS. Die Schwelle passt sich an
 *      (max(MS_MIN, 30 % der letzten Maxima)) statt fest 1000 mV im Paper.
 *   IC Fersenauftritt / Initialkontakt: erster Nulldurchgang von w nach MS
 *   TO Zehenablösung: negatives Maximum N von w vor MS; Gangzyklus Z = Abstand
 *      der letzten beiden MS. Suchfenster ab vorigem MS + 50 % Z bis
 *      N + 10 % Z (langsam, Z > 1 s, TO = Minimum von c) bzw. bis N
 *      (schnell, TO = Maximum von c). Ohne vorigen MS: TO = N.
 *
 * Ein Doppelschritt reicht von IC bis zum nächsten IC; Standphase = IC..TO,
 * Schwungphase = TO..IC. Die Kraft wird nachträglich über die Phasen
 * ausgewertet.
 */
#include "gait.h"
#include <math.h>
#include <string.h>

#define RING          512                     /* 5,12 s bei 100 Hz */
#define MS_PER_SAMPLE (1000 / GAIT_FS_HZ)
#define S(ms)         ((ms) / MS_PER_SAMPLE)

#define LP_FC_HZ      15.0f   /* Tiefpass für w */
#define LPC_FC_HZ     10.0f   /* Tiefpass für das Komplementärsignal */
#define MS_MIN_DPS    50.0f   /* kleinste Schwelle für die Schwungphase */
#define MS_REL        0.3f    /* Schwelle relativ zum mittleren Maximum */
#define MS_GAP        S(333)  /* Mindestabstand zweier MS */
#define NEG_WIN       S(600)  /* Suchfenster für das negative Maximum vor MS */
#define SLOW_CYCLE    S(1000) /* Gangzyklus länger -> "langsames" TO-Fenster */
#define IC_TIMEOUT    S(800)  /* kein Nulldurchgang so lange nach MS -> verwerfen */
#define STRIDE_MIN    S(400)
#define STRIDE_MAX    S(3000)
#define STOP_AFTER    S(3000) /* so lange keine Schwungphase -> Gang unterbrochen */
#define KG_CAP        600.0f

/* Gyro-Offset: in Ruhe über 1 s mitteln */
#define BIAS_BLOCK    GAIT_FS_HZ
#define BIAS_SD_DPS   1.5f
#define BIAS_ACC_TOL  0.05f

/* automatische Achse: Hauptachse der Drehrate bei Bewegung */
#define AUTO_MOVE_DPS 30.0f
#define AUTO_ALPHA    0.002f  /* Vergessen: ~500 bewegte Messungen = 5 s */
#define AUTO_MIN_N    300

enum state { ST_WAIT_MS, ST_PEAK, ST_IC };

static struct {
	/* Ausrichtung */
	bool calibrated;
	float axis_cal[3];
	/* automatische Achse */
	float cov[3][3], ev[3], skew;
	uint32_t auto_n;
	/* Gyro-Offset */
	float bias[3], bsum[3], bsq[3], bacc_min, bacc_max;
	int bn;
	bool bias_ok;
	/* Tiefpässe (Biquads) für w und c */
	struct bq { float b0, b1, b2, a1, a2, z1, z2; } lp, lpc;
	/* Ringpuffer */
	float rw[RING], rc[RING], rkg[RING];
	uint32_t rt[RING];
	uint32_t k;               /* Nummer der aktuellen Messung */
	/* Zustandsautomat */
	enum state st;
	float thr_avg, peak;
	uint32_t ms_k, ms_prev_k, ic_k, to_k;
	bool ms_prev_valid, ic_valid, to_valid, to_fallback;
	uint32_t seq;
} g;

#define R(i) ((i) % RING)
#define MINU(a, b) ((a) < (b) ? (a) : (b))
#define MAXU(a, b) ((a) > (b) ? (a) : (b))

static void normalize(float v[3])
{
	float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	if (n > 1e-9f) {
		v[0] /= n; v[1] /= n; v[2] /= n;
	}
}

static float dot(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* Butterworth-Tiefpass 2. Ordnung (bilineare Transformation) */
static void bq_lowpass(struct bq *f, float fc)
{
	const float K = tanf(3.14159265f * fc / GAIT_FS_HZ), q = 0.70710678f;
	const float nrm = 1.0f / (1.0f + K / q + K * K);

	*f = (struct bq){ 0 };
	f->b0 = K * K * nrm;
	f->b1 = 2.0f * f->b0;
	f->b2 = f->b0;
	f->a1 = 2.0f * (K * K - 1.0f) * nrm;
	f->a2 = (1.0f - K / q + K * K) * nrm;
}

/* transponierte Direktform II */
static float bq_run(struct bq *f, float x)
{
	float y = f->b0 * x + f->z1;

	f->z1 = f->b1 * x - f->a1 * y + f->z2;
	f->z2 = f->b2 * x - f->a2 * y;
	return y;
}

void gait_init(void)
{
	memset(&g, 0, sizeof(g));
	g.ev[0] = g.ev[1] = g.ev[2] = 0.57735f;
	g.bacc_min = 1e9f;
	g.bacc_max = -1e9f;
	bq_lowpass(&g.lp, LP_FC_HZ);
	bq_lowpass(&g.lpc, LPC_FC_HZ);
}

bool gait_frame_valid(const struct gait_frame *f)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float d = dot(f->r[i], f->r[j]);

			if (!(fabsf(d - (i == j ? 1.0f : 0.0f)) < 0.05f)) {   /* auch NaN */
				return false;
			}
		}
	}
	/* rechtshändig: x × y = z */
	float c[3] = {
		f->r[0][1] * f->r[1][2] - f->r[0][2] * f->r[1][1],
		f->r[0][2] * f->r[1][0] - f->r[0][0] * f->r[1][2],
		f->r[0][0] * f->r[1][1] - f->r[0][1] * f->r[1][0],
	};
	return dot(c, f->r[2]) > 0.9f;
}

void gait_set_frame(const struct gait_frame *f)
{
	if (f && f->stamp && gait_frame_valid(f)) {
		memcpy(g.axis_cal, f->r[1], sizeof(g.axis_cal));
		normalize(g.axis_cal);
		g.calibrated = true;
	} else {
		g.calibrated = false;
	}
}

static void auto_axis(float out[3])
{
	float s = g.skew < 0 ? -1.0f : 1.0f;

	for (int i = 0; i < 3; i++) {
		out[i] = s * g.ev[i];
	}
}

void gait_status(float axis[3], float bias_dps[3], uint32_t *count)
{
	if (g.calibrated) {
		memcpy(axis, g.axis_cal, sizeof(g.axis_cal));
	} else {
		auto_axis(axis);
	}
	memcpy(bias_dps, g.bias, sizeof(g.bias));
	*count = g.seq;
}

/* Gyro-Offset nachführen, wenn das Bein 1 s ruhig ist */
static void bias_update(const float gr[3], const float a[3])
{
	float an = sqrtf(dot(a, a));

	for (int i = 0; i < 3; i++) {
		g.bsum[i] += gr[i];
		g.bsq[i] += gr[i] * gr[i];
	}
	g.bacc_min = fminf(g.bacc_min, an);
	g.bacc_max = fmaxf(g.bacc_max, an);
	if (++g.bn < BIAS_BLOCK) {
		return;
	}
	bool still = fabsf(g.bacc_min - 1.0f) < BIAS_ACC_TOL && fabsf(g.bacc_max - 1.0f) < BIAS_ACC_TOL;

	for (int i = 0; i < 3 && still; i++) {
		float m = g.bsum[i] / g.bn, var = g.bsq[i] / g.bn - m * m;

		still = var < BIAS_SD_DPS * BIAS_SD_DPS;
	}
	if (still) {
		for (int i = 0; i < 3; i++) {
			float m = g.bsum[i] / g.bn;

			g.bias[i] = g.bias_ok ? 0.7f * g.bias[i] + 0.3f * m : m;
		}
		g.bias_ok = true;
	}
	memset(g.bsum, 0, sizeof(g.bsum));
	memset(g.bsq, 0, sizeof(g.bsq));
	g.bacc_min = 1e9f;
	g.bacc_max = -1e9f;
	g.bn = 0;
}

/* Kovarianz der Drehrate bei Bewegung, Hauptachse per Potenzmethode */
static void auto_update(const float w[3])
{
	if (sqrtf(dot(w, w)) < AUTO_MOVE_DPS) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			g.cov[i][j] += AUTO_ALPHA * (w[i] * w[j] - g.cov[i][j]);
		}
	}
	float p = dot(g.ev, w);

	g.skew += AUTO_ALPHA * (p * p * p - g.skew);
	if (++g.auto_n % 25 == 0) {
		for (int it = 0; it < 2; it++) {
			float v[3];

			for (int i = 0; i < 3; i++) {
				v[i] = dot(g.cov[i], g.ev);
			}
			normalize(v);
			memcpy(g.ev, v, sizeof(v));
		}
	}
}

void gait_phase_bounds(uint32_t l, uint32_t t, uint32_t b[GAIT_PHASES + 1])
{
	uint32_t half = l / 2, ic = MAXU((l + 25) / 50, 1);   /* 2 % */
	uint32_t ds = t > half ? t - half : 0;                  /* Doppelunterstützung */
	uint32_t ss_end = MINU(half, t);
	uint32_t sw = l - t;

	b[GP_IC] = 0;
	b[GP_LR] = MINU(ic, t);
	b[GP_MST] = MAXU(ds, b[GP_LR]);
	b[GP_TST] = MAXU((b[GP_MST] + ss_end) / 2, b[GP_MST]);
	b[GP_PSW] = MAXU(ss_end, b[GP_TST]);
	b[GP_ISW] = t;
	b[GP_MSW] = t + sw / 3;
	b[GP_TSW] = t + (2 * sw) / 3;
	b[GAIT_PHASES] = l;
}

static bool emit(uint32_t ic_new, struct gait_stride *o)
{
	uint32_t ic = g.ic_k, to = g.to_k;

	if (!g.ic_valid || !g.to_valid || to <= ic || to >= ic_new ||
	    ic_new - ic < STRIDE_MIN || ic_new - ic > STRIDE_MAX || g.k - ic >= RING) {
		return false;
	}
	memset(o, 0, sizeof(*o));
	float fmax = -1, sum = 0, ssum = 0;
	uint32_t kmax = ic;

	for (uint32_t i = ic; i < to; i++) {
		float f = g.rkg[R(i)];

		if (isinf(f)) {
			o->flags |= GAIT_F_OVERLOAD;
			f = KG_CAP;
		}
		if (f > fmax) {
			fmax = f;
			kmax = i;
		}
		sum += f;
	}
	for (uint32_t i = to; i < ic_new; i++) {
		ssum += fminf(g.rkg[R(i)], KG_CAP);
	}
	/* Verlauf über den normierten Gangzyklus */
	uint16_t bn[GAIT_BINS] = { 0 };

	for (uint32_t i = ic; i < ic_new; i++) {
		uint32_t b = (i - ic) * GAIT_BINS / (ic_new - ic);

		o->f_bins[b] += fminf(g.rkg[R(i)], KG_CAP);
		bn[b]++;
	}
	for (int b = 0; b < GAIT_BINS; b++) {
		o->f_bins[b] = bn[b] ? o->f_bins[b] / bn[b] : 0;
	}
	/* Gangphasen (Rancho Los Amigos) */
	uint32_t pb[GAIT_PHASES + 1];

	gait_phase_bounds(ic_new - ic, to - ic, pb);
	for (int p = 0; p < GAIT_PHASES; p++) {
		float sm = 0, pk = 0;

		for (uint32_t i = ic + pb[p]; i < ic + pb[p + 1]; i++) {
			float f = fminf(g.rkg[R(i)], KG_CAP);

			sm += f;
			pk = fmaxf(pk, f);
		}
		o->f_ph_mean[p] = pb[p + 1] > pb[p] ? sm / (float)(pb[p + 1] - pb[p]) : 0;
		o->f_ph_peak[p] = pk;
	}
	o->seq = ++g.seq;
	o->t_ic_ms = g.rt[R(ic)];
	o->stride_ms = (uint16_t)((ic_new - ic) * MS_PER_SAMPLE);
	o->stance_ms = (uint16_t)((to - ic) * MS_PER_SAMPLE);
	o->ms_dps = g.peak;
	o->ic_dps = g.rw[R(ic)];
	o->to_dps = g.rw[R(to)];
	o->f_peak_kg = fmax;
	o->f_mean_kg = sum / (float)(to - ic);
	o->f_imp_kgs = sum / GAIT_FS_HZ;
	o->t_fpeak_ms = (uint16_t)((kmax - ic) * MS_PER_SAMPLE);
	o->f_swing_kg = ssum / (float)(ic_new - to);
	if (g.calibrated) {
		o->flags |= GAIT_F_CALIBRATED;
	} else if (g.auto_n < AUTO_MIN_N) {
		o->flags |= GAIT_F_AUTO_SIGN;
	}
	if (g.to_fallback) {
		o->flags |= GAIT_F_TO_SIMPLE;
	}
	return true;
}

/* TO für den gerade bestätigten MS (g.ms_k) suchen */
static void find_to(void)
{
	uint32_t ms = g.ms_k, from = ms > NEG_WIN ? ms - NEG_WIN : 1, neg = ms;
	float nv = INFINITY;

	if (g.ic_valid && g.ic_k + 1 > from) {
		from = g.ic_k + 1;           /* nicht vor dem letzten Auftritt suchen */
	}
	for (uint32_t i = from; i < ms; i++) {   /* negatives Maximum N vor MS */
		if (g.rw[R(i)] < nv) {
			nv = g.rw[R(i)];
			neg = i;
		}
	}
	g.to_valid = nv < 0;
	g.to_k = neg;
	g.to_fallback = true;
	if (!g.to_valid || !g.ms_prev_valid || ms - g.ms_prev_k > STRIDE_MAX) {
		return;
	}
	uint32_t cyc = ms - g.ms_prev_k, a = g.ms_prev_k + cyc / 2, b;
	bool slow = cyc > SLOW_CYCLE;

	if (a < from) {
		a = from;
	}
	b = slow ? neg + cyc / 10 : neg;
	if (b > ms) {
		b = ms;
	}
	if (a >= b) {
		return;
	}
	uint32_t best = a;

	for (uint32_t i = a; i <= b; i++) {
		if (slow ? g.rc[R(i)] < g.rc[R(best)] : g.rc[R(i)] > g.rc[R(best)]) {
			best = i;
		}
	}
	g.to_k = best;
	g.to_fallback = false;
}

void gait_pause(void)
{
	g.ic_valid = g.ms_prev_valid = g.to_valid = false;
	g.thr_avg = 0;
	g.st = ST_WAIT_MS;
	g.lp.z1 = g.lp.z2 = g.lpc.z1 = g.lpc.z2 = 0;
	g.bn = 0;
	memset(g.bsum, 0, sizeof(g.bsum));
	memset(g.bsq, 0, sizeof(g.bsq));
	g.bacc_min = 1e9f;
	g.bacc_max = -1e9f;
}

bool gait_update(uint32_t t_ms, const float g_dps[3], const float a_g[3], float kg,
		 struct gait_stride *out)
{
	float gr[3], axis[3];
	bool done = false;

	bias_update(g_dps, a_g);
	for (int i = 0; i < 3; i++) {
		gr[i] = g_dps[i] - g.bias[i];
	}
	auto_update(gr);
	if (g.calibrated) {
		memcpy(axis, g.axis_cal, sizeof(axis));
	} else {
		auto_axis(axis);
	}

	/* sagittale Drehrate, gefiltert, und Komplementärsignal */
	float x = dot(axis, gr);
	float w = bq_run(&g.lp, x);
	float c = bq_run(&g.lpc, x - w);

	uint32_t k = ++g.k;

	g.rw[R(k)] = w;
	g.rc[R(k)] = c;
	g.rkg[R(k)] = kg > KG_CAP ? INFINITY : (kg > 0 ? kg : 0);
	g.rt[R(k)] = t_ms;

	float thr = fmaxf(MS_MIN_DPS, MS_REL * g.thr_avg);

	switch (g.st) {
	case ST_WAIT_MS:
		if (g.ic_valid && k - g.ic_k > STOP_AFTER) {
			g.ic_valid = false;          /* stehen geblieben */
			g.ms_prev_valid = false;
			g.thr_avg = 0;
		}
		if (w > thr && k > NEG_WIN && (!g.ms_prev_valid || k - g.ms_prev_k >= MS_GAP)) {
			g.peak = w;
			g.ms_k = k;
			g.st = ST_PEAK;
		}
		break;
	case ST_PEAK:
		if (w > g.peak) {
			g.peak = w;
			g.ms_k = k;
		}
		if (w < thr) {
			g.thr_avg = g.thr_avg > 0 ? 0.75f * g.thr_avg + 0.25f * g.peak : g.peak;
			find_to();
			g.ms_prev_k = g.ms_k;
			g.ms_prev_valid = true;
			g.st = ST_IC;
		}
		break;
	case ST_IC:
		if (w < 0) {
			/* Nulldurchgang: die Messung näher an 0 ist IC */
			uint32_t ic = (-w < g.rw[R(k - 1)]) ? k : k - 1;

			done = emit(ic, out);
			g.ic_k = ic;
			g.ic_valid = true;
			g.to_valid = false;
			g.st = ST_WAIT_MS;
		} else if (k - g.ms_k > IC_TIMEOUT) {
			g.to_valid = false;          /* kein Auftritt: Bewegung verwerfen */
			g.st = ST_WAIT_MS;
		}
		break;
	}
	return done;
}

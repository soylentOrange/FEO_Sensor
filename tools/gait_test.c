/*
 * PC-Test der Schritterkennung (src/gait.c) mit synthetischen Gangdaten oder
 * einer Aufnahme der Web-App (CSV mit t_ms,ax..gz,...,kg).
 *   cc -O2 -Isrc tools/gait_test.c src/gait.c -lm -o gait_test
 *   ./gait_test                 synthetischer Gang (Pause, 20 Doppelschritte, Pause, 10)
 *   ./gait_test aufnahme.csv    Aufnahme (kg aus Spalte 12, falls vorhanden)
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gait.h"

static float gs(float x, float m, float s) { return expf(-0.5f * (x - m) * (x - m) / (s * s)); }

/* sagittale Drehrate (°/s) und Fersenlast (kg) über der Gangphase 0..1 (IC bei 0) */
static void model(float p, float *w, float *kg)
{
	*w = -110 * gs(p, 0.02f, 0.015f) - 40 * gs(p, 0.3f, 0.12f) - 190 * gs(p, 0.57f, 0.035f)
	     + 330 * gs(p, 0.78f, 0.08f) - 110 * gs(p, 1.02f, 0.015f) + 20 * gs(p, 0.95f, 0.03f);
	*kg = p < 0.6f ? 25 * gs(p, 0.12f, 0.06f) + 3 * gs(p, 0.35f, 0.1f) : 0;
}

static float nrand(void) { return ((rand() / (float)RAND_MAX) - 0.5f) * 3.46f; }

static void print(const struct gait_stride *s)
{
	printf("#S %3u t=%6u stride=%4u stance=%4u MS=%6.1f IC=%7.1f TO=%7.1f Fpk=%5.1f Fm=%5.1f imp=%5.2f tpk=%3u Fsw=%5.2f fl=%x\n",
	       s->seq, s->t_ic_ms, s->stride_ms, s->stance_ms, s->ms_dps, s->ic_dps, s->to_dps,
	       s->f_peak_kg, s->f_mean_kg, s->f_imp_kgs, s->t_fpeak_ms, s->f_swing_kg, s->flags);
	if (s->seq == 5) {
		printf("   Kraft über den Gangzyklus (je 5 %%):");
		for (int b = 0; b < GAIT_BINS; b++) printf(" %.1f", s->f_bins[b]);
		printf("\n   Phasen IC/LR/MSt/TSt/PSw/ISw/MSw/TSw Mittel:");
		for (int p = 0; p < GAIT_PHASES; p++) printf(" %.1f", s->f_ph_mean[p]);
		printf("\n   Maximum:");
		for (int p = 0; p < GAIT_PHASES; p++) printf(" %.1f", s->f_ph_peak[p]);
		uint32_t b[GAIT_PHASES + 1];
		gait_phase_bounds(100, 60, b);
		printf("\n   Grenzen bei T = 60 %%:");
		for (int p = 0; p <= GAIT_PHASES; p++) printf(" %u", b[p]);
		printf("\n");
	}
}

int main(int argc, char **argv)
{
	struct gait_stride s;
	int n = 0;

	gait_init();
	if (argc > 1) {
		FILE *f = fopen(argv[1], "r");
		char line[512];

		if (!f) { perror(argv[1]); return 1; }
		while (fgets(line, sizeof(line), f)) {
			double v[16] = { 0 };
			int c = 0;
			char *p = line, *e;

			if (line[0] < '0' || line[0] > '9') continue;
			while (c < 16) {
				v[c++] = strtod(p, &e);
				if (*e != ',') break;
				p = e + 1;
			}
			float a[3] = { v[1], v[2], v[3] }, gy[3] = { v[4], v[5], v[6] };
			float kg = c > 12 ? v[12] : 0;

			if (gait_update((uint32_t)v[0], gy, a, kg, &s)) { print(&s); n++; }
		}
		printf("%d Doppelschritte\n", n);
		return 0;
	}
	/* Sensor schräg montiert: Sagittalachse (nach rechts) in Sensorkoordinaten */
	float ax[3] = { 0.3f, -0.8f, 0.5f }, nn = sqrtf(0.09f + 0.64f + 0.25f);
	float bias[3] = { 1.5f, -2.0f, 0.7f }, t = 0;
	for (int i = 0; i < 3; i++) ax[i] /= nn;
	const float T = 1.15f;   /* Doppelschrittdauer (s) */
	const int phases[][2] = { { 0, 300 }, { 1, 20 }, { 0, 400 }, { 1, 10 }, { 0, 200 } };

	for (unsigned ph = 0; ph < sizeof(phases) / sizeof(phases[0]); ph++) {
		int len = phases[ph][0] ? (int)(phases[ph][1] * T * 100) : phases[ph][1];

		for (int i = 0; i < len; i++, t += 0.01f) {
			float w = 0, kg = 0, a[3] = { 0, 0, 1 }, gy[3];

			if (phases[ph][0]) model(fmodf(i * 0.01f / T + 0.3f, 1.0f), &w, &kg);
			for (int j = 0; j < 3; j++)
				gy[j] = ax[j] * w + bias[j] + 0.8f * nrand() + (phases[ph][0] ? 25 * sinf(t * 6.28f / T * 2 + j) * (j == 1) : 0);
			if (phases[ph][0]) a[0] += 0.3f * sinf(t * 9);
			if (gait_update((uint32_t)(t * 1000), gy, a, kg + 0.05f * nrand(), &s)) { print(&s); n++; }
		}
	}
	float axis[3], b[3];
	uint32_t cnt;
	gait_status(axis, b, &cnt);
	printf("%d Doppelschritte (erwartet ~28); Achse %.3f %.3f %.3f (wahr %.3f %.3f %.3f); Offset %.2f %.2f %.2f\n",
	       n, axis[0], axis[1], axis[2], ax[0], ax[1], ax[2], b[0], b[1], b[2]);
	return 0;
}

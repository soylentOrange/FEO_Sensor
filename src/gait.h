/*
 * Schritterkennung (Fadillioglu et al. 2020) aus der Drehrate des Unterschenkels in der Sagittalebene
 * und Kraft (kg) pro Schritt. Reines C ohne Zephyr-Abhängigkeiten, damit der
 * Algorithmus auch auf dem PC getestet werden kann (tools/gait_test.c).
 *
 * Segment-Koordinatensystem des Unterschenkels (Zeilen von gait_frame.r,
 * jeweils als Einheitsvektor in Sensorkoordinaten):
 *   x  nach vorn (anterior)
 *   y  nach rechts; Drehachse der Sagittalebene. Positive Drehung um +y
 *      = Fuß schwingt nach vorn (Schwungphase)
 *   z  nach unten, entlang des Unterschenkels Richtung Fuß
 * Ohne Kalibrierung (stamp == 0) schätzt die Firmware die Drehachse selbst
 * (Hauptachse der Drehrate beim Gehen, Vorzeichen aus der Schiefe).
 */
#ifndef GAIT_H
#define GAIT_H

#include <stdint.h>
#include <stdbool.h>

#define GAIT_FS_HZ 100
#define GAIT_BINS  20     /* Kraftverlauf: Mittelwerte über je 5 % des Gangzyklus */

/*
 * Gangphasen nach Rancho Los Amigos (Perry). Mit einem Sensor an einem Bein sind
 * nur IC und TO dieses Beins bekannt; die Grenzen dazwischen werden unter der
 * Annahme eines symmetrischen Gangs abgeleitet (Doppelschritt L, Standphase T):
 *   IC   Initial Contact   0 .. 2 % L
 *   LR   Loading Response  .. T - L/2   (Ende der 1. Doppelunterstützung =
 *                                       Zehenablösung des anderen Beins)
 *   MSt  Mid Stance        .. Mitte der Einbeinstandphase
 *   TSt  Terminal Stance   .. L/2       (Fersenauftritt des anderen Beins)
 *   PSw  Pre-Swing         .. T         (TO)
 *   ISw, MSw, TSw          Schwungphase in Dritteln
 * Bei normalem Gang (T = 60 % L) ergibt das 0/2/10/30/50/60/73/87/100 %.
 */
#define GAIT_PHASES 8
enum gait_phase { GP_IC, GP_LR, GP_MST, GP_TST, GP_PSW, GP_ISW, GP_MSW, GP_TSW };
/* Phasengrenzen in Messungen ab IC für Doppelschritt l und Standphase t;
 * Phase p reicht von b[p] bis b[p + 1] (b[0] = 0, b[GAIT_PHASES] = l) */
void gait_phase_bounds(uint32_t l, uint32_t t, uint32_t b[GAIT_PHASES + 1]);

struct gait_frame {
	float r[3][3];          /* Zeilen x, y, z (siehe oben) in Sensorkoordinaten */
	uint32_t stamp;         /* Unix-Zeit der Kalibrierung, 0 = automatisch */
};

/* flags in gait_stride */
#define GAIT_F_OVERLOAD   0x0001   /* Kraftsensor in der Standphase übersteuert */
#define GAIT_F_CALIBRATED 0x0002   /* Achse aus der Kalibrierung (sonst automatisch) */
#define GAIT_F_AUTO_SIGN  0x0004   /* automatische Achse, Vorzeichen noch unsicher */
#define GAIT_F_TO_SIMPLE  0x0008   /* TO nur als negatives Maximum vor MS (kein voriger MS) */

/* Ein Doppelschritt des Beins mit Sensor: Fersenauftritt (IC) bis zum nächsten IC */
struct gait_stride {
	uint32_t seq;           /* laufende Nummer seit dem Einschalten, ab 1 */
	uint32_t t_ic_ms;       /* Zeitpunkt des Fersenauftritts (Uptime) */
	uint16_t stride_ms;     /* Dauer bis zum nächsten Fersenauftritt */
	uint16_t stance_ms;     /* Standphase: IC bis Zehenablösung (TO) */
	float ms_dps;           /* Drehrate in der Mitte der Schwungphase (Maximum) */
	float ic_dps;           /* Drehrate beim Initialkontakt (~0, Nulldurchgang) */
	float to_dps;           /* Drehrate bei der Zehenablösung */
	float f_peak_kg;        /* größte Kraft in der Standphase */
	float f_mean_kg;        /* mittlere Kraft in der Standphase */
	float f_imp_kgs;        /* Kraftstoß der Standphase (kg*s) */
	uint16_t t_fpeak_ms;    /* Zeit vom IC bis zur größten Kraft */
	float f_swing_kg;       /* mittlere Kraft in der Schwungphase (sollte ~0 sein) */
	float f_bins[GAIT_BINS];/* Kraft über den Gangzyklus, IC = 0 %, nächster IC = 100 % */
	float f_ph_mean[GAIT_PHASES];  /* Kraft je Gangphase (Rancho Los Amigos), Mittel */
	float f_ph_peak[GAIT_PHASES];  /* ... und Maximum */
	float rom_deg;          /* sagittal shank excursion over the stride (max - min angle, deg) */
	float f_rate_kgps;      /* largest heel loading rate in stance before the peak (kg/s, 30 ms slope) */
	uint8_t heel_pct;       /* share of stance with heel load > heel threshold (%) */
	uint16_t flags;
};

void gait_init(void);
/* Kalibrierte Ausrichtung setzen; NULL oder stamp == 0 -> automatisch */
void gait_set_frame(const struct gait_frame *f);
/* Prüft, ob die Zeilen ein (ungefähr) orthonormales, rechtshändiges System bilden */
bool gait_frame_valid(const struct gait_frame *f);
/* Achse, die gerade für die Schritterkennung benutzt wird, und Gyro-Offset */
void gait_status(float axis[3], float bias_dps[3], uint32_t *count);

/* Heel load threshold for heel_pct (kg) */
void gait_set_heel_thr(float kg);

/*
 * Tilt of the shank against vertical (deg, 0..90) for a mean acceleration a (g):
 * angle between a and the shank axis (z of the calibrated frame, otherwise the mean
 * acceleration over the last strides, i.e. the shank direction while walking).
 * NAN as long as neither is known. About 0 standing or sitting, about 90 lying or
 * with the leg up on a stool.
 */
float gait_tilt_deg(const float a[3]);

/* Messung unterbrochen (Ruhemodus): laufenden Schritt verwerfen */
void gait_pause(void);

/*
 * Eine Messung verarbeiten (100 Hz). g in Grad/s, a in g, kg = Kraft
 * (INFINITY = Überlast). Gibt true zurück, wenn ein Doppelschritt fertig ist.
 */
bool gait_update(uint32_t t_ms, const float g_dps[3], const float a_g[3], float kg,
		 struct gait_stride *out);

#endif

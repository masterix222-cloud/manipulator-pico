#include "kinematics.h"
#include <math.h>

// Macierz 9 -> 5, wyliczona z pomiarow treningowych tego montazu
// (troikaty 120 st., trzy magnesy, odleglosc ~10 mm)
static const float MAC[5][9] = {
  { -0.17355f,  0.20681f,  0.50162f,  1.77300f, -1.21028f,  0.54345f, -0.03686f,  0.31001f,  1.37849f },
  {  0.37114f,  0.85255f,  0.27474f, -0.76410f,  0.19178f, -0.71944f,  0.63842f, -0.24763f,  0.08458f },
  {  0.34596f, -0.27635f, -0.89907f, -0.98649f,  0.84768f,  0.57308f,  0.11286f,  0.31552f, -0.26555f },
  {  0.36144f, -0.31805f, -0.80754f, -0.02331f, -0.27134f, -0.36926f, -0.51783f, -0.44973f,  0.12959f },
  { -0.85082f, -0.45803f,  0.40878f, -0.74004f,  0.05932f,  0.35860f,  0.02737f, -0.54115f, -1.02919f },
};

// Skala per os — pelny ruch daje ok. 1.0
static const float SKALA_OS[5] = { 1.0f/1.1f, 1.0f/1.0f, 1.0f/1.1f, 1.0f/1.0f, 1.0f/1.1f };

// Martwe strefy (w jednostkach znormalizowanych)
static const double MARTWA[5] = { 0.06, 0.06, 0.10, 0.07, 0.07 };

// Filtr EMA — Z (zoom) wolniejszy, reszta szybsza
static const double ALFA[5] = { 0.25, 0.25, 0.12, 0.20, 0.20 };

static double zero9[9] = {0};
static double filtr[5] = {0};
static bool skalibrowane = false;

static double strefa(double v, double prog) {
    if (fabs(v) < prog) return 0.0;
    double znak = (v > 0) ? 1.0 : -1.0;
    double t = (fabs(v) - prog) / (1.0 - prog);
    if (t > 1.0) t = 1.0;
    return znak * t;
}

void kin_calibrate_zero(const double v[9]) {
    for (int i = 0; i < 9; i++) zero9[i] = v[i];
    skalibrowane = true;
    for (int o = 0; o < 5; o++) filtr[o] = 0.0;
}

void kin_calc(const double v[9], double os[5]) {
    double d9[9];
    for (int i = 0; i < 9; i++) d9[i] = skalibrowane ? (v[i] - zero9[i]) : v[i];

    for (int o = 0; o < 5; o++) {
        double acc = 0;
        for (int i = 0; i < 9; i++) acc += (double)MAC[o][i] * d9[i];
        os[o] = acc * (double)SKALA_OS[o];
    }
}

void kin_update(const double v[9], double os[5]) {
    double surowe[5];
    kin_calc(v, surowe);

    for (int o = 0; o < 5; o++) {
        surowe[o] = strefa(surowe[o], MARTWA[o]);
        filtr[o] += ALFA[o] * (surowe[o] - filtr[o]);
        os[o] = filtr[o];
    }
}

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>

// Kalibracja zera (9 wartosci) — wywolac gdy galka jest puszczona.
void kin_calibrate_zero(const double v[9]);

// Przeliczenie 9 surowych wartosci na 5 osi: X, Y, Z(zoom), Rz(obrot), Ry(przechyl)
// Wynik w zakresie ok. -1..1
void kin_calc(const double v[9], double os[5]);

// Wygodne opakowanie: odczyt + macierz + filtr. Zwraca 5 osi.
void kin_update(const double v[9], double os[5]);

#endif

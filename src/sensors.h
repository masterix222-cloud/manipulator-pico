#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#define TCA_ADDR   0x70
#define TLV_ADDR   0x5E
#define PIN_SDA    4
#define PIN_SCL    5

// Inicjalizacja magistrali i trzech czujnikow za multiplekserem.
bool sensors_init(void);

// Odczyt wszystkich trzech czujnikow -> 9 surowych wartosci [mT]:
// v[0..2] = S0 x,y,z ; v[3..5] = S1 ; v[6..8] = S2
bool sensors_read9(double v[9]);

#endif

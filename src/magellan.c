/*
 * magellan.c — wersja dla DIY SpaceMouse na TLV493D przez TCA9548A.
 * Warstwa USB (deskryptory, tusb_config) pochodzi z dzialajacego projektu
 * jfedor2/magellan-spacemouse (licencja MIT). Zamiast RS-232 czytamy
 * trzy czujniki TLV493D za multiplekserem i przeliczamy je macierza 9->5.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <bsp/board.h>
#include <tusb.h>

#include <pico/stdlib.h>

#include "sensors.h"
#include "kinematics.h"

// ---------------- piny ----------------
#define BTN1      6
#define BTN2      7
#define BTN3      8
#define LED_PIN   9
#define LED_COUNT 12

// ---------------- nastawy wyjscia ----------------
#define MNOZ_PRZESUW   350.0
#define MNOZ_OBROT     350.0
#define PROG_ODRZUTU   600
#define RAMKA_MS       5
#define PROG_PRZYCISK  20

// ---------------- ring WS2812B ----------------
static uint8_t ring_buf[LED_COUNT * 3];

static void ring_wyslij(void) {
    uint32_t maska = 1u << LED_PIN;
    uint32_t irq = save_and_disable_interrupts();
    for (int i = 0; i < LED_COUNT * 3; i++) {
        uint8_t bajt = ring_buf[i];
        for (int bit = 7; bit >= 0; bit--) {
            if (bajt & (1u << bit)) {
                sio_hw->gpio_set = maska;
                busy_wait_us_32(1);
                sio_hw->gpio_clr = maska;
            } else {
                sio_hw->gpio_set = maska;
                __asm volatile ("nop; nop; nop; nop");
                sio_hw->gpio_clr = maska;
                busy_wait_us_32(1);
            }
        }
    }
    restore_interrupts(irq);
    busy_wait_us_32(60);
}

static void ring_wypelnij(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < LED_COUNT; i++) {
        ring_buf[i*3+0] = g;
        ring_buf[i*3+1] = r;
        ring_buf[i*3+2] = b;
    }
    ring_wyslij();
}

// ---------------- raporty HID (uklad jak w magellanie) ----------------
uint16_t trans_report[3];
uint16_t rot_report[3];
uint8_t  buttons_report[6];

uint8_t trans_pending = 0;
uint8_t rot_pending = 0;
uint8_t buttons_pending = 0;

// ---------------- main ----------------
int main(void) {
    board_init();
    tusb_init();
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    ring_wypelnij(40, 0, 0);          // czerwony: start

    gpio_init(BTN1); gpio_set_dir(BTN1, GPIO_IN); gpio_pull_up(BTN1);
    gpio_init(BTN2); gpio_set_dir(BTN2, GPIO_IN); gpio_pull_up(BTN2);
    gpio_init(BTN3); gpio_set_dir(BTN3, GPIO_IN); gpio_pull_up(BTN3);

    printf("=== Manipulator 5DOF (magellan + TLV493D) ===\n");

    // czujniki
    if (!sensors_init()) {
        printf("BLAD: czujniki nie odpowiadaja!\n");
        while (1) {
            ring_wypelnij(60, 0, 0); sleep_ms(120);
            ring_wypelnij(0, 0, 0);   sleep_ms(120);
        }
    }
    printf("Czujniki OK.\n");

    // kalibracja zera — nie dotykaj galki
    ring_wypelnij(60, 60, 60);
    printf("Kalibracja zera - nie dotykaj galki (3 s)\n");
    for (int i = 0; i < 30; i++) { double v[9]; sensors_read9(v); sleep_ms(20); }

    double sum9[9] = {0};
    int udane = 0;
    for (int i = 0; i < 150; i++) {
        double v[9];
        if (sensors_read9(v)) {
            for (int j = 0; j < 9; j++) sum9[j] += v[j];
            udane++;
        }
        sleep_ms(20);
    }
    if (udane > 20) {
        double mean9[9];
        for (int j = 0; j < 9; j++) mean9[j] = sum9[j] / udane;
        kin_calibrate_zero(mean9);
        printf("Kalibracja OK.\n");
    } else {
        printf("BLAD kalibracji.\n");
    }

    ring_wypelnij(0, 80, 0); sleep_ms(300);
    ring_wypelnij(0, 0, 20);

    bool pb[3] = {false, false, false};
    uint32_t pbt[3] = {0, 0, 0};
    int16_t poprz[3] = {0, 0, 0};

    while (true) {
        tud_task();

        // --- wysylka raportow ---
        if (trans_pending && tud_hid_ready()) {
            tud_hid_report(1, trans_report, 6);
            trans_pending = 0;
        }
        if (rot_pending && tud_hid_ready()) {
            tud_hid_report(2, rot_report, 6);
            rot_pending = 0;
        }
        if (buttons_pending && tud_hid_ready()) {
            tud_hid_report(3, buttons_report, 6);
            buttons_pending = 0;
        }

        // --- odczyt czujnikow ---
        double v[9];
        double os[5] = {0, 0, 0, 0, 0};
        bool os_ok = false;

        if (sensors_read9(v)) {
            kin_update(v, os);
            os_ok = true;

            // auto-zero, gdy osie stoja
            static int stoj = 0;
            static double suma[9] = {0};
            bool stoja = true;
            for (int o = 0; o < 5; o++) if (fabs(os[o]) > 0.08) stoja = false;
            if (stoja) {
                for (int i = 0; i < 9; i++) suma[i] += v[i];
                stoj++;
                if (stoj >= 60) {
                    double m9[9];
                    for (int i = 0; i < 9; i++) m9[i] = suma[i] / stoj;
                    kin_calibrate_zero(m9);
                    stoj = 0;
                    for (int i = 0; i < 9; i++) suma[i] = 0;
                }
            } else {
                stoj = 0;
                for (int i = 0; i < 9; i++) suma[i] = 0;
            }

            int16_t tx = (int16_t)( os[0] * MNOZ_PRZESUW);
            int16_t ty = (int16_t)(-os[1] * MNOZ_PRZESUW);
            int16_t tz = (int16_t)( os[2] * MNOZ_PRZESUW);
            int16_t rx = (int16_t)( os[4] * MNOZ_OBROT);
            int16_t ry = (int16_t)(-os[3] * MNOZ_OBROT);

            // odrzucanie pojedynczych skokow
            int16_t s[3] = {tx, ty, tz};
            for (int a = 0; a < 3; a++) {
                int32_t r = (int32_t)s[a] - poprz[a];
                if (r > PROG_ODRZUTU || r < -PROG_ODRZUTU) s[a] = poprz[a];
                poprz[a] = s[a];
            }

            trans_report[0] = (uint16_t)s[0];
            trans_report[1] = (uint16_t)s[1];
            trans_report[2] = (uint16_t)s[2];
            rot_report[0] = (uint16_t)rx;
            rot_report[1] = (uint16_t)ry;
            rot_report[2] = 0;

            trans_pending = 1;
            rot_pending = 1;
        }

        // --- przyciski ---
        {
            uint8_t piny[3] = {BTN1, BTN2, BTN3};
            uint32_t teraz = to_ms_since_boot(get_absolute_time());
            bool zmiana = false;
            for (int b = 0; b < 3; b++) {
                bool w = (gpio_get(piny[b]) == 0);
                if (w != pb[b] && (teraz - pbt[b]) > PROG_PRZYCISK) {
                    pb[b] = w;
                    pbt[b] = teraz;
                    zmiana = true;
                }
            }
            if (zmiana) {
                memset(buttons_report, 0, sizeof(buttons_report));
                if (pb[0]) buttons_report[0] |= 1u << 0;
                if (pb[1]) buttons_report[0] |= 1u << 1;
                if (pb[2]) buttons_report[0] |= 1u << 2;
                buttons_pending = 1;
            }
        }

        // --- wskaznik na ringu ---
        {
            static uint64_t ost = 0;
            uint64_t teraz = to_us_since_boot(get_absolute_time());
            if (teraz - ost > 100000) {
                ost = teraz;
                double maks = 0;
                for (int o = 0; o < 5; o++) if (fabs(os[o]) > maks) maks = fabs(os[o]);
                if (pb[0])      ring_wypelnij(60, 0, 0);
                else if (pb[1]) ring_wypelnij(0, 0, 60);
                else if (pb[2]) ring_wypelnij(0, 60, 0);
                else if (os_ok && maks > 0.15) {
                    uint8_t j = (uint8_t)(30 + maks * 200);
                    if (j > 255) j = 255;
                    ring_wypelnij(0, j, 0);
                } else {
                    ring_wypelnij(0, 0, 20);
                }
            }
        }

        sleep_ms(RAMKA_MS);
    }

    return 0;
}

// ---------------- HID: zapytania z komputera ----------------
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)itf; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)itf; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}

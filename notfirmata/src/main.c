#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

static inline void init_pwm() {
    // arduino pins 9(OC2B) and 10(OC2A)
    TCCR2A |= _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B |= _BV(CS22);  // divide clock by 64

    // arduino pin 11(OC1A)
    TCCR1A |= _BV(COM1A1) | _BV(WGM10);
    TCCR1B |= _BV(CS10) | _BV(CS11) | _BV(WGM12); // divide clock by 64
}

static inline void set_pwm(uint8_t m1, uint8_t m2, uint8_t m3) {
    //11 = m1
    OCR1AL = m1;
    //9 = m2
    OCR2B = m2;
    //10 = m3
    OCR2A = m3;
}

int main (void)
{
    // set pins to output
    DDRB |= _BV(DDB5); // 11
    DDRB |= _BV(DDB4); // 10
    DDRH |= _BV(DDH6); // 9
    init_pwm();
    while (1) {
        for (uint8_t i = 0; i < 0xff; i++) {
            set_pwm(i, i, i);
            _delay_ms(1);
        }
        for (uint8_t i = 0xff; i != 0; i--) {
            set_pwm(i, i, i);
            _delay_ms(1);
        }
    }

}

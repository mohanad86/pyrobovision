#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../lib/andygock_avr-uart/uart.h"

#define BAUDRATE 19200

static inline void init_pwm() {
    // arduino pins 9(OC2B) and 10(OC2A)
    TCCR2A |= _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B |= _BV(CS22);  // divide clock by 64

    // arduino pin 11(OC1A)
    TCCR1A |= _BV(COM1A1) | _BV(WGM10);
    TCCR1B |= _BV(CS10) | _BV(CS11) | _BV(WGM12); // divide clock by 64

    // set pins to output
    DDRB |= _BV(DDB5); // 11
    DDRB |= _BV(DDB4); // 10
    DDRH |= _BV(DDH6); // 9
}

static inline void set_pwm(uint8_t m1, uint8_t m2, uint8_t m3) {
    //11 = m1
    OCR1AL = m1;
    //9 = m2
    OCR2B = m2;
    //10 = m3
    OCR2A = m3;
}

static inline void init_adc() {

}

static inline uint8_t read_uart(uint8_t byte_count) {
    char bin[10] = {0};
    if (uart0_available()) {
        uart0_putc(' ');
        uint16_t in = uart0_getc();

        if (bit_is_set(in, UART_NO_DATA)) {
            return byte_count; // do nothing
        }

        if (bit_is_set(in, UART_BUFFER_OVERFLOW ||
                           UART_OVERRUN_ERROR ||
                           UART_FRAME_ERROR)) {
            //return 0; // reset reading of the packet
        }

        uint8_t data = (uint8_t)in;
        itoa(byte_count, bin, 10);
        //uart0_puts(bin);
        //uart0_putc(' ');
        //uart0_putc('\r');
        //uart0_putc('\n');
        //uart0_putc(data);

        // find start of a packet with two 0xAA bytes
        if (data == 0xAA && byte_count == 0) {
                //uart0_puts("first start\n\r");
                return 1;
        } else if (data == 0xAA && byte_count == 1) {
                //uart0_puts("second start\n\r");
                return 2;
        }

        //uart0_puts("reading data\n\r");
        switch (byte_count) {
            case 2:
                //uart0_puts("data1\n\r");
                OCR1AL = data;
                break;
            case 3:
                //uart0_puts("data2\n\r");
                OCR2B = data;
                break;
            case 4:
                //uart0_puts("data3\n\r");
                OCR2A = data;
                return 0; // end of packet
            default:
                //uart0_puts("default\n\r");
                return 0; // something went wrong, end of packet and try to resync
        }
        return ++byte_count;
    }
    return byte_count;
}

int main (void)
{

    init_pwm();
    uart0_init(UART_BAUD_SELECT(BAUDRATE, F_CPU));
    sei();
    DDRF = 0;
    uint8_t byte_count = 0;

    //char bin[10] = {0};
    uart0_puts("Start");

    while (1) {
        byte_count = read_uart(byte_count);

        //itoa(byte_count, bin, 2);
        //uart0_puts(bin);
        //uart0_putc('\r');
        //uart0_putc('\n');
        //_delay_ms(1000);
        //for (uint8_t i = 0; i < 0xff; i++) {
        //    set_pwm(i, i, i);
        //    _delay_ms(1);
        //}
        //for (uint8_t i = 0xff; i != 0; i--) {
        //    set_pwm(i, i, i);
        //    _delay_ms(1);
        //}
    }

}

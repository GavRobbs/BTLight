#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__ 
#endif

#define F_CPU 1000000UL
#define BAUD 9600

#include<avr/io.h>
#include<util/setbaud.h>
#include<util/delay.h>
#include<avr/interrupt.h>
#include<stdint.h>

#define RED 1
#define GREEN 2
#define BLUE 3
#define NONE 4

#define BUFFER_SIZE 16
#define INTERVAL 1600

/*A large part of the code here came from or was inspired by
https://hackaday.com/2015/10/29/embed-with-elliot-going-round-with-circular-buffers/ */

enum BufferStatus{EMPTY = 0, FULL = 1, OK = 2};

struct RingBuffer{
    uint8_t data[BUFFER_SIZE];
    uint8_t write_index;
    uint8_t read_index;
};

enum BufferStatus readRingBuffer(volatile struct RingBuffer *b, uint8_t *output){
    if(b->write_index == b->read_index){
        return EMPTY;
    }

    *output = b->data[b->read_index];
    b->read_index = (b->read_index + 1) % BUFFER_SIZE;
    return OK;
}

enum BufferStatus writeRingBuffer(volatile struct RingBuffer *b, uint8_t d){
    uint8_t next_location = (b->write_index + 1) % BUFFER_SIZE;

    if(next_location == b->read_index){
        return FULL;
    }

    b->data[b->write_index] = d;
    b->write_index = next_location;
    return OK;
}

void initTimer(){
    TCCR1B = (1 << CS12) | (1 << CS10);
}

void initSerial(){
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

volatile uint8_t myChar;
uint8_t currentColor = NONE;
uint16_t lastTime;
volatile struct RingBuffer buffer;

int main(void){
    DDRB = (1 << RED) | (1 << BLUE);

    initSerial();
    initTimer();
    sei();

    lastTime = TCNT1;
    enum BufferStatus status = OK;

    while(1){
        uint16_t temp = TCNT1;
        uint16_t delta = temp - lastTime;
        if(delta >= INTERVAL){
            lastTime = temp;
            status = readRingBuffer(&buffer, &currentColor);
            if(status == EMPTY || currentColor == NONE){
                PORTB = 0;
            } else{
                PORTB = (1 << currentColor);
            }
        }
        _delay_ms(2);
    }

    return 0;
}

ISR(USART_RX_vect){

    volatile uint8_t tc;

    myChar = UDR0;

    if(myChar == 'R'){
        tc = RED;
    } else if(myChar == 'G'){
        tc = GREEN;
    } else if(myChar == 'B'){
        tc = BLUE;
    } else{
        tc = NONE;
    }

    writeRingBuffer(&buffer, tc);

    UCSR0B &= ~(1 << RXEN0); //This flushes any characters in the buffer that came in during the interval
    UCSR0B |= (1 << RXEN0); 
}

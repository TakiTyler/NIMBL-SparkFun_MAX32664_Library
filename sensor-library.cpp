#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "sensor-library.h"

// creating UART for debugging since printing doesn't seem to work
void initUART(){
    // configure uart pins, 1.4 = TX & 1.5 = RX
    P1SEL0 |= BIT4 | BIT5;
//    P1SEL0 &= ~(BIT4 | BIT5);

    // configure uart for 9600 baud
    UCA0CTLW0 |= UCSWRST; // eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK; // choose smclk

    UCA0BR0 = 104;
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0x1100;

    UCA0CTLW0 &= ~UCSWRST; // release from reset
}

void uart_write_char(char c){
    while(!(UCA0IFG & UCTXIFG)); // wait for tx buffer to be empty
    UCA0TXBUF = c;
}

void uart_write_string(const char* str){
    while(*str){
        uart_write_char(*str++);
    }
}

void uart_write_int(int num){
    char buf[10];
    int i = 0;
    if(num == 0){
        uart_write_char('0');
        return;
    }
    while(num > 0 && i < 10){
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while(--i >= 0){
        uart_write_char(buf[i]);
    }
}

int main(void){

    // stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // disable gpio
    PM5CTL0 &= ~LOCKLPM5;

    P1DIR |= BIT0;   // Set P1.0 to output direction
    P1OUT |= BIT0;   // Turn on the LED

    initUART();

    bioData mySensorData;
    version firmwareVersion;

    mySensorData.heartRate = 72;
    mySensorData.confidence = 98;
    mySensorData.oxygen = 99;

    firmwareVersion.major = 1;
    firmwareVersion.minor = 0;
    firmwareVersion.revision = 3;

    uart_write_string("--- Sensor Test ---\n");

    uart_write_string("Heart Rate: ");
    uart_write_int((int) mySensorData.heartRate);
    uart_write_string(" bpm\n");

    uart_write_string("Oxygen: ");
    uart_write_int((int) mySensorData.oxygen);
    uart_write_string("%%\n");

    uart_write_string("Firmware: v");
    uart_write_int((int) firmwareVersion.major);
    uart_write_string(".");
    uart_write_int((int) firmwareVersion.minor);
    uart_write_string(".");
    uart_write_int((int) firmwareVersion.revision);
    uart_write_string("\n");
    P1OUT &= ~BIT0;   // Turn on the LED

//    printf("--- Sensor Test ---\n");
//    printf("Heart Rate: %d bpm\n", (int) mySensorData.heartRate);
//    printf("Oxygen: %d%%\n", (int) mySensorData.oxygen);
//    printf("Firmware: v%d.%d.%d\n", (int) firmwareVersion.major, (int) firmwareVersion.minor, (int) firmwareVersion.revision);


    while(1);
}

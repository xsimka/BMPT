/*
---------------------------------------------
----------Remote Controller Program----------
---------------------------------------------
*/
/*interupt na logickou nulu
PD2 - INT0 - J7 pin6
0000 0100
     8421*/
/**
       @file
       @mainpage ProjektBMPT
       @authors Marek Šimka and Dominik Walach
       @date 3.12.2018
       @copyright (c) 2018 Marek Šimka and Dominik Walach
*/
/*-------------- Libraries --------------*/
#include "settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include "uart.h"
#include "lcd.h"

/**
 *  @brief Define UART buad rate.
 */
#define UART_BAUD_RATE 9600

/**
 *  @brief Define number of scanned samples
 */
#define N 250   // number of scanned samples

/* declaration of variables we going to use */
/**
 *  @brief String of samples
 */
volatile uint8_t sken[N];
/**
 *  @brief String for UART
 */
char uart_string[5];
/**
 *  @brief Declaration of sample counter
 */
volatile uint16_t citac;
/**
 *  @brief When value=1, all samples are in sken[N] string
 */
volatile uint8_t sken_ready;

/**
 *  @brief When value=0, starts saving data from port PD2
 */
volatile uint8_t INT0_enable;
//volatile uint8_t pom=0;

void init();

void init()
{
  /**
   *  @vrief Initialization of I/O ports, UART and LCD display
   */
  INT0_enable=1;
  sken_ready=0;
  DDRD&=~_BV(PD2);    // setting up port PD2 as an inpupt
  PORTD|=_BV(PD2);    // pull up aktivation on port PD2
  DDRB |= _BV(PB5);   // setting up port PB5 as an ouput
  PORTB &= ~_BV(PB5);  // out port for LCD diplay
  /*
  setting up uart
  */
  citac = 0;
  lcd_init(LCD_DISP_ON);  //switch on display
  uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));   // Initialize UART: asynchronous, 8-bit data, no parity, 1-bit stop
  lcd_command(1<<LCD_CGRAM);
  lcd_clrscr();     //lcd clear
  sei();      //allow interrupt
  EIMSK|=_BV(INT0);
}

ISR(INT0_vect) // when first zero comes in, start scaning
{
  /**
  @vrief This function starts when input PIN detects interrupt from our IR receiver
  */

  PORTB ^= _BV(PB5);    //turn on display on port PB5
  TIMSK0 |= _BV(TOIE0); // allow timer
  //cli();  // deny interrupt to do not start again before scan[N] is not full yet
  //citac = 0;
   while (citac <=N)  //scanning all N samples
   {
     sken[citac] = (PIND&0b00000100)>>2;  //determination if scanned value is 0 or 1
     citac ++ ;
     _delay_us(500);  //dealyed time between impuls, has been set from remote standard catalog
   }
  sken_ready=1; // when all samples are scanned, sken_ready=1
}

int main(void)
{
  /**
  @vrief Main function where we specify input signals and correct output values
  */
 uint8_t i;
 init();  //basic settings for all ports
 while(1)
 {
   if (sken_ready==1) // is sken is completed, then continue
   {
     for(i=0;i<N;i++) // scanning all gathered samples from 0 to N
     {
       itoa(sken[i], uart_string,10);   //convert sken[i] value into a decimal uart_string
     }
              uart_puts("\r\n");
              uart_puts("\r\n");
              sken_ready=0;
/* print all samples in uart, we used it for test communication
     for(i=0;i<N;i++)
     {
       itoa(sken[i], uart_string,10);
       uart_puts(uart_string);
     }
*/
/* Evaluation of all scanned samples into specific LCD outputs-------------*/
/*-- Phillips Remote --------------------------------------------------------*/
      if (((sken[9]&&sken[10]&&sken[44]&&sken[46]&&sken[48]&&
            sken[75]&&sken[76]&&sken[77]&&sken[78]&&sken[247]&&
            sken[249])==1)              // position of specific bits in scan[n], which has to be 1
        &&((sken[8]&&sken[11]&&sken[43]&&sken[45]&&sken[47]&&
            sken[49]&&sken[50])==0))    // position of specific bits in scan[n], which has to be 0
      {
        lcd_clrscr();                   // clear LCD
        uart_puts(" Tlacitko 1");       // display pressed number on UART
        lcd_puts("Ovladac Philips");    // display remote name on LCD
        lcd_gotoxy(0, 1);               // change to second line
        lcd_puts("Tlacitko 1");         // display pressed number on LCD
      }

      else if (((sken[47]&&sken[50]&&sken[51])==1)
              &((sken[49])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko ticho");
        lcd_puts("Ovladac Philips");
        lcd_gotoxy(0, 1);
        lcd_puts("Ztlumit TV");
      }

      else if (((sken[47]&&sken[51])==1)
             &&((sken[50]&&sken[75])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko vyp");
        lcd_puts("Ovladac Philips");
        lcd_gotoxy(0, 1);
        lcd_puts("Vypnout TV");
      }

      else if (((sken[47]&&sken[48])==1)
             &&((sken[49]&&sken[51]&&sken[75])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko zap");
        lcd_puts("Ovladac Philips");
        lcd_gotoxy(0, 1);
        lcd_puts("Zapnout TV");
      }

      else if (((sken[44]&&sken[46]&&sken[49]&&sken[50]&&sken[75]&&
                 sken[76]&&sken[77]&&sken[78])==1)
             &&((sken[43]&&sken[45]&&sken[47]&&sken[48])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 2");
        lcd_puts("Ovladac Philips");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 2");
      }

      else if (((sken[44]&&sken[46]&&sken[75]&&sken[76]&&sken[77]&&sken[78])==1)
             &&((sken[43]&&sken[45]&&sken[47]&&sken[48]&&sken[49]&&sken[50])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 3");
        lcd_puts("Ovladac Philips");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 3");
      }

      else if (((sken[44]&&sken[47]&&sken[48]&&sken[50]&&sken[75]&&
                 sken[76]&&sken[77]&&sken[78])==1)
             &&((sken[43]&&sken[45]&&sken[46]&&sken[49])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 4");
        lcd_puts("Ovladac Philips");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 4");
      }

      else if (((sken[44]&&sken[47]&&sken[50]&&sken[75]&&
                 sken[76]&&sken[77]&&sken[78])==1)
             &&((sken[43]&&sken[45]&&sken[46]&&sken[48]&&sken[49])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 5");
        lcd_puts("Ovladac Philips");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 5");
      }

/*-- OVP Remote -------------------------------------------------------------*/
      else if (((sken[241]&&sken[242]&&sken[245]&&sken[246]&&sken[247])==1)
             &&((sken[243]&&sken[244]&&sken[248]&&sken[249])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 1 OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 1");
      }

      else if (((sken[248]&&sken[249]&&sken[243]&&sken[244])==1)
             &&((sken[245]&&sken[246]&&sken[247])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 2 OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 2");
      }

      else if (((sken[244]&&sken[246]&&sken[247])==1)
             &&((sken[245]&&sken[248]&&sken[249])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 3 OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 3");
      }

      else if (((sken[242]&&sken[245]&&sken[248]&&sken[249]&&sken[238])==1)
             &&((sken[243]&&sken[244]&&sken[246]&&sken[247])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 4 OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 4");
      }

      else if (((sken[245]&&sken[246]&&sken[247]&&sken[2]&&sken[238])==1)
             &&((sken[248]&&sken[249])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko 5 OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Tlacitko 5");
      }

      else if (((sken[245]&&sken[248]&&sken[209]&&sken[249])==1)
             &&((sken[244]&&sken[246]&&sken[247])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko vypnuto OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Vypnout TV");
      }

      else if (((sken[245]&&sken[248]&&sken[249])==1)
             &&((sken[244]&&sken[246]&&sken[247]&&sken[209])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko zapnuto OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Zapnout TV");
      }

      else if (((sken[241]&&sken[242]&&sken[232]&&sken[233])==1)
             &&((sken[240]&&sken[234]&&sken[235])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko volume+ OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Zapnout TV");
      }

      else if (((sken[241]&&sken[242]&&sken[245])==1)
             &&((sken[243]&&sken[244]&&sken[248]&&sken[249])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko volume- OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Zapnout TV");
      }

      else if (((sken[240]&&sken[247]&&sken[245])==1)
             &&((sken[241]&&sken[242]&&sken[243]&&sken[244])==0))
      {
        lcd_clrscr();
        uart_puts(" Tlacitko ztlumit OVP");
        lcd_puts("Ovladac OVP");
        lcd_gotoxy(0, 1);
        lcd_puts("Ztlumit TV");
      }
    }   //end of scan ready condition
  }   //while (1)
}   //int main (void)

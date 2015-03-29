/* Copyright (C) 2015 Sergey Sharybin <sergey.vfx@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <pic18.h>
#include <pic18f4550.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000

#pragma config PLLDIV   = 5         /* 20Mhz external oscillator */
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         /* Clock source from 96MHz PLL/2 */
#pragma config FOSC     = INTOSC_EC
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
/* #pragma config CCP2MX   = ON */
#pragma config STVREN   = ON
#pragma config LVP      = OFF
/* #pragma config ICPRT    = OFF */
#pragma config XINST    = OFF
#pragma config CP0      = OFF
#pragma config CP1      = OFF
/* #pragma config CP2      = OFF */
/* #pragma config CP3      = OFF */
#pragma config CPB      = OFF
/* #pragma config CPD      = OFF */
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
/* #pragma config WRT2     = OFF */
/* #pragma config WRT3     = OFF */
#pragma config WRTB     = OFF
#pragma config WRTC     = OFF
/* #pragma config WRTD     = OFF */
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
/* #pragma config EBTR2    = OFF */
/* #pragma config EBTR3    = OFF */
#pragma config EBTRB    = OFF

#define CD  LATBbits.LB0
#define RD  LATBbits.LB1
#define WR  LATBbits.LB2
#define CE  LATBbits.LB3
#define RST LATBbits.LB4
#define MD2 LATEbits.LE1
#define FS  LATEbits.LE2

#define D   LATD

/* Register setting. */
#define LCD_SET_CURSOR_PTR         0b00100000
#define LCD_SET_OFFSET_REGISTER    0b00100010
#define LCD_SET_ADDR_PTR           0b00100100
/* Set control word. */
#define LCD_SET_TEXT_HOME_ADDR     0b01000000
#define LCD_SET_TEXT_AREA          0b01000001
#define LCD_SET_GRAPHICS_HOME_ADDR 0b01000010
#define LCD_SET_GRAPHICS_AREA      0b01000011
/* Mode set. */
#define LCD_MODE                   0b10000000
#define LCD_MODE_OR                0b00000000
#define LCD_MODE_EXOR              0b00000001
#define LCD_MODE_AND               0b00000011
#define LCD_MODE_TEXT_ATTR         0b00000100
#define LCD_MODE_INT_CG_ROM        0b00000000
#define LCD_MODE_EXT_CG_ROM        0b00001000
/* Display mode. */
#define LCD_DISP_MODE              0b10010000
#define LCD_DISP_MODE_OFF          0b10010000
#define LCD_DISP_MODE_BLINK        0b00000001
#define LCD_DISP_MODE_CURSOR       0b00000010
#define LCD_DISP_MODE_TEXT         0b00000100
#define LCD_DISP_MODE_GRAPHICS     0b00001000
/* Cursor pattern. */
/* TODO(sergey): Fill in. */
/* Data read/write. */
/* TODO(sergey): Fill in. */
/* Date auto read/write */
#define LCD_AUTO_WRITE_MODE        0b10110000
#define LCD_AUTO_READ_MODE         0b10110001
#define LCD_AUTO_MODE_RESET        0b10110010

#define LCD_SCREEN_PEEK            0b11100000
#define LCD_SCREEN_COPY            0b11101000

/* Bit set/reset. */
/* TODO(sergey): Fill in. */

void main(void) {
  //OSCCONbits.IRCF = 0b111;
  OSCCONbits.IRCF = 0b100;

  TRISA = 0b0000000;
  TRISB = 0b0000000;
  TRISC = 0b0000000;
  TRISD = 0b0000000;
  TRISE = 0b0000000;

  PORTA = 0b00000000;
  PORTB = 0b00000000;
  PORTC = 0b00000000;
  PORTD = 0b00000000;
  PORTE = 0b00000000;

  LATA = 0b00000000;
  LATB = 0b00000000;
  LATC = 0b00000000;
  LATD = 0b00000000;
  LATE = 0b00000000;

//#define NOP asm(" nop")
#define LCD_PREPARE() { RST = 0; } (void)0
#define LCD_INIT() \
  { \
    WR = 1; RD = 1; CE = 1; CD = 1; FS = 0; MD2 = 0; RST= 1; \
  } (void)0
#define LCD_WRITE(is_command, data) \
  { \
    D = data; CD = is_command; CE = 0; WR = 0; CE = 1; WR = 1; \
  } (void)0
#define LCD_COMMAND(command) LCD_WRITE(1, command)
#define LCD_DATA(data) LCD_WRITE(0, data)
#define LCD_COMMAND_ARGS2(cmd, data_lo, data_hi) \
  {  \
    LCD_DATA(data_lo); \
    LCD_DATA(data_hi); \
    LCD_COMMAND(cmd); \
  } (void)0
#define LCD_GRAPHICS_SIZE 0xf00
#define LCD_GRAPHICS_CLS(pattern) \
  { \
    int counter; \
    LCD_COMMAND_ARGS2(LCD_SET_ADDR_PTR, 0b00000000, 0b01000000); \
    LCD_COMMAND(LCD_AUTO_WRITE_MODE); \
    for (counter = 0; counter < LCD_GRAPHICS_SIZE; ++counter) { \
      LCD_WRITE(0, pattern); \
    } \
    LCD_COMMAND(LCD_AUTO_MODE_RESET); \
  } (void)0

  /* Perform a reset and init all output lines. */
  LCD_PREPARE();
  __delay_ms(100);
  LCD_INIT();

  /* Initialize state. */
  LCD_COMMAND_ARGS2(LCD_SET_TEXT_HOME_ADDR, 0b00000000, 0b00000000);
  LCD_COMMAND_ARGS2(LCD_SET_GRAPHICS_HOME_ADDR, 0b00000000, 0b01000000);
  LCD_COMMAND_ARGS2(LCD_SET_TEXT_AREA, 0b00011110, 0b00000000);
  LCD_COMMAND_ARGS2(LCD_SET_GRAPHICS_AREA, 0b0011110, 0b00000000);
  LCD_COMMAND(LCD_MODE|LCD_MODE_OR);
  LCD_COMMAND(LCD_DISP_MODE|LCD_DISP_MODE_GRAPHICS);

  LCD_GRAPHICS_CLS(0b00000000);

  while (1) {
    LCD_GRAPHICS_CLS(0b11001100);
    __delay_ms(500);
    LCD_GRAPHICS_CLS(0b00110011);
    __delay_ms(500);
  }
}

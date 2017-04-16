#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ILI9163C.h" // include other code
//#include<stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = ON // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void display_character(char m, int x, int y, unsigned short colorOn, unsigned short colorOff);
void display_message(char * msg, int x, int y,unsigned short colorOn, unsigned short colorOff);
void display_bar(int x, int y, int height, int barLength, int maxBarLength,unsigned short colorOn, unsigned short colorOff);


void display_character(char m, int x, int y, unsigned short colorOn, unsigned short colorOff){
    int col = 0;
    int row = 0;
    while(col < 5){
        row = 0;
        if(col+x < 128){
           while(row < 8){
               if(row+y < 128){
                   int xDisp = x + col;
                   int yDisp = y + row;
                   int c = (ASCII[m-0x20][col] >> row) & 1;
                   unsigned short color = colorOn;
                   if(c == 0){
                       color = colorOff;
                   }
                   LCD_drawPixel(xDisp, yDisp, color); 
               }
               row = row+1;
            } 
        }
        col=col+1;
    }
 
}

void display_message(char * msg, int x, int y, unsigned short colorOn, unsigned short colorOff){
    int i = 0;
    
    while(msg[i]){
        int xDisp = x+(i*6);
        display_character(msg[i],xDisp,y,colorOn,colorOff);
        i++;
    }
}

void display_bar(int x, int y, int height, int barLength, int maxBarLength, unsigned short colorOn, unsigned short colorOff){
    int xDisp = x;
    int yDisp = y;
    while(xDisp < x+barLength){
        yDisp = y;
        while(yDisp < y+height){
            LCD_drawPixel(xDisp, yDisp, colorOn);
            yDisp = yDisp + 1;
        }
        xDisp = xDisp + 1;
    }
    
    while(xDisp < x+maxBarLength){
        yDisp = y;
        while(yDisp < y+height){
            LCD_drawPixel(xDisp, yDisp, colorOff);
            yDisp = yDisp + 1;
        }
        xDisp = xDisp+1;
    }
    
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    __builtin_enable_interrupts();
    
    char message[30];
    int counterDisp = 0;
    _CP0_SET_COUNT(0);
    while(1) {  
        if(_CP0_GET_COUNT() > 300000*4){
            if(counterDisp > 100)
            {
                sprintf(message,"Hello world %d",counterDisp);
                display_message(message,28,32,BLACK,BLACK);
                counterDisp = 0;
            }
            sprintf(message,"Hello world %d",counterDisp);
            display_message(message,28,32,WHITE,BLACK);
            display_bar(14, 50, 10, counterDisp, 100, WHITE,BLACK);
            counterDisp = counterDisp+1;
            
            _CP0_SET_COUNT(0);
        }
    }
}

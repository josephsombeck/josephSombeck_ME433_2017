#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h> // math for sin function

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

// SS -> pin 16, RPB7, 0011
// SDO1 -> pin 24, RPB13, 0011
// SCK1 -> pin 25, 
#define CS LATBbits.LATB7 // chip select pin
// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

// channel is 0 (A) or 1 (B), voltage is the 8-bit output level
void setVoltage(unsigned char channel,unsigned char voltage){
    // send as (channel)111(4-high bits of voltage)
    // then send (4-low bits of voltage)0000
    //unsigned char highBits = ((channel<<7) | (0b01110000)) | (voltage >> 4);
    //unsigned char lowBits = (voltage  & 0b00001111) << 4;
    unsigned char highBits = (((channel<<7)) | (0b01110000)) | (voltage >> 4);
    unsigned char lowBits = voltage << 4;
    CS = 0; // set low
    spi_io(highBits);
    spi_io(lowBits);
    CS = 1; // set high
}

void initSPI1(){
    // setup spi1
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x5;            // baud rate to 10 MHz [SPI4BRG = (50000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKP = 0;
    SPI1CONbits.CKE = 0;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.SSEN = 0;
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 1
    
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
    
    __builtin_enable_interrupts();

    // do your TRIS and LAT commands here   
   // TRISBbits.TRISB14 = 0; // set clock as output
    
    RPB13Rbits.RPB13R = 0b0011; // SDO1 pin
    TRISBbits.TRISB13 = 0; // set as output
    SDI1Rbits.SDI1R = 0b0100; // SDI1 pin (nc for this circuit)
    TRISBbits.TRISB8 = 1; // set as input
    
    TRISAbits.TRISA4 = 0; // green led for on and running
    LATAbits.LATA4 = 1; // green led for on and running
    
    RPB15Rbits.RPB15R = 0b0011;; // chip select
    TRISBbits.TRISB7 = 0; // chip select is output
    CS = 1; // set high initially
    
    initSPI1();

    // initialize arrays of voltages to send
    // 255 = 3.3V, 0 = 0.0V
    char sineWaveChannel = 0;
    char triangleWaveChannel = 1;
    unsigned char sineWaveArray[1000]; 
    unsigned char triangleWaveArray[1000];
    int i = 0;
    while(i < 1000){
        float sineWaveVal = (sin(2*3.14*(i%100)/100.0)+1)/2.0;
        float triangleWaveVal = (i%200)/200.0;
        
        sineWaveArray[i] = sineWaveVal*255;
        triangleWaveArray[i] = triangleWaveVal*255;
               
                i = i + 1;
    }
    // we can now do things
    int BLINK_COUNT = 24000*2;
    int idx = 0;
    _CP0_SET_COUNT(0);
    int flag = 1;
    while(1){
        if(_CP0_GET_COUNT() >= BLINK_COUNT/2)
        {
            _CP0_SET_COUNT(0);
            idx = idx+1;
            if(idx >= 1000) {
                idx = 0;
            }
            
            setVoltage(sineWaveChannel,sineWaveArray[idx]);
            setVoltage(triangleWaveChannel,triangleWaveArray[idx]);
        }
    }
    
}


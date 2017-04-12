#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

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

unsigned char startByte = 0b01000000;
unsigned char startByteRead = 0b01000001;
unsigned char ioDirAddress = 0x00;
unsigned char ioDirections = 0b10000000;
unsigned char gpioAddress = 0x09;
unsigned char olatAddress = 0x0A;

void initExpander(){
    // set GP7 pin as input, and set GP0 pin as output
    i2c_master_start();
    i2c_master_send(startByte);
    i2c_master_send(ioDirAddress);
    i2c_master_send(ioDirections);
    i2c_master_stop();
}
 
void setExpander(unsigned char pin){
    i2c_master_start();
    i2c_master_send(startByte);
    i2c_master_send(olatAddress);
    i2c_master_send(pin);
    i2c_master_stop();
}

unsigned char getExpander(){
    i2c_master_start();
    i2c_master_send(startByte);
    i2c_master_send(gpioAddress);
    i2c_master_restart();
    i2c_master_send(startByteRead);
    unsigned char mOut= i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return mOut;
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

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0; // led for pic running
    LATAbits.LATA4 = 1;
    TRISBbits.TRISB4 = 1;
    
    __builtin_enable_interrupts();

    // set pins as digital
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;;

    // setup i2c
    i2c_master_setup();
    
    // setup expander
    initExpander();
    
    // if GP7 pin is high, then set GP0 pin to be high to turn LED on
    while(1) {
	    // read from GP7 pin
        unsigned char gpio = getExpander();
        if((gpio) == 0b10000000){ // if GP7 is high
            setExpander(0b00000001); // set pin GP0 as high
        }
        else {
            setExpander(0b00000000);
        }
            
        
    }
}

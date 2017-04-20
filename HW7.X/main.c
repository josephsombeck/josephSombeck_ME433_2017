#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ILI9163C.h" // include other code

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

unsigned char IMUaddress = 0b01101011;
unsigned char CTRL1_XL = 0x10;
unsigned char CTRL2_G = 0x11;
unsigned char CTRL3_C = 0x12;
unsigned char OUT_TEMP_L = 0x20;
unsigned char WHOAMI = 0x0F;

void display_character(char m, int x, int y, unsigned short colorOn, unsigned short colorOff);
void display_message(char * msg, int x, int y,unsigned short colorOn, unsigned short colorOff);
void display_bar_x(int x, int y, int height, int barLength, int maxBarLength,unsigned short colorOn, unsigned short colorOff);
void display_bar_y(int x, int y, int height, int barLength, int maxBarLength,unsigned short colorOn, unsigned short colorOff);


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

void display_bar_x(int x, int y, int height, int barLength, int maxBarLength, unsigned short colorOn, unsigned short colorOff){
    int xDisp = x;
    int yDisp = y;
    
    while(xDisp < x+maxBarLength){
        yDisp = y;
        while(yDisp < y+height){
            LCD_drawPixel(xDisp, yDisp, colorOff);
            yDisp = yDisp + 1;
        }
        xDisp = xDisp+1;
    }
    xDisp=x;
    while(xDisp > x-maxBarLength){
        yDisp = y;
        while(yDisp < y+height){
            LCD_drawPixel(xDisp, yDisp, colorOff);
            yDisp = yDisp + 1;
        }
        xDisp = xDisp-1;
    }
    xDisp = x;
    while((barLength > 0 && xDisp < x+barLength) || (barLength < 0 && xDisp>x+barLength)){
        yDisp = y;
        while(yDisp < y+height){
            LCD_drawPixel(xDisp, yDisp, colorOn);
            yDisp = yDisp + 1;
        }
        if(barLength > 0){
            xDisp = xDisp + 1;
        }
        else{
            xDisp=xDisp-1;
        }
    }

}

void display_bar_y(int x, int y, int height, int barLength, int maxBarLength, unsigned short colorOn, unsigned short colorOff){
    int xDisp = x;
    int yDisp = y;
    
    while(yDisp < y+maxBarLength){
        xDisp = x;
        while(xDisp < x+height){
            LCD_drawPixel(xDisp, yDisp, colorOff);
            xDisp = xDisp + 1;
        }
        yDisp = yDisp+1;
    }
    yDisp=y;
    while(yDisp > y-maxBarLength){
        xDisp = x;
        while(xDisp < x+height){
            LCD_drawPixel(xDisp, yDisp, colorOff);
            xDisp = xDisp + 1;
        }
        yDisp = yDisp-1;
    }
    yDisp = y;
    while((barLength > 0 && yDisp < y+barLength) || (barLength < 0 && yDisp>y+barLength)){
        xDisp = x;
        while(xDisp < x+height){
            LCD_drawPixel(xDisp, yDisp, colorOn);
            xDisp=xDisp + 1;
        }
        if(barLength > 0){
            yDisp=yDisp + 1;
        }
        else{
            yDisp=yDisp-1;
        }
    }

}

void initIMU(){
    i2c_master_start();
    i2c_master_send(IMUaddress<<1 | 0); // write
    i2c_master_send(CTRL1_XL);
    i2c_master_send(0b10000010); // set acceleraometer
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(IMUaddress<<1 | 0); // write
    i2c_master_send(CTRL2_G);
    i2c_master_send(0b10001000); // set acceleraometer
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(IMUaddress<<1 | 0); // write
    i2c_master_send(CTRL3_C);
    i2c_master_send(0b00000100); // set acceleraometer
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length){
    i2c_master_start();
    i2c_master_send(address<<1 | 0);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(address<<1 | 1);
    int i;
    for(i=0; i < length; i++){
        data[i]=i2c_master_recv();

        if(i < length-1){
            i2c_master_ack(0);
        }
        else{
            i2c_master_ack(1);
        }
    }
    i2c_master_stop();
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

    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    
    // setup i2c
    i2c_master_setup();
    
    // setup imu
    initIMU();
   
    int xdisp = 50;
    int ydisp = 50;
    unsigned char msg[30];
    // variable initialization
    
    unsigned char dataRaw[14];
    unsigned char *dataRawPointer;
    dataRawPointer = dataRaw;
    int lengthDataRaw = 14;
    short temperature = 0;
    short gyroX = 0;
    short gyroY = 0;
    short gyroZ = 0;
    short accelX = 0;
    short accelY = 0;
    short accelZ = 0;
    int xCenter = 63;
    int yCenter = 63;
    _CP0_SET_COUNT(0);

    while(1) {    
        // read from IMU
        if(_CP0_GET_COUNT() > 1200000){
            I2C_read_multiple(IMUaddress, OUT_TEMP_L, dataRawPointer, lengthDataRaw);
            //I2C_read_multiple(IMUaddress, CTRL3_C, dataRawPointer, 1);

            temperature = (dataRaw[1]<<8 | dataRaw[0]);
            gyroX = (dataRaw[3]<<8 | dataRaw[2]);
            gyroY = (dataRaw[5]<<8 | dataRaw[4]);
            gyroZ = (dataRaw[7]<<8 | dataRaw[6]);
            accelX = (dataRaw[9]<<8 | dataRaw[8]);
            accelY = (dataRaw[11]<<8 | dataRaw[10]);
            accelZ = (dataRaw[13]<<8 | dataRaw[12]);
            
          
            display_bar_x(xCenter, yCenter, 5, (-1*accelX)/3000.0*63, 66, WHITE, BLACK);
            display_bar_y(xCenter, yCenter, 5, (-1*accelY)/3000.0*63, 66, WHITE, BLACK);
            
            _CP0_SET_COUNT(0);
        }
        // display accelerations
        

    }
}

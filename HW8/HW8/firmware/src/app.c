/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
unsigned char IMUaddress = 0b01101011;
unsigned char CTRL1_XL = 0x10;
unsigned char CTRL2_G = 0x11;
unsigned char CTRL3_C = 0x12;
unsigned char OUT_TEMP_L = 0x20;
unsigned char WHOAMI = 0x0F;

/* TODO:  Add any necessary local functions.
*/
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

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
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
    _CP0_SET_COUNT(0);


}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
               
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
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
            while(PORTBbits.RB4 == 0) {}; // pause program until unpushed
        
            if(_CP0_GET_COUNT() > 1800000){
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
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1619
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

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
 */

#include "mcc_generated_files/mcc.h"

//-----Sensibilidad del sensor capacitivo
#define SENSIBILIDAD	0x0100

int sensor_mtouch(void);

/*
                         Main application
 */
void main(void) {
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    int sensor_value;

    while (1) {
        // Add your application code
        sensor_value=sensor_mtouch();	//muestreo del adc
		
		//Encendido del led sugune el estado del sensor
		if( sensor_value < SENSIBILIDAD)
			D4_SetHigh();	//Encendido LED prueba
		else
			D4_SetLow();	//Apagado LED prueba
			
			
		__delay_ms(2);	
    }
}

int sensor_mtouch(void)
{
	int value;

	  // 1.- Drive secondary channel to VDD as digital output.
	  RC2_TRIS= 0;
	  TRISC1= 0;	// pines como salida digital
      
      RC2_SetHigh();	//Canal secundario a Vdd
      
      // 2. Point ADC to the secondary VDD pin (charges CHOLD to VDD).
      ADC1_SetChannel(channel_Secundary); //Point ADC to the secondary VDD pin (charges CHOLD to VDD)
      
      // 3. Ground sensor line.
      LATC1 = 0; //Ground sensor line.

      // 4. Turn sensor line as input (TRISx = 1).
	  TRISC1 = 1;	//La linea del sensor como entrada

      // 5. Point ADC to sensor channel (voltage divider from sensor to CHOLD)
      value = ADC1_GetConversion(channel_mTouch);//Begin ADC conversion.
	
	  return value;

}
/**
 End of File
 */
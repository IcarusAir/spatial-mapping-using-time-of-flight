// 2DX3 FINAL PROJECT DELIVERABLE 1
// Written by Nicholas Mair (mairn1)
// Achieve a student-specific clock speed and have a motor turn 360 degrees as a response to a button press
// Every 11.25 degrees, LED D4 Flashes and a distance measurement is recorded

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long FallingEdges = 0;

void PortF_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};//allow time for clock to stabilize
GPIO_PORTF_DIR_R=0b00010001; //Make PF0 and PF4 outputs, to turn on LED's
GPIO_PORTF_DEN_R=0b00010001;
return;
}

void PortK_Init(void){ //LEDs
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9; //activate the clock for Port K
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){}; //allow time for clock to stabilize
	GPIO_PORTK_DIR_R = 0b00000011; // Make PK0 output
	GPIO_PORTK_DEN_R = 0b00000011; // Enable PK0
return;
}

void PortM_Interrupt_Init(void){
	
		FallingEdges = 0;       // initialize counter

	
		GPIO_PORTM_IS_R = 0x0;    	// (Step 1) PM0 is edge sensitive 
		GPIO_PORTM_IBE_R = 0x0;    	//     			PM0 is not both edges 
		GPIO_PORTM_IEV_R = 0x0;    	//     			PM0 falling edge event 
		GPIO_PORTM_ICR_R = 0x1;     // 					clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTM_IM_R = 0x1;     	// 					arm interrupt on PM0 by setting proper bit in IM register
    
		NVIC_EN2_R |= 0x00000100;           // (Step 2) enable interrupt 72 in NVIC (Bit 8 on EN2)
	
		NVIC_PRI18_R |= 0x000000A0; 				// (Step 4) set interrupt priority 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}


void PortM_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //activate the clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; //allow time for clock to stabilize
	GPIO_PORTM_DIR_R = 0b00000000; // Make PM0:PM3 inputs, reading if the button is pressed or not
	GPIO_PORTM_DEN_R = 0b00000001; // Enable PM0:PM3
return;
}



//Turns on D2, D1
void PortN_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; //activate the clock for Port N
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};//allow time for clock to stabilize
GPIO_PORTN_DIR_R=0b00000011; //Make PN0 and PN1 outputs, to turn on LED's
GPIO_PORTN_DEN_R=0b00000011; //Enable PN0 and PN1
return;
}


//Turns on D3, D4


//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortH_Init(void){ //Motor
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //activate the clock for Port H
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; //allow time for clock to stabilize
	GPIO_PORTH_DIR_R = 0b00001111; // Make PH0:PH3 outputs
	GPIO_PORTH_DEN_R = 0b00001111; // Enable PH0:PH3
return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}



//-------------MOTOR CODE---------------- For reference throughout the project, DELETE AFTER FINAL VERSION
//				GPIO_PORTH_DATA_R = 0b00001100;
//				SysTick_Wait10ms(speed);
//				GPIO_PORTH_DATA_R = 0b00000110;
//				SysTick_Wait10ms(speed);
//				GPIO_PORTH_DATA_R = 0b00000011;
//				SysTick_Wait10ms(speed);
//				GPIO_PORTH_DATA_R = 0b00001001;
//				SysTick_Wait10ms(speed);
//			} else if (dir == 1) { //1 is CW
//				GPIO_PORTH_DATA_R = 0b00001100;
//				SysTick_Wait10ms(speed);
//				GPIO_PORTH_DATA_R = 0b00001001;
//				SysTick_Wait10ms(speed);
//				GPIO_PORTH_DATA_R = 0b00000011;
//				SysTick_Wait10ms(speed);
//				GPIO_PORTH_DATA_R = 0b00000110;
//				SysTick_Wait10ms(speed);	


uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
uint16_t distanceData[500];
int pos = 0;

void GPIOM_IRQHandler() { //ISR for button
	uint8_t dataReady;
	uint16_t Distance;
	
	GPIO_PORTK_DATA_R = 0b00000001;
	for (int i = 0;i<32;i++) {	
		//Make a Scan
		while (dataReady == 0){
			status = VL53L1X_CheckForDataReady(dev, &dataReady);
      FlashLED3(1);
      VL53L1_WaitMs(dev, 5);
		}
		dataReady = 0;
	  
		//read the data values from ToF sensor
		status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
		FlashLED4(1); //Flashes D4 when interrupted by the angle measurement

		status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		distanceData[pos++] = Distance;
		
		//Turn motor for next step
		for(int i = 0;i<16;i++) {
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait(32000);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait(32000);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait(32000);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait(32000);

	  }

  }
	//Untwist Wires
	for(int i = 0;i<512;i++) {
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait(32000);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait(32000);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait(32000);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait(32000);
			
			//Code to transmit sensor data
	}
	//Set Interrupt flag
	GPIO_PORTM_ICR_R = 0b00000001;
}

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t wordData;
  
	int i=0;
	int frames = 2;
	int steps = 32;

	//call initializaiton functions
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortM_Init();
	PortM_Interrupt_Init();
	PortK_Init();
	PortN_Init();
	PortF_Init();

	
	// hello world!
	UART_printf("2DX3 FINAL PROJECT - Nicholas Mair - mairn1\r\n");
	
	
/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);
	
	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
	  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
		
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	UART_printf("Ready to Measure!\r\n"); //Lets us know that ToF init has finished
	
	//Start procedural code
	for (int j = 0;j<frames;j++) {
	 WaitForInt();
	 GPIO_PORTK_DATA_R = 0b00000000;
	 GPIO_PORTF_DATA_R = 0b00000000;
	}
	
	status = VL53L1X_StopRanging(dev); //Disable ranging
	
	
	
	
	//Serial Communication Code
	int input = 0;
	while(1){ //Polling method, change to interrupt?
			input = UART_InChar();
			if (input == 's')
				break;
	}
	//Send distanceData[] to PC
	for (int j = 0;j<frames;j++) {
		for (int k = 0;k<steps;k++) {
				sprintf(printf_buffer,"%d, ",distanceData[k+(j*steps)]);
				//send string to uart
				UART_printf(printf_buffer);
		}
		UART_printf("\r\n");
	}
	

	FlashAllLEDs();
}



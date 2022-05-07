/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

*/


//Ayesha Arshad - arshaa13 - 400320499
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

void PortH_Init(void){ //port for motor
	//Use PortH pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTH_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTH_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	
	return;
}

void PortM_Init(void){
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}


//Turns on D1
void PortN1_Init(void){ //use PortN for LED
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; //activate the clock for Port N
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};//allow time for clock to stabilize
GPIO_PORTN_DIR_R=0b00000010; //Make N1 outputs, to turn on LED
GPIO_PORTN_DEN_R=0b00000010; //Enable PN1
return;
}

void PortN_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x05;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTN_AFSEL_R &= ~0x05;     								// disable alt funct on PN0
  GPIO_PORTN_DEN_R |= 0x05;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x05;     								// disable analog functionality on PN0		
	
	GPIO_PORTN_DATA_R ^= 0b00000001; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTN_DATA_R ^= 0b00000001;	
	return;
}

void PortL_Init(void){
	//Use PortL to read button input 
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0x0;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTL_AFSEL_R &= ~0xFF;     							
  GPIO_PORTL_DEN_R |= 0xFF;        								
																									// configure PN1 as GPIO

  GPIO_PORTL_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}



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


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;


uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
uint16_t wordData;
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;
uint8_t dataReady;



void captureDist(){		
	//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//read the data values from ToF sensor
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);
    
		FlashLED4(1);

	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		//sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
		sprintf(printf_buffer,"%u\r\n", Distance);
		UART_printf(printf_buffer);
	  SysTick_Wait10ms(50);
  }


// ------------DUTY CYCLE CODE ----------------
float period = 0.5; //in seconds 
//gen one period of a PWM signal 
void DutyCycle_Percent(uint8_t duty){
		float percent;	
		percent = ((float)duty*1000)/(255); //255 -> RGB range //gives percentage up till 1000%
		int percent_int;	
		percent_int = (int)percent;
	
		float waitTime; 
	  waitTime = period/1000;  //time to wait for systick function
	
		GPIO_PORTN_DATA_R ^= 0b00000100; //turn on LED (toggle)
		SysTick_WaitVarTime(percent_int, waitTime); //waits percent_int*500us 
		GPIO_PORTN_DATA_R ^= 0b00000100; //turn off LED (toggle)
		SysTick_WaitVarTime(1000-percent_int, waitTime); //waits percent_int*500us  
}


//-------------------------------

uint32_t x = 1;//delay for systick function 
float w = 0.003; //wait for systick function in sec 0.002

int steps = 0;
int dir = -1; 


char stopCmd[] = "STOP\r\n";

//function that controls dir of rotation and num of steps (full step mode) 
void ctrlSpinFull(int dir, int steps){

	
	//counter clockwise
	if(dir == 1){
		//one full step
		for(int i=0; i<steps;i++){
			
			//for every 45 deg turn, capture distance measurement + flash LED
			if(((i%8) ==0)){ 
				GPIO_PORTN_DATA_R = 0b00000010;
				SysTick_Wait10ms(1); 
				GPIO_PORTN_DATA_R = 0b00000000;
				captureDist(); 
				
			}
			
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_WaitVarTime(x, w);
			
		}
	}
	//clockwise
	else if(dir == 0){
		//one full step
		for(int i=0; i<steps;i++){
			//for every 45 deg turn, capture distance measurement + flash LED	
			if(((i%8)==0)){ 
				GPIO_PORTN_DATA_R = 0b00000010;
				SysTick_Wait10ms(1); 
				GPIO_PORTN_DATA_R = 0b00000000;
				captureDist(); 

			}				
		
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_WaitVarTime(x, w);			
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_WaitVarTime(x, w);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_WaitVarTime(x, w);
			
			
			if((GPIO_PORTL_DATA_R&0b00000010)==0){ //if input button is pressed 
				GPIO_PORTN_DATA_R = 0b00000000;
				UART_printf(stopCmd); //send "STOP" to PC
				break;
			}
							
			
		}
}
	
}




int main(void) {

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init(); 
	PortN1_Init();
	PortM_Init();
	
	//PortN_Init();
	//PortL_Interrupt_Init(); //Initalize and configure the Interrupt on Port J
	//PortM_Interrupt_Init(); //Initalize and configure the Interrupt on Port J

	PortL_Init();
	
	int input = 0;
	
	

	
	// always wait for the transmission code from pc. if 's' recieved then send data		
		//wait for the right transmition initiation code
		while(1){
			input = UART_InChar();
			if (input == 's')
				break;
			
		}
	
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer); //UART is communication bewteen the PC and Microcontroller


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData); //I2C is between ToF sensor and MC

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState); //check if sensor booted 
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	//spin motor 

	while(1) {
		
	if((GPIO_PORTL_DATA_R&0b00000001)==0){ //if input button is pressed 
				dir = 0; 
				steps = 256; 
				ctrlSpinFull(dir,steps);  //start rotation
			
		} 
		//WaitForInt();	
	}
	
	uint8_t duty = 26;
	while(1){
		DutyCycle_Percent(duty);
	}
	
	
	
	VL53L1X_StopRanging(dev);
  while(1) {
	}

}

	/*
	call spin function 
		in spin function : 
		every 45 deg turn, capture distance measurement 
	
	*/

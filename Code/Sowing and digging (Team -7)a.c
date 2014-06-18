/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010

 Application example: Robot control over serial port via XBee wireless communication module 
 					  (located on the ATMEGA260 microcontroller adaptor board)

 Concepts covered:  serial communication
 
 Serial Port used: UART0

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:  	
 						
  Motion control:		L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1; 


  Serial Communication:	PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
						PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

						PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
						PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

						PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
						PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

						PORTJ 0 --> RXD3 UART3 receive available on microcontroller expansion socket
						PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expansion socket

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.

Commands:
			Keyboard Key   ASCII value	Action
				8				0x38	Forward
				2				0x32	Backward
				4				0x34	Left
				6				0x36	Right
				5				0x35	Stop
				7				0x37	Buzzer on
				9				0x39	Buzzer off

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 						options below figure 2.22 in the Software Manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
 	Rest of the things are the same. 

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

 4. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current 
	surge which can reset the microcontroller because of sudden fall in voltage. 
	It is a good practice to stop the motors for at least 0.5seconds before changing 
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/


#include<avr/io.h>
#include<avr/interrupt.h>
//#include<util/delay.h>
#include "lcd.c"
#include<math.h>
unsigned char ADC_Value;
volatile int shaftcountright=0;
volatile int shaftcountleft=0;
unsigned char flag = 0;
unsigned char Left_black_line ;
unsigned char Center_black_line ;
unsigned char Right_black_line ;
volatile int mov;
volatile int a;
unsigned char data; //to store received data from UDR1
volatile int h;


 ISR(INT5_vect)
 {
 cli();
 shaftcountright++;
sei();
  
 }
ISR(INT4_vect)
 {
 cli();
 shaftcountleft++;
 sei();
  
 }

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
	buzzer_pin_config();
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; //11059200 Hz
// UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;

}

unsigned char ADC_Conversion(unsigned char Ch) 
{ cli();
    unsigned char a; 
  if(Ch>7) 
 { 
  ADCSRB = 0x08; 
 } 
 Ch = Ch & 0x07;      
 ADMUX= 0x20| Ch;        
 ADCSRA = ADCSRA | 0x40;  //Set start conversion bit 
  while((ADCSRA&0x10)==0);  //Wait for ADC conversion to complete 
 a=ADCH; 
 ADCSRA = ADCSRA|0x10;   //clear ADIF (ADC Interrupt Flag) by writing 1 to it 
 ADCSRB = 0x00; 
  sei();
  return a; 
}

void init_l(void)
{
DDRE= DDRE & 0xEF;
PORTE= PORTE | 0x10;
}

void init_r(void)
{
DDRE= DDRE & 0xDF;
PORTE= PORTE | 0x20;
}

 void left_position_encoder_interrupt_init(void)
 {
 cli();
 EICRB=EICRB | 0x02;
EIMSK= EIMSK | 0x10;
sei();
 }
 
 void right_position_encoder_interrupt_init(void)
 {
 cli();
 EICRB=EICRB | 0x08;
EIMSK= EIMSK | 0x20;
sei();
 }

void motion_init(void)
{
DDRA= 0x0F;
PORTA=0x00;
//PORTA=PORTA & 0xF0;
DDRL=0x18;
PORTL=0x18;
}
void stop(void)
{
PORTA=0x00;
}
void forward(void)
{
PORTA=0x06;
}
void backward(void)
{
PORTA=0x09;
}
void right(void)
{
PORTA=0x0A;
}
void left(void)
{
PORTA=0x05;
}

void adc_pin_config (void) 
{ 
 DDRF = 0x00; //set PORTF direction as input 
 PORTF = 0x00; //set PORTF pins floating 
 DDRK = 0x00; //set PORTK direction as input 
 PORTK = 0x00; //set PORTK pins floating 
} 
 
 //Function to Initialize ADC 
void adc_init() 
{ 
 ADCSRA = 0x00; 
 ADCSRB = 0x00;  //MUX5 = 0 
 ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000 
 ACSR = 0x80; 
 ADCSRA = 0x86;  //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0 
}


void velocity(unsigned char left_motor,unsigned char right_motor)
{
OCR5AL=(unsigned char)left_motor;
OCR5BL=(unsigned char)right_motor;
}
void linear_distance_black(unsigned int distance)
{  init_devices();

	float reqshaftcount = 0;
	unsigned long int reqshaftcountint = 0;

	reqshaftcount = distance/ 5.338; // division by resolution to get shaft count
	reqshaftcountint = (unsigned long int) reqshaftcount;

	 shaftcountright = 0;
	
	while(1)
	{
		
		if(shaftcountright > reqshaftcountint)
  		{
  			break;
  		}
		Left_black_line = ADC_Conversion(3);	//Getting data of Left Black Line Sensor
		Center_black_line = ADC_Conversion(2);	//Getting data of Center Black Line Sensor
		Right_black_line = ADC_Conversion(1);	//Getting data of Right Black Line Sensor
flag=0;
		lcd_print(1,1,Left_black_line,3);
     
	if(Center_black_line>0x28)															// if bot get center sensor black then it will move straight
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_black_line>0x28) && (flag==0))											// if bot get left sensor black then it will move slightly left
		{
			flag=1;
			forward();
			velocity(70,150);
		}

		if((Right_black_line>0x28) && (flag==0))									// if bot get right sensor black then it will move slightly right
		{
			flag=1;
			forward();
			velocity(150,70);
		}

		if(Center_black_line<0x28 && Left_black_line<0x28 && Right_black_line<0x28)  // if bot get all  three  sensor white then it will turn 195 degree and call home function
		{ 
			velocity(150,150);
			stop();
			
			a=0;
			soft_right_2();
			angle_rotate(195);
  			home();
			break;
		}	
	
	}
	stop();
}

void linear_distance_black1(unsigned int distance)
{  init_devices();

	float reqshaftcount = 0;
	unsigned long int reqshaftcountint = 0;

	reqshaftcount = distance/ 5.338; // division by resolution to get shaft count
	reqshaftcountint = (unsigned long int) reqshaftcount;

	 shaftcountright = 0;
	
	while(1)
	{
		
		if(shaftcountright > reqshaftcountint)
  		{
  			break;
  		}
		Left_black_line = ADC_Conversion(3);	//Getting data of Left Black Line Sensor
		Center_black_line = ADC_Conversion(2);	//Getting data of Center Black Line Sensor
		Right_black_line = ADC_Conversion(1);	//Getting data of Right Black Line Sensor
flag=0;
		lcd_print(1,1,Left_black_line,3);
     
	if(Center_black_line>0x28) 														// // if bot get center sensor black then it will move straight
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_black_line>0x28) && (flag==0))										// if bot get left sensor black then it will move slightly left
		{
			flag=1;
			forward();
			velocity(70,150);
		}

		if((Right_black_line>0x28) && (flag==0))									// if bot get right sensor black then it will move slightly right
		{
			flag=1;
			forward();
			velocity(150,70);
		}

		if(Center_black_line<0x28 && Left_black_line<0x28 && Right_black_line<0x28) // if bot get all  three  sensor white then it will stop
		{ 
			stop();
			velocity(0,0);
		}	
	
	}
	stop();
}
void angle_rotate(unsigned int Degrees)
{
float ReqdShaftCount = 0;
unsigned long int ReqdShaftCountInt = 0;
ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
shaftcountright = 0;
shaftcountleft = 0;
while (1)
{
if((shaftcountright >= ReqdShaftCountInt) | (shaftcountleft >= ReqdShaftCountInt))
break;
}
stop(); //Stop robot
}
void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x05);
}
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}

void sow_dig()						// function for sowing and digging
{
  init_devices();					// initializing all devices
	a=1;
   unsigned int i = 0;

  servo_1(200);						// moving Digging arm to Initial position
 servo_3(0);						// moving sowing piston to Initial position	
_delay_ms(1000);					// delay for sorvo movment
 servo_1_free();					// free servo to save power
 servo_3_free();					// free servo to save power
while(a==1)
{
//forward();
linear_distance_black(mov);			// moving bot on black line according to the value of Global variable mov
stop();
_delay_ms(1000);

	
 for(i=0;i<1;i++)					// loop for digging movment of servo1(digging arm)
 {
  servo_1(170);
  _delay_ms(1000);
  servo_1(200);
_delay_ms(1000);
 servo_1_free();
  }

for(i=0;i<1;i++)					// loop for sowing work of servo2(sowing piston)
 {
  servo_3(130);
  _delay_ms(1000);
  servo_3(0);
_delay_ms(1000);
 servo_3_free();
  }

}
stop();
}

void home() 						// function which is made to came home
{
 linear_distance_black1(100000);    // function is made for come home with following black line Untill all three sensor get white
}
void servo1_pin_config (void)
{
 	DDRB  = DDRB | 0x20;  		//making PORTB 5 pin output
 	PORTB = PORTB | 0x20; 		//setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 	DDRB  = DDRB | 0x40;  		//making PORTB 6 pin output
 	PORTB = PORTB | 0x40; 		//setting PORTB 6 pin to logic 1
}



//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 	DDRB  = DDRB | 0x80;  		//making PORTB 7 pin output
 	PORTB = PORTB | 0x80; 		//setting PORTB 7 pin to logic 1
}


void port_init_servo(void)
{ 
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();

}

void init(){
cli();
motion_init();
init_r();
init_l();
left_position_encoder_interrupt_init();
right_position_encoder_interrupt_init();
adc_init();
adc_pin_config();
sei();
}
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void timer1_init(void)
{
 	TCCR1B = 0x00; 				//stop
 	TCNT1H = 0xFC; 				//Counter high value to which OCR1xH value is to be compared with
 	TCNT1L = 0x01;				//Counter low value to which OCR1xH value is to be compared with
 	OCR1AH = 0x03;				//Output compare eegister high value for servo 1
 	OCR1AL = 0xFF;				//Output Compare Register low Value For servo 1
 	OCR1BH = 0x03;				//Output compare eegister high value for servo 2
 	OCR1BL = 0xFF;				//Output Compare Register low Value For servo 2
 	OCR1CH = 0x03;				///Output compare eegister high value for servo 3
 	OCR1CL = 0xFF;				//Output Compare Register low Value For servo 3
 	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB; 				/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 									For Overriding normal port functionalit to OCRnA outputs.
				  				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C; 				//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

void init_devices_servo(void)
{
 	cli(); 						//disable all interrupts
 	port_init_servo();
 	timer1_init();
 	sei(); 						//re-enable interrupts 
}
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 3) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}
void servo_1_free (void) 	//makes servo 1 free rotating
{
 	OCR1AH = 0x03; 
 	OCR1AL = 0xFF; 			//Servo 1 off
}

void servo_2_free (void) 	//makes servo 2 free rotating
{
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF; 			//Servo 2 off
}

void servo_3_free (void) 	//makes servo 3 free rotating
{
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF; 			//Servo 3 off
} 

void sow_dig1()
{
 PORTA=0x06;
}

void function(int data)
{
	mov = data*0.875*20;          // modifying mov(global variable) according to data
	sow_dig();                   //calling sow_dig function
}

//INTERFACE CODE STARTS

char fcall[5][5];
int i =0, j = 0;
int botId;


// Pass the message to be passed as a string
void send_status(char msg[])
{
	int i;
	UDR0 = 0x28;
	for(i = 0; msg[i]!=0; i++) {
		while ( !( UCSR0A & (1<<UDRE0)) );
		UDR0 = msg[i];
	}
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = 0x29;
}

void function_caller()
{
	int val, par;
	val = atoi(fcall[1]);
	par = atoi(fcall[2]);
	switch(val) {
		case 1 : function(par); break;
		default: UDR0 = 0x26;
	}
}

//FORMAT "botId$funCode$par1$par2$par3#"
SIGNAL(SIG_USART0_RECV)								// this is the  zigbee module of Alex Project of common interface
{
	cli();
	data = UDR0;

	if(data == 0x23) // #
	{
		if(atoi(fcall[0]) == botId) {
			if(j != 0) {
				fcall[i][j] = 0;
				sei();
				function_caller();
				cli();
			}
			UDR0 = data;
		}
		i = 0;
		j = 0;
	}
	else if(data == 0x24) // $
	{
		fcall[i][j] = 0;
		i++;
		j = 0;
	}
	else
	{	
		fcall[i][j] = data;
		j++;
	}
	sei();
}


//INTERFACE CODE ENDS


/*
SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 
	
	UDR0 = data; 				//echo data back to PC
    
	
	
	
	  if(data == 0x38) //ASCII value of 8
		{
		   mov = 140;

			sow_dig();
//			PORTA=0x06;  //forward
		}

		if(data == 0x32) //ASCII value of 2
		{
			mov = 35;

			sow_dig();		
		}

		if(data == 0x34) //ASCII value of 4
		{
		mov = 70;

			sow_dig();
		}

		if(data == 0x36) //ASCII value of 6
		{
mov = 105;

			sow_dig();
		}

		if(data == 0x35) //ASCII value of 5
		{
		mov = 87;

			sow_dig();
		}

		if(data == 0x37) //ASCII value of 7
		{
		mov = 121;

			sow_dig();
		}

		if(data == 0x39) //ASCII value of 9
		{
		mov = 157;

			sow_dig();
		}

}
*/



//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 adc_init(); 
 port_init();  //Initializes all the ports
 uart0_init(); //Initailize UART1 for serial communiaction
 sei();   //Enables the global interrupts
}

//Main Function
int main(void)
{
 
init(); 
  timer1_init();
  timer5_init();
  init_devices_servo();
  lcd_set_4bit();
	lcd_init();
  lcd_port_config();
  init_devices();
  botId = 7;
while(1);
}


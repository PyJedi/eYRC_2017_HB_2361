// Team ID : 2361
// Author List : Vishnu Vardhan Raju, K Pranath Reddy
// Filename : Embedded_C.c
// Theme : Harvestor Bot
// Functions : linefollower, obstacle, xy_diff, decisionto_move, main 
// Global variables : N,S,E,W, direction, ShaftCountLeft, ShaftCountRight, Degrees, Apple_Lcount, 
// Apple_Mcount, Apple_Scount, Blueberry_Lcount, Blueberry_Mcount, Blueberry_Scount, Orange_Lcount, Orange_Mcount, Orange_Scount 

#define F_CPU 14745600

#define THRESHOLD 10
#define VELOCITY_MAX 250
#define VELOCITY_MIN 220
#define VELOCITY_LOW 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>          //included to support power function
#include "lcd.h"

#define N 1
#define S 2
#define E 3
#define W 4

  int direction = N; //variable to store robot initial direction
  long int ShaftCountLeft = 0; //to keep track of left position encoder
  long int ShaftCountRight = 0; //to keep track of right position encoder
  int Degrees; //to accept angle in degrees for turning

  unsigned int Apple_Lcount = 1;
  unsigned int Apple_Mcount = 1;
  unsigned int Apple_Scount = 1;
  unsigned int Blueberry_Lcount = 1;
  unsigned int Blueberry_Mcount = 1;
  unsigned int Blueberry_Scount = 1;
  unsigned int Orange_Lcount = 0;
  unsigned int Orange_Mcount = 2;
  unsigned int Orange_Scount = 0;

  int j,k,d1,d2;
  int y[5]={4,2,1,5};
  int x[5]={2,3,5,5};
  int direction;
  int l;
  int m;
  int Nx;
  int Ny;
  unsigned int pick =0;
  unsigned char data; //to store received data from UDR1

void port_init();
void timer5_init();
void velocity(  unsigned char,   unsigned char);
void motors_delay();

  unsigned char ADC_Conversion(  unsigned char);
  unsigned char ADC_Value;
  unsigned char Left_white_line =0;
  unsigned char Center_white_line =0;
  unsigned char Right_white_line =0;
  unsigned char sharpIR_1 =0;

//Here, first configuring ports for LCD,ADC,Motion_control
//Function to configure LCD Port
void lcd_port_config(void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; //all the LCD pin's set to logic 0 except PORTC 7
}

//ADC pin configuration
void ADC_pin_config(void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config(void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL & 0x18; //PL3 and PL4 pins are for velocity control using PWM
}
//Function to configure ports to enable buzzer
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//MOSFET switch port configuration
void MOSFET_switch_config(void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void servo1_pin_config()
{
	DDRB = DDRB | 0x20; //Making  PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}
void servo2_pin_config()
{
	DDRB = DDRB | 0x40; //Making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Now initializing Ports
//Function to initialize ports
void port_init()
{
	lcd_port_config();
	ADC_pin_config();
	motion_pin_config();
	buzzer_pin_config();
	MOSFET_switch_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	servo1_pin_config();
	servo2_pin_config();
}

//Initialize the ports
//TIMER1 initialization in 10 bit fast PWM mode
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz
void timer1_init(void)
{
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
	TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
	OCR1AH = 0x03;	//Output compare Register high value for servo 1
	OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
	OCR1BH = 0x03;	//Output compare Register high value for servo 2
	OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
	OCR1CH = 0x03;	//Output compare Register high value for servo 3
	OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
	ICR1H  = 0x03;
	ICR1L  = 0xFF;
	TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
					For Overriding normal port functionality to OCRnA outputs.
				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
	TCCR1C = 0x00;
	TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	//Here we are using only lower register in OCR5A and OCR5B for output comparision
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

//Initializing ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;   //MUX5 = 0
	ADMUX = 0x20;    //Vref = 5V external ----ADLAR = 1 ----MUX4:0 = 0000
	ADCSRA = 0x86;   //ADEN = 1 ----- ADIE = 1 -----ADPS2:0 =110
	ACSR = 0x80;
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}
//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
	UCSR2B = 0x00; //disable while setting baud rate
	UCSR2A = 0x00;
	UCSR2C = 0x06;
	UBRR2L = 0x5F; //set baud rate lo
	UBRR2H = 0x00; //set baud rate hi
	UCSR2B = 0x98;
}


//SIGNAL(SIG_USART2_RECV)
ISR(USART2_RX_vect) 		// ISR for receive complete interrupt
{
	data = UDR2;                                  //making copy of data from UDR2 in 'data' variable
	UDR2 = data;                                   //echo data back to PI
	if(data == 0x31)
	{
		Blueberry_Scount = Blueberry_Scount-1;
	}
	if(data == 0x32)
	{
		Orange_Scount = Orange_Scount -1;
	}
		//lcd_string("small_orange");
	if(data == 0x33)
	{
		Apple_Scount = Apple_Scount -1;
	}
		//lcd_string("small_apple");
	if(data == 0x34)
	{
		Blueberry_Mcount = Blueberry_Mcount-1;
		//lcd_string("medium_blueberry");
	}
	if(data == 0x35)
	{
		Orange_Mcount = Orange_Mcount -1;
		//lcd_string("medium_orange");
	}
	if(data == 0x36)
	{
		Apple_Mcount = Apple_Mcount -1;
		//lcd_string("medium_apple");
	}
	if(data == 0x37)
	{
		Blueberry_Lcount = Blueberry_Lcount-1;
		//lcd_string("large_blueberry");
	}
	if(data == 0x38)
	{
		Orange_Lcount = Orange_Lcount -1;
		//lcd_string("large_orange");
	}
	if(data == 0x39)
	{
		Apple_Lcount = Apple_Lcount -1;
		//lcd_string("large_apple");
	}
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}


//Function For ADC Conversion
  unsigned char ADC_Conversion(  unsigned char Ch)
{
	  unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sensor Values At Desired Row And Column Location on LCD
void print_sensor(unsigned char row, unsigned char coloumn,  unsigned char channel)
{

	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}
//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees/1.86)+35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char)PositionPanServo;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees/1.86)+35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}
//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.


void servo_1_free() //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}
void servo_2_free() //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
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



void turn_on_sharp234_wl (void) //turn on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG & 0xFB;
}

void turn_off_sharp234_wl (void) //turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG | 0x04;
}

void turn_on_sharp15 (void) //turn on Sharp IR range sensors 1,5
{
	PORTH = PORTH & 0xFB;
}

void turn_off_sharp15 (void) //turn off Sharp IR range sensors 1,5
{
	PORTH = PORTH | 0x04;
}

void turn_on_ir_proxi_sensors (void) //turn on IR Proximity sensors
{
	PORTH = PORTH & 0xF7;
}

void turn_off_ir_proxi_sensors (void) //turn off IR Proximity sensors
{
	PORTH = PORTH | 0x08;
}

void turn_on_all_proxy_sensors (void) // turn on Sharp 2, 3, 4, red LED of the white line sensors
                                      // Sharp 1, 5 and IR proximity sensor
{
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

void turn_off_all_proxy_sensors (void) // turn off Sharp 2, 3, 4, red LED of the white line sensors
                                       // Sharp 1, 5 and IR proximity sensor
{
	PORTH = PORTH | 0x0C; //set PORTH 3 and PORTH 1 pins to 1
	PORTG = PORTG | 0x04; //set PORTG 2 pin to 1
}

//Function for velocity control
void velocity (  unsigned char left_motor,   unsigned char right_motor)
{
	OCR5AL = (  unsigned char)left_motor;
	OCR5BL = (  unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (  unsigned char Direction)
{
	  unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibble for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

//order of motion --- LB , LF , RF , RB

void forward (void) //both wheels forward
{
	motion_set (0x06);
}

void backward (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void backward_left (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void backward_right (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}


void stop (void) //stop
{
	motion_set (0x00);
}

//Function used for turning robot by specified degrees
void angle_rotate(  int Degrees)
{
	float ReqdShaftCount = 0;
	  long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (  int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(  int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}



void forward_mm(unsigned int DistanceInMM)
{
	//Threshold for black line following
	//W=values read by white line sensors when they are on white surface  and B = values read by white line sensors when they are on black surface
	//Threshold value = ((W+B)/2)-(B/3)
	//forward();
	//linear_distance_mm(DistanceInMM);
	linefollower(DistanceInMM);
}

void back_mm(  int DistanceInMM)
{
	backward();
	linear_distance_mm(DistanceInMM);
	//linefollower(DistanceInMM);
}

void left_degrees(  int Degrees)
{
	if(Degrees == 180)
	{
		forward_mm(130);
		_delay_ms(250);
		// 88 pulses for 360 degrees rotation 4.090 degrees per count
		left(); //Turn left
		angle_rotate(Degrees);
	}
	else
	{
		forward_mm(75);
		_delay_ms(250);
		// 88 pulses for 360 degrees rotation 4.090 degrees per count
		left(); //Turn left
		angle_rotate(Degrees);
		_delay_ms(250);
		back_mm(77);
	}
}



void right_degrees(  int Degrees)
{
	if(Degrees == 180)
	{
		forward_mm(130);
		_delay_ms(250);
		// 88 pulses for 360 degrees rotation 4.090 degrees per count
		right(); //Turn right
		angle_rotate(Degrees);
	}
	else
	{
		forward_mm(75);
		_delay_ms(250);
		// 88 pulses for 360 degrees rotation 4.090 degrees per count
		right(); //Turn right
		angle_rotate(Degrees);
		_delay_ms(250);
		back_mm(77);
	}
}


void soft_left_degrees(  int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(  int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(  int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	backward_left(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(  int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	backward_right();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

//Function to initialize devices
void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	adc_init();
	timer1_init();
	timer5_init();
	uart2_init();
	sei();   //Enables the global interrupts
}

  int Sharp_GP2D12_estimation(  unsigned char adc_reading)
{
	float distance;
	  int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}
/*
 *Function name:    whiteline_values
 *Input:            no input
 *output:           no output
 *Logic:            none
 *Example call:     whiteline_values()
 */
void whiteline_values()
{
	Left_white_line=ADC_Conversion(3);
	Center_white_line=ADC_Conversion(2);
	Right_white_line=ADC_Conversion(1);
}
/*
 *Function name:   sharp_dist
 *Input:           no input
 *output:          sharpIR_1 -> Sharp IR sensor value
 *Logic:           none
 *Example call:    sharp_dist()
 */
float sharp_dist()
{
	unsigned char dist = ADC_Conversion(11);
	int sharpIR_1 = Sharp_GP2D12_estimation(dist);
	return sharpIR_1;
}
/*
 *Function name: linefollower
 *Input:         DistanceInMM -> distance to be covered in millimeters
 *output:        follow the black line
 *Logic:         If the sensor values are deviating from threshould values, they are
                 corrected using this function by turning in the direction opposite
                 to the direction of deviation
 *Example call:  linefollower(400)
 */

void linefollower(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	ShaftCountRight = 0;
	forward();
	unsigned int wr = 1;
	while(wr==1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		print_sensor(1,6,3);
		print_sensor(1,10,2);
		print_sensor(1,14,1);

		unsigned int flag = 0;

		if(Center_white_line > THRESHOLD)
		{
			flag =1;
			if(ShaftCountRight > ReqdShaftCountInt)
			{
				stop();
                break;
			}
			velocity(250,250);
			//velocity(VELOCITY_MAX,VELOCITY_MAX);
		}
		if((Left_white_line > THRESHOLD) && (flag==0))
		{
			flag =1;
			if(ShaftCountRight > ReqdShaftCountInt)
			{
				stop();
				break;
			}
			velocity(230,250);
			//velocity(VELOCITY_MIN,VELOCITY_MAX);
		}
		if((Right_white_line > THRESHOLD) && (flag==0))
		{
			flag =1;
			if(ShaftCountRight > ReqdShaftCountInt)
			{
				stop();
				break;
			}
			velocity(250,230);
			//velocity(VELOCITY_MAX,VELOCITY_MIN);
		if((Center_white_line < THRESHOLD) && (Left_white_line < THRESHOLD) && (Right_white_line < THRESHOLD) && (flag==0))
		{
			flag =1;
			if(ShaftCountRight > ReqdShaftCountInt)
			{
				stop();
				//lcd_print(2,8,ShaftCountRight,3);
				break;
			}
			velocity(0,0);
			//velocity(VELOCITY_LOW,VELOCITY_LOW);
		}


	}
}
/*
 *Function name: pluck
 *Input:         no input
 *output:        picking up the fruit
 *Logic:         first servo 2 is active and searches for the fruit, followed by
                 servo 3 catching and plucking the fruit
 *Example call:  pluck()
 */
void pluck()
{
	unsigned int i =0;
	servo_1(0);
	servo_2(0);
	_delay_ms(300);
	if(Apple_Lcount != 0)
	{ 
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
	if(Apple_Mcount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
	if(Apple_Scount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
	if(Orange_Lcount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
	if(Orange_Mcount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
	if(Orange_Scount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
	if(Blueberry_Lcount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(500);
		}
	}
	if(Blueberry_Mcount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
	if(Blueberry_Scount != 0)
	{
		for (i = 0 ; i <= 17; i++)
		{
			servo_2(i);
			_delay_ms(1000);
		}
	}
			
}

/*
 *Function name:  drop
 *Input:          no input
 *output:         drop fruit at the destination place of deposition
 *Logic:          servo 2 gets activated and releases the fruit in the deposition zone
 *Example call:   drop()
 */
void drop()
{
	unsigned int i = 0;
	for (i = 0 ; i <= 90; i++)
	{
		servo_1(i);
		_delay_ms(300);
	}
	servo_2_free();
	servo_1_free();

}
/*
 *Structure name: t x_y_diff
 *Input:          Nx -> x coordinate to be covered by bot
                  Ny -> y coordinate to be covered by bot
                  direction -> present direction of the bot
 *output:         bot movement in the arena
 *Logic:          we take cases when Nx is 0, Ny is 0 , both Nx and Ny not equal to 0 with different
                  directions defined as N,S,E and W. Accordingly code is written where the bot moves
                  in the arena with given values of Nx and Ny.
 *Example call:   struct t{2,3,400}
 */
struct t{
	  int Nx,Ny;
	  unsigned char direction;
};
struct t x_y_diff(  unsigned char direction,  int Nx,   int Ny,float sharpIR_1)
{
	if(Nx == 0)
	{
		if(direction == E)
		{
			float sharpIR_1 = sharp_dist();
			if(Ny>=0)
			{
				left_degrees(88);
				direction = N;
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					stop();
					Ny = Ny-1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					right_degrees(88);
					direction = E;
					forward_mm(360);
					stop();
					left_degrees(88);
					direction = N;
					stop();
					forward_mm(800);
					stop();
					left_degrees(88);
					direction = W;
					stop();
					forward_mm(360);
					stop();
					right_degrees(88);
					direction = N;
					Ny=Ny-2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
			if(Ny<0)
			{
				left_degrees(88);
				direction = S;
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					stop();
					Ny = Ny+1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					right_degrees(88);
					direction = W;
					stop();
					forward_mm(360);
					stop();
					left_degrees(88);
					direction = S;
					stop();
					forward_mm(800);
					stop();
					left_degrees(88);
					direction = E;
					stop();
					forward_mm(360);
					stop();
					right_degrees(88);
					direction = S;
					Ny=Ny+2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
		}
		if(direction == W)
		{
			float sharpIR_1 = sharp_dist();
			if(Ny>=0)
			{
				right_degrees(88);
				direction = N;
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					stop();
					Ny = Ny-1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					left_degrees(88);
					direction = W;
					stop();
					forward_mm(360);
					stop();
					right_degrees(88);
					direction = N;
					stop();
					forward_mm(800);
					stop();
					right_degrees(88);
					direction = E;
					stop();
					forward_mm(360);
					stop();
					left_degrees(88);
					direction = N;
					Ny=Ny-2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
			if(Ny<0)
			{
				left_degrees(88);
				direction = S;
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					stop();
					Ny = Ny+1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					left_degrees(88);
					stop();
					direction = E;
					forward_mm(360);
					stop();
					right_degrees(88);
					direction = S;
					stop();
					forward_mm(800);
					stop();
					right_degrees(88);
					direction = W;
					stop();
					forward_mm(360);
					stop();
					left_degrees(88);
					direction = S;
					Ny=Ny+2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
		}
		/*if(direction == N)
		{
			if(sharpIR_1>400)
			{
				forward_mm(360);
				Ny = Ny-1;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			else
			{
				left_degrees(88);
				direction = W;
				forward_mm(360);
				right_degrees(88);
				direction = N;
				forward_mm(800);
				right_degrees(88);
				direction = E;
				forward_mm(360);
				left_degrees(88);
				direction = N;
				Ny=Ny-2;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
		}*/
		if(direction == N)
		{
			if(Ny>=0)
			{
				sharpIR_1 = sharp_dist();
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					Ny = Ny-1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					left_degrees(88);
					direction =W;
					forward_mm(360);
					right_degrees(88);
					direction = N;
					forward_mm(800);
					right_degrees(88);
					direction = E;
					forward_mm(360);
					left_degrees(88);
					direction = N;
					Ny=Ny-2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
			if(Ny<0)
			{
				right_degrees(180);
				direction = S;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
		}
		if(direction == S)
		{
			if(Ny>=0)
			{
				right_degrees(180);
				direction = N;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Ny<0)
			{
				sharpIR_1 = sharp_dist();
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					Ny = Ny+1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					right_degrees(88);
					direction = W;
					forward_mm(360);
					left_degrees(88);
					direction = S;
					forward_mm(800);
					left_degrees(88);
					direction = E;
					forward_mm(360);
					right_degrees(88);
					direction = S;
					Ny = Ny+2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
		}
	}
	if(Ny == 0)
	{
		if(direction == N)
		{
			if(Nx>=0)
			{
				right_degrees(88);
				sharpIR_1 = sharp_dist();
				direction = E;
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					Nx = Nx-1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					left_degrees(88);
					direction =N;
					forward_mm(360);
					right_degrees(88);
					direction = E;
					forward_mm(800);
					right_degrees(88);
					direction = S;
					forward_mm(360);
					left_degrees(88);
					direction = E;
					Nx=Nx-2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
			if(Nx<0)
			{
				left_degrees(88);
				direction = W;
				sharpIR_1 = sharp_dist();
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					Nx = Nx+1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					left_degrees(88);
					direction = S;
					forward_mm(360);
					right_degrees(88);
					direction = W;
					forward_mm(800);
					right_degrees(88);
					direction = N;
					forward_mm(360);
					left_degrees(88);
					direction = W;
					Nx=Nx+2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
		}
		if(direction == S)
		{
			if(Nx>=0)
			{
				left_degrees(88);
				direction = E;
				sharpIR_1 = sharp_dist();
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					stop();
					Nx = Nx-1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					right_degrees(88);
					direction = S;
					forward_mm(360);
					left_degrees(88);
					direction = E;
					forward_mm(800);
					left_degrees(88);
					direction = N;
					forward_mm(360);
					right_degrees(88);
					direction = E;
					Nx=Nx-2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
			if(Nx<0)
			{
				right_degrees(88);
				direction = W;
				sharpIR_1 = sharp_dist();
				if(sharpIR_1 >400)
				{
					forward_mm(360);
					Nx = Nx+1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(sharpIR_1<400)
				{
					right_degrees(88);
					direction = N;
					forward_mm(360);
					left_degrees(88);
					direction = W;
					forward_mm(800);
					left_degrees(88);
					direction = S;
					forward_mm(360);
					right_degrees(88);
					direction = W;
					Nx = Nx+2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
			}
		}
		if(direction == E)
		{
			/*struct	q{
				int Nx;
				int Ny;
				int direction;
			};
			struct q sol = obstacle(direction,Nx,Ny,sharpIR_1);
			struct t res;
			res.Nx = sol.Nx;
			res.Ny = sol.Ny;
			res.direction = sol.direction;
			return res;*/
			if(Nx > 0 )
			{
			sharpIR_1 = sharp_dist();
			if(sharpIR_1>400)
			{
				forward_mm(360);
				Nx = Nx-1;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			else
			{
				left_degrees(88);
				direction = N;
				forward_mm(360);
				right_degrees(88);
				direction = E;
				forward_mm(800);
				right_degrees(88);
				direction = S;
				forward_mm(360);
				left_degrees(88);
				direction = E;
				Nx=Nx-2;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			}
			if(Nx < 0)
			{
				right_degrees(180);
				direction = W;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
		}
		if(direction == W)
		{
			if(Nx >= 0)
			{
				right_degrees(180);
				direction = E;
				struct t res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Nx<0)
			{
				sharpIR_1 = sharp_dist();
				if(sharpIR_1>400)
				{
					forward_mm(360);
					Nx = Nx+1;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				else
				{
					left_degrees(88);
					direction = S;
					forward_mm(360);
					right_degrees(88);
					direction = W;
					forward_mm(800);
					right_degrees(88);
					direction = N;
					forward_mm(360);
					left_degrees(88);
					direction = W;
					Nx=Nx+2;
					struct t res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
			}
			}
		}
	}
}
struct q{
	int Nx,Ny;
	  unsigned char direction;
};
/*
 *structure name: q obstacle
 *Input:          direction -> N,S,E or W
                  Nx -> x coordinate value i.e difference between x coordinates of destination and source
                  Ny -> y coordinate value i.e difference between y coordinates of destination and source
                  sharpIR_1 -> sharpIR sensor value
 *output:         movement of bot in case of obstacle i.e sharpIR_1 < 400
 *Logic:          if sharpIR_1 < 400, it means there is an obstacle in the way of the bot. These codes causes
                  movement in bot in case of obstacle and provides an alternative path for the movement of
                  bot in the arena.
 *Example call:   struct q obstacle( N,2,3,400)
 */


struct q obstacle(  int direction,  int Nx,   int Ny,  int sharpIR_1)
{
	sharpIR_1 = sharp_dist();
	lcd_print(2,10,sharpIR_1,3);
	if((sharpIR_1 > 400))
	{
		if(direction == N)
		{
			if (Ny>=0)
			{
				forward_mm(360);
				Ny=Ny-1;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if (Ny<0)
			{
				right_degrees(180);
				direction = S;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
		}
		if(direction == S)
		{
			if(Ny>= 0)
			{
				right_degrees(180);
				direction = N;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if ( Ny<0 )
			{
				forward_mm(360);
				stop();
				Ny = Ny+1;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
		}
		if(direction == E)
		{
			if(Nx>=0)
			{
				forward_mm(360);
				stop();
				Nx = Nx-1;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Nx<0)
			{
				right_degrees(180);
				direction = W;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
		}
		if(direction == W)
		{
			if(Nx>=0)
			{
				right_degrees(180);
				direction = E;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Nx<0)
			{
				forward_mm(360);
				stop();
				Nx = Nx+1;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
		}
	}
	else
	{
		if(direction == N)
		{
			right_degrees(88);
			whiteline_values();
			/*struct w whiteline = whiteline_values();
			Left_white_line = whiteline.Left_white_line;
			Center_white_line = whiteline.Center_white_line;
			Right_white_line = whiteline.Right_white_line;*/
			if(Right_white_line>=7 && Center_white_line >=7 && Left_white_line >=7) //important here on black line three values i have given as greater than 7 but in real it does not happen so i have to change
			{
				direction = E;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
			{
				right_degrees(88);
				whiteline_values();
				/*struct w whiteline = whiteline_values();
				Left_white_line = whiteline.Left_white_line;
				Center_white_line = whiteline.Center_white_line;
				Right_white_line = whiteline.Right_white_line;*/
				if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
				{
					direction = S;
					struct q res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
				{
					right_degrees(88);
					whiteline_values();
					/*struct w whiteline = whiteline_values();
					Left_white_line = whiteline.Left_white_line;
					Center_white_line = whiteline.Center_white_line;
					Right_white_line = whiteline.Right_white_line;*/
					if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
					{
						direction = W;
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
					if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
					{
						direction = W;
						stop();
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
				}
			}
		}
		else if(direction == S)
		{
			right_degrees(88);
			whiteline_values();
			/*struct w whiteline = whiteline_values();
			Left_white_line = whiteline.Left_white_line;
			Center_white_line = whiteline.Center_white_line;
			Right_white_line = whiteline.Right_white_line;*/
			if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
			{
				direction = W;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
			{
				right_degrees(88);
				whiteline_values();
				/*struct w whiteline = whiteline_values();
				Left_white_line = whiteline.Left_white_line;
				Center_white_line = whiteline.Center_white_line;
				Right_white_line = whiteline.Right_white_line;*/
				if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
				{
					direction = N;
					struct q res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
				{
					right_degrees(88);
					whiteline_values();
					/*struct w whiteline = whiteline_values();
					Left_white_line = whiteline.Left_white_line;
					Center_white_line = whiteline.Center_white_line;
					Right_white_line = whiteline.Right_white_line;*/
					if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
					{
						direction = E;
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
					if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
					{
						direction = E;
						stop();
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
				}
			}
		}
		else if(direction == E)
		{
			right_degrees(88);
			whiteline_values();
			/*struct w whiteline = whiteline_values();
			Left_white_line = whiteline.Left_white_line;
			Center_white_line = whiteline.Center_white_line;
			Right_white_line = whiteline.Right_white_line;*/
			if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
			{
				direction = S;
				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
			{
				right_degrees(88);
				whiteline_values();
				/*struct w whiteline = whiteline_values();
				Left_white_line = whiteline.Left_white_line;
				Center_white_line = whiteline.Center_white_line;
				Right_white_line = whiteline.Right_white_line;*/
				if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
				{
					direction = W;
					struct q res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
				{
					right_degrees(88);
					whiteline_values();
					/*struct w whiteline = whiteline_values();
					Left_white_line = whiteline.Left_white_line;
					Center_white_line = whiteline.Center_white_line;
					Right_white_line = whiteline.Right_white_line;*/
					if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
					{
						direction = N;
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
					if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
					{
						direction = N;
						stop();
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
				}
			}
		}
		else if(direction == W)
		{
			right_degrees(88);
			whiteline_values();
			/*struct w whiteline = whiteline_values();
			Left_white_line = whiteline.Left_white_line;
			Center_white_line = whiteline.Center_white_line;
			Right_white_line = whiteline.Right_white_line;*/
			if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
			{
				direction = N;

				struct q res;
				res.Nx = Nx;
				res.Ny = Ny;
				res.direction = direction;
				return res;
			}
			if(Right_white_line<=7 && Center_white_line <=7 && Left_white_line <=7)
			{
				right_degrees(88);
				whiteline_values();
				/*struct w whiteline = whiteline_values();
				Left_white_line = whiteline.Left_white_line;
				Center_white_line = whiteline.Center_white_line;
				Right_white_line = whiteline.Right_white_line;*/
				if(Right_white_line<=7 && Center_white_line >=7 && Left_white_line <=7)
				{
					direction = E;
					struct q res;
					res.Nx = Nx;
					res.Ny = Ny;
					res.direction = direction;
					return res;
				}
				if(Right_white_line<=7 && Center_white_line <=40 && Left_white_line <=40)
				{
					right_degrees(88);
					whiteline_values();
					/*struct w whiteline = whiteline_values();
					Left_white_line = whiteline.Left_white_line;
					Center_white_line = whiteline.Center_white_line;
					Right_white_line = whiteline.Right_white_line;*/
					if(Right_white_line<=40 && Center_white_line >=40 && Left_white_line <=40)
					{
						direction = S;
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
					if(Right_white_line<=40 && Center_white_line <=40 && Left_white_line <=40)
					{
						direction = S;
						stop();
						struct q res;
						res.Nx = Nx;
						res.Ny = Ny;
						res.direction = direction;
						return res;
					}
				}
			}
		}
	}
}
/*
 *Function name: decision_toMove_1
 *Input:         xd -> x coordinate of destination node
                 xp -> x coordinate of source node
                 yd -> y coordinate of destination node
                 yp -> y coordinate of source node
                 xf -> x coordinate of deposition zone's selected node
                 yf -> y coordinate of deposition zone's selected node
 *output:        bot movement
 *Logic:         Nx and Ny are defined and calculated through the difference in the destination and source coordinate
                 values which define the distance to be covered by the bot to reach the destination by the arena. These
                 values are used for bot movement.
 *Example call:  decision_toMove_1(1,2,3,4,5,6)
 */
void decision_toMove_1(int xd, int xp, int yd, int yp, int xf, int yf)
{
	unsigned char direction = N;
	int Nx=xd-xp;
	int Ny=yd-yp;

	while((Nx!=0) || (Ny!=0))
	{
		//forward_mm(360);
		float sharpIR_1 = sharp_dist();
		if (Nx == 0 && Ny != 0)
		{
			struct t values0= x_y_diff(direction,Nx,Ny,sharpIR_1);
			Nx = values0.Nx;
			Ny = values0.Ny;
			direction = values0.direction;
		}
		if (Ny == 0 && Nx != 0)
		{
			struct t values1= x_y_diff(direction,Nx,Ny,sharpIR_1);
			Nx = values1.Nx;
			Ny = values1.Ny;
			direction = values1.direction;
		}
		if(Nx != 0 && Ny !=0)
		{
			struct q values=obstacle(direction,Nx,Ny,sharpIR_1);
			Nx = values.Nx;
			Ny = values.Ny;
			direction = values.direction;
		}
		if(Nx == 0 && Ny == 0)
		{
			if(direction == W)
			{
				right_degrees(88);
				_delay_ms(15000);
				left_degrees(88);
				_delay_ms(15000);
			}
			else
			{
				left_degrees(88);
				_delay_ms(30000);
				right_degrees(88);
			}
			stop();
		}
	}
}
/*
 *Function name: decision_toMove_1
 *Input:         xd -> x coordinate of destination node
                 xp -> x coordinate of source node
                 yd -> y coordinate of destination node
                 yp -> y coordinate of source node
                 xf -> x coordinate of deposition zone's selected node
                 yf -> y coordinate of deposition zone's selected node
                 fruit_pick
 *output:        bot movement
 *Logic:         Nx,Nxx and Ny,Nyy are defined and calculated through the difference in the destination and source coordinate
                 values which define the distance to be covered by the bot to reach the destination by the arena. These
                 values are used for bot movement.
 *Example call:  decision_toMove_1(1,2,3,4,5,6,2)
 */

void decision_toMove(int xd, int xp, int yd, int yp, int xf, int yf,int fruit_pick)
{
	  unsigned char direction = N;
	  int Nx=xd-xp;
	  int Ny=yd-yp;
	  
	while((Nx!=0) || (Ny!=0))
	{
		//forward_mm(360);
		float sharpIR_1 = sharp_dist();
		//lcd_print(2,10,sharpIR_1,3);
		lcd_print(1,4,Nx,2);
		lcd_print(2,6,Ny,2);
		lcd_print(2,10,direction,1);
		if (Nx == 0 && Ny != 0)
		{
			struct t values0= x_y_diff(direction,Nx,Ny,sharpIR_1);
			Nx = values0.Nx;
			Ny = values0.Ny;
			direction = values0.direction;
		}
		if (Ny == 0 && Nx != 0)
		{
			struct t values1= x_y_diff(direction,Nx,Ny,sharpIR_1);
			Nx = values1.Nx;
			Ny = values1.Ny;
			direction = values1.direction;
		}
		if(Nx != 0 && Ny !=0)
		{
			struct q values=obstacle(direction,Nx,Ny,sharpIR_1);
			Nx = values.Nx;
			Ny = values.Ny;
			direction = values.direction;
			lcd_print(2,14,values.direction,2);
		}			
		if(Nx == 0 && Ny == 0)
		{
			//int revolve = 1;
			//int node_update_x = 0;
			//int node_update_y = 0;
				//left_degrees(88);
				//_delay_ms(15000);
				//right_degrees(88);
				if(direction == W && fruit_pick == 1)
				{
					right_degrees(88);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					left_degrees(88);
					_delay_ms(15000);
				}
				if(direction == W && fruit_pick == 3)
				{
					left_degrees(88);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					right_degrees(88);
					_delay_ms(15000);
				}
				if(direction == W && fruit_pick == 4)
				{
					left_degrees(180);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					left_degrees(88);
					direction = N;
					_delay_ms(15000);
				}
				if(direction == E && fruit_pick == 1)
				{
					left_degrees(88);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					right_degrees(88);
				}
				if(direction == E && fruit_pick == 3)
				{
					right_degrees(88);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					left_degrees(88);
				}
				if(direction == E && fruit_pick == 2)
				{
					right_degrees(180);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					right_degrees(88);
					direction = N;
				}
				if(direction == S && fruit_pick == 2)
				{
					right_degrees(88);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					left_degrees(88);
				}
				if(direction == S && fruit_pick == 4)
				{
					left_degrees(88);
					forward_mm(20);
					pluck();
					back_mm(20);
					_delay_ms(15000);
					left_degrees(88);
					direction = N;
				}
				
					 int Nxx=xf-xd;
					 int Nyy=yf-yd;
					 while((Nxx!=0) || (Nyy!=0))
					 {
						 //forward_mm(50);
						 float sharpIR_1 = sharp_dist();
						 lcd_print(2,10,sharpIR_1,3);
						 lcd_print(1,4,Nxx,2);
						 lcd_print(2,6,Nyy,2);
						 lcd_print(2,10,direction,1);
						 if (Nxx == 0 && Nyy != 0)
						 {
							 struct t values0= x_y_diff(direction,Nxx,Nyy,sharpIR_1);
							 Nxx = values0.Nx;
							 Nyy = values0.Ny;
							 direction = values0.direction;
						 }
						 if (Nyy == 0 && Nxx != 0)
						 {
							 struct t values1= x_y_diff(direction,Nxx,Nyy,sharpIR_1);
							 Nxx = values1.Nx;
							 Nyy = values1.Ny;
							 direction = values1.direction;
						 }
						 if(Nxx != 0 && Nyy !=0)
						 {
							 struct q values=obstacle(direction,Nxx,Nyy,sharpIR_1);
							 Nxx = values.Nx;
							 Nyy = values.Ny;
							 direction = values.direction;
							 lcd_print(2,14,values.direction,2);
						 }
						 if(Nxx == 0 && Nyy == 0)
						 {
							 stop();
							 drop();
						 }
					}
					//decision_toMove_1()
				 //}
								 					 
		}
			
		
	}		
}
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	int k =1;
	while(k==1)
    {  
		print_sensor(1,6,3);
		print_sensor(1,10,2);
		print_sensor(1,14,1);
		whiteline_values();
		
		float sharpIR_1 = sharp_dist();
		
		if(Center_white_line > 7 && Left_white_line > 7 && Right_white_line > 7)
		{
			decision_toMove(2,0,0,0,3,5,1);
			_delay_ms(1000);
			decision_toMove(3,3,1,5,3,5,2);
			_delay_ms(1000);
			decision_toMove(2,3,2,5,3,5,3);
			_delay_ms(1000);
			decision_toMove(1,3,1,5,3,5,4);
			_delay_ms(3000);
			decision_toMove(4,3,1,5,5,5,1);
			_delay_ms(1000);
			decision_toMove(5,5,2,5,5,5,2);
			_delay_ms(1000);
			decision_toMove(4,5,3,5,5,5,3);
			_delay_ms(1000);
			decision_toMove(3,5,2,5,5,5,4);
			_delay_ms(3000);
			left_degrees(88);
			_delay_ms(1000);
			decision_toMove(1,5,3,5,0,5,1);
			_delay_ms(1000);
			decision_toMove(2,0,4,5,1,5,2);
			_delay_ms(1000);
			right_degrees(88);
			_delay_ms(1000);
			decision_toMove(1,1,5,5,1,5,3);
			right_degrees(180);
			_delay_ms(3000);
			//pick();
			left_degrees(180);
			_delay_ms(1000);
			decision_toMove(0,1,4,5,0,5,4);
			buzzer_on();
			_delay_ms(10000);
			buzzer_off();
			k =2;
		}
				
	}
}
		




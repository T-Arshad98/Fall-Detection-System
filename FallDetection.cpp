#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
String incoming;

int wait_time = 20;
int requested = 0;

double overallAcc = 0.0;
double XZAcc = 0.0;
double YZAcc = 0.0;
double orient[3] = {0.0, 0.0, 0.0};
double linearAcc[3] = {0.0, 0.0, 0.0};

bool fallen = false;
bool alertSend = true;
bool fallAlert = false;
bool sendData = false;
bool moving = false;
bool stationary = false;
bool layingDown = false;
bool frontBackDown = false;
//bool straightDown = false;
//bool sideDown = false;

double printEvent(sensors_event_t* event, int requested) ;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


void setup(void)
{
	delay(2000);
	Serial.begin(76800);
	
	/* Initialise the sensor */
	if (!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		while (1);
	}
	
	DDRB |= (1<<PORTB0);
	PORTB &= ~(1<<PORTB0);
	DDRB |= (1<<PORTB1);
	PORTB &= ~(1<<PORTB1);
	DDRB |= (1<<PORTB2);
	PORTB &= ~(1<<PORTB2);
	
	PORTB |= (1<<PORTB0);
	PORTB |= (1<<PORTB1);
	PORTB |= (1<<PORTB2);
	delay(1000);
	PORTB &= ~(1<<PORTB0);
	PORTB &= ~(1<<PORTB1);
	PORTB &= ~(1<<PORTB2);
}

void loop(void)
{
	if (Serial.available() > 0)
	{
		incoming = Serial.readString();
		if (incoming == "detection\n")
		{
			PORTB |= (1<<PORTB1);
			fallAlert = true;
		}
		else if (incoming == "confirmed\n")
		{
			alertSend = true;
			fallen = false;
			frontBackDown = false;
			layingDown = false;
			stationary = false;
			moving = false;
			Serial.print("reset\n");
			PORTB &= ~(1<<PORTB0);
			PORTB &= ~(1<<PORTB2);
		}
		else if (incoming == "show\n")
		{
			sendData = true;
		}
		else if (incoming == "hide\n")
		{
			sendData = false;
		}
	}
	
	sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
	bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
	bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
	bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

	orient[0] = printEvent(&orientationData, 0);
	orient[1] = printEvent(&orientationData, 1);
	orient[2] = printEvent(&orientationData, 2);

	linearAcc[0] = printEvent(&linearAccelData,0);
	linearAcc[1] = printEvent(&linearAccelData,1);
	linearAcc[2] = printEvent(&linearAccelData,2);
	
	overallAcc = sqrt(sq(linearAcc[0]) + sq(linearAcc[1]) + sq(linearAcc[2]));
	YZAcc = sqrt( sq(linearAcc[1]) + sq(linearAcc[2]) );

	if (overallAcc > 1.5) //if device is accelerating in any direction
	{
		moving = true;
		if ( (YZAcc > 10) && (YZAcc < 20) )
		{
			if ( ( (linearAcc[1] < -7) && (linearAcc[1] > -15) ) && ( (linearAcc[2] < -5) && (linearAcc[2] > -12) ) || ( (linearAcc[2] > 5) && (linearAcc[2] < 12) ))
			{
				frontBackDown = true;	//patient is accelerating forwards/towards ground
			}
		}
	}
	else
	{
		moving = false;	//patient is not moving
	}
	
	if (fallAlert == true)	//if device is connected to phone app and can send/receive data
	{
		if (fallen == false)	//if patient has not been found to have fallen
		{
			if (frontBackDown == true)	//if person is accelerating down and forwards
			{
				if (wait_time >= 0)	//if number of cycles since start of this loop started is between 0 and 20
				{
					PORTB |= (1<<PORTB2);	//turn on blue LED
					if ((orient[2] < -150.0) || ((orient[2] > -45.0)))	//if person's orientation is close to horizontal
					{
						fallen = true;	//patient has been found to be falling
					}
					else
					{
						wait_time--;	//reduce number of remaining cycles to loop by 1 if not horizontal
					}
				}
				
				else    //if all cycles have been completed
				{
					PORTB &= ~(1<<PORTB2); //turn off blue LED
					frontBackDown = false;	//reset forwards/downwards acceleration status
					wait_time = 20;	//reset number of cycles to loop
				}
			}
			else if (moving == false)	//if patient is not moving
			{
				if ( ((orient[2] <= -160) || (orient[2] >= -30)) )	//if orientation is almost horizontal
				{
					stationary = false;
					layingDown = true;
				}
				else
				{
					stationary = true;
					layingDown = false;
				}
			}

		}
		
		if (alertSend == true)	//if can send alerts to phone
		{
			if (fallen == true)	//if patient has been found to have fallen
			{
				PORTB |= (1<<PORTB0);	//turn on red LED
				Serial.print("|detected|\n");	//send alert message to phone
				alertSend = false;	//turn off ability to send more alerts
			}
			else if ( (moving == true) && (layingDown == false))	//if patient is moving
			{
				Serial.print("|moving|\n");
			}
			else if ( (layingDown == true) && (moving == false) )	//if patient is laying down
			{
				Serial.print("|layingdown|\n");
			}
			else if (stationary == true)   //if patient is not moving
			{
				Serial.print("|stationary|\n"); //send alert message to phone
			}
		}
	}
	
	if (sendData == true)
	{
		delay(15);
		Serial.print("|orientX|");
		Serial.print(orient[0],2);
		Serial.println("|");
		delay(15);
		Serial.print("|orientY|");
		Serial.print(orient[1],2);
		Serial.println("|");
		delay(15);
		Serial.print("|orientZ|");
		Serial.print(orient[2],2);
		Serial.println("|");
		delay(15);
		
		Serial.print("|linAccX|");
		Serial.print(linearAcc[0],2);
		Serial.println("|");
		delay(15);
		Serial.print("|linAccY|");
		Serial.print(linearAcc[1],2);
		Serial.println("|");
		delay(15);
		Serial.print("|linAccZ|");
		Serial.print(linearAcc[2],2);
		Serial.println("|");
		delay(15);
		
		Serial.print("|totAcc|");
		Serial.print(overallAcc,2);
		Serial.println("|");
		delay(15);

	}
	else
	{
		delay(BNO055_SAMPLERATE_DELAY_MS);
	}
	
	
	uint8_t system, gyro, accel, mag = 0;
	bno.getCalibration(&system, &gyro, &accel, &mag);
	
}

double printEvent(sensors_event_t* event, int requested) {
	double data[3] = {-1000, -1000 , -1000}; //dumb values, easy to spot problem
	if (event->type == SENSOR_TYPE_ACCELEROMETER) {
		data[0] = event->acceleration.x;
		data[1] = event->acceleration.y;
		data[2] = event->acceleration.z;
		
	}
	else if (event->type == SENSOR_TYPE_ORIENTATION) {
		data[0] = event->orientation.x;
		data[1] = event->orientation.y;
		data[2] = event->orientation.z;

	}
	else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
		data[0] = event->magnetic.x;
		data[1] = event->magnetic.y;
		data[2] = event->magnetic.z;
	}
	else if (event->type == SENSOR_TYPE_GYROSCOPE) {
		data[0] = event->gyro.x;
		data[1] = event->gyro.y;
		data[2] = event->gyro.z;
	}
	else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
		data[0] = event->gyro.x;
		data[1] = event->gyro.y;
		data[2] = event->gyro.z;
	}
	else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
		data[0] = event->acceleration.x;
		data[1] = event->acceleration.y;
		data[2] = event->acceleration.z;
	}
	
	return data[requested];
}



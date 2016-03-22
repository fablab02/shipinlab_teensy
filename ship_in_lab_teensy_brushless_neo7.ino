

#include <NewPing.h>
#include "gps_reader.h"




char c;


//
// Ship In Lab
//
//
// ARDUINO (UNO) SETUP:
// =======================
// Ping sensor = 5V, GRND, D11 (for both trigger & echo)
// LCD Display = I2C :  SCL (A5) &  SDA (A4)
// Adafruit GPS = D7 & D8 (GPS Shield, but pins used internally) 
// IR Receiver = D5
// Adafruit Magnetometer Adafruit_HMC5883 = I2C :  SCL (A5) &  SDA (A4)
// SD Card (D10, D11, D12, D13)
// drive pin 9

//ARDUINO (MEGA) SETUP:
//=======================
// Ping sensor = 5V, GRND, D11 (for both trigger & echo)
// LCD Display = I2C :  SCL  &  SDA 
// Adafruit GPS = D14 & D15 (GPS Shield, but pins used internally) 
// IR Receiver = D44
// Adafruit Magnetometer Adafruit_HMC5883 = I2C :  SCL &  SDA 
// SD Card (D10, D11, D12, D13)
// drive pin 3
// drive pin 4


//TEENSY SETUP:
//=======================
// Ping sensor = 5V, GRND, trig D11 echo d12
// LCD Display = I2C :  SCL  &  SDA 
// GPS NEO = D9 & D10 (GPS Shield, but pins used internally) 
// IR Receiver = D5
// Adafruit Magnetometer Adafruit_HMC5883 = I2C :  SCL &  SDA 
// SD Card (D10, D11, D12, D13)
// drive pin 3
// drive pin 4


#include <Servo.h>
#include <Wire.h>                                 // used by: motor driver
//#include <Adafruit_MotorShield.h>                 // motor driver
//#include "utility/Adafruit_PWMServoDriver.h"      // motor driver
//#include <NewPing.h>                              // Ping sonar
#include <LiquidCrystal_I2C.h>                    // LCD library
#include <Adafruit_Sensor.h>                      // part of mag sensor
#include <Adafruit_HMC5883_U.h>                   // mag sensor
#include "waypointClass.h"                        // custom class to manaage GPS waypoints
//#include <Adafruit_GPS.h>                         // GPS
//#include <SoftwareSerial.h>                       // used by: GPS
#include <math.h>                                 // used by: GPS
#include "moving_average.h"                       // simple moving average class; for Sonar functionality
#include <PString.h>                            // PString class, for "message" variable; LCD display


// Select optional features
// COMMENT OUT IF NOT DESIRED, don't just change to "NO"
#define USE_GRAPHING YES            // comment out to skip graphing functions in LCD display
#define USE_LCD_BACKLIGHT YES       // use backlight on LCD; commenting out may help in direct sunlight
#define DEBUG YES                   // debug mode; uses Serial Port, displays diagnostic information, etc. 
//#define USE_IR NO                // comment out to skip using the IR sensor/remote
//#define NO_GPS_WAIT YES           // define this for debugging to skip waiting for GPS fix


// Setup magnemeter  (compass); uses I2C
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
sensors_event_t compass_event;


// Create the motor shield object with the default I2C address
//Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 


// Setup motor controllers for both drive and steering (turn).   
/*Adafruit_DCMotor *turnMotor = AFMS.getMotor(1);
  Adafruit_DCMotor *driveMotor = AFMS.getMotor(3);*/

#define TURN_LEFT 45
#define TURN_RIGHT 135
#define TURN_STRAIGHT 90


//define servos
Servo Direction;
Servo Drive_motor;


// LCD Display
LiquidCrystal_I2C lcd(0x20, 20, 4);  // Set the LCD I2C address and size (4x20)
#define LEFT_ARROW 0x7F
#define RIGHT_ARROW 0x7E
#define DEGREE_SYMBOL 0xDF
//char lcd_buffer[20];      
//PString message(lcd_buffer, sizeof(lcd_buffer));    // holds message we display on line 4 of LCD

// drive_motor
//const int drive_motor= 9;
//const int inv_drive_motor=4;


// Ultrasonic ping sensor
#define TRIGGER_PIN  11      
#define ECHO_PIN  12         
#define MAX_DISTANCE_CM 250                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches
int sonarDistance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);     // NewPing setup of pins and maximum distance.
MovingAverage <unsigned int, 3> sonarAverage(MAX_DISTANCE_IN);       // moving average of last n pings, initialize at MAX_DISTANCE_IN


// Compass navigation
int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading


// GPS Navigation
#define GPSECHO true           // set to TRUE for GPS debugging if needed
//#define GPSECHO true           // set to TRUE for GPS debugging if needed
//SoftwareSerial mySerial(50, 51);    // digital pins 7 & 8
HardwareSerial mySerial = Serial2;
GpsReader gpsReader;
//Adafruit_GPS GPS(&mySerial);
//boolean usingInterrupt = false;
double currentLat,
       currentLong,
       targetLat,
       targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it


// Waypoints in decimal degres
//waypoint jaulgonne  res _current 49.0962600708, 3.5357027054 ,LAT = 49.0961647034, 3.5356161594 , LAT = 49.0961914063, 3.5356535912
// waypoint fablab  interieur LAT =  49.0403060913, 3.4082915783   49.0402221680, 3.4079160690   49.0402641296, 3.4080543518
#define WAYPOINT_DIST_TOLERANE  2   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 5          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypointClass waypointList[NUMBER_WAYPOINTS] = {
    waypointClass(3.4082915783, 49.0403060913) 
    , waypointClass(3.4079160690, 49.0402221680)
    , waypointClass(3.4082915783, 49.0403060913)
    , waypointClass(3.4079160690, 49.0402221680)
    , waypointClass(3.4080543518, 49.0402641296)
};


// Steering/turning 
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;


// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 40
#define STOP_DISTANCE 12


// Speeds (range: 0 - 255)
#define FAST_SPEED 1750
#define NORMAL_SPEED 1700
#define TURN_SPEED 1675
#define SLOW_SPEED 16250
int spd = NORMAL_SPEED;



// IR Receiver
#ifdef USE_IR
#include "IRremote.h"            // IR remote
#define IR_PIN 6
IRrecv IR_receiver(IR_PIN);           // create instance of 'irrecv'
decode_results IR_results;            // create instance of 'decode_results'
#endif


// IR result codes
#define IR_CODE_ONOFF 0xFD00FF 
#define IR_CODE_VOLPLUS 0xFD807F  
#define IR_CODE_FUNCSTOP 0xFD40BF
#define IR_CODE_LAST 0xFD20DF
#define IR_CODE_PLAYPAUSE 0xFDA05F
#define IR_CODE_NEXT 0xFD609F
#define IR_CODE_DOWN 0xFD10EF
#define IR_CODE_VOLMOINS 0xFD906F
#define IR_CODE_UP 0xFD50AF
#define IR_CODE_EQ 0xFDB04F
#define IR_CODE_STREPT 0xFD708F
#define IR_CODE_1 0xFD08F7  
#define IR_CODE_2 0xFD8877       
#define IR_CODE_3 0xFD48B7   
#define IR_CODE_4 0xFD28D7   
#define IR_CODE_5 0xFDA857 
#define IR_CODE_6 0xFD6897 
#define IR_CODE_7 0xFD18E7 
#define IR_CODE_8 0xFD9867
#define IR_CODE_9 0xFD58A7
#define IR_CODE_0 0xFD30CF     

/*

// 
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) 
{
GPS.read();
}


//
// turn interrupt on and off
void useInterrupt(boolean v) 
{
if (v) {
// Timer0 is already used for millis() - we'll just interrupt somewhere
// in the middle and call the "Compare A" function above
OCR0A = 0xAF;
TIMSK0 |= _BV(OCIE0A);
usingInterrupt = true;
} else {
// do not call the interrupt function COMPA anymore
TIMSK0 &= ~_BV(OCIE0A);
usingInterrupt = false;
}
}
 */
//boolean usingInterrupt = false;
/*void useInterrupt(boolean)// Func prototype keeps Arduino 0023 happy
  {
  } */
void setup() 
{

	delay(3000);
	Drive_motor.attach(22);
	Direction.attach(23);
	Direction.write(90);
	Drive_motor.writeMicroseconds(1500);

	//pinMode (drive_motor,OUTPUT);
	//pinMode (inv_drive_motor,OUTPUT);
	// turn on serial monitor 
	Serial.begin(115200);        // we need this speed for the GPS
	Serial.println("ok1");

	//
	// Start LCD display
	lcd.begin();            // start the LCD...new version doesn't require size startup parameters
#ifdef USE_LCD_BACKLIGHT
	lcd.backlight();
#else
	lcd.noBacklight();      
#endif
	lcd.clear();
#ifdef USE_GRAPHING
	createLCDChars();  // initialize LCD with graphing characters
#endif


	//
	//
	// start Mag / Compass
	if(!compass.begin()) {
		lcd.print(F("COMPASS ERROR"));
		loopForever(); 

#ifdef DEBUG
		Serial.println(F("COMPASS ERROR"));
#endif
		/* lcd.print(F("COMPASS ERROR"));
		   loopForever(); */        // loop forever, can't operate without compass
	}
	lcd.setCursor(0, 1);
	lcd.print(F("COMPASS OK"));
	delay(1000);

	//
	// start GPS and set desired configuration
	Serial2.begin(9600);                                // 9600 NMEA default speed
	Serial2.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
	Serial2.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
	Serial2.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
	Serial2.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
	Serial2.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
	Serial2.print("$PUBX,40,RMC,0,0,0,0*47\r\n");

	Serial2.flush();
	Serial2.clear();
	/*  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);     // turns on RMC and GGA (fix data)
	    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);       // 1 Hz update rate
	    GPS.sendCommand(PGCMD_NOANTENNA);     // turn off antenna status info
	    GPS.sendCommand(PMTK_CMD_AIC_MODE);
	// GPS.sendCommand(PMTK_ENABLE_WAAS);
	GPS.sendCommand(PMTK_ENABLE_SBAS);
	useInterrupt(false);                            // use interrupt to constantly pull data from GPS
	delay(1000);
	 */
	//
	// Wait for GPS to get signal
#ifndef NO_GPS_WAIT
	lcd.setCursor(0, 0);
	lcd.print(F("Waiting for GPS"));
	unsigned long startTime = millis();
	while (!Serial2.available())                      // wait for fix, updating display with each new NMEA sentence received
	{
		lcd.setCursor(0, 1);
		lcd.print(F("Wait Time: "));
		lcd.print((int) (millis() - startTime) / 1000);     // show how long we have waited  
		if (!Serial2.available()){
			char c = Serial2.read();
		Serial.println(F(c));
		//   GPS.parse(GPS.lastNMEA());      
	}
}// while (!GPS.fix)
	//delay(1000);
#endif


	//   
	// Start the IR receiver
#ifdef USE_IR
	IR_receiver.enableIRIn(); // Start the receiver
	// Wait for operator to press key to start moving
	lcd.setCursor(0, 3);
	lcd.print(F("Press key to start"));  
	while(!IR_receiver.decode(&IR_results)) ;   // wait for key press
	/*  if (IR_results.value==IR_CODE_0) {
	    lcd.clear();

	    lcd.print(F("config_moteur"));
	    Drive.writeMicroseconds(FAST_SPEED);
	    lcd.setCursor(0, 1);
	    lcd.print("brancher batterie");
	    lcd.setCursor(0, 2);
	    lcd.print(F(" attendre bip"));
	    lcd.setCursor(0, 3);
	    lcd.print(F(" et appuyer sur 1"));
	    IR_receiver.resume();
	    while(!IR_receiver.decode(&IR_results)) {Drive.writeMicroseconds(FAST_SPEED);};  // wait for key press

	    Drive.writeMicroseconds(SLOW_SPEED); 

	    delay(5000);
	    }*/
	IR_receiver.resume();  // get ready for any additional input
#else
	
	// fi""""""""""""""""""""""""""""""                                                                                                                                                                                "    """                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                n config brushless

	lcd.clear();
	lcd.print(F("GPS Acquired"));
	lcd.setCursor(0, 1);
	lcd.print(F("Starting in..."));
	lcd.setCursor(0, 2);
        while(gpsReader.m_lastFix == 0){ 
            lcd.setCursor(0, 2);
            lcd.print(F("Wait GPS fix..."));
      	    gpsReader.readNextFrame();
        }
	for (int i = 10; i > 0; i--)
	{
		lcd.print(i);
		lcd.print(F(" "));
		gpsReader.readNextFrame();
                double lastLatitudeDeg = gpsReader.m_lastLatitudeDeg;
                double lastLongitudeDeg = gpsReader.m_lastLongitudeDeg;
    
                Serial.print("res2 ");
                Serial.print(gpsReader.m_lastLatitudeDeg, 10);
	        Serial.print(", ");
                Serial.print(gpsReader.m_lastLongitudeDeg, 10);
	        Serial.print("\n");
	}
#endif
	Serial.println("ok2");   
	Serial.print("Location: ");
	Serial.print(gpsReader.m_lastLatitudeDeg, 10);
	Serial.print(", ");
	Serial.print(gpsReader.m_lastLongitudeDeg, 10);
	Serial.print("\n");

	//
	// get initial waypoint; also sets the distanceToTarget and courseToTarget varilables
	nextWaypoint();

} // setup()




void loop()
{
        Serial.print("loop\n");
        
        //  Serial.println(F("ok"));
	// check for manual kill switch pressed
#ifdef USE_IR
	checkKillSwitch();
#endif          

	// Process GPS 
	processGPS();

	// navigate 
	currentHeading = readCompass();    // get our current heading
	calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      

	// distance in front of us, move, and avoid obstacles as necessary
	checkSonar();
	moveAndAvoid();  

	// update display and serial monitor    
	updateDisplay();    

}  // loop()




//
// Called after new GPS data is received; updates our position and course/distance to waypoint
void processGPS(void)
{
        gpsReader.readNextFrame();
        double lastLatitudeDeg = gpsReader.m_lastLatitudeDeg;
        double lastLongitudeDeg = gpsReader.m_lastLongitudeDeg;
    
        Serial.print("res2 ");
        Serial.print(gpsReader.m_lastLatitudeDeg, 10);
	Serial.print(", ");
        Serial.print(gpsReader.m_lastLongitudeDeg, 10);
	Serial.print("\n");
	//V1
	currentLat = GpsReader::convertDegMinToDecDeg(lastLatitudeDeg);
	// Serial.print("currentlat processgps");
	// Serial.println(currentLat,8);
	currentLong = GpsReader::convertDegMinToDecDeg(lastLongitudeDeg);
        Serial.print("res _current ");
        Serial.print(currentLat, 10);
	Serial.print(", ");
        Serial.print(currentLong, 10);
	Serial.print("\n");

        Serial.print("res _target ");
        Serial.print(targetLat, 10);
	Serial.print(", ");
        Serial.print(targetLong, 10);
	Serial.print("\n");
	// Serial.print("currentlong processgps");
	// Serial.println(currentLong,8);

	/*
	//V2
	currentLat = GPS.latitudeDegrees;
	currentLong =GPS.longitudeDegrees;
	 */          
	/*  if (GPS.lat == 'S')  {          // make them signed
	    currentLat = -currentLat;}
	    if (GPS.lon == 'W') { 
	    currentLong = -currentLong;} */

	// update the course and distance to waypoint based on our new position
	distanceToWaypoint();
	courseToWaypoint();         

}   // processGPS(void)




void checkSonar(void)
{   
	int dist;

	dist = sonar.ping_in();                   // get distqnce in inches from the sensor
	if (dist == 0)                                // if too far to measure, return max distance;
	dist = MAX_DISTANCE_IN;  
	sonarDistance = sonarAverage.add(dist);      // add the new value into moving average, use resulting average
} // checkSonar()




int readCompass(void)
{
	compass.getEvent(&compass_event);    
	float heading = atan2(compass_event.magnetic.y, compass_event.magnetic.x);
	/* Serial.print("heading ");
	   Serial.println(heading);*/
	// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
	// Find yours here: http://www.magnetic-declination.com/ 
	// Cedar Park, TX: Magnetic declination: 4° 11' EAST (POSITIVE);  1 degreee = 0.0174532925 radians
	//chateau thierry: +0°40' east(positive) 0.00698132
	// jaulgonne : +0°41' east (positive) 0.00715585

#define DEC_ANGLE 0.00715585
	heading += DEC_ANGLE;

	// Correct for when signs are reversed.
	if(heading < 0){
		heading += 2*PI;}

	// Check for wrap due to addition of declination.
	if(heading > 2*PI){
		heading -= 2*PI;}

	// Convert radians to degrees for readability.
	float headingDegrees = heading * 180/M_PI; 

	return ((int)headingDegrees); 
}  // readCompass()



void calcDesiredTurn(void)
{
	// calculate where we need to turn to head to destination
	headingError = targetHeading - currentHeading;
	/*  Serial.print("heading eror");
	    Serial.println(headingError);*/
	// adjust for compass wrap
	if (headingError < -180){      
		headingError += 360;}
	if (headingError > 180){
		headingError -= 360;}

	// calculate which way to turn to intercept the targetHeading
	if (abs(headingError) <= HEADING_TOLERANCE) {     // if within tolerance, don't turn
		turnDirection = straight;  }
	else if (headingError < 0){
		turnDirection = left;}
	else if (headingError > 0){
		turnDirection = right;}
	else
	{
		turnDirection = straight;
	}
}  // calcDesiredTurn()




void moveAndAvoid(void)
{

	if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
	{
		if (turnDirection == straight)
			spd = FAST_SPEED;
		else
			spd = TURN_SPEED;

		//analogWrite(drive_motor, spd);
		// digitalWrite(inv_drive_motor, LOW);
		// Drive->run(FORWARD);  
		Direction.write(turnDirection); 
		Drive_motor.writeMicroseconds(spd);   

		return;
	}

	if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)    // not yet time to turn, but slow down
	{
		if (turnDirection == straight){
			spd = NORMAL_SPEED;}
		else
		{
			spd = TURN_SPEED;
			Direction.write(turnDirection);      // alraedy turning to navigate
		}
		Drive_motor.writeMicroseconds(spd);
		//analogWrite(drive_motor, spd);
		//digitalWrite(inv_drive_motor, LOW);
		// driveMotor->run(FORWARD);       
		return;
	}

	if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
	{
		spd = SLOW_SPEED;
		Drive_motor.writeMicroseconds(spd);


		//analogWrite(drive_motor, spd);
		// digitalWrite(inv_drive_motor, LOW); // slow down
		//  driveMotor->run(FORWARD); 
		switch (turnDirection)
		{
			case straight:                  // going straight currently, so start new turn
				{
					if (headingError <= 0)
						turnDirection = left;
					else
						turnDirection = right;
					Direction.write(turnDirection);  // turn in the new direction
					break;
				}
			case left:                         // if already turning left, try right
				{
					Direction.write(TURN_RIGHT);    
					break;  
				}
			case right:                       // if already turning right, try left
				{
					Direction.write(TURN_LEFT);
					break;
				}

		} // end SWITCH

		return;
	}  


	if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
	{
		//  Drive.write(RELEASE);            // stop 
		Direction.write(TURN_STRAIGHT);  // straighten up
		Drive_motor.writeMicroseconds(1500);  
		delay(100);  
		// analogWrite(drive_motor, 0);
		turnDirection = straight;
		//       digitalWrite(inv_drive_motor, HIGH);
		//    analogWrite(drive_motor, NORMAL_SPEED);
		Drive_motor.writeMicroseconds(1250);  
		//Drive.write(NORMAL_SPEED);  // go back at higher speet
		// driveMotor->run(BACKWARD);           
		while (sonarDistance < TURN_DISTANCE)       // backup until we get safe clearance
		{
			
				processGPS();  
			currentHeading = readCompass();    // get our current heading
			calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      
			checkSonar();
			updateDisplay();
			delay(100);
		} // while (sonarDistance < TURN_DISTANCE)
		// driveMotor->run(RELEASE);        // stop backing up
		return;
	} // end of IF TOO CLOSE

}   // moveAndAvoid()





void nextWaypoint(void)
{
	waypointNumber++;  lcd.setCursor(0, 1);
	lcd.clear();
	lcd.println(F("waypoint n"));
	lcd.print(waypointNumber);
	delay(1000);
	targetLat = waypointList[waypointNumber].getLat();
	targetLong = waypointList[waypointNumber].getLong();
	Serial.print("targetlat");
	Serial.println(targetLat);
	Serial.print("targetlong");
	Serial.println(targetLong);

	if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
	{
		Drive_motor.writeMicroseconds(1500);
		//   analogWrite(drive_motor, 0);//Drive.write(RELEASE);    // make sure we stop
		Direction.write(TURN_STRAIGHT);  
		lcd.clear();
		lcd.println(F("* LAST WAYPOINT *"));
		loopForever();
	}

	processGPS();
        Serial.print("toto");
        distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
	Serial.print("toto2");
        courseToWaypoint();
        Serial.print("toto3");
        
}  // nextWaypoint()




// returns distance in meters between two positions, both specified 
// as signed decimal-degrees latitude and longitude. Uses great-circle 
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint() 
{

	float delta = radians(currentLong - targetLong);
	float sdlong = sin(delta);
	float cdlong = cos(delta);
	float lat1 = radians(currentLat);
	float lat2 = radians(targetLat);
	float slat1 = sin(lat1);
	float clat1 = cos(lat1);
	float slat2 = sin(lat2);
	float clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
	delta = sq(delta); 
	delta += sq(clat2 * sdlong); 
	delta = sqrt(delta); 
	float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
	delta = atan2(delta, denom); 
	distanceToTarget =  delta * 6372795; 

	// check to see if we have reached the current waypoint
	if (distanceToTarget <= WAYPOINT_DIST_TOLERANE){
		nextWaypoint();}

	return distanceToTarget;
}  // distanceToWaypoint()




// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
int courseToWaypoint() 
{
	float dlon = radians(targetLong-currentLong);
	float cLat = radians(currentLat);
	float tLat = radians(targetLat);
	float a1 = sin(dlon) * cos(tLat);
	float a2 = sin(cLat) * cos(tLat) * cos(dlon);
	a2 = cos(cLat) * sin(tLat) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0)
	{
		a2 += TWO_PI;
	}
	targetHeading = degrees(a2);
	return targetHeading;
}   // courseToWaypoint()




/*
// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
double convertDegMinToDecDeg (float degMin) 
{
Serial.println(degMin);
double min = 0.0;
double decDeg = 0.0;

//get the minutes, fmod() requires double
min = fmod((double)degMin, 100.0);

//rebuild coordinates in decimal degrees
degMin = (int) ( degMin / 100.0 );
decDeg = degMin + ( min / 60.0 );
Serial.println(decDeg);
return decDeg;

}*/
/*
// version de clement
double convertDegMinToDecDeg (float degMin)
{
Serial.println(degMin);
int h = degMin;
int min = (degMin-h)*100;
float s = ((degMin-h) - min/100.0)*10000;

double decDeg = h + (min/60.0) + s/3600.0;
Serial.println(decDeg);
return decDeg;
}*/


//
// Uses 4 line LCD display to show the following information:
// LINE 1: Target Heading; Current Heading;
// LINE 2: Heading Error; Distance to Waypoint; 
// LINE 3: Sonar Distance; Speed;
// LINE 4: Memory Availalble; Waypoint X of Y;  

void updateDisplay(void)
{

	static unsigned long lastUpdate = millis();       // for controlling frequency of LCD updates
	unsigned long currentTime;

	// check time since last update
	currentTime = millis();
	if (lastUpdate > currentTime) {  // check for time wrap around
		lastUpdate = currentTime; }     

	if (currentTime >= lastUpdate + 500 )   // limit refresh rate
	{
		lastUpdate = currentTime;

		// line 1
		lcd.clear();
		lcd.print(F("tH= "));
		lcd.print(targetHeading, DEC);
		lcd.write(DEGREE_SYMBOL);
		lcd.print(F(" cH= "));
		lcd.print(currentHeading, DEC);
		lcd.write(DEGREE_SYMBOL);

		// line 2
		lcd.setCursor(0, 1);
		lcd.print(F("Err "));
		if (headingError < 0)
			lcd.write(LEFT_ARROW);
		lcd.print(abs(headingError), DEC);
		if (headingError > 0)
			lcd.write(RIGHT_ARROW);
		lcd.print(F(" Dist "));
		lcd.print(distanceToTarget, DEC);
		lcd.print(F("m "));
#ifdef USE_GRAPHING
		lcd.write(map(distanceToTarget, 0, originalDistanceToTarget, 0, 7));    // show tiny bar graph of distance remaining
#endif

		// line 3
		lcd.setCursor(0, 2);
		lcd.print(F("Snr "));
		lcd.print(sonarDistance, DEC);
#ifdef USE_GRAPHING
		lcd.write(map(sonarDistance, 0, MAX_DISTANCE_IN, 0, 7));
#endif
		lcd.print(F(" Spd "));
		lcd.print(spd, DEC);
#ifdef USE_GRAPHING
		lcd.write(map(spd, 0, 255, 0, 7));
#endif

		// line 4
		lcd.setCursor(0, 3);
		lcd.print(F("Mem "));
		lcd.print(freeRam(), DEC);
		lcd.print(F(" WPT "));
		lcd.print(waypointNumber + 1, DEC);
		lcd.print(F(" OF "));
		lcd.print(NUMBER_WAYPOINTS - 1, DEC);


#ifdef DEBUG
		//Serial.print("GPS Fix:");
		//Serial.println((int)GPS.fix);
		/*  Serial.println(GPS.lastNMEA());
		    Serial.print("Location (in degrees, works with Google Maps): ");
		    Serial.print(GPS.latitudeDegrees, 6);
		    Serial.print(", "); 
		    Serial.println(GPS.longitudeDegrees, 6);
		    Serial.print(F("LAT = "));
		    Serial.print(currentLat,8);
		    Serial.print(F(" LON = "));
		    Serial.println(currentLong,8);*/
		Serial.print("Waypint ");
		Serial.print(waypointNumber);
		Serial.print(" LAT ="); 
		Serial.print(waypointList[waypointNumber].getLat(),8);
		Serial.print(F(" Long = "));
		Serial.print(waypointList[waypointNumber].getLong(),8);
		Serial.print(F(" Dist "));
		Serial.print(distanceToWaypoint(),4);
		Serial.print(F(" Original Dist "));
		Serial.println(originalDistanceToTarget);
		Serial.print(F("Compass Heading "));
		Serial.println(currentHeading);
		//   Serial.print(F("GPS Heading "));
		//   Serial.println(GPS.angle);
		//  Serial.println("num sat");
		//  Serial.println(GPS.satellites);
		//   Serial.println("qualite");
		//   Serial.println(GPS.fixquality);

		//Serial.println(GPS.lastNMEA());

		//Serial.print(F("Sonar = "));
		//Serial.print(sonarDistance, DEC);
		//Serial.print(F(" Spd = "));
		//Serial.println(speed, DEC);
		//Serial.print(F("  Target = "));
		//Serial.print(targetHeading, DEC);
		//Serial.print(F("  Current = "));
		//Serial.print(currentHeading, DEC);
		//Serial.print(F("  Error = "));
		//Serial.println(headingError, DEC);
		//Serial.print(F("Free Memory: "));
		//Serial.println(freeRam(), DEC);
#endif

	} //  if (currentTime >= lastUpdate + 500 )

}  // updateDisplay()  



//
// Display free memory available
//#ifdef DEBUG
int freeRam ()   // display free memory (SRAM)
{/*
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); */
	int v;
	v=0;
	return(int)v;
} // freeRam()
//#endif




// end of program routine, loops forever
void loopForever(void)
{
	while (1)
		;
}




// 
// Graphing (mini-inline bar graph for LCD display)
#ifdef USE_GRAPHING
void createLCDChars(void)
{
	int lvl = 0;
	byte arry[8];
	for (int a = 7; a >= 0; a--)
	{
		for (int b = 0; b <= 7; b++)
		{
			if (b >= lvl)
				arry[b] = B11111;       // solid
			else
				//arry[b] = B00000;     // blank row
				arry[b] = B10001;       // hollow but with sides
		}
		lcd.createChar(a, arry);
		lvl++;
	}
} // createLCDChars(void)
#endif




//
// Implement an IR "kill switch" if selected in configuration options
#ifdef USE_IR
void checkKillSwitch(void)
{
	if(IR_receiver.decode(&IR_results))     // check for manual "kill switch"
	{
		Drive_motor.write(1500);
		//    analogWrite(drive_motor,0); 
		//  digitalWrite(inv_drive_motor,LOW);
		Direction.write(TURN_STRAIGHT);              
		// Drive.write (RELEASE);              

		lcd.clear(); 
		lcd.print(F("Press to resume"));
		delay(1000); 

		IR_receiver.resume();  
		while(!IR_receiver.decode(&IR_results)) ;  // wait for key press
		IR_receiver.resume();  // get ready for any additional input
	}
}  // checkKillSwitch()
#endif






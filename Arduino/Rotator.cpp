/***********************************
Azimuth control library


****************************************/
#include "Rotator.h"
#include "config.h"
#include <DuePWM.h>
#include "PID_v1.h"
#include "VNH5019MotorShield.h"
#include "SFE_LSM9DS0.h"


volatile int az_state = 0;
volatile float az_current_deg;
boolean az_forward_dir = true;

volatile int el_state = 0;
volatile float el_current_deg;
boolean el_forward_dir = true;

//float AZ_prev_target_deg = 0;
//float AZ_target_deg;
//float floating_az_target_deg;
//unsigned long AZ_prev_set_time;
//unsigned long AZ_last_set_time;
//unsigned long AZ_motor_start_time;
//unsigned long prev_cmd_AZ_time;
//boolean plus_360 = false; //Flag to determine if we went past 360 degrees on the AZ
//boolean is_az_parking = false; //Flag for when we are parking (blocking)
//boolean is_tracking = false;//Are we actively tracking
//double az_target_multiplier;
double az_prev_motor_speed = 50;
unsigned long az_prev_ramp_time;
double el_prev_motor_speed = 50;
unsigned long el_prev_ramp_time;
//boolean az_motor_ramp = false;
//double az_motor_ramap_speed = MOTOR_START_SPEED;

double AZ_Setpoint = 0;
double AZ_Input = 0;
double AZ_Output = 0;

boolean new_el_motor_start = 0;
double EL_Setpoint = 0;
double EL_Input = 0;
double EL_Output = 0;

//AZ PID
//From a start
double AZ_Kp_start = 24; //4
double AZ_Ki_start = 0.02; //0.01
double AZ_Kd_start = 0.05; //0
//After start but more than 10 degrees from target
double AZ_Kp_cruise = 14; //70
double AZ_Ki_cruise = 0.008; //0.5
double AZ_Kd_cruise = 0.05; //0
//Within 10 degrees of the target
double AZ_Kp_track = 14; //2
double AZ_Ki_track = 0.008; //0.1
double AZ_Kd_track = 0.05; //0

//EL PID
//From a start
double EL_Kp_start = 22;
double EL_Ki_start = 0.01;
double EL_Kd_start = 0.05;
//After start but more than 10 degrees from target
double EL_Kp_cruise = 14;
double EL_Ki_cruise = 0.008;
double EL_Kd_cruise = 0.05;
//Within 10 degrees of the target
double EL_Kp_track = 14;
double EL_Ki_track = 0.008;
double EL_Kd_track = 0.05;


PID AZ_PID(&AZ_Input, &AZ_Output, &AZ_Setpoint, AZ_Kp_start, AZ_Ki_start, AZ_Kd_start, DIRECT);
PID EL_PID(&EL_Input, &EL_Output, &EL_Setpoint, EL_Kp_start, EL_Ki_start, EL_Kd_start, DIRECT);
//PID myPID(&Input, &Output, &Setpoint, AZ_Kp_start, AZ_Ki_start, AZ_Kd_start, DIRECT);
//int AZ_AZ_motor_speed = 61;
#define PWM_FREQ1  20000
#define PWM_FREQ2  20000	
VNH5019MotorShield md;
DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );


// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
uint32_t count = 0;  // used to control display output rate
uint32_t delt_t = 0; // used to control display output rate
float pitch, yaw, roll, heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float temperature;



Rotator::Rotator(){
	
	
}

void Rotator::init(void){
	
	Serial.print("Enabling encoder interrupts\n");
	pinMode(24, INPUT);
	attachInterrupt(24, AZStateChange, CHANGE);
	pinMode(25, INPUT);
	attachInterrupt(25, ELStateChange, CHANGE);
	
		
	Serial.println("Dual VNH5019 Motor Shield\n");
	
	
	AZ_motor_speed = 61;
	EL_motor_speed = 61;
	
	uint32_t status = dof.begin();
	Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
	Serial.println(status, HEX);
	Serial.println("Should be 0x49D4");
	Serial.println();
	
	
	Serial.print("Setting PWM pins\n");
	pwm.pinFreq1(8);  // Pin 9 freq set to "pwm_freq2" on clock B
	pwm.pinFreq1(9);
	//MotorShield md_el(26,28,30,A1,9);
	Serial.print("Initializing the motor controller\n");
	md.init();
	
	//PID
	//From a start
	//AZ_Kp_start = 3;
	//AZ_Ki_start = 0.01;
	//AZ_Kd_start = 0;
	//After start but more than 10 degrees from target
	//AZ_Kp_cruise = 70;
	//AZ_Ki_cruise = 0.5;
	//AZ_Kd_cruise = 0.3;

	//Within 10 degrees of the target
	//AZ_Kp_track = 70;
	//AZ_Ki_track = 0.25;
	//AZ_Kd_track = 0;
	
	//turn the PID on
	//PID AZ_PID(&AZ_Input, &AZ_Output, &AZ_Setpoint, 3, 0.01, 0, DIRECT);
	AZ_PID.SetMode(AUTOMATIC);
	AZ_PID.SetOutputLimits(50, 400);
	
	EL_PID.SetMode(AUTOMATIC);
	EL_PID.SetOutputLimits(50, 400);
	
	plus_360 = false; //Flag to determine if we went past 360 degrees on the AZ
	is_az_parking = false; //Flag for when we are parking (blocking)
	is_tracking = false;//Are we not actively tracking
	is_az_blocked = false;  //Not blocking AZ commands.  Block when parking and under zero
	
	AZ_prev_target_deg = 0;
	az_forward_dir = true;
	el_forward_dir = true;
	int az_forward_counter = 0;
	int el_forward_counter = 0;
	
	az_prev_ramp_time = 0;
	el_prev_ramp_time = 0;
	
	is_el_parking = false;
	is_az_parked = true; //start off parked
	is_el_parked = true;
	
	delay(2000); 
	// Set data output ranges; choose lowest ranges for maximum resolution
	// Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
    dof.setAccelScale(dof.A_SCALE_2G);
	// Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
    dof.setGyroScale(dof.G_SCALE_245DPS);
	// Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
    dof.setMagScale(dof.M_SCALE_2GS);
	// Set output data rates  
 // Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50, 
 //                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)
    dof.setAccelODR(dof.A_ODR_200); // Set accelerometer update rate at 100 Hz
 // Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
 // Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
 // Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
 // to get a smooth ~150 Hz filter update rate
    dof.setAccelABW(dof.A_ABW_50); // Choose lowest filter setting for low noise
 
 // Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
 //                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
    dof.setGyroODR(dof.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

 // Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
    dof.setMagODR(dof.M_ODR_125); // Set magnetometer to update every 80 ms
    
 // Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
 // all subsequent measurements.
    dof.calLSM9DS0(gbias, abias);
    Serial.print(String(gbias[0]) + "," + String(gbias[1]) + "," + String(gbias[2]) + "\n");
    Serial.print(String(abias[0]) + "," + String(abias[1]) + "," + String(abias[2]) + "\n");
	
	
}

double Rotator::get_az_current_deg(void){
	
	return az_current_deg;
}

double Rotator::get_el_current_deg(void){
	
	return el_current_deg;
}

double Rotator::get_az_target_deg(void){
	
	return floating_az_target_deg;
}

double Rotator::get_el_target_deg(void){
	
	return floating_el_target_deg;
}

double Rotator::get_az_plus_360(void){
	
	return plus_360;
}

double Rotator::get_az_absolute_pos(void){
	
	double AZ_absolute_pos;
	
	//return AZ_absolute_pos;
	
	//Get the position we are reporting back the the Head
	if (az_current_deg < 0) {
		//Prob negative.  Add 360 to get a more positive number for the Head
		AZ_absolute_pos = az_current_deg + 360;
	} else {
		if (plus_360) {
			//We've crossed over 360, so present a zero to 360 heading
			AZ_absolute_pos = az_current_deg - 360;
		} else {
			AZ_absolute_pos = az_current_deg;
		}
	}
	return AZ_absolute_pos;
}

double Rotator::get_el_absolute_pos(void){
	
	return el_current_deg;
}

void Rotator::set_az_target_deg(float val){
	Serial.println("New AZ position: " + String(val));
	
	
	//AZ_prev_set_time = AZ_last_set_time;
	//AZ_last_set_time = millis();
	//AZ_prev_target_deg = AZ_target_deg;
	AZ_target_deg = val;
	TrackAZ();
	
}

void Rotator::set_el_target_deg(float val){
	Serial.println("New EL position: " + String(val));
	
	
	//AZ_prev_set_time = AZ_last_set_time;
	//AZ_last_set_time = millis();
	//AZ_prev_target_deg = AZ_target_deg;
	EL_target_deg = val;
	TrackEL();
	
}

void Rotator::track(float azval, float elval) {
	
	unsigned long tmp_set_time;
	
	
	tmp_set_time = millis();
	if (is_az_blocked){
		//Don't accept commands until the AZ is unblocked (past zero)
		Serial.println("Blocking new positions " + String(azval) + " - " + String(elval));
	} else {
		//Not blocked, so accept the position
		if (!is_tracking){
			//First position of this track
			enable_tracking();
			Serial.println("Starting track at " + String(azval) + " - " + String(elval));
			//prev_set_time = millis();
			//We want to wait until the second position to start moving
			//Set the target to the current location
			AZ_target_deg = az_current_deg;
			EL_target_deg = el_current_deg;
			
			//first track, so re-init the PID
			PID AZ_PID(&AZ_Input, &AZ_Output, &AZ_Setpoint, AZ_Kp_start, AZ_Ki_start, AZ_Kd_start, DIRECT);
			PID EL_PID(&EL_Input, &EL_Output, &EL_Setpoint, EL_Kp_start, EL_Ki_start, EL_Kd_start, DIRECT);
			AZ_PID.SetMode(AUTOMATIC);
			AZ_PID.SetOutputLimits(50, 400);
			EL_PID.SetMode(AUTOMATIC);
			EL_PID.SetOutputLimits(50, 400);
		} else {
			
			Serial.println("Tracking at " + String(azval) + " - " + String(elval));
			AZ_target_deg = azval;
			//AZ_prev_target_deg = AZ_target_deg;
					
			EL_target_deg = elval;
			//EL_prev_target_deg = EL_target_deg;
			is_az_parked = false;
			is_el_parked = false;
			TrackAZ();
			TrackEL();
		}
		
		
		//Grab the time we got the request and associate it with the last position received
		prev_set_time = tmp_set_time;
		
	}
	
	
	
	
	
}

void Rotator::TrackAZ(void) {
  double degdiff, diff;

  degdiff = 0;
  
  if (is_tracking){//Only if we are tracking
  

	  //First we need to get the difference between this new target and the last target
	  //This is somewhat complicated by crossing over zero to 359 degrees as well as crossing 360 degrees
	  Serial.println("Prev target: " + String(AZ_prev_target_deg) + " - Current target: " + String(AZ_target_deg) + " - Prev set time: " + String(prev_set_time));
	  //First we need to know which direction we are going and did we "cross over"?
	  if (AZ_prev_target_deg == AZ_target_deg) {
		degdiff = 0; //same same
	  } else {

		if (AZ_prev_target_deg >= 0 && AZ_target_deg >= 0) {
		  //We didn't just now cross zero, but are we already below zero?
		  if (AZ_prev_target_deg < 0) {
			//Already below zero
			AZ_target_deg = AZ_target_deg - 360;
			degdiff = AZ_target_deg - AZ_prev_target_deg;

		  } else {
			if (AZ_prev_target_deg == 0 && AZ_target_deg > 0){
				//we must be going clockwise from zero	
				Serial.println("AZ going clockwise from zero");				
				degdiff = AZ_target_deg;
			} else {
				if (AZ_prev_target_deg < AZ_target_deg) {
				  //We are going clockwise, above zero and below 360
				  //Get the difference in degrees between the two positions
				  degdiff = AZ_target_deg - AZ_prev_target_deg;
				} else {
				  //We are going counter clockwise
				  degdiff = AZ_prev_target_deg - AZ_target_deg;
				}
			}
		  }


		  //Did we cross 360?
		  if (plus_360) {
			//We've previously crossed over 360.  Add them up
			AZ_target_deg = 360 + AZ_target_deg;
			degdiff = AZ_target_deg - AZ_prev_target_deg;  //Add 360 to the prev target so we're on the same "scale"
		  } else {

			if (AZ_prev_target_deg > 340 && AZ_target_deg > 0 && AZ_target_deg < 20) { //such as 359 and 4.  Assuming 300 and 20 is a good point to compare against
			  //We just crossed over clockwise, from 359 to zero+
			  Serial.println("We just crossed over clockwise, from 359 to zero+");
			  plus_360 = true;
			  AZ_target_deg = AZ_target_deg + 360;  //Add 360 to the value over zero
			  degdiff = AZ_target_deg - AZ_prev_target_deg;  //Add 360 to the prev target so we're on the same "scale"
			} else {
				if (AZ_prev_target_deg != 0){
					if (AZ_prev_target_deg < 20 && AZ_target_deg > 340) { //such as 4 and 359
					//We just now crossed over counter clockwise, from 0 to 359 and below
					Serial.println("We just now crossed over counter clockwise, from 0 to 359 and below");
					AZ_target_deg = AZ_target_deg - 360;
					degdiff = AZ_target_deg + AZ_prev_target_deg;

					}
				}
			}
		  }

		} else {
		  //We have crossed zero at some point
		  if (is_az_parking) {
			Serial.println("AZ crossed zero while parking.  Not likely!");
			degdiff = AZ_target_deg - AZ_prev_target_deg;
		  } else {
			if (AZ_prev_target_deg < 0) {
			  //We crossed below zero at some point
			  Serial.println("AZ crossed zero");
			  AZ_target_deg = AZ_target_deg - 360;
			  degdiff = AZ_target_deg - AZ_prev_target_deg;
			}
		  }
		}
	  }
	  
	  
	  
	  Serial.println("AZ_prev_target_deg: " + String(AZ_prev_target_deg) + " AZ_target_deg: " + String(AZ_target_deg) + "\n");
	  Serial.println("Over 360: " + String(plus_360) + "\n");
	  //Now that we have the difference, calculate the rate
	  double msdiff = millis() - prev_set_time;  //current time - time of previous position
	  Serial.println("AZ degdiff:" + String(degdiff));
	  Serial.println("AZ msdiff: " + String(msdiff));
	  diff = double(degdiff) / double(msdiff);
	  Serial.println("AZ degdiff/msdiff: ");
	  Serial.println(diff, 16);
	  if (diff == 0) {
		Serial.println(String("AZ DIFF is zero"));
		az_target_multiplier = 0;
	  } else {
		az_target_multiplier = diff;
	  }
	  
	  
	  
	  Serial.println("AZ Multiplier: ");
	  Serial.println(az_target_multiplier, 16);
	  Serial.println("Floating target: " + String(floating_az_target_deg));
	  AZ_prev_target_deg = AZ_target_deg;
	  //AZ_prev_set_time = millis();

	  
	  //Reversing directions, when and where
	  Serial.println("az_current_deg: " + String(az_current_deg) + "AZ_target_deg: " + String(AZ_target_deg) + "\n");
	  double targetdiff = 0;
	  if (az_current_deg < AZ_target_deg) {
		//We are moving forward, increasing the degree position
		if (az_forward_dir == false) {
		  targetdiff = AZ_target_deg - az_current_deg;
		  //We should be moving forward, but currently not.
		  //We need two things here -
		  //	-	Don't bounce back and forth
		  //	-	Start tracking asap
		  
		  if (targetdiff > 3) { //Only change direction if the gap is more than 3 degrees
			stop_az_motor();
			//Now we can change directions.  ****We can only change directions when the motor has stopped****
			az_forward_dir = true;
			//az_move_counter = 0; //reset the counter
		  }
		}
	  } else {
		//We are moving backward, decreasing the degree position
		if (az_forward_dir == true) {
		  targetdiff = az_current_deg - AZ_target_deg;
		  //We should be moving backward, but currently not.
		  //Add a buffer to prevent the rotator from bouncing back and forth if we overshoot the target
		  
		  if (targetdiff > 3) { //Only change direction if the gap is more than 3 degrees
			stop_az_motor();
			//Now we can change directions.  ****We can only change directions when the motor has stopped****
			az_forward_dir = false;
			//az_move_counter = 0; //reset the counter
		  }
		}
	  }

  }


}

void Rotator::TrackEL(void) {
	double degdiff, diff;

	degdiff = 0;
  
	if (is_tracking){//Only if we are tracking
		Serial.println("Prev EL target: " + String(EL_prev_target_deg) + " - Current target: " + String(EL_target_deg) + " - Prev set time: " + String(prev_set_time));
	
		if (EL_prev_target_deg == EL_target_deg) {
			degdiff = 0; //same same
		} else {
			if (EL_prev_target_deg < EL_target_deg) {
			  //We are going clockwise, above zero and below 90
			  //Get the difference in degrees between the two positions
			  degdiff = EL_target_deg - EL_prev_target_deg;
			} else {
			  //We are going counter clockwise
			  degdiff = EL_prev_target_deg - EL_target_deg;
			}
			
		}
		
		
		Serial.println("EL_prev_target_deg: " + String(EL_prev_target_deg) + " EL_target_deg: " + String(EL_target_deg) + "\n");
		
		//Now that we have the difference, calculate the rate
		double msdiff = millis() - prev_set_time;  //current time - time of previous position
		Serial.println("EL degdiff:" + String(degdiff));
		Serial.println("EL msdiff: " + String(msdiff));
		diff = double(degdiff) / double(msdiff);
		Serial.println("diff: ");
		Serial.println(diff, 8);
		if (diff == 0) {
			Serial.println(String("EL DIFF is zero"));
			el_target_multiplier = 0;
		} else {
			el_target_multiplier = diff;
		}
		
		
		
		Serial.println("EL Multiplier: ");
		Serial.println(el_target_multiplier, 8);
		EL_prev_target_deg = EL_target_deg;
		//AZ_prev_set_time = millis();

		Serial.println("EL_current_deg: " + String(el_current_deg) + "EL_target_deg: " + String(EL_target_deg) + "\n");
		double targetdiff = 0;
		if (el_current_deg < EL_target_deg) {
			//We are moving forward, increasing the degree position
			if (el_forward_dir == false) {
				targetdiff = EL_target_deg - el_current_deg;
				//We should be moving forward, but currently not.
				//Add a buffer to prevent the rotator from bouncing back and forth if we overshoot the target
				if (targetdiff > 2) { //Only change direction if the gap is more than 2 degrees
					stop_el_motor();
					//Now we can change directions.  ****We can only change directions when the motor has stopped****
					el_forward_dir = true;
				}
			}
		} else {
			//We are moving backward, decreasing the degree position
			if (el_forward_dir == true) {
				targetdiff = el_current_deg - EL_target_deg;
				//We should be moving backward, but currently not.
				//Add a buffer to prevent the rotator from bouncing back and forth if we overshoot the target
				if (targetdiff > 2) { //Only change direction if the gap is more than 2 degrees
					stop_el_motor();
					//Now we can change directions.  ****We can only change directions when the motor has stopped****
					el_forward_dir = false;
				}
			}
		}
	
	}
}

void Rotator::processAZPosition(void) {
	double pid_degdiff;
	
	if (is_az_parked){
		return; //No point processing a parked rotator
	} 
	
	if (az_current_deg < 360 && plus_360){
		plus_360 = false;
	}
	
	if (is_az_blocked){
		if (az_current_deg > 0){//AZ is past zero, we can unblock and accept new commands
			is_az_blocked = false;
		}
	}
	// Flip the motor direction if we're going the wrong way towards park
	// Forces the target to the park position
	if (is_az_parking){
		floating_az_target_deg = PARK_DEG; //This is the absolute position we need to go to
		if (az_forward_dir) { //we are parking, lets figure out if we need to change directions before parking
			if (az_current_deg > floating_az_target_deg){ //moving forward, where's the target?
				//Looks like the park target is behind our current position, so stop and turn around
				stop_az_motor();
				az_forward_dir = false;
			}
			

		} else { //we are moving backwards
			if (az_current_deg < floating_az_target_deg){ //we are going the wrong way!
				//Looks like the park target is forward our current position, so stop and turn around
				stop_az_motor();
				az_forward_dir = true;
			}
		}
	}
	
	
	
	if (!is_az_parking){ //No point calculating a new position if we are parking
		CalculateNewPos();
	}
	
	
	if (az_forward_dir == true) {
		AZ_Input = az_current_deg;
		AZ_Setpoint = floating_az_target_deg;
		pid_degdiff = floating_az_target_deg - az_current_deg;
	} else {
		AZ_Input = floating_az_target_deg;
		AZ_Setpoint = az_current_deg;
		pid_degdiff = az_current_deg - floating_az_target_deg;
	}
	
	if (is_az_parking){
		if (pid_degdiff < 1){ // we are there, within a degree.  So stop
			is_az_parking = false; //we are parked
			AZ_motor_speed = 0; //stopped
			AZ_Setpoint = 50; //stopped
			md.setM1Speed(AZ_motor_speed);
			is_az_parked = true;
			return; //we are done, no point going further
		}
	}
	
  
  //pid_degdiff should always be a positive number
  //Don't want to deal with the occasional negative brought on by an overshoot
  //Set both to the same so it should stop the motor
  if (pid_degdiff < 0){
	  pid_degdiff = 0;
	  AZ_Setpoint = AZ_Input;
  }
  
	//Serial.println("AZ Input (current degrees):" + String(AZ_Input));
	//Serial.println("AZ Setpoint:" + String(AZ_Setpoint));
	//Serial.println("AZ Output:" + String(AZ_Output));
	//Serial.println("AZ Deg Diff:" + String(pid_degdiff));
		
  if (pid_degdiff < 15){
	if (pid_degdiff <= 0){
	  AZ_motor_speed = 0;  //Shouldn't be less than zero, but just in case
	} else {
		//Use PID for less than a 15 degree difference, but not less than 0

				
		AZ_PID.Compute();
		//Serial.println("AZ Input (current degrees):" + String(AZ_Input));
		//Serial.println("AZ Setpoint:" + String(AZ_Setpoint));
		//Serial.println("AZ Output:" + String(AZ_Output));

		if (AZ_Output < AZ_MOTOR_START_SPEED) {
			AZ_motor_speed = 0; //This will stop it

		} else {
			AZ_motor_speed = AZ_Output;
		}
	}
  } else { //Not less than 15
	  
	//It must be more than 15 degrees difference, since it's not less than 15 or 0
	//Crank it up to max and let the ramping figure it out
	AZ_motor_speed = 400; 
  }
	  
  
  
  
 // Ramp up/down if the motor speed is more than the start speed
 
 //Serial.println("AZ_motor_speed " + String(AZ_motor_speed) + " az_prev_motor_speed " + String(az_prev_motor_speed));
  if (AZ_motor_speed > az_prev_motor_speed){
	if (AZ_motor_speed - az_prev_motor_speed > 35){
		//large increase in speed
		if (az_prev_ramp_time +  AZ_MOTOR_RAMP_CHANGE_INTERVAL < millis()){ //change the speed if X ms passed
			Serial.println("AZ ramp up to " + String(AZ_motor_speed) + " from " + String(az_prev_motor_speed));
			AZ_motor_speed = az_prev_motor_speed + 1;
			
			az_prev_ramp_time = millis();
		} else {
			AZ_motor_speed = az_prev_motor_speed; //timer hasn't counted down, so keep the same speed as before
		}
		
		
	}
  }
 
 
  //Ramp down the motor speed if it's greatly different from the last speed
  if (az_prev_motor_speed > AZ_motor_speed){
	if (az_prev_motor_speed - AZ_motor_speed > 35){
		//large decrease in speed
		if (az_prev_ramp_time +  AZ_MOTOR_RAMP_CHANGE_INTERVAL < millis()){ //change the speed if X ms passed
		Serial.println("AZ ramp down to " + String(AZ_motor_speed) + " from " + String(az_prev_motor_speed));
			AZ_motor_speed = az_prev_motor_speed - 1;
			
			az_prev_ramp_time = millis();
		} else {
			AZ_motor_speed = az_prev_motor_speed; //timer hasn't counted down, so keep the same speed as before
		}
	}
  }
 
  
  
   
  az_prev_motor_speed = AZ_motor_speed;
  
  //Make it negative to reverse the motor direction
  if (az_forward_dir == false) {
    if (AZ_motor_speed > 0) {
      AZ_motor_speed = AZ_motor_speed * -1;
    }
  }
  
  //Serial.println(AZ_motor_speed);
  /*
  //Stop if we are close and the target is static
  if (az_target_multiplier == 0){
	  if (pid_degdiff < 0.5){ //Stop if we are withing a half degree
		  //Serial.println("AZ BOOM! - On target");
		  //AZ_motor_speed = 0;
		  //new_az_motor_start = 0;
	  } 
  }
  */
  
  md.setM1Speed(AZ_motor_speed);
  
  
}

void Rotator::processELPosition(void) {
	double pid_degdiff;
	
	if (is_el_parked){
		return; //No point processing a parked rotator
	} 
	
	if (is_el_parking){
		floating_el_target_deg = 0;
	}
	
	// Flip the motor direction if we're going the wrong way towards park
	// Forces the target to the park position
	if (is_el_parking){
		floating_el_target_deg = 0; //This is the absolute position we need to go to
		if (el_forward_dir) { //we are parking, lets figure out if we need to change directions before parking
			if (el_current_deg > floating_el_target_deg){ //moving forward, where's the target?
				//Looks like the park target is behind our current position, so stop and turn around
				stop_el_motor();
				el_forward_dir = false;
			}
			

		} else { //we are moving backwards
			if (el_current_deg < floating_el_target_deg){ //we are going the wrong way!
				//Looks like the park target is forward our current position, so stop and turn around
				stop_el_motor();
				el_forward_dir = true;
			}
		}
	}
	
	
	if (!is_el_parking){ //No point calculating a new position if we are parking
		CalculateNewPos();
	}
	
	if (el_forward_dir == true) {
		EL_Input = el_current_deg;
		EL_Setpoint = floating_el_target_deg;
		pid_degdiff = floating_el_target_deg - el_current_deg;
	} else {
		EL_Input = floating_el_target_deg;
		EL_Setpoint = el_current_deg;
		pid_degdiff = el_current_deg - floating_el_target_deg;
	}
  
  if (is_el_parking){
		if (pid_degdiff < 2){
			is_el_parking = false; //we are parked, more or less
		}
	}
  
  
  //pid_degdiff should always be a positive number
  //Don't want to deal with the occasional negative brought on by an overshoot
  //Set both to the same so it should stop the motor
  if (pid_degdiff < 0){
	  pid_degdiff = 0;
	  EL_Setpoint = EL_Input;
  }
  
  
  if (pid_degdiff < 15){
	if (pid_degdiff <= 0){
	  EL_motor_speed = 0;  //Shouldn't be less than zero, ut just in case
	} else {
		//Use PID for less than a 15 degree difference, but not less than 0

		EL_PID.Compute();
		//Serial.println("EL Input (current degrees):" + String(EL_Input));
		//Serial.println("EL Setpoint:" + String(EL_Setpoint));
		//Serial.println("EL Output:" + String(EL_Output));

		if (EL_Output < EL_MOTOR_START_SPEED) {
			EL_motor_speed = 0; //This will stop it

		} else {
			EL_motor_speed = EL_Output;
		}
	}
  } else { //Not less than 15
	  
	//It must be more than 15 degrees difference, since it's not less than 15 or 0
	//Crank it up to max and let the ramping figure it out
	EL_motor_speed = 400; 
  }
	  
  
  
  
 // Ramp up/down if the motor speed is more than the start speed
  //Ramp up the motor speed if it's greatly different from the last speed
  if (EL_motor_speed > el_prev_motor_speed){
	if (EL_motor_speed - el_prev_motor_speed > 35){
		//large increase in speed
		if (el_prev_ramp_time +  EL_MOTOR_RAMP_CHANGE_INTERVAL < millis()){ //change the speed if X ms passed
			Serial.println("EL ramp up to " + String(EL_motor_speed) + " from " + String(el_prev_motor_speed));
			EL_motor_speed = el_prev_motor_speed + 1;
			
			el_prev_ramp_time = millis();
		} else {
			EL_motor_speed = el_prev_motor_speed; //timer hasn't counted down, so keep the same speed as before
		}
		
		
	}
  }
  //Ramp down the motor speed if it's greatly different from the last speed
  if (el_prev_motor_speed > EL_motor_speed){
	if (el_prev_motor_speed - EL_motor_speed > 35){
		//large decrease in speed
		if (el_prev_ramp_time +  EL_MOTOR_RAMP_CHANGE_INTERVAL < millis()){ //change the speed if X ms passed
			Serial.println("EL ramp down to " + String(EL_motor_speed) + " from " + String(el_prev_motor_speed));
			EL_motor_speed = el_prev_motor_speed - 1;
			
			el_prev_ramp_time = millis();
		} else {
			EL_motor_speed = el_prev_motor_speed; //timer hasn't counted down, so keep the same speed as before
		}
	}
  }
  
  
   
  el_prev_motor_speed = EL_motor_speed;
  
  //Make it negative to reverse the motor direction
  if (el_forward_dir == false) {
    if (EL_motor_speed > 0) {
      EL_motor_speed = EL_motor_speed * -1;
    }
  }
  
  //Serial.println(EL_motor_speed);
  /*
  //Stop if we are close and the target is static
  if (el_target_multiplier == 0){
	  if (pid_degdiff < 0.5){ //Stop if we are withing a half degree
		  //Serial.println("EL BOOM! - On target");
		  //EL_motor_speed = 0;
		  //new_el_motor_start = 0;
	  } 
  }
  */
  
  md.setM2Speed(EL_motor_speed);
	
}

void Rotator::processPosition(void) {
	
	processAZPosition();
	processELPosition();
	readLSM9DSO();
	
}

void Rotator::park(void) {
	stopIfFaultAZ();
	stopIfFaultEL();
	disable_tracking(); //turn off tracking, since we are parking and not tracking
	is_az_parking = true;
	is_el_parking = true;
	if (az_current_deg < 0) { //block the parking so long as we are under zero degrees
		is_az_blocked = true;
	}
	
	
	floating_az_target_deg = PARK_DEG;
	floating_el_target_deg = 0;
	//TrackAZ();
	//TrackEL();
	processPosition();
	/*
	if (AZ_prev_target_deg < 0) {
		//Advance past zero before we let anything else go on
		AZ_prev_set_time = millis() - 1000;
		AZ_prev_target_deg = az_current_deg;

		while (az_current_deg < 0) { //block the parking so long as we are under zero degrees
			is_az_blocked = true;
			AZ_target_deg = PARK_DEG;
			floating_az_target_deg = PARK_DEG;
			TrackAZ();
			AZ_prev_target_deg = az_current_deg;
			processAZPosition();
			md.setM1Speed(AZ_motor_speed);
			stopIfFaultAZ();
			delay(1000);
		}
	} else {
		floating_az_target_deg = PARK_DEG;
		AZ_target_deg = PARK_DEG;
		TrackAZ();
		processAZPosition();

	}*/

	//prev_cmd_AZ_time = 0;


}

void Rotator::CalculateNewPos(void) {
	
	
  //AZ_prev_set_time = millis();
  //AZ_prev_target_deg = AZ_target_deg;
  if (is_tracking) {
    unsigned long tmp = millis() - prev_set_time;
	
	floating_az_target_deg = AZ_prev_target_deg + (double(tmp) * double(az_target_multiplier));
	
	
	
	floating_el_target_deg = EL_prev_target_deg + (double(tmp) * double(el_target_multiplier));
	
  }




  //Serial.print("\nNew Target: ");
  //Serial.print(target_deg_tmp);


  //floating_az_target_deg = now_time/4000.12;  //0.25 degrees / sec

  //floating_az_target_deg = now_time/2857.142;  //0.35 degrees / sec
  //floating_az_target_deg = now_time/1250.01;  //0.80 degrees / sec
  //floating_az_target_deg = now_time/12500.01;  //0.08 degrees / sec
  //floating_az_target_deg = now_time/25000.01;  //0.04 degrees / sec
  //floating_az_target_deg = now_time/50000.01;  //0.02 degrees / sec
}

void Rotator::stop_az_motor(void) {
	//Hate to block, be we really need to control the slow down and stop, so we can turn around
	//No point in processing anything new until we are done

	Serial.print("Stopping the AZ motor\n");
	if (AZ_motor_speed == 0){
		Serial.print("AZ motor already stopped\n");
	} else {
		md.setM1Speed(AZ_motor_speed);
		az_prev_motor_speed = AZ_motor_speed;
		//delay(75);
		if (AZ_motor_speed > 0) { //moving forward
			while (AZ_motor_speed > 50) {
				
				//large decrease in speed
				if (az_prev_ramp_time +  AZ_MOTOR_RAMP_CHANGE_INTERVAL < millis()){ //change the speed if X ms passed
					Serial.println("AZ ramp up to " + String(AZ_motor_speed) + " from " + String(az_prev_motor_speed));
					AZ_motor_speed = az_prev_motor_speed - 1;
					
					az_prev_ramp_time = millis();
				} else {
					AZ_motor_speed = az_prev_motor_speed; //timer hasn't counted down, so keep the same speed as before
				}
				
				
				//Eventually we'll get to stopping it			
				md.setM1Speed(AZ_motor_speed);
				az_prev_motor_speed = AZ_motor_speed;
			}
		} else { //moving backward
			while (AZ_motor_speed < 50) {
				
				//large decrease in speed
				if (az_prev_ramp_time +  AZ_MOTOR_RAMP_CHANGE_INTERVAL < millis()){ //change the speed if X ms passed
					Serial.println("AZ ramp down to " + String(AZ_motor_speed) + " from " + String(az_prev_motor_speed));
					AZ_motor_speed = az_prev_motor_speed + 1;
					
					az_prev_ramp_time = millis();
				} else {
					AZ_motor_speed = az_prev_motor_speed; //timer hasn't counted down, so keep the same speed as before
				}
				
				//Eventually we'll get to stopping it			
				md.setM1Speed(AZ_motor_speed);
				az_prev_motor_speed = AZ_motor_speed;
			}
		}
	}

	//At this point we are stopped

	md.setM1Speed(0);
	//delay(75);
	Serial.print("Motor stopped\n");
}

void Rotator::stop_el_motor(void) {
	Serial.print("Stopping the EL motor\n");
	md.setM2Speed(EL_motor_speed);
	delay(75);
	if (EL_motor_speed > 0) {
	while (EL_motor_speed > 50) {
	  EL_motor_speed = EL_motor_speed - 3;
	  //md.setM1Speed(AZ_motor_speed);
	  delay(50);
	}
	} else {
	while (EL_motor_speed < 50) {
	  EL_motor_speed = EL_motor_speed + 3;
	  md.setM2Speed(EL_motor_speed);
	  delay(50);

	}
	}



	md.setM1Speed(0);
	Serial.print("Motor stopped\n");
	
}

void Rotator::disable_tracking(void){
	
	is_tracking = false;
	
}

void Rotator::enable_tracking(void){
	
	is_tracking = true;
	//is_az_parking = false; //Flip these off in Track() instead
	//is_el_parking = false;
	
	
}

boolean Rotator::is_rot_tracking(void){
	
	return is_tracking;
	
}

boolean Rotator::is_rot_parking(void){
	
	if (is_az_parking || is_el_parking){
		return true;
	} else {
		return false;
	}
}

void Rotator::set_new_PID(String axis, double p, double i, double d){
	
		
	if (axis == "AZ"){
		
		AZ_Kp_start = p;
		AZ_Ki_start = i;
		AZ_Kd_start - d;
		
		
		
	} else {
		
		EL_Kp_start = p;
		EL_Ki_start = i;
		EL_Kd_start = d;
		
		
	}
	
}

String Rotator::get_PID(String axis){
	
	if (axis == "AZ"){
		
		return String(AZ_Kp_start) + "," + String(AZ_Ki_start) + "," + String(AZ_Kd_start);
		
		
		
	} else {
		
		return String(EL_Kp_start) + "," + String(EL_Ki_start) + "," + String(EL_Kd_start);
		
		
	}
	
}

void Rotator::stopIfFaultAZ(void){  //TODO - Add mA chack and return something
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while (1);
  }
  
  if (md.getM1CurrentMilliamps() > 1500){
	  Serial.println("AZ mA: " + String(md.getM1CurrentMilliamps()));
  }
  
}

void Rotator::stopIfFaultEL(void){ //TODO - Add mA chack and return something
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while (1);
  }
  if (md.getM2CurrentMilliamps() > 1500){
	  Serial.println("EL mA: " + String(md.getM2CurrentMilliamps()));
  }
  
}

double Rotator::getAZEncoderCount(void){
	
	//return 0;
	return az_state;
}

double Rotator::getELEncoderCount(void){
	
	//return 0;
	return el_state;
}

String Rotator::getAZDegPerSec(void){
	String val = String(az_target_multiplier*1000,16);
	return val;
}

String Rotator::getELDegPerSec(void){
	String val = String(el_target_multiplier*1000,16);
	return val;
}

String Rotator::getAZMotorCurrent(void){
	return String(md.getM1CurrentMilliamps());
}

String Rotator::getELMotorCurrent(void){
	return String(md.getM2CurrentMilliamps());
}

void AZStateChange() {
	
  if (az_forward_dir) {
    az_state = (az_state + 1);  //moving forward, increment the encoder counter
  } else {
    az_state = (az_state - 1);
  }
  az_current_deg = az_state / AZ_PULSES_PER_DEG;
  

}

void ELStateChange() {
	
  if (el_forward_dir) {
    el_state = (el_state + 1);  //moving forward, increment the encoder counter
  } else {
    el_state = (el_state - 1);
  }
  el_current_deg = el_state / EL_PULSES_PER_DEG;
  

}

void readLSM9DSO(){
	
	dof.readGyro();           // Read raw gyro data
	gx = dof.calcGyro(dof.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
	gy = dof.calcGyro(dof.gy) - gbias[1];
	gz = dof.calcGyro(dof.gz) - gbias[2];
	

	
	dof.readAccel();         // Read raw accelerometer data
	ax = dof.calcAccel(dof.ax) - abias[0];   // Convert to g's, remove accelerometer biases
	ay = dof.calcAccel(dof.ay) - abias[1];
	az = dof.calcAccel(dof.az) - abias[2];
	

	
	dof.readMag();           // Read raw magnetometer data
	mx = dof.calcMag(dof.mx);     // Convert to Gauss and correct for calibration
	my = dof.calcMag(dof.my);
	mz = dof.calcMag(dof.mz);

	dof.readTemp();
	temperature = 21.0 + (float) dof.temperature/8.; // slope is 8 LSB per degree C, just guessing at the intercept
	Now = micros();
	deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;
	// Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
	// This is ok by aircraft orientation standards!  
	// Pass gyro rate as rad/s
	MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
	// Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate
		printHeading(mx, my);
		//printOrientation(ax, ay, az);
		yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		pitch *= 180.0f / PI;
		yaw   *= 180.0f / PI; 
		yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		roll  *= 180.0f / PI;
		
		Serial.print("temperature = "); Serial.println(temperature, 2);
		
		Serial.print("Yaw, Pitch, Roll: ");
		Serial.print(yaw, 2);
		Serial.print(", ");
		Serial.print(pitch, 2);
		Serial.print(", ");
		Serial.println(roll, 2);
		//Serial.print("filter rate = "); Serial.println(1.0f/deltat, 1);
		count = millis();
	}
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
	// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
	// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
	// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
	// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
	// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
	// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

void printHeading(float hx, float hy){
	if (hy > 0)
	{
	heading = 90 - (atan(hx / hy) * (180 / PI));
	}
	else if (hy < 0)
	{
	heading = - (atan(hx / hy) * (180 / PI));
	}
	else // hy = 0
	{
	if (hx < 0) heading = 180;
	else heading = 0;
	}

	Serial.print("Heading: ");
	Serial.println(heading, 2);
}


void printOrientation(float x, float y, float z){
	
	// float pitch, roll;

	pitch = atan2(x, sqrt(y * y) + (z * z));
	roll = atan2(y, sqrt(x * x) + (z * z));
	pitch *= 180.0 / PI;
	roll *= 180.0 / PI;

	Serial.print("Pitch, Roll: ");
	Serial.print(pitch, 2);
	Serial.print(", ");
	Serial.println(roll, 2);
}



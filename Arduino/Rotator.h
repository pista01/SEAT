#ifndef Rotator_h
#define Rotator_h

/***********************************
Azimuth control library


****************************************/
#include "Arduino.h"
#include "config.h"
#include <DuePWM.h>
#include "VNH5019MotorShield.h"
#include <Adafruit_BNO055.h>
//#include "PID_v1.h"



class Rotator {
 public:
  // CONSTRUCTORS
  Rotator();
 
  
  // PUBLIC METHODS
   void init(int, int, String); 
   void processPosition(void);
   void track(float,float);
   void park(void);
   double get_az_absolute_pos(void);
   double get_el_absolute_pos(void);
   double get_az_current_deg(void);
   double get_az_target_deg(void);
   double get_el_current_deg(void);
   double get_el_target_deg(void);
   int getAZEncoderCount(void);
   int getELEncoderCount(void);
   String getAZMotorCurrent(void);
   String getELMotorCurrent(void);
   String getAZDegPerSec(void);
   String getELDegPerSec(void);
   void CalculateNewPos(void);
   void displaySensorOffsets(void);
   void displayCalStatus(void);
   void displaySensorStatus(void);
   void displaySensorDetails(void);
   void setSensorOffsets(String);
   adafruit_bno055_offsets_t getSensorOffsets(void);
   String getCalStatus(void);
   void sanityCheckPosition(void);
   boolean calibrateToZero(void);
   
   
   String get_PID(String);
   void set_new_PID(String, double, double, double);
   void enable_tracking(void);
   void disable_tracking(void);
   void set_az_target_deg(float);
   void set_el_target_deg(float);
   boolean is_rot_tracking(void);
   boolean is_rot_parking(void);
   boolean is_rot_moving(void);
   boolean is_az_parked;
   float getMagHeading(void);
   float getMagEL(void);
  
   
   
   
   
	//DuePWM pwm(int, int);
	 //PID
	//From a start
	//double Kp_start;
	//double Ki_start;
	//double Kd_start;
	//After start but more than 10 degrees from target
	//double Kp_cruise;
	//double Ki_cruise;
	//double Kd_cruise;

	//Within 10 degrees of the target
	//double Kp_track;
	//double Ki_track ;
	//double Kd_track;
	
	double az_target_multiplier;
	boolean plus_360;
	boolean is_az_parking;
	boolean is_az_blocked;
	boolean is_el_parking;
	boolean is_tracking;
	boolean is_calibrating;
	int az_move_counter;
	
	unsigned long prev_set_time;
	unsigned long AZ_last_set_time;
	unsigned long AZ_motor_start_time;
	//unsigned long prev_cmd_AZ_time;
	
	double el_target_multiplier;
	//unsigned long EL_prev_set_time;
	unsigned long EL_last_set_time;
	unsigned long EL_motor_start_time;
	//unsigned long prev_cmd_EL_time;
	
	
	float AZ_prev_target_deg;
	float AZ_target_deg;
	float floating_az_target_deg;
	boolean is_el_parked;
	
	
	float EL_prev_target_deg;
	float EL_target_deg;
	float floating_el_target_deg;
	int el_move_counter;
	
	//volatile float az_current_deg;
	//boolean AZ_forward_dir;
	
	//double AZ_Setpoint;
	//double AZ_Input;
	//double AZ_Output;
  
  // PRIVATE METHODS
 private:
	void stopIfFaultAZ(void);
	void stopIfFaultEL(void);
	void processAZPosition(void);
	void processELPosition(void);
	void TrackAZ(void);
	void TrackEL(void);
	double get_az_plus_360(void);
	void stop_az_motor(void);
	
	
	int AZ_motor_speed;
	void stop_el_motor(void);
	
	String get_EL_PID(String);
	int EL_motor_speed;
	
	
  
  
};

	
	//extern volatile int az_state;
	void AZStateChange();
	void ELStateChange();
	
	//extern void stopIfFault();

#endif
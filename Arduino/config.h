/*---------------- macros -----------------*/
//7fb05c60e1ed60c8128fc342c8e92596dac107ab
#define PWM_FREQ1 20000
#define PWM_FREQ2 20000
#define AZ_PULSES_PER_DEG 182.9685185185185
#define EL_PULSES_PER_DEG 182.9685185185185
#define MIN_AZ -180
#define MAX_AZ 540
#define AZ_MOTOR_START_SPEED 62 //The speed at which we can reliably start the motor
#define AZ_MOTOR_RAMP_CHANGE_INTERVAL 15 //Number of ms to wait between speed changes when ramping motor speed up or down
#define EL_MOTOR_START_SPEED 62 //The speed at which we can reliably start the motor
#define EL_MOTOR_RAMP_CHANGE_INTERVAL 15 //Number of ms to wait between speed changes when ramping motor speed up or down
#define PARK_DEG 180 // Target degree for the park.  Must be between 10 and 350 (to allow for overshooting the target
#define DISABLE_TRACKING_DELAY 10000  //Wait X seconds for no commmands from the tracker, then park
//Can't just monitor client.connected() because gpredict appears to open
//new sessions with each position.  Both 'P' and 'p' commands are counted


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO true




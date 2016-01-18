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

//LSM9DSO
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f





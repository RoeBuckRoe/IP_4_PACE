

/* CMSIS */

   /*
   * Apply Swing Up algorithm developed by Markus Dauberschmidt
   */

   #define swing_up 1

   /*
   * Apply acceleration
   */
   #define M_TWOPI         (M_PI * 2.0)
   #define ACCEL_CONTROL_DATA 0		// Set to 1 for display of timing data
   #define ACCEL_CONTROL 1 			// Set to 1 to enable acceleration control. Set to 0 to use position target control.
   #define PWM_COUNT_SAFETY_MARGIN 2
   #define MAXIMUM_ACCELERATION 131071
   #define MAXIMUM_DECELERATION 131071
   #define MAXIMUM_SPEED 131071

   #define RCC_SYS_CLOCK_FREQ 84000000 // should equal HAL_RCC_GetSysClockFreq()
   #define RCC_HCLK_FREQ 84000000 // should equal HAL_RCC_GetHCLKFreq()


   #define T_SAMPLE_DEFAULT 0.002

   #define ENABLE_HIGH_SPEED_SAMPLING_MODE 			0
   #define SAMPLE_BAUD_RATE							230400

   #define CONTROLLER_GAIN_SCALE 						1
   #define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.888889	//	Stepper position read value in steps per degree
   #define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	STEPPER_READ_POSITION_STEPS_PER_DEGREE
   #define ENCODER_READ_ANGLE_SCALE 					6.666667 // Angle Scale 6.66667 for 600 Pulse Per Rev Resolution Optical Encoder
   #define FULL_STATE_FEEDBACK_SCALE 					1.00 // Scale factor for Full State Feedback Architecture

   #define ENABLE_CYCLE_DELAY_WARNING 1			// Enable warning and control loop exit if loop delay exceeds threshold

   #define ENCODER_ANGLE_POLARITY -1.0				// Note that physical system applies negative polarity to pendulum angle
                                       // by definition of coordinate system.

   #define CYCLE_LIMIT 100000 						// Cycle limit determines run time as product of cycle limit and cycle time (typical 5 msec).
   #define ENABLE_CYCLE_INFINITE 1 				// ENABLE_CYCLE_INFINITE set to 1 if continuous operation is to be enabled


   #define MAX_TORQUE_CONFIG 800 					// 400 Selected Value for normal control operation
   #define MAX_TORQUE_SWING_UP 800					// 800 Selected Value for Swing Up operation

   #define OVERCURRENT_THRESHOLD 2000				// 2000 Selected Value for Integrated Rotary Inverted Pendulum System
   #define SHUTDOWN_TORQUE_CURRENT 0				// 0 Selected Value for Integrated Rotary Inverted Pendulum System
   #define TORQ_CURRENT_DEFAULT MAX_TORQUE_CONFIG				// Default torque current	// Default torque current

   /*
   * Note that Speed Profiles are set at run time during execution.
   *
   * Default speed Profiles are set in l6474_target_config.h
   */

   #define MAX_SPEED_UPPER_INIT 10000						// Initialization value
   #define MIN_SPEED_UPPER_INIT 10000						// Initialization value
   #define MAX_SPEED_LOWER_INIT 30							// Initialization value
   #define MIN_SPEED_LOWER_INIT 30							// Initialization value
   #define MAX_ACCEL_UPPER_INIT 10000			 			// Initialization value
   #define MAX_DECEL_UPPER_INIT 10000	 					// Initialization value

   /*
   * Configuration of Motor Speed Profile at initialization
   */

   #define MAX_SPEED 2000
   #define MIN_SPEED 800
   #define MAX_ACCEL 6000
   #define MAX_DECEL 6000

   #define MAX_SPEED_MODE_2 2000
   #define MIN_SPEED_MODE_2 1000
   #define MAX_SPEED_MODE_1 2000
   #define MIN_SPEED_MODE_1 800
   #define MAX_SPEED_MODE_3 2000
   #define MIN_SPEED_MODE_3 600
   #define MAX_SPEED_MODE_4 2000
   #define MIN_SPEED_MODE_4 400
   #define MAX_SPEED_MODE_5 2000
   #define MIN_SPEED_MODE_5 800

   #define ENABLE_SUSPENDED_PENDULUM_CONTROL 0     // Set to 0 for Inverted Pendulum Mode - Set to 1 for Suspended Pendulum Mode

   #define ENCODER_START_OFFSET 0 				// Encoder configurations may display up to 1 degree initial offset
   #define ENCODER_START_OFFSET_DELAY 0			// Encoder offset delay limits application of initial offset
   #define START_ANGLE 1							// Pendulum angle tolerance for system pendulum orientation at start
   #define START_ANGLE_DELAY 0						// Delay at start for orientation of pendulum upright

   /*
   * Pendulum Swing Up Configuration
   */

   /*
   * Iinital Measurement of Edukit Platform angle relative to vertical.
   *
   * The Edukit system may be resting on a sloped surface.  Therefore, accurate measurement of Pendulum upright angle
   * includes a slope error.  This is corrected for by the Angle Calibration System that operates at start time
   *
   */

   #define ENABLE_ANGLE_CAL 1
   #define ANGLE_CAL_OFFSET_STEP_COUNT 1801	// Full span angle range from negative to positive rotor angle
   #define ANGLE_AVG_SPAN 50 					// Angle element smoothing span in rotor steps reduces full span by twice set value
   #define ANGLE_CAL_ZERO_OFFSET_DWELL 5000	// 10 second zero offset measurement period
   #define ANGLE_CAL_ZERO_OFFSET_SETTLING 2000 // 4 second settling period after offset correction
   #define ANGLE_CAL_COMPLETION 2000			// 4 seconds settling period after final offset update prior to assigning new controller


   /*
   * Single PID, Dual PID and LQR Controllers are implemented as summation of Primary
   * and Secondary PID controller structures.  These two controller outputs are summed
   * and supplied to Rotor Control
   *
   * PID Controller parameters.  LQR Controllers are implemented with integral gain of 0.
   *
   */

   #define ENABLE_PID_INTEGRATOR_LIMIT 0			// Enable PID filter integrator limit
   #define PRIMARY_WINDUP_LIMIT 1000				// Integrator wind up limits for PID Pendulum controller
   #define SECONDARY_WINDUP_LIMIT 1000				// Integrator wind up limits for PID Rotor controller

   /*
   * High Speed Mode Values: 5, 25, 30
   */
   #define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 10  		// 10 - Corner frequency of low pass filter of Primary PID derivative
   #define LP_CORNER_FREQ_ROTOR 100 						// 100 - Corner frequency of low pass filter of Rotor Angle
   #define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 - Corner frequency of low pass filter of Secondary PID derivative
   #define LP_CORNER_FREQ_STEP 50							// Low pass filter operating on rotor reference step command signal


   /* Mode 1 is Dual PID Demonstration Mode */
   #define PRIMARY_PROPORTIONAL_MODE_1 	300
   #define PRIMARY_INTEGRAL_MODE_1     	0.0
   #define PRIMARY_DERIVATIVE_MODE_1   	30

   #define SECONDARY_PROPORTIONAL_MODE_1 	15.0
   #define SECONDARY_INTEGRAL_MODE_1     	0.0
   #define SECONDARY_DERIVATIVE_MODE_1   	7.5

   #define PRIMARY_PROPORTIONAL_MODE_2 	518.0
   #define PRIMARY_INTEGRAL_MODE_2     	0
   #define PRIMARY_DERIVATIVE_MODE_2   	57.0

   #define SECONDARY_PROPORTIONAL_MODE_2 	2.20
   #define SECONDARY_INTEGRAL_MODE_2     	0.0
   #define SECONDARY_DERIVATIVE_MODE_2   	4.82

   #define PRIMARY_PROPORTIONAL_MODE_3 	300
   #define PRIMARY_INTEGRAL_MODE_3     	0.0
   #define PRIMARY_DERIVATIVE_MODE_3   	30.0

   #define SECONDARY_PROPORTIONAL_MODE_3 	15.0
   #define SECONDARY_INTEGRAL_MODE_3     	0.0
   #define SECONDARY_DERIVATIVE_MODE_3   	15.0

   #define PRIMARY_PROPORTIONAL_MODE_5 	300
   #define PRIMARY_INTEGRAL_MODE_5     	0.0
   #define PRIMARY_DERIVATIVE_MODE_5   	30.0

   #define SECONDARY_PROPORTIONAL_MODE_5 	15.0
   #define SECONDARY_INTEGRAL_MODE_5     	0.0
   #define SECONDARY_DERIVATIVE_MODE_5   	15.0

   /*
   * Single PID Mode Gains
   */

   #define ROTOR_PID_PROPORTIONAL_GAIN_SINGLE_PID_MODE  15.0
   #define ROTOR_PID_INTEGRAL_GAIN_SINGLE_PID_MODE		 0.0
   #define ROTOR_PID_DIFFERENTIAL_GAIN_SINGLE_PID_MODE	 7.5

   /*
   * Mode 4 is Dual PID Suspended Demonstration Mode
   */

   #define PRIMARY_PROPORTIONAL_MODE_4 	-10.0
   #define PRIMARY_INTEGRAL_MODE_4     	 0.0
   #define PRIMARY_DERIVATIVE_MODE_4   	-5.0

   #define SECONDARY_PROPORTIONAL_MODE_4 	-2.0
   #define SECONDARY_INTEGRAL_MODE_4     	 0.0
   #define SECONDARY_DERIVATIVE_MODE_4   	-2.0


   #define DEFAULT_START_MODE	mode_1

   #define ENABLE_ADAPTIVE_MODE 0
   #define ADAPTIVE_THRESHOLD_LOW 30				// Default value 30
   #define ADAPTIVE_THRESHOLD_HIGH 2				// Default value 2
   #define ADAPTIVE_STATE 0
   #define ADAPTIVE_DWELL_PERIOD 2000				// Determines dwell period during state transition

   #define USER_TRANSITION_DWELL 500

   /*
   * START_DEFAULT_MODE_TIME determines time delay for waiting for user input after start or reset.
   * For a time (in ticks) greater than this period, control will initiate with default mode 1 if
   * no user input appears.
   *
   * This permits system operation in default mode independent of external command from separate
   * host.
   */

   #define START_DEFAULT_MODE_TIME 5000			// 5 second delay to permit user input after system start
                                       // If no user response then set default values
   #define PENDULUM_ORIENTATION_START_DELAY 10000	// Time permitted to user to orient Pendulum vertical at start

   #define INITIAL_START_DELAY 1000				// Determines time available for ensuring pendulum down and prior to user prompt
   #define CONTROL_START_DELAY 1000 				// Determines time available to user after prompt for adjusting pendulum upright
   #define INITIAL_PENDULUM_MOTION_TEST_DELAY 2000 // Determines delay time between successive evaluations of pendulum motion

   #define ENABLE_CONTROL_ACTION 1							// Enable control operation - default value of 1 (may be disabled for test operations)
   #define ENABLE_DUAL_PID 1						// Note ENABLE_DUAL_PID is set to 1 by default for summation of PID controllers
                                       // for either Dual PID or LQR systems

   #define STATE_FEEDBACK_CONFIG_ENABLE 1			// Default selection of Dual PID Architecture - Set to 1 for State Feedback			1

   /*
   * Swing Up System Parameters : Swing Up Algorithm developed and provided by Markus Dauberschmidt
   * Please see
   */
   #define ENABLE_SWING_UP 1						// Enable Pendulum Swing Up system
   #define SWING_UP_CONTROL_CONFIG_DELAY 3000 		// Delay in cycles prior to switching from Swing Up controller to user selected controller
   #define STAGE_0_AMP 200							// Swing Up Impulse amplitude for initial state
   #define STAGE_1_AMP 130							// Swing Up Impulse amplitude for intermediate state
   #define STAGE_2_AMP 120							// Swing Up Impulse amplitude for final state



   /*
   * ENCODER ANGLE SLOPE CORRECTION compensates for any error introduced by a platform tilt relative to vertical.
   * The correction is computed over a time constant of greater than 100 seconds to avoid any distortion in measurements
   * that are conducted over shorter intervals of up to 10 seconds.
   */
   #define ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION 	0		// Default 1 for system operation enable
   #define OFFSET_FILTER_GAIN 						1		// Offset compensation gain value
   #define LP_CORNER_FREQ_LONG_TERM 				0.01	// Corner frequency of low pass filter - default to 0.001
   #define ENCODER_ANGLE_SLOPE_CORRECTION_SCALE 	200		// Set to 200
   #define ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT	0	// Sets limit on operation time for slope angle correction
                                             // If set to zero, slope correction operates at all times
                                             // Default set to zero
   /*
   * Rotor position limits are defined to limit rotor rotation to one full rotation in clockwise or
   * counterclockwise motion.
   */

   #define ROTOR_POSITION_POSITIVE_LIMIT 240		// Maximum allowed rotation in positive angle in degrees
   #define ROTOR_POSITION_NEGATIVE_LIMIT -240		// Minimum allowed rotation in negative angle in degrees

   /*
   * Encoder position limits are defined to detect excursions in pendulum angle corresponding to
   * departure from control and to initiate control loop exit
   */

   #define ENCODER_POSITION_POSITIVE_LIMIT  120		// Maximum allowed rotation in positive angle in steps
   #define ENCODER_POSITION_NEGATIVE_LIMIT -120		// Minimum allowed rotation in negative angle in steps

   #define ENABLE_TORQUE_CURRENT_ENTRY		0		    // Enables user input of torque current configuration in general mode
   /*
   * Setting ENABLE_MOD_SIN_ROTOR_TRACKING to 1 enables a Rotor Position tracking command in the
   * form of an amplitude modulated sine wave signal
   * Frequency units and Rate are Hz
   * Amplitude units are steps
   *
   */

   #define ENABLE_MOD_SIN_ROTOR_TRACKING 1		// If selected, disable all other modulation inputs
   #define MOD_SIN_CARRIER_FREQ 0.15			// 0.15 default
   #define MOD_SIN_START_CYCLES 5000			// Sine modulation starts at completion of angle calibration if enabled
   #define MOD_SIN_AMPLITUDE 300				// 400 default
   #define MOD_SIN_MODULATION_FREQ  0.02		// 0.02 default
   #define MOD_SIN_MODULATION_MIN 0			// Default 0
   /* Define for High Speed System */
   #define MOD_SIN_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)  // Equals system sample rate
   #define ENABLE_SIN_MOD 1					// 1 default

   #define ENABLE_ROTOR_CHIRP 0				// If selected, disable all other modulation inputs
   #define ROTOR_CHIRP_START_FREQ 0.01			// 0.001 default
   #define ROTOR_CHIRP_END_FREQ 15				// 5 default
   #define ROTOR_CHIRP_PERIOD 20000			// 20000 default
   #define ROTOR_CHIRP_SWEEP_DELAY 1000		// 0 default - enables control system to recover between sweeps
   /* Define for High Speed System */
   #define ROTOR_CHIRP_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)  // Equals system sample rate
   #define ROTOR_CHIRP_START_CYCLES 			// 0 default
   #define ROTOR_CHIRP_STEP_AMPLITUDE 2  		// 0.3 default

   #define ENABLE_ROTOR_TRACK_COMB_SIGNAL 0				// If selected, disable all other modulation inputs
   #define ROTOR_TRACK_COMB_SIGNAL_SAMPLE_RATE 500.0		// 500.0 default for Low Speed
   #define ROTOR_TRACK_COMB_SIGNAL_START_CYCLES 0			// 0 default
   #define ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE 5.0			// 0.05 default

   #define ENABLE_DISTURBANCE_REJECTION_STEP 	1
   #define LOAD_DISTURBANCE_SENSITIVITY_SCALE 	20			// Scale factor applied to increase measurement resolution for Load Disturbance Sensitivity Function

   /*
   * Set ENABLE_ENCODER_TEST to 1 to enable a testing of encoder response for verification
   * occurring prior to control system start.
   *
   * This will be set to 0 and disabled for normal operation
   *
   */

   #define ENABLE_ENCODER_TEST 0

   /*
   * Set ENABLE_ROTOR_ACTUATOR_TEST to 1 to enable a testing of encoder response for verification
   * occurring prior to control system start.
   *
   * This will be set to 0 and disabled for normal operation
   *
   */

   #define ENABLE_ROTOR_ACTUATOR_TEST 0
   #define ROTOR_ACTUATOR_TEST_CYCLES 1

   /*
   * Setting ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE = 1 applies a Rotor Position tracking
   * command input step signal
   */

   #define ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE 1			// If selected, disable all other modulation inputs
   #define ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE 20		// Default 8. Amplitude of step cycle. Note: Peak-to-Peak amplitude is double this value
   #define ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL 16384 	// Default 10240
   #define STEP_RESPONSE_AMP_LIMIT_ENABLE 0					// Enables limit of Step Response if rotor amplitude exceeds limit
                                                // Useful for protecting operation if summing step and sine drive
   #define STEP_RESPONSE_AMP_LIMIT 350							// Angle limit for Step Response action
   /*
   * Setting ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE = 1 applies a Rotor Position tracking
   * command input impulse signal
   */
   #define ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE 0			// If selected, disable all other modulation inputs
   #define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE 8		// Amplitude of impulse in degrees
   #define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD 500 		// Duration of impulse in cycles
   #define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 5000	    // Interval between impulse events in cycles
   /* Define for High Speed System */
   #define ROTOR_IMPULSE_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)  		// Equals system sample rate							// Default sample rate
   /*
   * Setting ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE = 1 applies a Pendulum Position tracking
   * command input impulse signal
   */
   #define ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE 0		// If selected, disable all other modulation inputs
   #define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE 500	// Amplitude of step cycle in steps equaling 75 degrees. Note: Peak-to-Peak amplitude is double this value
   #define PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE 4				// Amplitude scaling of impulse for Suspended and Inverted Mode
   #define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD 2		// Duration of impulse in cycles
   #define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 18000	// Interval between impulse events in cycles
   /* Define for High Speed System */
   #define PENDULUM_IMPULSE_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)       // Equals system sample rate 						// Default sample rate

   /*
   * DATA_REPORT_SPEED_SCALE enables reduced data rate for bandwidth constrained data acquisition systems
   */

   #define DATA_REPORT_SPEED_SCALE 20


#ifndef MFI_INVERTEDPENDULUM_GAM_H_
#define MFI_INVERTEDPENDULUM_GAM_H_

#include "GAM.h"
#include "Constants.h"

namespace MFI {

 /*
   * Structure for the augmented floating-point PID Control.
   * This includes additional state associated with derivative filter
   *
   * This follows the ARM CMSIS architecture
   */

   typedef struct
   {
      float state_a[4];  /** The filter state array of length 4. */
      float Kp;          /** The proportional gain. */
      float Ki;          /** The integral gain. */
      float Kd;          /** The derivative gain. */
      float int_term;    /** The controller integral output */
      float control_output; /** The controller output */
   } arm_pid_instance_a_f32;


//enum MOTOR_DIRECTION { FORWARD=1, BACKWARD };
/// Direction options
typedef enum {
  BACKWARD = 0u,
  FORWARD = 1u,
  UNKNOW_DIR = ((u_int8_t)0xFF)
} MOTOR_DIRECTION;


typedef enum {
  STATE_INITIALIZATION = 0u,
  STATE_PENDULUM_STABLIZATION = 1u,
  STATE_SWING_UP = 2u,
  STATE_MAIN = 3u
} STATE;



/**
 * @brief A GAM for controlling an inverted pendulum
 * 
 * @details The GAM operates an inverted pendulum set-up which has a motor with a rotor attached 
 * to it and a pendulum attached to the to an encoder 
 * 
 * The GAM shall have the following configuration (the object name `InvertedPendulumGAM` is an example - 
 * it is arbitrary):
 * <pre>
 * +InvertedPendulumGAM = {
 *    Class = InvertedPendulumGAM
 *    InputSignals = { 
 *       encoder_counter = {
 *          DataSource = STM32
 *       }
 *       rotor_position_steps = {
 *          DataSource = STM32
 *       }
 *    }
 *    OutputSignals = {               
 *       motor_ImpuleAmplitude = {
 *          DataSource = STM32
 *       Type = int32
 *       }
 *       motor_Direction = {
 *          DataSource = STM32
 *          Type = uint8
 *       }
 *       motor_Acceleration = {
 *          DataSource = STM32
 *          Type = int32
 *       }
 *       break_Control_Loop = {
 *          DataSource = STM32
 *          Type = uint8
 *       }
 *       state = {
 *          DataSource = STM32
 *          Type = uint8
 *       }
 *       encoder_position = {
 *          DataSource = STM32
 *          Type = float32
 *       }            
 *    }
 * }
 * </pre>
 * 
 * 
 * The input signals are:
 * 
 * rotor_position_steps - this is the current position of the motor rotor. It should typically be read from the motor hardware datasource
 * encoder_counter - this is the current counter of the encoder attached with the pendulum. 
 * 
 * The output signals are:
 *
 * motor_ImpuleAmplitude - this is the impulse amplitude produced by the GAM for swinging the pendulum by deriving the motor to this current value
   motor_Direction - this is the direction of pendulum swigning
   motor_Acceleration - this is the control signal produced to control accelelaration of the pendulum
   break_Control_Loop - this is a parameter that signifies breaking of the control loop and re-initialization of the pendulum system
   state - this communicates the current execution state of the GAM
   encoder_position - this is the computed encoder position

 * The GAM operates in four execution states: (0)INITIALIZATION, (1)PENDULUM_STABLIZATION, (2)SWING_UP, and (4)MAIN
 * GAM is:
 * 
 * 
 */
class InvertedPendulumGAM : public MARTe::GAM {


private:
   
   //********************############### Input Signals #########################*************************************
   MARTe::int32*  INPUT_rotor_position_steps;
   MARTe::uint32* INPUT_encoder_counter;
   //********************########################################################*************************************

   //********************############### Out Signals #########################*************************************
   MARTe::int32*   OUTPUT_motor_ImpuleAmplitude;
   MARTe::uint8*   OUTPUT_motor_Direction;
   MARTe::int32*   OUTPUT_motor_Acceleration;
   MARTe::uint8*   OUTPUT_break_Control_Loop;
   MARTe::uint8*   OUTPUT_state;
   MARTe::float32* OUTPUT_encoder_position;
   //********************########################################################*************************************


 public:
    CLASS_REGISTER_DECLARATION();
    
    InvertedPendulumGAM();
    virtual ~InvertedPendulumGAM();

   bool control_logic_State_Main();
   void control_logic_State_Main_Prepare();

   bool control_logic_State_PendulumStablisation();
   void control_logic_State_PendulumStablisation_Prepare();
   int control_logic_State_PendulumStablisation_testCount;
   bool control_logic_State_PendulumStablisation_isSecondRead;

   bool control_logic_State_SwingingUp();
   void control_logic_State_SwingingUp_Prepare();
   bool control_logic_State_SwingingUp_checkSwingUp();

   void control_logic_State_Initialization();
   
   void control_logic_Initialise();
   int encoder_position_read(int *encoder_position, int encoder_position_init );

   void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error, float * sample_period, float * Deriv_Filt);

   
   void assign_mode_1(arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);
   void assign_mode_2(arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);
   void assign_mode_3(arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);

    void show_error();

    virtual bool Initialise(MARTe::StructuredDataI & data);

    virtual bool Setup();

    virtual bool Execute();    

 private:
   
   STATE state;

   // volatile u_int16_t gLastError;
   /* Private function prototypes -----------------------------------------------*/
   void Error_Handler(u_int16_t error);
   void select_mode_1(void);
   void restart_execution(void);
   void user_configuration(void);
   void Main_StepClockHandler();
   void apply_acceleration(float * acc, float* target_velocity_prescaled, float t_sample);
   float L6474_Board_Pwm1PrescaleFreq( float freq );

   
   /* System data reporting */
   char msg_display[256];
   
     /* Control system output signal */
   float rotor_control_target_steps;
   float rotor_control_target_steps_curr;
   float rotor_control_target_steps_prev;

   /* Control system variables */
   int rotor_position_delta;
   //int initial_rotor_position;
   int cycle_count;
   int i, j, k, m;
   int ret;

   /* PID control system variables */
   float windup, rotor_windup;
   float *current_error_steps, *current_error_rotor_steps;
   float *sample_period, *sample_period_rotor;

   /* Loop timing measurement variables */
   int cycle_period_start;
   int cycle_period_sum;
   int enable_cycle_delay_warning;

   /* PID control variables */
   float *deriv_lp_corner_f;
   float *deriv_lp_corner_f_rotor;
   float proportional, rotor_p_gain;
   float integral, rotor_i_gain;
   float derivative, rotor_d_gain;

   /* State Feedback variables */
   int enable_state_feedback;
   float integral_compensator_gain;
   float feedforward_gain;
   float current_error_rotor_integral;

   /* Reference tracking command */
   float reference_tracking_command;

   /* Pendulum position and tracking command */

   /* Rotor position and tracking command */
   int rotor_position_steps;
   float rotor_position_command_steps;
   float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
   float rotor_position_command_deg;
   float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
   float rotor_position_diff, rotor_position_diff_prev;
   float rotor_position_diff_filter, rotor_position_diff_filter_prev;
   int rotor_target_in_steps;
   int initial_rotor_position;

   /* Rotor Plant Design variables */
   int select_rotor_plant_design, enable_rotor_plant_design, enable_rotor_plant_gain_design;
   int rotor_control_target_steps_int;
   float rotor_damping_coefficient, rotor_natural_frequency;
   float rotor_plant_gain;
   float rotor_control_target_steps_gain;
   float rotor_control_target_steps_filter_2, rotor_control_target_steps_filter_prev_2;
   float rotor_control_target_steps_prev_prev, rotor_control_target_steps_filter_prev_prev_2;
   float c0, c1, c2, c3, c4, ao, Wn2;
   float fo_r, Wo_r, IWon_r, iir_0_r, iir_1_r, iir_2_r;

   /* Encoder position variables */
   u_int32_t cnt3;
   int range_error;
   float encoder_position;
   int encoder_position_steps;
   int encoder_position_init;
   int previous_encoder_position;
   int max_encoder_position;
   int global_max_encoder_position;
   int prev_global_max_encoder_position;
   int encoder_position_down;
   int encoder_position_curr;
   int encoder_position_prev;

   /* Angle calibration variables */
   float encoder_position_offset;
   float encoder_position_offset_zero;
   int enable_angle_cal;
   int enable_angle_cal_resp;
   int offset_end_state;
   int offset_start_index;
   int angle_index;
   int angle_avg_index;
   int angle_avg_span;
   int offset_angle[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
   float encoder_position_offset_avg[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
   int angle_cal_end;
   int angle_cal_complete;

   /* Swing Up system variables */
   int enable_swing_up;
   int enable_swing_up_resp;
   bool peaked;
   bool handled_peak;
   bool zero_crossed;
   MOTOR_DIRECTION swing_up_direction;
   int swing_up_state, swing_up_state_prev;
   int stage_count;
   int impulse_amp;


   /* Low pass filter variables */
   float fo, Wo, IWon, iir_0, iir_1, iir_2;
   float fo_LT, Wo_LT, IWon_LT;
   float iir_LT_0, iir_LT_1, iir_LT_2;
   float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

   /* Slope correction system variables */
   int slope;
   int slope_prev;
   float encoder_angle_slope_corr_steps;

   /* Adaptive control variables */
   float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
   float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
   int adaptive_entry_tick, adaptive_dwell_period;
   int enable_adaptive_mode, adaptive_state, adaptive_state_change;
   float rotor_position_command_steps_prev;

   /* Rotor impulse variables */
   int rotor_position_step_polarity;
   int impulse_start_index;

   /* User configuration variables */
   int clear_input;
   int max_speed_read, min_speed_read;
   int select_suspended_mode;
   int motor_response_model;
   int enable_rotor_actuator_test, enable_rotor_actuator_control;
   int enable_encoder_test;
   int enable_rotor_actuator_high_speed_test;
   int enable_motor_actuator_characterization_mode;
   int motor_state;
   float torq_current_val;


   /* Rotor chirp system variables */
   int enable_rotor_chirp;
   int chirp_cycle;
   int chirp_dwell_cycle;
   float chirp_time;
   float rotor_chirp_start_freq;
   float rotor_chirp_end_freq;
   float rotor_chirp_period ;
   float rotor_chirp_frequency;
   float rotor_chirp_amplitude;
   int rotor_chirp_step_period;

   float pendulum_position_command_steps;

   /* Modulates sine tracking signal system variables */
   int enable_mod_sin_rotor_tracking;
   int enable_rotor_position_step_response_cycle;
   int disable_mod_sin_rotor_tracking;
   int sine_drive_transition;
   float mod_sin_amplitude;
   float rotor_control_sin_amplitude;
   float rotor_sine_drive, rotor_sine_drive_mod;
   float rotor_mod_control;
   float mod_sin_carrier_frequency;

   /* Pendulum impulse system variables */
   int enable_pendulum_position_impulse_response_cycle;

   /* Rotor high speed test system variables */
   int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
   int rotor_test_acceleration_max, swing_deceleration_max;
   int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
   int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
   u_int16_t current_speed;

   /*Pendulum system ID variable */
   int enable_pendulum_sysid_test;

   /* Full system identification variables */
   int enable_full_sysid;
   float full_sysid_max_vel_amplitude_deg_per_s;
   float full_sysid_min_freq_hz;
   float full_sysid_max_freq_hz;
   int full_sysid_num_freqs;
   float full_sysid_freq_log_step;
   int full_sysid_start_index;

   /* Rotor comb drive system variables */
   int enable_rotor_tracking_comb_signal;
   float rotor_track_comb_signal_frequency;
   float rotor_track_comb_command;
   float rotor_track_comb_amplitude;

   /* Sensitivity function system variables */
   int enable_disturbance_rejection_step;
   int enable_noise_rejection_step;
   int enable_plant_rejection_step;
   int enable_sensitivity_fnc_step;
   float load_disturbance_sensitivity_scale;

   /* Noise rejection sensitivity function low pass filter */

   float noise_rej_signal_filter, noise_rej_signal;
   float noise_rej_signal_prev, noise_rej_signal_filter_prev;


   int mode_index_prev, mode_index_command;
   int mode_transition_tick;
   int mode_transition_state;
   int transition_to_adaptive_mode;

   /*
   * Real time user input system variables
   */

   char config_message[16];
   int config_command;
   int display_parameter;
   int step_size;
   float adjust_increment;
   int mode_index;

   /* Real time data reporting index */
   int report_mode;
   int speed_scale;
   int speed_governor;

   


   /* CMSIS Variables */
   arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
   float Deriv_Filt_Pend[2];
   float Deriv_Filt_Rotor[2];
   float Wo_t, fo_t, IWon_t;

   /* System timing variables */


   u_int32_t t_sample_cpu_cycles;
   float Tsample, Tsample_rotor, test_time;
   float angle_scale;
   int enable_high_speed_sampling;

   /* Reset state tracking */
   int reset_state;

   /* Motor configuration */
   u_int16_t min_speed, max_speed, max_accel, max_decel;

   /* Serial interface variables */
   u_int32_t RxBuffer_ReadIdx;
   u_int32_t RxBuffer_WriteIdx;
   u_int32_t readBytes;


   float init_r_p_gain;
   float init_r_i_gain;
   float init_r_d_gain;
   float init_p_p_gain;
   float init_p_i_gain;
   float init_p_d_gain;
   int init_enable_state_feedback;
   int init_integral_compensator_gain;
   int init_feedforward_gain;
   int init_enable_disturbance_rejection_step;
   int init_enable_sensitivity_fnc_step ;
   int init_enable_noise_rejection_step;
   int init_enable_rotor_plant_design;
   int init_enable_rotor_plant_gain_design;


};

} // namespace MFI

#endif // MFI_INVERTEDPENDULUM_GAM_H_

#include "InvertedPendulumGAM.h"
#include "AdvancedErrorManagement.h" 
#include "GAMSignalChecker.h"
#include <string.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <assert.h>

using namespace MARTe;

namespace MFI {

InvertedPendulumGAM::InvertedPendulumGAM() : GAM() {

   INPUT_rotor_position_steps    = NULL_PTR(int32*);
   INPUT_encoder_counter        = NULL_PTR(uint32*);

   OUTPUT_motor_ImpuleAmplitude    = NULL_PTR(int32*);
   OUTPUT_motor_Direction                     = NULL_PTR(uint8*);
   OUTPUT_motor_Acceleration        = NULL_PTR(int32*);
   OUTPUT_break_Control_Loop            = NULL_PTR(uint8*);
   OUTPUT_state                         = NULL_PTR(uint8*);
   OUTPUT_encoder_position              = NULL_PTR(float32*);

}

InvertedPendulumGAM::~InvertedPendulumGAM() {

}

void InvertedPendulumGAM::control_logic_Initialise() {
    /* Initialize reset state indicating that reset has occurred */

    state = STATE_INITIALIZATION;

	reset_state = 1;

	/* Initialize default start mode and reporting mode */
	mode_index = 1;
	report_mode = 1;

	/*Initialize serial read variables */
	RxBuffer_ReadIdx = 0u;
	RxBuffer_WriteIdx = 0u;
	readBytes = 0u;

	/*Initialize encoder variables */
	encoder_position = 0;
	encoder_position_down = 0;
	encoder_position_curr = 0;
	encoder_position_prev = 0;
	angle_scale = ENCODER_READ_ANGLE_SCALE;

	/*Initialize rotor control variables */
	rotor_control_target_steps = 0;
	rotor_control_target_steps_curr = 0;
	rotor_control_target_steps_prev = 0;

	/*Initialize rotor plant design transfer function computation variables */
	rotor_control_target_steps_filter_prev_2 = 0.0;
	rotor_control_target_steps_filter_prev_prev_2 = 0.0;
	rotor_control_target_steps_prev_prev = 0.0;

	/* Initialize LQR integral control variables */
	current_error_rotor_integral = 0;

	/*Initialize rotor tracking signal variables */
	enable_rotor_chirp = 0;
	rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	enable_rotor_position_step_response_cycle = ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	disable_mod_sin_rotor_tracking = 0;
	sine_drive_transition = 0;
	mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;

	/*Initialize sensitivity function selection variables */
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;
	enable_pendulum_position_impulse_response_cycle = 0;

	/*Initialize user adjustment variables */
	step_size = 0;
	adjust_increment = 0.5;


	/* Default select_suspended_mode */
	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;

	
	/* Default Starting Control Configuration */
	max_accel = (u_int16_t) MAX_ACCEL;
	max_decel = (u_int16_t) MAX_DECEL;
	max_speed = (u_int16_t) MAX_SPEED_MODE_1;
	min_speed = (u_int16_t) MIN_SPEED_MODE_1;
	

	/* Default controller gains */
	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;

	/* Enable State Feedback mode and Integral Action Compensator by default and set
	 * precompensation factor to unity
	 */
	enable_state_feedback = 1;
	integral_compensator_gain = 0;
	feedforward_gain = 1;

	/* Disable adaptive_mode by default */
	enable_adaptive_mode = 0;

	
	/* Controller structure and variable allocation */
	current_error_steps = (float*)malloc(sizeof(float));
	if (current_error_steps == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
        REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	current_error_rotor_steps = (float*)malloc(sizeof(float));
	if (current_error_rotor_steps == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	sample_period = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	deriv_lp_corner_f = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	deriv_lp_corner_f_rotor = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	sample_period_rotor = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}

	
	/* Configure controller filter and sample time parameters */
	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	t_sample_cpu_cycles = (u_int32_t) round(T_SAMPLE_DEFAULT * RCC_HCLK_FREQ);
	Tsample = (float) t_sample_cpu_cycles / RCC_HCLK_FREQ;
	*sample_period = Tsample;
	Tsample_rotor = Tsample;
	*sample_period_rotor = Tsample_rotor;

	/* PID Derivative Low Pass Filter Coefficients */

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

	
	/* Configure primary controller parameters */
	windup = PRIMARY_WINDUP_LIMIT;

	/* Configure secondary Rotor controller parameters */
	rotor_windup = SECONDARY_WINDUP_LIMIT;

	/* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction */
	fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * Tsample);
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);
	fo_s = LP_CORNER_FREQ_STEP;
	Wo_s = 2 * 3.141592654 * fo_s;
	IWon_s = 2 / (Wo_s * Tsample);
	iir_0_s = 1 / (1 + IWon_s);
	iir_1_s = iir_0_s;
	iir_2_s = iir_0_s * (1 - IWon_s);
	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * Tsample);
	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);
	/*
	 * Request user input for mode configuration
	 */

	enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	adaptive_threshold_low = ADAPTIVE_THRESHOLD_LOW;
	adaptive_threshold_high = ADAPTIVE_THRESHOLD_HIGH;
	adaptive_state = ADAPTIVE_STATE;
	adaptive_state_change = 0;
	adaptive_dwell_period = ADAPTIVE_DWELL_PERIOD;

}

void InvertedPendulumGAM::control_logic_State_Initialization(){

   
     user_configuration();


    PID_Pend.Kp = proportional * CONTROLLER_GAIN_SCALE;
    PID_Pend.Ki = integral * CONTROLLER_GAIN_SCALE;
    PID_Pend.Kd = derivative * CONTROLLER_GAIN_SCALE;

    PID_Rotor.Kp = rotor_p_gain * CONTROLLER_GAIN_SCALE;
    PID_Rotor.Ki = rotor_i_gain * CONTROLLER_GAIN_SCALE;
    PID_Rotor.Kd = rotor_d_gain * CONTROLLER_GAIN_SCALE;

    PID_Pend.Kp = proportional * CONTROLLER_GAIN_SCALE;
    PID_Pend.Ki = integral * CONTROLLER_GAIN_SCALE;
    PID_Pend.Kd = derivative * CONTROLLER_GAIN_SCALE;

    PID_Rotor.Kp = rotor_p_gain * CONTROLLER_GAIN_SCALE;
    PID_Rotor.Ki = rotor_i_gain * CONTROLLER_GAIN_SCALE;
    PID_Rotor.Kd = rotor_d_gain * CONTROLLER_GAIN_SCALE;

    PID_Pend.state_a[0] = 0;
    PID_Pend.state_a[1] = 0;
    PID_Pend.state_a[2] = 0;
    PID_Pend.state_a[3] = 0;

    PID_Rotor.state_a[0] = 0;
    PID_Rotor.state_a[1] = 0;
    PID_Rotor.state_a[2] = 0;
    PID_Rotor.state_a[3] = 0;

    integral_compensator_gain = integral_compensator_gain * CONTROLLER_GAIN_SCALE;

    /* Assign Rotor Plant Design variable values */


    /* Transfer function model of form 1/(s^2 + 2*Damping_Coefficient*Wn*s + Wn^2) */
    if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){
        Wn2 = rotor_natural_frequency * rotor_natural_frequency;
        rotor_plant_gain = rotor_plant_gain * Wn2;
        ao = ((2.0F/Tsample)*(2.0F/Tsample) + (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
                + rotor_natural_frequency*rotor_natural_frequency);
        c0 = ((2.0F/Tsample)*(2.0F/Tsample)/ao);
        c1 = -2.0F * c0;
        c2 = c0;
        c3 = -(2.0F*rotor_natural_frequency*rotor_natural_frequency - 2.0F*(2.0F/Tsample)*(2.0F/Tsample))/ao;
        c4 = -((2.0F/Tsample)*(2.0F/Tsample) - (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
                + rotor_natural_frequency*rotor_natural_frequency)/ao;
    }

    /* Transfer function model of form 1/(s^2 + Wn*s) */
    if (enable_rotor_plant_design == 2){
        IWon_r = 2 / (Wo_r * Tsample);
        iir_0_r = 1 - (1 / (1 + IWon_r));
        iir_1_r = -iir_0_r;
        iir_2_r = (1 / (1 + IWon_r)) * (1 - IWon_r);
    }

    // Optional Transfer function model of form Wn/(s^3 + Wn*s^2)
    if (enable_rotor_plant_design == 3 && enable_state_feedback == 0){
        IWon_r = 2 / (Wo_r * Tsample);
        iir_0_r = 1 / (1 + IWon_r);
        iir_1_r = iir_0_r;
        iir_2_r = iir_0_r * (1 - IWon_r);
    }
    
}

void InvertedPendulumGAM::control_logic_State_SwingingUp_Prepare() {
    /*
        * Apply controller parameters for initial operation at completion of
        * Swing Up
    */

    *current_error_steps = 0;
    *current_error_rotor_steps = 0;
    PID_Pend.state_a[0] = 0;
    PID_Pend.state_a[1] = 0;
    PID_Pend.state_a[2] = 0;
    PID_Pend.state_a[3] = 0;
    PID_Pend.int_term = 0;
    PID_Pend.control_output = 0;
    PID_Rotor.state_a[0] = 0;
    PID_Rotor.state_a[1] = 0;
    PID_Rotor.state_a[2] = 0;
    PID_Rotor.state_a[3] = 0;
    PID_Rotor.int_term = 0;
    PID_Rotor.control_output = 0;

    /* Initialize Pendulum PID control state */
    pid_filter_control_execute(&PID_Pend, current_error_steps, sample_period, Deriv_Filt_Pend);

    /* Initialize Rotor PID control state */
    *current_error_rotor_steps = 0;
    pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, sample_period_rotor, Deriv_Filt_Rotor);

    /* Initialize control system variables */

    cycle_count = CYCLE_LIMIT;
    i = 0;
    //rotor_position_steps = 0;
    rotor_position_steps_prev = 0;
    rotor_position_filter_steps = 0;
    rotor_position_filter_steps_prev = 0;
    rotor_position_command_steps = 0;
    rotor_position_diff = 0;
    rotor_position_diff_prev = 0;
    rotor_position_diff_filter = 0;
    rotor_position_diff_filter_prev = 0;
    rotor_position_step_polarity = 1;
    encoder_angle_slope_corr_steps = 0;
    rotor_sine_drive = 0;
    sine_drive_transition = 0;
    rotor_mod_control = 1.0;
    enable_adaptive_mode = 0;
    enable_cycle_delay_warning = ENABLE_CYCLE_DELAY_WARNING;
    chirp_cycle = 0;
    chirp_dwell_cycle = 0;
    pendulum_position_command_steps = 0;
    impulse_start_index = 0;
    mode_transition_state = 0;
    transition_to_adaptive_mode = 0;
    error_sum_prev = 0;
    error_sum_filter_prev = 0;
    adaptive_state = 4;
    rotor_control_target_steps_prev = 0;
    rotor_position_command_steps_prev = 0;
    rotor_position_command_steps_pf_prev = 0;
    enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
    slope_prev = 0;
    rotor_track_comb_command = 0;
    noise_rej_signal_prev = 0;
    noise_rej_signal_filter_prev = 0;
    full_sysid_start_index = -1;
    speed_scale = DATA_REPORT_SPEED_SCALE;
    speed_governor = 0;
    encoder_position_offset = 0;
    encoder_position_offset_zero = 0;

    for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++){
        offset_angle[m] = 0;
    }


    /*
        * Record user selected operation variable values.  Values will be
        * restored after Swing Up completion or after Angle Calibration
        * completion
        */

    /* Initial control state parameter storage */

    init_r_p_gain = PID_Rotor.Kp;
    init_r_i_gain = PID_Rotor.Ki;
    init_r_d_gain = PID_Rotor.Kd;
    init_p_p_gain = PID_Pend.Kp;
    init_p_i_gain = PID_Pend.Ki;
    init_p_d_gain = PID_Pend.Kd;
    init_enable_state_feedback = enable_state_feedback;
    init_integral_compensator_gain = integral_compensator_gain;
    init_feedforward_gain = feedforward_gain;
    //int init_enable_state_feedback = enable_state_feedback;
    init_enable_disturbance_rejection_step = enable_disturbance_rejection_step;
    init_enable_sensitivity_fnc_step = enable_sensitivity_fnc_step;
    init_enable_noise_rejection_step = enable_noise_rejection_step;
    init_enable_rotor_plant_design = enable_rotor_plant_design;
    init_enable_rotor_plant_gain_design = enable_rotor_plant_gain_design;

    if(select_suspended_mode == 1){
        load_disturbance_sensitivity_scale = 1.0;
    }
    if(select_suspended_mode == 0){
        load_disturbance_sensitivity_scale = LOAD_DISTURBANCE_SENSITIVITY_SCALE;
    }


    /*
        * Initiate Pendulum Swing Up with automatic system requiring no user action
        *
        * This system was developed by Markus Dauberschmidt see
        * https://github.com/OevreFlataeker/steval_edukit_swingup
        *
        */


    if (enable_swing_up == 1 && select_suspended_mode == 0){

        /*
            * Apply controller parameters for initial operation at completion of
            * Swing Up
            */

        PID_Rotor.Kp = 20;
        PID_Rotor.Ki = 10;
        PID_Rotor.Kd = 10;
        PID_Pend.Kp = 300;
        PID_Pend.Ki = 0.0;
        PID_Pend.Kd = 30.0;
        enable_state_feedback = 0;
        integral_compensator_gain = 0;
        feedforward_gain = 1;
        rotor_position_command_steps = 0;
        enable_state_feedback = 0;
        enable_disturbance_rejection_step = 0;
        enable_sensitivity_fnc_step = 0;
        enable_noise_rejection_step = 0;
        enable_rotor_plant_design = 0;
        enable_rotor_plant_gain_design = 0;

        /* Initialize position and motion variables */
        max_encoder_position = 0;
        global_max_encoder_position = 0;
        peaked = 0;
        handled_peak = 0;
        swing_up_state = 0;
        swing_up_state_prev = 0;
        zero_crossed = 0;
        stage_count = 0;
        /* Select initial amplitude for rotor impulse */
        impulse_amp = STAGE_0_AMP;
        // /* Initiate first swing */
        swing_up_direction = FORWARD;

    }

}

void InvertedPendulumGAM::control_logic_State_PendulumStablisation_Prepare(){
    encoder_position_init = 0;
    encoder_position_prev=-1;
    control_logic_State_PendulumStablisation_testCount = 0;
    control_logic_State_PendulumStablisation_isSecondRead = false;
}

void InvertedPendulumGAM::control_logic_State_Main_Prepare(){

}
bool InvertedPendulumGAM::control_logic_State_PendulumStablisation() {

    ret = encoder_position_read(&encoder_position_steps, encoder_position_init);

    /* Calibrate down angle */
    if( control_logic_State_PendulumStablisation_testCount == 2 ){
        control_logic_State_PendulumStablisation_testCount++;
         /*
            * Initialize Pendulum Angle Read offset by setting encoder_position_init
        */
        encoder_position_init = encoder_position_steps; 
        return false; 

    }else if( control_logic_State_PendulumStablisation_testCount == 3 ){
        encoder_position_down = encoder_position_steps;
        return true;
    }

    if( control_logic_State_PendulumStablisation_isSecondRead ){
        encoder_position_curr = encoder_position_steps;
        control_logic_State_PendulumStablisation_isSecondRead = false; 
    }else{
        encoder_position_prev = encoder_position_steps;
        control_logic_State_PendulumStablisation_isSecondRead = true;
    }

    if( (encoder_position_prev == encoder_position_curr)){  
        control_logic_State_PendulumStablisation_testCount++; 
    }
 

    return false;
}

bool InvertedPendulumGAM::control_logic_State_SwingingUp_checkSwingUp(){

    /* return TRUE if pendulum angle relative to vertical meets tolerance (for clockwise or counter clockwise approach */
    if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
        return true;//state change
    }
    if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
        encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
        return true;//state change
    }

//otherwise return FALSE
    return false;
}

bool InvertedPendulumGAM::control_logic_State_SwingingUp() {

    *OUTPUT_motor_ImpuleAmplitude= 0;
		
    ret = encoder_position_read(&encoder_position_steps, encoder_position_init);

    if( control_logic_State_SwingingUp_checkSwingUp() ) return true;

    if (zero_crossed)
    {//
        zero_crossed = false;
        // Push it aka put some more kinetic energy into the pendulum
        if (swing_up_state == 0){
            //BSP_MotorControl_Move(0, swing_up_direction, impulse_amp);
            //BSP_MotorControl_WaitWhileActive(0);
            *OUTPUT_motor_ImpuleAmplitude=impulse_amp;
            *OUTPUT_motor_Direction = swing_up_direction;
            stage_count++;

            if (prev_global_max_encoder_position != global_max_encoder_position && stage_count > 4){
                if (abs(global_max_encoder_position) < 600){
                    impulse_amp = STAGE_0_AMP;
                }
                if (abs(global_max_encoder_position) >= 600 && abs(global_max_encoder_position) < 1000){
                    impulse_amp = STAGE_1_AMP;
                }
                if (abs(global_max_encoder_position) >= 1000){
                    impulse_amp = STAGE_2_AMP;
                }
            }
            prev_global_max_encoder_position = global_max_encoder_position;
            global_max_encoder_position = 0;
            return false;
            //ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
        }
    }

    // We have a peak but did not handle it yet
    if (peaked && !handled_peak)
    {
        // Ensure we only enter this branch one per peak
        handled_peak = true;
        // Reset maximum encoder value to reassess after crossing the bottom
        max_encoder_position = 0;
        // Switch motor direction
        swing_up_direction = swing_up_direction == FORWARD ? BACKWARD : FORWARD;
    }

    return false;
}

bool InvertedPendulumGAM::control_logic_State_Main() {

   rotor_position_steps    = *INPUT_rotor_position_steps;
   *OUTPUT_break_Control_Loop = 0u;

    if (enable_swing_up == 1 && i == SWING_UP_CONTROL_CONFIG_DELAY && enable_angle_cal == 0){
        PID_Rotor.Kp = init_r_p_gain;
        PID_Rotor.Ki = init_r_i_gain;
        PID_Rotor.Kd = init_r_d_gain;
        PID_Pend.Kp = init_p_p_gain;
        PID_Pend.Ki = init_p_i_gain;
        PID_Pend.Kd = init_p_d_gain;
        enable_state_feedback = init_enable_state_feedback;
        integral_compensator_gain = init_integral_compensator_gain;
        feedforward_gain = init_feedforward_gain;
        enable_state_feedback = init_enable_state_feedback;
        enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
        enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
        enable_noise_rejection_step = init_enable_noise_rejection_step;
        enable_rotor_plant_design = init_enable_rotor_plant_design;
        enable_rotor_plant_gain_design = init_enable_rotor_plant_gain_design;
    }

    config_command = 0;
   
   
    /*
        * *************************************************************************************************
        *
        * Initiate Measurement and Control
        *
        * *************************************************************************************************
        */

    /*
        * Optional Reset and clear integrator error during initial start of controllers
        */
    if (i < 1){
        PID_Pend.int_term = 0;
        PID_Rotor.int_term = 0;
    }

    /*
        * Acquire encoder position and correct for initial angle value of encoder measured at
        * vertical down position at system start including 180 degree offset corresponding to
        * vertical upwards orientation.
        *
        * For case of Suspended Mode Operation the 180 degree offset is not applied
        *
        * The encoder_position_offset variable value is determined by the Automatic Inclination
        * Angle Calibration system
        */

    //##################TO REVISIT ##################################
    ret = encoder_position_read(&encoder_position_steps, encoder_position_init);
    if (select_suspended_mode == 0) {
        encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
        encoder_position = encoder_position - encoder_position_offset;
    }
    if (select_suspended_mode == 1) {
        encoder_position = encoder_position_steps - encoder_position_down;
        encoder_position = encoder_position - encoder_position_offset;
    }

    /*  Detect pendulum position excursion exceeding limits and exit */

    if(select_suspended_mode == 0){
        if (((encoder_position)/ ENCODER_READ_ANGLE_SCALE) > ENCODER_POSITION_POSITIVE_LIMIT) {
            //sprintf(msg_display, "Error Exit Encoder Position Exceeded: %i\r\n", encoder_position_steps);
            *OUTPUT_break_Control_Loop = 1u;
            return false;
        }
        if (((encoder_position)/ ENCODER_READ_ANGLE_SCALE) < ENCODER_POSITION_NEGATIVE_LIMIT) {
            //sprintf(msg_display, "Error Exit Encoder Position Exceeded: %i\r\n", encoder_position_steps);
            *OUTPUT_break_Control_Loop = 1u;
            return false;
        }
    }

    // /* Detect rotor position excursion exceeding limits and exit */

    if (rotor_position_steps > (ROTOR_POSITION_POSITIVE_LIMIT * STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
        // sprintf(msg_display, "Error Exit Motor Position Exceeded: %i\r\n", rotor_position_steps);
        *OUTPUT_break_Control_Loop = 1u;
        return false;
    }

    if (rotor_position_steps < (ROTOR_POSITION_NEGATIVE_LIMIT * STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
        // sprintf(msg_display, "Error Exit Motor Position Exceeded: %i\r\n", rotor_position_steps);
        *OUTPUT_break_Control_Loop = 1u;
        return false;
    }

    /*
        * Encoder Angle Error Compensation
        *
        * Compute Proportional control of pendulum angle compensating for error due to
        * encoder offset at start time or system platform slope relative to horizontal.
        *
        * Apply optional time limit to correct for encoder error or slope.  For cycle count
        * greater than ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT, encoder angle
        * slope correction remains constant.
        *
        * If ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT = 0, then encoder angle slope
        * correction continues operation for all time
        *
        * Note: This system is *not* required in the event that Automatic Inclination
        * Angle Calibration is selected.
        *
        */

    /* Compute Low Pass Filtered rotor position difference */

    rotor_position_diff_prev = rotor_position_diff;

    if (enable_disturbance_rejection_step == 0){
        rotor_position_diff = rotor_position_filter_steps
                - rotor_position_command_steps;
    }
    if (enable_disturbance_rejection_step == 1){
        rotor_position_diff = rotor_position_filter_steps;
    }



    /* Apply slope correction */
    if (ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION == 1 && i > angle_cal_complete) {
        rotor_position_diff_filter =
                (float) (rotor_position_diff * iir_LT_0)
                + rotor_position_diff_prev * iir_LT_1
                - rotor_position_diff_filter_prev * iir_LT_2;
        if ((i < ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT) || (ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT == 0)) {
            encoder_angle_slope_corr_steps = rotor_position_diff_filter / ENCODER_ANGLE_SLOPE_CORRECTION_SCALE;
        }
        rotor_position_diff_filter_prev = rotor_position_diff_filter;
    }


    /*
        *  Compute current_error_steps input for Primary Controller
        *
        *  current_error_steps is sum of encoder angle error compensation and encoder position
        *  in terms of stepper motor steps
        *
        *  An Encoder offset may be introduced.  The Encoder offset may remain at all times if
        *  ENCODER_OFFSET_DELAY == 0 or terminate at a time (in ticks) of ENCODER_OFFSET_DELAY
        */

    if ((i > ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)){
        encoder_position = encoder_position - ENCODER_START_OFFSET;
    }

    /*
        * Compute error between Pendulum Angle and Pendulum Tracking Angle in units of steps
        * Apply scale factor to match angle to step gain of rotor actuator
        *
        */

    *current_error_steps = encoder_angle_slope_corr_steps
    + ENCODER_ANGLE_POLARITY * (encoder_position / ((float)(ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

    /*
        *
        * Pendulum Controller execution
        *
        * Include addition of Pendulum Angle track signal impulse signal
        * Compute control signal, rotor position target in step units
        *
        * Pendulum tracking command, pendulum_position_command, also supplied in step units
        *
        */

    *current_error_steps = *current_error_steps + pendulum_position_command_steps;

    pid_filter_control_execute(&PID_Pend,current_error_steps, sample_period, Deriv_Filt_Pend);

    rotor_control_target_steps = PID_Pend.control_output;

    /* Acquire rotor position and compute low pass filtered rotor position */


   /* Optional rotor position filter */
    rotor_position_filter_steps = (float) (rotor_position_steps) * iir_0 + rotor_position_steps_prev * iir_1
            - rotor_position_filter_steps_prev * iir_2;
    rotor_position_steps_prev = (float) (rotor_position_steps);
    rotor_position_filter_steps_prev = rotor_position_filter_steps;


    rotor_position_filter_steps = rotor_position_steps;

    /*
        * 		Compute rotor chirp tracking signal with chirp sweep from rotor_chirp_start_freq
        * 		to rotor_chirp_end_freq in time period rotor_chirp_period in units of control
        * 		loop cycle periods.  Each chirp is separated by delay of ROTOR_CHIRP_SWEEP_DELAY.
        */

    if (enable_rotor_chirp == 1 && enable_mod_sin_rotor_tracking == 0
            && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {

        if (i < ROTOR_CHIRP_PERIOD - 1){
            chirp_cycle = 0;
        }
        if (chirp_cycle > ROTOR_CHIRP_PERIOD - 1) {
            chirp_cycle = 0;
            chirp_dwell_cycle = ROTOR_CHIRP_SWEEP_DELAY;
        }
        if (chirp_dwell_cycle > 0){
            chirp_dwell_cycle--;
            chirp_cycle = 0;
        }
        if (chirp_dwell_cycle == 0 && i >= ROTOR_CHIRP_PERIOD - 1){
            chirp_cycle = chirp_cycle + 1;
            chirp_time = (float)((chirp_cycle - 1)/ROTOR_CHIRP_SAMPLE_RATE);
            rotor_chirp_frequency = rotor_chirp_start_freq + (rotor_chirp_end_freq - rotor_chirp_start_freq)*((float)(chirp_cycle/rotor_chirp_period));
            rotor_position_command_steps = ((float)(ROTOR_CHIRP_STEP_AMPLITUDE*STEPPER_READ_POSITION_STEPS_PER_DEGREE))*sin(2.0*3.14159*rotor_chirp_frequency*chirp_time);
        }
    }


    /*  Create rotor track "comb" signal */
    if (enable_rotor_tracking_comb_signal > 0 && i > 1000 && i > angle_cal_complete) {

        chirp_time = ((float)(i - 1))/500.0;
        rotor_track_comb_signal_frequency = 0.01;
        rotor_track_comb_command = ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.017783;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.031623;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.056234;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.1;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.17783;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.31623;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.56234;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 1.0;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 1.7783;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 3.1623;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 5.6234;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 10;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
    }

    if (enable_rotor_chirp == 0 && enable_mod_sin_rotor_tracking == 0
            && enable_rotor_tracking_comb_signal == 1) {
        rotor_position_command_steps = rotor_track_comb_command;
    }

    /*  Create rotor angle reference tracking modulated sine signal */

    rotor_sine_drive = 0;
    if (enable_mod_sin_rotor_tracking == 1 && ENABLE_ROTOR_CHIRP == 0 && i > angle_cal_complete) {

        if (ENABLE_ROTOR_CHIRP == 0){
            mod_sin_carrier_frequency = MOD_SIN_CARRIER_FREQ;
        }

        if (i > MOD_SIN_START_CYCLES && enable_mod_sin_rotor_tracking == 1) {
            rotor_sine_drive =
                    (float) (mod_sin_amplitude
                            * (1 + sin(-1.5707 + ((i - MOD_SIN_START_CYCLES)/MOD_SIN_SAMPLE_RATE) * (MOD_SIN_MODULATION_FREQ * 6.2832))));
            rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
            rotor_sine_drive = rotor_sine_drive + MOD_SIN_MODULATION_MIN;
            rotor_sine_drive = rotor_sine_drive * rotor_sine_drive_mod * rotor_mod_control;
        }

        if (i > MOD_SIN_START_CYCLES && ENABLE_SIN_MOD == 0) {
            rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
            rotor_sine_drive = rotor_control_sin_amplitude * rotor_sine_drive_mod * rotor_mod_control;
        }

        if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 1 && sine_drive_transition == 1){
            rotor_mod_control = 0.0;
            sine_drive_transition = 0;
        }
        if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 0 && sine_drive_transition == 1){
            rotor_mod_control = 1.0;
            sine_drive_transition = 0;
        }

        if (enable_rotor_position_step_response_cycle == 0){
            rotor_position_command_steps = rotor_sine_drive;
        }

    }

    /*  Create rotor angle reference tracking impulse signal */

    if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0 && i > angle_cal_complete) {
        if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
            rotor_position_command_steps =
                    (float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
                            * STEPPER_READ_POSITION_STEPS_PER_DEGREE);
            impulse_start_index = 0;
        }
        if (impulse_start_index
                > ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
            rotor_position_command_steps = 0;
        }
        impulse_start_index++;
    }

    /*
        * Create pendulum angle reference tracking impulse signal.  Polarity of impulse alternates
        */

    if (enable_pendulum_position_impulse_response_cycle == 1 && i != 0 && i > angle_cal_complete) {

        if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
            if (select_suspended_mode == 1) {
                pendulum_position_command_steps =
                        (float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;
            }
            if (select_suspended_mode == 0) {
                pendulum_position_command_steps =
                        (float) (PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
                                /PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE);
            }
            chirp_cycle = 0;
            impulse_start_index = 0;
        }
        if (impulse_start_index
                > PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
            pendulum_position_command_steps = 0;
        }
        impulse_start_index++;
        chirp_cycle++;
    }

    /*  Create rotor angle reference tracking  step signal */

    if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0 && enable_rotor_position_step_response_cycle == 1 && i > angle_cal_complete) {
        rotor_position_step_polarity = -rotor_position_step_polarity;
        if (rotor_position_step_polarity == 1){
            chirp_cycle = 0;
        }
    }

    if (enable_rotor_position_step_response_cycle == 1 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {
        if (STEP_RESPONSE_AMP_LIMIT_ENABLE == 1 && abs(rotor_sine_drive) > STEP_RESPONSE_AMP_LIMIT){
            chirp_cycle = chirp_cycle + 1;
        } else {
            if (enable_mod_sin_rotor_tracking == 1){
                rotor_position_command_steps = rotor_sine_drive + (float) ((rotor_position_step_polarity)
                        * ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
                        * STEPPER_READ_POSITION_STEPS_PER_DEGREE);
            }
            if (enable_mod_sin_rotor_tracking == 0){
                rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
                        * ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
                        * STEPPER_READ_POSITION_STEPS_PER_DEGREE);
            }
            chirp_cycle = chirp_cycle + 1;
        }
    }

    /*
        * Rotor tracking reference, rotor_position_command_steps, is low pass filtered to prevent
        * aliasing of measurement during operation of Real Time Workbench sampling that occurs at
        * 50 Hz (in support of connected computing platform bandwidth limitations).  This filter
        * application is not applied during selection of high speed sampling at 500 Hz.
        */

    if (enable_rotor_position_step_response_cycle == 1 && enable_mod_sin_rotor_tracking == 0 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete){
        rotor_position_command_steps = rotor_position_command_steps_pf * iir_0_s
                + rotor_position_command_steps_pf_prev * iir_1_s
                - rotor_position_command_steps_prev * iir_2_s;
        rotor_position_command_steps_pf_prev = rotor_position_command_steps_pf;
    }

    /*
        *  Automatic Inclination Angle Calibration System
        *
        *  The Edukit system may be resting on a surface with a slight incline. This then
        *  produces a Rotor Angle dependent error between the measurement of Pendulum Angle
        *  and the angle corresponding to true vertical of the gravitational vector.
        *
        *  This system computed true vertical angle relative to the gravity vector for each
        *  Rotor Step.  This provides an encoder_offset_angle calibration value for all
        *  orientations of the Rotor.
        */

    if (enable_angle_cal == 1){
        /*
            * Angle Calibration system state values applied during Angle Calibration
            * Period.  User selected system state values restored after Angle Calibration
            */

        if (i == 1 && select_suspended_mode == 0){
            PID_Rotor.Kp = 21.1;
            PID_Rotor.Ki = 0;
            PID_Rotor.Kd = 17.2;
            PID_Pend.Kp = 419;
            PID_Pend.Ki = 0.0;
            PID_Pend.Kd = 56;
            enable_state_feedback = 1;
            integral_compensator_gain = 10;
            feedforward_gain = 1;
            rotor_position_command_steps = 0;
            current_error_rotor_integral = 0;
        }

        if (i == 1 && select_suspended_mode == 1){
            PID_Rotor.Kp = -23.86;
            PID_Rotor.Ki = 0;
            PID_Rotor.Kd = -19.2;
            PID_Pend.Kp = -293.2;
            PID_Pend.Ki = 0.0;
            PID_Pend.Kd = -41.4;
            enable_state_feedback = 1;
            integral_compensator_gain = -11.45;
            feedforward_gain = 1;
            rotor_position_command_steps = 0;
            current_error_rotor_integral = 0;
        }


        /* Initialize angle calibration variables */

        if (i == 1){
            offset_end_state = 0;
            offset_start_index = 4000;					// initial start index for sweep
            angle_index = ANGLE_CAL_OFFSET_STEP_COUNT;	// Number of angle steps
            angle_cal_end = INT_MAX;
            angle_cal_complete = INT_MAX;				// Allowed start time for stimulus signals
            encoder_position_offset_zero = 0;
        }

        if (offset_end_state == 0){
            /* Suspend loop delay warning since computation may lead to control loop cycle delay during
                * period after measurement and during computation of smoothed offset data array
                */
            enable_cycle_delay_warning = 0;
            /* Advance to upper angle of 90 degrees*/
            if (i > 1 && i < 4000){
                rotor_position_command_steps = (i/4000.0) * ANGLE_CAL_OFFSET_STEP_COUNT/2;
                offset_start_index = i + 4000;
            }
            /* Delay for time increment to avoid transient response in measurement.
                * Acquire samples for time-average of offset
                */
            if (i >= offset_start_index && i < (offset_start_index + 10)){
                //offset_angle[angle_index] = offset_angle[angle_index] + encoder_position;
                //offset_angle[angle_index] = encoder_position;
            }
            /* Compute time-averages offset and advance to next lower angle increment */
            if (i == offset_start_index + 10 && angle_index > 0){
                offset_angle[angle_index] = encoder_position;
                //offset_angle[angle_index] = offset_angle[angle_index]/10;
                angle_index = angle_index - 1;
                offset_start_index = offset_start_index + 10;
                rotor_position_command_steps = rotor_position_command_steps - 1;
            }

            /* Compute average encoder position offset over angle index range from ANGLE_AVG_SPAN to ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN */
            /* Suspend delay warning */

            if (angle_index >= 2*ANGLE_AVG_SPAN && angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1){
                for (angle_avg_index = angle_index - 2*ANGLE_AVG_SPAN; angle_avg_index < (angle_index + 1); angle_avg_index++){
                    encoder_position_offset_avg[angle_index] = 0;
                            for (angle_avg_index = angle_index - ANGLE_AVG_SPAN; angle_avg_index < (1 + angle_index + ANGLE_AVG_SPAN); angle_avg_index++){
                                    encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + offset_angle[angle_avg_index];
                            }
                            encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index]/(float)(2*ANGLE_AVG_SPAN + 1);
                }
            }

            /* Restore rotor angle to zero degrees */
            if (angle_index == 0){
                rotor_position_command_steps = rotor_position_command_steps + 0.02*STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            }
            /* Terminate offset measurement and initialize angle_cal_end at time of termination */
            if (rotor_position_command_steps >= 0 && angle_index == 0){
                offset_end_state = 1;
                angle_cal_end = i;
                /* Restore loop delay warning */
                enable_cycle_delay_warning = 1;
            }
        }
    }

    /* Apply offset angle for correction of pendulum angle according to rotor position */

    if (offset_end_state == 1 && i > angle_cal_end){
        /* Compute angle index corresponding to rotor position */
        angle_index = (int)((ANGLE_CAL_OFFSET_STEP_COUNT - 1)/2) + rotor_position_filter_steps;
        if (angle_index < ANGLE_AVG_SPAN ){
            angle_index = ANGLE_AVG_SPAN;
        }
        if (angle_index >  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN ){
            angle_index =  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN;
        }
        encoder_position_offset = 2.0 * encoder_position_offset_avg[angle_index];
    }

    /* Measure residual offset at zero rotor position */
    if (offset_end_state == 1 && i > angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING && i < angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL){
        encoder_position_offset_zero = encoder_position_offset_zero + encoder_position;
    }

    /* Correct offset angle array values for any residual offset */
    if (i == (angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1)){
        encoder_position_offset_zero = encoder_position_offset_zero/ANGLE_CAL_ZERO_OFFSET_DWELL;
        for (angle_index = 0; angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1; angle_index++){
            encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + encoder_position_offset_zero;
        }
        angle_cal_complete = angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1 + ANGLE_CAL_COMPLETION;
    }

    /* Restore user selected system state configuration */
    if (offset_end_state == 1 && (enable_angle_cal == 1) && i == angle_cal_complete + 1){
        PID_Rotor.Kp = init_r_p_gain;
        PID_Rotor.Ki = init_r_i_gain;
        PID_Rotor.Kd = init_r_d_gain;
        PID_Pend.Kp = init_p_p_gain;
        PID_Pend.Ki = init_p_i_gain;
        PID_Pend.Kd = init_p_d_gain;
        current_error_rotor_integral = 0;
        enable_state_feedback = init_enable_state_feedback;
        integral_compensator_gain = init_integral_compensator_gain;
        feedforward_gain = init_feedforward_gain;
        enable_state_feedback = init_enable_state_feedback;
        enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
        enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
        enable_noise_rejection_step = init_enable_noise_rejection_step;
        enable_rotor_plant_design = init_enable_rotor_plant_design;
    }


    if (ENABLE_DUAL_PID == 1) {

        /*
            * Secondary Controller execution including Sensitivity Function computation
            */

        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
            *current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
        }
        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 1 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
            *current_error_rotor_steps = rotor_position_filter_steps;
        }
        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 1){
            *current_error_rotor_steps = rotor_position_filter_steps + rotor_position_command_steps;
        }

        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 1 && enable_noise_rejection_step == 0){
            *current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
        }

        /*
            * Select Reference signal input location at input of controller for Dual PID architecture
            * for Output Feedback Architecture or at output of controller and plant input for Full State
            * Feedback Architecture
            */

        if (enable_state_feedback == 1){
            *current_error_rotor_steps = rotor_position_filter_steps;
        }

        /*
            * PID input supplied in units of stepper motor steps with tracking error determined
            * as difference between rotor position tracking command and rotor position in units
            * of stepper motor steps.
            */

        pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps,
                sample_period_rotor,  Deriv_Filt_Rotor);

        rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;


        if (enable_state_feedback == 1 && integral_compensator_gain != 0){
            /*
                * If integral action is included, state feedback plant input equals
                * the time integral of difference between reference tracking signal and rotor angle,
                * current_error_rotor_integral.  This is summed with the controller output, rotor_control_target_steps.
                * The integral_compensator_gain as input by user includes multiplicative scale factor matching
                * scaling of controller gain values.
                */
            current_error_rotor_integral = current_error_rotor_integral + (rotor_position_command_steps*feedforward_gain - rotor_position_filter_steps)*(*sample_period_rotor);
            rotor_control_target_steps = rotor_control_target_steps - integral_compensator_gain*current_error_rotor_integral;
        }

        if (enable_state_feedback == 1 && integral_compensator_gain == 0){
            /*
                * If integral compensator is not included, full state feedback plant input equals difference
                * between control output and reference tracking signal with scale factor matching
                * scaling of controller gain values.
                *
                * If Plant Design system is applied, the effects of small numerical error in transfer function
                * computation is compensated for by removal of average offset error.
                *
                */

            rotor_control_target_steps = rotor_control_target_steps - rotor_position_command_steps*feedforward_gain;

        }


        /*
            * Load Disturbance Sensitivity Function signal introduction with scale factor applied to increase
            * amplitude of Load Disturbance signal to enhance signal to noise in measurement.  This scale factor
            * then must be applied after data acquisition to compute proper Load Disturbance Sensitivity Function.
            * Note that Load Disturbance Sensitivity Function value is typically less than -20 dB
            *
            */
        if (enable_disturbance_rejection_step == 1){
            rotor_control_target_steps = rotor_control_target_steps + rotor_position_command_steps * load_disturbance_sensitivity_scale;
        }
    }


    if (full_sysid_start_index != -1 && i >= full_sysid_start_index && i > angle_cal_complete) {
        float total_acc = 0;
        float t = (i - full_sysid_start_index) * Tsample;
        float w = full_sysid_min_freq_hz * M_TWOPI;
        for (int k_step = 0; k_step < full_sysid_num_freqs; k_step++) {
            float wave_value = w * cosf(w * t); // multiply acceleration wave by omega to keep consistent velocity amplitude
            total_acc += wave_value;
            w *= full_sysid_freq_log_step;
        }
        rotor_control_target_steps = ((full_sysid_max_vel_amplitude_deg_per_s/full_sysid_num_freqs) * total_acc * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE);
    }


    /*
        *
        * Plant transfer function design based on two stage first order high pass IIR
        * filter structures applied to rotor_control_target_steps.
        *
        * Second order system computed at all cycle times to avoid transient upon switching between
        * operating modes with and without Rotor Plant Design enabled
        *
        */


    if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){

            rotor_control_target_steps_filter_2 = c0* rotor_control_target_steps + c1* rotor_control_target_steps_prev
                    + c2*rotor_control_target_steps_prev_prev + c3*rotor_control_target_steps_filter_prev_2
                    + c4*rotor_control_target_steps_filter_prev_prev_2;

            rotor_control_target_steps_prev_prev = rotor_control_target_steps_prev;
            rotor_control_target_steps_filter_prev_prev_2 = rotor_control_target_steps_filter_prev_2;
            rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
    }

    if ((enable_rotor_plant_design == 2 )){
        rotor_control_target_steps_filter_2 = iir_0_r* rotor_control_target_steps + iir_1_r*rotor_control_target_steps_prev
                - iir_2_r*rotor_control_target_steps_filter_prev_2;
        rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
    }


    /*
        * Record current value of rotor_position_command tracking signal
        * and control signal, rotor_control_target_steps for rotor position
        * rotor position filters, rotor plant design, performance monitoring and adaptive control
        */

    rotor_control_target_steps_prev = rotor_control_target_steps;
    rotor_position_command_steps_prev = rotor_position_command_steps;

    //##################TO REVISIT HARWARE CALL TO TAKE ACTION##################################
    if (ACCEL_CONTROL == 1) {
        if (enable_rotor_plant_design != 0){
            rotor_control_target_steps_filter_2 = rotor_plant_gain*rotor_control_target_steps_filter_2;
            *OUTPUT_motor_Acceleration = rotor_control_target_steps_filter_2;
            //apply_acceleration(&rotor_control_target_steps_filter_2, &target_velocity_prescaled, Tsample);
        /* Applies if Rotor Gain defined */
        } else if (enable_rotor_plant_gain_design == 1){
            rotor_control_target_steps_gain = rotor_plant_gain * rotor_control_target_steps;
            *OUTPUT_motor_Acceleration = rotor_control_target_steps_gain;
            //apply_acceleration(&rotor_control_target_steps_gain, &target_velocity_prescaled, Tsample);
        /* Applies if no Rotor Design is selected */
        } else {
            *OUTPUT_motor_Acceleration = rotor_control_target_steps;
            //apply_acceleration(&rotor_control_target_steps, &target_velocity_prescaled, Tsample);
        }
    } else {
        *OUTPUT_motor_ImpuleAmplitude = rotor_control_target_steps/2;
        //BSP_MotorControl_GoTo(0, rotor_control_target_steps/2);
    }

    /*
        * *************************************************************************************************
        *
        * Data Report Sequence Start
        *
        * *************************************************************************************************
        */


    if (enable_pendulum_position_impulse_response_cycle == 1) {
        reference_tracking_command = pendulum_position_command_steps;
    } else {
        reference_tracking_command = rotor_position_command_steps;
    }

    if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && ACCEL_CONTROL_DATA == 1){
        if (enable_pendulum_position_impulse_response_cycle == 1) {
            reference_tracking_command = pendulum_position_command_steps;
        } else {
            reference_tracking_command = rotor_position_command_steps;
        }
    }

    /* Select display parameter corresponding to requested selection of Sensitivity Functions */
    if (enable_disturbance_rejection_step == 1) { display_parameter = rotor_position_steps/load_disturbance_sensitivity_scale; }
    else if (enable_noise_rejection_step == 1) { noise_rej_signal = rotor_control_target_steps; }
    else if (enable_sensitivity_fnc_step == 1)  { display_parameter = rotor_position_command_steps - rotor_position_steps; }
    else { display_parameter = rotor_position_steps; }


    if (enable_noise_rejection_step == 1){
        display_parameter = noise_rej_signal;
    }



    /* Increment cycle counter */

    i++;

    return true;
		
}


bool InvertedPendulumGAM::Initialise(MARTe::StructuredDataI & data) {
    bool ok = GAM::Initialise(data);

    control_logic_Initialise();
    
    return ok;
}

bool InvertedPendulumGAM::Setup() {
    StreamString gam_name;
    
    bool ok = GetQualifiedName(gam_name);
    if (!ok) {
        REPORT_ERROR(ErrorManagement::ParametersError, "Cannot get the qualified name");
    }

    if (ok) {
        uint32 nOfInputSignals = GetNumberOfInputSignals();
        ok = (nOfInputSignals == 2u); // Will need to be changed if any input signals are added or removed
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of input signals must be 2", gam_name.Buffer());
        }
    } 
    uint32 signalIdx;
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "rotor_position_steps", InputSignals, SignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            INPUT_rotor_position_steps = (int32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal rotor_position_steps ");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "encoder_counter", InputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            INPUT_encoder_counter = (uint32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal encoder_counter ");
        }
    }

    if (ok) {
        uint32 nOfOutputSignals = GetNumberOfOutputSignals();
        ok = (nOfOutputSignals == 6u); // Will need to be changed if any output signals are added or removed
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of output signals must be 6", gam_name.Buffer());
        }
    }
    
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "state", OutputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_state = (uint8*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal State");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "motor_ImpuleAmplitude", OutputSignals, SignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_motor_ImpuleAmplitude = (int32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal motor_ImpuleAmplitude");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "motor_Direction", OutputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_motor_Direction = (uint8*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal motor_Direction");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "motor_Acceleration", OutputSignals, SignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_motor_Acceleration = (int32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal motor_Acceleration");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "break_Control_Loop", OutputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_break_Control_Loop = (uint8*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal break_Control_Loop");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "encoder_position", OutputSignals, Float32Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_encoder_position = (float32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal encoder_position");
        }
    }


    return ok;
}


/*
 * PID Controller with low pass filter operating on derivative component
 */

void InvertedPendulumGAM::pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error, float * sample_period, float * Deriv_Filt) {

	float int_term, diff, diff_filt;

	  /* Compute time integral of error by trapezoidal rule */
	  int_term = PID->Ki*(*sample_period)*((*current_error) + PID->state_a[0])/2;

	  /* Compute time derivative of error */
	  diff = PID->Kd*((*current_error) - PID->state_a[0])/(*sample_period);

	  /* Compute first order low pass filter of time derivative */
	  diff_filt = Deriv_Filt[0] * diff
				+ Deriv_Filt[0] * PID->state_a[2]
				- Deriv_Filt[1] * PID->state_a[3];

	  /* Accumulate PID output with Integral, Derivative and Proportional contributions*/

	  PID->control_output = diff_filt + int_term + PID->Kp*(*current_error);

	  /* Update state variables */
	  PID->state_a[1] = PID->state_a[0];
	  PID->state_a[0] = *current_error;
	  PID->state_a[2] = diff;
	  PID->state_a[3] = diff_filt;
	  PID->int_term = int_term;
}


/*
 * Configure system based on user selection
 */

void InvertedPendulumGAM::user_configuration(void){

	enable_rotor_actuator_test = 0;
	enable_rotor_actuator_control = 0;
	enable_encoder_test = 0;
	enable_rotor_actuator_high_speed_test = 0;
	enable_motor_actuator_characterization_mode = 0;
	enable_full_sysid = 0;

	enable_rotor_tracking_comb_signal = 0;
	rotor_track_comb_amplitude = 0;
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;

//############## Jawad Modification  -->> ###############################
	enable_state_feedback = 0;
	select_suspended_mode = 0;
	proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
	integral = 			PRIMARY_INTEGRAL_MODE_1;
	derivative = 		PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
	max_speed =(u_int16_t)MAX_SPEED_MODE_1;
	min_speed =(u_int16_t)MIN_SPEED_MODE_1;
	enable_rotor_plant_design = 0;
	enable_rotor_plant_gain_design = 0;
	enable_rotor_position_step_response_cycle = 0;
	enable_pendulum_position_impulse_response_cycle = 0;
	enable_rotor_chirp = 0;
	enable_mod_sin_rotor_tracking = 1;
	enable_angle_cal = 1;
	enable_swing_up = 1;
	
}

void InvertedPendulumGAM::restart_execution(){
    state = STATE_INITIALIZATION;
}

bool InvertedPendulumGAM::Execute() {

   *OUTPUT_motor_ImpuleAmplitude=0;
   *OUTPUT_motor_Direction = UNKNOW_DIR;
   *OUTPUT_motor_Acceleration = 0;
   *OUTPUT_break_Control_Loop = 0;

    bool  ret = true;
 
    if( state == STATE_INITIALIZATION){
        control_logic_State_Initialization();
        state = STATE_PENDULUM_STABLIZATION;
        control_logic_State_PendulumStablisation_Prepare();
    }
    if( state == STATE_PENDULUM_STABLIZATION){
        ret = control_logic_State_PendulumStablisation();
        if( ret ){
            state = STATE_SWING_UP;
            control_logic_State_SwingingUp_Prepare();
        }
    }
    if( state == STATE_SWING_UP){ //initialisation state
        ret = control_logic_State_SwingingUp();
        if( ret ){//state change
            state = STATE_MAIN;
            control_logic_State_Main_Prepare();
        }
    }
    if( state == STATE_MAIN ) {   // main state 
        ret = control_logic_State_Main();
        if( !ret ){//state change
            restart_execution();
        }
    }

    
    *OUTPUT_state = state;
    *OUTPUT_encoder_position = encoder_position_steps;        
    return true;
}

bool oppositeSigns(int x, int y) {
	return ((x ^ y) < 0);
}

int InvertedPendulumGAM::encoder_position_read(int *encoder_position, int encoder_position_init) {
    
//************* This is hardware call, as such, it has been abstracted to the GAM input
	//cnt3 = __HAL_TIM_GET_COUNTER(htim3);

    uint32 cnt3 = *INPUT_encoder_counter;

	if (cnt3 >= 32768u) {
		*encoder_position = (int) (cnt3);
		*encoder_position = *encoder_position - 65536;
	} else {
		*encoder_position = (int) (cnt3);
	}

	range_error = 0;
	if (*encoder_position <= -32768) {
		range_error = -1;
		*encoder_position = -32768;
	}
	if (*encoder_position >= 32767) {
		range_error = 1;
		*encoder_position = 32767;
	}

	*encoder_position = *encoder_position - encoder_position_init;

	/*
	 *  Detect if we passed the bottom, then re-arm peak flag
	 *  oppositeSigns returns true when we pass the bottom position
	 */


	if (oppositeSigns(*encoder_position, previous_encoder_position))
	{
		peaked = false;
		zero_crossed = true;
	}

	if (!peaked) // We don't need to evaluate anymore if we hit a maximum when we're still in downward motion and didn't cross the minimum
	{
		// Add global maximum
		if (abs(*encoder_position) >= abs(global_max_encoder_position))
		{
			global_max_encoder_position = *encoder_position;
		}
		// Check if new maximum
		if (abs(*encoder_position) >= abs(max_encoder_position))
		{
			max_encoder_position = *encoder_position;
		}
		else
		{
			// We are at the peak and disable further checks until we traversed the minimum position again
			peaked = true;
		    handled_peak = false;
		}
	}

	previous_encoder_position = *encoder_position;


	return range_error;
}


CLASS_REGISTER(InvertedPendulumGAM, "1.0");

} // namespace MFI

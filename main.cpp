#include <mbed.h>

#include "PM2_Drivers.h"

# define M_PI 3.14159265358979323846 // number pi, an example in case you need it


bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and shows how you can run a code segment only once
bool rest = false;
// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(PC_13);  // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
void user_button_pressed_fcn(); // custom functions which get executed when user button gets pressed, definition below


// main runs as an own thread
int main()
{
    // states and actual state for state machine
    const int ROBOT_STATE_INIT  = 0;
    const int ROBOT_STATE_FORWARD  = 1;
    const int ROBOT_STATE_CLIMB    = 2;
    const int ROBOT_STATE_SLEEP    = 3;
    int robot_state_actual = ROBOT_STATE_INIT;


    // attach button fall function to user button object, button has a pull-up resistor
    user_button.fall(&user_button_pressed_fcn);

    // while loop gets executed every main_task_period_ms milliseconds (simple aproach to repeatedly execute main)
    const int main_task_period_ms = 50; // define main task period time in ms e.g. 50 ms -> main task runs 20 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);       // create DigitalOut object to command user led
    // additional led
    DigitalOut additional_led(PB_9); // create DigitalOut object to command extra led (you need to add an aditional resistor, e.g. 220...500 Ohm)
    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate extra mechanical button, you need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential

    // Sharp GP2Y0A41SK0F, 4-40 cm IR Sensor
    float ir_distance_mV = 0.0f; // define variable to store measurement
    AnalogIn ir_analog_in(PC_2); // create AnalogIn object to read in infrared distance sensor, 0...3.3V are mapped to 0...1

    // 78:1, 100:1, ... Metal Gearmotor 20Dx44L mm 12V CB
    DigitalOut enable_motors(PB_15); // create DigitalOut object to enable dc motors

    FastPWM pwm_M1(PB_13); // motor M1 is used open-loop
    FastPWM pwm_M2(PA_9);  // motor M2 is closed-loop speed controlled (angle velocity)
    EncoderCounter  encoder_M1(PA_6, PC_7); // create encoder objects to read in the encoder counter values, since M1 is used open-loop no encoder would be needed for operation, this is just an example
    EncoderCounter  encoder_M2(PB_6, PB_7);

    // create SpeedController and PositionController objects, default parametrization is for 78.125:1 gear box
    const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
    const float counts_per_turn = 20.0f * 100.0f; // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn = 140.0f / 12.0f;               // define motor constant in RPM/V
    const float k_gear = 100.0f / 1.0f;         // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float kp = 0.1f;                         // define custom kp, this is the default speed controller gain for gear box 78.125:1

    const float counts_per_turn_M2 = 20.0f * 488.28125f; // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn_M2 = 28.0f / 12.0f;               // define motor constant in RPM/V
    const float k_gear_M2 = 488.28125f / 1.0f;         // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float kp_M2 = 0.1f;                         // define custom kp, this is the default speed controller gain for gear box 78.125:1

    SpeedController speedController_M1(counts_per_turn, kn, max_voltage, pwm_M1, encoder_M1); 
    
    SpeedController speedController_M2(counts_per_turn_M2, kn_M2, max_voltage, pwm_M2, encoder_M2); // default 78.125:1 gear box  with default contoller parameters
    // SpeedController speedController_M2(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_M2, encoder_M2); // parameters adjusted to 100:1 gear
    // PositionController positionController_M3(counts_per_turn, kn, max_voltage, pwm_M3, encoder_M3); // default 78.125:1 gear with default contoller parameters
    PositionController positionController_M1(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_M1, encoder_M1); // parameters adjusted to 100:1 gear, we need a different speed controller gain here
    PositionController positionController_M2(counts_per_turn_M2 * k_gear_M2, kn_M2 / k_gear_M2, max_voltage, pwm_M2, encoder_M2); // parameters adjusted to 100:1 gear, we need a different speed controller gain here
    
    main_task_timer.start();
    
    // this loop will run forever
    while (true) {

        main_task_timer.reset();

        if (do_execute_main_task) {

            // read analog input
            if (mechanical_button.read()) {
                ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;
                
            }
            // visual feedback that the main task is executed, setting this once would actually be enough
            additional_led = 1;

            // state machine
            switch (robot_state_actual) {
                case ROBOT_STATE_INIT:
                    enable_motors = 1;
                    if(positionController_M2.getRotation() != 0.00000f){
                        positionController_M2.setDesiredRotation(0.0f);
                        positionController_M2.setPositionCntrlGain(0.0f);
                    }
                    robot_state_actual= ROBOT_STATE_FORWARD;
                break;
                case ROBOT_STATE_FORWARD:
                  
                    positionController_M1.setDesiredRotation(-0.25f);
                    positionController_M1.setMaxAccelerationRPM(0.5f);
                    positionController_M1.setMaxVelocityRPM(0.5f);
                

                    if(ir_distance_mV >= 2500.0f && ir_distance_mV < 3000.0f){
                        robot_state_actual= ROBOT_STATE_CLIMB;
                    }
                    break;
                case ROBOT_STATE_CLIMB:    
                    if((positionController_M2.getRotation()*1000) == 0.0f){
                        positionController_M2.setDesiredRotation(0.001005f);
                    }
                    positionController_M2.setMaxVelocityRPM(0.01f);
                    positionController_M2.setMaxAccelerationRPM(0.01f);
                    if((positionController_M2.getRotation()*1000) == 0.001003f*1000.0f){
                        robot_state_actual= ROBOT_STATE_SLEEP;
                    }
                    break;
                case ROBOT_STATE_SLEEP:
                    positionController_M2.setDesiredRotation(-0.00005f);
                   positionController_M2.setMaxVelocityRPM(0.01f);
                    positionController_M2.setMaxAccelerationRPM(0.01f);
                default:
                    break;
            }
        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;
                ir_distance_mV = 0.0f;
                robot_state_actual = ROBOT_STATE_INIT;
                additional_led = 0;
            }            
        }
        // toggling the user led
        user_led = !user_led;
        // do only output via serial what's really necessary, this makes your code slow
        printf("IR sensor (mV): %3.3f, Encoder M2: %3d, Speed M2 (rps) %3.3f, Position M2 (rot): %3.3f\r\n",
            ir_distance_mV,
            encoder_M2.read(),
            speedController_M2.getSpeedRPS(),
            1000*positionController_M2.getRotation()
        );

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}
void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task) do_reset_all_once = true;
}
#include <mbed.h>
#include "PM2_Drivers.h"
# define M_PI 3.14159265358979323846 // number pi, an example in case you need it


bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(PC_13);  // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
void user_button_pressed_fcn(); // custom functions which get executed when user button gets pressed, definition below


// main runs as an own thread
int main()
{
    // states and actual state for state machine
    const int ROBOT_STATE_INIT     = 0;
    const int ROBOT_STATE_FORWARD  = 1;
    const int ROBOT_STATE_BACKWARD = 2;
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


    // Futaba Servo S3001 20mm 3kg Analog

    
    //int servo_counter = 0;    // define servo counter, this is an additional variable to make the servos move
    const int loops_per_seconds = static_cast<int>(ceilf( 1.0f / (0.001f * (float)main_task_period_ms) ));


    // 78:1, 100:1, ... Metal Gearmotor 20Dx44L mm 12V CB
    DigitalOut enable_motors(PB_15); // create DigitalOut object to enable dc motors

    FastPWM pwm_M1(PB_13); // motor M1 is used open-loop
    FastPWM pwm_M2(PA_9);  // motor M2 is closed-loop speed controlled (angle velocity)
    FastPWM pwm_M3(PA_10); // motor M3 is closed-loop position controlled (angle controlled)

    EncoderCounter  encoder_M1(PA_6, PC_7); // create encoder objects to read in the encoder counter values, since M1 is used open-loop no encoder would be needed for operation, this is just an example
    EncoderCounter  encoder_M2(PB_6, PB_7);
    EncoderCounter  encoder_M3(PA_0, PA_1);

    // create SpeedController and PositionController objects, default parametrization is for 78.125:1 gear box
    const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
    const float counts_per_turn = 20.0f * 78.125f; // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn = 180.0f / 12.0f;               // define motor constant in RPM/V
    const float k_gear = 100.0f / 78.125f;         // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
    const float kp = 0.1f;                         // define custom kp, this is the default speed controller gain for gear box 78.125:1

    SpeedController speedController_M2(counts_per_turn, kn, max_voltage, pwm_M2, encoder_M2); // default 78.125:1 gear box  with default contoller parameters
    // SpeedController speedController_M2(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_M2, encoder_M2); // parameters adjusted to 100:1 gear
    speedController_M2.setMaxAccelerationRPS(1.0f);
    // PositionController positionController_M3(counts_per_turn, kn, max_voltage, pwm_M3, encoder_M3); // default 78.125:1 gear with default contoller parameters
    PositionController positionController_M3(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_M3, encoder_M3); // parameters adjusted to 100:1 gear, we need a different speed controller gain here
    positionController_M3.setSpeedCntrlGain(kp * k_gear);   // adjust internal speed controller gain, this is just an example
    float max_speed_rps = 0.5f; // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
    positionController_M3.setMaxVelocityRPS(max_speed_rps); // adjust max velocity for internal speed controller


    main_task_timer.start();
    
    // this loop will run forever
    while (true) {

        main_task_timer.reset();

        if (do_execute_main_task) {
            
            // read analog input
            ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;

            // visual feedback that the main task is executed, setting this once would actually be enough
            additional_led = 1;

            // commanding the servos
        

            // state machine
            switch (robot_state_actual) {

                case ROBOT_STATE_INIT:

                    // check if servos are enabled, should be alreay disabled at this point, it's just an example
 

                    enable_motors = 1; // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled

                    robot_state_actual = ROBOT_STATE_FORWARD;
                    break;

                case ROBOT_STATE_FORWARD:

                    if (mechanical_button.read()) {
                        pwm_M1.write(ir_distance_mV); // write output voltage to motor M1
                        speedController_M2.setDesiredSpeedRPS(ir_distance_mV); // set a desired speed for speed controlled dc motors M2
                        positionController_M3.setDesiredRotation(ir_distance_mV); // set a desired rotation for position controlled dc motors M3

                        robot_state_actual = ROBOT_STATE_BACKWARD;
                    }
                    break;

                case ROBOT_STATE_BACKWARD:

                    if (positionController_M3.getRotation() >= 1.45f) {
                        pwm_M1.write(0.25f);
                        speedController_M2.setDesiredSpeedRPS(-0.5f);
                        positionController_M3.setDesiredRotation(0.0f);

                        robot_state_actual = ROBOT_STATE_SLEEP;
                    }
                    break;

                case ROBOT_STATE_SLEEP:

                    if (positionController_M3.getRotation() <= 0.05f) {
                        enable_motors = 0;
                        pwm_M1.write(0.5f);
                        speedController_M2.setDesiredSpeedRPS(0.5f);
                        
                        // robot_state_actual is not changed, there for the state machine remains in here until the blue button is pressed again
                    }

                default:
                
                    // do nothing
                    break;
            }

        } else {

            if (do_reset_all_once) {
                do_reset_all_once = false;

                ir_distance_mV = 0.0f;

                pwm_M1.write(0.5f);
                speedController_M2.setDesiredSpeedRPS(0.0f);
                positionController_M3.setDesiredRotation(0.0f);
                robot_state_actual = ROBOT_STATE_INIT;

                //servo_S1_angle = 0;
                //servo_S2_angle = 0;
                //servo_S1.disable();
                //servo_S2.disable();

                additional_led = 0;
            }            
        }

        // toggling the user led
        user_led = !user_led;

        // do only output via serial what's really necessary, this makes your code slow
        printf("IR sensor (mV): %3.3f, Encoder M1: %3d, Speed M2 (rps) %3.3f, Position M3 (rot): %3.3f\r\n",
               ir_distance_mV,
               encoder_M1.read(),
               speedController_M2.getSpeedRPS(),
               positionController_M3.getRotation());

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
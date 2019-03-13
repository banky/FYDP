/*
 *  Copyright Beach Cleaning Automated
 * 
 *  Author: Bankole Adebajo
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define MAX_LEFT 75
#define MAX_RIGHT 110
#define STRAIGHT 93
#define MAX_KEYBOARD_INPUT 120
#define M_PI 3.14159265358979323846
#define DO_CALIBRATION 1

extern HardwareSerial Serial;

Servo drivetrain; // create esc signal object to control esc
Servo steering;   // create servo object to control a servo
Servo conveyor;
Servo brush;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Servo variables
int rotation = 0; // variable to store the servo position
int initialRotation = 0;

// Incoming serial data
int incomingByte = 0;

// Motor Speed variables
int motorSpeed = 20;
int delta = 5;

// Bristle Roller Speed
int bristleRollerSpeed = 0;

// Conveyor Speed
int conveyorSpeed = 0;

enum JETSON_CMD
{
    MOTOR_SPEED = MAX_KEYBOARD_INPUT,
    SERVO_ANGLE,
    BRISTLE_ROLLER,
    CONVEYOR,
    JETSON_CMD_END
};

// Pin configs
int conveyorPin = 11; //the CONVEYOR signal output pins
int brushPin = 12;    //the BRUSH signal output pins
int drivePin = 9;     //the BRUSH signal output pins
int steeringPin = 10; //the BRUSH signal output pins

void setup()
{
    Serial.begin(9600); // Opens serial port, sets data rate to 9600 bps

    drivetrain.attach(drivePin);
    steering.attach(steeringPin);
    conveyor.attach(conveyorPin);
    brush.attach(brushPin);

    drivetrain.write(30); // Arms ESC

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    delay(1000);

    bno.setExtCrystalUse(true);

    delay(1000); // Show time.

#ifdef DO_CALIBRATION
    // Perform calibration every time
    calibrate();
#endif
}

void loop()
{
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    // publish_imu_event(event);

    // send data only when you receive data:
    if (Serial.available() > 0)
    {
        // read the incoming byte:
        incomingByte = Serial.read();

        // Used for keyboard commands
        if (incomingByte < MAX_KEYBOARD_INPUT)
        {
            keyboardCommand(incomingByte);
        }

        // Used for inputs from Jetson
        else if (incomingByte < JETSON_CMD_END)
        {
            // JETSON_CMD cmd = Serial.parseInt();
            JETSON_CMD cmd = (JETSON_CMD)incomingByte;
            incomingByte = Serial.parseInt();
            char val = (char)incomingByte;

            jetsonCommand(cmd, val);
        }

        updateMotors();
    }

    delay(100);
}

/*
 *  Publishes an event from IMU over serial to Jetson
 */
void publish_imu_event(sensors_event_t event)
{
    float yaw, roll, pitch;

    // Note that these are defined incorrectly. Tested on IMU
    yaw = event.orientation.x;
    roll = event.orientation.y;
    pitch = event.orientation.z;

    /* Display the floating point data */
    // Yaw
    if (yaw > 180)
        yaw = M_PI / 180 * (yaw - 360);
    else
        yaw = M_PI / 180 * yaw;

    // Take angle from x axis
    if (yaw > 0)
        yaw = M_PI / 2 - yaw;
    else
        yaw = M_PI / 2 - yaw;
    
    // Angle Wrap
    while (yaw < -M_PI || yaw > M_PI)
    {
        if (yaw < -M_PI)
        {
            yaw = yaw + 2 * M_PI;
        } else 
        {
            yaw = yaw - 2 * M_PI;
        }
    }

    Serial.print(yaw);
    
    Serial.print(",");

    // Roll
    if (roll > 180)
        Serial.print(M_PI / 180 * (roll - 360));
    else
        Serial.print(M_PI / 180 * roll);

    Serial.print(",");

    // Pitch
    if (pitch > 180)
        Serial.println(M_PI / 180 * (pitch - 360));
    else
        Serial.println(M_PI / 180 * pitch);
}

// Calibrates ESC
void calibrate()
{
    delay(1000);

    // Full throttle calibration
    motorSpeed = 95;
    drivetrain.write(motorSpeed);
    delay(3500);

    // Full brake calibration
    motorSpeed = 20;
    drivetrain.write(motorSpeed);
    delay(2000);

    // Neutral calibration
    motorSpeed = 95;
    drivetrain.write(motorSpeed);
    delay(2500);

    motorSpeed = 97;
    delay(100);

    steering.attach(10); // Attaches the servo on pin 10 to the servo object

    // Turn wheels to straight position
    rotation = steering.read();
    initialRotation = rotation;
    // Serial.println("Initial Servo Value: ");
    // Serial.println(rotation);
}

void updateMotors()
{
    drivetrain.write(motorSpeed); //Set motor speed on ESC
    steering.write(rotation);

    // Update conveyor
    int PWMvalue = conveyorSpeed * 5 + 1500; //scale up to 1000-2000
    conveyor.writeMicroseconds(PWMvalue);

    // Update bristle roller
    PWMvalue = bristleRollerSpeed * 5 + 1500; //scale up to 1000-2000
    brush.writeMicroseconds(PWMvalue);
}

/*
 *  WARN: Keyboard commands. Only these should be allowed to give serial output
 *        This is to ensure the jetson doesn't get confused thinking text is IMU data
 */

// Adds i to motors. MINIMUM TO START MOVING IS 99.
void goForward(int i)
{
    motorSpeed = motorSpeed + i;
}

// Subtracts i to motors.
void goBack(int i)
{
    motorSpeed = motorSpeed - i;
}

void goLeft(int i)
{
    rotation = rotation - i;
}

void goRight(int i)
{
    rotation = rotation + i;
}

void goStraight()
{
    rotation = initialRotation;
}

void goStop()
{
    motorSpeed = 97;
}

void goConveyor()
{
    int percent;

    Serial.println("Entered goConveyor.\n");
    while (Serial.available() < 1)
    {
        delay(100);
    }
    Serial.println("Done waiting.\n");

    long proposedValue = Serial.parseInt();
    Serial.println("received following value: ");
    Serial.println(proposedValue);
    Serial.println("\n");

    if (proposedValue == 0)
    {
        percent = 0;
        Serial.println("Conveyor Motor Stopped.\n");
    }

    else if (proposedValue >= -100 && proposedValue <= 100)
    {
        percent = proposedValue;
        Serial.print("Conveyor Motor set to: ");
        Serial.print(percent);
        Serial.println("%\n");
    }

    else
    {
        Serial.println("oh dear. that won't do.");
    }

    int PWMvalue = percent * 5 + 1500; //scale up to 1000-2000
    conveyor.writeMicroseconds(PWMvalue);
}

void goBrush()
{
    int percent;

    Serial.println("Entered goBrush.\n");
    while (Serial.available() < 1)
    {
        delay(100);
    }
    Serial.println("Done waiting.\n");

    long proposedValue = Serial.parseInt();
    Serial.println("received following value: ");
    Serial.println(proposedValue);
    Serial.println("\n");

    if (proposedValue == 0)
    {
        percent = 0;
        Serial.println("Brush Motor Stopped.\n");
    }

    else if (proposedValue >= -100 && proposedValue <= 100)
    {
        percent = proposedValue;
        Serial.print("Brush Motor set to: ");
        Serial.print(percent);
        Serial.println("%\n");
    }

    else
    {
        Serial.println("oh dear. that won't do.");
    }

    int PWMvalue = percent * 5 + 1500; //scale up to 1000-2000
    brush.writeMicroseconds(PWMvalue);
}

void autoTest()
{
    goStraight();
    goForward(2);
    updateMotors();
    delay(2000);

    goRight(15);
    updateMotors();
    delay(2000);

    goStraight();
    goLeft(15);
    updateMotors();
    delay(2000);

    goStop();
    goStraight();
    updateMotors();
    delay(2000);
}

void keyboardCommand(int cmd)
{
    // Calibrate if "c" is received.
    if (incomingByte == 99)
    {
        Serial.println("calibrating");
        calibrate();
        Serial.println("Done calibration");
    }

    // goForward if "w" is received.
    else if (incomingByte == 119)
    {
        goForward(1);
    }

    // goBack if "s" is received.
    else if (incomingByte == 115)
    {
        goBack(1);
    }

    // goLeft if "a" is received.
    else if (incomingByte == 97)
    {
        goLeft(1);
    }

    // goRight if "d" is received.
    else if (incomingByte == 100)
    {
        goRight(1);
    }

    // goStraight if "q" is received.
    else if (incomingByte == 113)
    {
        goStraight();
    }

    // goStop if "e" is received.
    else if (incomingByte == 101)
    {
        goStop();
    }

    // goConveyor if "v" is received.
    else if (incomingByte == 118)
    {
        goConveyor();
    }

    // goBrush if "b" is received.
    else if (incomingByte == 98)
    {
        goBrush();
    }

    // autoTest if "p" is received.
    else if (incomingByte == 112)
    {
        autoTest();
    }

    else
    {
    }
}

/*
 * END of Keyboard commands
 */

/**
 * cmd: Command being sent by Jetson
 * val: Value to use for specified command
 */
void jetsonCommand(JETSON_CMD cmd, char val)
{
    switch (cmd)
    {
    case MOTOR_SPEED:
        // Motor speed value between 0 and 256
        // 0 is full reverse, 256 is full forward, 128 is stop

        Serial.print("Setting motor speed to: ");
        Serial.println(val);
        motorSpeed = val;
        updateMotors();
        break;

    case SERVO_ANGLE:
        // Rotation is value between 0 and 255
        // 0 is -pi/4 255 is pi/4
        Serial.print("Setting servo angle to: ");
        Serial.println(val);
        rotation = ((val / 255) * (MAX_RIGHT - MAX_LEFT)) + MAX_LEFT;
        updateMotors();
        break;

    case BRISTLE_ROLLER:
        // Speed is a value between 0 and 255
        // 0 is full reverse, 128 is stop, 255 is full forward
        Serial.print("Setting bristle roller speed to: ");
        Serial.println(val);
        bristleRollerSpeed = int((val / 256) * 200 - 100);
        updateMotors();
        break;

    case CONVEYOR:
        // Speed is a value between 0 and 255
        // 0 is full reverse, 128 is stop, 255 is full forward
        Serial.print("Setting conveyor speed to: ");
        Serial.println(val);
        conveyorSpeed = int((val / 256) * 200 - 100);
        updateMotors();
        break;

    default:
        break;
    }
}

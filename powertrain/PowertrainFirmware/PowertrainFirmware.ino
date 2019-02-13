#include <Servo.h>

Servo esc_signal; // create esc signal object to control esc
Servo myservo;    // create servo object to control a servo

extern HardwareSerial Serial;

// Servo limits NEED TO BE UPDATED
#define maxLeft 90
#define maxRight 100
#define MAX_KEYBOARD_INPUT 120

// Servo variables
int rotation = 0; // variable to store the servo position
int initialRotation = 0;

// Incoming serial data
int incomingByte = 0;

// Motor Speed variables
int motorSpeed = 20;
int delta = 5;

enum JETSON_CMD
{
    MOTOR_SPEED = MAX_KEYBOARD_INPUT,
    SERVO_ANGLE,
    JETSON_CMD_END
};

void setup()
{
    Serial.begin(9600); // Opens serial port, sets data rate to 9600 bps

    esc_signal.attach(9); // Specifies pin used for ESC
    esc_signal.write(30); // Arms ESC

    myservo.attach(10); // Attaches the servo on pin 10 to the servo object

    Serial.println("Starting PowerTrain FW");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    delay(2000); // Show time.
}

void loop()
{
    // send data only when you receive data:
    if (Serial.available() > 0)
    {
        // read the incoming byte:
        incomingByte = Serial.read();

        // say what you got:
        Serial.print("I received: ");
        Serial.println(incomingByte);

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

        else
        {
            Serial.println("I don't know what that command does");
        }

        Serial.print("motorSpeed: ");
        Serial.println(motorSpeed);

        Serial.print("servoPosition: ");
        Serial.println(rotation);

        updateMotors();

        delay(100);
    }
}

// Calibrates ESC
void calibrate()
{
    delay(1000);

    // Full throttle calibration
    motorSpeed = 95;
    esc_signal.write(motorSpeed);
    delay(3500);

    // Full brake calibration
    motorSpeed = 20;
    esc_signal.write(motorSpeed);
    delay(2000);

    // Neutral calibration
    motorSpeed = 95;
    esc_signal.write(motorSpeed);
    delay(2500);

    motorSpeed = 97;
    delay(100);

    myservo.attach(10); // Attaches the servo on pin 10 to the servo object

    // Turn wheels to straight position
    rotation = myservo.read();
    initialRotation = rotation;
    Serial.println("Initial Servo Value: ");
    Serial.println(rotation);
}

void updateMotors()
{
    esc_signal.write(motorSpeed); //Set motor speed on ESC
    myservo.write(rotation);
}

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

    // autoTest if "p" is received.
    else if (incomingByte == 112)
    {
        Serial.println("testing");
        autoTest();
    }

    else
    {
        Serial.println("I don't know what that command does");
    }
}

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

        motorSpeed = val;
        updateMotors();
        digitalWrite(LED_BUILTIN, HIGH);
        break;
    case SERVO_ANGLE:
        // Rotation is value between 0 and 256
        // 0 is -pi/4 256 is pi/4
        rotation = val;
        updateMotors();
        digitalWrite(LED_BUILTIN, LOW);
        break;

    default:
        break;
    }
}

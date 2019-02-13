#include <Servo.h>
Servo esc_signal;

int incomingByte = 0;           // for incoming serial data
int motorSpeed = 20;
int delta = 5;

void setup() {
        esc_signal.attach(9);   //Specifies pin used for ESC
        esc_signal.write(30);   //Arms ESC
        
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps

        delay(2000);            //Show time.
}

void loop() {

        // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte);

                if(incomingByte == 119) {
                    motorSpeed = motorSpeed + delta;             
                }

                else if(incomingByte == 115) {
                    motorSpeed = motorSpeed - delta;             
                }

                Serial.print("motorSpeed: ");
                Serial.println(motorSpeed);

                esc_signal.write(motorSpeed);     //Set motor speed on ESC
                
                delay(100);
        }
}

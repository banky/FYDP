int ultrasonicPin = A0;
int pwPin1 = 3;
int ultrasonicVal = 0;
long sensor, anVolt, mm, inches = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwPin1, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

int read_sensor (){
//  anVolt = analogRead(ultrasonicPin);
//  mm = anVolt*5; //Takes bit count and converts it to mm
//  inches = mm/25.4; //Takes mm and converts it to inches

  sensor = pulseIn(pwPin1, HIGH);
  inches = sensor / 147;
  mm = inches * 25.4;
  return mm;
}

void loop() {
  // put your main code here, to run repeatedly:
  ultrasonicVal = read_sensor();
  Serial.print("Value: ");
  Serial.println(ultrasonicVal);
  delay(500);
}

int sw;
int pot;

void setup() {

  pinMode(A0, INPUT);       // pot input for fan speed control
  pinMode(6, OUTPUT);       // fan PWM outout to motor driver
  pinMode(9, OUTPUT);       // motor PWM for ball input
  pinMode(5, INPUT_PULLUP); // fan enable switch

  Serial.begin(115200);

}

void loop() {

  sw = digitalRead(5);
  pot = analogRead(A0);
  pot = map(pot,0,1023,0,255);

  Serial.print(pot);
  Serial.print(" , ");
  Serial.println(sw);

  if (sw == 0) {
    analogWrite(6, pot);
  }

  else {
    analogWrite(6, 0);
  }

  analogWrite(9, 40);    // hard coded motor output for ball input
  
  delay(5);
}

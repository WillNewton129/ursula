const int svPinR = 5;   // PWM pin RIGHT1
const int enPinR1 = 12;   // Enable RIGHT1
const int frPinR1 = 13;   // Direction of spinning RIGHT1
const int enPinR2 = 3;   // Enable RIGHT1
const int frPinR2 = 4;   // Direction of spinning RIGHT2
const int bkPinR1 = 2; // Hard Break RIGHT1
 
const int svPinL = 6;   // PWM pin LEFT1
const int enPinL1 = 11;   // Enable LEFT1
const int frPinL1 = 7;   // Direction of spinning LEFT1
const int enPinL2 = 8;   // Enable LEFT2
const int frPinL2 = 9;   // Direction of spinning LEFT2
const int bkPinL1 = 10; // Hard Break LEFT1
 
int previousSpeedL = 0;
int previousSpeedR = 0;
 
void brakeBoth(int &prevL, int &prevR) {
  int maxPrev = max(abs(prevL), abs(prevR));
  for (int t = maxPrev; t >= 0; t -= 50) { // depending on which one is turning more, drop the speed down on each side in t steps of 30 at the same rate
    int speedL = max(0, t - (maxPrev - abs(prevL)));
    int speedR = max(0, t - (maxPrev - abs(prevR)));
    analogWrite(svPinL, speedL);
    analogWrite(svPinR, speedR);
    delay(20);
  }
  prevL = 0;
  prevR = 0;
}
 
void brakeOne(int &prev, int svPin) {
  for (int t = prev; t >= 0; t -= 50) { // originally had steps of t = 10 and delays of 200ms, changed them to have bigger jumps and quicker
    analogWrite(svPin, t);
    delay(20);
  }
  prev = 0;
}
 
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);  // e.g. 20 ms instead of default ~1000 ms to get more information more frequently and avoid Arduino waiting 1s for Jetson to send a full command if it fails once
  pinMode(svPinR, OUTPUT);
  pinMode(frPinR1, OUTPUT);
  pinMode(frPinR2, OUTPUT);
  pinMode(enPinR1, OUTPUT);
  pinMode(enPinR2, OUTPUT);
  pinMode(svPinL, OUTPUT);
  pinMode(frPinL1, OUTPUT);
  pinMode(frPinL2, OUTPUT);
  pinMode(enPinL1, OUTPUT);
  pinMode(enPinL2, OUTPUT);
  pinMode(bkPinR1, OUTPUT);
  pinMode(bkPinL1, OUTPUT);
  // Set PWM frequency on pin 5 and 6 to ≈ 2 kHz - MIGHT NOT NEED THIS IF WE ARE NOT USING PWM
  TCCR0B = (TCCR0B & 0b11111000) | 0x02;
  digitalWrite(enPinR1, LOW); // Enabling motor controller from the start rn, we can change it so its activated after pressing one button and maybe add an LED to know if it is ON
  analogWrite(svPinR, 0);
  digitalWrite(enPinR2, LOW);
  digitalWrite(enPinL1, LOW);
  analogWrite(svPinL, 0);
  digitalWrite(enPinL2, LOW);
  digitalWrite(bkPinL1, HIGH);
  digitalWrite(bkPinR1, HIGH);
}
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n'); // Read input until ENTER
    line.trim();
    // Read left, right and enable (emergency) string digits and convert to integers one by one
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1); // These commands read the position of the COMMA in the string, not the value
    int speedL = line.substring(c1 + 1, c2).toInt();
    int speedR = -1*(line.substring(c2 + 1, c3).toInt());
    int estop  = line.substring(c3 + 1).toInt();
 
    if (line.startsWith("CMD") && estop==1) { //Input is given in the CMD ... format and emergency button has not been pressed
      // If a command is passed that means the motors have to be enabled - low enable = working
      digitalWrite(enPinR1, LOW);
      digitalWrite(enPinR2, LOW);
      digitalWrite(enPinL1, LOW);
      digitalWrite(enPinL2, LOW);
      digitalWrite(bkPinL1, HIGH);
      digitalWrite(bkPinR1, HIGH);
 
      // FULL BRAKE
      if (speedL == 0 && speedR == 0) {
        brakeBoth(previousSpeedL, previousSpeedR);
        return;
      }
      // LEFT MOTOR
      if (speedL == 0) {
        brakeOne(previousSpeedL, svPinL);
      } else {
        digitalWrite(frPinL1, (speedL > 0) ? HIGH : LOW); // if speed >0 drive forward, if speed <0 drive backwards
        digitalWrite(frPinL2, (speedL > 0) ? HIGH : LOW); // if speed >0 drive forward, if speed <0 drive backwards
        int csL = constrain(abs(speedL), 0, 255);
        analogWrite(svPinL, csL);
        previousSpeedL = csL; // Store before next command
      }
      // RIGHT MOTOR
      if (speedR == 0) {
        brakeOne(previousSpeedR, svPinR);
      } else {
        digitalWrite(frPinR1, (speedR > 0) ? HIGH : LOW); // if speed >0 drive forward, if speed <0 drive backwards
        digitalWrite(frPinR2, (speedR > 0) ? HIGH : LOW); // if speed >0 drive forward, if speed <0 drive backwards
        int csR = constrain(abs(speedR), 0, 255);
        analogWrite(svPinR, csR);
        previousSpeedR = csR; // Store before next command
      }
    }
    else { // if no valid command is passed (not starting with CMD) OR emergency button has been pressed (c3=1), the motors are disabled
      if (!line.startsWith("CMD")){
        digitalWrite(enPinR1, HIGH);
        digitalWrite(enPinR2, HIGH);
        digitalWrite(enPinL1, HIGH);
        digitalWrite(enPinL2, HIGH);
        Serial.println("Motor DISABLED - invalid input");
      }
      if (estop != 1) {
        // BREAK
        digitalWrite(bkPinL1, LOW);
        digitalWrite(bkPinR1, LOW);
        previousSpeedL = 0;
        previousSpeedR = 0;
        // DISABLE THE MOTOR CONTROLLERS
        digitalWrite(enPinR1, HIGH);
        digitalWrite(enPinR2, HIGH);
        digitalWrite(enPinL1, HIGH);
        digitalWrite(enPinL2, HIGH);
        Serial.println("Motor DISABLED - EMERGENCY STOP");
      }
      return;
    }
  }
}
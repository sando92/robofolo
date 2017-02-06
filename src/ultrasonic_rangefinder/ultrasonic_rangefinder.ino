 const int TRIGLeft = 22; //TRIG = red or grey(middle sensor) scotch tape, grey wire
const int ECHOLeft = 23; //ECHO = yellow scotch tape, white wire
//Vcc with blue scotch tape, purple wire
//GND with brown scotch tape ,black or yellow(right sensor) wire

const int TRIGMiddle = 24;
const int ECHOMiddle = 25;

const int TRIGRight = 26;
const int ECHORight = 27;

long retourEchoLeft;
long distanceLeft = 4000;
long retourEchoMiddle;
long distanceMiddle = 4000;
long retourEchoRight;
long distanceRight = 4000;

//bool ultrasonicRangeFinderSTOP = false;

void setup() {
  pinMode(TRIGLeft, OUTPUT);
  digitalWrite(TRIGLeft, LOW);
  pinMode(ECHOLeft, INPUT);

  pinMode(TRIGMiddle, OUTPUT);
  digitalWrite(TRIGMiddle, LOW);
  pinMode(ECHOMiddle, INPUT);

  pinMode(TRIGRight, OUTPUT);
  digitalWrite(TRIGRight, LOW);
  pinMode(ECHORight, INPUT);

  Serial.begin(38400);
  Serial.println("Setup done, pins initialized");
}

void loop() {
  digitalWrite(TRIGLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGLeft, LOW);
  retourEchoLeft = pulseIn(ECHOLeft, HIGH);   //return the width of high signal on ECHO when obstacle detected
  delay(5);
  //Serial.print("Width of left pulse echo : ");
  //Serial.println(retourEchoLeft);

  digitalWrite(TRIGMiddle, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGMiddle, LOW);
  retourEchoMiddle = pulseIn(ECHOMiddle, HIGH);
  delay(5);
  //Serial.print("Width of middle pulse echo : ");
  //Serial.println(retourEchoMiddle);

  digitalWrite(TRIGRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGRight, LOW);
  retourEchoRight = pulseIn(ECHORight, HIGH);

  //Serial.print("Width of right pulse echo : ");
  //Serial.println(retourEchoRight);

  if (retourEchoLeft != 0) {
    distanceLeft = retourEchoLeft * 17 / 100;
    Serial.print("LEFT (mm): ");
    Serial.println(distanceLeft);
  }
  if (retourEchoMiddle != 0) {
    distanceMiddle = retourEchoMiddle * 17 / 100;
    Serial.print("MIDDLE (mm): ");
    Serial.println(distanceMiddle);
  }

  if (retourEchoRight != 0) {
    distanceRight = retourEchoRight * 17 / 100;
    Serial.print("RIGHT (mm): ");
    Serial.println(distanceRight);
  }

  if (distanceLeft < 200 || distanceMiddle < 200 || distanceRight < 200) {
    //ultrasonicRangeFinderSTOP = true;
    Serial.println(" STOP ! ");
  }

  delay(100);
}

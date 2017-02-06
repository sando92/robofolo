#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <SimpleTimer.h> // to be deleted and use timer in freeRTOS

#define _DEBUG true

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define two Tasks for DigitalRead & AnalogRead
//void TaskMotorsController( void *pvParameters );
void TaskUltrasonicRangeFinder( void *pvParameters );

//global var
bool ultrasonicRangeFinderSTOP = false;

void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(38400); //bien tester que un baud aussi grand ne gène pas la communication pour Rangefinder

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two Tasks to run independently.
  /*xTaskCreate(
    TaskMotorsController
    ,  (const portCHAR *)"MotorsController"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );*/

  xTaskCreate(
    TaskUltrasonicRangeFinder
    ,  (const portCHAR *)"UltrasonicRangeFinder"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


/*---------------- Motors Controller ---------------*/

/*SimpleTimer timer;
const int motor1 = 10;
const int motor2 = 11;
int countEncoder1 = 0;
int countEncoder2 = 0;
int cmd1 = 0;
int cmd2 = 0;

const float controlEngineeringFrequency = 50.0;
const int gearRatio = 100;
const int encoderCPR = 32;

float spinPerSecWanted = 0.2; //perimeter's wheel of 48,5cm 

float sumError1 = 0;
float sumError2 = 0;
float precError1 = spinPerSecWanted; // pas trop compris pourquoi encore -> compris car vitesse initiale egale à 0
float precError2 = spinPerSecWanted;

void TaskMotorsController( void *pvParameters __attribute__((unused)) )  // This is a Task.
{ 
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  //delay(5000); necessary ?
  
  // ##### PROBLEME AVEC L'UTILISATION D'INTERRUPT DANS UNE TASK ???
  
  attachInterrupt(0, incrCountEncoder1, CHANGE); //Interruption 0 is on pin2
  attachInterrupt(1, incrCountEncoder2, CHANGE); //Interruption 1 is on pin3
  timer.setInterval(1000/controlEngineeringFrequency, PIDController); //every 20 ms, the function is called

  for (;;)
  {

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      
      Serial.println("Hello TaskMotorsController");
      
        timer.run();
        //delay(10); necessary ?

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void incrCountEncoder1(){
 countEncoder1++; 
}  

void incrCountEncoder2(){
 countEncoder2++;
}

void PIDController(){
  unsigned long time1 = millis();
  float kp1 = 200; // kps must be fixed by testing, for a Proportional control
  float kp2 = 200;
  
  float ki1 = 2.5;
  float ki2 = 2.9; //kis must be fixed by testing, for an Integral control
  
  float kd1 = 10666; //kds must be fixed by testing, for a Derivative control
  float kd2 = 9333;

  float encoderFrequency1 = controlEngineeringFrequency*(float)countEncoder1;
  float encoderFrequency2 = controlEngineeringFrequency*(float)countEncoder2;
  
  float spinPerSec1 = encoderFrequency1/(float)encoderCPR/(float)gearRatio;
  float spinPerSec2 = encoderFrequency2/(float)encoderCPR/(float)gearRatio;
  float error1 = spinPerSecWanted - spinPerSec1;
  float error2 = spinPerSecWanted - spinPerSec2;
  
  countEncoder1 = 0;
  countEncoder2 = 0;
  
  sumError1 += error1;
  sumError2 += error2;
  float deltaError1 = error1 - precError1;
  float deltaError2 = error2 - precError2;
  precError1 = error1;
  precError2 = error2;
  
  cmd1 = kp1*error1 + kd1*deltaError1 + ki1*sumError1; 
  cmd2 = kp2*error2 + kd2*deltaError2 + ki2*sumError2;
  
  if(cmd1 < 0) {
    cmd1 = 0;
  }
  else if (cmd1 > 105) {
    cmd1 = 105;
  }
  
  if(cmd2 < 0) {
    cmd2 = 0;
  }
  else if (cmd2 > 105) {
    cmd2 = 105;
  }
  
  // if value equals 0 -> motor runs, if equals 105 motor stops
  
  if (ultrasonicRangeFinderSTOP){
    analogWrite(motor1, 105);
    analogWrite(motor2, 105);
  } else {
    analogWrite(motor1, 105 - cmd1);
    analogWrite(motor2, 105 - cmd2);
  }
  
  if (_DEBUG) {
    Serial.print(" Tour par sec 1 :");
    Serial.println(spinPerSec1, 8);
    Serial.print(" Tour par sec 2 :");
    Serial.println(spinPerSec2, 8);
    
    unsigned long time2 = millis();
    if (time2-time1 < 10) {
      delay(10 - time2 + time1);
    }
  }
}  */


/*------------------ Range Finder ----------------*/

void TaskUltrasonicRangeFinder( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  const int TRIGLeft = 22;
  const int ECHOLeft = 23;

  const int TRIGMiddle = 24;
  const int ECHOMiddle = 25;

  const int TRIGRight = 26;
  const int ECHORight = 27;

  long retourEchoLeft;
  long distanceLeft;
  long retourEchoMiddle;
  long distanceMiddle;
  long retourEchoRight;
  long distanceRight;
  
  pinMode(TRIGLeft, OUTPUT);
  digitalWrite(TRIGLeft, LOW);
  pinMode(ECHOLeft, INPUT);

  pinMode(TRIGMiddle, OUTPUT);
  digitalWrite(TRIGMiddle, LOW);
  pinMode(ECHOMiddle, INPUT);

  pinMode(TRIGRight, OUTPUT);
  digitalWrite(TRIGRight, LOW);
  pinMode(ECHORight, INPUT);
  
  for (;;)
  {

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      
      Serial.println("Hello TaskUltrasonicRangeFinder");
      
      digitalWrite(TRIGLeft, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGLeft,LOW);
      retourEchoLeft = pulseIn(ECHOLeft, HIGH);   //return the width of high signal on ECHO when obstacle detected
      delay(5);
      //Serial.print("Width of left pulse echo : ");
      //Serial.println(retourEchoLeft);

      digitalWrite(TRIGMiddle, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGMiddle,LOW);
      retourEchoMiddle = pulseIn(ECHOMiddle, HIGH);
      delay(5);
      //Serial.print("Width of middle pulse echo : ");
      //Serial.println(retourEchoMiddle);

      digitalWrite(TRIGRight, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGRight,LOW);
      retourEchoRight = pulseIn(ECHORight, HIGH);

      //Serial.print("Width of right pulse echo : ");
      //Serial.println(retourEchoRight);
      
      if (retourEchoLeft != 0) {
        distanceLeft = retourEchoLeft*17/100;
        Serial.print("LEFT (mm): ");
        Serial.println(distanceLeft);
      }
      if (retourEchoMiddle != 0) {
        distanceMiddle = retourEchoMiddle*17/100;
        Serial.print("MIDDLE (mm): ");
        Serial.println(distanceMiddle);
      }
      if (retourEchoRight != 0) {
        distanceRight = retourEchoRight*17/100;
        Serial.print("RIGHT (mm):  ");
        Serial.println(distanceRight);
      }
      
      if (distanceLeft < 200 || distanceMiddle < 200 || distanceRight < 200) {
        ultrasonicRangeFinderSTOP = true;
        Serial.println("! STOP !");
      }
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    
     /* Block for 500ms. */
     //const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    
    vTaskDelay(5 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
  }
}

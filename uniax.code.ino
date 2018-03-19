
//------------------------------------------------------------------------------
// This simple sketch will permit users to control a linear actuator with analog positon feedback,
// tell it to move a certain distance (percent strain) and return to the start position, all the
// while printing position and force data read from the potentiometer of the motor and a load cell
// respectively.
//
// See end of file for usage and attribution
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// experiment parameters
//------------------------------------------------------------------------------

float tol = 4;//tolerance of stop positions (+/- tol) in pot units
const float per = .25; // strain percentage/100
int count = 5;//number of cycles
int relax = 50; //time (ms) to pause after pull, set low (<50 ms to have  quick switch from pull to push)

//Speed of motors. Alter according to sample length
//to achieve your desired strain rate. May very with motor model
int motSpeed = 220;

//------------------------------------------------------------------------------
//calibration variables
//------------------------------------------------------------------------------

////************ calibration values for motor position (calipers help!!)  ****************//////
float len = 26;// distance between clamps when motor is fully extended (mm)
float potMax = 862; //max pot reading
float potMin = 13; //min pot reading
float strokeMax = 111; //max position (mm) of motor from a reference point (e.g., collar of housing)
float strokeMin = 15; // min position (mm) of motor from same reference point

////************ calibration values for load cell  ****************//////
float ReadingA_Strain1 = 75.0;//first raw analog reading
float LoadA_Strain1 = 100; // first load (Kg,lbs..)
float ReadingB_Strain1 = 148;//second raw analog reading
float LoadB_Strain1 = 200; // second (Kg,lbs..)


//------------------------------------------------------------------------------
// global variables. change with caution.
//------------------------------------------------------------------------------

float stroke = 0;//stroke goal
int sensorPin = 4; // select the input pin for the potentiometer wire from motor
int i = 0; //for loop to run through cycles
int cycle = 0; // to record cycle number

//motor control board variables
const int enable = 8; //enable D8 for switch control, make use jumper spans D8 on motor control shield
const int PWMA = 11; //A switch
const int PWMB = 3; //B switch

int mm; //distance between clamps in mm

float newReading_Strain1 = 0; //for load cell reading

int startPos = 0;//start position of motor

//buttons variables
const int buttonGo = 2;//buttons to move the motor
const int buttonPush = 13;//buttons to move the motor
const int buttonPull = 12;//buttons to move the motor

//time variables, best to store as unsigned long
unsigned long t ;
unsigned long tstop = 0; //time when motor stopped retracting
long time = 0; // to store time

//variables for goal and current positions
int goalPosition = 0;
int goalPositionPush = 0;
int goalPositionPull = 0;
int CurrentPosition = 0;
int StartPosition = 0;


String state = "nada"; //to store state of motor (push, pull, pushnudge, pullnudge)


float lenPot = len * (potMax - potMin) / (strokeMax - strokeMin); //length beteen clamps in pot units

//------------------------------------------------------------------------------
// functions/methods
//------------------------------------------------------------------------------


//function to print data, takes instaneous values in loop
void printDat(unsigned long t, String state, int cycle)
{
  float stroke = per * (lenPot + (potMax - startPos));//lenth of stroke motor will take
  float pot = (analogRead(sensorPin)); //pot reading raw

  //compute instantaneous length between motors (i.e., length of sample)
  float mm = len + (strokeMax - strokeMin) * ((potMax - pot) / (potMax - potMin)); //len is distance btw clamps
  //take strain reading and compute load based on calibration
  float newReading_Strain1 = analogRead(0);//strain1 is on amplifier is read through A0
  float load_Strain1 = ((LoadB_Strain1 - LoadA_Strain1) / (ReadingB_Strain1 - ReadingA_Strain1)) *
                       (newReading_Strain1 - ReadingA_Strain1) + LoadA_Strain1;

  //print the data
  Serial.print(t); //time in ms
  Serial.print(",");
  Serial.print(pot);//pot reading
  Serial.print(",");
  Serial.print(mm, 3); //postion data
  Serial.print(",");
  Serial.print(stroke, 3); // stroke in mm
  Serial.print(",");
  Serial.print(newReading_Strain1, 3); // raw strain value (comes in handy when calibrating load cell)
  Serial.print(",");
  Serial.print(load_Strain1, 2); // strain
  Serial.print(",");
  Serial.print(state); // state (adjusting, pull, push)
  Serial.print(",");
  Serial.println(cycle); // state (adjusting, pull, push)
}

//function to stop actuator (i.e., kill power)
void stopActuator() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

//function to push actuator as fast as motSpeed
void pushActuator() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, motSpeed);
}

//function to push actuator very slowly during adjustments
void pushActuatorSlow() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 50);

}

//function to push actuator as fast as motSpeed
void pullActuator() {
  analogWrite(PWMB, 0);
  analogWrite(PWMA, motSpeed);
}

//function to pull actuator very slowly during adjustments
void pullActuatorSlow() {
  analogWrite(PWMA, 50);
  analogWrite(PWMB, 0);
}



void setup() {

  //start serial connection, increase for
  Serial.begin(57600);//baud rate, increasse for higher sampling rate
  digitalWrite(enable, HIGH);//set enable pin to high
  pinMode(enable, OUTPUT);
  time = millis(); //store first time reading

  //set button pins as input
  pinMode(buttonGo, INPUT);
  pinMode(buttonPush, INPUT);
  pinMode(buttonPull, INPUT);

  //set initial button pins as high, button will cause drop

  digitalWrite(buttonGo, HIGH);//enable internal pullups
  digitalWrite(buttonPull, HIGH);//enable internal pullups
  digitalWrite(buttonPush, HIGH);//enable internal pullups

  //print a header for serial monitor
  Serial.print("ms"); //time in ms
  Serial.print(", ");
  Serial.print("pot val");//pot reading
  Serial.print(", ");
  Serial.print("mm"); //from conversion in this script
  Serial.print(", ");
  Serial.print("stroke");// stroke in mm
  Serial.print(",");
  Serial.print("strain");//raw strain value from load cell
  Serial.print(", ");
  Serial.print("g");// force in grams
  Serial.print(", ");
  Serial.print("state"); // state (nudgepull, nudgepush, pull, push)
  Serial.print(", ");
  Serial.println("cycle"); // which cycle
}


void loop() {

  //take a load reading and do math
  float newReading_Strain1 = analogRead(0);
  float load_Strain1 = ((LoadB_Strain1 - LoadA_Strain1) / (ReadingB_Strain1 - ReadingA_Strain1)) *
                       (newReading_Strain1 - ReadingA_Strain1) + LoadA_Strain1;

  startPos = analogRead(sensorPin); //store startting position in pot units
  t = millis();//store time
  //establish motor states variables and set to false
  boolean go = false;
  boolean push = false;
  boolean pull = false;
  boolean nudgePush = false;
  boolean nudgePull = false;
  /////// go button signal

  //if go button is pushed, set go to true
  if (digitalRead(buttonGo) == LOW) {
    go = true;
  }
  else {
    go = false;
  }

  //if push button is pushed, set push to true
  if (digitalRead(buttonPush) == LOW) {
    push = true;
  }
  else {
    push = false;
  }


  //if pull button is pushed, set pull to true
  if (digitalRead(buttonPull) == LOW) {
    pull = true;
  }
  else {
    pull = false;
  }

  ////************ evaluate button states and do what should be done ****************//////
  if ( pull == true) {
    pullActuatorSlow();
    printDat(t = millis(), state = "nudgepull", cycle = i);
  }

  if ( push == true) {
    //    //do the dialing
    pushActuatorSlow();
    printDat(t = millis(), state = "nudgepush", cycle = i);
    //estblish goal positions given position of pot

  }


  ////************ if go button is pushed, start cycles ****************//////

  boolean pressed = false;
  if (go == true) {
    startPos = analogRead(sensorPin); //set start position
    pressed = true; //go button is pressed
    stroke = per * (lenPot + (potMax - startPos));//stroke is now calculated
    goalPositionPull = startPos - stroke; //set goal position during retraction
    goalPositionPush = startPos; //set goal position during extension as start position
  }
  else {
    pressed = false;
  }

  CurrentPosition = analogRead(sensorPin);//store current position

  //make sure goal position isn't beyond motor limits
  if (goalPositionPull < potMin) {
    goalPositionPull = potMin + 20; //print a message
    //Serial.println("WARNING: strain exceeds retraction limit. Still gonna run, but no all the way");
  }

  //while the state of pressed is true, run experiments
  while (pressed == true) {
    CurrentPosition = analogRead(sensorPin);
    if (time < millis() + relax && i <= count) { //if i is less than max cycle number, go
      //if current time exceeds time since stop + relax time then push
      if (millis() > tstop + relax && tstop != 0) {
        //if current positon is not within tolerance, push
        if (CurrentPosition < goalPositionPush + tol || CurrentPosition < goalPositionPush - tol) {
          pushActuator();
          //take strain readings and do math
          float newReading_Strain1 = analogRead(0);
          float load_Strain1 = ((LoadB_Strain1 - LoadA_Strain1) / (ReadingB_Strain1 - ReadingA_Strain1)) *
                               (newReading_Strain1 - ReadingA_Strain1) + LoadA_Strain1;
          // print the results to the serial monitor:
          printDat(t = millis(), state = "push", cycle = i);
        }
        else
        {
          //else stop actuator
          stopActuator();
          time = millis();
          tstop = 0;
          i++;
        }
      }
      else {
        CurrentPosition = analogRead(sensorPin);
        //if current positon is not within pull tolerance, pull
        if (CurrentPosition > goalPositionPull + tol || CurrentPosition > goalPositionPull - tol) {
          pullActuator();
          tstop = millis();//last reading as motor is pulling will signal when motor stopped

          //take new reading and do math
          float newReading_Strain1 = analogRead(0);
          float load_Strain1 = ((LoadB_Strain1 - LoadA_Strain1) / (ReadingB_Strain1 - ReadingA_Strain1)) *
                               (newReading_Strain1 - ReadingA_Strain1) + LoadA_Strain1;
          // print the results to the serial monitor:
          printDat(t = millis(), state = "pull", cycle = i);
        }
        else
        {
          while ( millis() < tstop + relax) {  //if time is the time since stop + relax time then stop
            //take reading and do math
            float newReading_Strain1 = analogRead(0);
            float load_Strain1 = ((LoadB_Strain1 - LoadA_Strain1) / (ReadingB_Strain1 - ReadingA_Strain1)) *
                                 (newReading_Strain1 - ReadingA_Strain1) + LoadA_Strain1;
            stopActuator();
            // print the results to the serial monitor:
          }
        }
      }
    }
  }
  if (pressed == false) {
    stopActuator();//stop if pressed state if false
  }

} //void loop

/**
  This demo code is distributed with the hope that it will be useful to researchers and scientists;
  however, it is done so WITHOUT ANY WARRANTY; without even the implied warranty of
  FITNESS FOR A PARTICULAR PURPOSE.

  Should the user use this sketch and data from its use result in publication of her own work, please
  attribute the following:

  Kenaley, C. P.,  . . . . , 2017. Skin Stiffness in Ray-finned Fishes: Contrasting Material Properties
  Between Species and Body Regions. J. Morphology. XX:XX--XX

  [add bibtex record]
*/



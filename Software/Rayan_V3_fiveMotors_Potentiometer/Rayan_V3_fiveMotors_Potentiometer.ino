// Overall
#define SWITCHPIN 15
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 4000
int switchState;
const int maxPWM = 1000;
// int strength = 0; 0=weak, 1=medium, 2=strong

// Index and middle finger variables
#define AI1_INDEX 26
#define AI2_INDEX 25
#define BI1_MIDDLE 2
#define BI2_MIDDLE 0
#define PWM_IM 4
#define C1_IM 32
#define C2_IM 33
#define potentiometer 34

const int bufferSizeIM = 10; // Size of the moving average buffer, adjust as needed
float bufferIM[bufferSizeIM];
int bufferIndexIM = 0;
float sumIM = 0;
float movingAverageIM = 0;

int PWM_IM_val = 1000;
float voltage1IM;
float voltage2IM;
float currentIM;
bool thresholdReachedIM = 0;
// Three distinct current values represent the strength that the linear actuators will apply. It will never reach 999 since that exceeds the max current that can be sent to the
// actuators. We read the current being sent to the actuators.
// int currentThresholdIM[3] = {90, 130, 999}; // tailored to specific motor


// Represents the PWM that the motor controller sends to the linear actuators. Varies the amount of PWM that the motor controller sends to the linear actuators depending on
// the current strength selection
// int minPWM_IM[3] = {100, 500, maxPWM}; // tailored to specific motor



// Ring and pinky finger variables
#define AI1_RING 14
#define AI2_RING 27
#define BI1_PINKY 21
#define BI2_PINKY 22
#define PWM_RP 17
#define C1_RP 38
#define C2_RP 39

const int bufferSizeRP = 10; // Size of the moving average buffer, adjust as needed
float bufferRP[bufferSizeRP];
int bufferIndexRP = 0;
float sumRP = 0;
float movingAverageRP = 0;

int PWM_RP_val = 1000;
float voltage1RP;
float voltage2RP;
float currentRP;
bool thresholdReachedRP = 0;
// int currentThresholdRP[3] = {20, 60, 999}; //tailored to specific motor
// int minPWM_RP[3] = {100, 200, maxPWM}; //tailored to specific motor


// Thumb variables
#define AI1_THUMB 23
#define AI2_THUMB 19
#define PWM_T 16
#define C1_T 36
#define C2_T 37

const int bufferSizeT = 10; // Size of the moving average buffer, adjust as needed
float bufferT[bufferSizeT];
int bufferIndexT = 0;
float sumT = 0;
float movingAverageT = 0;

int PWM_T_val = 1000;
float voltage1T;
float voltage2T;
float currentT;
bool thresholdReachedT = 0;
// int currentThresholdT[3] = {100, 160, 999}; //tailored to specific motor
// int minPWM_T[3] = {100, 700, maxPWM}; //tailored to specific motor

//--------------------------------------------------------------------------------
void setup() {
  // initialize digital pins as outputs.
  pinMode(SWITCHPIN, INPUT);

  pinMode(PWM_IM, OUTPUT);
  pinMode(PWM_RP, OUTPUT);
  pinMode(PWM_T, OUTPUT);

  pinMode(AI1_INDEX, OUTPUT);
  pinMode(AI2_INDEX, OUTPUT);
  pinMode(BI1_MIDDLE, OUTPUT);
  pinMode(BI2_MIDDLE, OUTPUT);
  pinMode(AI1_RING, OUTPUT);
  pinMode(AI2_RING, OUTPUT);
  pinMode(BI1_PINKY, OUTPUT);
  pinMode(BI2_PINKY, OUTPUT);
  pinMode(AI1_THUMB, OUTPUT);
  pinMode(AI2_THUMB, OUTPUT);

  pinMode(C1_IM, INPUT);
  pinMode(C2_IM, INPUT);
  pinMode(C1_RP, INPUT);
  pinMode(C2_RP, INPUT);
  pinMode(C1_T, INPUT);
  pinMode(C2_T, INPUT);
  pinMode(potentiometer, INPUT);

  // Setup PWMs
  ledcAttach(PWM_IM, PWM_FREQUENCY, PWM_RESOLUTION); // I changed this from the previous version so suspect error here
  ledcAttach(PWM_RP, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWM_T, PWM_FREQUENCY, PWM_RESOLUTION);

  // Initialize boxcars for current sensing
  for (int i = 0; i < bufferSizeIM; i++) {
    bufferIM[i] = 0;
  }
  for (int i = 0; i < bufferSizeRP; i++) {
    bufferRP[i] = 0;
  }
  for (int i = 0; i < bufferSizeT; i++) {
    bufferT[i] = 0;
  }

  // For plotting current sensor
  Serial.begin(115200);

}

// the loop function runs over and over again forever
void loop() {
  
  int val = analogRead(potentiometer);
  int contPWM = map(val, 0, 1023, 0, 255);
  PWM_IM_val = contPWM;
  PWM_RP_val = contPWM;
  PWM_T_val = contPWM;

  ledcWrite(PWM_IM, PWM_IM_val);
  ledcWrite(PWM_RP, PWM_RP_val);
  ledcWrite(PWM_T, PWM_T_val);
  
  switchState = digitalRead(SWITCHPIN);

  if(switchState == 1) {
    // move linear actuators to close hand
    digitalWrite(AI1_INDEX, HIGH);
    digitalWrite(AI2_INDEX, LOW);
    digitalWrite(BI1_MIDDLE, HIGH);
    digitalWrite(BI2_MIDDLE, LOW);
    digitalWrite(AI1_RING, HIGH);
    digitalWrite(AI2_RING, LOW);
    digitalWrite(BI1_PINKY, HIGH);
    digitalWrite(BI2_PINKY, LOW);
    digitalWrite(AI1_THUMB, HIGH);
    digitalWrite(AI2_THUMB, LOW);

    // decrease power to linear actuator if current threshold reached
    if(thresholdReachedIM){
      PWM_IM_val = 0;
    }
    if(thresholdReachedRP){
      PWM_RP_val =  0;
    }
    if(thresholdReachedT){
      PWM_T_val = 0;
    }
    

  } // actuator will stop extending automatically if the limit is reached
  else{
    // move linear actuators to open hand
    digitalWrite(AI1_INDEX, LOW);
    digitalWrite(AI2_INDEX, HIGH);
    digitalWrite(BI1_MIDDLE, LOW);
    digitalWrite(BI2_MIDDLE, HIGH);
    digitalWrite(AI1_RING, LOW);
    digitalWrite(AI2_RING, HIGH);
    digitalWrite(BI1_PINKY, LOW);
    digitalWrite(BI2_PINKY, HIGH);
    digitalWrite(AI1_THUMB, LOW);
    digitalWrite(AI2_THUMB, HIGH);

    // reset current threshold flag and power
    thresholdReachedIM = 0;
    thresholdReachedRP = 0;
    thresholdReachedT = 0;
    
    PWM_IM_val = contPWM;
    PWM_RP_val = contPWM;
    PWM_T_val  = contPWM;

  } // actuator will stop extending automatically when reaching the limit

  // Current sensor with boxcar average for index and middle finger
  voltage1IM = analogRead(C1_IM);
  voltage2IM = analogRead(C2_IM);
  currentIM = voltage2IM - voltage1IM;
  sumIM -= bufferIM[bufferIndexIM];
  bufferIM[bufferIndexIM] = currentIM;
  sumIM += bufferIM[bufferIndexIM];
  bufferIndexIM = (bufferIndexIM + 1) % bufferSizeIM;
  movingAverageIM = sumIM / bufferSizeIM;
  if(movingAverageIM > PWM_IM_val){
    thresholdReachedIM = 1;
  }

  // Current sensor with boxcar average for ring and pinky finger
  voltage1RP = analogRead(C1_RP);
  voltage2RP = analogRead(C2_RP);
  currentRP = voltage2RP - voltage1RP;
  sumRP -= bufferRP[bufferIndexRP];
  bufferRP[bufferIndexRP] = currentRP;
  sumRP += bufferRP[bufferIndexRP];
  bufferIndexRP = (bufferIndexRP + 1) % bufferSizeRP;
  movingAverageRP = sumRP / bufferSizeRP;
  if(movingAverageRP > PWM_RP_val){
    thresholdReachedRP = 1;
  }

  // Current sensor with boxcar average for thumb
  // Reads voltage from both pins in third motor controller.
  voltage1T = analogRead(C1_T);
  voltage2T = analogRead(C2_T);

  // Takes potential difference between both pins using V = IR. Resistor is 1 ohm so it is a 1:1 relationship (Check more on this)
  currentT = voltage2T - voltage1T;

  // Uses boxcar averaging to smoothen the noisy potential difference values
  sumT -= bufferT[bufferIndexT];
  bufferT[bufferIndexT] = currentT;
  sumT += bufferT[bufferIndexT];
  bufferIndexT = (bufferIndexT + 1) % bufferSizeT;
  movingAverageT = sumT / bufferSizeT;

  // If the deteced current value is greater than the selected current value, then the detected current value has reached its threshold value. It will tell the motor controller to
  // stop sending PWM at this point (no more current is sent to the linear actuators).
  if(movingAverageT > PWM_T_val){
    thresholdReachedT = 1;
  }

  delay(100);

  Serial.print(thresholdReachedRP*500);
  Serial.print(",");
  Serial.print(PWM_RP_val);
  Serial.print(",");
  Serial.print(movingAverageT*10);
  Serial.print(",");
  Serial.println(switchState*300);

  delay(10);
}

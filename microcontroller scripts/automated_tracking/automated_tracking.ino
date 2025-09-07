#include <ArduinoJson.h>

unsigned long previousMicrosForX = 0;  
long intervalx = 1;                // interval at which to blink (microseconds)
unsigned long previousMicrosForY = 0;  
long intervaly = 1;                
unsigned long currentMicrosX;
unsigned long currentMicrosY;

JsonDocument doc;

const int MOTORX_STEP_PIN = 23;
const int MOTORX_DIRECTION_PIN = 32;
const int MOTORX_MS1_PIN = 13;
const int MOTORX_MS2_PIN = 12;
const int MOTORX_MS3_PIN = 14;

const int MOTORY_STEP_PIN = 25;
const int MOTORY_DIRECTION_PIN = 33;
const int MOTORY_MS1_PIN = 22;
const int MOTORY_MS2_PIN = 16;
const int MOTORY_MS3_PIN = 17;

const int LASER_PIN = 18;

int x_vel = 0;
int y_vel = 0;
int availableBytes = 0;
int x_dir=0;
int y_dir=0;
int laser=0;

char test[128];


void setup() {
  Serial.begin(921600);

  //X stepper
  pinMode(MOTORX_STEP_PIN, OUTPUT);
  pinMode(MOTORX_DIRECTION_PIN, OUTPUT);

  pinMode(MOTORX_MS1_PIN, OUTPUT);
  pinMode(MOTORX_MS2_PIN, OUTPUT);
  pinMode(MOTORX_MS2_PIN, OUTPUT);

  digitalWrite(MOTORX_MS1_PIN, HIGH);
  digitalWrite(MOTORX_MS2_PIN, HIGH);
  digitalWrite(MOTORX_MS3_PIN, HIGH);


  //Y stepper
  pinMode(MOTORY_STEP_PIN, OUTPUT);
  pinMode(MOTORY_DIRECTION_PIN, OUTPUT);
  
  pinMode(MOTORY_MS1_PIN, OUTPUT);
  pinMode(MOTORY_MS2_PIN, OUTPUT);
  pinMode(MOTORY_MS2_PIN, OUTPUT);

  digitalWrite(MOTORY_MS1_PIN, HIGH);
  digitalWrite(MOTORY_MS2_PIN, HIGH);
  digitalWrite(MOTORY_MS3_PIN, HIGH);
  

  pinMode(LASER_PIN, OUTPUT); //Laser
}

void loop() {
  handleCommunication();

  if(laser==1){
    digitalWrite(LASER_PIN, HIGH);
  }else{
    digitalWrite(LASER_PIN, LOW);
  
    currentMicrosX = micros();
    if (x_vel != 0) {
      intervalx = 100 / x_vel;
      if (currentMicrosX - previousMicrosForX >= intervalx) {
        previousMicrosForX = currentMicrosX;
        if (x_dir == 0) {
          digitalWrite(MOTORX_DIRECTION_PIN, LOW);
        } else {
        digitalWrite(MOTORX_DIRECTION_PIN, HIGH);
          
        }
        digitalWrite(MOTORX_STEP_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(MOTORX_STEP_PIN, LOW);
      }
    }


    currentMicrosY = micros();
    if (y_vel != 0) {
      intervaly = 100 / y_vel;
      if (currentMicrosY - previousMicrosForY >= intervaly) {
        previousMicrosForY = currentMicrosY;
        if (y_dir == 0) {
          digitalWrite(MOTORY_DIRECTION_PIN, LOW);   
          
        } else {
          digitalWrite(MOTORY_DIRECTION_PIN, HIGH);
        
        }
        digitalWrite(MOTORY_STEP_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(MOTORY_STEP_PIN, LOW);
      }
    }
  }
}

void handleCommunication() {
  availableBytes = Serial.available();
  if (availableBytes>0){
    for(int i=0; i<availableBytes; i++)
    {
        test[i] = Serial.read();
        test[i+1] = '\0'; // Append a null
    }
  }

  DeserializationError error = deserializeJson(doc, test);

  // Test if parsing succeeds.
  if (error) {
    // Serial.print(F("deserializeJson() failed: "));
    // Serial.println(error.f_str());
    return;
  } else {
    x_vel = doc["vx"];
    y_vel = doc["vy"];
    y_dir = doc["dy"];
    x_dir = doc["dx"];
    laser = doc["laser"];
  }
}

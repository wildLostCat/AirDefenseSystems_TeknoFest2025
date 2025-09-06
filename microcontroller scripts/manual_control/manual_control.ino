const int MOTORX_STEP_PIN = 23;
const int MOTORX_DIRECTION_PIN = 32;
const int MOTORX_MS1_PIN = 13;
const int MOTORX_MS2_PIN = 12;
const int MOTORX_MS3_PIN = 14;


const int MOTORY_DIRECTION_PIN = 33;
const int MOTORY_STEP_PIN = 25;
const int MOTORY_MS1_PIN = 22;
const int MOTORY_MS2_PIN = 16;
const int MOTORY_MS3_PIN = 17;

const int LAZER_PIN = 18;

const int JOYSTICK_X_PIN = 4;
const int JOYSTICK_Y_PIN = 2;

#define DEADZONE 500
#define JOY_CENTER 2048
#define SABIT_HIZ_INTERVAL (1/1000 * 100)
#define SABIT_HIZ_INTERVAL_Y 1

unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
unsigned long currentMillis = 0;

bool hareketX = false;
bool hareketY = false;
bool moving_left_X=false;


long turn_angle=0; 

void setup() {
  Serial.begin(115200);

  pinMode(MOTORX_STEP_PIN, OUTPUT);
  pinMode(MOTORX_DIRECTION_PIN, OUTPUT);

  pinMode(MOTORX_MS1_PIN, OUTPUT);
  pinMode(MOTORX_MS2_PIN, OUTPUT);
  pinMode(MOTORX_MS2_PIN, OUTPUT);

  digitalWrite(MOTORX_MS1_PIN, HIGH);
  digitalWrite(MOTORX_MS2_PIN, HIGH);
  digitalWrite(MOTORX_MS3_PIN, HIGH);
  
  // Y stepper
  pinMode(MOTORY_STEP_PIN, OUTPUT);
  pinMode(MOTORY_DIRECTION_PIN, OUTPUT);
  
  pinMode(MOTORY_MS1_PIN, OUTPUT);
  pinMode(MOTORY_MS2_PIN, OUTPUT);
  pinMode(MOTORY_MS2_PIN, OUTPUT);

  digitalWrite(MOTORY_MS1_PIN, HIGH);
  digitalWrite(MOTORY_MS2_PIN, HIGH);
  digitalWrite(MOTORY_MS3_PIN, HIGH);
  

  pinMode(LAZER_PIN, OUTPUT);
  pinMode(19, INPUT);
}

void loop() {
  if (digitalRead(19)){

  currentMillis = millis();
  
  int joyX = analogRead(JOYSTICK_X_PIN);
  int joyY = analogRead(JOYSTICK_Y_PIN);

  // X yön ve hareket kontrolü
  if (joyX > JOY_CENTER + DEADZONE) {
    digitalWrite(MOTORX_DIRECTION_PIN, LOW);  // sağ
    hareketX = true;
    moving_left_X=false;
  } else if (joyX < JOY_CENTER - DEADZONE) {
    digitalWrite(MOTORX_DIRECTION_PIN, HIGH); // sol
    hareketX = true;
    moving_left_X=true;

  } else {
    hareketX = false;
  }

  // Y yön ve hareket kontrolü
  if (joyY > JOY_CENTER + DEADZONE) {
     digitalWrite(MOTORY_DIRECTION_PIN, HIGH); // aşağı

    hareketY = true;
  } else if (joyY < JOY_CENTER - DEADZONE) {
    digitalWrite(MOTORY_DIRECTION_PIN, LOW);  // yukarı

    
    hareketY = true;
  } else {
    hareketY = false;
  }

  // X adım atma
  if (hareketX && currentMillis - previousMillisX >= SABIT_HIZ_INTERVAL) {
    previousMillisX = currentMillis;
    digitalWrite(MOTORX_STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(MOTORX_STEP_PIN, LOW);
    if (moving_left_X){
      turn_angle+=0.2;
    }
    else{
      turn_angle-=0.2;
    }
  }

  // Y adım atma
  if (hareketY && currentMillis - previousMillisY >= SABIT_HIZ_INTERVAL_Y ) {
    previousMillisY = currentMillis;
    digitalWrite(MOTORY_STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(MOTORY_STEP_PIN, LOW);

  }
  }
  // Lazer: herhangi bir motor hareket ediyorsa aç
  if(turn_angle>15 || turn_angle<-15){
  digitalWrite(LAZER_PIN, digitalRead(19) ? LOW : HIGH);
  }
}

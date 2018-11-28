#define LED_PWM_PIN 6
#define SENSOR_PIN 12

#define FADING_TIME 3 // seconds
#define TURNED_OFF 0
#define LIGHTUP 1
#define FADE 2
#define TURNED_ON 3

#define DEBUG

int sensorVal = 0;
int fadingDelta = FADING_TIME * 1000 / 255;
int fadingVal = 0;
int ledState = TURNED_OFF;

unsigned long timeCheckpoint = 0;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(SENSOR_PIN, INPUT);

  pinMode(LED_PWM_PIN, OUTPUT);
  analogWrite(LED_PWM_PIN, 0);   // turn the LED on

  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Kitchen LED started");    
  #endif
}

void loop() {
  sensorVal = digitalRead(SENSOR_PIN);
  if (sensorVal == HIGH && (ledState == TURNED_OFF || ledState == FADE) && !(ledState == LIGHTUP || ledState == TURNED_ON)) {
    ledState = LIGHTUP;
    //timeCheckpoint = millis();
  }
  else if (sensorVal == LOW && (ledState == TURNED_ON || ledState == LIGHTUP) && !(ledState == FADE || ledState == TURNED_OFF)) {
    ledState = FADE;
    //timeCheckpoint = millis();
  }

  delay_led();
   
}

void delay_led() {
  unsigned long time = millis();
  if (time - timeCheckpoint > 1) {
    int delta = 1;//(time - timeCheckpoint) / 1 * fadingDelta;
    control_led(delta);

    #ifdef DEBUG
    Serial.println("---");
      Serial.println(fadingVal, DEC);
      Serial.println(time, DEC);
      Serial.println(timeCheckpoint, DEC);
      Serial.println(delta, DEC);
    #endif
    timeCheckpoint = millis();    
  }

  #ifdef DEBUG
    //Serial.println(time - timeCheckpoint, DEC);
  #endif
}

void control_led(int delta) {
  if (ledState == TURNED_OFF) analogWrite(LED_PWM_PIN, 0);
  else if (ledState == TURNED_ON) analogWrite(LED_PWM_PIN, 255);
  else if (ledState == LIGHTUP) {
    fadingVal += delta;
    if (fadingVal >= 255) {
      fadingVal = 255;
      ledState = TURNED_ON;
    }
    analogWrite(LED_PWM_PIN, fadingVal);
  }
  else if (ledState == FADE) {
    fadingVal -= delta;
    if (fadingVal <= 0) {
      fadingVal = 0;
      ledState = TURNED_OFF;
    }
    analogWrite(LED_PWM_PIN, fadingVal);
  }
}


#define LEFT_BTN_PIN 2
#define RIGHT_BTN_PIN 3
#define LEFT_OUT_PIN 4
#define RIGHT_OUT_PIN 5
#define WHEEL_PIN 6
#define POT_PIN A0
#define THRESHOLD 100


const double max_speed = 170.0;

volatile bool left_interrupt_request = false;
volatile unsigned long left_interrupt_time;
volatile bool right_interrupt_request = false;
volatile unsigned long right_interrupt_time;

unsigned long int wheel_last_update;
unsigned long int indicators_value_last_update;

volatile bool left_indicator;
volatile bool right_indicator;
volatile bool wheel;
volatile bool indicators_value;

double wheel_diameter = 1.9;


void leftInterruptHandler() {
  //the interrupt is disabled for some time after its execution for debouncing
  if (!left_interrupt_request) {
    left_interrupt_request = true;
    left_interrupt_time = millis();
  }
}

void rightInterruptHandler() {
  //the interrupt is disabled for some time after its execution for debouncing
  if (!right_interrupt_request) {
    right_interrupt_request = true;
    right_interrupt_time = millis();
  }
}

void setup() {
  //set pin directions
  pinMode(WHEEL_PIN, OUTPUT);
  pinMode(LEFT_OUT_PIN, OUTPUT);
  pinMode(RIGHT_OUT_PIN, OUTPUT);
  pinMode(LEFT_BTN_PIN, INPUT);
  pinMode(RIGHT_BTN_PIN, INPUT);
  pinMode(POT_PIN, INPUT);

  //attach interrupts for the indicators buttons
  attachInterrupt(digitalPinToInterrupt(LEFT_BTN_PIN), leftInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BTN_PIN), rightInterruptHandler, RISING);

  //initialize time variables
  wheel_last_update = millis();
  indicators_value_last_update = millis();
}

void loop() {
  //toggle the global indicators flsg every 2 seconds
  if (millis() - indicators_value_last_update > 2000) {
    indicators_value_last_update = millis();
    indicators_value = !indicators_value;
  }

  //handle indicators interrupts requests
  if (left_interrupt_request && (millis() - left_interrupt_time) > 100) {
    left_interrupt_request = false;
    left_indicator = !left_indicator;
  }
  if (right_interrupt_request && (millis() - right_interrupt_time) > 100) {
    right_interrupt_request = false;
    right_indicator = !right_indicator;
  }

  //turn the indicator on only if the global indicator flag and the corresponding indicator state are true
  digitalWrite(LEFT_OUT_PIN, left_indicator && indicators_value);
  digitalWrite(RIGHT_OUT_PIN, right_indicator && indicators_value);

  //read the analog input from the potentiometer and calculate the corresponding speed
  float vel = 0.0;
  int analog_read = analogRead(POT_PIN);
  if (analog_read > THRESHOLD) {
    vel = (float)(analog_read - THRESHOLD) / (1023 - THRESHOLD) * max_speed;
  }

  //calculate the wheel rotation period corresponding with the speed and toggle the output pin accordingly
  int period = 1000 * wheel_diameter / (vel / 3.6);
  if (vel > 0.0) {
    if (millis() - wheel_last_update > period) {
      wheel_last_update = millis();
      digitalWrite(WHEEL_PIN, HIGH);
    } else if (millis() - wheel_last_update > period / 2) {
      digitalWrite(WHEEL_PIN, LOW);
    }
  }

  delay(1);
}

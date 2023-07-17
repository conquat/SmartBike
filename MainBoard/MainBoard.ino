#include "Arduino.h"
#include "LoRa_E220.h"
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

#define BUFFERSIZE 3
#define MOVEMENT_INTERRUPT_DELAY 10000
#define SEND_ALARM_WAIT_TIME 3000

#define POWER_PIN 18
#define MOVEMENT_PIN 19
#define WHEEL_PIN 20
#define SCREEN_PWR_PIN 24
#define LORA_PWR_PIN 22
#define LORA_TX_PIN 26
#define LORA_RX_PIN 28
#define SCREEN_TX_PIN 30
#define SCREEN_RX_PIN 32
#define LEFT_INDICATOR_PIN 34
#define RIGHT_INDICATOR_PIN 36


volatile bool powerInterruptRequest;
volatile unsigned long powerInterruptTime;

volatile unsigned long lastTimeDataSent;

const float wheelDiameter = 1.9;
volatile unsigned long circularBuffer[BUFFERSIZE];
volatile uint8_t a, b;
volatile bool enterWheelInterrupt, wheelInterruptEnable, wheelInterruptRequest;

volatile bool movement_detected;
volatile unsigned int entered_wait_to_go_sleep, entered_wait_to_send_alarm, entered_wait_to_turn_lora_off;

typedef enum {
  SCREEN_ON_INIT,//1
  SCREEN_ON,//2
  WAIT_TO_GO_SLEEP_INIT,//3
  WAIT_TO_GO_SLEEP,//4
  SLEEP,//5
  WAIT_TO_SEND_ALARM,//6
  WAIT_TO_TURN_LORA_OFF,//7
  NUM_STATES
} State_t;

typedef struct {
  State_t state;
  void (*func)(void);
} StateMachine_t;

/* state machine function prototypes */
void fn_ScreenOnInit(void);
void fn_ScreenOn(void);
void fn_WaitToGoSleepInit(void);
void fn_WaitToGoSleep(void);
void fn_Sleep(void);
void fn_WaitToSendAlarm(void);
void fn_WaitToTurnLoraOff(void);

/* variable that holds the current state */
volatile State_t cur_state = NUM_STATES;
StateMachine_t fsm[] = {
  { SCREEN_ON_INIT, fn_ScreenOnInit },
  { SCREEN_ON, fn_ScreenOn },
  { WAIT_TO_GO_SLEEP_INIT, fn_WaitToGoSleepInit },
  { WAIT_TO_GO_SLEEP, fn_WaitToGoSleep },
  { SLEEP, fn_Sleep },
  { WAIT_TO_SEND_ALARM, fn_WaitToSendAlarm },
  { WAIT_TO_TURN_LORA_OFF, fn_WaitToTurnLoraOff }
};
volatile bool screen_on_init_request = false;
volatile bool wait_to_go_sleep_init_request = false;

//update the state machine
void run(void) {
  if (cur_state < NUM_STATES) {
    (*fsm[cur_state].func)();
  } else {
    DEBUG_PRINTLN("INVALID STATE");
    while (1) {
      delay(1000);
    }
  }
}

SoftwareSerial lora(LORA_TX_PIN, LORA_RX_PIN);
SoftwareSerial screen(SCREEN_TX_PIN, SCREEN_RX_PIN);

//updates the buffer start and end indeces and stores the current micros() value in the last position
void updateBuffer() {
  a = (a + 1) % BUFFERSIZE;
  b = (b + 1) % BUFFERSIZE;
  circularBuffer[b] = micros();
  enterWheelInterrupt = true;
}

//calculates wheel speed, checks indicators state and send the informations to the screen in JSON format
void sendData() {
  float vel;
  //momentarily disables the circular buffer updates while reading from it
  wheelInterruptEnable = false;
  unsigned long timeLow = circularBuffer[a];
  unsigned long timeHigh = circularBuffer[b];
  //if the interrupt requested an update while we were reading from it, perform the update
  if (wheelInterruptRequest) {
    wheelInterruptRequest = false;
    updateBuffer();
  }
  wheelInterruptEnable = true;

  //calculate the wheel speed
  if ((millis() - timeHigh / 1000) > (timeHigh - timeLow) / 1000) {
    vel = 0.0;
  } else {
    float time = (timeHigh - timeLow) / (BUFFERSIZE - 1) / 1000000.0;
    vel = wheelDiameter / time * 3.6;
  }
  //check indicators values
  bool left_indicator = digitalRead(LEFT_INDICATOR_PIN);
  bool right_indicator = digitalRead(RIGHT_INDICATOR_PIN);

  //prepare the JSON message and send it to the scrren via UART
  String message = (String) "{\"vel\":" + vel + ", \"right\":" + right_indicator + ", \"left\":" + left_indicator + "}";
  DEBUG_PRINT("message: ");
  DEBUG_PRINTLN(message);
  screen.println(message);
}

void sendAlarm() {
  lora.write(1);
  DEBUG_PRINTLN("Alarm sent!");
}

//callback for the wheel sensor interrupt
void wheelInterruptHandler() {
  //only update the buffer if the previous update has finished and we are not reading from it
  if (enterWheelInterrupt) {
    enterWheelInterrupt = false;
    if (wheelInterruptEnable) {
      updateBuffer();
    } else {
      //if we are reading from the buffer set a flag
      wheelInterruptRequest = true;
    }
  }
}

//callback for the movement sensor interrupt
void movementInterruptHandler() {
  movement_detected = true;
}

//callback for the switch interrupt
void powerInterruptHandler() {
  if (!powerInterruptRequest) {
    powerInterruptRequest = true;
    powerInterruptTime = millis();
  }
}

//function to execute when the state machine is in the SCREEN_ON_INIT state
void fn_ScreenOnInit() {
  screen_on_init_request = false;
  //turn the screen on
  digitalWrite(SCREEN_PWR_PIN, HIGH);
  digitalWrite(LORA_PWR_PIN, LOW);

  lastTimeDataSent = millis();
  enterWheelInterrupt = true;
  wheelInterruptEnable = true;
  wheelInterruptRequest = false;

  //update interrupt settings
  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelInterruptHandler, RISING);
  detachInterrupt(digitalPinToInterrupt(MOVEMENT_PIN));

  //initialize the buffer
  for (int i = 0; i < BUFFERSIZE; ++i) {
    circularBuffer[i] = 0;
  }
  a = 0;
  b = BUFFERSIZE - 1;

  //enter SCREEN_ON state
  cur_state = SCREEN_ON;
  DEBUG_PRINT("new state: ");
  DEBUG_PRINTLN(cur_state);
}

//function to execute when the state machine is in the SCREEN_ON state
void fn_ScreenOn() {
  //send data to the screen every 1100ms
  if (millis() - lastTimeDataSent > 1100) {
    sendData();
    lastTimeDataSent = millis();
  }

  if (screen_on_init_request) {
    cur_state = SCREEN_ON_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  } else if (wait_to_go_sleep_init_request) {
    cur_state = WAIT_TO_GO_SLEEP_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  }
}

//function to execute when the state machine is in the WAIT_TO_GO_SLEEP_INIT state
void fn_WaitToGoSleepInit() {
  wait_to_go_sleep_init_request = false;
  //turn the screen and lora off
  digitalWrite(SCREEN_PWR_PIN, LOW);
  digitalWrite(LORA_PWR_PIN, LOW);

  //update interrupt settings
  detachInterrupt(digitalPinToInterrupt(WHEEL_PIN));
  attachInterrupt(digitalPinToInterrupt(MOVEMENT_PIN), movementInterruptHandler, RISING);

  //enter WAIT_TO_GO_SLEEP state
  entered_wait_to_go_sleep = millis();
  cur_state = WAIT_TO_GO_SLEEP;
  DEBUG_PRINT("new state: ");
  DEBUG_PRINTLN(cur_state);
}

//function to execute when the state machine is in the WAIT_TO_GO_SLEEP state
void fn_WaitToGoSleep(void) {
  if (screen_on_init_request) {
    cur_state = SCREEN_ON_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  } else if (wait_to_go_sleep_init_request) {
    cur_state = WAIT_TO_GO_SLEEP_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  } else if (millis() - entered_wait_to_go_sleep > MOVEMENT_INTERRUPT_DELAY) { //after MOVEMENT_INTERRUPT_DELAY, enter SLEEP state and set the microcontroller in power down mode
    movement_detected = false;
    cur_state = SLEEP;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
    sleep_mode();
  }
}

//function to execute when the state machine is in the SLEEP state
void fn_Sleep(void) {
  if (screen_on_init_request) {
    cur_state = SCREEN_ON_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  } else if (wait_to_go_sleep_init_request) {
    cur_state = WAIT_TO_GO_SLEEP_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  } else if (movement_detected) { //when a movement is detected in the SLEEP state, turn LORA on and enter the WAIT_TO_SEND_ALARM state
    digitalWrite(LORA_PWR_PIN, HIGH);
    entered_wait_to_send_alarm = millis();
    cur_state = WAIT_TO_SEND_ALARM;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  }
}

//function to execute when the state machine is in the WAIT_TO_SEND_ALARM state
void fn_WaitToSendAlarm(void) {
  if (screen_on_init_request) {
    digitalWrite(LORA_PWR_PIN, LOW);
    cur_state = SCREEN_ON_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  } else if (wait_to_go_sleep_init_request) {
    digitalWrite(LORA_PWR_PIN, LOW);
    cur_state = WAIT_TO_GO_SLEEP_INIT;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  } else if (millis() - entered_wait_to_send_alarm > SEND_ALARM_WAIT_TIME) { //after SEND_ALARM_WAIT_TIME, send the alarm and enter WAIT_TO_TURN_LORA_OFF state
    sendAlarm();
    entered_wait_to_turn_lora_off = millis();
    cur_state = WAIT_TO_TURN_LORA_OFF;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
  }
}

//function to execute when the state machine is in the WAIT_TO_TURN_LORA_OFF state
void fn_WaitToTurnLoraOff(void) {
  //after 2s, turn lora off and transition to the SLEEP state
  if (millis() - entered_wait_to_turn_lora_off > 2000) {
    digitalWrite(LORA_PWR_PIN, LOW);
    movement_detected = false;
    cur_state = SLEEP;
    DEBUG_PRINT("new state: ");
    DEBUG_PRINTLN(cur_state);
    sleep_mode();
  }
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  lora.begin(9600);
  screen.begin(9600);

  pinMode(POWER_PIN, INPUT);
  pinMode(MOVEMENT_PIN, INPUT);
  pinMode(WHEEL_PIN, INPUT);
  pinMode(SCREEN_PWR_PIN, OUTPUT);
  pinMode(LORA_PWR_PIN, OUTPUT);
  pinMode(LEFT_INDICATOR_PIN, INPUT);
  pinMode(RIGHT_INDICATOR_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //turn the screen and LORA off
  digitalWrite(SCREEN_PWR_PIN, LOW);
  digitalWrite(LORA_PWR_PIN, LOW);

  //configure the sleep mode
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();

  //attach the interrupt for the power pin
  attachInterrupt(digitalPinToInterrupt(POWER_PIN), powerInterruptHandler, CHANGE);

  //initialize the state machine in the correct state
  if (digitalRead(POWER_PIN)) {
    cur_state = SCREEN_ON_INIT;
  } else {
    cur_state = WAIT_TO_GO_SLEEP_INIT;
  }
  DEBUG_PRINT("setup state: ");
  DEBUG_PRINTLN(cur_state);

  delay(500);
}

void loop() {
  //update the power flags and debounce the signal
  if (powerInterruptRequest && (millis() - powerInterruptTime) > 10) {
    bool powerPinState = digitalRead(POWER_PIN);
    powerInterruptRequest = false;   
    if (powerPinState) {
      screen_on_init_request = true;
    } else {
      wait_to_go_sleep_init_request = true;
    }
  }

  //update the state machine
  run();

  //toggle an LED to see when the microcontroller is not sleeping
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
}

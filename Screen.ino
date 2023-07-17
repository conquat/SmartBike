#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <UTFTGLUE.h>

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define PORTRAIT 0
#define LANDSCAPE 1
#define LEFT 0
#define RIGHT 9999
#define CENTER 9998
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320

UTFTGLUE myGLCD(0x9486, A3, A2, A1, A0, A4);
MCUFRIEND_kbv tft;

int vel = 0;
int last_velocity = vel;
bool right = false, left = false;
volatile bool updateScreenEnable = false;


//this function sets the screen cursor in the position for the text we want to write to be centered
void settextcursor(char *v, int x, int y, int pad = 0) {
  int16_t pos, x1, y1;
  uint16_t len, w, h;
  tft.getTextBounds(v, 0, 0, &x1, &y1, &w, &h); //gets the dimensions the text wll have on the screen
  len = x1 + w + 0;
  if (pad >= len) pad = pad - len;
  pos = (SCREEN_WIDTH - len - pad);
  if (x == CENTER) x = pos / 2;
  else if (x == RIGHT) x = pos - 1;
  if (strlen(v) == 3 && v[0] == '1') {
    x = x - 43;
  }
  //set the cursor in the calculated position
  tft.setCursor(x + pad, y);
}

void updateVelocity(int v) {
  //set the cursor in the correct position to overwrite the previous velocity
  settextcursor((char *)(String(last_velocity).c_str()), CENTER, 280);
  //overwrite the previous velocity with white text
  tft.setTextColor(WHITE, WHITE);
  tft.println(String(last_velocity));
  last_velocity = v;

  tft.setTextSize(4);
  //set the cursor in the correct position to write the new velocity
  settextcursor((char *)(String(v).c_str()), CENTER, 280);
  //write the new velocity
  tft.setTextColor(BLACK, WHITE);
  tft.println(String(v));
}

//draws the left indicator in the desired color
void drawLeftIndicator(uint16_t color) {
  tft.fillTriangle(50 - 1, 10 + 1, 50 - 1, 50 - 1, 10 + 1, 30, color);
  tft.fillRect(50 - 1, 20 + 1, 40, 20 - 2, color);
}

//draws the right indicator in the desired color
void drawRightIndicator(uint16_t color) {
  tft.fillTriangle(SCREEN_WIDTH - 50 + 1, 10 + 1, SCREEN_WIDTH - 50 + 1, 50 - 1, SCREEN_WIDTH - 10 - 1, 30, color);
  tft.fillRect(SCREEN_WIDTH - 50 - 40 + 1 + 1, 20 + 1, 40, 20 - 2, color);
}

//update the image shown on the screen
void updateScreen() {
  int v = vel;
  //write the new velocity
  updateVelocity(v);

  //draw the indicators
  if (left) {
    drawLeftIndicator(GREEN);
  } else {
    drawLeftIndicator(WHITE);
  }
  if (right) {
    drawRightIndicator(GREEN);
  } else {
    drawRightIndicator(WHITE);
  }
}

//timer that periodically enables screen update flag
ISR(TIMER1_OVF_vect) {
  updateScreenEnable = true;
}

void setup() {
  //configure the timer1 to execute ISR roughly every second
  TCCR1A = 0;
  TCCR1B = 0b00000100;
  TCNT1 = 0;
  TIMSK1 = 0b00000001;

  Serial.begin(9600);

  //initialize the screen
  tft.reset();
  uint16_t identifier = tft.readID();
  tft.begin(identifier);
  tft.setRotation(LANDSCAPE);
  tft.setFont(SevenSegNumFont);
  tft.fillScreen(WHITE);

  //draw indicators borders on the scren
  tft.drawTriangle(50, 10, 50, 50, 10, 30, BLACK);
  tft.drawTriangle(SCREEN_WIDTH - 50, 10, SCREEN_WIDTH - 50, 50, SCREEN_WIDTH - 10, 30, BLACK);
  tft.drawRect(50, 20, 40, 20, BLACK);
  tft.drawRect(SCREEN_WIDTH - 50 - 40 + 1, 20, 40, 20, BLACK);
  tft.fillTriangle(50 - 1, 10 + 1, 50 - 1, 50 - 1, 10 + 1, 30, WHITE);
  tft.fillTriangle(SCREEN_WIDTH - 50 + 1, 10 + 1, SCREEN_WIDTH - 50 + 1, 50 - 1, SCREEN_WIDTH - 10 - 1, 30, WHITE);
  tft.fillRect(50 - 1, 20 + 1, 40, 20 - 2, WHITE);
  tft.fillRect(SCREEN_WIDTH - 50 - 40 + 1 + 1, 20 + 1, 40, 20 - 2, WHITE);
  updateScreen();
}

void loop() {
  String json;
  //save incoming data incoming from the main board
  if (Serial.available()) {
    json = Serial.readString();
    Serial.println(json);
  }

  //update the image shown on the screen only when the flag is set
  if (updateScreenEnable) {
    //unset the flag
    updateScreenEnable = false;

    //parse the incoming data
    if (json.length()) {
      json.trim();
      StaticJsonDocument<100> doc;
      DeserializationError e = deserializeJson(doc, json);
      if (e) {
        Serial.println(e.f_str());
      } else {
        vel = doc["vel"];
        left = doc["left"];
        right = doc["right"];
      }
    }

    //update screen image with the new information
    updateScreen();
  }
  delay(1);
}

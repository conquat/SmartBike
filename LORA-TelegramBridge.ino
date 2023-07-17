#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include "EEPROM.h"

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

const char* ssid = "YOUR-SSID";
const char* password = "YOUR-PASSWORD";
#define BOTtoken "TELEGRAM-BOT-TOKEN"
#define CHAT_ID_LEN 16
#define BOT_REQUEST_DELAY 1000
#define MAX_CHATS 10

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

unsigned long lastTimeBotRan;
unsigned long lastTimeSent;

//looks for the chatID in the EEPROM and returns its index in the n entries long array, -1 if not found
int checkChatID(String chat_id, uint8_t n) {
  //loops the array and stops if it finds the chatId or reaches the end
  for (uint8_t i = 0; i < n; ++i) {
    //calculates the length of the chatID in the i position of the array
    uint8_t len = 0;
    while (len < CHAT_ID_LEN && EEPROM.read(1 + i * CHAT_ID_LEN + len)) {
      ++len;
    }

    bool found = true;
    //chacks wether the the chatID lengths correspond
    if (chat_id.length() != len) {
      found = false;
    } else {
      //conpares the the chatIDs
      for (uint8_t j = 0; j < len; ++j) {
        if (EEPROM.read(1 + i * CHAT_ID_LEN + j) != chat_id[j]) {
          found = false;
          break;
        }
      }
    }

    if (found) {
      return i;
    }
  }
  return -1;
}

void handleNewMessages(int numNewMessages) {
  DEBUG_PRINTLN("----------------------------------");
  //loops all the incoming messages
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    String text = bot.messages[i].text;

    if (text == "/start") {
      DEBUG_PRINTLN("Bot started");
      DEBUG_PRINTLN(chat_id);

      //read the number of subscribed chats from the EEPROM 
      uint8_t n_chats = EEPROM.read(0);
      if (n_chats < MAX_CHATS) {
        DEBUG_PRINT("number of chats: ");
        DEBUG_PRINTLN(n_chats);

        //checks the chatID index in the array stored in the EEPROM
        int chat_id_index = checkChatID(chat_id, n_chats);

        if (chat_id_index == -1) { //if chatID not found
          // add chat id to EEPROM
          DEBUG_PRINTLN("No match found");
          for (int i = 0; i < chat_id.length(); ++i) {
            EEPROM.write(1 + n_chats * CHAT_ID_LEN + i, chat_id[i]);
          }
          if (chat_id.length() < CHAT_ID_LEN) {
            EEPROM.write(1 + n_chats * CHAT_ID_LEN + chat_id.length(), '\0');
          }
          ++n_chats;
          EEPROM.write(0, n_chats);
          EEPROM.commit();

          bot.sendMessage(chat_id, "You are now registered! You will receive updates on the device motion.");

          chat_id_index = checkChatID(chat_id, n_chats);
          DEBUG_PRINT("n_chats: ");
          DEBUG_PRINTLN(n_chats);
          DEBUG_PRINT("chat_id index: ");
          DEBUG_PRINTLN(chat_id_index);
        } else {
          DEBUG_PRINT("Chat index: ");
          DEBUG_PRINTLN(chat_id_index);
        }
      }
    } else if (text == "/end") {
      uint8_t n_chats = EEPROM.read(0);
      int chat_id_index = checkChatID(chat_id, n_chats);

      //removes chatID from the array only if it's present
      if (chat_id_index != -1) {
        if (chat_id_index != n_chats - 1) { //if the chatID to be removed isn't the last in the array, replace it with that
          for (uint8_t i = 0; i < CHAT_ID_LEN; ++i) {
            EEPROM.write(1 + chat_id_index * CHAT_ID_LEN + i, EEPROM.read(1 + (n_chats - 1) * CHAT_ID_LEN + i));
          }
        }
        --n_chats;
        EEPROM.write(0, n_chats);
        EEPROM.commit();
      }
      DEBUG_PRINT("Chat index deleted: ");
      DEBUG_PRINTLN(chat_id_index);
      DEBUG_PRINT("New number of chats: ");
      DEBUG_PRINTLN(n_chats);
    }
  }
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  Serial2.begin(9600);
  DEBUG_PRINTLN("RX!");

  //initialize EEPROM
  if (!EEPROM.begin(4096)) {
    DEBUG_PRINTLN("failed to initialise EEPROM");
    delay(1000000);
  }

  //connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);  //add root certificate for api.telegram.org

  //wait wifi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    DEBUG_PRINTLN("Connecting to WiFi..");
  }
  DEBUG_PRINTLN("CONNECTED TO WiFi");
}

void loop() {
  //run the bot every BOT_REQUEST_DELAY milliseconds
  if (millis() - lastTimeBotRan > BOT_REQUEST_DELAY) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      DEBUG_PRINTLN("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }

  //receive message from LORA
  if (Serial2.available()) {
    int n = Serial2.read();
    DEBUG_PRINT("LORA sent: ");
    DEBUG_PRINTLN(n);
    if (n == 1) { //only send message if the received message contains 1
      char chat_id_buffer[CHAT_ID_LEN + 1];
      int n_chats;

      //loop all saved chatIDs
      n_chats = EEPROM.read(0);
      for (int i = 0; i < n_chats; ++i) {
        uint8_t len = 0;
        while (len < CHAT_ID_LEN && EEPROM.read(1 + i * CHAT_ID_LEN + len)) {
          chat_id_buffer[len] = EEPROM.read(1 + i * CHAT_ID_LEN + len);
          ++len;
        }
        chat_id_buffer[len] = '\0';
        String chat_id(chat_id_buffer);

        //send message to the chat
        bot.sendMessage(chat_id, "The device has just moved!");
        DEBUG_PRINT("Message sent to ");
        DEBUG_PRINTLN(chat_id);
      }
    }
  }
}
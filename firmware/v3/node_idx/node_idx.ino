#include <Preferences.h>

Preferences prefs;

void setup() {
  Serial.begin(115200);
  prefs.begin("config", false);

  // Write
  prefs.putUInt("node_idx", 1);

  // Read
  uint32_t value = prefs.getUInt("node_idx", 10); 
  // 0 = default if not present
  pinMode(D3, OUTPUT);
}

void loop() {
  uint32_t node_idx = prefs.getUInt("node_idx", 10);
  Serial.println(node_idx);

  uint32_t blink_time = 0;
  if (node_idx != 10){
    if (node_idx == 0){
      blink_time = 500;
      digitalWrite(D3, HIGH);
      delay(500);
      digitalWrite(D3,LOW);
    }
    else{
      blink_time = node_idx * 300;
      for(uint32_t i=0;i<node_idx;i++){
        digitalWrite(D3, HIGH);
        delay(150);
        digitalWrite(D3,LOW);
        delay(150);

      }
    }
  }

  


  delay(2000 - blink_time);
}

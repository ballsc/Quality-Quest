#include <Servo.h>

#define VREF 5.0            // Arduino Uno ADC reference voltage
#define SCOUNT 30
#define TDS_PIN A0
#define PH_PIN A1
#define TIMEIN 45000
#define TIMEOUT 90000


 // ms -> 10000 = 10 seconds

const int pump1Pin = 9;
const int pump2Pin = 10;

Servo pump1;
Servo pump2;

bool collecting = false;

void setup() {
  Serial.begin(115200);

  pinMode(TDS_PIN, INPUT);
  pinMode(PH_PIN, INPUT);

  pump1.attach(pump1Pin);
  pump2.attach(pump2Pin);

  pump1.write(90);  // Neutral (off)
  pump2.write(90);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "START") {
      collecting = true;

      // Pump in for 10 seconds
      pump1.write(0);
      pump2.write(0);
      delay(TIMEIN);
      pump1.write(90);
      pump2.write(90);
    } 
    else if (command == "STOP") {
      collecting = false;

      // Pump out for 10 seconds
      pump1.write(180);
      pump2.write(180);
      delay(TIMEOUT);
      pump1.write(90);
      pump2.write(90);
    }
  }

  if (collecting) {
    // === TDS Reading ===
    int totalTDS = 0;
    for (int i = 0; i < SCOUNT; i++) {
      totalTDS += analogRead(TDS_PIN);
      delay(20);
    }
    float voltageTDS = (totalTDS / (float)SCOUNT) * (VREF / 1023.0);
    float tdsValue = (133.42 * pow(voltageTDS, 3) -
                      255.86 * pow(voltageTDS, 2) +
                      857.39 * voltageTDS) * 0.5;

    // === pH Reading ===
    int rawPH = analogRead(PH_PIN);
    float voltagePH = rawPH * (VREF / 1023.0);
    float pHValue = 3.5 * voltagePH + 0.5;

    // Print as CSV: pH,TDS
    Serial.print(pHValue, 2);
    Serial.print(",");
    Serial.println(tdsValue, 2);

    delay(1000);
  }
}

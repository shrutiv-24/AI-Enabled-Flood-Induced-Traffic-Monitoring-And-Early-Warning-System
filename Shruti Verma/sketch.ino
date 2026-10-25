// AI-Enabled Flood-Induced Traffic Monitoring and Early Warning System

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---- Pins ----
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int RAIN_PIN = 34;   // ADC
const int LED_G = 25;
const int LED_Y = 26;
const int LED_R = 27;
const int BUZZER = 14;
const int BTN_VEHICLE = 33; // each press = one vehicle detected

// ---- Thresholds (bench values) ----
const float CAUTION_DEPTH_CM = 2.0;
const float CRITICAL_DEPTH_CM = 5.0;
const int   RAIN_HIGH_PCT = 50;
const int   DENSE_TRAFFIC = 6;        // vehicles in last 60 s
const unsigned long WINDOW_MS = 60000;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

float baselineCm = 0;
unsigned long vehicleTimes[60];
int vehicleCount = 0;
int lastBtn = HIGH;
int confirm = 0;
int shownRisk = 0;                    // 0 Normal, 1 Caution, 2 Critical
unsigned long lastBeep = 0, lastSample = 0;

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
        long us = pulseIn(ECHO_PIN, HIGH, 30000);
          if (us == 0) return -1;
            return us * 0.0343 / 2.0;
            }

            float avgDistance(int n) {
              float sum = 0; int ok = 0;
                for (int i = 0; i < n; i++) {
                    float d = readDistanceCm();
                        if (d > 0) { sum += d; ok++; }
                            delay(60);
                              }
                                return ok ? sum / ok : -1;
                                }

                                int rainPercent() {
                                  int raw = analogRead(RAIN_PIN);
                                    return constrain(map(raw, 4095, 0, 0, 100), 0, 100);
                                    }

                                    void countVehicles() {
                                      int b = digitalRead(BTN_VEHICLE);
                                        if (lastBtn == HIGH && b == LOW && vehicleCount < 60) {
                                            vehicleTimes[vehicleCount++] = millis();
                                              }
                                                lastBtn = b;
                                                  int k = 0;
                                                    for (int i = 0; i < vehicleCount; i++)
                                                        if (millis() - vehicleTimes[i] <= WINDOW_MS) vehicleTimes[k++] = vehicleTimes[i];
                                                          vehicleCount = k;
                                                          }

                                                          int computeRisk(float depth, int rain, int vehicles) {
                                                            bool dense = vehicles >= DENSE_TRAFFIC;
                                                              bool heavyRain = rain >= RAIN_HIGH_PCT;
                                                                if (depth >= CRITICAL_DEPTH_CM && dense) return 2;
                                                                  if (depth >= CRITICAL_DEPTH_CM) return 1;
                                                                    if (depth >= CAUTION_DEPTH_CM && (heavyRain || dense)) return 1;
                                                                      return 0;
                                                                      }

                                                                      void alarm(int risk) {
                                                                        digitalWrite(LED_G, risk == 0);
                                                                          digitalWrite(LED_Y, risk == 1);
                                                                            digitalWrite(LED_R, risk == 2);
                                                                              unsigned long now = millis();
                                                                                if (risk == 2 && now - lastBeep > 300)  { tone(BUZZER, 2000, 150); lastBeep = now; }
                                                                                  else if (risk == 1 && now - lastBeep > 2000) { tone(BUZZER, 1000, 150); lastBeep = now; }
                                                                                  }

                                                                                  void setup() {
                                                                                    Serial.begin(115200);
                                                                                      pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
                                                                                        pinMode(LED_G, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_R, OUTPUT);
                                                                                          pinMode(BUZZER, OUTPUT);
                                                                                            pinMode(BTN_VEHICLE, INPUT_PULLUP);
                                                                                              display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
                                                                                                display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(1);
                                                                                                  display.setCursor(0, 0); display.println("Calibrating dry road..."); display.display();
                                                                                                    baselineCm = avgDistance(5);
                                                                                                      Serial.print("Baseline distance (cm): "); Serial.println(baselineCm);
                                                                                                        Serial.println("time_s,depth_cm,rain_pct,vehicles_per_min,risk");
                                                                                                        }

                                                                                                        void loop() {
                                                                                                          countVehicles();
                                                                                                            unsigned long now = millis();
                                                                                                              if (now - lastSample >= 500) {
                                                                                                                  lastSample = now;
                                                                                                                      float d = readDistanceCm();
                                                                                                                          float depth = (d > 0 && baselineCm > 0) ? max(0.0f, baselineCm - d) : 0;
                                                                                                                              int rain = rainPercent();
                                                                                                                                  int risk = computeRisk(depth, rain, vehicleCount);

                                                                                                                                      if (risk > shownRisk) { if (++confirm >= 2) { shownRisk = risk; confirm = 0; } }
                                                                                                                                          else { shownRisk = risk; confirm = 0; }

                                                                                                                                              const char* names[] = {"NORMAL", "CAUTION", "CRITICAL"};
                                                                                                                                                  display.clearDisplay(); display.setCursor(0, 0);
                                                                                                                                                      display.print("Water: "); display.print(depth, 1); display.println(" cm");
                                                                                                                                                          display.print("Rain: ");  display.print(rain);     display.println(" %");
                                                                                                                                                              display.print("Traffic: "); display.print(vehicleCount); display.println(" /min");
                                                                                                                                                                  display.setTextSize(2); display.setCursor(0, 44);
                                                                                                                                                                      display.println(names[shownRisk]); display.setTextSize(1);
                                                                                                                                                                          display.display();

                                                                                                                                                                              Serial.print(now / 1000); Serial.print(',');
                                                                                                                                                                                  Serial.print(depth, 1);   Serial.print(',');
                                                                                                                                                                                      Serial.print(rain);       Serial.print(',');
                                                                                                                                                                                          Serial.print(vehicleCount); Serial.print(',');
                                                                                                                                                                                              Serial.println(names[shownRisk]);
                                                                                                                                                                                                }
                                                                                                                                                                                                  alarm(shownRisk);
                                                                                                                                                                                                  }
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* =======================KONFIGURASI PID======================= */

float Kp = 1.2;
float Ki = 0.02;
float Kd = 0.5;

int setPoint = 600;   // Target kelembapan tanah

/* =======================VARIABEL PID======================= */

float error, lastError = 0;
float integral = 0;
float derivative;
float pidOutput;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // Pompa OFF awal (relay aktif LOW)

  lcd.setCursor(0, 0);
  lcd.print("IRRIGATION");
  lcd.setCursor(0, 1);
  lcd.print("SYSTEM IS ON");
  delay(3000);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0;
  lastTime = now;

  int value = analogRead(A0);

  /* =======================HITUNG PID======================= */
  error = value - setPoint;   // TANAH KERING → error positif
  integral += error * deltaTime;
  derivative = (error - lastError) / deltaTime;

  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  /* =======================KONTROL POMPA (ON / OFF)======================= */
   
  if (pidOutput > 20) {        // Deadband agar stabil
    digitalWrite(2, LOW);     // Pompa ON
    lcd.setCursor(0, 0);
    lcd.print("Water Pump ON ");
  } else {
    digitalWrite(2, HIGH);    // Pompa OFF
    lcd.setCursor(0, 0);
    lcd.print("Water Pump OFF");
  }

  /* =======================DISPLAY KELEMBAPAN======================= */
   
  lcd.setCursor(0, 1);
  if (value < 300) {
    lcd.print("Moisture: HIGH");
  } else if (value < 950) {
    lcd.print("Moisture: MID ");
  } else {
    lcd.print("Moisture: LOW ");
  }

  /* =======================SERIAL MONITOR ======================= */
   
  Serial.print("Moisture: ");
  Serial.print(value);
  Serial.print(" | PID Output: ");
  Serial.println(pidOutput);

  delay(500);
}

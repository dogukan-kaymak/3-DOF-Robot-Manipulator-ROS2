#include <Arduino.h>

// ================= PİNLER (v2.1 — Optimize Edilmiş Gruplu Layout) =================
// Motor 1 — Sol taraf üst grup (GPIO 34, 35, 33, 32)
const int ENC1_A = 34; const int ENC1_B = 35;
const int M1_IN1 = 33; const int M1_IN2 = 32;  // Orijinal pin sırası

// Motor 2 — Sol taraf alt grup (GPIO 25, 26, 14, 27)
const int ENC2_A = 25; const int ENC2_B = 26;
const int M2_IN1 = 14; const int M2_IN2 = 27;

// Motor 3 — Sağ taraf grup (GPIO 19, 18, 16, 17 — GPIO5 atlanır)
const int ENC3_A = 19; const int ENC3_B = 18;
const int M3_IN1 = 16; const int M3_IN2 = 17;

// ================= ENCODER MATEMATİĞİ =================
// Full quadrature: 48 CPR × 74.83 gear ratio = ~3592 pulse/tur
const float PULSE_PER_DEGREE = 3592.0 / 360.0; // ~9.978 pulses per degree

// ================= VARIABLES =================
volatile long enc_value[3] = {0, 0, 0};       // Current real positions (in pulses)
long target_position[3] = {0, 0, 0};          // Desired target positions (in pulses)
bool pid_active[3] = {false, false, false};   // Starts in COAST mode

// Motor yön çarpanları: encoder-motor uyumu için
// Motor 1 ters bağlandıysa -1 yap, diğerleri 1
const int motor_dir[3] = {1, 1, 1};  // Hepsi orijinal yön

// ==== PID COEFFICIENTS (TUNED — Overshoot Önlemeli) ====
float Kp = 1.5;   // Proportional: Düşürüldü — agresif tepki önleme
float Ki = 0.7;   // Integral: Yerçekimi kompanzasyonu için yeterli
float Kd = 0.3;   // Derivative: Artırıldı — fren gücü yüksek
const int MAX_PWM = 255;  // PWM çıkış limiti (0-255 arası, ani tepki önler)

// PID Memory Variables
float integral[3] = {0, 0, 0};
long prev_error[3] = {0, 0, 0};
unsigned long prev_time[3] = {0, 0, 0};

unsigned long lastPrintTime = 0;

// ================= KESMELER — Full Quadrature (Ch.A + Ch.B) =================
// Motor 1
void IRAM_ATTR enc1A_isr() { if(digitalRead(ENC1_A)==digitalRead(ENC1_B)) enc_value[0]++; else enc_value[0]--; }
void IRAM_ATTR enc1B_isr() { if(digitalRead(ENC1_A)!=digitalRead(ENC1_B)) enc_value[0]++; else enc_value[0]--; }
// Motor 2
void IRAM_ATTR enc2A_isr() { if(digitalRead(ENC2_A)==digitalRead(ENC2_B)) enc_value[1]++; else enc_value[1]--; }
void IRAM_ATTR enc2B_isr() { if(digitalRead(ENC2_A)!=digitalRead(ENC2_B)) enc_value[1]++; else enc_value[1]--; }
// Motor 3
void IRAM_ATTR enc3A_isr() { if(digitalRead(ENC3_A)==digitalRead(ENC3_B)) enc_value[2]++; else enc_value[2]--; }
void IRAM_ATTR enc3B_isr() { if(digitalRead(ENC3_A)!=digitalRead(ENC3_B)) enc_value[2]++; else enc_value[2]--; }

void setup() {
  Serial.begin(9600);
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  // Encoder Pinleri
  pinMode(ENC1_A, INPUT); pinMode(ENC1_B, INPUT);           // GPIO 34,35: input-only, pull-up YOK
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP); // GPIO 25,26: pull-up destekli
  pinMode(ENC3_A, INPUT_PULLUP); pinMode(ENC3_B, INPUT_PULLUP); // GPIO 19,18: pull-up destekli

  // Full Quadrature: Ch.A + Ch.B interrupt
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1A_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1B_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2A_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2B_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3A_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_B), enc3B_isr, CHANGE);

  // Set all motors to coast initially
  for(int i=1; i<=3; i++) coastMotor(i);

  Serial.println("\n=============================================");
  Serial.println("--- 3-AXIS SYNC PID (DEGREES MODE) ---");
  Serial.println("Usage format: Angle1 Angle2 Angle3");
  Serial.println("Example: 90.0 -45 180");
  Serial.println("=============================================\n");

  // Boot sırasında oluşan sahte pulse'ları temizle
  delay(10);
  enc_value[0] = 0; enc_value[1] = 0; enc_value[2] = 0;
}

void coastMotor(int motorNo) {
  int pin1, pin2;
  if(motorNo==1){pin1=M1_IN1; pin2=M1_IN2;}
  else if(motorNo==2){pin1=M2_IN1; pin2=M2_IN2;}
  else if(motorNo==3){pin1=M3_IN1; pin2=M3_IN2;}
  else return;
  
  analogWrite(pin1, 0); analogWrite(pin2, 0); // Cut power entirely
}

void driveMotor(int motorNo, int power) {
  int pin1, pin2;
  if(motorNo==1){pin1=M1_IN1; pin2=M1_IN2;}
  else if(motorNo==2){pin1=M2_IN1; pin2=M2_IN2;}
  else if(motorNo==3){pin1=M3_IN1; pin2=M3_IN2;}
  else return;

  // Constrain power to safe limits (overshoot önleme)
  if (power > MAX_PWM) power = MAX_PWM;
  if (power < -MAX_PWM) power = -MAX_PWM;

  // Minimum power threshold to overcome static friction (Deadband compensation)
  if (power > 0 && power < 40) power = 40; 
  if (power < 0 && power > -40) power = -40;

  if (power > 0) { 
    analogWrite(pin1, power); analogWrite(pin2, 0); 
  } else if (power < 0) { 
    analogWrite(pin1, 0); analogWrite(pin2, abs(power)); 
  } else { 
    analogWrite(pin1, 255); analogWrite(pin2, 255); // Brake / Lock state
  }
}

void loop() {
  unsigned long current_time = millis();

  // 1. READ 3 ANGLES FROM USER SIMULTANEOUSLY
  if (Serial.available() > 0) {
    float angle1 = Serial.parseFloat();   
    float angle2 = Serial.parseFloat();   
    float angle3 = Serial.parseFloat();   
    
    // Buffer'daki TÜM artık karakterleri temizle (\r, \n, boşluk vs.)
    delay(10);  // Tüm baytların gelmesini bekle
    while(Serial.available() > 0) Serial.read();

    // Geçersiz komutu yoksay (3'ü de 0 ise muhtemelen sahte parse)
    if (angle1 == 0.0 && angle2 == 0.0 && angle3 == 0.0) {
      // Kullanıcı gerçekten 0 0 0 gönderdiyse coastMotor yap
      for(int i=0; i<3; i++) {
        pid_active[i] = false;
        coastMotor(i + 1);
      }
      Serial.println("\n>>> MOTORS RELEASED (COAST MODE)");
    } else {
      // Convert Angles to Pulses and Save targets
      target_position[0] = angle1 * PULSE_PER_DEGREE;
      target_position[1] = angle2 * PULSE_PER_DEGREE;
      target_position[2] = angle3 * PULSE_PER_DEGREE;

      // Engage PID for all 3 motors
      for(int i=0; i<3; i++) {
        pid_active[i] = true; 
        integral[i] = 0; // Reset integral memory to prevent sudden jerks
        prev_error[i] = 0;
        prev_time[i] = current_time;
      }

      Serial.print("\n>>> 3 MOTORS ENGAGED! TARGETS -> M1: "); Serial.print(angle1, 1);
      Serial.print(" deg | M2: "); Serial.print(angle2, 1);
      Serial.print(" deg | M3: "); Serial.print(angle3, 1); Serial.println(" deg");
    }
  }

  // 2. FULL PID LOOP (Runs on native pulses for precision and speed)
  for (int i = 0; i < 3; i++) {
    if (pid_active[i]) {
      float dt = (current_time - prev_time[i]) / 1000.0; // Time difference in seconds
      if (dt <= 0.0) dt = 0.01; // Prevent division by zero

      long error = target_position[i] - enc_value[i]; 
      
      int motor_power = 0;

      // HOLDING MODE (Yerçekimine Karşı Asılı Kalma)
      if (abs(error) <= 10) {  // ~1° deadband
        // Hedefteyiz! P ve D'yi kapat, sadece o anki İntegral gücünü motora vermeye devam et.
        // Hata artmadığı için İntegral şişmez, motor yerçekimine karşı taş gibi asılı kalır.
        motor_power = (int)(Ki * integral[i]);
        prev_error[i] = 0;
        prev_time[i] = current_time;
      } else {
        // HAREKET MODU
        integral[i] += error * dt;
        if (integral[i] > 1000) integral[i] = 1000;
        if (integral[i] < -1000) integral[i] = -1000;

        float derivative = (error - prev_error[i]) / dt;
        float pid_output = (Kp * error) + (Ki * integral[i]) + (Kd * derivative);
        motor_power = (int)pid_output;
        
        prev_error[i] = error;
        prev_time[i] = current_time;
      }
      
      driveMotor(i + 1, motor_power * motor_dir[i]);
    } else {
      // If no target is set, let the motor coast freely
      coastMotor(i + 1);
    }
  }

  // 3. PRINT INFO TO SERIAL MONITOR (Every 1 second)
  if (current_time - lastPrintTime > 1000) {
    lastPrintTime = current_time;
    
    for (int i = 0; i < 3; i++) {
      float current_angle = (float)enc_value[i] / PULSE_PER_DEGREE;
      float target_angle = (float)target_position[i] / PULSE_PER_DEGREE;
      
      Serial.print("M"); Serial.print(i + 1); Serial.print(": "); 
      if (pid_active[i]) {
        Serial.print(target_angle, 1); Serial.print(" deg ");
      } else {
        Serial.print("COAST ");
      }
      Serial.print("(Curr: "); Serial.print(current_angle, 1); Serial.print(" deg)");
      if (i < 2) Serial.print("  |  ");
    }
    Serial.println();
  }

  delay(10); // Loop runs at roughly 100Hz
}

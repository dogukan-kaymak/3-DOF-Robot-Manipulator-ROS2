#include <Arduino.h>

// ================= PİNLER (v2.1 — Optimize Edilmiş Gruplu Layout)
// ================= Motor 1 — Sol taraf üst grup (GPIO 34, 35, 32, 33)
const int ENC1_A = 34; // Input-only, pull-up YOK
const int ENC1_B = 35; // Input-only, pull-up YOK
const int M1_IN1 = 33; // PWM çıkışı (A4950) — yer değiştirildi
const int M1_IN2 = 32; // PWM çıkışı (A4950) — yer değiştirildi

// Motor 2 — Sol taraf alt grup (GPIO 25, 26, 27, 14)
const int ENC2_A = 25; // Pull-up destekli
const int ENC2_B = 26; // Pull-up destekli
const int M2_IN1 = 14; // PWM çıkışı (A4950)
const int M2_IN2 = 27; // PWM çıkışı (A4950)

// Motor 3 — Sağ taraf grup (GPIO 19, 18, 17, 16 — GPIO5 atlanır)
const int ENC3_A = 19; // Pull-up destekli
const int ENC3_B = 18; // Pull-up destekli
const int M3_IN1 = 16; // PWM çıkışı (A4950)
const int M3_IN2 = 17; // PWM çıkışı (A4950)

// ================= DEĞİŞKENLER =================
int hiz = 100; // Baslangic PWM Hızı (0-255)
volatile long enc1_count = 0;
volatile long enc2_count = 0;
volatile long enc3_count = 0;

// ================= ENCODER MATEMATİĞİ =================
const float ENCODER_CPR = 48.0;     // Full quadrature (Ch.A + Ch.B)
const float GEAR_RATIO = 74.83;     // Redüktör oranı
const float PULSE_PER_REV = ENCODER_CPR * GEAR_RATIO;  // ~3592 pulse/tur
const float DEG_PER_PULSE = 360.0 / PULSE_PER_REV;     // ~0.1002°/pulse

unsigned long sonKomutZamani = 0;
const int ZAMAN_ASIMI =
    150; // 150 milisaniye boyunca komut gelmezse otomatik kilitle
unsigned long sonYazdirma = 0;

// ================= KESMELER — Full Quadrature (Ch.A + Ch.B) =================
// Motor 1 ISR'leri
void IRAM_ATTR enc1A_isr() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B))
    enc1_count++;
  else
    enc1_count--;
}
void IRAM_ATTR enc1B_isr() {
  if (digitalRead(ENC1_A) != digitalRead(ENC1_B))
    enc1_count++;
  else
    enc1_count--;
}

// Motor 2 ISR'leri
void IRAM_ATTR enc2A_isr() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B))
    enc2_count++;
  else
    enc2_count--;
}
void IRAM_ATTR enc2B_isr() {
  if (digitalRead(ENC2_A) != digitalRead(ENC2_B))
    enc2_count++;
  else
    enc2_count--;
}

// Motor 3 ISR'leri
void IRAM_ATTR enc3A_isr() {
  if (digitalRead(ENC3_A) == digitalRead(ENC3_B))
    enc3_count++;
  else
    enc3_count--;
}
void IRAM_ATTR enc3B_isr() {
  if (digitalRead(ENC3_A) != digitalRead(ENC3_B))
    enc3_count++;
  else
    enc3_count--;
}

void setup() {
  Serial.begin(9600);

  // Motor Pinleri
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  // Encoder Pinleri
  // Motor 1: GPIO 34, 35 — input-only, harici pull-up veya encoder dahili
  // pull-up gerekli
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  // Motor 2: GPIO 25, 26 — dahili pull-up destekli
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  // Motor 3: GPIO 19, 18 — dahili pull-up destekli
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);

  // Kesmeleri (Interrupt) Bağla — Full Quadrature: Ch.A + Ch.B
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1A_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1B_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2A_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2B_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3A_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_B), enc3B_isr, CHANGE);

  // Başlangıçta hepsini kilitle
  motorKilit(1);
  motorKilit(2);
  motorKilit(3);

  Serial.println("\n=============================================");
  Serial.println("--- YENI TEST ARAYUZU (Hiz Kontrollu) ---");
  Serial.println("Ayrilan tuslara art arda bas/gonder.");
  Serial.println("Gondermeyi kestigin an motor otomatik KILITLENIR.");
  Serial.println("\nMotor 1: Y (Ileri) --- H (Geri)");
  Serial.println("Motor 2: U (Ileri) --- J (Geri)");
  Serial.println("Motor 3: I (Ileri) --- K (Geri)");
  Serial.println("\nHIZ AYARI:");
  Serial.println("  [+] tusu : Hizi 10 artirir");
  Serial.println("  [-] tusu : Hizi 10 azaltir");
  Serial.print("\nSU ANKI HIZ: ");
  Serial.println(hiz);
  Serial.println("=============================================\n");

  // Tüm setup işlemleri bitti — sahte pulse'ları temizle
  delay(10); // Elektriksel geçişlerin oturmasını bekle
  enc1_count = 0;
  enc2_count = 0;
  enc3_count = 0;
}

void motorKontrol(int motorNo, int in1_pwm, int in2_pwm) {
  if (motorNo == 1) {
    analogWrite(M1_IN1, in1_pwm);
    analogWrite(M1_IN2, in2_pwm);
  } else if (motorNo == 2) {
    analogWrite(M2_IN1, in1_pwm);
    analogWrite(M2_IN2, in2_pwm);
  } else if (motorNo == 3) {
    analogWrite(M3_IN1, in1_pwm);
    analogWrite(M3_IN2, in2_pwm);
  }
}

void motorKilit(int motorNo) {
  motorKontrol(motorNo, 255, 255); // A4950 için Fren (Brake) konumu
}

void loop() {
  // Seri porttan veri geliyorsa oku
  if (Serial.available() > 0) {
    char komut = Serial.read();
    komut = toupper(komut); // Küçük harfleri büyüğe çevir

    bool gecerliHareketKomutu = true;

    if (komut == 'Y')
      motorKontrol(1, hiz, 0);
    else if (komut == 'H')
      motorKontrol(1, 0, hiz);
    else if (komut == 'U')
      motorKontrol(2, hiz, 0);
    else if (komut == 'J')
      motorKontrol(2, 0, hiz);
    else if (komut == 'I')
      motorKontrol(3, hiz, 0);
    else if (komut == 'K')
      motorKontrol(3, 0, hiz);
    else if (komut == '+') {
      hiz += 10;
      if (hiz > 255)
        hiz = 255; // Maksimum sınır
      Serial.print("\n>>> YENI HIZ: ");
      Serial.print(hiz);
      Serial.println(" <<<");
      gecerliHareketKomutu = false; // Hız değişimi motoru döndürmez, bu yüzden
                                    // kilitleme süresini etkilemez
    } else if (komut == '-') {
      hiz -= 10;
      if (hiz < 30)
        hiz = 30; // Minimum sınır (motor dönmeyecek kadar güçsüz kalmasın)
      Serial.print("\n>>> YENI HIZ: ");
      Serial.print(hiz);
      Serial.println(" <<<");
      gecerliHareketKomutu = false;
    } else
      gecerliHareketKomutu =
          false; // \n, \r veya alakasız harfler geldiğinde yoksay

    // Eğer tuş geçerli bir yön tuşuysa, zaman sayacını sıfırla (Motoru
    // döndürmeye devam et)
    if (gecerliHareketKomutu) {
      sonKomutZamani = millis();
    }
  }

  // Eğer son hareket komutunun üstünden 150 ms geçtiyse ve yeni tuş
  // basılmadıysa HER ŞEYİ KİLİTLE
  if (millis() - sonKomutZamani > ZAMAN_ASIMI) {
    motorKilit(1);
    motorKilit(2);
    motorKilit(3);
  }

  // Her yarım saniyede bir Encoder verilerini ekrana bas
  if (millis() - sonYazdirma > 500) {
    sonYazdirma = millis();

    float aci1 = enc1_count * DEG_PER_PULSE;
    float aci2 = enc2_count * DEG_PER_PULSE;
    float aci3 = enc3_count * DEG_PER_PULSE;

    Serial.print("M1: ");
    Serial.print(aci1, 1);
    Serial.print("\xC2\xB0 (");
    Serial.print(enc1_count);
    Serial.print(")\t| M2: ");
    Serial.print(aci2, 1);
    Serial.print("\xC2\xB0 (");
    Serial.print(enc2_count);
    Serial.print(")\t| M3: ");
    Serial.print(aci3, 1);
    Serial.print("\xC2\xB0 (");
    Serial.print(enc3_count);
    Serial.println(")");
  }
}

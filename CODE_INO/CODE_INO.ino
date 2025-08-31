#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <esp_sleep.h>

// ================== KONFIGURASI WIFI ==================
const char* ssid = "rusli";
const char* password = "rusli123";

#define BOT_TOKEN "8180955722:AAGoqixITv49DbCcdJqeinVIm_oZwBay-5A"
#define CHAT_ID "7720160177"

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOT_TOKEN, clientTCP);

// ================== NTP WAKTU ==================
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // GMT+7

// ================== PIN SENSOR ULTRASONIC ==================
#define TRIG_PIN 12 
#define ECHO_PIN 15
#define LED_FLASH 4
#define STATUS_LED 33  // Optional status LED

// ================== KONFIGURASI ESP32-CAM ==================
// Konfigurasi untuk AI-Thinker ESP32-CAM (yang paling umum)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ================== KONSTANTA KONFIGURASI ==================
const unsigned long COOLDOWN_PERIOD = 3000;    // 30 detik cooldown
const unsigned long DEBOUNCE_DELAY = 2000;      // 2 detik debounce
const unsigned long STATUS_INTERVAL = 3600000;  // 1 jam status update
const unsigned long WIFI_TIMEOUT = 20000;       // 20 detik timeout WiFi
const unsigned long BOT_CHECK_INTERVAL = 1000;  // Cek pesan bot setiap detik
const int DETECTION_THRESHOLD = 50;             // Jarak deteksi dalam cm
const int CONSECUTIVE_READINGS = 3;              // Jumlah pembacaan berturut-turut untuk konfirmasi

// ================== VARIABEL GLOBAL ==================
unsigned long lastDetectionTime = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastBotCheck = 0;
bool objectDetected = false;
int detectionCount = 0;
int consecutiveDetections = 0;
bool systemArmed = true;  // Sistem dapat diaktifkan/nonaktifkan

// Variabel untuk mengirim foto binary
uint8_t* photoBuffer = nullptr;
size_t photoLength = 0;
size_t photoIndex = 0;

// Buffer untuk multiple readings ultrasonic
int distanceReadings[CONSECUTIVE_READINGS];
int readingIndex = 0;

// ================== FUNGSI INISIALISASI KAMERA ==================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 8000000;   // Clock sangat rendah untuk stabilitas maksimal
  config.pixel_format = PIXFORMAT_JPEG;

  // Konfigurasi paling aman - tanpa PSRAM untuk menghindari DMA overflow
  Serial.println("Menggunakan konfigurasi ultra-stabil tanpa PSRAM");
  config.frame_size = FRAMESIZE_CIF;     // 352x288 - sangat stabil
  config.jpeg_quality = 20;              // Quality rendah untuk size kecil
  config.fb_count = 1;                   // Single frame buffer
  config.fb_location = CAMERA_FB_IN_DRAM; // Gunakan DRAM, bukan PSRAM
  config.grab_mode = CAMERA_GRAB_LATEST;  // Always get latest frame

  // Coba inisialisasi kamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init gagal: 0x%x\n", err);
    
    // Coba konfigurasi paling minimal
    Serial.println("Mencoba konfigurasi minimal...");
    
    esp_camera_deinit();
    delay(500);
    
    // Konfigurasi minimal - paling aman
    config.pin_pwdn = -1;              // Disable power down completely
    config.frame_size = FRAMESIZE_QQVGA; // 160x120 - terkecil
    config.jpeg_quality = 30;          // Quality paling rendah
    config.fb_count = 1;
    config.xclk_freq_hz = 6000000;     // Clock paling rendah
    config.fb_location = CAMERA_FB_IN_DRAM;
    
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init minimal juga gagal: 0x%x\n", err);
      return false;
    }
  }

  Serial.println("Kamera berhasil diinisialisasi");

  // Bersihkan DMA buffer beberapa kali
  Serial.println("Membersihkan DMA buffer...");
  for (int i = 0; i < 5; i++) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
      delay(100);
    }
  }

  // Test foto dengan retry
  Serial.println("Testing kamera dengan retry...");
  camera_fb_t * fb = nullptr;
  for (int retry = 0; retry < 3; retry++) {
    fb = esp_camera_fb_get();
    if (fb && fb->len > 500) { // Minimal 500 bytes untuk foto valid
      break;
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = nullptr;
    }
    delay(200);
  }
  
  if (!fb || fb->len < 500) {
    Serial.println("Test foto gagal setelah beberapa percobaan");
    return false;
  }
  
  Serial.printf("Test foto berhasil - Size: %dx%d, Format: %d, Len: %d\n", 
                fb->width, fb->height, fb->format, fb->len);
  esp_camera_fb_return(fb);

  // Konfigurasi sensor minimal
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    // Hanya konfigurasi paling dasar untuk menghindari masalah
    s->set_brightness(s, 0);     
    s->set_contrast(s, 0);       
    s->set_whitebal(s, 1);       
    s->set_exposure_ctrl(s, 1);  
    Serial.println("Sensor kamera dikonfigurasi minimal");
  }
  
  // Delay untuk stabilisasi
  delay(1000);
  Serial.println("Kamera siap digunakan");
  
  return true;
}

// ================== FUNGSI KONEKSI WIFI ==================
bool connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan WiFi");
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nGagal terhubung WiFi!");
    return false;
  }
  
  Serial.println("\nWiFi Terhubung!");
  Serial.println("IP address: " + WiFi.localIP().toString());
  Serial.println("Signal strength: " + String(WiFi.RSSI()) + " dBm");
  return true;
}

// ================== FUNGSI TANGGAL DAN WAKTU ==================
String getFormattedDateTime() {
  timeClient.update();
  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti = localtime(&rawtime);
  
  char buffer[64];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", 
          ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday,
          ti->tm_hour, ti->tm_min, ti->tm_sec);
  
  return String(buffer);
}

// ================== CALLBACK FUNCTIONS FOR BINARY PHOTO ==================
bool moreDataAvailable() {
  return (photoIndex < photoLength);
}

uint8_t getNextByte() {
  return (photoIndex >= photoLength) ? 0 : photoBuffer[photoIndex++];
}

uint8_t* getNextBuffer() {
  return (photoIndex >= photoLength) ? nullptr : (photoBuffer + photoIndex);
}

// ================== FUNGSI ULTRASONIC DENGAN FILTER ==================
long readUltrasonicFiltered() {
  const int numReadings = 3;
  long readings[numReadings];
  long total = 0;
  int validReadings = 0;
  
  // Ambil beberapa pembacaan
  for (int i = 0; i < numReadings; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 35000); // timeout 35ms
    if (duration > 0) {
      readings[i] = duration * 0.034 / 2;
      total += readings[i];
      validReadings++;
    } else {
      readings[i] = -1;
    }
    delay(50); // Delay antar pembacaan
  }
  
  if (validReadings == 0) return -1;
  
  // Return rata-rata dari pembacaan valid
  return total / validReadings;
}

// ================== FUNGSI DETEKSI OBJEK DENGAN DEBOUNCING ==================
bool detectObjectStable() {
  long distance = readUltrasonicFiltered();
  
  // Simpan pembacaan ke buffer circular
  distanceReadings[readingIndex] = distance;
  readingIndex = (readingIndex + 1) % CONSECUTIVE_READINGS;
  
  // Cek apakah semua pembacaan terakhir menunjukkan objek terdeteksi
  int detectionVotes = 0;
  for (int i = 0; i < CONSECUTIVE_READINGS; i++) {
    if (distanceReadings[i] > 0 && distanceReadings[i] < DETECTION_THRESHOLD) {
      detectionVotes++;
    }
  }
  
  return (detectionVotes >= CONSECUTIVE_READINGS - 1); // Allow 1 false reading
}

// ================== FUNGSI AMBIL FOTO & KIRIM ==================
bool sendPhotoTelegram() {
  if (!systemArmed) {
    Serial.println("Sistem tidak aktif, foto tidak dikirim");
    return false;
  }
  
  String timestamp = getFormattedDateTime();
  
  // Flush DMA buffer sebelum ambil foto untuk menghindari overflow
  camera_fb_t * dummy_fb = esp_camera_fb_get();
  if (dummy_fb) {
    esp_camera_fb_return(dummy_fb);
  }
  delay(100);
  
  // Nyalakan LED flash
  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_FLASH, HIGH);
  delay(150);  // Delay sedikit lebih lama untuk flash yang lebih baik
  
  Serial.println("Mengambil foto...");
  camera_fb_t * fb = esp_camera_fb_get();
  
  // Matikan LED flash
  digitalWrite(LED_FLASH, LOW);
  
  if (!fb) {
    Serial.println("Gagal mengambil foto");
    bot.sendMessage(CHAT_ID, "❌ Gagal mengambil foto pada " + timestamp, "");
    return false;
  }

  // Cek ukuran foto yang wajar
  if (fb->len < 1000 || fb->len > 200000) {
    Serial.printf("Ukuran foto tidak normal: %d bytes\n", fb->len);
    esp_camera_fb_return(fb);
    bot.sendMessage(CHAT_ID, "⚠️ Foto berukuran abnormal pada " + timestamp, "");
    return false;
  }

  // Setup untuk pengiriman binary
  photoBuffer = fb->buf;
  photoLength = fb->len;
  photoIndex = 0;

  Serial.printf("Mengirim foto (%d bytes) ke Telegram...\n", photoLength);
  
  // Kirim foto dengan callback yang sudah diperbaiki
  String sent = bot.sendPhotoByBinary(CHAT_ID, "image/jpeg", photoLength,
                                     moreDataAvailable, getNextByte, getNextBuffer,
                                     nullptr);

  bool success = (sent != "");
  
  // Kirim caption
  String caption = success ? "🚨 DETEKSI GERAKAN! 🚨\n" : "⚠️ Foto gagal terkirim ⚠️\n";
  caption += "📅 Waktu: " + timestamp + "\n";
  caption += "📏 Jarak: " + String(readUltrasonicFiltered()) + " cm\n";
  caption += "📊 Deteksi hari ini: " + String(detectionCount) + "x\n";
  caption += "📶 WiFi: " + String(WiFi.RSSI()) + " dBm\n";
  caption += "📸 Ukuran foto: " + String(photoLength) + " bytes";
  
  bot.sendMessage(CHAT_ID, caption, "");
  
  esp_camera_fb_return(fb);
  
  // Reset variabel foto
  photoBuffer = nullptr;
  photoLength = 0;
  photoIndex = 0;
  
  Serial.println(success ? "Foto berhasil dikirim!" : "Gagal mengirim foto!");
  return success;
}

// ================== FUNGSI STATUS UPDATE ==================
void sendStatusUpdate() {
  String timestamp = getFormattedDateTime();
  
  String message = "📊 STATUS SISTEM MONITORING\n";
  message += "━━━━━━━━━━━━━━━━━━━━━━\n";
  message += "📅 Waktu: " + timestamp + "\n";
  message += "🌐 IP: " + WiFi.localIP().toString() + "\n";
  message += "📶 WiFi: " + String(WiFi.RSSI()) + " dBm\n";
  message += "🎯 Deteksi hari ini: " + String(detectionCount) + "x\n";
  message += "⚡ Status: " + String(systemArmed ? "AKTIF" : "NONAKTIF") + "\n";
  message += "💾 Free heap: " + String(ESP.getFreeHeap()) + " bytes\n";
  message += "🔋 Uptime: " + String(millis() / 1000) + "s\n";
  message += "━━━━━━━━━━━━━━━━━━━━━━\n";
  message += systemArmed ? "✅ Sistem monitoring aktif" : "⏸️ Sistem dalam mode standby";
  
  bot.sendMessage(CHAT_ID, message, "");
  Serial.println("Status update dikirim");
}

// ================== FUNGSI HANDLE PESAN BOT ==================
void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) continue; // Hanya terima dari chat ID yang ditentukan
    
    String text = bot.messages[i].text;
    String from_name = bot.messages[i].from_name;
    
    Serial.println("Pesan dari " + from_name + ": " + text);
    
    if (text == "/start" || text == "/help") {
      String help = "🤖 ESP32-CAM Motion Detector Bot\n\n";
      help += "📋 Perintah yang tersedia:\n";
      help += "/status - Lihat status sistem\n";
      help += "/photo - Ambil foto sekarang\n";
      help += "/arm - Aktifkan sistem\n";
      help += "/disarm - Nonaktifkan sistem\n";
      help += "/reset - Reset counter deteksi\n";
      
      help += "/help - Tampilkan bantuan ini";
      
      bot.sendMessage(chat_id, help, "");
    }
    else if (text == "/status") {
      sendStatusUpdate();
    }
    else if (text == "/photo") {
      bot.sendMessage(chat_id, "📸 Mengambil foto...", "");
      sendPhotoTelegram();
    }
    else if (text == "/arm") {
      systemArmed = true;
      bot.sendMessage(chat_id, "✅ Sistem monitoring DIAKTIFKAN", "");
    }
    else if (text == "/disarm") {
      systemArmed = false;
      bot.sendMessage(chat_id, "⏸️ Sistem monitoring DINONAKTIFKAN", "");
    }
    else if (text == "/reset") {
      detectionCount = 0;
      bot.sendMessage(chat_id, "🔄 Counter deteksi direset", "");
    }
    else {
      bot.sendMessage(chat_id, "❓ Perintah tidak dikenali. Ketik /help untuk bantuan.", "");
    }
  }
}

// ================== SETUP ==================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Matikan brownout detector
  Serial.begin(115200);
  Serial.println("\n=== ESP32-CAM Motion Detector ===");

  // Setup pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_FLASH, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(LED_FLASH, LOW);
  digitalWrite(STATUS_LED, LOW);

  // Inisialisasi buffer readings
  for (int i = 0; i < CONSECUTIVE_READINGS; i++) {
    distanceReadings[i] = -1;
  }

  // Koneksi WiFi dengan retry
  int wifiRetries = 3;
  while (wifiRetries > 0 && !connectWiFi()) {
    wifiRetries--;
    if (wifiRetries > 0) {
      Serial.println("Retry koneksi WiFi dalam 5 detik...");
      delay(5000);
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Gagal terhubung WiFi setelah beberapa percobaan. Restart...");
    ESP.restart();
  }

  // Sync waktu NTP
  timeClient.begin();
  int ntpRetries = 10;
  while (!timeClient.update() && ntpRetries > 0) {
    timeClient.forceUpdate();
    delay(1000);
    ntpRetries--;
  }
  
  if (ntpRetries > 0) {
    Serial.println("Waktu sinkron: " + getFormattedDateTime());
  } else {
    Serial.println("Gagal sinkronisasi waktu NTP");
  }

  // Setup SSL untuk Telegram
  clientTCP.setInsecure();

  // Inisialisasi kamera
  if (!initCamera()) {
    Serial.println("Gagal inisialisasi kamera. Restart...");
    ESP.restart();
  }
  Serial.println("Kamera berhasil diinisialisasi");

  // LED status berkedip untuk indikasi sistem siap
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);
    delay(200);
  }

  // Kirim pesan startup
  String startupMsg = "🚀 SISTEM MONITORING AKTIF\n";
  startupMsg += "📅 Waktu mulai: " + getFormattedDateTime() + "\n";
  startupMsg += "🌐 IP: " + WiFi.localIP().toString() + "\n";
  startupMsg += "📶 WiFi: " + String(WiFi.RSSI()) + " dBm\n";
  startupMsg += "✅ Siap monitoring gerakan!";
  
  bot.sendMessage(CHAT_ID, startupMsg, "");
  
  // Delay untuk stabilisasi sistem sebelum masuk loop
  delay(2000);
  Serial.println("=== Setup selesai, mulai monitoring ===");
}

// ================== MAIN LOOP ==================
void loop() {
  // Debug: Print status setiap 10 detik untuk memastikan loop berjalan
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 10000) {
    Serial.printf("Loop aktif - Uptime: %lu detik, Free heap: %d\n", 
                  millis() / 1000, ESP.getFreeHeap());
    lastDebug = millis();
  }

  // Cek koneksi WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi terputus, mencoba reconnect...");
    if (!connectWiFi()) {
      delay(30000); // Wait 30 seconds before retry
      return;
    }
  }

  // Cek pesan Telegram dengan error handling
  if (millis() - lastBotCheck > BOT_CHECK_INTERVAL) {
    try {
      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      if (numNewMessages > 0) {
        Serial.printf("Menerima %d pesan baru\n", numNewMessages);
        handleNewMessages(numNewMessages);
      }
    } catch (...) {
      Serial.println("Error saat cek pesan Telegram");
    }
    lastBotCheck = millis();
  }

  // Deteksi gerakan jika sistem aktif
  if (systemArmed) {
    bool stableDetection = detectObjectStable();
    
    if (stableDetection && !objectDetected) {
      unsigned long currentTime = millis();
      
      if (currentTime - lastDetectionTime > COOLDOWN_PERIOD) {
        Serial.println("🚨 Objek terdeteksi stabil! Mengirim foto...");
        digitalWrite(STATUS_LED, HIGH);
        
        objectDetected = true;
        detectionCount++;
        
        if (sendPhotoTelegram()) {
          lastDetectionTime = currentTime;
        }
        
        digitalWrite(STATUS_LED, LOW);
      } else {
        Serial.println("Deteksi dalam cooldown period, diabaikan");
      }
    } else if (!stableDetection) {
      objectDetected = false;
    }
  }

  // Status update berkala
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    Serial.println("Mengirim status update...");
    sendStatusUpdate();
    lastStatusUpdate = millis();
    
    // Reset detection count daily (simplified)
    detectionCount = 0;
  }

  // Status LED blink setiap 5 detik untuk indikasi hidup
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 5000) {
    digitalWrite(STATUS_LED, HIGH);
    delay(50);
    digitalWrite(STATUS_LED, LOW);
    lastBlink = millis();
  }

  delay(100); // Main loop delay
}
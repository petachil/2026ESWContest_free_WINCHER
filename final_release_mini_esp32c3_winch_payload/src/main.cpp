#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

// ==========================================================
// 1. [D] 기기별 설정 파라미터 (1호기 / 2호기 업로드 시 MAC만 수정)
// ==========================================================
// [D] 윈치 메인 ESP32의 MAC 주소 입력
uint8_t receiverAddress[] = {0xB4, 0xBF, 0xE9, 0xFB, 0x29, 0x1C}; // [D] 윈치 메인 ESP32 MAC 주소

// ==========================================================
// 2. 센서 및 I2C 정의
// ==========================================================
#define MPU6050_ADDR         0x68
#define MPU6050_CONFIG       0x1A
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define I2C_SDA 6
#define I2C_SCL 7

typedef struct __attribute__((packed)) {
    float roll;     // rad 단위
    float pitch;    
    float gyroX;    // rad/s 단위
    float gyroY;    
    float gyroZ;    
    float accX;     // G 단위 
    float accY;     
    float accZ;     
} struct_message; 

struct_message payloadData; 
esp_now_peer_info_t peerInfo; 

// ==========================================================
// 3. 채널 탐색 및 상태 동기화 변수
// ==========================================================
uint8_t currentChannel = 1;
bool channelLocked = false;
uint16_t failCount = 0;
const uint16_t MAX_FAIL_COUNT = 15; // 약 60ms(15회 연속 실패) 동안 ACK가 없으면 채널 언락

volatile bool lastSendSuccess = false; // 콜백과 loop() 간 동기화용 플래그
volatile bool newAckArrived = false;   // 신규 ACK 수신 확인용 플래그

unsigned long lastChannelSwitchTime = 0;
const unsigned long CHANNEL_SWITCH_INTERVAL = 250; // 채널당 250ms 체류하며 탐색

// Wi-Fi 물리 채널 변경 및 ESP-NOW 피어 동적 재등록 함수 (loop()에서만 안전하게 호출)
void updateWifiChannel(uint8_t ch) {
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // 기존 피어 안전하게 삭제 후 새 채널로 바인딩
    esp_now_del_peer(receiverAddress);
    peerInfo.channel = ch;
    esp_now_add_peer(&peerInfo);
}

// ESP-NOW 송신 결과 콜백 (인터럽트 맥락: 최소한의 플래그 기록만 수행)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
    newAckArrived = true;
}

// ==========================================================
// 4. Adaptive Mahony 필터 알고리즘
// ==========================================================
#define KP_DEFAULT 0.8f    
#define KI_DEFAULT 0.0f  
#define ACCEL_G_LOWER 0.1f 
#define ACCEL_G_UPPER 3.0f 

float gx_offset = 0.0f, gy_offset = 0.0f, gz_offset = 0.0f; 

class AdaptiveMahony {
private:
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
public: 
    float roll_rad = 0.0f, pitch_rad = 0.0f; 
    float gyroX_rads = 0.0f, gyroY_rads = 0.0f, gyroZ_rads = 0.0f; 

    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
        gyroX_rads = gx; 
        gyroY_rads = gy;
        gyroZ_rads = gz; 

        float accel_norm = sqrtf(ax * ax + ay * ay + az * az); 
        float current_kp = KP_DEFAULT;
        
        if (accel_norm < ACCEL_G_LOWER || accel_norm > ACCEL_G_UPPER) current_kp = 0.0f; 

        if (accel_norm > 0.0f) { 
            ax /= accel_norm; ay /= accel_norm; az /= accel_norm; 
            float vx = 2.0f * (q1 * q3 - q0 * q2); 
            float vy = 2.0f * (q0 * q1 + q2 * q3);
            float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
            float ex = (ay * vz - az * vy); 
            float ey = (az * vx - ax * vz);
            float ez = (ax * vy - ay * vx);

            gx += current_kp * ex; gy += current_kp * ey; gz += current_kp * ez; 
        }
        gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt); 

        float qa = q0, qb = q1, qc = q2; 
        q0 += (-qb * gx - qc * gy - q3 * gz); 
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); 
        if (q_norm > 0.0f) { q0 /= q_norm; q1 /= q_norm; q2 /= q_norm; q3 /= q_norm; } 

        roll_rad  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)); 
        float sinp = 2.0f * (q0 * q2 - q3 * q1); 
        if (fabsf(sinp) >= 1.0f) pitch_rad = copysignf(1.5707963f, sinp); 
        else pitch_rad = asinf(sinp); 
    }
};

AdaptiveMahony filter; 

// ==========================================================
// 5. MPU6050 센서 제어 함수
// ==========================================================
void readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(MPU6050_ACCEL_XOUT_H); 
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true); 

    int16_t rawAx = Wire.read() << 8 | Wire.read(); 
    int16_t rawAy = Wire.read() << 8 | Wire.read();
    int16_t rawAz = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); 
    int16_t rawGx = Wire.read() << 8 | Wire.read(); 
    int16_t rawGy = Wire.read() << 8 | Wire.read();
    int16_t rawGz = Wire.read() << 8 | Wire.read();

    ax = (float)rawAx / 8192.0f; 
    ay = (float)rawAy / 8192.0f; 
    az = (float)rawAz / 8192.0f;
    gx = ((float)rawGx / 65.5f) * DEG_TO_RAD; 
    gy = ((float)rawGy / 65.5f) * DEG_TO_RAD; 
    gz = ((float)rawGz / 65.5f) * DEG_TO_RAD;
}

void initMPU6050() {
    Wire.begin(I2C_SDA, I2C_SCL); 
    Wire.setClock(400000); 

    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(MPU6050_PWR_MGMT_1); Wire.write(0x00); 
    Wire.endTransmission(true); 

    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(MPU6050_CONFIG); Wire.write(0x04); 
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(0x1B); Wire.write(0x08); 
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(MPU6050_ACCEL_CONFIG); Wire.write(0x08); 
    Wire.endTransmission(true);
}

void calibrateGyroOnly() {
    float sumGx = 0, sumGy = 0, sumGz = 0; 
    for (int i = 0; i < 500; i++) { 
        float ax, ay, az, gx, gy, gz; 
        readMPU6050(ax, ay, az, gx, gy, gz); 
        sumGx += gx; sumGy += gy; sumGz += gz; 
        delay(2); 
    }
    gx_offset = sumGx / 500.0f; 
    gy_offset = sumGy / 500.0f; 
    gz_offset = sumGz / 500.0f;
}

// ==========================================================
// 6. setup() 및 loop()
// ==========================================================
unsigned long lastFilterUpdate = 0; 

void setup() {
    Serial.begin(115200); 
    initMPU6050(); 
    calibrateGyroOnly(); 
    
    WiFi.mode(WIFI_STA); 

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        return;
    }

    // 콜백 함수 등록
    esp_now_register_send_cb(OnDataSent);

    // 피어 기본 설정
    memcpy(peerInfo.peer_addr, receiverAddress, 6); 
    peerInfo.channel = currentChannel; 
    peerInfo.encrypt = false; 
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer!");
    }

    updateWifiChannel(currentChannel);

    // 송신 전력 설정은 Wi-Fi 및 ESP-NOW 초기화 완료 후 최하단에서 처리
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    lastFilterUpdate = micros(); 
}

void loop() {
    unsigned long nowMicros = micros(); 
    unsigned long nowMillis = millis();

    // 1. 센서 연산 및 ESP-NOW 데이터 송신 (250Hz / 4ms 고정 루프)
    if (nowMicros - lastFilterUpdate >= 4000) { 
        float dt = (nowMicros - lastFilterUpdate) / 1000000.0f; 
        lastFilterUpdate = nowMicros; 

        // 콜백에서 수신한 ACK 상태를 loop() 메인 스레드에서 평가
        if (newAckArrived) {
            newAckArrived = false;
            if (lastSendSuccess) {
                failCount = 0;
                channelLocked = true; // ACK 수신 성공 시 채널 고정
            } else {
                failCount++;
                if (failCount >= MAX_FAIL_COUNT) {
                    channelLocked = false; // 연속 실패 시 채널 언락
                }
            }
        }

        float ax, ay, az, gx, gy, gz; 
        readMPU6050(ax, ay, az, gx, gy, gz); 
        
        filter.update(ax, ay, az, gx - gx_offset, gy - gy_offset, gz - gz_offset, dt); 

        payloadData.roll  = filter.roll_rad; 
        payloadData.pitch = filter.pitch_rad;
        payloadData.gyroX = filter.gyroX_rads;
        payloadData.gyroY = filter.gyroY_rads;
        payloadData.gyroZ = filter.gyroZ_rads;
        payloadData.accX  = ax;
        payloadData.accY  = ay;
        payloadData.accZ  = az;

        // ESP-NOW 패킷 송신
        esp_now_send(receiverAddress, (uint8_t *) &payloadData, sizeof(payloadData)); 
    }

    // 2. 독립 채널 탐색 타이머 (채널 언락 상태일 때 250ms 간격으로 순회)
    if (!channelLocked) {
        if (nowMillis - lastChannelSwitchTime >= CHANNEL_SWITCH_INTERVAL) {
            lastChannelSwitchTime = nowMillis;
            
            currentChannel = (currentChannel % 13) + 1; // 1 -> 2 -> ... -> 13 -> 1 채널 순회
            updateWifiChannel(currentChannel);         // 물리 채널 및 피어 정보 재등록
            failCount = 0;
        }
    }
}
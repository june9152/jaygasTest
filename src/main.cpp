#include <SPI.h>
#include <Arduino.h>

// SPI 핀 정의
#define MOSI_PIN 6
#define MISO_PIN 7
#define SCLK_PIN 5

// 센서 선택
// 사용할 센서의 번호를 0에서 3 사이로 선택하세요.
// 0: Formaldehyde, 1: Acetic Acid, 2: Acetaldehyde, 3: Toluene
#define USE_SENSOR 1

// 센서 구조체
struct Sensor {
  uint8_t cs_pin;
  const char* name;
  uint8_t channel;  // ADS1220 입력 채널
  float offset;     // 캘리브레이션 오프셋
};

#if USE_SENSOR == 0
Sensor sensors[] = { {4,  "Formaldehyde", 0, 0.0} };
#elif USE_SENSOR == 1
Sensor sensors[] = { {17, "Acetic Acid",  0, 0.0} };
#elif USE_SENSOR == 2
Sensor sensors[] = { {16, "Acetaldehyde", 0, 0.0} };
#elif USE_SENSOR == 3
Sensor sensors[] = { {15, "Toluene",      0, 0.0} };
#else
#error "Invalid sensor selected. Please choose a value between 0 and 3."
#endif

#define NUM_SENSORS 1

// ADS1220 명령어
#define ADS1220_CMD_RESET    0x06
#define ADS1220_CMD_START    0x08
#define ADS1220_CMD_POWERDOWN 0x02
#define ADS1220_CMD_RDATA    0x10
#define ADS1220_CMD_RREG     0x20
#define ADS1220_CMD_WREG     0x40

// Function declarations
void testConnections();
void initSensor(uint8_t index);
int32_t readSensor(uint8_t index, uint8_t channel);
float adcToVoltage(int32_t adc);
float adcToPPM(int32_t adc, uint8_t sensorType);
void readAllRegisters(uint8_t cs);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);
  
  Serial.println("\n=== ADS1220 Single Sensor System ===");
  
  // SPI 초기화
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
  SPI.setFrequency(1000000); // 1MHz
  
  // CS 핀 초기화
  pinMode(sensors[0].cs_pin, OUTPUT);
  digitalWrite(sensors[0].cs_pin, HIGH);
  
  delay(10);
  
  // 센서 연결 테스트
  testConnections();
  
  // 센서 초기화
  initSensor(0);
  
  // 캘리브레이션 (옵션)
  // performCalibration();
}

void testConnections() {
  Serial.println("\n--- Connection Test ---");
  
  digitalWrite(sensors[0].cs_pin, LOW);
  delayMicroseconds(50);
  
  // 리셋 후 기본값 확인
  SPI.transfer(ADS1220_CMD_RESET);
  delay(10);
  
  // CONFIG0 읽기
  SPI.transfer(ADS1220_CMD_RREG);
  SPI.transfer(0x00);
  uint8_t value = SPI.transfer(0x00);
  
  digitalWrite(sensors[0].cs_pin, HIGH);
  
  Serial.print(sensors[0].name);
  Serial.print(": ");
  if (value == 0xFF || value == 0x00) {
    Serial.println("Connection FAILED!");
  } else {
    Serial.print("OK (0x");
    Serial.print(value, HEX);
    Serial.println(")");
  }
  
  delay(10);
}

void initSensor(uint8_t index) {
  uint8_t cs = sensors[index].cs_pin;
  
  Serial.print("Initializing ");
  Serial.print(sensors[index].name);
  Serial.print("... ");
  
  digitalWrite(cs, LOW);
  delayMicroseconds(10);
  
  // 리셋
  SPI.transfer(ADS1220_CMD_RESET);
  delay(10);
  
  // 설정 쓰기
  // CONFIG0: 단일 종단 입력 모드 (AIN0 to AVSS)
  SPI.transfer(ADS1220_CMD_WREG | 0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x08); // AIN0/AVSS, Gain=1, PGA disabled
  
  // CONFIG1: 데이터 레이트 설정
  SPI.transfer(ADS1220_CMD_WREG | 0x01);
  SPI.transfer(0x00);
  SPI.transfer(0x04); // 20 SPS, Normal mode
  
  // CONFIG2: 전압 레퍼런스 설정
  SPI.transfer(ADS1220_CMD_WREG | 0x02);
  SPI.transfer(0x00);
  SPI.transfer(0x10); // Internal 2.048V reference
  
  // CONFIG3: IDAC 설정
  SPI.transfer(ADS1220_CMD_WREG | 0x03);
  SPI.transfer(0x00);
  SPI.transfer(0x00); // IDAC off
  
  digitalWrite(cs, HIGH);
  
  Serial.println("Done");
  delay(5);
}

int32_t readSensor(uint8_t index, uint8_t channel) {
  uint8_t cs = sensors[index].cs_pin;
  int32_t result = 0;
  
  // 채널 선택을 위한 CONFIG0 값 계산
  uint8_t config0 = (channel << 4) | 0x08; // channel을 AINx/AVSS로 설정
  
  // 트랜잭션 시작
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);
  delayMicroseconds(10);
  
  // 채널 설정
  SPI.transfer(ADS1220_CMD_WREG | 0x00);
  SPI.transfer(0x00);
  SPI.transfer(config0);
  
  // 변환 시작
  SPI.transfer(ADS1220_CMD_START);
  
  // CS를 HIGH로 하여 변환 진행
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  
  // 변환 대기 (20SPS = 50ms)
  delay(55);
  
  // 데이터 읽기
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);
  delayMicroseconds(10);
  
  SPI.transfer(ADS1220_CMD_RDATA);
  
  // 24비트 읽기
  uint8_t data[3];
  data[0] = SPI.transfer(0x00);
  data[1] = SPI.transfer(0x00);
  data[2] = SPI.transfer(0x00);
  
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  
  // 24비트를 32비트로 변환
  result = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
  
  // 부호 확장
  if (result & 0x800000) {
    result |= 0xFF000000;
  }
  
  return result;
}

float adcToVoltage(int32_t adc) {
  // 2.048V reference, 23-bit resolution
  return (adc * 2.048) / 8388607.0;
}

float adcToPPM(int32_t adc, uint8_t sensorType) {
  float voltage = adcToVoltage(adc);
  
  // 센서별 변환 공식 (예시)
  // 실제 센서 데이터시트에 맞게 수정 필요
  switch(sensorType) {
    case 0: // Formaldehyde
      return voltage * 100.0; // 예시 변환
    case 1: // Acetic Acid
      return voltage * 50.0;
    case 2: // Acetaldehyde
      return voltage * 75.0;
    case 3: // Toluene
      return voltage * 150.0;
    default:
      return 0.0;
  }
}

void loop() {
  Serial.println("\n=== Sensor Reading ===");
  
  // AIN0~AIN3 모두 읽기
  for (uint8_t channel = 0; channel < 4; channel++) {
    int32_t adc = readSensor(0, channel);
    float voltage = adcToVoltage(adc);
    
    Serial.print("AIN");
    Serial.print(channel);
    Serial.print(": ADC=");
    Serial.print(adc);
    Serial.print(" | V=");
    Serial.print(voltage, 4);
    Serial.println("V");
    
    // 비정상 값 체크
    if (voltage > 2.0 || voltage < 0.0) {
      Serial.print("  WARNING: Abnormal voltage on AIN");
      Serial.print(channel);
      Serial.println("!");
    }
  }
  
  delay(1000);
}

// 디버깅용 함수
void readAllRegisters(uint8_t cs) {
  Serial.println("\n--- Register Dump ---");
  
  digitalWrite(cs, LOW);
  delayMicroseconds(10);
  
  for (int i = 0; i < 4; i++) {
    SPI.transfer(ADS1220_CMD_RREG | i);
    SPI.transfer(0x00);
    uint8_t value = SPI.transfer(0x00);
    
    Serial.print("CONFIG");
    Serial.print(i);
    Serial.print(" = 0x");
    Serial.println(value, HEX);
  }
  
  digitalWrite(cs, HIGH);
}
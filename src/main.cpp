#include <SPI.h>
#include <Arduino.h>
#include "Protocentral_ADS1220.h"

// ADS1220 설정
#define PGA          1                 // Programmable Gain = 1
#define VREF         2.048            // Internal reference of 2.048V
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1)

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
  uint8_t drdy_pin;
  const char* name;
  uint8_t channel;  // ADS1220 입력 채널
  float offset;     // 캘리브레이션 오프셋
  Protocentral_ADS1220* ads1220;  // ADS1220 인스턴스 포인터
};

// DRDY 핀 정의 (각 센서마다 다른 핀 필요)
#if USE_SENSOR == 0
Sensor sensors[] = { {4,  8, "Formaldehyde", 0, 0.0, nullptr} };
#elif USE_SENSOR == 1
Sensor sensors[] = { {17, 18, "Acetic Acid",  0, 0.0, nullptr} };
#elif USE_SENSOR == 2
Sensor sensors[] = { {16, 9, "Acetaldehyde", 0, 0.0, nullptr} };
#elif USE_SENSOR == 3
Sensor sensors[] = { {15, 10, "Toluene",      0, 0.0, nullptr} };
#else
#error "Invalid sensor selected. Please choose a value between 0 and 3."
#endif

#define NUM_SENSORS 1

// Function declarations
void testConnections();
void initSensor(uint8_t index);
int32_t readSensor(uint8_t index, uint8_t channel);
float convertToMilliV(int32_t i32data);
float adcToVoltage(int32_t adc);
float adcToPPM(int32_t adc, uint8_t sensorType);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);
  
  Serial.println("\n=== ADS1220 Single Sensor System (Using Protocentral Library) ===");
  
  // SPI 초기화
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
  
  // ADS1220 인스턴스 생성
  sensors[0].ads1220 = new Protocentral_ADS1220();
  
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
  
  // ADS1220 초기화 시도
  sensors[0].ads1220->begin(sensors[0].cs_pin, sensors[0].drdy_pin);
  
  // 테스트 읽기 수행
  sensors[0].ads1220->set_conv_mode_single_shot();
  int32_t testValue = sensors[0].ads1220->Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH0);
  
  Serial.print(sensors[0].name);
  Serial.print(": ");
  if (testValue == 0xFFFFFF || testValue == 0x000000) {
    Serial.println("Connection FAILED!");
  } else {
    Serial.print("OK (ADC Value: ");
    Serial.print(testValue);
    Serial.println(")");
  }
  
  delay(10);
}

void initSensor(uint8_t index) {
  Serial.print("Initializing ");
  Serial.print(sensors[index].name);
  Serial.print("... ");
  
  Protocentral_ADS1220* ads = sensors[index].ads1220;
  
  // ADS1220 초기화
  ads->begin(sensors[index].cs_pin, sensors[index].drdy_pin);
  
  // 설정
  ads->set_data_rate(DR_20SPS);           // 20 SPS
  ads->set_pga_gain(PGA_GAIN_1);          // Gain = 1
  ads->PGA_OFF();                          // PGA 비활성화 (2.048V 범위 사용)
  ads->set_conv_mode_single_shot();       // 단일 샷 모드
  ads->set_VREF(VREF_2048);               // 내부 2.048V 레퍼런스 사용
  ads->set_FIR_Filter(FIR_OFF);   // FIR 필터 비활성화
  
  Serial.println("Done");
  delay(5);
}

int32_t readSensor(uint8_t index, uint8_t channel) {
  Protocentral_ADS1220* ads = sensors[index].ads1220;
  
  // 채널별 MUX 설정 계산
  uint8_t mux_channel = MUX_SE_CH0 + (channel << 4);
  
  // 단일 샷 읽기
  int32_t adc_data = ads->Read_SingleShot_SingleEnded_WaitForData(mux_channel);
  
  return adc_data;
}

float convertToMilliV(int32_t i32data) {
  return (float)((i32data * VFSR * 1000) / FULL_SCALE);
}

float adcToVoltage(int32_t adc) {
  return convertToMilliV(adc) / 1000.0;
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
  
  // 온도 센서 읽기 (내장 온도 센서)
  sensors[0].ads1220->TemperatureSensorMode_enable();
  int32_t temp_adc = sensors[0].ads1220->Read_SingleShot_WaitForData();
  sensors[0].ads1220->TemperatureSensorMode_disable();
  
  // 온도 계산 (ADS1220 데이터시트 참조)
  float temperature = (float)temp_adc * 0.03125;  // 온도 변환 계수
  Serial.print("\nInternal Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");
  
  delay(1000);
}
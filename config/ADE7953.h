

#include "esphome.h"

#define ADE7953_PREF    1540
#define ADE7953_UREF    26000
#define ADE7953_IREF    10000

#define ADE7953_ADDR    0x38

using namespace esphome;

static const char *TAG = "sensor.ade7953";

class ADE7953Component : public PollingComponent {

    int Ade7953RegSize(uint16_t reg)
    {
      int size = 0;
      switch ((reg >> 8) & 0x0F) {
        case 0x03:
          size++;
        case 0x02:
          size++;
        case 0x01:
          size++;
        case 0x00:
        case 0x07:
        case 0x08:
          size++;
      }
      return size;
    }
    
    void Ade7953Write(uint16_t reg, uint32_t val)
    {
      int size = Ade7953RegSize(reg);
      if (size) {
        Wire.beginTransmission(ADE7953_ADDR);
        Wire.write((reg >> 8) & 0xFF);
        Wire.write(reg & 0xFF);
        while (size--) {
          Wire.write((val >> (8 * size)) & 0xFF);  // Write data, MSB first
        }
        Wire.endTransmission();
        delayMicroseconds(5);    // Bus-free time minimum 4.7us
      }
    }
    
    uint32_t Ade7953Read(uint16_t reg)
    {
    	uint32_t response = 0;
    
      int size = Ade7953RegSize(reg);
      if (size) {
        Wire.beginTransmission(ADE7953_ADDR);
        Wire.write((reg >> 8) & 0xFF);
        Wire.write(reg & 0xFF);
        Wire.endTransmission(0);
        Wire.requestFrom(ADE7953_ADDR, size);
        if (size <= Wire.available()) {
          for (int i = 0; i < size; i++) {
            response = response << 8 | Wire.read();   // receive DATA (MSB first)
          }
        }
      }
    	return response;
    }

 public:

  sensor::Sensor *voltage_ = new sensor::Sensor();
  sensor::Sensor *current1_ = new sensor::Sensor();
  sensor::Sensor *current2_ = new sensor::Sensor();
  sensor::Sensor *power1_ = new sensor::Sensor();
  sensor::Sensor *power2_ = new sensor::Sensor();

  // constructor
  ADE7953Component() : PollingComponent(60000) {}

  void setup() override {
    ESP_LOGCONFIG(TAG, "Setting up ADE7953...");
    delay(100);                     // Need 100mS to init ADE7953
    Ade7953Write(0x102, 0x0004);    // Locking the communication interface (Clear bit COMM_LOCK), Enable HPF
    Ade7953Write(0x0FE, 0x00AD);    // Unlock register 0x120
    Ade7953Write(0x120, 0x0030);    // Configure optimum setting
  }

    void dump_config() override {
      ESP_LOGCONFIG(TAG, "ADE7953:");

      LOG_SENSOR("  ", "Voltage", voltage_);
      LOG_SENSOR("  ", "Current 1", current1_);
      LOG_SENSOR("  ", "Current 2", current2_);
      LOG_SENSOR("  ", "Power 1", power1_);
      LOG_SENSOR("  ", "Power 2", power2_);
      ESP_LOGCONFIG(TAG, "  i2c address: %p", ADE7953_ADDR);
      LOG_UPDATE_INTERVAL(this);
    }

  // This will be called every "update_interval" milliseconds.
  void update() override {

    uint32_t ade7953_active_power1 = 0;
    uint32_t ade7953_active_power2 = 0;
    uint32_t ade7953_current_rms1 = 0;
    uint32_t ade7953_current_rms2 = 0;
    uint32_t ade7953_voltage_rms = 0;
    float voltage = 0;
    float current1 = 0;
    float current2 = 0;
    float power1 = 0;
    float power2 = 0;

    ade7953_voltage_rms = Ade7953Read(0x31C);      // Both relays
    
    ade7953_current_rms1 = Ade7953Read(0x31B);     // Relay 1
    if (ade7953_current_rms1 < 2000) {             // No load threshold (20mA)
        ade7953_current_rms1 = 0;
        ade7953_active_power1 = 0;
    } else {
        ade7953_active_power1 = abs((int32_t)Ade7953Read(0x313));  // Relay 1
    }
    
    ade7953_current_rms2 = Ade7953Read(0x31A);     // Relay 2
    if (ade7953_current_rms2 < 2000) {             // No load threshold (20mA)
        ade7953_current_rms2 = 0;
        ade7953_active_power2 = 0;
    } else {
        ade7953_active_power2 = abs((int32_t)Ade7953Read(0x312));  // Relay 2
    }

    voltage = (float)ade7953_voltage_rms / ADE7953_UREF;
    ESP_LOGD(TAG, "'%s': Got voltage=%.1fV", voltage_->get_name().c_str(), voltage);
    current1 = (float)ade7953_current_rms1 / (ADE7953_IREF * 10);
    ESP_LOGD(TAG, "'%s': Got current=%.3fA", current1_->get_name().c_str(), current1);
    current2 = (float)ade7953_active_power1 / (ADE7953_PREF / 10);
    ESP_LOGD(TAG, "'%s': Got current=%.3fA", current2_->get_name().c_str(), current2);
    power1 = (float)ade7953_current_rms2 / (ADE7953_IREF * 10);
    ESP_LOGD(TAG, "'%s': Got power=%.2fW", power1_->get_name().c_str(), power1);
    power2 = (float)ade7953_active_power2 / (ADE7953_PREF / 10);
    ESP_LOGD(TAG, "'%s': Got power=%.2fW", power2_->get_name().c_str(), power2);
    
    voltage_->publish_state(voltage);
    current1_->publish_state(current1);
    current2_->publish_state(current2);
    power1_->publish_state(power1);
    power2_->publish_state(power2);

  }
};



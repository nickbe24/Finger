#include "FaulhaberController.h"
#include <Arduino.h>
#include <Arduino_CAN.h> 
#include <EEPROM.h>

void sendSDO(uint8_t nodeId, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size) {
  uint32_t cob_id = 0x600 + nodeId;
  uint8_t data[8] = { 0 };

  switch (size) {
    case 1: data[0] = 0x2F; break; // 1 byte
    case 2: data[0] = 0x2B; break; // 2 bytes
    case 4: data[0] = 0x23; break; // 4 bytes
  }

  data[1] = index & 0xFF;
  data[2] = index >> 8;
  data[3] = subindex;
  memcpy(&data[4], &value, size);

  CAN.write(CanMsg(CanStandardId(cob_id), sizeof(data), data));
}

uint32_t readSDO(uint8_t nodeID, uint16_t index, uint8_t subindex) {
  uint32_t cob_id_req = 0x600 + nodeID;
  uint32_t cob_id_res = 0x580 + nodeID;

  uint8_t request[8] = {
    0x40,               // SDO read command
    (uint8_t)(index & 0xFF),
    (uint8_t)(index >> 8),
    subindex,
    0x00, 0x00, 0x00, 0x00
  };

  CAN.write(CanMsg(CanStandardId(cob_id_req), 8, request));

  unsigned long start = millis();
  while (millis() - start < 100) {
    if (CAN.available()) {
      CanMsg msg = CAN.read();
      if (msg.id == cob_id_res && (msg.data[0] & 0x4F) == 0x43) {
        // Success response: extract 4-byte value from bytes 4–7
        uint32_t value;
        memcpy(&value, &msg.data[4], 4);
        return value;
      }
    }
  }

  Serial.println("⚠️ SDO read timeout or error.");
  return 0xFFFFFFFF;  // Use this as an invalid/error value
}


void enable_op(uint8_t nodeID){

    sendSDO(nodeID, 0x6040, 0x00, 0x06, 2); // Shutdown
  delay(10);

  sendSDO(nodeID, 0x6040, 0x00, 0x07, 2); // Switch On
  delay(10);

  sendSDO(nodeID, 0x6040, 0x00, 0x0F, 2); // Enable Operation
  delay(10);

}

void fault_fix(uint8_t nodeID) {
  // Step 1: Fault Reset (controlword = 0x0080)
  sendSDO(nodeID, 0x6040, 0x00, 0x80, 2);
  delay(10);

  // Step 2: Shutdown (controlword = 0x0006)
  sendSDO(nodeID, 0x6040, 0x00, 0x06, 2);
  delay(10);

  // Step 3: Enable Operation (controlword = 0x000F)
  sendSDO(nodeID, 0x6040, 0x00, 0x0F, 2);
  delay(10);
}


void set_position_mode(uint8_t nodeID) {
  sendSDO(nodeID, 0x6060, 0x00, 1, 1); // Set mode to Profile Position Mode (1)
  delay(10);
}

void set_velocity_mode(uint8_t nodeID) {
  sendSDO(nodeID, 0x6060, 0x00, 3, 1); // Set to Profile Velocity Mode (mode 3)
  delay(10);
}

void set_target_velocity(uint8_t nodeID, int32_t velocity_rpm) {
  sendSDO(nodeID, 0x60FF, 0x00, velocity_rpm, 4); // Target Velocity
  delay(10);
}

int16_t read_motor_current(uint8_t nodeID) {
  uint32_t cob_id_req = 0x600 + nodeID;
  uint32_t cob_id_res = 0x580 + nodeID;

  uint8_t request[8] = {
    0x40, 0x78, 0x60, 0x00, 0, 0, 0, 0 // SDO read of 0x6078
  };

  CAN.write(CanMsg(CanStandardId(cob_id_req), 8, request));

  unsigned long start = millis();
  while (millis() - start < 100) {
    if (CAN.available()) {
      CanMsg msg = CAN.read();

      if (msg.id == cob_id_res && msg.data[0] == 0x4B) {
        int16_t current_mA;
        memcpy(&current_mA, &msg.data[4], 2); // Extract bytes 4–5
        return current_mA;
      }
    }
  }

  //Serial.println("Failed to read motor current.");
  return 0;
}

void home_motor(uint8_t nodeID, int16_t current_threshold_mA, int32_t& min, int32_t& max) {
  int32_t home;
  if (nodeID == 1){
    sendVelocityPDO(nodeID,20);
  }else{
    sendVelocityPDO(nodeID, 500);
  }
  int total = 0;
  int count = 0;
  while (true) {
    int16_t current = read_motor_current(nodeID);
    //Serial.print("Current: ");
    //Serial.print(current);
    //Serial.println(" mA");
    total = total+current;
    count++;
    if (abs(current) >= current_threshold_mA) {
      //Serial.println("Homing threshold reached. Stopping motor.");
      break;
    }

    delay(10);
  }

  // Step 4: Stop the motor
  sendVelocityPDO(nodeID, 0);
  //Serial.print("Average Current: ");Serial.println(total/count);

  //Pull backs and set home/min/max

  if(nodeID == 1){
     set_cyclic_sync_position_mode(1);
     delay(100);
     home = read_encoder_SDO(1);
     max = home+(2.0/360.0)*16*4096; // 27/360 * 16:1 gearing * 4096 resolution encoder;
     min = home-(52.0/360.0)*16*4096;
     sendPositionPDO(1, (int32_t)(home - 5000));
     sendSYNC();
  }else if(nodeID == 2){
    home = read_encoder_SDO(2);
    max = home + 65000; // .5 mm
    min = home - 2752512; // 21 mm
  }else{
    home = read_encoder_SDO(3);
    max = home + 131072; // 1 mm
    min = home - 1836000; // 14 mm 
  }
}


void sendSYNC() {
  uint8_t sync_data[0] = {}; // Empty data
  CAN.write(CanMsg(CanStandardId(0x80), 0, sync_data));
}


void sendVelocityPDO(uint8_t nodeID, int32_t velocity_rpm) {
  uint32_t cob_id = 0x400 + nodeID;

  uint8_t data[6];
  data[0] = 0x0F;
  data[1] = 0x00;
  data[2] = velocity_rpm & 0xFF;
  data[3] = (velocity_rpm >> 8) & 0xFF;
  data[4] = (velocity_rpm >> 16) & 0xFF;
  data[5] = (velocity_rpm >> 24) & 0xFF;

  CanMsg msg(CanStandardId(cob_id), 6, data);
  CAN.write(msg);  
  delayMicroseconds(500); // let the line clear or something idk
}

void set_software_position_limits(uint8_t nodeID, int32_t pos_min, int32_t pos_max) {
  sendSDO(nodeID, 0x607D, 0x01, pos_min, 4);  // min position limit
  delay(10);
  sendSDO(nodeID, 0x607D, 0x02, pos_max, 4);  // max position limit
  delay(10);
}

void sendNMTStart(uint8_t nodeID) {
  uint8_t data[2] = { 0x01, nodeID };
  bool success = CAN.write(CanMsg(CanStandardId(0x000), 2, data));
  if (success) {
    //Serial.print("✅ NMT Start sent to Node ");
    //Serial.println(nodeID);
  } else {
    Serial.println("❌ Failed to send NMT Start");
  }
}

void set_cyclic_sync_velocity_mode(uint8_t nodeID) {
  sendSDO(nodeID, 0x6060, 0x00, 9, 1); // Mode 9 = CSVM
  delay(10);
}

void set_cyclic_sync_position_mode(uint8_t nodeID) {
  sendSDO(nodeID, 0x6060, 0x00, 8, 1); // Mode 8 = CSPM
  delay(10);
}

int32_t read_encoder_SDO(uint8_t nodeID) {
  uint32_t cob_id_req = 0x600 + nodeID; // SDO request
  uint32_t cob_id_res = 0x580 + nodeID; // SDO response

  uint8_t request[8] = {
    0x40, 0x64, 0x60, 0x00, 0, 0, 0, 0  // Read 0x6064: Position Actual Value
  };

  CAN.write(CanMsg(CanStandardId(cob_id_req), 8, request));

  unsigned long start = millis();
  while (millis() - start < 100) {
    if (CAN.available()) {
      CanMsg msg = CAN.read();
      if (msg.id == cob_id_res && msg.data[0] == 0x43) {
        int32_t position;
        memcpy(&position, &msg.data[4], 4); // Bytes 4–7 = int32_t position
        return position;
      }
    }
  }

  Serial.println("⚠️ Failed to read encoder via SDO.");
  return 0;
}

void sendPositionPDO(uint8_t nodeID, int32_t target_pos) {
  uint32_t cob_id = 0x300 + nodeID;  // RxPDO2 COB-ID

  uint8_t data[6];
  data[0] = 0x0F;  // Controlword LSB (Enable + New Setpoint)
  data[1] = 0x00;  // Controlword MSB

  data[2] = target_pos & 0xFF;
  data[3] = (target_pos >> 8) & 0xFF;
  data[4] = (target_pos >> 16) & 0xFF;
  data[5] = (target_pos >> 24) & 0xFF;

  CanMsg msg(CanStandardId(cob_id), 6, data);
  CAN.write(msg);
  delayMicroseconds(500);  // Ensure bus spacing
}

void set_max_velocity(uint8_t nodeID, uint32_t rpm)
{
  sendSDO(nodeID, 0x6080, 0x00, rpm, 4);  // Max profile velocity
}

void set_profile_velocity(uint8_t nodeID, uint32_t rpm)
{
  sendSDO(nodeID, 0x6081, 0x00, rpm, 4);  // Max profile velocity
}

void set_profile_acceleration(uint8_t nodeID, uint32_t acc)
{
  sendSDO(nodeID, 0x6083, 0x00, acc, 4);  // Profile acceleration
}

void set_profile_deceleration(uint8_t nodeID, uint32_t dec)
{
  sendSDO(nodeID, 0x6084, 0x00, dec, 4);  // Profile deceleration
}

void read_encoders_PDO(int32_t positions[3]) {
  const uint8_t nodes[3] = {1, 2, 3};
  const uint16_t base_cob_id = 0x280;
  bool received[3] = {false, false, false};

  // Clear CAN buffer
  while (CAN.available()) CAN.read();

  unsigned long start = micros();

  while (micros() - start < 5000) {  // 5 ms timeout
    if (CAN.available()) {
      CanMsg msg = CAN.read();

      for (int i = 0; i < 3; ++i) {
        if (msg.id == base_cob_id + nodes[i]) {
          int32_t pos;
          memcpy(&pos, &msg.data[2], 4);  // Bytes 2–5 = Position Actual Value
          positions[i] = pos;
          received[i] = true;
        }
      }

      // Early exit if all received
      if (received[0] && received[1] && received[2]) break;
    }
  }

  // Default any missing responses to 0
  for (int i = 0; i < 3; ++i) {
    if (!received[i]) {
      positions[i] = 0;
    }
  }
}

void save_encoder_positions(const int32_t positions[3]) {
  int addr = 0;
  for (int i = 0; i < 3; i++) {
    for (int b = 0; b < 4; b++) {
      EEPROM.write(addr++, (positions[i] >> (8 * b)) & 0xFF);
    }
  }
}

void load_encoder_positions(int32_t positions[3]) {
  int addr = 0;
  for (int i = 0; i < 3; i++) {
    int32_t val = 0;
    for (int b = 0; b < 4; b++) {
      val |= ((int32_t)EEPROM.read(addr++)) << (8 * b);
    }
    positions[i] = val;
  }
}

void send_position_sdo(uint8_t nodeID, int32_t pos) {
  // 1. Set the target position
  sendSDO(nodeID, 0x607A, 0x00, pos, 4);
  delay(2);

  // 2. Set controlword to trigger motion: bit 4 = New Set-point (1), bit 5 = Change Set-point (1)
  // Profile Position Mode expects toggling of bit 4 to latch and execute the position
  // Typical sequence: 0x003F (Enable + Start Set-point)
  sendSDO(nodeID, 0x6040, 0x00, 0x003F, 2);  // Enable op + new setpoint + change setpoint
  delay(2);

  // 3. Clear bit 4 (toggle New Set-point off) to prepare for next command
  sendSDO(nodeID, 0x6040, 0x00, 0x002F, 2);  // Clear New Set-point bit
}



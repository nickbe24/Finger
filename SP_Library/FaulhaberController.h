#pragma once
#include <Arduino.h>

void sendSDO(uint8_t nodeId, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size);

uint32_t readSDO(uint8_t nodeID, uint16_t index, uint8_t subindex);

void enable_op(uint8_t nodeID);

void fault_fix(uint8_t nodeID);

void set_position_mode(uint8_t nodeID);

void set_velocity_mode(uint8_t nodeID);

void set_target_velocity(uint8_t nodeID, int32_t velocity_rpm);

int16_t read_motor_current(uint8_t nodeID);

void home_motor(uint8_t nodeID, int16_t current_threshold_mA, int32_t& min, int32_t& max);

void sendSYNC();

void sendVelocityPDO(uint8_t nodeID, int32_t velocity_rpm);

void set_software_position_limits(uint8_t nodeID, int32_t pos_min, int32_t pos_max);

void sendNMTStart(uint8_t nodeID);

void set_cyclic_sync_velocity_mode(uint8_t nodeID);

void set_cyclic_sync_position_mode(uint8_t nodeID);

int32_t read_encoder_SDO(uint8_t nodeID);

void sendPositionPDO(uint8_t nodeID, int32_t target_pos);

void set_max_velocity(uint8_t nodeID, uint32_t rpm);

void set_profile_velocity(uint8_t nodeID, uint32_t rpm);

void set_profile_acceleration(uint8_t nodeID, uint32_t acc);

void set_profile_deceleration(uint8_t nodeID, uint32_t dec);

void read_encoders_PDO(int32_t pos[3]);

void save_encoder_positions(const int32_t positions[3]);

void load_encoder_positions(int32_t positions[3]);

void send_position_sdo(uint8_t nodeID, int32_t pos);
#include <Arduino_CAN.h>
#include <SP_Hand.h>


int32_t m1_home, m1_start, m1_end, m2_start, m2_end, m3_start, m3_end, m2_home, m3_home,m1_min, m1_max, m2_min, m2_max, m3_min, m3_max;
int32_t cur_pos[3];
int32_t motor_positions[3];

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }

  delay(100);

  // Enable op, enable PDOs, set cyclic velocity mode
  for(int i = 1; i<=3; i++){
    enable_op(i);
    delay(100);
    sendNMTStart(i);
    delay(100);
    set_cyclic_sync_position_mode(i);
    
    delay(100);
  }

  // remove previous limits
  set_software_position_limits(1, -2147483648, 2147483647);
  set_software_position_limits(2, -2147483648, 2147483647);
  set_software_position_limits(3, -2147483648, 2147483647);



  sendSYNC();
  read_encoders_PDO(cur_pos);
  for (int i = 0; i < 3; i++) {
    motor_positions[i] = cur_pos[i];
  }

  save_encoder_positions(cur_pos);
  for (int i = 0; i < 3; i++) {
    Serial.print("Saved Positions: ");Serial.println(cur_pos[i]);
  } 

}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  // Ignore newline and carriage return characters
  if (cmd == '\n' || cmd == '\r') return;

  int8_t motor = 0;
  int32_t delta = 0;

  switch (cmd) {
    case '1': motor = 1; delta = 50; break;
    case '2': motor = 1; delta = -50; break;
    case '3': motor = 2; delta = 10000; break;
    case '4': motor = 2; delta = -10000; break;
    case '5': motor = 3; delta = 10000; break;
    case '6': motor = 3; delta = -10000; break;
    case '7': motor = 2; delta = 1000; break;
    case '8': motor = 2; delta = -1000; break;
    case '9': motor = 3; delta = 1000; break;
    case '0': motor = 3; delta = -1000; break;
    case 'a': motor = 2; delta = -324928; break;
    case 'd': motor = 3; delta = -441713; break;
    case 'j': motor = 2; delta = 324928; break; //324928 219546
    case 'l': motor = 3; delta = 441713; break; //441713 1208222
    default:
      Serial.println("Invalid command.");
      return;
  }
  motor_positions[motor-1] += delta;
  sendPositionPDO(motor, motor_positions[motor - 1]);
  sendSYNC();

  Serial.print("Motor ");
  Serial.print(motor);
  Serial.print(" supposed to move to ");
  Serial.print(motor_positions[motor-1]);
  Serial.print(", actually moved to: ");
  sendSYNC();
  read_encoders_PDO(cur_pos);
  Serial.println(cur_pos[motor-1]);

}

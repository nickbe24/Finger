// Pauses for a few seconds, then executes a task space plan - expects a list of motor positions for the plan.

#include <Arduino_CAN.h>
#include <SP_Hand.h>

// SHOULD PROB BE MADE AS A USER PASS IN
double task_vel = 5; // mm/s
Matrix<50,3> points; // MAX 50 points input
Matrix<50,3> actual_points;
int max_time = 5; // 5 second max time for consecutive points
double dt = .01; // 100 Hz
float error_tol = .1; // x mm in each direction
double vel_limit = 9; // mm/s
double m1_vel_limit = 5400; // 5400 increments per second 
int pointCount = 0;
int32_t m1_home, m1_start, m1_end, m2_start, m2_end, m3_start, m3_end, m2_home, m3_home, m1_min, m1_max, m2_min, m2_max, m3_min, m3_max;
float run_time;
int32_t cur_pos[3];
Matrix<200, 3> ang;          
Matrix<200, 3> motors;  

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
    set_cyclic_sync_velocity_mode(i);
    
    delay(100);
  }

  // remove previous limits
  set_software_position_limits(1, -2147483648, 2147483647);
  set_software_position_limits(2, -2147483648, 2147483647);
  set_software_position_limits(3, -2147483648, 2147483647);


  // HOMING SEQUENCE 
  int32_t loaded[3];
  load_encoder_positions(loaded);
  sendSYNC();
  read_encoders_PDO(cur_pos);
  /*Serial.print("cur pos 1: ");Serial.println(cur_pos[0]);
  Serial.print("cur pos 2: ");Serial.println(cur_pos[1]);
  Serial.print("cur pos 3: ");Serial.println(cur_pos[2]);
  Serial.print("Loaded pos 1: ");Serial.println(loaded[0]);
  Serial.print("Loaded pos 2: ");Serial.println(loaded[1]);
  Serial.print("Loaded pos 3: ");Serial.println(loaded[2]);*/
  m2_home = cur_pos[1]-loaded[1];
  m2_max = m2_home + 65000; // .5 mm
  m2_min = m2_home - 2752512; // 21 mm
  m3_home = cur_pos[2]-loaded[2];
  m3_max = m3_home + 131072; // 1 mm
  m3_min = m3_home - 1836000; // 14 mm
  set_software_position_limits(2, m2_min, m2_max);
  set_software_position_limits(3, m3_min, m3_max);
  set_profile_velocity(2, 5000);
  delay(10);
  set_profile_velocity(3, 5000);
  delay(10);
  set_position_mode(2);
  delay(10);
  set_position_mode(3);
  delay(10);
  send_position_sdo(3,-300000+m3_home);
  delay(1000);
  send_position_sdo(2,-300000+m2_home);
  delay(1000);
  home_motor(1,600,m1_min,m1_max);
  set_software_position_limits(1, m1_min, m1_max);
  delay(5000);
  send_position_sdo(2,m2_home);
  delay(1000);
  send_position_sdo(3, m3_home);
  delay(1000);
  sendSYNC();
  read_encoders_PDO(cur_pos);
  m1_home = cur_pos[0];
  save_encoder_positions(cur_pos);




  Serial.println("READY");  // let matlab know we are ready to roll

}

void loop(){ /* ---------- wait for a full line ---------- */
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line == "END") {
    Serial.print("Received "); Serial.print(pointCount); Serial.println(" points.");
    for (int i = 0; i < pointCount; i++) {
      Serial.print("Point "); Serial.print(i); Serial.print(": ");
      Serial.print(points(i, 0)); Serial.print(", ");
      Serial.print(points(i, 1)); Serial.print(", ");
      Serial.println(points(i, 2));
    }

    // ============================================================== run shit here =============================================================

    set_cyclic_sync_position_mode(1);
    set_cyclic_sync_position_mode(2);
    set_cyclic_sync_position_mode(3);
    delay(100);
    int32_t m1_start_enc = read_encoder_SDO(1) - m1_home;
    //Serial.println(m1_start_enc);
    int32_t m2_start_enc = read_encoder_SDO(2) - m2_home;
    //Serial.println(m2_start_enc);
    int32_t m3_start_enc = read_encoder_SDO(3) - m3_home;
    //Serial.println(m3_start_enc);

    double m2_start_mm = increments_to_mm(m2_start_enc);
    //Serial.println(m2_start_mm);
    double m3_start_mm = increments_to_mm(m3_start_enc);
    //Serial.println(m3_start_mm);

    // =================== calculate forward kinematics to get starting Joint angles ================
    double x_s,y_s,z_s,q1_s,q2_s,q3_s,q4_s;
    FK_3DOF(m1_start_enc, m2_start_mm, m3_start_mm, x_s, y_s, z_s, q1_s, q2_s, q3_s, q4_s);

    //=============== Move to first position in joint space to avoid singularity ================
    Matrix<3> target = { points(0, 0), points(0, 1), points(0, 2) };
    Matrix<3> m_1;
    Matrix<4> a_1;
    IK_3DOF(target, m_1, a_1);

    //Serial.print("Starting Angles: q1 = ");Serial.print(q1_s);Serial.print(" q2 = ");Serial.print(q2_s);Serial.print(" q3 = ");Serial.print(q3_s);Serial.print(" q4 = ");Serial.println(q4_s);
    //Serial.print("Starting Position (x,y,z): (");Serial.print(x_s);Serial.print(", ");Serial.print(y_s);Serial.print(", ");Serial.print(z_s);Serial.println(")");
    double m1_diff = m_1(0) - m1_start_enc;
    double m2_diff = m_1(1) - m2_start_mm;
    double m3_diff = m_1(2) - m3_start_mm;

    double m1_time = abs(m1_diff/m1_vel_limit);
    double m2_time = abs(m2_diff/vel_limit);
    double m3_time = abs(m3_diff/vel_limit);
    double time;
    if(m1_time > m2_time && m1_time > m3_time){
      time = m1_time;
    }else if(m2_time > m3_time && m2_time > m1_time){
      time = m2_time;
    } else{
      time = m3_time;
    }
    double ang_s[3] = {q1_s, q2_s, q3_s};
    double ang_e[3] = {a_1(0), a_1(1), a_1(2)};
    int length = 0;
    bool flag = false;
    while (!flag) {
      lin_interpolate_trajectory(dt, time, ang_s, ang_e, ang, length);

      // Convert angles to motor positions
      for (int i = 0; i <= length; i++) {
          double m1, m2, m3;
          double q1 = ang(i, 0);
          double q2 = ang(i, 1);
          double q3 = ang(i, 2);
          angles2motor_3DOF(q1, q2, q3, m1, m2, m3);
          motors(i, 0) = m1;
          motors(i, 1) = m2;
          motors(i, 2) = m3;
      }

      // Compute finite difference to get motor speeds
      bool exceeds_limit = false;
      for (int i = 0; i < length; i++) {
        for (int j = 0; j < 3; j++) {
            double vel = (motors(i+1, j) - motors(i, j)) / dt;
              if (j == 0){
                if (fabs(vel) > m1_vel_limit){
                  exceeds_limit = true;
                }
              }else{
                if (fabs(vel) > vel_limit) {
                  exceeds_limit = true;
                }
              }
            }
      }

      if (exceeds_limit) {
          time += 0.25;
      } else {
          flag = true;
      }
    }
    unsigned long base_time = micros();
    for(int i = 0; i <= length; i++){
      unsigned long st = micros();
      sendPositionPDO(1, motors(i,0)+m1_home);
      sendPositionPDO(2, mm_to_increments(motors(i,1))+m2_home);
      sendPositionPDO(3, mm_to_increments(motors(i,2))+m3_home);
      sendSYNC();
      unsigned long target = base_time + (unsigned long)((i + 1) * dt * 1e6);
      while ((long)(micros() - target) < 0);  // handles wraparound
    }
    delay(10); // motor has 20 ms settling time
    unsigned long end_time = micros();
    double actual_time = (end_time-base_time)/1000000.0;
    Serial.print("Actual Time: "); Serial.println(actual_time,3);
    int32_t m1_end_enc = read_encoder_SDO(1) - m1_home;
    int32_t m2_end_enc = read_encoder_SDO(2) - m2_home;
    int32_t m3_end_enc = read_encoder_SDO(3) - m3_home;

    double m2_end_mm = increments_to_mm(m2_end_enc);
    double m3_end_mm = increments_to_mm(m3_end_enc);

    Serial.print("Motor 1 was expected to move ");Serial.print(static_cast<double>(m1_diff)*360.0/(4096.0*16.0), 4);Serial.print(" and actually moved ");Serial.print(static_cast<double>(m1_end_enc - m1_start_enc) * 360.0 / (4096.0 * 16.0), 4);Serial.println(" degrees.");
    Serial.print("Motor 2 was expected to move ");Serial.print(m2_diff, 4);Serial.print(" and actually moved ");Serial.print(increments_to_mm(m2_end_enc-m2_start_enc),4);Serial.println(" mm.");
    Serial.print("Motor 3 was expected to move ");Serial.print(m3_diff, 4);Serial.print(" and actually moved ");Serial.print(increments_to_mm(m3_end_enc-m3_start_enc),4);Serial.println(" mm.");

    double x_e,y_e,z_e,q1_e,q2_e,q3_e,q4_e;
    FK_3DOF(m1_end_enc, m2_end_mm, m3_end_mm, x_e, y_e, z_e, q1_e, q2_e, q3_e, q4_e);
    Serial.print("Ending Angles: q1 = ");Serial.print(q1_e);Serial.print(" q2 = ");Serial.print(q2_e);Serial.print(" q3 = ");Serial.print(q3_e);Serial.print(" q4 = ");Serial.println(q4_e);
    Serial.print("Ending Position (x,y,z): (");Serial.print(x_e);Serial.print(", ");Serial.print(y_e);Serial.print(", ");Serial.print(z_e);Serial.println(")");
    //Serial.print("M2 started at: ");Serial.print(m2_start_mm,4);Serial.print(", and ended at: ");Serial.println(m2_end_mm);
    //Serial.print("M3 started at: ");Serial.print(m3_start_mm,4);Serial.print(", and ended at: ");Serial.println(m3_end_mm);

    sendSYNC();
    read_encoders_PDO(cur_pos);
    save_encoder_positions(cur_pos);

    set_cyclic_sync_velocity_mode(1);
    set_cyclic_sync_velocity_mode(2);
    set_cyclic_sync_velocity_mode(3);

    // Prime the PDO communication - wait for 1 or 2 SYNC cycles
    for (int i = 0; i < 3; i++) {
      sendSYNC();
      delay(2); // allow time for device to respond with TPDOs
    }
    int32_t dummy_encs[3];
    read_encoders_PDO(dummy_encs); 



    Serial.println("Now at the first position, starting the path following in 3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("GO");
    sendSYNC();
    read_encoders_PDO(cur_pos);
    save_encoder_positions(cur_pos);
    // =================================================================================================== Now at first position, start the path following =========================================
    // max time, max number of steps, error tolerance defined at the top
    unsigned long st = micros();
    int while_count = 0;
    for (int i = 1; i < pointCount; i++){  //already at point(0), start indexing at 1
      double pos_goal_x = points(i,0);
      double pos_goal_y = points(i,1);
      double pos_goal_z = points(i,2);
      //Serial.print("Goal pos:");Serial.print(pos_goal_x);Serial.print(", ");Serial.print(pos_goal_y);Serial.print(", ");Serial.println(pos_goal_z);
      double error_x = 100;
      double error_y = 100;
      double error_z = 100;
      run_time = 0;
      while ((abs(error_x) > error_tol || abs(error_y) > error_tol || abs(error_z) > error_tol) && run_time * dt < max_time) {


        double pos_diff_x = pos_goal_x - x_e;
        double pos_diff_y = pos_goal_y - y_e;
        double pos_diff_z = pos_goal_z - z_e;
        double magnitude = sqrt(pos_diff_x * pos_diff_x + pos_diff_y * pos_diff_y + pos_diff_z * pos_diff_z);
        if (magnitude < 1e-6) return;

        double dir_x = pos_diff_x / magnitude;
        double dir_y = pos_diff_y / magnitude;
        double dir_z = pos_diff_z / magnitude;

        double lin_vel_x = task_vel * dir_x;
        double lin_vel_y = task_vel * dir_y;
        double lin_vel_z = task_vel * dir_z;
        Matrix<3> lin_vels = {lin_vel_x, lin_vel_y, lin_vel_z};

        Matrix<3, 3> J = J_3DOF(m1_end_enc, m2_end_mm, m3_end_mm, q1_e, q2_e, q3_e, q4_e);
        Matrix<3> md = pinv(J) * lin_vels;

        Matrix<3> scaled_md;
        scaleMotorVelocities(md, vel_limit, scaled_md);
        //Serial.print("Scaled Motor Velocities; ");Serial.print(scaled_md(0));Serial.print(", ");Serial.print(scaled_md(1));Serial.print(", ");Serial.println(scaled_md(2));

        //sent_vels(while_count,0) = scaled_md(0)*16*60/(2*3.14159);
        //sent_vels(while_count,1) = scaled_md(1)*2*16*60;
        //sent_vels(while_count,2) = scaled_md(2)*2*16*60;

        sendVelocityPDO(1, scaled_md(0)*16*60/(2*3.14159)); // rad/s to RPM
        sendVelocityPDO(2, scaled_md(1)*2*16*60); // converts from mm/s to RPM
        sendVelocityPDO(3, scaled_md(2)*2*16*60); // ^^
        sendSYNC();
        int32_t encs[3];
        read_encoders_PDO(encs);
        m1_end_enc = encs[0] - m1_home;
        m2_end_enc = encs[1] - m2_home;
        m3_end_enc = encs[2] - m3_home;

        

        m2_end_mm = increments_to_mm(m2_end_enc);
        m3_end_mm = increments_to_mm(m3_end_enc);

        FK_3DOF(m1_end_enc, m2_end_mm, m3_end_mm, x_e, y_e, z_e, q1_e, q2_e, q3_e, q4_e);
        //intermed_pts(while_count,0) = x_e;
        //intermed_pts(while_count,1) = y_e;
        //intermed_pts(while_count,2) = z_e;

        error_x = pos_goal_x - x_e;
        error_y = pos_goal_y - y_e;
        error_z = pos_goal_z - z_e;

        run_time++;
        while_count++;

        
      }

      // store ending pos in an array
      //Serial.println("Point reached, going to the next one!");
      actual_points(i,0) = x_e;
      actual_points(i,1) = y_e;
      actual_points(i,2) = z_e;

      // move to next point  
    }

    sendVelocityPDO(1, 0); 
    sendVelocityPDO(2, 0); 
    sendVelocityPDO(3, 0); 
    sendSYNC();

    unsigned long et = micros();
    for(int c = 1; c < pointCount; c++){
      Serial.print("Actual Point "); Serial.print(c); Serial.print(": ");
      Serial.print(actual_points(c, 0)); Serial.print(", ");
      Serial.print(actual_points(c, 1)); Serial.print(", ");
      Serial.println(actual_points(c, 2));
    }

    Serial.print("Time Elapsed: ");Serial.print((et - st) / 1000000.0, 4);Serial.println(" s");
    Serial.print("Loops Elapsed: ");Serial.println(while_count);

    /*for(int q = 0; q < while_count; q++){
      Serial.print("Loop # ");Serial.print(q);Serial.print("'s sent vels were: ");Serial.print(sent_vels(q,0));Serial.print(", ");Serial.print(sent_vels(q,1));Serial.print(", ");Serial.print(sent_vels(q,2));
      Serial.print(", and recorded position was: ");Serial.print(intermed_pts(q,0));Serial.print(", ");
      Serial.print(intermed_pts(q,1));Serial.print(", ");Serial.println(intermed_pts(q,2));
    }
    intermed_pts.Fill(0.0f);
    sent_vels.Fill(0.0f); */
    pointCount = 0;
    return;
    sendSYNC();
    read_encoders_PDO(cur_pos);
    save_encoder_positions(cur_pos);
  }

  int space1 = line.indexOf(' ');
  int space2 = line.indexOf(' ', space1 + 1);
  if (space1 == -1 || space2 == -1) {
    Serial.print("ERR: "); Serial.println(line);
    return;
  }

  double x = line.substring(0, space1).toDouble();
  double y = line.substring(space1 + 1, space2).toDouble();
  double z = line.substring(space2 + 1).toDouble();

  if (pointCount < 50) {
    points(pointCount, 0) = x;
    points(pointCount, 1) = y;
    points(pointCount, 2) = z;
    pointCount++;
  }


}


void lin_interpolate_trajectory(double dt, double time, double ang_s[3], double ang_e[3], Matrix<200, 3>& ang,int& length_out){

  int l = ceil(time / dt);
  length_out = l;

  for (int i = 0; i <= l; i++) {
      double ti = i * dt;
      for (int j = 0; j < 3; j++) {
          double start = ang_s[j];
          double end = ang_e[j];
          if (fabs(end - start) < 1e-6) {
              ang(i, j) = end;
          } else {
              double delta = (end - start) / l;
              ang(i, j) = start + i * delta;
          }
      }
  }
}



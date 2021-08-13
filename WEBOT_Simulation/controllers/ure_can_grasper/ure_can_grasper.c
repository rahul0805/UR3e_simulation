/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/display.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 32

enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };


static void val_show(int label, const char *msg, double v) {
  char str[128];
  sprintf(str, "%s %7.3f", msg, v);
  wb_supervisor_set_label(label, str, 0.01, 0.01 + 0.05 * label, 0.1, 0xffffff, 0.0, "Arial");
}
static void vec_show(int label, const char *msg, const double v[3]) {
  char str[128];
  sprintf(str, "%s %7.3f %7.3f %7.3f", msg, v[0], v[1], v[2]);
  wb_supervisor_set_label(label, str, 0.01, 0.01 + 0.05 * label, 0.1, 0x0000ff, 0.0, "Arial");
}
// distance between 2 vectors
static double vec_dist(const double a[3], const double b[3]) {
  double d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  return sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

// vector addition
static void vec_add(double result[3], const double b[3]) {
  result[0] += b[0];
  result[1] += b[1];
  result[2] += b[2];
}

// matrix * vector multiplicarion: result = m * v
void vec_rotate(double result[3], const double m[9], const double v[3]) {
  result[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
  result[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
  result[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}
static void step() {
  const double time_step = wb_robot_get_basic_time_step();
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}
int dist;
int main(int argc, char **argv) {
  wb_robot_init();
  int counter = 0, i = 0;
  int state = WAITING;
  const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};
  double speed = 1.0;

  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");
  for (i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed/10);

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);
  
  WbNodeRef gripper = wb_supervisor_node_get_from_def("Gripper");
  WbNodeRef can1 = wb_supervisor_node_get_from_def("can1");
    //val_show(0, "distance:", ur_motors[0]);
  while (wb_robot_step(TIME_STEP) != -1) {
  const double *gpos = wb_supervisor_node_get_position(gripper);
  const double *tpos = wb_supervisor_node_get_position(can1);

    // get gripper's current 3x3 rotation matrix
    const double *m3x3 = wb_supervisor_node_get_orientation(gripper);

    // center point of the gripper in local (WRIST) coordinates
    const double center[3] = {0, 0.18, 0};

    // change center point from WRIST to world coordinates
    // cpos = m3x3 * center + gpos
    double cpos[3];
    vec_rotate(cpos, m3x3, center);
    vec_add(cpos, gpos);
    vec_show(1, "gripper: ", gpos);
    counter = 0;
    if (counter <= 0) {
      switch (state) {
        case WAITING:
        for (i = 0; i < 4; ++i){
            wb_motor_set_position(ur_motors[i], target_positions[i]);
            step();}
          printf("Rotating arm\n");
          state = ROTATING;
          step();
          break;

          
        case ROTATING:
          if (wb_distance_sensor_get_value(distance_sensor) < 500) {
            state = GRASPING;
            counter = 8;
            printf("Grasping can\n");
            for (i = 0; i < 3; ++i){
              wb_motor_set_position(hand_motors[i], 0.85);
              step();}
              step();
          }
          break;
          
        case GRASPING:
          for (i = 0; i < 4; ++i){
              wb_motor_set_position(ur_motors[i], 0.0);
              step();}
          printf("Rotating arm back\n");
          state = ROTATING_BACK;
          step();
          break;
        case ROTATING_BACK:
        if (wb_position_sensor_get_value(position_sensor) < -2.3) {
            counter = 8;
            printf("Releasing can\n");
            state = RELEASING;
            for (i = 0; i < 3; ++i){
              wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));
              step();
               }
           step();
          }
          break;
        case RELEASING:
          if (wb_position_sensor_get_value(position_sensor) > -0.1) {
            state = WAITING;
            printf("Waiting can\n");
          }
          break;
      }
    vec_rotate(cpos, m3x3, center);
    vec_add(cpos, gpos);
    vec_show(1, "gripper: ", gpos);
    vec_show(2, "can1: ", tpos);
    dist = 0;
    }

    vec_rotate(cpos, m3x3, center);
    vec_add(cpos, gpos);
    vec_show(1, "gripper: ", gpos);
    vec_show(2, "can1: ", tpos);
    dist = 0;

    counter--;
  };
  // 0.973761 1.03064 -0.098263
  //0.843761  0.98064  -0.088263
  // 0.843761  0.912902 -0.098263
  wb_robot_cleanup();
  return 0;
}

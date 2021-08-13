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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 32
#define NUMBER_OF_JOINTS 4
#define NUMBER_OF_JOINTS1 4
static void step() {
  const double time_step = wb_robot_get_basic_time_step();
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}
/*
// Movement decomposition
static void movement_decomposition(const double *target, double duration) {
  const double time_step = wb_robot_get_basic_time_step();
  const int n_steps_to_achieve_target = duration * 1000 / time_step;
  double step_difference[NUMBER_OF_JOINTS];
  double current_position[NUMBER_OF_JOINTS];

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    current_position[i] = wb_motor_get_target_position(ur_motors[i]);
    step_difference[i] = (target[i] - current_position[i]) / n_steps_to_achieve_target;
  }

  for (int i = 0; i < n_steps_to_achieve_target; ++i) {
    for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
      current_position[j] += step_difference[j];
      wb_motor_set_position(motors[j], current_position[j]);
    }
    step();
  }
}
*/
enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };
int flag =0;
// const double initial_time = wb_robot_get_time();
//  while (wb_robot_get_time() - initial_time < 8)
double initial_time;
int main(int argc, char **argv) {
  wb_robot_init();
  int counter = 0, i = 0;
  int state = WAITING;
  const double target_positions[] = {0.0633 ,-0.0102 ,0.0370 ,-0.0268 ,0 ,-0.4582};
  
  
  
  //const double target_positions[] = {0, -2.88, 0.7, -1.08, -2.38, 0};
  //const double target_positions[] = {0,-1.88, -2.14, -2.38, -1.51,0};
  //const double target_positions[] = {0,0,0,0,0,0};
  
  //const double target_positions[] = {-0.4582,-0.0102 ,0.0370 ,-0.0268 ,0 ,0.0633};
  //const double target_positions[] = { 0.4075,-0.3806 ,0 ,3.5222, 0 ,-0.4255};
  double speed = 0.10;

  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  WbDeviceTag ur_motors[6];
  ur_motors[0] = wb_robot_get_device("shoulder_pan_joint");
  ur_motors[1] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[2] = wb_robot_get_device("elbow_joint");
  ur_motors[3] = wb_robot_get_device("wrist_1_joint");
  ur_motors[4] = wb_robot_get_device("wrist_2_joint");
  ur_motors[5] = wb_robot_get_device("wrist_3_joint");
  
  /*
  for (i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);
  */

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);
  double pp;
  double ini;
  const double initial_time = wb_robot_get_time();
  while (wb_robot_get_time() - initial_time < 8) {
 
   for (i = 0; i < 6; ++i)
     wb_motor_set_position(ur_motors[i], target_positions[i]);
    if(flag==0){
    ini = wb_robot_get_time();
    printf("%lf\n",ini);
    }
          //break;
          printf("Rotating arm\n");
          //state = ROTATING;
    //wb_motor_set_position(ur_motors[0],  2*sin(2 * wb_robot_get_time()) + 0.6);
    //pp = 2*sin(2 * wb_robot_get_time()) + 0.6;
    //printf("%lf\n",pp);  // Upperarm movement
    //wb_motor_set_position(motors[5], 0.4 * sin(2 * wb_robot_get_time()));        // Forearm movement
    step();
  }

  wb_robot_cleanup();
  return 0;
}
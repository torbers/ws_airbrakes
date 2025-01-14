#include "main.h"

void state::updateState () {

  // Get global acceleration values

  // Temporary quaternion values
  float temp_w;
  float temp_x;
  float temp_y;
  float temp_z;

  // Use quaternions to find global acceleration

  temp_w = qw * 0 - qx * ax_local - qy * ay_local - qz * az_local;
  temp_x = qw * ax_local + qx * 0 + qy * az_local - qz * ay_local;
  temp_y = qw * ay_local - qx * az_local + qy * 0 + qz * ax_local;
  temp_z = qw * az_local + qx * ay_local - qy * ax_local + qz * 0;

  ax = temp_w * (-qx) + temp_x * qw + temp_y * (-qz) - temp_z * (-qy);
  ay = temp_w * (-qy) - temp_x * (-qz) + temp_y * qw + temp_z * (-qx);
  az = temp_w * (-qz) + temp_x * (-qy) - temp_y * (-qx) + temp_z * qw - gravity;
  

  // Update velocity values

  vx += ax * delta_t;
  vy += ay * delta_t;
  vz += az * delta_t;

  // Update position values (need to implement Kalman filter, this uses simple complimentary filter)

  x += vx * delta_t;
  y += vy * delta_t;
  z += vz * delta_t;

  // Very simple complimentary filter
  
  altitude = 0.9 * (altitude + vz * delta_t) + 0.1 * baroAltitude; 
}
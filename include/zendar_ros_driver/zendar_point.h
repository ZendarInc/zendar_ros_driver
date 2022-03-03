#pragma once

#include <pcl/register_point_struct.h>

namespace zen {
struct ZendarPoint {
  float x;
  float y;
  float z;
  double x_ecef;
  double y_ecef;
  double z_ecef;
  float mag;
  float az_var;
  float el_var;
  float r;
  float rad_vel;
  float az;
  float el;
  float doa_snr_db;
  float rd_mean_snr_db;
};
}  // namespace zen

POINT_CLOUD_REGISTER_POINT_STRUCT(
    zen::ZendarPoint,
    (float, x, x)(float, y, y)(float, z, z)(double, x_ecef,
                                            x_ecef)(double, y_ecef, y_ecef)(
        double, z_ecef, z_ecef)(float, mag, mag)(float, az_var,
                                                 az_var)(float, el_var, el_var)(
        float, r, r)(float, rad_vel, rad_vel)(float, az, az)(float, el, el)(
        float, doa_snr_db, doa_snr_db)(float, rd_mean_snr_db, rd_mean_snr_db));

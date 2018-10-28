//
// Created by chen on 18-7-18.
//

#ifndef LOCALIZER_FLAGS_H
#define LOCALIZER_FLAGS_H

#include <global.hpp>

DECLARE_string(map_prefix);
DECLARE_bool(debug);
DECLARE_string(transform);
DECLARE_int32(num_gps);
DECLARE_int32(num_imu);
DECLARE_bool(use_markers);
DECLARE_int32(marker_intensity);
DECLARE_double(std_var_gnss);
DECLARE_double(std_var_heading);
DECLARE_double(std_var_wheel_speed);
DECLARE_double(std_var_yaw_velocity);
DECLARE_double(shift_x);
DECLARE_double(shift_y);
DECLARE_double(shift_yaw);
DECLARE_double(utm_x);
DECLARE_double(utm_y);

#endif //LOCALIZER_FLAGS_H

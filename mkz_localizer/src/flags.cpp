//
// Created by chen on 18-7-18.
//

#include "flags.h"

DEFINE_string(map_prefix, "/data/changpin/2018_07_13_11_31_00_resampled_", "prefix of the path of the map files");
DEFINE_bool(debug, true, "publish features points or not");
DEFINE_string(transform, "-0.896890049116527,-0.442253592179594,0,0,0.285459954799665,-0.578912636117389,0.763788435333626,0,-0.337788179191528,0.685034247281011,0.645466673075079,0,-2157470.042367,4375348.095488,4095029.576024,1", "transform string in the map rpp file");
DEFINE_int32(num_gps, 0, "number of gps particles");
DEFINE_int32(num_imu, 400, "number of imu particles");
DEFINE_bool(use_markers, false, "if true, the markers will be used; else, the markers will not be used");
DEFINE_int32(marker_intensity, 35, "marker intensity threshold, intensity higher than this threshold will be identified as markers");
DEFINE_double(std_var_gnss, 2.0, "standard variance of the gnss position, in meter");
DEFINE_double(std_var_heading, 0.1047, "standard variance of the gnss heading, in radian");
DEFINE_double(std_var_wheel_speed, 0.1, "standard variance of the wheel speed, in m/s");
DEFINE_double(std_var_yaw_velocity, 0.0001, "standard variance of the yaw velocity, in rad/s");

DEFINE_double(shift_x, 0.0, "shift x of vector map");
DEFINE_double(shift_y, 0.0, "shift y of vector map");
DEFINE_double(shift_yaw, 0.0, "shift yaw of vector map");
DEFINE_double(utm_x, 435785.8311, "utm origin x shift");
DEFINE_double(utm_y, 4450415.544, "utm origin y shift");

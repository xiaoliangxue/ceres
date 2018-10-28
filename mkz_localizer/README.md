## lidar based localizer for CAINIAO mkz
1. subscribe to gnss/position, gnss/velocity, middle rslidar, wheel odometry to get sensor info, subscribe to patch to get map;
2. extract features from combined lidar scan;
3. localize the scan with points map;
4. fuse lidar scan and yaw velocity, gnss velocity by ekf;
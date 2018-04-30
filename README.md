# lRF_IMU_Localizaiton_Mapping
Lidar odometry and Mapping method using a 2D tilting lidar and IMU.(2016)

We designed a Lidar odometry and Mapping method using a 2D tilting lidar and IMU. A person can hold the Lidar module and work indoor to reconstruct the 3D indoor sense. Lidar scans at 1HZ, while IMU is integrating the angular rates to get orientation at high frequency. The key idea of our method is to compensate the Lidar odometry rotation error using EKF.
![image](http://ltwang.xyz/assets/post_images/2a.jpg)
Useage please refer to doc file in root folder (LRFSensor and IMU.docx).

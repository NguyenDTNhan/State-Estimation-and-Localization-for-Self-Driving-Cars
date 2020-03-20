# State-Estimation-and-Localization-for-Self-Driving-Cars
In the World of increasing Digitalization, Autonomous Driving is an essential part to boost up the IoT application. To navigate reliably, autonomous vehicles require an estimate of their pose (position and orientation) in the world (and on the road) at all times. Much like for modern aircraft, this information can be derived from a combination of GPS measurements and inertial navigation system (INS) data. In recent years, LIDAR (light detection and ranging) sensing is an enabling technology for self-driving vehicles. LIDAR sensors can ‘see’ farther than cameras and are able to provide accurate range information. 

In the following section, you will finds the source codes and description for Autonomous Driving Car project I participated. The sections are going from simple concept such as Least Square Estimation of vehicle to more detail and realistic state estimation using sensors measurements. In the final section, you are given the state estimation of a full vehicle that use data from the CARLA simulator, an error-state extended Kalman filter-based estimator that incorporates GPS, IMU and LIDAR measurements (sensor fusion).

# Least Square Method
This section introduces you to the most simple concept in estimation theory: Least Square Method. The code is in examine the relationship between Voltage and Current going through a resistor by Ohm's Law. Even though this seems to be easy, it contains a lot of theory of Batch and Recursive Least Squares, Unweighted and Weighted Least Squares. This allow the analysis from simple assumption (constant uncertainty of measurement: Unweighted) of the relationship to more realistic assumption (measurement dependent uncertainty: Weighted). Besides, the technique to transform from traditional 'batch' least squares estimator to a recursive form that is suitable for online and real-time estimation is also presented in the source codes.

# LIDAR
This section applies a well-known concept Kalman filter to estimate the position and orientation (states) of vehicle in 2D plane. The example measurements are given for you to assess the performance of the estimation of state by Extended Kalman filter.

The vehicle is equipped with a very simple type of LIDAR sensor, which returns range and bearing measurements corresponding to individual landmarks in the environment. The global positions of the landmarks are assumed to be known beforehand. We will also assume known data association, that is, which measurements belong to which landmark.

# INS_GNSS_LIDAR
This section estimates the states (Location, Velocity and Pose) of Car, Robot, People by using INS, GNSS and LIDAR. The goal of this section is to develop a full vehicle state estimator. You will see how we build, using data from the CARLA simulator, an error-state extended Kalman filter-based estimator that incorporates GPS, IMU, and LIDAR measurements to determine the vehicle position and orientation on the road at a high update rate. There will be an opportunity to observe what happens to the quality of the state estimate when one or more of the sensors either 'drop out' or are disabled.

# ENJOY _^_

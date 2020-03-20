# State-Estimation-and-Localization-for-Self-Driving-Cars
In the World of increasing Digitalization, Autonomous Driving is an essential part to boost up the IoT application. In the following section, you will finds the source codes and description for Autonomous Driving Car project I participated.

# Least Square Method
This module introduces you to the most simple concept in estimation theory: Least Square Method. The code is in examine the relationship between Voltage and Current going through a resistor by Ohm's Law. Even though this seems to be easy, it contains a lot of theory of Batch and Recursive Least Squares, Unweighted and Weighted Least Squares. This allow the analysis from simple assumption (constant uncertainty of measurement: Unweighted) of the relationship to more realistic assumption (measurement dependent uncertainty: Weighted). Besides, the technique to transform from traditional 'batch' least squares estimator to a recursive form that is suitable for online and real-time estimation is also presented in the source codes.

# LIDAR
This module applied a well-known concept Kalman filter to estimate the position and pose of vehicle in 2D plane. The example measurements are given for you to examine the estimation of Kalman filter. The codes recursively estimate the position and pose of the vehicle by information of motion model.

The vehicle is equipped with a very simple type of LIDAR sensor, which returns range and bearing measurements
corresponding to individual landmarks in the environment. The global positions of the landmarks are assumed to be
known beforehand. We will also assume known data association, that is, which measurements belong to which landmark.

# INS_GNSS_LIDAR
Estimate the States (Location, Velocity and Pose) of Car, Robot, People by using INS, GNSS and Lidar
	

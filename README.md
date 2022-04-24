# Cooperative-Localization-of-Three-Vehicles
* Multi-Sensor Data Fusion, Estimation for Robot Navigation

- Team of 2 students:

•	In this project, we present three methods for estimating the states and their associated covariance of three vehicles moving around a roundabout (in Compiègne), the dataset is real and taken from the GNSS and sensors implanted on the vehicles.

•	All vehicles are able to:
  -	Measure their global pose using a GNSS receiver
  -	Measure their linear and angular velocities using embedded sensors
  -	Measure the relative poses of the two other vehicles using a 360° LiDAR
  -	Provide a ground-truth state for validation purposes

•	The project was divided into three parts:
  1.	Standalone Extended Kalman Filter (EKF) Perception
    o	In this part, we want the three vehicles to independently filter their pose and the pose of the two others using only embedded sensors (with no information exchange between vehicles).
  2.	Cooperative EKF
    o	In this part, we consider that the three vehicles exchange their perception i.e., the states they have estimated about others without sharing their own state.
  3.	Cooperative EK/CI Filter
    o	In order to solve the correlation of measurement errors seen before, we want to replace some Kalman updates with CI (Covariance Intersection) updates, as they are resilient to these errors.

•	Output of the project: 
  o	Three MATLAB files:
    o	main_standalone.m
    o	main_cooperative_EKF.m
    o	main_cooperative_CI.m
  o	A screen recording of the three filters working and of their final results
  o	A short report (personal interpretation).

•	Skills token from this project:
-	MATLAB
-	KF, EKF, CIF 



![image](https://user-images.githubusercontent.com/85926752/164971766-bcd4dd2e-3242-41ee-8d93-aba39343f229.png)


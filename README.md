# Kalman_estimation
This repository focusses on making different modules of kalman filter in simulink, python and arduino version

-------

Kalman filters are the state estimation method of eliminating noises recieved by the sensors and giving a very close estimation of actual state values. Kalman filters works based on assuming noises to be of Gaussian distribution and so deals with covariances. 

Simply a Kalman filter deals with a real life system with alot of sensor noises and so to overcome this situation, Kalman filter uses two sets of dynamical model of the system, one is the actual measurement output given by the sensors with noises and the other is a similar dynamical model but only thing is that it prior estimates the states with no noises based on previous timestep's posteriori estimate i.e final estimate of kalman filter eliminating noise. Also this process internally is solved by the Kalman algorithm by solving the Ricatti Equation to get the feedback Kalman gain whose optimal value is when the ricatti equation solves for minimum posteriori error covariance P.

![image](https://user-images.githubusercontent.com/69386934/125238176-7c512880-e304-11eb-8ee6-03212e69f126.png)

Look into the image above, here the main motive of Kalman filter is to estimate X_hat at any timestep k. x_hat_dash is called a prior state estimate and this added with Kalman gain and measurement value gives actual X_hat also called posteriori state estimate. Here P as mentioned earlier is error covariance, which algorithm tries to minimize as this eliminates the effect of noise in the estimation. Then in the update phase we update the Kalman gain value for each timestep. This is how a standard Kalman filter works assuming that the system on which its working is a linear system. For in-depth mathematics behind Kalman filters refer to the tutorial in Reference section.

----

## Reference ##

1. Kalman filter Matlab tech talks - https://in.mathworks.com/videos/understanding-kalman-filters-part-1-why-use-kalman-filters--1485813028675.html
2. UAV model used - https://arxiv.org/abs/1908.07401

In this arduino code, a 1D rig was been balanced by using two bldc motors at both the ends and battery and an arduino at the center of gravity of the rig. The code mainly comprises of getting gyro and accelerometer values from MPU6050 to estimate the attitude of the rig, also then bldc motors are then controlled based on PID control to stabilize the rig. Also a reciever interrupt is used so that using a transmitter if we give any disturbance as an input still the rig will manage to stabilize.

Now since the rig is not a very rigid system and also the MPU6050 imu was placed such that it vibrates alot so the values it gave was tremendously noisy after using just a complementary filter. Thus for this purpose Kalman logic was used for both gyro and accelerometer outputs individually.

KF_gyro() and KF_acc functions are made to implement kalman logic by calculating ricatti equation for each timestep and for each sensor gyro and accelerometer.

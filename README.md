# Extended Kalman Filter in C++
(Udacity self driving car nanodegree term #2 - project #1)

With simulated lidar and radar measurements detecting a bicycle that travels around your vehicle we will use a Kalman filter, lidar measurements and radar measurements to track the bicycle's position and velocity.

#### Result:
px, py, vx, and vy RMSE RMSE values for input laser-radar-measurement-data #1:
0.0651649
0.0605378
0.54319
0.544191

px, py, vx, and vy RMSE RMSE values for input aser-radar-measurement-data #2:
0.185465
0.190254
0.476509
0.810787

### Visualization:

![Alt text](/Sample-files/EKF_plot.png?)

### Details:

#### To compile:
From the root of the repo:
1. mkdir build && cd build
2. cmake .. && make
3. ./ExtendedKF path/to/input.txt path/to/output.txt

#### Here is a brief overview of what happens when you run code files:

1. Main.cpp reads in the data and sends a sensor measurement to FusionEKF.cpp
2. FusionEKF.cpp takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. FusionEKF.cpp has a variable called ekf_, which is an instance of a KalmanFilter class. The ekf_ will hold the matrix and vector values. We will also use the ekf_ instance to call the predict and update equations.
3. The KalmanFilter class is defined in kalman_filter.cpp and kalman_filter.h;'kalman_filter.cpp' contains functions for the prediction and update steps.

#### Main.cpp will output the following results to a text file:

1. the ground truth
2. sensor measurements
3. your filter's belief about location and velocity

#### Summary of main.cpp
1. reads in the data file and stores the data into ground truth and sensor measurement lists.
2. iterates through the data sending each sensor measurement to the Kalman filter
3. stores the ground truth and kalman Filter position and velocity in lists
4. outputs RMSE

Every time main.cpp calls fusionEKF.ProcessMeasurement(measurement_pack_list[k]), the code in FusionEKF.cpp will run. - If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

#### Predict and Update Steps in FusionEKF.cpp:
Once the Kalman filter gets initialized, the next iterations of the for loop will call the ProcessMeasurement() function to do the predict and update steps.

In FusionEKF.cpp, you will see references to a variable called ekf_. The ekf_ variable is an instance of the KalmanFilter class. You will use ekf_ to store your Kalman filter variables (x, P, F, H, R, Q) and call the predict and update functions. 

#### KalmanFilter Class:
kalman_filter.h defines the KalmanFilter class containing the x vector as well as the P, F, Q, H and R matrices. The KalmanFilter class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

kalman_filter.cpp contains prediction and update equations. 

Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. The Update function will use the standard Kalman filter equations. The UpdateEKF will use the extended Kalman filter equations

#### Tools.cpp
Contains root mean squared error and the Jacobian matrix equations.

#### Kalman equation:
		 /*
			 * KF Measurement update step
			 */
			VectorXd y = z - H * x; // Hj (jacobian) for Radar 
			MatrixXd Ht = H.transpose();
			MatrixXd S = H * P * Ht + R;
			MatrixXd Si = S.inverse();
			MatrixXd K =  P * Ht * Si;

			//new state
			x = x + (K * y);
			P = (I - K * H) * P;

			/*
			 * KF Prediction step
			 */
			x = F * x + u;
			MatrixXd Ft = F.transpose();
			P = F * P * Ft + Q;
    
#### Reference:
1. https://home.wlu.edu/~levys/kalman_tutorial/
2. https://medium.com/@tjosh.owoyemi/kalman-filter-predict-measure-update-repeat-20a5e618be66
3. https://medium.com/towards-data-science/kalman-filter-intuition-and-discrete-case-derivation-2188f789ec3a


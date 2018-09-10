## Week 2: Assembling the Rover

#### Flashing your Raspberry Pi SD Card (copied from Coursera)

First download the Pi image. Once you have downloaded the image, use the tutorial matching your operating system to help you flash the Pi image onto your SD card (you will need to uncompress the file). When this is done, insert the SD card into the Pi before assembly.

Please see the video on connecting to the Pi for connection instructions.

Linux installation instructions:

[https://www.raspberrypi.org/documentation/installation/installing-images/linux.md](https://www.raspberrypi.org/documentation/installation/installing-images/linux.md)

The linked instructions suggested to install the program **Etcher** to flash the SD card. This method worked seamlessly.

#### Connecting to the Pi

You can use SSH to connect to your Raspberry Pi from a Linux computer, a Mac, or another Raspberry Pi, without installing additional software.

You will need to know your Raspberry Pi's IP address to connect to it. To find this, type `hostname -I` from your Raspberry Pi terminal.

To connect to your Pi from a different computer type `ssh pi@<IP>` where `<IP>` is replaced with your specific IP address.

The IP address for my Raspberry Pi is `192.168.1.11` and the default password is `raspberry`.

#### A Basic Program to Move the Rover (copied from Coursera)

Now that you've set up ssh for the Pi, ssh into the Pi, and navigate to the folder:

```
/home/pi/catkin_ws/src/robot_control/src/
```

In this folder, you will find the function `RobotControl.py`. This will be your main script for controlling the Pi. It imports the `ROSInterface` class, which interacts with the Robotic Operating System framework and gathers information from the sensors as well as sending information to the motors.

Using your preferred file editor, edit `RobotControl.py`. The function you will work with is `process_measurements()`, at line 39. This function is called at 60Hz when the script is run, and is very similar to the one you used for the simulator. The interface commands are the same as the simulator, except we now use `self.ros_interface` instead of self.robot_sim. The three functions are: `self.ros_interface.get_measurements()`, `self.ros_interface.get_imu()` and `self.ros_interface.command_velocity(lin_vel, ang_vel)`

They have the same inputs and outputs as in the simulator. To test moving the rover, let's command the robot to move with a linear velocity of 30cm/s. That is:

```
self.ros_interface.command_velocity(0.3, 0)
```

Save the file, and move to the next section, where we will run this piece of code.

#### Running Your Code (copied from Coursera)

You will need two terminal windows to run your code. To do this, you can just ssh to the Pi in two separate windows.

In the first terminal window, run the following command:

```
$ roslaunch robot_launch robot.launch
```

The `robot.launch` file contains the packages that control the robot. Namely, they are the AprilTag detector, the IMU driver, the camera driver and the motor driver. You won't need most of them except the motor driver for this reading, but it doesn't hurt to run them all.

In the second terminal window, launch your `RobotControl` code with the following:

`$ roslaunch robot_control robot_control.launch`

With the code from the previous section, you should see the rover move forwards. To stop the rover, type `Ctrl-C` to kill the program. Don't worry if the speed is too quick or too slow, we will calibrate this next week.

## Week 3: Calibration

#### Camera calibration

Print out the provided checkerboard of alternating black and white squares. Measure the length of the squares and note this for later. My squares had length 0.02 meters, but it could also be something close to this value. Also count the number of squares in each dimension of the checkerboard. In the tutorial video, the checkerboard was size 11x9. My checkerboard was size 9x7.

To begin calibration, open up two terminal windows in your Raspberry Pi. In one terminal window, run the launch file that starts the ROS camera node (called `raspicam_node`):

```
$ roslaunch robot_launch robot.launch
```

This also starts other nodes for the april tag detector, motor driver, and IMU even though we will not be using them here (which is okay). In the other terminal window, start the camera calibration program:

```
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.02 image:=/camera/image_raw camera:=/camera
```

The input argument for `--size` will vary depending on your checkerboard. For example, in the tutorial video where the checkerboard is 11x9 I would input the size as `10x8`. This convention for the camera calibration program is to ignore the outer squares when counting the size of the checkerboard. Similarly, for my checkerboard of size 9x7, I would input the size as `8x6`. The input argument for `--square` will also depend on your checkerboard measurement. The input `--square 0.02` means each square has a length of 0.02 meters.

After running the above command, a new display window will open showing the camera output. On the right side of the window will display three buttons: calibrate, save, and commit. These buttons cannot be clicked until the calibration is ready. Now place your checkerboard in the field of view of the camera. If the camera recognizes the checkerboard, the right side of the window should automatically pop up four categories: **X**, **Y**, **Size**, and **Skew**. Moving the checkerboard around in front of the camera should increase the  calibration level for each category. Periodically, the camera will sample the image at specific points to use for calibration. Concretely, here is how to increase each of the calibration levels:

* X: Move the checkerboard left and right.
* Y: Move the checkerboard up and down.
* Size: Move the checkerboard closer and farther away from the camera.
* Skew: Rotate the checkerboard around in different axes individually and simultaneously (X, Y, and Z).

Once the calibration level is ready for all four categories (they should all be green), the **CALIBRATE** button should be highlighted. Clicking on the **CALIBRATE** will automatically calculate the calibration parameters and save them in a file so they are ready to be used. For our purposes, the **SAVE** and **COMMIT** buttons will not be used. The output in the terminal window will show the calculated calibration parameters:

```
('D = ', [0.079..., 0.020..., ... ])
('K = ', [423.7..., 0.0..., ... ])
('R = ', [1.0, 0.0, ... ])
('P = ', [429.9..., 0.0, ... ])
...
```

This window may now be frozen, but that's okay. You can simply halt the camera calibration program by typing `ctrl+c` . 

### Motor Calibration

#### Motor Model (copied from Coursera)

We can model the rover as a differential drive, whose linear and angular velocity is related to the rate of rotation of each wheel by:
$$
v = r\frac{u_r + u_l}{2} \\\

\omega = \frac{r}{l}(u_r - u_l)
$$
where $u_l$ and $u_r$ are the angular velocities of the left and right wheels respectively, $r$ is the radius of each wheel and $L$ is the distance between the wheels. So, given a commanded linear and angular velocity, we can compute the left and right wheel angular velocities with the following equations:
$$
u_l = \frac{v-\frac{\omega L}{2}}{r}\\
u_r = \frac{v-\frac{\omega L}{2}}{r}
$$
Finally, making the assumption that there is a linear relationship between motor voltage and wheel angular velocity, we can compute the commanded voltage for the motor to be:
$$
V_l = u_l\frac{\text{gear ratio}} {\text{rpm}_{\text{max}}}\\
V_r = u_r\frac{\text{gear ratio}} {\text{rpm}_{\text{max}}}
$$
While it is easy to measure the wheel radius and the distance between the wheels, it is difficult to determine the true RPM of the motors, which is why we need to perform an extra calibration step.

If you would like more information on differential drive models, please refer to:

[http://planning.cs.uiuc.edu/node659.html](http://planning.cs.uiuc.edu/node659.html)

#### Calibration

For this calibration, we will perform a very basic gain tuning, where we drive the rover forwards at a set speed for 1 second, and then measuring the actual distance traveled by the robot, and using the difference to tune the ratio between gear ratio and max rpm. One simple way to do this is by lining the wheels of a robot up with a ruler and measuring where it ends up, as seen in the image below.

![Motorcalibration.resized](/home/victor/Git/Coursera/RoboticsCapstone_UPenn/RoboticsCapstone_ArTrack/Motorcalibration.resized.jpg)

Hint: To spin the wheels for a set amount of time, you can measure the start time using the following ROS API call:

```python
time = rospy.get_time()
```

Then, you can command the robot to move at a certain velocity and, once `rospy.get_time()-time` exceeds 1, command the robot to stop.

To change the gains of the robot, edit the file located at:

```
/home/pi/catkin_ws/src/dc_motor_driver/params/motor_params.yaml
```

The parameter `motor_gain` is the ratio between gear ratio and max rpm. Increasing this value will increase the distance the rover travels within a second at a given velocity. You may also change the parameters `wheel_radius` and `wheel_sep`, which are r and L as defined above, but you should not have to if you are using the course kit.

#### Calibration Results

I added the following code to `RobotControl.py` in order to command the robot to move at 0.3 m/s for 1 second:

```python
time = rospy.get_time()
while not rospy.is_shutdown():
    robotControl.process_measurements()
    # Calibration
    # command robot forward at 0.3 m/s for 1 sec and measure how far it travels
    if rospy.get_time() - time < 1:
        robotControl.ros_interface.command_velocity(0.3,0)
    else:
        robotControl.ros_interface.command_velocity(0,0)
    r.sleep()
```

Over four trials, the robot moved 0.22 m, 0.21 m, 0.215 m, and 0.21 m. Let's say on average, the robot moved 0.21 m in 1 second. The robot moved relatively straight based on observation so each motor can be calibrated using the same constant. Because the robot was ideally supposed to travel 0.3 m in 1 second, we have to increase the power output of the motor to achieve the desired speed. To calculate the calibration constant, we have:
$$
k_{cal} = \frac{v_{actual}}{v_{command}} = \frac{0.21}{0.3} = 0.7
$$
Thus, we have to divide our commanded speed by a factor of 0.7 to increase the effective commanded speed. This should get us closer to our desired (or close to desired) speed. In the `motor_params.yaml` file the default value of `motor_gain` is 1. So we will change this value to 1/0.7 = 1.4.

### Camera to Body Calibration

We want all of our position and velocity values to be in the coordinate frame of the robot body, which is in the center of the rotational axis of the two wheels. However, the camera has its own coordinate frame so we will have to convert measured distances from the camera from the camera coordinate frame to the robot body coordinate frame. The coordinate axes of the body and camera are shown in the figure below.

![camera_to_body_cal](/home/victor/Git/Coursera/RoboticsCapstone_UPenn/RoboticsCapstone_ArTrack/camera_to_body_cal.jpg)

Given a measurement in the camera frame $\begin{bmatrix} X_{CAM} & Y_{CAM} & Z_{CAM} \end{bmatrix}^T$, we can transform the measurement into the body frame  $\begin{bmatrix} X_{BODY} & Y_{BODY} & Z_{BODY} \end{bmatrix}^T$using the equation given below.
$$
\begin{bmatrix}X_{BODY}\\Y_{BODY}\\Z_{BODY}\end{bmatrix} = ^{CAM}R_{BODY}\begin{bmatrix}X_{CAM}\\Y_{CAM}\\Z_{CAM}\end{bmatrix} +^{CAM}T_{BODY}\\
\begin{bmatrix}X_{BODY}\\Y_{BODY}\\Z_{BODY}\end{bmatrix} = ^{CAM}R_{BODY}\begin{bmatrix}X_{CAM}\\Y_{CAM}\\Z_{CAM}\end{bmatrix} +
\begin{bmatrix}T_X\\T_Y\\T_Z\end{bmatrix}
$$
$^{CAM}R_{BODY}$, pronounced as "R cam to body", is the transformation matrix that transforms the camera measurement into the body frame. In this case, the two coordinate axis are parallel so we can by find the rotation matrix by inspection. Start by ignoring the translational component and assuming it is zero. From the diagram, we can see that the $X_{BODY}$ is aligned with $Z_{CAM}$. Hence, the first row is $\begin{bmatrix} 0 & 0 & 1 \end{bmatrix}$. $Y_{BODY}$ is aligned with $X_{CAM}$ and thus the second row is $\begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$. $Z_{BODY}$ is aligned with $Y_{CAM}$ and thus the third row is $\begin{bmatrix} 0 & 1 & 0 \end{bmatrix}$. Overall, the body frame and camera frame are related as
$$
\begin{bmatrix}X_{BODY}\\Y_{BODY}\\Z_{BODY}\end{bmatrix} = \begin{bmatrix}0&0&1\\1&0&0\\0&1&0\end{bmatrix}
\begin{bmatrix}X_{CAM}\\Y_{CAM}\\Z_{CAM}\end{bmatrix}
$$
To make sure we have a valid rotation matrix, the determinant should equal 1. In this case, we have $\text{det}(R) = 1$ so this rotation matrix is valid. Also, $R^TR$ should equal the identity matrix, and it does.

$^{CAM}T_{BODY}$, pronounced as "T cam to body", is the translation from the body to the camera, in the body frame. We can see based on the figure that $T_X$ will be the largest positive value because the camera is in front of the body origin, $T_Y$ will be slightly negative because the camera is slightly to the right of the body origin, and $T_Z$ will be positive because the camera is higher than the body origin. The measurements are: $T_X$ = 11 cm, $T_Y$ = -0.75 cm, and $T_Z$ = 3.5 cm.

After measuring $T_X$, $T_Y$, and $T_Z$, they should be recorded in the `params.yaml` file located at

```
~/catkin_ws/src/robot_control/params/params.yaml
```

$^{CAM}R_{BODY}$ does not need to be recorded in the `params.yaml` file because it is the same rotation matrix for everyone so it is hard-coded into the program.

### IMU Accelerometer Calibration

#### (Copied from Coursera)

In order to make sense of the values from the accelerometer, you will need to calibrate the min/max values of the accelerometer. This involves moving the IMU around as quickly as possible. To run the calibration, ssh into the Pi, and navigate to this folder:

```
/home/pi/catkin_ws/src/imu_rtimulib/params/
```

Then, run the following program:

```
RTIMULibCal
```

You will then see the following menu:

```
Options are:

  m - calibrate magnetometer with min/max
  e - calibrate magnetometer with ellipsoid (do min/max first)
  a - calibrate accelerometers
  x - exit

Enter option:
```

Press `a` to calibrate the accelerometers. The goal here is to accelerate the IMU in each direction as quickly as possible. Pick up the rover, and press `e`. This will activate the x axis calibration. Move the rover along the x axis of the IMU quickly, and you should see the min/max values change. Keep going until they stop changing, and then press space to move onto the y axis, where you should press `e` again. Repeat this for the z axis, and then press `x` to save when you are done. This will save a calibration file `RTIMULib.ini`. Replace the current `imu_calib.ini` file with your calibration, and the calibration will be loaded the next time you run `robot.launch`.

#### Results

After running `RTIMULibCal`, the following error message appeared:

```
RTIMULibCal - using RTIMULib.ini
Settings file not found. Using defaults and creating settings file
Failed to open SPI bus 0, select 0
Failed to open SPI bus 0, select 1
No IMU detected
Using fusion algorithm RTQF
No IMU found
```

Opening the `RTIMULib.ini` file shows that the **Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002** is not one of the supported IMUs in the **RTIMULib** IMU library. It turns out the `RTIMULib.ini` file was configured to "auto-detect" whatever device was connected to the I2C or SPI bus.  In the error message, it appears as though the program is trying to open the SPI bus. However, the description says that it uses I2C as its method of connection. Make sure that I2C is enabled by running `sudo raspi-config` and following the instructions. Reboot the Pi after making any changes to the raspi-config. Even after enabling I2C, the IMU was not detected. Instead of trying to make this IMU work with the RTIMULib, I will try the tutorials on the Adafruit website:

[https://www.adafruit.com/product/3463](https://www.adafruit.com/product/3463)




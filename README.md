## Week 2: Assembling the Rover

### Flashing your Raspberry Pi SD Card (copied from Coursera)

First download the Pi image. Once you have downloaded the image, use the tutorial matching your operating system to help you flash the Pi image onto your SD card (you will need to uncompress the file). When this is done, insert the SD card into the Pi before assembly.

Please see the video on connecting to the Pi for connection instructions.

Linux installation instructions:

[https://www.raspberrypi.org/documentation/installation/installing-images/linux.md](https://www.raspberrypi.org/documentation/installation/installing-images/linux.md)

The linked instructions suggested to install the program **Etcher** to flash the SD card. This method worked seamlessly.

### Connecting to the Pi

You can use SSH to connect to your Raspberry Pi from a Linux computer, a Mac, or another Raspberry Pi, without installing additional software.

You will need to know your Raspberry Pi's IP address to connect to it. To find this, type `hostname -I` from your Raspberry Pi terminal.

To connect to your Pi from a different computer type `ssh pi@<IP>` where `<IP>` is replaced with your specific IP address.

The IP address for my Raspberry Pi is `192.168.1.11` and the default password is `raspberry`.

### A Basic Program to Move the Rover (copied from Coursera)

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

### Running Your Code (copied from Coursera)

You will need two terminal windows to run your code. To do this, you can just ssh to the Pi in two separate windows.

In the first terminal window, run the following command:

```
$ roslaunch robot_launch robot.launch
```

The `robot.launch` file contains the packages that control the robot. Namely, they are the AprilTag detector, the IMU driver, the camera driver and the motor driver. You won't need most of them except the motor driver for this reading, but it doesn't hurt to run them all.

In the second terminal window, launch your `RobotControl` code with the following:

`$ roslaunch robot_control robot_control.launch`

With the code from the previous section, you should see the rover move forwards. To stop the rover, type `Ctrl-C` to kill the program. Don'Camera calibratiot worry if the speed is too quick or too slow, we will calibrate this next week.

## Week 3: Calibration

### Camera calibration

Print out the provided checkerboard of alternating black and white squares. Measure the length of the squares and note this for later. My squares had length 0.025 meters, but it could also be something close to this value. Also count the number of squares in each dimension of the checkerboard. In the tutorial video, the checkerboard was size 11x9. My checkerboard was size 9x7.

To begin calibration, open up two terminal windows in your Raspberry Pi. In one terminal window, run the launch file that starts the ROS camera node (called `raspicam_node`):

```
$ roslaunch robot_launch robot.launch
```

This also starts other nodes for the april tag detector, motor driver, and IMU even though we will not be using them here (which is okay). In the other terminal window, start the camera calibration program:

```
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/image_raw camera:=/camera
```

The input argument for `--size` will vary depending on your checkerboard. For example, in the tutorial video where the checkerboard is 11x9 I would input the size as `10x8`. This convention for the camera calibration program is to ignore the outer squares when counting the size of the checkerboard. Similarly, for my checkerboard of size 9x7, I would input the size as `8x6`. The input argument for `--square` will also depend on your checkerboard measurement. The input `--square 0.02` means each square has a length of 0.02 meters.

After running the above command, a new display window will open showing the camera output. On the right side of the window will display three buttons: calibrate, save, and commit. These buttons cannot be clicked until the calibration is ready. Now place your checkerboard in the field of view of the camera. If the camera recognizes the checkerboard, the right side of the window should automatically pop up four categories: **X**, **Y**, **Size**, and **Skew**. Moving the checkerboard around in front of the camera should increase the  calibration level for each category. Periodically, the camera will sample the image at specific points to use for calibration. Concretely, here is how to increase each of the calibration levels:

* X: Move the checkerboard left and right at varying depths.
* Y: Move the checkerboard up and down at varying depths.
* Size: Move the checkerboard closer and farther away from the camera.
* Skew: Rotate the checkerboard around in different axes individually and simultaneously (X, Y, and Z).

Once the calibration level is ready for all four categories (they should all be green), the **CALIBRATE** button should be highlighted. Click on the **CALIBRATE** button to start the calibration. After the calibration parameters are calculated, they will be displayed in the terminal as shown below:

```
('D = ', [0.079..., 0.020..., ... ])
('K = ', [423.7..., 0.0..., ... ])
('R = ', [1.0, 0.0, ... ])
('P = ', [429.9..., 0.0, ... ])
...
```

The calibration process could take up to several minutes, so be patient. If you sample too many points (>50) before starting the calibration, then it could take much longer. After calibration is complete, the **COMMIT** and **SAVE** buttons will become available. Now hit the **COMMIT** button to write and save the new calibration parameters into the `~/catkin_ws/src/raspicam_node/calibrations/camera.yaml` file where they will be used by the controller. After your parameters are saved in `camera.yaml`, there will be an output confirming this in the terminal where you ran `robot.launch`. You should also check to verify that the calibration parameters in `camera.yaml` do indeed match the output shown in the terminal. Hitting the **SAVE** button will save the calibration parameters in an external file, but this part is only optional.

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

#### Calibration Procedure

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
Thus, we have to divide our commanded speed by a factor of 0.7 to increase the effective commanded speed. This should get us closer to our desired (or close to desired) speed. In the `motor_params.yaml` file the default value of `motor_gain` is 1. So we will change this value to 1/0.7 = 1.4. To edit the calibration file using the Nano text editor, enter the command:

```
nano $HOME/catkin_ws/src/dc_motor_driver/params/motor_params.yaml
```

| Velocity command (m/s) | Duration (sec) | Distance traveled (m) | Difference factor | Comments               |
| ---------------------- | -------------- | --------------------- | ----------------- | ---------------------- |
| 0.1                    | 1              | 0.04                  | 0.4               | drifted right          |
| 0.1                    | 1              | 0.035                 |                   | drifted right          |
| 0.1                    | 1              | 0.05                  |                   | drifted right slightly |
| 0.1                    | 1              | 0.04                  | 0.4               | straight               |
|                        |                |                       |                   |                        |



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

### Introduction to April Tags

April Tags are planar fiducial markers that consist of a black square with a unique pattern inside. This is the main method of perception we will be using for this course. There are hundreds of unique tags and each one corresponds to a different number. We will be using the **Pose from Tomography** algorithm to get the 3D pose of the tag with respect to the camera. Because we are dealing with a planar robot in this course, the API will only be returning the 2D X and Y translations of the tag as well as its orientation around the Z axis, which we call $\theta$. To get accurate pose information, you will need to measure one of the sides of the black square and update one of the parameter files with this value.

> Note: The sides of the April tag may not all be the same length depending on your printer settings.

#### Printing the April Tags (copied from Coursera)

For resized versions of the first 16 tags, please download this pdf file:

(link to pdf)

Each tag in the pdf includes its ID number, as well as the direction that is 'up' in the world.

Alternatively, this link contains all possible tags:

<http://april.eecs.umich.edu/software/tag36h10.tgz>

However, you will need to resize them yourself. Note that, when resizing, you must not resample, as otherwise the tags will become very blurred.

For more information about the tags, please refer to: https://april.eecs.umich.edu/wiki/AprilTags, where the original paper is also referenced. For a ROS interface, we use the following package: <http://wiki.ros.org/apriltags>.

#### Software (copied from Coursera)

In order to inform the AprilTag detector of the AprilTags it needs to look for as well as the size of the tags, you must edit the file located at:

`/home/pi/catkin_ws/src/robot_launch/launch/tag_detection.launch`

In this file, you will see a list of tag_ids and their corresponding sizes (length of one side of the black square. Only tags listed in this file will be detected, so you must edit this file each time you change the tags that you are using.

> Note: The code requires that the tags defined in the world map be in consecutive order. That is, if you want to use 4 tags, you must have tags 0, 1, 2, 3, and not 1, 2, 3, 4 or 0, 1, 3, 4 etc.

#### Updating Camera Drivers and Software

To check if the camera is recognizing April Tags, you first need to start it by opening a new terminal and running:

```
$ roslaunch robot_launch robot.launch
```

If the Pi is hooked up to an HDMI monitor (rather than SSH), you can see what is being displayed by the camera by opening a new terminal and running `rqt_image_view`. In the drop-down menu, select the option `/camera/tag_detections_image`. If the April Tag is detected by the camera, the border of the April Tag image should get highlighted and a number should pop up indicating what number April Tag it corresponds to.

If the April Tag is not detected, you could try updating the software:

```
$ sudo apt-get update
$ sudo apt-get updgrade
$ sudo rpi-update
```

> Initially, my camera was not detecting the April Tag but it was fixed after updating the software.

#### Results

The resized April Tags measured 7.1 cm on one side and 7.5 cm on another side. I put the April Tag size as 7.2 cm in the `tag_detection.launch` file.

It turns out that the resized April tags that were about 7.1 cm are **too large**, which restricts the range in which the robot can detect the tag. For example, if the tag is too big then at a close enough distance it will cover the entire field of view (FOV) of the camera and thus be undetectable. By making the tag smaller, it will remain detectable at closer distances. Thus, it was necessary to do a custom resize of the April Tags to make them even smaller. To do this in Ubuntu, I installed an image editing software called **Gimp Image Manipulation Program**.

> Note: There is another software called **Gimp Image Editor**, but this is not the correct one.

First load the image into Gimp. The original size of the April Tag is only 10 x 10 pixels. Then go the **Image > Scale Image** and change the **Width** and **Height** of the **Image Size** to the desired dimensions. I chose a dimension of 3 x 3 cm and this worked well. Leave the resolution the same. It should be 300 x 300 pixels/in by default. Change the **Quality** setting to **None** and then hit **Scale**. The April Tag should now be the correct dimension without any indication of distortion or loss in quality compared to the original image. After resizing the April Tags, I copied and pasted them into a word processing software so I could print out multiple tags on the same sheet.

### IMU Accelerometer Calibration (Copied from Coursera)

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

> Another promising idea is to try following the I2C accelerometer tutorial on Derek Malloy's Exploring Raspberry Pi book. It's possible that I did not activate the I2C channels on the Pi correctly so it could not be read.

## Week 4: Designing a Controller for the Rover

### Robot Kinematic Model

The robot kinematic model is defined as shown in the figure below:

![robot_kinematic_model](/home/victor/Git/Coursera/RoboticsCapstone_UPenn/RoboticsCapstone_ArTrack/robot_kinematic_model.jpg)

Here, the goal position represented by the thick dot is defined to be position of the AprilTag where we want the robot to move to. Let's say that we want to align the X-axis of the robot with the X-axis of the AprilTag. The AprilTag axes are defined as $X_G$ and $Y_G$ and the robot axes are defined as $X_R$ and $Y_R$. The $\Delta x$ and $\Delta y$ are the distances between the robot and the tag along the X and Y axes of the tag. The angle $\theta$ defines the angle between the AprilTag X-axis and the robot X-axis. The distance $\rho$ is the straight line distance between the robot and the tag, $\alpha$ is the angle between the robot X-axis and this line, and $\beta$ is the angle between this line and the tag X-axis. We also have the control inputs, $v$ and $\omega$, which are the linear and angular velocities of the robot, respectively. By defining these terms, we can write a mathematical model describing how the control inputs will affect the robot's movements. Consequently, we will be able to minimize $\rho, \alpha,$ and $\beta$, allowing the robot to move to the tag.

From each April Tag measurement, we can diretly observe $\Delta x$, $\Delta y$, and $\theta$. From these values, we can then calculate $\rho$, $\alpha$, and $\beta$ as
$$
\begin{align}
	\rho &= \sqrt{\Delta x^2 + \Delta y^2} \\
	\alpha &= -\theta + atan2(\Delta y, \Delta x) \\
	\beta &= -\theta - \alpha
	\end{align}
$$

Here, $\rho$ is the distance between the robot position and tag position, $\alpha$ is the angular error between the heading direction of the robot and the direct line to the tag, and $\beta$ is the error between the heading direction of the robot and the desired heading direction at the end. In order for the robot to have reached the tag and achieve the desired heading, we must ultimately have all three of these state variables be zero.

Using geometry, we can calculate the rate of change of each of these quantities that we want to minimize in terms of the control inputs:
$$
\begin{equation}
	\begin{bmatrix}
	\dot{\rho}\\
	\dot{\alpha}\\
	\dot{\beta}
	\end{bmatrix}
	=
	\begin{bmatrix}
	-\cos{\alpha} & 0\\
	(\sin{\alpha})/\rho & -1\\
	(\sin{\alpha})/\rho & 0
	\end{bmatrix}
	\begin{bmatrix}
	v\\
	\omega
	\end{bmatrix}
	\end{equation}
$$
Notice that we have control over all three state variables from the two control inputs (controllability is not discussed here).

### Control Law and Stability

Consider the control law:
$$
\begin{equation}
	\begin{bmatrix}
	v \\
	w
	\end{bmatrix}
	=
	\begin{bmatrix}
	k_{\rho} & 0 & 0 \\
	0 & k_{\alpha} & k_{\beta} \\
	\end{bmatrix}
	\begin{bmatrix}
	\rho \\
	\alpha \\
	\beta
	\end{bmatrix}
	\end{equation}
$$
Substituting these values back into the kinematics, we get:
$$
\begin{equation} \label{eq:state_eqn}
	\begin{bmatrix}
	\dot{\rho} \\
	\dot{\alpha} \\
	\dot{\beta}
	\end{bmatrix}
	=
	\begin{bmatrix}
	-k_{\rho}\rho \cos{\alpha} \\
	k_{\rho}\sin{\alpha} - k_{\alpha}\alpha - k_{\beta}\beta \\
	-k_{\rho}\sin{\alpha}
	\end{bmatrix}
	\end{equation}
$$
Assuming $\alpha\approx0$, we can linearize the system of equations above and obtain:
$$
\begin{bmatrix}
	\dot{\rho} \\
	\dot{\alpha} \\
	\dot{\beta}
	\end{bmatrix}
	=
	\begin{bmatrix}
	-k_{\rho} & 0 & 0 \\
	0 & -(k_{\alpha} - k_{\rho}) & -k_{\beta} \\
	0 & -k_{\rho} & 0
	\end{bmatrix}
	\begin{bmatrix}
	\rho \\
	\alpha \\
	\beta 
	\end{bmatrix}
$$
The characteristic polynomial of the system above is:
$$
(\lambda + k_{\rho})(\lambda^2 + (k_{\alpha} - k_{\rho})\lambda - k_{\rho}k_{\beta}) = 0
$$
The system is locally exponential stable if all eigenvalues of the system have negative real parts. The eigenvalues can be found by solving for the roots of the characteristic polynomial. For the system to be stable, the control gains must be set as $k_{\rho} > 0$, $k_{\beta} < 0$, and $k_{\alpha} - k_{\rho} > 0$. When the control gains are tuned in such a way, the coefficients of the characteristic polynomial are all greater than 0 which ensures that all of its roots have a negative real part.

### Simulation Implementation

First you should implement the controller in `DiffDriveController.py` and get it to work in simulation before testing it on the robot. Here are some key implementation details:

* Let the tag coordinate frame be aligned with the global coordinate frame as in the robot kinematic model diagram. We want to define our `goal` and `state` variables in the tag coordinate frame. Set `goal = np.array([0, 0])` in `process_measurements()` in `RobotControl.py`. This means we are defining the location of the tag as the origin. `meas` is an N x 5 list of visible tags or `None`. For testing purposes, we will only use one tag for this week's assignment. The tags are in the form (x, y, theta, id, time) with x, y being the 2D position of the marker relative to the robot and theta being the relative orientation of the marker with the respect to the robot. However, for the `state` we want the position of the robot with respect to the tag (or global) coordinate frame. Hence we need to negate `meas` to get the state: `state = -np.array(meas[0][0:3])`.
* The X-axis of the tag points in the opposite direction of the tag image. Hence, the camera will only detect the tag if the image side is facing the camera in its field of view. In the simulation, the image side of the tag appears like a nub and the positive direction of the tag X-axis points towards the flat side.
* For simulation, the control gains $k_p = 2$, $k_a = 10$, and $k_b$ = 0 work well. According to the video lecture, with a single tag it's best to set $k_b=0$ to avoid paths where the rover loses the tag from its field of view. 

### Testing the Controller

#### Transferring files to the Pi

After verifying that your code works in simulation, it's time to transfer the file `DiffDriveController.py` from your computer to the Raspberry Pi. To do this, you will use SCP (Secure Copy Protocol) which is a command for sending files over SSH. This means you can copy files between computers, say from your Raspberry Pi to your desktop or laptop, or vice-versa. First you will need to know your Raspberry Pi's IP address by running `hostname -I` in a Raspberry Pi terminal. Note that the IP address of the Raspberry may change after power cycling so check to make sure you have the correct address if the SSH connection does not work.

To copy the file `DiffDriveController.py` from your computer to the Pi, go to a terminal on your host computer, `cd` into the directory where `DiffDriveController.py` lives, and run:

```
$ scp DiffDriveController.py pi@192.168.1.11:/home/pi/catkin_ws/src/robot_control/src
```

where the last argument is the directory in the Pi where you want to copy the file to. Also, make sure that the IP address of the Pi is the correct one.

#### Fixing error in t_cam_to_body

There are some errors that need to be fixed in order to make the tag detection work correctly on the actual robot. Add an extra 0 to the end of the`t_cam_to_body` vector so that it has dimension 4x1. As given, it has dimension 3x1. This parameter can be found in the `params.yaml` file locate in `/catkin_ws/src/robot_control/params`. Also, update the first three elements of the `t_cam_to_body` vector based on your own measurements. 

#### Implementation

Here are some key implementation details:

* The controller runs at a default rate of 60 Hz and it checks the camera output at each iteration. However, the camera sometimes (usually) does not pick up an April Tag measurement at every sampling instance. Hence, when there is no April Tag measurement I set both the velocity and angular velocity commands to zero. This causes the control commands to make sudden jumps between some positive values and zero, resulting in choppy movements by the robot. This will be fixed next week after implementing a Kalman filter.
* To make debugging easier, I reduce the controller sample rate to 1 Hz and print out `meas`, `v`, and `omega` to make sure that the control commands are directionally correct in trying to reduce the error.

## Week 5: State Estimation

### Unicycle Model

We will be using a unicycle model to describe the dynamics of the robot. The state vector is given by $q_k := \left[ \begin{matrix} x & y & \theta \end{matrix} \right]^T$ and the control vector is given by $u_k := \left[ \begin{matrix} v + n_v & \omega + n_{\omega} \end{matrix} \right]^T$, where $n_v$ and $n_\omega$ represent inaccuracies in the actual input. The state prediction model is given by
$$
\begin{align}
	q_k &= f(q_{k-1}, u_{k-1}, n_{k-1})\\
	&= q_{k-1} + \Delta t
	\begin{bmatrix}
	(v+n_v)\cos{\theta} \\
	(v+n_v)\sin{\theta} \\
	\omega + n_{\omega}
	\end{bmatrix} \\
	&= q_{k-1} + \Delta t
	\begin{bmatrix}
	v\cos{\theta} \\
	v\sin{\theta} \\
	\omega
	\end{bmatrix}
	+ \Delta t
	\begin{bmatrix}
	n_v\cos{\theta} \\
	n_v\sin{\theta} \\
	n_{\omega}
	\end{bmatrix}
	\end{align}
$$
where $\Delta t$ is the sampling period. The measurement model is given by
$$
z_k = h(q_k,v_k) = q_k + v_k
$$
where $v_k$ is the measurement noise and $R_k$ is the covariance of the measurement noise (to be used later). Note that all of the states in this system are directly observable.

The first step of the EKF is to predict the state at the current time step $k$ by
$$
\hat{q}_{k|k-1} = f(\hat{q}_{k-1|k-1},u_{k-1},0)
$$
The covariance is predicted by
$$
P_{k|k-1} = F_{k-1}P_{k-1|k-1}F^T_{k-1}+N_{k-1}Q_{k-1}N^T_{k-1}
$$
where $Q_{k-1}$ is the covariance of the process noise, $F_{k-1}$ is the Jacobian of the prediction model with respect to the states calculated as
$$
\begin{align}
	F_{k-1} &= \left.\frac{\partial{f}}{\partial{q}}\right|_{\hat{q}_{k-1|k-1},u_{k-1}} \\
	&=
	\begin{bmatrix}
	1 & 0 & -v\sin{\theta}\cdot\Delta{t} \\
	0 & 1 & v\cos{\theta}\cdot\Delta{t} \\
	0 & 0 & 1
	\end{bmatrix}
	\end{align}
$$
and $N_{k-1}$ is the Jacobian of the prediction model with respect to the noise calculated as
$$
Math
				
					
				
				
						
				
			\begin{align}
	N_{k-1} &= \left.\frac{\partial{f}}{\partial{n}}\right|_{\hat{q}_{k-1|k-1},u_{k-1}} \\
	&= \Delta{t}
	\begin{bmatrix}
	\cos{\theta} & 0 \\
	\sin{\theta} & 0 \\
	0 & 1
	\end{bmatrix}
	\end{align}
$$
The next step is to update the state estimate after getting measurements. First calculate the Kalman gain by
$$
K_k = P_{k|k-1}H_k^T(H_kP_{k|k-1}H_k^T + R_k)^{-1}
$$
where $H_k$ is the Jacobian of the measurement model with respect to the states calculated as
$$
H_{k} = \left.\frac{\partial{h}}{\partial{q}}\right|_{\hat{q}_{k|k-1},u_{k}}
	= I
$$
which is used to update the state estimate by
$$
\hat{q}_{k|k} = \hat{q}_{k|k-1} + K_k(z_k - h(\hat{q}_{k|k-1},0))
$$
as well as to update the covariance estimate by
$$
P_{k|k} = (I - K_kH_k)P_{k|k-1}
$$

### Coordinate Transformations

The purpose of the extended Kalman filter is to estimate the robot's pose in the world frame ($W$), denoted as $(x,y,\theta)$. Let the position and orientation of the tag in the world frame be known apriori as $(x_w,y_w,\theta_w)$ and the AprilTag in the robot frame ($R$) be measured as $(x_r,y_r,\theta_r)$. An arbitrary point ${}^T\vec{p}_{B/T}$ in the AprilTag frame ($T$) can be expressed in the world frame by
$$
\begin{align}
	{}^W\vec{p}_{B/W} &= {}^{W}H_{T}{}^T\vec{p}_{B/T} \\
	&=
	\begin{bmatrix}
	\cos{\theta_w} & -\sin{\theta_w} & x_w \\
	\sin{\theta_w} & \cos{\theta_w} & y_w \\
	0 & 0 & 1
	\end{bmatrix}
	\begin{bmatrix}
	x_b \\
	y_b \\
	1
	\end{bmatrix}
	\end{align}
$$
where ${}^{W}H_{T}$ is the homogeneous transformation matrix that transforms coordinates expressed in the AprilTag frame to coordinates expressed in the world frame. This matrix can be broken down further into intermediate transformations as
$$
\begin{align}
	{}^{W}H_{T} &= {}^{W}H_{R}{}^{R}H_{T} \\
	&=
	\begin{bmatrix}
	\cos{\theta} & -\sin{\theta} & x \\
	\sin{\theta} & \cos{\theta} & y \\
	0 & 0 & 1
	\end{bmatrix}
	\begin{bmatrix}
	\cos{\theta_r} & -\sin{\theta_r} & x_r \\
	\sin{\theta_r} & \cos{\theta_r} & y_w \\
	0 & 0 & 1
	\end{bmatrix}
	\end{align}
$$
Since ${}^{W}H_{T}$ and ${}^{R}H_{T}$ are known, ${}^{W}H_{R}$ can be calculated as
$$
\begin{equation}
	{}^{W}H_{R} = {}^{W}H_{T}{}^{R}H_{T}^{-1} 
	\end{equation}
$$
where $x$, $y$, and $\theta$ can be extracted as $x = {}^{W}H_{R}(0,2)$, $y = {}^{W}H_{R}(1,2)$, and $\theta = \text{atan2}({}^{W}H_{R}(1,0),{}^{W}H_{R}(0,0))$ (0 indexed).

## Deep dive into ROS nodes

First we'll look into what the `robot.launch` file does, which is run by typing:

```
$ roslaunch robot_launch robot.launch
```

A new package named `robot_launch` created in order to initialize all of the robot components at once: the motors, IMU, and camera. The tree of the `robot_launch` directory looks like:

```
.
|-- CMakeLists.txt
|-- include
|   `-- robot_launch
|-- launch
|   |-- robot.launch
|   |-- robot.launch~
|   |-- tag_detection.launch
|   `-- tag_detection.launch~
|-- package.xml
`-- src
```

The **include > robot_launch** subdirectories are empty. The `robot.launch` file launches other launch files:

```
<launch>
  <include file="$(find raspicam)/launch/raspicam.launch"/>
  <include file="$(find robot_launch)/launch/tag_detection.launch"/>
  <include file="$(find imu_rtimulib)/launch/imu_rtimulib.launch"/>
  <include file="$(find dc_motor_driver)/launch/dc_motor_driver.launch"/>
</launch>
```

This includes the Raspberry Pi camera, the April tag detection node, the IMU, and the DC motor driver. The `tag_detection.launch` file launches the `apriltag_detector_node` from the `april_tags_ros` package.

```
<launch>
  <node pkg="apriltags_ros" type="apriltag_detector_node" name="apriltag_detector" output="screen" ns="camera">
    <!-- Enter your AprilTag information below. Add all tag IDs that will be used, with their sizes -->
    <rosparam param="tag_descriptions">[
      {id: 0, size: 0.032},
      {id: 1, size: 0.032},
      {id: 2, size: 0.032},
      {id: 3, size: 0.032},
      {id: 4, size: 0.081},
      {id: 5, size: 0.081},
      {id: 6, size: 0.054},
      {id: 7, size: 0.054},
      {id: 8, size: 0.054},
      {id: 9, size: 0.054},
      {id: 10, size: 0.054},
      {id: 11, size: 0.054},
      {id: 12, size: 0.054},
      {id: 13, size: 0.054},
      {id: 14, size: 0.054},
      {id: 15, size: 0.054},
      ]
    </rosparam>
    <remap from="image_rect" to="image_raw"/>
  </node>
</launch>
```

Optional attributes are `output="screen"` which means stdout/stderr from the node will be sent to the screen. Also there is `ns="camera"`, which means the namespace will be "camera" for any topics that this node publishes. For instance, a topic name could be called `/camera/image_raw`.

The `<rosparam>` tag is put inside of a `<node>` tag, which means the parameter is treated like a private name. In roslaunch files, `<param>` tags are for setting a single parameter and `<rosparam>` tags are for setting groups or more complex structures of parameters. The `<rosparam>` tag enables users to define a batch of related parameters simultaneously. These can be read from a [YAML](http://www.yaml.org/) string which can either be put inline in the launchfile or can be loaded from a file (the `rosparam dump` command generates YAML output). Unlike the `<param>` tag, the YAML specification allows nested structures of parameters and parameters with list values. More can be found here: http://wiki.ros.org/ROS/Patterns/Parameterization#Static_Parameters.



This `<rosparam>` defines a parameter called `tag_descriptions`, which includes a list of dictionaries describing each tag that the camera should recognize. Each tag description includes the **id** and **size** fields.



The `<remap>` tag allows you to pass in name remapping arguments to the ROS node that you are launching in a more structured manner than settig the `args` attribute of a `<node>` directly. Here for example, the `apriltag_detector_node` is subscribes to a node called "image_rect", but another node publishes a topic called "image_raw" which is the same type as "image_rect". To pipe the "image_raw" topic into the `apriltag_detector_node` which wants the "image_rect" topic, just remap the arguments are shown above.

In the `package.xml` file for the `robot_launch` package, it shows dependencies on catkin, roscpp, and rospy:

```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<run_depend>roscpp</run_depend>
 <run_depend>rospy</run_depend>
```

### Understanding raspicam_node/raspicam.launch

The tree for the `raspicam_node` looks like:

```
.
|-- CMakeLists.txt
|-- README.md
|-- calibrations
|   |-- camera.yaml
|   `-- camera.yaml~
|-- include
|   |-- RaspiCLI.h
|   `-- RaspiCamControl.h
|-- launch
|   |-- raspicam.launch
|   `-- raspicam.launch~
|-- package.xml
`-- src
    |-- RaspiCLI.cpp
    |-- RaspiCamControl.cpp
    |-- raspicam_node.cpp
    `-- raspicam_raw_node.cpp
```

The camera calibration file `camera.yaml`  contains information about the image_width, image_height, camera_name, camera_matrix, distortion_model, distortion_coefficients, rectification_matrix, and projection_matrix. This file is populated automatically after running the calibration script.

The `raspicam.launch` file has the following:

```
<launch>

  <node pkg="raspicam" type="raspicam_raw_node" name="raspicam_node" respawn="true" ns="camera" output="screen">
    <param name="quality" type="double" value="10.0"/>
    <param name="framerate" type="double" value="50.0"/>
    <param name="width" type="double" value="320.0" />
    <param name="height" type="double" value="240.0" />
  </node>

  <!--node pkg="image_proc" type="image_proc" name="image_proc" respawn="true" ns="/camera"/-->
</launch>
```

This launches a node named `raspicam_node`using the `raspicam_raw_node.cpp` executable from the `raspicam` package. Any published topics will have the "/camera" namespace and all stderr/stdout commands will be displayed on the screen. If the node dies, it will automatically respawn. Here, four parameters are set including the quality, framerate, width, and height. (QUESTION) Why is the package name "raspicam" in the launch file when the directory name for the package is "raspicam_node"? If you go to the `package.xml` file, between the `<name>` tags is the name "raspicam". Hence, the directory name for the package is not necessarily the same as the package name.



## Starting a new catkin workspace in ROS Kinetic

First install ROS Kinetic according to the instructions. Make sure that in the `.bashrc` file that you have included:

```
source /opt/ros/kinetic/setup.bash
```

and have commented out:

```
#source /opt/ros/indigo/setup.bash
```

### Initialize a new catkin workspace

```
$ mkdir -p upenn_kinetic_ws/src
$ cd upenn_kinetic_ws
$ catkin_make
```



### Install ROS packages one by one

Start by creating the ROS packages that have no dependencies, for example the `raspicam_node` package. Because this OS is Raspbian Jessie, you will need to build the packages from source by following the instructions here: https://github.com/fpasteau/raspicam_node

The `raspicam_node` package depends on other ROS packages like `image_transport`, `image_transport_plugins`, and `camera_info_manager`, which you will have to build from source by following instructions here: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi. Before cloning the `raspicam_node` package to your catkin workspace, you need to build the ROS package dependencies listed above from source using the Kinetic ROS distribution. Note that the instructions on the GitHub repo are for ROS Groovy. However, the UPenn catkin workspace is using ROS Indigo, and I want to get it working on ROS Kinetic.



## Shortcuts

### ROS

`roslaunch robot_launch robot.launch`

`roslaunch robot_control robot_control.launch`

### Files

`cd $HOME/catkin_ws/src/robot_control/src/ `

`nano $HOME/catkin_ws/src/robot_control/src/RobotControl.py`

`nano $HOME/catkin_ws/src/robot_control/src/DiffDriveController.py`

`nano $HOME/catkin_ws/src/robot_control/src/KalmanFilter.py`

`nano $HOME/catkin_ws/src/robot_control/params/params.yaml` - t_cam_to_body, dt, occupancy_map, max_omega, max_vel, pos_init, pos_goal, world_map (April tag IDs and poses), x_spacing, y_spacing




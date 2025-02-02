### robot_chase

#### Overview

Depends on the existence of the package [`barista_robot_description`](https://github.com/ivogeorg/barista_robot_description.git). It uses a `tf2_ros::TransformerListener` to allow one robot to chase another as it moves.

#### Submission notes

1. Launching:
   1. Start Gazebo and spawn the 2 robots:
      ```
      cd ~/ros2_ws/src
      git clone https://github.com/ivogeorg/barista_robot_description.git
      git clone https://github.com/ivogeorg/robot_chase.git
      cd ~/ros2_ws
      colcon build
      source install/setup.bash
      ros2 launch barista_robot_description barista_two_robots.launch.py
      ```
   2. Start the robot chase executable:
      ``` 
      cd ~/ros2_ws
      source install/setup.bash
      ros2 run robot_chase robot_chase
      ```
   3. Start the teleoperator for the robot that will be chased:
      ```
      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel
      ```
2. Note that once in a while, maybe if `robot_chase` is started too quickly after the robot spawning, `lookupTransform` fails reporting that there is no link between the source and target frames.
3. Expected result:
   | Robot chase |
   | --- |
   | ![Robot chase 1](assets/robot_chase_1.png) |
   | ![Robot chase 2](assets/robot_chase_2.png) |
   | ![Robot chase 3](assets/robot_chase_3.png) |
   | ![Robot chase 4](assets/robot_chase_4.png) |
   | ![Robot chase 5](assets/robot_chase_5.png) |
   | ![Robot chase 6](assets/robot_chase_6.png) |
   | ![Robot chase 7](assets/robot_chase_7.png) |
   | ![Robot chase 8](assets/robot_chase_8.png) |
   | ![Robot chase 9](assets/robot_chase_9.png) |
   
#### Implementation notes

##### 1. Robot names and roles

1. Robot `morty` is the `RobotRole::MOVER`. It moves (that is, is teleoperated to move) independently.
2. Robot `rick` is the `RobotRole::FOLLOWER`. It calculates its distance (`error_distance`) from and direction (`error_yaw`) toward `morty` and follows it.

##### 2. Base links and frames

1. The "base link" in the Barista robot model is `base_footprint`.
2. The two frames that the transform between will be the centerpiece of `RobotChase` are therefore `/rick/base_footprint` and `/morty/base_footprint`.
3. The main idea is that the `FOLLOWER` wants to match its frame with that of the `MOVER`, and the transform is used to derive linear and angular velocities for a `Twist` message to the `/rick/cmd_vel` topic. It's not exactly clear which frame should be which:
   1. In the tutorial referenced in the next section, the two frames are called `fromFrameRel` and `toFrameRel`.
   2. However, `fromFrameRel` is initialized with something called `target_frame_`.
   3. The tutorial, however, is solving the very same problem, with `turtle1` and `turtle2`, so just follow along and later _rationalize_ the choice of frame naming.
      1. `turtle1` is the `MOVER` and is teleoperated. `target_frame_` and also `fromFrameRel` are of `turtle1`.
      2. `turtle2` is the `FOLLOWER` and receives velocity messages. `toFrameRel` is of `turtle2`. So, it looks like the transform is taken from the target (the `MOVER`) to the `FOLLOWER`. _TODO: It is a bit unintuitive that the velocities to be sent to the FOLLOWER will be extracted from such a tranform. The tutorial actually does exactly that._


##### 3. ROS objects

The relevant [tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html).  

1. A `tf2_ros::TransformListener` (with a `tf2_ros::TransformListener`) will be used to get the tranform between the two frames.
2. The (tranform listener) buffer returns a `geometry_msgs::msg::TransformStamped` containing the tranform.
3. The central call looks like this:
   ```
   try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
    ```
4. Notice the order of the the frames in `lookupTransform`.  
5. A velocity publisher of `geometry_msgs::msg::Twist` messages to topic `/rick/cmd_vel`.
6. A timer is used to call the tf listener at a given period (or frequency).

##### 4. Robot chase dynamics

1. `FOLLOWER` follows `MOVER`.
2. If the `MOVER` turns, the `FOLLOWER` turns, too.
3. If the `MOVER` goes straight, `FOLLOWER` first picks up speed, then decelerates.
4. If the `MOVER` stops, `FOLLOWER` approaches decelerating and stops, too. 
5. It might be advantageous to borrow the parameter array of tuples from [`robot_patrol`](https://github.com/ivogeorg/robot_patrol/blob/7c9d6edda7bfc00803aba78509cf9fedba80a6c4/src/patrol_with_service.cpp#L116).  

##### 5. Robot name service

The `barista_robot_description` may provide a `robot_name` service. This could be a separate `std_srvs/srv/Empty` service `barista_robot_names` node launched in `barista_two_robots.launch.py`, which returns 
```
robot_1_name string
robot_1_role string
robot_2_name string
robot_2_role string
robot_radius float64
```

In this case there would be a client of the service in the `RobotChase` class. Currently, these values are hardcoded as arguments to the constructor.


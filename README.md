# Gazebo ROS Battery plugin

## Functionality
This is a Gazebo classic plugin that simulates an open-circuit battery model. This is a fairly extensible and reusable battery plugin for any kind of Gazebo compatible robots.
The base of this plugin was primarily developed for the challenge problem 1 (CP1) of the BRASS Project at CMU and substantially extended by Czech Technical University in Prague.

This power model simulates the power consumption of a robot. The amount of power consumed by each component of the robot depends on its usage. The state of charge of the battery is updated after each iteration considering `dt`, the power loads of all components in the robot that consume energy and voltage of the battery (which behaves according to the open circuit voltage model).

## Support
This plugin is tested for ROS Melodic/Gazebo 9 and ROS Noetic/Gazebo 11.

## Build
This is a standard catkin package. Just add it to your workspace, install dependencies via rosdep, and build it.

Compiling will result in a shared library `$WORKSPACE/devel/lib/libgazebo_ros_battery_discharge.so` that can be inserted in a Gazebo simulation (and other libraries for consumers).

## Usage

See `example/battery_demo.world` file for an example on how to integrate the plugin in your model.

You can run the example world using:

```bash
rosrun gazebo_ros gazebo --verbose $WORKSPACE/src/gazebo_ros_battery/examples/battery_demo.world
```

## libgazebo_ros_battery_discharge.so

This is the main plugin that represents the discharging process of a battery.

### ROS topics

The plugin reports its state on the following topics:

```
battery_state  # discharge plugin
charge_level_wh  # discharge plugin
```

### Configuration

The plugin accepts the following configuration XML tags:

- `<robotNamespace>` (string, default ''): If nonempty, this value prefixes the ROS topics created by the plugin.
- `<link_name>` (string): Name of the link the battery is attached to.
- `<battery_name>` (string): Name of the battery (has to correspond to the `<battery>` tag in the SDF).
- `<allow_charging>` (bool, defalt `true`): Whether the battery can charge ("consume" negative power).
- `<update_rate>` (float, default 1.0 Hz): Rate at which the state topics will be published.
- `<constant_coeff>` (float): Constant parameter of the open circuit model. This is the highest voltage the battery can have.
- `<linear_coeff>` (float): The linear parameter of the open circuit model. This value should be negative. `<constant_coeff> + 1 * <linear_coeff>` should give the minimum voltage of the battery.
- `<initial_charge>` (float): Initial charge of the battery in Ah.
- `<capacity>` (float): Capacity of the battery in Ah.
- `<design_capacity>` (float): Capacity of the battery when it was new in Ah.
- `<resistance>` (float): Internal resistance of the battery in Ohms. It affects how large is the influence of power load to voltage drop.
- `<smooth_current_tau>` (float): Parameter of a low-pass filter that filters the power load measurements. Should be between 0 and 1.
- `<frame_id>` (string, default `<battery_name>`): The frame ID used in ROS messages.
- `<location>` (string, default ''): Value of the `location` field in the ROS messages.
- `<serial_number>` (string, default ''): Value of the `serial_number` field in the ROS messages.
- `<technology>` (string, default ''): Value of the `power_supply_technology` field in the ROS messages. Can be on of `''`, `NIMH`, `LION`, `LIPO`, `LIFE`, `NICD`, `LIMN`.
- 

## Provided consumer plugins

The following consumer plugins are provided by this plugin (and more plugins can be implemented by extending class `BatteryConsumerBase`).

### ROS Topics

Each consumer plugin publishes its power load to topic `~/power_load` (type `std_msgs/Float64`).

### Configuration

Each consumer plugin has these XML configuration options (and some other specific to the plugin):

- `<robotNamespace>` (string, default ''): If nonempty, this value prefixes the ROS topics created by the plugin.
- `<link_name>` (string): Name of the link the battery is attached to.
- `<battery_name>` (string): Name of the battery (has to correspond to the `<battery>` tag in the SDF).
- `<consumer_name>` (string, default `<battery_name>`): Name of the consumer (will be used as prefix for the `~/power_load` topic and others).

### libgazebo_ros_battery_consumer.so

Constant load consumer that just applies the given load all the time.

The applied load can be changed by messages published to topic `~/power_load_cmd` (type `std_msgs/Float64`). The load can also be negative, which means charging.

The following configuration options are available:

- `<power_load>` (float): The initial power load in Watts.

### libgazebo_ros_cmd_vel_battery_consumer.so

Consumes battery based on the values published to a `cmd_vel` topic.

The following configuration options are available:

- `<power_load_rate_x>` (float, default 0.0): Linear coefficient of `Twist.linear.x` of the messages that converts it to power load in Watts.
- `<power_load_rate_y>` (float, default 0.0): Linear coefficient of `Twist.linear.y` of the messages that converts it to power load in Watts.
- `<power_load_rate_z>` (float, default 0.0): Linear coefficient of `Twist.linear.z` of the messages that converts it to power load in Watts.
- `<power_load_rate_roll>` (float, default 0.0): Linear coefficient of `Twist.angular.x` of the messages that converts it to power load in Watts.
- `<power_load_rate_pitch>` (float, default 0.0): Linear coefficient of `Twist.angular.y` of the messages that converts it to power load in Watts.
- `<power_load_rate_yaw>` (float, default 0.0): Linear coefficient of `Twist.angular.z` of the messages that converts it to power load in Watts.
- `<consumer_idle_power>` (float): Minimum power consumed by the consumer (in Watts).
- `<command_duration>` (float, default 0.1 s): How long is each `cmd_vel` command effective. The load will only be counted for this long.
- `<gz_pose_topic>` (string, default ''): If nonempty, the plugin watches this Gazebo topic for velocity commands in the form of `msgs::Pose` messages.
- `<gz_twist_topic>` (string, default ''): If nonempty, the plugin watches this Gazebo topic for velocity commands in the form of `msgs::Twist` messages.
- `<ros_cmd_vel_topic>` (string, default ''): If nonempty, the plugin watches this ROS topic for velocity commands in the form of `geometry_msgs/Twist` messages.

### libgazebo_ros_motor_battery_consumer.so

Consumes battery based on the effort of a joint reported on a `joint_states` topic (type `sensor_msgs/JointStates`).

The following configuration options are available:

- `<efficiency>` (float, default 1.0): Ratio between required mechanical power and the consumed electrical charge. It should be a number between 0 and 1.
- `<consumer_idle_power>` (float): Minimum power consumed by the consumer (in Watts).
- `<joint>` (string, multiple tags allowed): Names of the joints this plugin handles. It reads the `<joint_states_topic>` messages and if it finds any of the joints in the message, it calculates the power load of all the handled joints. All handled joints must come together in a single message.
- `<joint_states_topic>` (string, default `joint_states`): Name of the ROS topic to subscribe.

### libgazebo_ros_mechanical_energy_battery_consumer.so

Consumes battery according to the changes in the robot's kinetic and potential energy. This plugin is useful if you don't know what force is applied by the locomotion system of the robot, but you know it has to correspond to the change in the robot's energy.

The following configuration options are available:

- `<efficiency>` (float, default 1.0): Ratio between required mechanical power and the consumed electrical charge. It should be a number between 0 and 1.
- `<friction>` (float, default 0.0): Ratio of kinetic energy that will be counted as additional power load. This allows drawing power even in case the robot doesn't accelerate, but has non-zero velocity.
- `<consumer_idle_power>` (float): Minimum power consumed by the consumer (in Watts).
- `<ignore_first_duration>` (float, default 1.0 s): A few first seconds after simulation start can be ignored by the plugin, so that it doesn't consume energy e.g. when the robot is falling and settling on the terrain.

## Notes about conversions
For converting capacity and charge rate (in `Ah`) to power (`mWh`) which is consumed by the robot, the formula is `(Ah)*(V) = (Wh)`. For example, if you have a `3 Ah` battery rated at `5 V`, the power is `3 Ah * 5 V = 15 Wh` or `15000 mWh`.
For converting `Watts` to `Watt-hours`, we do `Watt * hour`, e.g., `6 Watts / 3600 (Wh)` per seconds. 

## Acknowledgements
CMU authors used/inspired by existing theory of open circuit battery model. This battery discharge/charge plugin uses the Gazebo `Battery` class which is shipped by the default simulator.

Further references: [r1](http://security.livewatch.com/forum-ref/ohms-law-calculator), [r2](http://batteriesbyfisher.com/determining-charge-time), [r3](https://electronics.stackexchange.com/questions/24160/how-to-calculate-the-time-of-charging-and-discharging-of-battery).

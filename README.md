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

## Exposed ROS topics

The Gazebo plugins report their state on the following topics:

```
battery_state  # discharge plugin
charge_level_wh  # discharge plugin
~/power_load  # all consumer plugins
```

The constant battery consumer can be controlled via topic `~/power_load_cmd` which changes its power consumption.

## Notes about conversions
For converting capacity and charge rate (in `Ah`) to power (`mWh`) which is consumed by the robot, the formula is `(Ah)*(V) = (Wh)`. For example, if you have a `3 Ah` battery rated at `5 V`, the power is `3 Ah * 5 V = 15 Wh` or `15000 mWh`.
For converting `Watts` to `Watt-hours`, we do `Watt * hour`, e.g., `6 Watts / 3600 (Wh)` per seconds. 

## Acknowledgements
CMU authors used/inspired by existing theory of open circuit battery model. This battery discharge/charge plugin uses the Gazebo `Battery` class which is shipped by the default simulator.

Further references: [r1](http://security.livewatch.com/forum-ref/ohms-law-calculator), [r2](http://batteriesbyfisher.com/determining-charge-time), [r3](https://electronics.stackexchange.com/questions/24160/how-to-calculate-the-time-of-charging-and-discharging-of-battery).

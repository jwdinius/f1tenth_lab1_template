# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer:

Both commands source a workspace.  "Sourcing", in this context, means setting up the necessary runtime context for running ROS executables and for building new and custom user-defined packages.

The first command sources the _underlay_ workspace: the system-wide installation of ROS packages that were installed by `apt`.  The second command sources the _overlay_ workspace: the compiled packages that are written by developers and that are built on top of the underlay workspace.

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer:

There is no `queue_size` argument defined in the [C++ API docs](https://docs.ros2.org/galactic/api/rclcpp/classrclcpp_1_1Node.html#ad1dfc9d04d67ab93353e04a7df72bc9a) nor in the [Python API docs](https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.create_publisher).  I assume, though, that `queue_size` is intended to mean the _depth_ of the queue that the middleware needs to maintain between a publisher and a subscriber.  Increasing the depth means that older messages will be maintained, which may impact message delivery (i.e., may increase latency because older messages will be delivered first).  However, if you really want to make sure that a message gets through without forcing reliable transport, then increasing the depth can improve the chances of the message getting across, particularly if there is bursty traffic (lots of messages published at once).

In the context of the first lab, as well as for similar scenarios where data will be published at very high rates, we don't really care about all messages getting across.  We care about latency.  We know that messages are going to be published at very high rates and that the content of those messages will change slowly relative to the rate of their publication.  I believe that the best approach is to set the depth to 1, as I have done in my solution.

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer:

It depends.  If you are calling launch using the launchfile installed in the `install/{package}` folder then, yes, you will need to run `colcon build` again.  If you are referencing a path (local _or_ global) to your launchfile when you run then, no, you won't need to rebuild.  I don't like to run files in my source tree.  Rather, I prefer to only interact with files installed by `colcon build`.

## Notes

The docs for `ament_cmake_python` are pretty terrible.  I believe that this is the right package type to use when wanting to mix python and C++ code in the same package, but there were a couple of red flags:

* There is no `build-type` option for `ament_cmake_python` when using `ros2 pkg create`.
* `colcon build` fails with a cryptic error when trying to build an `ament_cmake_python` package from the command-line:

```bash
colcon build --packages-select lab1_pkg
[0.248s] WARNING:colcon.colcon_core.verb:No task extension to 'build' a 'ros.ament_cmake_python' package
                     
Summary: 0 packages finished [0.13s]
```

I just built the package as `ament_cmake`.  I will keep python and C++ packages separated for the remainder of the labs unless there are better examples to draw from.
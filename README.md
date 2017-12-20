# hybrid_exploration
* Autonomous SLAM
* testing on the kobuki with kinect

## run(after activating kobuki and SLAM) <br>
* hybrid exploration(switch from sensor-based to frontier-based)
<pre>
roslaunch hybrid_exploration hybrid_exploration.launch
</pre>
* only sensor-based exploration<br>
<pre>
roslaunch hybrid_exploration sensor_based_exploration.launch
</pre>
* only frontier-based exploration<br>
<pre>
roslaunch hybrid_exploration frontier_based_exploration.launch
</pre>


# res_g627_one_ws
**Hardware** **platform**: Unitree GO2, Nvidia Orin NX 16GB, Livox MID360 

**Software**: Fast-lio, octomap_server, Pointcloud_to_Laserscan, RRT_exploration, move_base, ROS1, ROS2, ros1-bridge, Unitree_sdk2, Unitree_ros2



**Environment**:

- ros-noetic
- ros-foxy
- ubuntu 20.04



##### Simulation:

- run simulation based on Gezabo first:

  ```bash
  roslaunch robot_description simulation.launch
  ```

- run slam, here you can simply run gammping:

  ```bash
  roslaunch robot_navigation gmapping.launch simulation:=true
  ```

- run navigation stack "move_base" package:

  ```bash
   roslaunch robot_navigation move_base.launch simulation:=true planner:=teb
  ```

- run rviz.launch

  ```bash
  roslaunch rrt_exploration rrt_rviz.launch
  ```

- run rrt_exploration

  ```bash
  roslaunch rrt_exploration simple.launch
  ```

##### Run on own robot

- Run directly, here SLAM algorithm is fast-lio:

  ```bash
  roslaunch go2_rrt_exploration whole.launch
  ```

- Attention:

  - The 2d grid map is projected from fast-lio and topic name is **/projected_map**, you should make sure add the topic name remap in the **move_base.launch** and **simple.launch**:

    ```xml
    <remap from="map" to="projected_map"/>
    ```

  - Besides, the python interpreter for the scripts is **python3**


<launch>
  <arg name="left_config" default="$(find mvs_ros2_driver)/config/left_camera_trigger.yaml"/>
  <arg name="right_config" default="$(find mvs_ros2_driver)/config/right_camera_trigger.yaml"/>

  <node pkg="mvs_ros2_driver"
        type="mvs_camera_node"
        name="left_camera"
        args="$(arg left_config)"
        respawn="true"
        output="screen"/>

  <node pkg="mvs_ros2_driver"
        type="mvs_camera_node"
        name="right_camera"
        args="$(arg right_config)"
        respawn="true"
        output="screen"/>

  <node pkg="rviz"
        type="rviz"
        name="rviz"
        args="-d $(find mvs_ros2_driver)/rviz_cfg/mvs_camera.rviz"
        output="screen"/>
</launch>

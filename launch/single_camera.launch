<launch>
  <arg name="config" default="$(find mvs_ros2_driver)/config/left_camera_trigger.yaml"/>
  <arg name="use_rviz" default="false"/>

  <node pkg="mvs_ros2_driver"
        type="mvs_camera_node"
        name="mvs_camera_trigger"
        args="$(arg config)"
        respawn="true"
        output="screen"/>

  <node pkg="rviz"
        type="rviz"
        name="rviz"
        args="-d $(find mvs_ros2_driver)/rviz_cfg/mvs_camera.rviz"
        output="screen"
        if="$(arg use_rviz)"/>
</launch>

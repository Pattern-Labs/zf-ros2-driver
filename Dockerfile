FROM ros:humble

RUN apt update && apt install -y \
    ros-humble-foxglove-bridge \
    libboost-all-dev

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ros2_ws
COPY ./frgen21_ros2_driver /ros2_ws/src/frgen21_ros2_driver
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]


#  If not sudo apt install ros-[VERSION]-rviz2 
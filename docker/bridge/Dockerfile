# crane-bridge: crane's own ROS side for `crane sim up --stack bridge` (config/stacks/bridge.yaml): ROS 2 Jazzy, the
# Unity ROS-TCP endpoint, the message packages the player speaks (std, geometry, sensor, nav, tf2, vision), and
# what the T1 node needs (rclpy). No robot software. The endpoint is the team's fork at the commit astro_dock vendors
# (src/ros_tcp_endpoint: upstream 0.7.0 plus the reconnect and socket-teardown fixes), so both stacks behave alike.
#
#   docker build -t crane-bridge:jazzy docker/bridge
#   docker run --rm --network host crane-bridge:jazzy bash -c 'ros2 pkg list | grep ros_tcp_endpoint'
FROM ros:jazzy-ros-base
ARG ENDPOINT_REPO=https://github.com/MHSeals/ROS-TCP-Endpoint.git
ARG ENDPOINT_REF=3c3d405
RUN apt-get update \
 && apt-get install -y --no-install-recommends git python3-colcon-common-extensions ros-jazzy-vision-msgs \
      ros-jazzy-nav-msgs ros-jazzy-sensor-msgs ros-jazzy-tf2-msgs \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir -p /opt/crane_bridge/src \
 && git clone "$ENDPOINT_REPO" /opt/crane_bridge/src/ros_tcp_endpoint \
 && git -C /opt/crane_bridge/src/ros_tcp_endpoint checkout --quiet "$ENDPOINT_REF" \
 && rm -rf /opt/crane_bridge/src/ros_tcp_endpoint/.git \
 && . /opt/ros/jazzy/setup.sh \
 && cd /opt/crane_bridge && colcon build --merge-install \
 && rm -rf build log \
 && printf '. /opt/ros/jazzy/setup.bash\n. /opt/crane_bridge/install/setup.bash\n' > /opt/crane_bridge/env.sh \
 && chmod -R a+rX /opt/crane_bridge
ENV BASH_ENV=/opt/crane_bridge/env.sh
CMD ["bash", "-c", "ros2 pkg list | grep ros_tcp_endpoint"]

FROM ros:rolling

RUN apt-get update && apt-get install -qy prometheus prometheus-pushgateway
ADD prometheus.yml /etc/prometheus

ADD . /workspace
WORKDIR /workspace
RUN apt-get update && . /opt/ros/rolling/setup.sh && rosdep update --rosdistro=rolling && rosdep install -iy --from-path /workspace/src
RUN . /opt/ros/rolling/setup.sh && colcon build --event-handlers console_direct+

CMD /usr/bin/prometheus --config.file=/etc/prometheus/prometheus.yml & /usr/bin/prometheus-pushgateway &  . /workspace/install/local_setup.sh && /workspace/install/test_prometheus/lib/test_prometheus/test_prometheus
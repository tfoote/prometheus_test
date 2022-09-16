FROM ros:rolling

RUN apt-get update && apt-get install -qy libcurlpp-dev libzstd-dev


RUN apt-get update && apt-get install -qy prometheus prometheus-pushgateway
ADD prometheus.yml /etc/prometheus

ADD . /workspace
WORKDIR /workspace
RUN . /opt/ros/rolling/setup.sh && colcon build --event-handlers console_direct+

CMD /usr/bin/prometheus --config.file=/etc/prometheus/prometheus.yml & /usr/bin/prometheus-pushgateway &  . /workspace/install/local_setup.sh && /workspace/install/test_prometheus/lib/test_prometheus/test_prometheus
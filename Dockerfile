FROM ros:rolling

RUN apt-get update && apt-get install -qy libcurlpp-dev libzstd-dev


RUN apt-get update && apt-get install -qy prometheus prometheus-pushgateway
ADD prometheus.yml /etc/prometheus

RUN git clone https://github.com/jupp0r/prometheus-cpp.git
WORKDIR /prometheus-cpp
RUN git submodule init && git submodule update

RUN mkdir -p /prometheus-cpp/build
WORKDIR /prometheus-cpp/build

RUN cmake .. -DBUILD_SHARED_LIBS=ON -DENABLE_PUSH=ON -DENABLE_COMPRESSION=OFF
RUN cmake --build . --parallel 4
RUN ctest -V
RUN cmake --install .

ADD . /workspace
WORKDIR /workspace
RUN . /opt/ros/rolling/setup.sh && colcon build --event-handlers console_direct+

CMD /usr/bin/prometheus --config.file=/etc/prometheus/prometheus.yml & /usr/bin/prometheus-pushgateway &  LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH /workspace/install/test_prometheus/lib/test_prometheus/test_prometheus
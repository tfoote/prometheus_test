docker build -t prometheustest .

rocker --net=host prometheustest


Visit: http://localhost:9090 to view the output
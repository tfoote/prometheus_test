docker build -t prometheustest .

rocker --net=host prometheustest


Visit: http://localhost:9090 to view the output

## Potential queries

* `rate(callback_2_count[1m])`

* `callback_1_count`

* `rate(timer_callbacks_total[1m])`

* `callback_2_gauge`
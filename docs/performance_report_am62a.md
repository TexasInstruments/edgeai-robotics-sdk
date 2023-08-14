# Performance Report

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## C920 UCB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A53 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| MCU1_0 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 23.49| 42.5763| 15.2609| 36.2415| 54.20| 3227| 2464| 5691| 80.78| 0.80| 9.13| 0| 0| 0| 6.75
ti_vision_cnn (objdet)| 28.79| 34.7379| 3.4872| 14.0175| 42.80| 1292| 754| 2046| 37.13| 0.79| 10.74| 0| 0| 0| 7.23

**Note**: "A53 Load (%)" are for dual A53 cores in a scale of 100%. For example, 100% A53 loading means that two A53 cores are fully loaded.

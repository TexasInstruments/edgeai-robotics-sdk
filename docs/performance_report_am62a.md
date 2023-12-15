# Performance Report

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## C920 UCB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A53 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| MCU1_0 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 23.88| 41.8727| 16.0046| 34.1663| 52.71| 2618| 2337| 4955| 73.11| 0.95| 8.75| 0| 0| 0| 6.35
ti_vision_cnn (objdet)| 26.73| 37.4123| 4.0000| 18.1254| 43.10| 2354| 726| 3080| 47.40| 0.75| 10.18| 0| 0| 0| 6.86

**Note**: "A53 Load (%)" are for dual A53 cores in a scale of 100%. For example, 100% A53 loading means that two A53 cores are fully loaded.

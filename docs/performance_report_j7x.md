# Performance Report

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## TDA4VM

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

### ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 14.95| 66.8828| NA| NA| 30.84| 1024| 956| 1980| 0.0| 0.0| 0.0| 3.0| 1.0| 0| 0| 0| 0| 4.58| 16.43| 0
ti_sde (w/ pcl)| 14.85| 67.3430| NA| NA| 56.41| 1577| 1504| 3081| 0.0| 16.0| 0.0| 4.0| 3.0| 6.56| 0| 0| 0| 4.48| 20.54| 0
ti_vision_cnn (semseg)| 15.08| 66.2917| 3.1853| 8.0032| 22.82| 817| 693| 1510| 11.0| 0.0| 0.0| 2.0| 1.0| 3.24| 0| 0| 0| 2.15| 0| 0
ti_vision_cnn (objdet)| 27.64| 36.1776| 1.1669| 5.5020| 71.89| 1491| 951| 2442| 18.0| 1.0| 0.0| 4.0| 1.0| 6.73| 0| 0| 0| 4.51| 0| 0
ti_estop| 15.11| 66.1794| 3.1551| 8.1098| 39.44| 1264| 1021| 2285| 11.0| 1.0| 1.0| 3.0| 1.0| 3.29| 0| 0| 0| 4.54| 16.16| 0

### C920 USB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.47| 32.8228| 3.1026| 8.0244| 50.26| 1259| 775| 2034| 22.0| 1.0| 0.0| 4.0| 1.0| 6.48| 0| 0| 0| 4.30| 0| 0
ti_vision_cnn (objdet)| 30.33| 32.9738| 1.0012| 5.0012| 46.91| 1095| 664| 1759| 16.0| 0.0| 0.0| 5.0| 1.0| 6.35| 0| 0| 0| 4.22| 0| 0

<!-- ======================================================================= -->
## AM68A

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

### ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.00| 66.6834| NA| NA| 32.57| 1088| 982| 2070| 1.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.25| 16.2| 0
ti_sde (w/ pcl)| 15.10| 66.2105| NA| NA| 57.28| 1708| 1542| 3250| 0.0| 12.0| 3.0| 2.0| 6.38| 0| 0| 0| 4.36| 20.98| 0
ti_vision_cnn (semseg)| 15.13| 66.0838| 3.3284| 9.0060| 21.53| 1540| 1392| 2932| 14.0| 1.0| 2.0| 1.0| 3.13| 0| 0| 0| 2.10| 0| 0
ti_vision_cnn (objdet)| 15.05| 66.4431| 1.0000| 5.0000| 17.55| 842| 658| 1500| 8.0| 0.0| 2.0| 1.0| 3.6| 0| 0| 0| 2.5| 0| 0
ti_estop| 15.12| 66.1226| 3.4029| 9.1439| 40.50| 1981| 1716| 3697| 14.0| 0.0| 2.0| 1.0| 3.13| 0| 0| 0| 4.31| 16.8| 0

### C920 USB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 28.79| 34.7380| 3.1113| 10.0025| 51.34| 2419| 1975| 4394| 25.0| 1.0| 2.0| 1.0| 5.61| 0| 0| 0| 3.79| 0| 0
ti_vision_cnn (objdet)| 30.42| 32.8687| 1.0000| 5.0012| 49.50| 1254| 686| 1940| 15.0| 1.0| 2.0| 1.0| 6.5| 0| 0| 0| 4.10| 0| 0

<!-- ======================================================================= -->
## AM69A

**Note**: "A72 Load (%)" are for 8x A72 cores in a scale of 100%. For example, 100% A72 loading means that 8x A72 cores are fully loaded.

### ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| C71_3 Load (%)| C71_4 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MCU3_0 Load (%)| MCU3_1 Load (%)| MCU4_0 Load (%)| MCU4_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1342| NA| NA| 7.12| 912| 936| 1848| 0.0| 0.0| 0.0| 0.0| 1.0| 1.0| 1.0| 1.0| 1.0| 1.0| 0| 0| 0| 0| 4.34| 16.30| 0
ti_sde (w/ pcl)| 15.14| 66.0401| NA| NA| 11.48| 1402| 1428| 2830| 0.0| 13.0| 0.0| 1.0| 3.0| 2.0| 1.0| 0.0| 1.0| 1.0| 6.38| 0| 0| 0| 4.36| 20.95| 0
ti_vision_cnn (semseg)| 15.14| 66.0430| 4.0000| 9.0029| 4.82| 1378| 1342| 2720| 12.0| 0.0| 0.0| 1.0| 2.0| 1.0| 1.0| 1.0| 1.0| 1.0| 3.15| 0| 0| 0| 2.10| 0| 0
ti_vision_cnn (objdet)| 15.09| 66.2763| 1.0000| 5.0187| 3.70| 691| 600| 1291| 8.0| 0.0| 1.0| 0.0| 2.0| 1.0| 1.0| 1.0| 1.0| 1.0| 3.11| 0| 0| 0| 2.8| 0| 0
ti_estop| 15.17| 65.9078| 4.0000| 9.1604| 9.11| 1736| 1638| 3374| 12.0| 0.0| 0.0| 1.0| 2.0| 1.0| 1.0| 1.0| 1.0| 1.0| 3.15| 0| 0| 0| 4.33| 16.13| 0

### C920 USB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| C71_3 Load (%)| C71_4 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MCU3_0 Load (%)| MCU3_1 Load (%)| MCU4_0 Load (%)| MCU4_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.53| 32.7597| 4.0000| 9.0141| 14.52| 2476| 2147| 4623| 26.0| 0.0| 0.0| 0.0| 2.0| 1.0| 1.0| 1.0| 1.0| 1.0| 6.17| 0| 0| 0| 4.34| 0| 0
ti_vision_cnn (objdet)| 30.55| 32.7333| 1.0000| 5.0035| 11.24| 1092| 645| 1737| 16.0| 0.0| 0.0| 1.0| 2.0| 1.0| 1.0| 1.0| 1.0| 1.0| 6.23| 0| 0| 0| 4.40| 0| 0

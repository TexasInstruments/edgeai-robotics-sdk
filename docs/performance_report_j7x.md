# Performance Report

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## TDA4VM

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

### ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 14.04| 71.2117| NA| NA| 35.82| 943| 884| 1827| 1.0| 1.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 3.16| 12.7| 0
ti_sde (w/ pcl)| 15.03| 66.5456| NA| NA| 56.53| 1565| 1503| 3068| 0.0| 17.0| 1.0| 4.0| 3.0| 6.65| 0| 0| 0| 4.54| 21.17| 0
ti_vision_cnn (semseg)| 15.14| 66.0599| 5.0000| 10.0000| 24.37| 1026| 890| 1916| 14.0| 1.0| 0.0| 2.0| 1.0| 3.21| 0| 0| 0| 2.14| 0| 0
ti_vision_cnn (objdet)| 15.11| 66.1995| 4.0000| 5.0146| 22.55| 801| 664| 1465| 8.0| 0.0| 0.0| 2.0| 1.0| 3.17| 0| 0| 0| 2.11| 0| 0
ti_estop| 15.05| 66.4585| 5.0195| 10.0049| 42.85| 1465| 1213| 2678| 14.0| 0.0| 1.0| 3.0| 1.0| 3.28| 0| 0| 0| 4.52| 16.37| 0

### C920 USB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.17| 33.1460| 5.0668| 10.0111| 56.4| 1569| 1111| 2680| 27.0| 0.0| 0.0| 4.0| 1.0| 6.4| 0| 0| 0| 4.4| 0| 0
ti_vision_cnn (objdet)| 30.30| 33.0048| 4.0538| 5.0490| 51.3| 1189| 709| 1898| 15.0| 1.0| 0.0| 4.0| 1.0| 6.43| 0| 0| 0| 4.21| 0| 0

<!-- ======================================================================= -->
## AM68A

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

### ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 14.99| 66.6916| NA| NA| 32.16| 1085| 978| 2063| 0.0| 0.0| 1.0| 1.0| 0| 0| 0| 0| 4.27| 16.39| 0
ti_sde (w/ pcl)| 15.00| 66.6619| NA| NA| 49.11| 1652| 1516| 3168| 1.0| 12.0| 3.0| 2.0| 6.21| 0| 0| 0| 4.26| 20.64| 0
ti_vision_cnn (semseg)| 15.13| 66.1060| 5.0211| 13.0000| 23.92| 1766| 1398| 3164| 19.0| 0.0| 1.0| 0.0| 3.14| 0| 0| 0| 2.9| 0| 0
ti_vision_cnn (objdet)| 15.08| 66.2920| 4.0170| 7.4393| 23.13| 1037| 785| 1822| 12.0| 1.0| 1.0| 1.0| 3.10| 0| 0| 0| 2.7| 0| 0
ti_estop| 15.09| 66.2777| 5.0098| 13.0000| 40.67| 2233| 1728| 3961| 19.0| 1.0| 2.0| 1.0| 3.12| 0| 0| 0| 4.29| 16.13| 0

### C920 USB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 29.54| 33.8578| 5.2307| 13.0228| 56.72| 3060| 2138| 5198| 37.0| 1.0| 2.0| 1.0| 6.11| 0| 0| 0| 4.2| 0| 0
ti_vision_cnn (objdet)| 30.18| 33.1354| 4.0749| 7.0012| 54.40| 1595| 920| 2515| 19.0| 0.0| 3.0| 0.0| 5.97| 0| 0| 0| 3.99| 0| 0

<!-- ======================================================================= -->
## AM69A

**Note**: "A72 Load (%)" are for 8x A72 cores in a scale of 100%. For example, 100% A72 loading means that 8x A72 cores are fully loaded.

### ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| C71_3 Load (%)| C71_4 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MCU3_0 Load (%)| MCU3_1 Load (%)| MCU4_0 Load (%)| MCU4_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1392| NA| NA| 7.22| 1397| 930| 2327| 0.0| 0.0| 1.0| 1.0| 2.0| 1.0| 1.0| 1.0| 1.0| 1.0| 0| 0| 0| 0| 4.37| 16.17| 0
ti_sde (w/ pcl)| 15.14| 66.0615| NA| NA| 12.0| 1915| 1450| 3365| 0.0| 13.0| 1.0| 1.0| 3.0| 3.0| 1.0| 0.0| 1.0| 1.0| 6.36| 0| 0| 0| 4.43| 20.96| 0
ti_vision_cnn (semseg)| 15.15| 66.0234| 5.0000| 9.0029| 5.13| 1913| 1367| 3280| 13.0| 0.0| 0.0| 1.0| 2.0| 1.0| 1.0| 0.0| 1.0| 1.0| 3.13| 0| 0| 0| 2.13| 0| 0
ti_vision_cnn (objdet)| 15.15| 66.0049| 4.0000| 7.0000| 4.50| 1268| 728| 1996| 10.0| 0.0| 0.0| 0.0| 1.0| 1.0| 1.0| 1.0| 1.0| 1.0| 3.10| 0| 0| 0| 2.10| 0| 0
ti_estop| 15.14| 66.0312| 5.0024| 9.0072| 9.19| 2298| 1687| 3985| 13.0| 0.0| 0.0| 0.0| 3.0| 1.0| 1.0| 1.0| 1.0| 1.0| 3.11| 0| 0| 0| 4.35| 16.45| 0

### C920 USB Camera, 30 FPS

**Source**: live C920 USB camera, 1280x720 in MJPG mode, at 30 FPS. "gscam2" ROS node and a demo ROS node are running in the ROS 2 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71_1 Load (%)| C71_2 Load (%)| C71_3 Load (%)| C71_4 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MCU3_0 Load (%)| MCU3_1 Load (%)| MCU4_0 Load (%)| MCU4_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.50| 32.7913| 4.9929| 9.0012| 13.55| 2990| 2137| 5127| 26.0| 0.0| 0.0| 0.0| 2.0| 1.0| 1.0| 1.0| 1.0| 1.0| 6.17| 0| 0| 0| 4.36| 0| 0
ti_vision_cnn (objdet)| 30.47| 32.8151| 3.9918| 5.0024| 14.2| 1717| 878| 2595| 16.0| 0.0| 1.0| 0.0| 3.0| 1.0| 1.0| 1.0| 1.0| 1.0| 6.21| 0| 0| 0| 4.38| 0| 0

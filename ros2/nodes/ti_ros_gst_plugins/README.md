ROS-GST Plugins
===============

TI ROS-GST plugins provides an optimized bridge between ROS 2 and GStreamer framework.

## Build Plugins

Please follow the [LINK](../../../docker/setting_docker_ros2.md) to setup Docker for ROS 2 on the target.

Once the Docker setup is done on target, use the following to build the
GStreamer plugins.

```bash
root@docker:~/j7ros_home/ros_ws$ colcon build --base-paths /opt/robotics_sdk/ros2 --packages-select ti_ros_gst_plugins --cmake-force-configure
```

The libraries for the plugins can be found under
`$COLCON_PREFIX_PATH/ti_ros_gst_plugins/`

## Inspect Plugins

Inspect all available ROS-GST plugins

```bash
root@docker: gst-inspect-1.0 $COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/libros_gst_plugins.so
```

Inspect specific ROS-GST plugins

```bash
root@docker: gst-inspect-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/ tirosimgsrc
```

## Plugins

### **tirosimgsrc**

This plugin can subscribe to ROS Image or CompressedImage(jpeg) topic
and put the received msg to GStreamer or TIOVX buffer.

There are two ways to define caps for the plugin

- Explicitly define caps as property, shown in example below (This is necessary for CompressedImage(jpeg) msg)

- Let first ROS msg set the caps automatically (Works only for Image msg)

You can use example rosbag file provided to publish msg, or you can you any other source

```bash
root@docker: ros2 bag play $WORK_DIR/data/ros_bag/zed1_2020-11-09-18-01-08/ --loop --topic /camera/left/image_raw
```

In another terminal run the pipeline to receive the msg being published to the topic.

```bash
# tirosimgsrc->display

root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/ tirosimgsrc ros-topic=/camera/left/image_raw caps="video/x-raw,width=1280,height=720,format=UYVY" ! kmssink driver-name=tidss sync=false
```

```bash
# tirosimgsrc->AI->display

root@docker: export MODEL_DIR=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416/

root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/ \
tirosimgsrc ros-topic=/camera/left/image_raw ! tiovxdlcolorconvert ! video/x-raw,format=NV12 ! tiovxmultiscaler name=split_01 \
\
split_01. ! queue ! tiovxdlpreproc model=$MODEL_DIR out-pool-size=4 ! application/x-tensor-tiovx ! tidlinferer target=1  model=$MODEL_DIR ! post_0.tensor \
\
split_01. ! queue ! video/x-raw, width=1280, height=720 ! post_0.sink \
\
tidlpostproc name=post_0 model=$MODEL_DIR viz-threshold=0.5 display-model=true ! queue ! mosaic_0. \
\
tiovxmosaic name=mosaic_0 target=1 \
src::pool-size=4 sink_0::startx="<320>" sink_0::starty="<150>" sink_0::widths="<1280>" sink_0::heights="<720>" \
! video/x-raw,format=NV12, width=1920, height=1080 ! queue ! tiperfoverlay ! kmssink driver-name=tidss sync=false
```

### **tirossink**

This plugin can publish specific ROS msg based on sink caps from gst pipeline.
You can also specify optional property `ros-msg-type`. If not define, the plugin will
pick appropriate message type based on the caps automatically.

Currently supported msgs:
  - sensor_msgs/Image
  - sensor_msgs/CompressedImage
  - std_msgs/String
  - custom_msgs/Detection2D

#### sensor_msgs/Image
```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/ videotestsrc ! video/x-raw,width=1280,height=720,format=NV12,framerate=30/1 ! tirossink ros-topic=image_raw
```

#### sensor_msgs/CompressedImage
```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/ videotestsrc ! video/x-raw,width=1280,height=720,format=NV12,framerate=30/1 ! jpegenc ! image/jpeg ! tirossink ros-topic=image_jpeg
```

```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/ videotestsrc ! video/x-raw,width=1280,height=720,format=NV12,framerate=30/1 ! pngenc ! image/png ! tirossink ros-topic=image_png
```

#### std_msgs/String
```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins/ multifilesrc location=x.txt ! text/x-raw ! tirossink ros-topic=string
```

#### custom_msgs/Detection2D
```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins v4l2src device=/dev/video-usb-cam0 io-mode=2 ! image/jpeg, width=1280, height=720 ! jpegdec ! \
tiovxdlcolorconvert ! video/x-raw, format=NV12 ! \
tiovxmultiscaler ! video/x-raw, width=416, height=416 ! \
tiovxdlpreproc model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416  out-pool-size=4 ! application/x-tensor-tiovx ! \
tidlinferer target=1  model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 ! post_0.tensor \
tidlpostproc name=post_0 model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 viz-threshold=0.6 \
\
post_0.text ! queue ! text/x-raw ! tirossink ros-topic=Det2D ros-msg-type=detection2d
```

#### custom_msgs/Detection2D & sensor_msgs/Image
```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins v4l2src device=/dev/video-usb-cam0 io-mode=2 ! image/jpeg, width=1280, height=720 ! jpegdec ! \
tiovxdlcolorconvert ! video/x-raw, format=NV12 ! \
tiovxmultiscaler name=split_01 \
split_01. ! queue ! video/x-raw, width=416, height=416 ! \
tiovxdlpreproc model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416  out-pool-size=4 ! application/x-tensor-tiovx ! \
tidlinferer target=1  model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 ! post_0.tensor \
\
split_01. ! queue ! video/x-raw, width=1280, height=720 ! \
post_0.sink \
\
tidlpostproc name=post_0 model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 viz-threshold=0.6 \
\
post_0.src ! queue ! video/x-raw ! tirossink ros-topic=Image ros-msg-type=image \
post_0.text ! queue ! text/x-raw ! tirossink ros-topic=Det2D ros-msg-type=detection2d
```

#### custom_msgs/Detection2D & sensor_msgs/CompressedImage
```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins v4l2src device=/dev/video-usb-cam0 io-mode=2 ! image/jpeg, width=1280, height=720 ! jpegdec ! \
tiovxdlcolorconvert ! video/x-raw, format=NV12 ! \
tiovxmultiscaler name=split_01 \
split_01. ! queue ! video/x-raw, width=416, height=416 ! \
tiovxdlpreproc model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416  out-pool-size=4 ! application/x-tensor-tiovx ! \
tidlinferer target=1  model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 ! post_0.tensor \
\
split_01. ! queue ! video/x-raw, width=1280, height=720 ! \
post_0.sink \
\
tidlpostproc name=post_0 model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 viz-threshold=0.6 \
\
post_0.src ! queue ! video/x-raw ! jpegenc ! tirossink ros-topic=CompressedImage ros-msg-type=compressed-image-jpeg \
post_0.text ! queue ! text/x-raw ! tirossink ros-topic=Det2D ros-msg-type=detection2d
```

## Example AI pipeline

**[SK]** On the target,
```bash
root@docker: gst-launch-1.0 --gst-plugin-path=$COLCON_PREFIX_PATH/ti_ros_gst_plugins/lib/ti_ros_gst_plugins v4l2src device=/dev/video-usb-cam0 io-mode=2 ! image/jpeg, width=1280, height=720 ! jpegdec ! \
tiovxdlcolorconvert ! video/x-raw, format=NV12 ! tee name=tee_split \
\
tee_split. ! queue ! tirossink ros-topic=camera/image_rect_nv12 \
\
tee_split. ! queue ! tiovxmultiscaler ! video/x-raw, width=416, height=416 ! \
tiovxdlpreproc model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416  out-pool-size=4 ! application/x-tensor-tiovx ! \
tidlinferer target=1  model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 ! post_0.tensor \
tidlpostproc name=post_0 model=/opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-coco-416x416 viz-threshold=0.6 \
\
post_0.text ! queue ! text/x-raw ! tirossink ros-topic=vision_cnn/tensor ros-msg-type=detection2d
```

**[Visualization on Ubuntu PC]**
```bash
ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py width:=1280 height:=720 approx_time_sync:=True
```
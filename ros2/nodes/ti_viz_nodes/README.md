Visualization Nodes
===================

## Disparity Map Visualization

This application subscribes to a raw disparity map topic, which is published by ti_sde, creates a color-coded disparity map and a confidence map from the raw disparity map and publish them.

We can run this application by launching `viz_disparity.launch` file, i.e.,
```
roslaunch ti_viz_nodes viz_disparity.launch
```
viz_disparity.launch file specifies the followings:
* Input topic name to read a raw disparity map.
* Output topic names to publish a color-coded disparity map and confidence map, respectively, which can be displayed directly by RViz.

<!-- =================================================================================================== -->
## Semantic Segmentation Map Visualization

This application subscribes to two topics, one is an input image topic, on which class dependent colors will be overlaid, and the other is a semantic segmentation CNN output tensor topic, which has class information for all pixels. It creates a color-coded semantic segmentation map from them, which is published as an output.

We can run this application by launching `viz_semseg.launch` file, i.e.,
```
roslaunch ti_viz_nodes viz_semseg.launch
```

`viz_semseg.launch` file specifies the followings:
* Input image topic name to read input images.
* CNN tensor topic name to read output tensors from a CNN network.
* Output topic name to publish a color-coded semantic segmentation map, which can be displayed directly by RViz.

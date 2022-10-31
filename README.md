# Video Processing Pipeline with ROS

The project contains ROS packages for streaming video, processing output frames with CV labeling algorithm and sending result metadata to external device.

## Prerequisites

- ``ROS Melodic/Noetic`` 
- ``OpenCV 3`` 

## Launch

Launch four nodes (described below) to simulate video processing pipeline

`roslaunch video_processing video_pipeline.launch --screen`


## Task

Every frame needs to be processed by a pipeline which is a black box algorithm(create a stub for it) that accepts a frame
takes ~30ms (0.03s) to execute and returns a collection of objects metadata in the frame
`{“id”: id, “box”: [center_x, center_y, width, height]}`.

Every frame needs to be sent to the UI for rendering(No need to implement UI, just create a stub).

Pipeline results should be rendered and sent to an external device via pub/sub protocol of your choice once available.

Note: The app should be realtime eg 25fps


## Framework choice

The task was to buid software with pub/sub protocol, running in real time and with possibilities of execution testing.

The ROS software consists of independently running processes called *nodes*.
Nodes communicate with each other using ROS communication pub/sub protocol called *topics*.
ROS provides this communication layer and it also allow to build software with nodes running on different machines connected to one network (typically Ethernet).
This framework was designed to control robotics devices and proved its reliability and flexibility.


## Software architecture

The software consists of four ROS packages:

 - `video_streamer` : Streams video to ROS `video_stream` topic. The package contains `file_streamer` node which streams video from file.
 - `cv_tools` : CV algorithms that subscribe to `video_stream` image topics and publish metadata with processing results. The package contains `image_labeling` stub node which simulate image processing with execution time 30ms and publish results to `image_labels` topic.
 - `ui_tools` : UI interface that subscribe to `video_stream` image topics.  The package contains `ui_tools` stub node. 
 - `external_device` : External device controllers that subscribe to metadata provided by CV tools.  The package contains `external_device` stub node sunscribed to `image_labels` topic.

The software architecture diagram presents nodes in circles and topics in rectangles.
The image is automatically generated with *rqt_graph* tool.

![](video_processing/docs/rosgraph.png)


## Testing

ROS provide wide rage of out the shelf tools to test "real-time" distributed systems.

### Software log stream

Each node in the presented software provide logging for executed operations. 
Logs are send on message publishing and message receiving.
ROS middleware adds timestamps to each message which allows to debug delays in the spftware operations.

Here is the log printed by all node when the whole software is running

```
[ INFO] [1667206637.776340193]: Frame 10 published.
[ INFO] [1667206637.776808830]: CV: Image received.
[ INFO] [1667206637.777042802]: UI: Image received
[ INFO] [1667206637.807023314]: CV: Image processed. Labels published.
[ INFO] [1667206637.807232743]: DEVICE: Image labeling received.
[ INFO] [1667206637.819124740]: Frame 11 published.
[ INFO] [1667206637.819448768]: CV: Image received.
[ INFO] [1667206637.819526531]: UI: Image received
[ INFO] [1667206637.849604974]: CV: Image processed. Labels published.
[ INFO] [1667206637.849781327]: DEVICE: Image labeling received.
[ INFO] [1667206637.855778156]: Frame 12 published.
[ INFO] [1667206637.856164230]: CV: Image received.
[ INFO] [1667206637.856207791]: UI: Image received
[ INFO] [1667206637.886336246]: CV: Image processed. Labels published.
[ INFO] [1667206637.886512581]: DEVICE: Image labeling received.
```

From the log output delay of message transport has been calculated:

```
Video stream to CV:  1667206637.855778156 - 1667206637.856164230 = -0.00038 s
Video stream to UI:  1667206637.855778156 - 1667206637.856207791 = -0.00042 s
Meatadata to DEVICE: 1667206637.855778156 - 1667206637.886512581 = -0.03073 s (includes 30 ms processing time)
```
Typical delay is message transportation is **0.00042 s**.
Video stream is published frame by frame with frequency **25 Hz** (frame interval T = 1/24 = 0.04 s).
So the transportation delay is about **1.05%** of system frame publishing interval.

### Video topic log from file_streamer

Also ROS allows to test frequency of individual topics using `rostopic hz` tool./video_stream.
The output provide by testing video stream topic `rostopic hz /video_stream`
```
subscribed to [/video_stream]
average rate: 25.031
        min: 0.035s max: 0.045s std dev: 0.00234s window: 23
average rate: 24.984
        min: 0.032s max: 0.047s std dev: 0.00274s window: 48
average rate: 25.055
        min: 0.032s max: 0.047s std dev: 0.00245s window: 73
average rate: 25.006
        min: 0.032s max: 0.049s std dev: 0.00282s window: 98
average rate: 25.004
        min: 0.032s max: 0.049s std dev: 0.00279s window: 123
average rate: 25.021
        min: 0.030s max: 0.050s std dev: 0.00323s window: 148
average rate: 25.015
        min: 0.030s max: 0.050s std dev: 0.00322s window: 173
```

Max divergence in publishing interval

*T = (1/25 Hz)-(1/25.055 Hz) = 0.000087 s = 0.087 ms*

### Metadata topic log from image_labeling

The output provide by testing video stream topic `rostopic hz /image_labels`
```
subscribed to [/image_labels]
average rate: 24.991
        min: 0.034s max: 0.045s std dev: 0.00235s window: 25
average rate: 25.005
        min: 0.034s max: 0.045s std dev: 0.00230s window: 50
average rate: 25.009
        min: 0.034s max: 0.047s std dev: 0.00236s window: 75
average rate: 24.997
        min: 0.034s max: 0.051s std dev: 0.00257s window: 100
average rate: 25.005
        min: 0.034s max: 0.051s std dev: 0.00243s window: 125
average rate: 25.002
        min: 0.034s max: 0.051s std dev: 0.00246s window: 150
average rate: 25.004
        min: 0.034s max: 0.051s std dev: 0.00234s window: 175
average rate: 24.999
```

Max divergence in publishing interval

*T = (1/25 Hz)-(1/25.009 Hz) = 0.000014 s = 0.014 ms*

## Conclusion

ROS does not provide any guarantees about publishing latency and processing time when using topics.
The Linux operating system is not real-time as well.
But the communication latency are small enough, in range on 1% of publishing cycle of 25Hz, as was calculated above.
This makes the ROS software middleware suitable for near real-time systems as robotics hardware or distributed video streaming as in this project.
And those characteristics was proven by many applications running on ROS.


# my_ros_tools
A collection of ROS related tools for debugging

### rosbag-record-param-generator

A console-gui tool for generating "rosbag record" command for certain topics.

Usage: 

1. ```rosrun my_ros_tools rosbag-record-param-generator.py```

2. It will list all the available topics. Use "Up" "Down" "Page Up" "Page Down" to navigate through the topics. Use "Space" to toggle selection.

3. Press "F2" or mouse click "Save" button to save it to a file. An input dialog will show up. You can modify the file name. "Enter" to confirm.

4. Press "q/ESC" or mouse click "Quit" to quit. A file (default name: record.sh) will be generated in the current folder, looks like "rosbag record TOPIC1 TOPIC2 ...". You can modify it according to your requirements

Advanced features:
* Use --help to see the additional arguments

Screenshot:


### enhanced-rostopic-statistic

A tool for measuring delay and frequency for a certain topic. Besides ```ros.Time.now()```, an other topic can be assigned as the reference.

All topics should have "std_msgs/Header".

Usage: 
* ```rosrun my_ros_tools enhanced-rostopic-statistic.py --help``` to see the usage

Screenshot:



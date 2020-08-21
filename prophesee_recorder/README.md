### Run prophesee_recorder
First download the dependencies for this package
    
    vcs-import < prophesee_ros_wrapper/prophesee_recorder/dependencies.yaml
    
Then build the node
    
    catkin build prophesee_recorder

Run the prophesee_recorder with the following command
```bash
roslaunch prophesee_recorder stereo_record_prophesees.launch \
          output_root:=/home/dani/Documents/temp \
          events_topic_right:=/prophesee/camera/right/cd_events_buffer \
          imu_topic_right:=/prophesee/camera/right/imu \
          trigger_topic_right:=/prophesee/camera/right/extTrigger \
          events_topic_left:=/prophesee/camera/left/cd_events_buffer \
          imu_topic_left:=/prophesee/camera/left/imu \
          trigger_topic_left:=/prophesee/camera/left/extTrigger \    
          v:=1                                      
```

This creates a folder with name `/home/dani/Documents/temp` where all the data is stored. 
After recording, kill the node to write the triggers.

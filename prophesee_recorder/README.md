### Run prophesee_recorder

'''bash
    roslaunch prophesee_recorder stereo_record_prophesees.launch \
    output_root:=/home/dani/Documents/temp \
    events_topic_right:=/prophesee/camera/right/cd_events_buffer \
    imu_topic_right:=/prophesee/camera/right/imu \
    trigger_topic_right:=/prophesee/camera/right/extTrigger \
    events_topic_left:=/prophesee/camera/left/cd_events_buffer \
    imu_topic_left:=/prophesee/camera/left/imu \
    trigger_topic_left:=/prophesee/camera/left/extTrigger \    
    event_packet_size:=100000 \
    imu_packet_size:=1000 \
    v:=1                                      
'''

This creates a folder with name `/home/dani/Documents/temp` where all the data is stored. 
After recording, kill the node to write the last packets of events/imu/triggers.

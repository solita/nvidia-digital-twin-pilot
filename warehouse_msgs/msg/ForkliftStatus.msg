string forklift_id
uint8 state
geometry_msgs/Pose pose
float32 battery_level
string current_task_id

uint8 IDLE=0
uint8 NAVIGATING_TO_SHELF=1
uint8 PICKING=2
uint8 NAVIGATING_TO_DOCK=3
uint8 DROPPING=4
uint8 ERROR=5
uint8 RECOVERING=6

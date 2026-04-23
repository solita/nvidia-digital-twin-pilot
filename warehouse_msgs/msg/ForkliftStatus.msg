# Forklift state enum:
# 0=IDLE, 1=NAVIGATING_TO_SHELF, 2=PICKING, 3=NAVIGATING_TO_DOCK, 4=DROPPING, 5=ERROR, 6=RECOVERING

string forklift_id
uint8 state
geometry_msgs/Pose pose
float32 battery_level
string current_task_id
builtin_interfaces/Time stamp

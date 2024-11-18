byte command           # Command type
string seq             # Unique identifier of the command
byte attribute         # Attribute selection
uint16[] floor_index   # Floor index number
uint16[] pcdmap_index  # PCD map index number
uint16[] topomap_index # Topomap index number
uint16[] node_edge_name # Name of the node or edge

float32 quaternion_x
float32 quaternion_y
float32 quaternion_z
float32 quaternion_w
float32 euler_roll
float32 euler_pitch
float32 euler_yaw
float32 translation_x
float32 translation_y
float32 translation_z
uint16[] state_1
float32[] state_2

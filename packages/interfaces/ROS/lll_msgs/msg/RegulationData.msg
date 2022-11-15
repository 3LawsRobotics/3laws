std_msgs/Header header

uint64 nu  # Number of inputs
uint64 n_safety_cstr  # Number of inputs
uint64 n_failsafes  # Number of inputs

InputConstraints input_cstr

float64[] lfh
float64[] lgh
float64[] safety_val
float64[] failsafe_input


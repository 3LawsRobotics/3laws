# The type of the shape
int32 POINT = 0
int32 SPHERE = 1
int32 CAPSULE = 2
int32 PLANE = 3
int32 BOX = 4

int32 type

# Dimensions of the shape
# POINT: []
# SPHERE: [radius]
# CAPSULE: [radius, length]
# PLANE: [normal_x, normal_y, normal_z, distance]
# BOX: [size_x, size_y, size_z]
float64[] parameters

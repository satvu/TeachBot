# Adjusts one field of Sawyer's current pose.
# This changes a field of Sawyer's current pose to a specified value.
# This is different from AdjustPoseBy, which increments the field rather than resetting it.
#
#Geometry you are setting
# 'position': the x, y, or z position coordinate
# 'orientation': the x, y, z, or w orientation
string geometry
#Axis along which you are setting
# 'x': the x-axis
# 'y': the y-axis
# 'z': the z-axis
# 'w': the w-axis, only available if geometry is 'orientation'
string axis
# amount to set the pose value
float32 amount

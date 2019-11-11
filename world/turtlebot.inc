define hokuyolaser ranger
(
  sensor
  (
    range_max 5.6
    fov 360
    samples 720
  )
  # generic model properties
  color "blue"
  size [ 0.05 0.05 0.05 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "gps"
  # odom_error [0.03 0.03 0 0 0 0.02]

  size [ 0.35 0.35 0.10 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  # gui_nose 1
  drive "diff"
  
  block( # this block approximates the circular shape of a turtlebot
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0 0.1]
  )

  color "gray"
  
  hokuyolaser(pose [ 0.0 0.0 0.1 0.0 ])
)

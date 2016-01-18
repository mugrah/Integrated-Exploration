
define pioneer_base position
(
  color "red"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  ranger_return 0.5            	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders
  fiducial_return 1           	# Seen as "1" fiducial finders

  #localization "gps"
  #localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  odom_error [ 0.05 0.05 0.0 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                        # (Uniform random distribution)

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-0.7 0.7 0 0 0 0 -140.0 140.0 ]
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]
)






define pioneer3at pioneer_base
(
  # Actual size
  size [0.508 0.381 0.277]

  # The pioneer's center of rotation is offset from its center of area
  origin [-0.04465 0.0 0.0 0.0]

  # Estimated mass in KG
  mass 12.0

  # Body shape:
  block(
    points 8
    point[0] [-0.254 -0.140 ]
    point[1] [-0.204 -0.190 ]
    point[2] [ 0.204 -0.190 ]
    point[3] [ 0.254 -0.140 ]
    point[4] [ 0.254  0.140 ]
    point[5] [ 0.204  0.190 ]
    point[6] [-0.204  0.190 ]
    point[7] [-0.254  0.140 ]
    z [ 0.08 0.277]
  )

  # left front wheel
  block
  (
    points 4
    point[0] [0.254 0.193]
    point[1] [0.254 0.283]
    point[2] [0.032 0.283]
    point[3] [0.032 0.193]
    z [0 0.222]
    color "gray15"
  )

  # left back wheel
  block
  (
    points 4
    point[0] [-0.254 0.193]
    point[1] [-0.254 0.283]
    point[2] [-0.032 0.283]
    point[3] [-0.032 0.193]
    z [0 0.222]
    color "gray15"
  )

  # right front wheel
  block
  (
    points 4
    point[0] [0.254 -0.193]
    point[1] [0.254 -0.283]
    point[2] [0.032 -0.283]
    point[3] [0.032 -0.193]
    z [ 0 0.222]
    color "gray15"
  )

  # right back wheel
  block
  (
    points 4
    point[0] [-0.254 -0.193]
    point[1] [-0.254 -0.283]
    point[2] [-0.032 -0.283]
    point[3] [-0.032 -0.193]
    z [ 0 0.222]
    color "gray15"
   )

  # Use the sonar array defined above
  #p3at_sonar( pose [ 0 0 -0.03 0 ] )
)

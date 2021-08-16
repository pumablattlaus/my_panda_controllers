# my_panda_controllers

run in Terminal:
rostopic pub /my_cartesian_velocity_controller/my_velocities my_panda_controllers/MyVelocity "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
vel: [0.00, 0.005, 0.0, 0.0, 0.0, 0.0]"


cartesian_motion_generator_joint_velocity_discontinuity: https://frankaemika.github.io/docs/libfranka.html
      https://frankaemika.github.io/docs/control_parameters.html#control-parameters-specifications
      begrenzung im Controller durch auslesen Topic und ändern der Werte?
      
velocity/acc_discontinuity abhängig von Momentan-Pose des Roboters

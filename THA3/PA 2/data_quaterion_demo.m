%solving the ax=xb equation using quartornian method
%script written by Carl Stott on 4/23/22

[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion();


X = Hand_Eye_Calibration(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config )
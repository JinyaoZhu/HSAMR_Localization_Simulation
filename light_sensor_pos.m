function pos=light_sensor_pos(state,params)
x = state(1);
y = state(2);
theta = state(3);
body_pose_l = [params.ROBOT_LENGTH_X/2; params.ROBOT_LENGTH_Y/2];
body_pose_r = [params.ROBOT_LENGTH_X/2;-params.ROBOT_LENGTH_Y/2];
rotate = [cos(theta),-sin(theta);
           sin(theta),cos(theta)];
reference_pose_l = rotate*body_pose_l + [x;y];
reference_pose_r = rotate*body_pose_r + [x;y];

pos = [reference_pose_r,reference_pose_l];
   
end
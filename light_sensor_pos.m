function pos=light_sensor_pos(state,params)
x = state(1);
y = state(2);
theta = state(3);
body_pose_l = [9; 2.5];%cm
body_pose_r = [9;-2.5];
rotate = [cos(theta),-sin(theta);
           sin(theta),cos(theta)];
reference_pose_l = rotate*body_pose_l + [x;y];
reference_pose_r = rotate*body_pose_r + [x;y];

pos = [reference_pose_r,reference_pose_l;
        pi/2,-pi/2];
   
end
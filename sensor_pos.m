function pos = sensor_pos(state,params)

theta = state(3);

R = [cos(theta),-sin(theta);
     sin(theta),cos(theta)];
 
ir_1_pos = [params.ROBOT_LENGTH_X/2;0];
ir_2_pos = [params.ROBOT_LENGTH_X/4;-params.ROBOT_LENGTH_Y/2];
ir_3_pos = [params.ROBOT_LENGTH_X/4;params.ROBOT_LENGTH_Y/2];
ir_4_pos = [-params.ROBOT_LENGTH_X/2;0];

ir_1_pos = state(1:2) + R*ir_1_pos;
ir_2_pos = state(1:2) + R*ir_2_pos;
ir_3_pos = state(1:2) + R*ir_3_pos;
ir_4_pos = state(1:2) + R*ir_4_pos;

pos = [ir_1_pos,ir_2_pos,ir_3_pos,ir_4_pos;
       theta+0,theta-pi/2,theta+pi/2,theta-pi];
end
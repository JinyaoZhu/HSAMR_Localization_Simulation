% get sensors pose in global frame
% input:
%   state:robot pose
function pos = sensor_pos(state,params)

theta = state(3);

R = [cos(theta),-sin(theta);
     sin(theta),cos(theta)];
 
ir_1_pos = state(1:2) + R* params.IR_1_POSE(1:2);
ir_2_pos = state(1:2) + R* params.IR_2_POSE(1:2);
ir_3_pos = state(1:2) + R* params.IR_3_POSE(1:2);
ir_4_pos = state(1:2) + R* params.IR_4_POSE(1:2);

pos = [ir_1_pos,ir_2_pos,ir_3_pos,ir_4_pos;
       theta+params.IR_1_POSE(3),...
       theta+params.IR_2_POSE(3),...
       theta+params.IR_3_POSE(3),...
       theta+params.IR_4_POSE(3)];
end
function params = sys_param()
params.MAP_X_MAX = 260; %cm
params.MAP_Y_MAX = 140; %cm
params.MAP_XY_MIN = 1; %cm
params.CAR_WHEEL_L = 2.8; %cm
params.CAR_WHEEL_R = 2.8; %cm
params.CAR_WIDTH = 13.5; %cm
% only for display
params.ROBOT_LENGTH_X = 18; %cm
params.ROBOT_LENGTH_Y = 10; %cm
params.MAX_IR_RANGE = 80; %cm
params.MIN_IR_RANGE = 10; %cm

params.IR_NOISE = 1; % +- 1cm std normal PDF
params.ENCODER_NOISE = 0.5; % +- rad/s

% sensor pose in body frame
params.IR_1_POSE = [params.ROBOT_LENGTH_X/2;0;0];
params.IR_2_POSE  = [params.ROBOT_LENGTH_X/4;-params.ROBOT_LENGTH_Y/2;-pi/2];
params.IR_3_POSE  = [params.ROBOT_LENGTH_X/4;params.ROBOT_LENGTH_Y/2;pi/2];
params.IR_4_POSE  = [-params.ROBOT_LENGTH_X/2;0;-pi];
% Transform matrix (vector in sensor frame represent in body frame)
params.T_b_s1 = [cos(params.IR_1_POSE(3)),-sin(params.IR_1_POSE(3)),params.IR_1_POSE(1);
    sin(params.IR_1_POSE(3)),cos(params.IR_1_POSE(3)),params.IR_1_POSE(2);
    0,0,1];
params.T_b_s2 = [cos(params.IR_2_POSE(3)),-sin(params.IR_2_POSE(3)),params.IR_2_POSE(1);
    sin(params.IR_2_POSE(3)),cos(params.IR_2_POSE(3)),params.IR_2_POSE(2);
    0,0,1];
params.T_b_s3 = [cos(params.IR_3_POSE(3)),-sin(params.IR_3_POSE(3)),params.IR_3_POSE(1);
    sin(params.IR_3_POSE(3)),cos(params.IR_3_POSE(3)),params.IR_3_POSE(2);
    0,0,1];
params.T_b_s4 = [cos(params.IR_4_POSE(3)),-sin(params.IR_4_POSE(3)),params.IR_4_POSE(1);
    sin(params.IR_4_POSE(3)),cos(params.IR_4_POSE(3)),params.IR_4_POSE(2);
    0,0,1];
end
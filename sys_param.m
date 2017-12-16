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
end
function u = controller(state,target,t,light_sensor_meas,params)

W = params.CAR_WIDTH;
Rl = params.CAR_WHEEL_L;
Rr = params.CAR_WHEEL_R;

persistent u_out;
persistent v;
persistent omega;

if isempty(omega)
    omega = 0;
end

v = 15;

if light_sensor_meas(1)==0 && light_sensor_meas(2)==1
    omega = 1.4;
elseif light_sensor_meas(1)==1 && light_sensor_meas(2)==0
    omega = -1.4;
end

% if light_sensor_meas(2) <=1
%     u_out = [4;1];
% elseif light_sensor_meas(1) <=1
%     u_out = [1;4];
% end

% v = 18;
% if light_sensor_meas(1) == 0
%     if light_sensor_meas(2) ~= 0
%         omega = -v*0.01*14.8;
%     end
% end
% 
% if light_sensor_meas(2) == 0
%     if light_sensor_meas(1) ~= 0
%         omega = v*0.01*14.8;
%     end
% end
% 
% if light_sensor_meas(1) > 0 && light_sensor_meas(2) > 0
%     omega = 0.01*(light_sensor_meas(1) - light_sensor_meas(2));
% end
J = [Rr/2,Rl/2;Rr/W,-Rl/W];
u_out = J\[v;omega];

u =u_out;

function u = controller(state,target,t,light_sensor_meas,params)

W = params.CAR_WIDTH;
Rl = params.CAR_WHEEL_L;
Rr = params.CAR_WHEEL_R;

persistent u_out;

if isempty(u_out)
    u_out = [4;1];
end

if light_sensor_meas(2)==1 && light_sensor_meas(1)==0
    u_out = [4;1];
elseif light_sensor_meas(1)==1 && light_sensor_meas(2)==0
    u_out = [1;4];
end
u =u_out;
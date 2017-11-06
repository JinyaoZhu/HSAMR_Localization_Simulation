clear all;
close all;

params = sys_param();

controlhandle = @controller;

% map = gen_map_square([params.MAP_X_MAX,params.MAP_Y_MAX]);
map = gen_map_1([params.MAP_X_MAX,params.MAP_Y_MAX]);
line_map = gen_line_map([params.MAP_X_MAX,params.MAP_Y_MAX]);

[x_real,x_est,t,measurement_hist,temp_data_hist] = simulation(map,line_map,params,controlhandle);

figure(2);
subplot(3,1,1);
plot(t,(x_real(1,:)-x_est(1,:)));
title('x error(cm)')
grid;

subplot(3,1,2);
plot(t,(x_real(2,:)-x_est(2,:)));
title('y error(cm)')
grid;

subplot(3,1,3);
error_angle = zeros(1,length(x_est(3,:)));
for i=1:length(x_est(3,:))
    error_angle(i) = warp_to_pi(x_real(3,i)-x_est(3,i));
end
plot(t,(error_angle));
title('\theta error(rad)')
grid;

figure(3)
plot(t,measurement_hist(2,:),t,measurement_hist(3,:));
xlabel('t/s');
ylabel('d/cm');
legend('sensor2','sensor3');
grid;

% figure(4)
% plot(t,temp_data_hist(1,:));
% xlabel('t/s');
% grid;
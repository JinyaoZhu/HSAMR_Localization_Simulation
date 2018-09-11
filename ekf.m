function E_x = ekf(x,u,z,dt,map,params)

persistent P;
if isempty(P)
    P = eye(3);
end

% prior estimation
W = params.CAR_WIDTH;
Rl = params.CAR_WHEEL_L;
Rr = params.CAR_WHEEL_R;
dV = dt*[Rr/2,Rl/2;Rr/W,-Rl/W]*u;

x_prior = zeros(3,1);
if abs(dV(2))>1e-5
    R = dV(1)/dV(2);
    x_prior(1) = x(1) + R*(sin(x(3)+dV(2))-sin(x(3)));
    x_prior(2) = x(2) + R*(cos(x(3))-cos(x(3)+dV(2)));
else
    x_prior(1) = x(1) + dV(1)*cos(x(3));
    x_prior(2) = x(2) + dV(1)*sin(x(3));
end
x_prior(3) = warp_to_pi(x(3)+dV(2));
F = dt*eye(3);
% model noise
L = dt*eye(3)*[cos(x(3)),0;sin(x(3)),0;0,1]*[Rr/2,Rl/2;Rr/W,-Rl/W];
Q = L*((4*params.ENCODER_NOISE)^2*eye(2))*L';

R = params.IR_NOISE^2*eye(4);

P = F*P*F'+Q;

H = zeros(4,3);

[~,z_predicted] = get_sensor(sensor_pos(x_prior,params),map,params);

% z_b measured distance between landmarks and robot's center
% z_b = z;
% z_b(1) = norm(params.T_b_s1 * [z(1);0;1] - [0;0;1]);
% z_b(2) = norm(params.T_b_s2 * [z(2);0;1] - [0;0;1]);
% z_b(3) = norm(params.T_b_s3 * [z(3);0;1] - [0;0;1]);
% z_b(4) = norm(params.T_b_s4 * [z(4);0;1] - [0;0;1]);
% % predicted measurement
% z_b_predicted = z;
% z_b_predicted(1) = norm(p_i(1:2,1) - x(1:2));
% z_b_predicted(2) = norm(p_i(1:2,2) - x(1:2));
% z_b_predicted(3) = norm(p_i(1:2,3) - x(1:2));
% z_b_predicted(4) = norm(p_i(1:2,4) - x(1:2));
% 
% y = z_b - z_b_predicted;
%  
% H(1,:) = [x(1) - p_i(1,1),x(2) - p_i(2,1),0]/z_b_predicted(1);
% H(2,:) = [x(1) - p_i(1,2),x(2) - p_i(2,2),0]/z_b_predicted(2);
% H(3,:) = [x(1) - p_i(1,3),x(2) - p_i(2,3),0]/z_b_predicted(3);
% H(4,:) = [x(1) - p_i(1,4),x(2) - p_i(2,4),0]/z_b_predicted(4);

dx = 1;
dy = 1;
dtheta = 0.06;

% compute the Jacobian of the measurement model H at point x_prior
for i = 1:4
    sensor_pose_1 = sensor_pos([x_prior(1)+dx;x_prior(2);x_prior(3)],params);
    sensor_pose_2 = sensor_pos([x_prior(1)-dx;x_prior(2);x_prior(3)],params);
    [~,d1] = get_scan_dist(sensor_pose_1(:,i),map,params.MAX_IR_RANGE);
    [~,d2] = get_scan_dist(sensor_pose_2(:,i),map,params.MAX_IR_RANGE);
    H(i,1) = (d1-d2)/(2*dx);
    
    sensor_pose_1 = sensor_pos([x_prior(1);x_prior(2)+dy;x_prior(3)],params);
    sensor_pose_2 = sensor_pos([x_prior(1);x_prior(2)-dy;x_prior(3)],params);
    [~,d1] = get_scan_dist(sensor_pose_1(:,i),map,params.MAX_IR_RANGE);
    [~,d2] = get_scan_dist(sensor_pose_2(:,i),map,params.MAX_IR_RANGE);
    H(i,2) = (d1-d2)/(2*dy);
    
    sensor_pose_1 = sensor_pos([x_prior(1);x_prior(2);warp_to_pi(x_prior(3)+dtheta)],params);
    sensor_pose_2 = sensor_pos([x_prior(1);x_prior(2);warp_to_pi(x_prior(3)-dtheta)],params);
    [~,d1] = get_scan_dist(sensor_pose_1(:,i),map,params.MAX_IR_RANGE);
    [~,d2] = get_scan_dist(sensor_pose_2(:,i),map,params.MAX_IR_RANGE);
    H(i,3) = (d1-d2)/(2*dtheta);
end

S = H*P*H' + R;
K = P*H'/S;
y = z - z_predicted;
E_x = x_prior + K*y;
P = (eye(3)-K*H)*P;
E_x(3) = warp_to_pi(E_x(3));
end
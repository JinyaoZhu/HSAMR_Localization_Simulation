function E_x = ekf(x,u,z,dt,map,params)

persistent P;
if isempty(P)
    P = eye(3);
end

% prior estimation
x_prior = x + dt*carEOM(x,u,params);
x_prior(3) = warp_to_pi(x_prior(3));
F = dt*eye(3);
% model noise
Q = [0.3^2 0 0;
    0 0.3^2 0;
    0 0    0.01^2];

R = 4*params.IR_NOISE^2*eye(4);

P = F*P*F'+Q;

sensor_pose = sensor_pos(x_prior,params);

[~,z_predicted] = get_sensor(sensor_pose,map,params);

y = z - z_predicted;

H = zeros(4,3);

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
K = P*H'*S^(-1);
E_x = x_prior + K*y;
P = (eye(3)-K*H)*P;
E_x(3) = warp_to_pi(E_x(3));
end
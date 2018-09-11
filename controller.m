function u = controller(state,target_x,t,light_sensor_meas,params)

W = params.CAR_WIDTH;
Rl = params.CAR_WHEEL_L;
Rr = params.CAR_WHEEL_R;

FOLLOW_LINE= true;

persistent u_out;
persistent v;
persistent omega;

if isempty(omega)
    omega = 0;
end
%%%%%%%%%%%%%%%%%%%%FOLLOW LINE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if FOLLOW_LINE
    v = 15;
    
    if light_sensor_meas(1)==0 && light_sensor_meas(2)==1
        omega = 1.4;
    elseif light_sensor_meas(1)==1 && light_sensor_meas(2)==0
        omega = -1.4;
    end
else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%GOTO TARGET%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    persistent i_target;
    
    if isempty(i_target)
        i_target = 1;
    end
    
    target = target_x(i_target,:)';
    
    kp = 1.2;
    ka = 2.5;
    kb = -2.5;
    a = warp_to_pi(atan2(target(2)-state(2),target(1)-state(1))-state(3));
    p = norm(state(1:2)-target(1:2));
    
    if a > pi / 2
        p = -p ;
        a = a - pi;
    elseif a <= -pi / 2
        p = -p ;
        a = a + pi;
    end
    
    phi = warp_to_pi(target(3)-state(3));
    b = warp_to_pi(phi - a);
    
    omega = ka*a + kb*b + kp*sin(2*a) - kp*sin(2*b);
    v= kp*cos(a)*p;
    v = 0.5*v + 0.5*min(v,20);
    omega = 0.5*omega + 0.5*max(-3,min(omega,3));
    
    if((p < 0.05)&&(abs(phi)<0.05))
        if(i_target < length(target_x(:,1)))
            i_target = i_target+1;
        else
            v = 0;
            omega=0;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

function [E_x,current_state] = complementary_filter(x,u,z,dt,params,x_real)
waypoints = [30,45;
    210,45;
    210,105;
    180,105;
    180,75;
    60,75;
    60,105;
    30,105];

% update_points = [33.8490   43.2928   -0.5082;
%                 209.5253   47.3481    0.8745;
%                 205.3303  104.7549    2.6070;
%                 180.1587  101.8389   -2.2902;
%                 175.2981   74.2171   -2.5872;
%                  58.9214   78.6820    2.1120;
%                  55.2238  104.8206    2.6070;
%                 30.0522  101.9046   -2.2902];

target_theta = [0,pi/2,pi,-pi/2,pi,pi/2,pi,-pi/2];

persistent state;

if(isempty(state))
    state = 1;
end

% prior estimation
E_x = x + dt*carEOM(x,u,params);
E_x(3) = warp_to_pi(E_x(3));

persistent corner_trigger;

if(isempty(corner_trigger))
    corner_trigger = 0;
end

persistent last_diff;
if(isempty(last_diff))
    last_diff = 0;
end

diff = u(1)-u(2);
if(diff)>0
    if(last_diff < 0)
        corner_trigger = 1;
    else
        corner_trigger = corner_trigger+1;
    end
elseif(diff)<0
    if(last_diff > 0)
        corner_trigger = -1;
    else
        corner_trigger = corner_trigger-1;
    end
end

persistent state_change_time;

if isempty(state_change_time)
    state_change_time = 0;
end

% update estimated state when the robot reach the conner
if(abs(corner_trigger) > 20 && state_change_time > 2)
    corner_trigger = 0;
    state_change_time = 0;
    state = state+1;
    if(state > length(waypoints))
        state = 1;
    end
    
    %     E_x(1:2) = E_x(1:2) + 1.0*(update_points(state,1:2)' - E_x(1:2));
    %     E_x(3) = E_x(3) + 1.0*(warp_to_pi(update_points(state,3) - E_x(3)));
elseif(abs(corner_trigger) < 10)
%      % update robot heading with the light sensors
    if z(1) == 1 && z(2)==0
        E_x(3) = E_x(3) + 0.1*(warp_to_pi(target_theta(state)+0.17 - E_x(3)));
    elseif z(1)==0 && z(2)==1
        E_x(3) = E_x(3) + 0.1*(warp_to_pi(target_theta(state)-0.17 - E_x(3)));
    end
     % update robot position with the light sensors
    if state == 1||state == 3||state == 5||state == 7
        E_x(2) =  E_x(2) + 0.1*(waypoints(state,2) - E_x(2));
    else
        E_x(1) =  E_x(1) + 0.1*(waypoints(state,1) - E_x(1));
    end
end
E_x(3) = warp_to_pi( E_x(3));
last_diff = diff;
state_change_time = state_change_time+dt;
current_state = state;
end
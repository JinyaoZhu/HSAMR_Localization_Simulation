function [endpoint,distance] = get_scan_dist(pose,map, max_range)
x0 = pose(1);
y0 = pose(2);
theta = pose(3);
theta = warp_to_pi(theta);
theta_deg = theta*180/pi;
tan_theta = tan(theta);
inv_tan_theta = 1/tan_theta;
x = x0;
y = y0;
MAP_X_MAX = length(map(:,1));
MAP_Y_MAX = length(map(1,:));
MAP_XY_MIN = 1;
while (true)
    
    if x>MAP_X_MAX||y>MAP_Y_MAX||x<MAP_XY_MIN||y<MAP_XY_MIN
        break;
    end
    
    dx = x - x0;
    dy = y - y0;
    if(sqrt(dx*dx+dy*dy) >= max_range)
        break;
    end
    
    if( map(round(x),round(y)) == true )
        break;
    end
    
    if theta_deg >= -45 && theta_deg < 45
        y = y + tan_theta;
        x = x + 1;
    elseif theta_deg >= 45 && theta_deg < 135
        x = x + inv_tan_theta;
        y = y + 1;
    elseif theta_deg >= -135 && theta_deg < -45
        x = x - inv_tan_theta;
        y = y - 1;
        % theta >=135 && theta < -135
    else
        y = y - tan_theta;
        x = x - 1;
    end
end

dx = x - x0;
dy = y - y0;

endpoint = [x;y];
distance = sqrt(dx*dx+dy*dy);

end
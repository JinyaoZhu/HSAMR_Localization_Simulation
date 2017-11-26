function measure = get_light_sensor(state,params,line_map)
sensor_pose = light_sensor_pos(state,params);
m_r = line_map(round(sensor_pose(1,1)),round(sensor_pose(2,1))); % right light sensor
m_l = line_map(round(sensor_pose(1,2)),round(sensor_pose(2,2))); % left light sensor
% [~,m_r] = get_scan_dist(sensor_pose(:,1),line_map);
% [~,m_l] = get_scan_dist(sensor_pose(:,2),line_map);
% 
% m_r = min(max(m_r,0),3);
% m_l = min(max(m_l,0),3);

measure = [m_r;m_l];
end
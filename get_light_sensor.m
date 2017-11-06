function measure = get_light_sensor(state,params,line_map)
sensor_pose = light_sensor_pos(state,params);
m_r = line_map(round(sensor_pose(1,1)),round(sensor_pose(2,1))); % right light sensor
m_l = line_map(round(sensor_pose(1,2)),round(sensor_pose(2,2))); % left light sensor
measure = [m_r;m_l];
end
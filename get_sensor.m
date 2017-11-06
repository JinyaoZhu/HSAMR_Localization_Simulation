function [endpoints,dist] = get_sensor(sensor_pos,map)

[endpoint_1,d1]=get_scan_dist(sensor_pos(:,1),map);
[endpoint_2,d2]=get_scan_dist(sensor_pos(:,2),map);
[endpoint_3,d3]=get_scan_dist(sensor_pos(:,3),map);
[endpoint_4,d4]=get_scan_dist(sensor_pos(:,4),map);

endpoints = [endpoint_1,endpoint_2,endpoint_3,endpoint_4];
dist = [d1,d2,d3,d4]';
end
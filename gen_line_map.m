% generate a map of the line that the robot should follow
function map = gen_line_map(size)
lx = size(1);
ly = size(2);
map = zeros(lx,ly,'logical');
waypoints = [30,45;
             210,45;
             210,105;
             180,105;
             180,75;
             60,75;
             60,105;
             30,105];

half_line_width = 2;
         
for i=waypoints(1,2)-half_line_width:waypoints(1,2)+half_line_width
    for j = waypoints(1,1)-half_line_width:waypoints(2,1)+half_line_width
        map(j,i) = 1;
    end
end

for i=waypoints(2,1)-half_line_width:waypoints(2,1)+half_line_width
    for j = waypoints(2,2)-half_line_width:waypoints(3,2)+half_line_width
        map(i,j) = 1;
    end
end

for i=waypoints(4,2)-half_line_width:waypoints(4,2)+half_line_width
    for j = waypoints(4,1)-half_line_width:waypoints(3,1)+half_line_width
        map(j,i) = 1;
    end
end

for i=waypoints(4,1)-half_line_width:waypoints(4,1)+half_line_width
    for j = waypoints(5,2)-half_line_width:waypoints(4,2)+half_line_width
        map(i,j) = 1;
    end
end

for i=waypoints(5,2)-half_line_width:waypoints(5,2)+half_line_width
    for j = waypoints(6,1)-half_line_width:waypoints(5,1)+half_line_width
        map(j,i) = 1;
    end
end

for i=waypoints(6,1)-half_line_width:waypoints(6,1)+half_line_width
    for j = waypoints(6,2)-half_line_width:waypoints(7,2)+half_line_width
        map(i,j) = 1;
    end
end

for i=waypoints(7,2)-half_line_width:waypoints(7,2)+half_line_width
    for j = waypoints(8,1)-half_line_width:waypoints(7,1)+half_line_width
        map(j,i) = 1;
    end
end

for i=waypoints(8,1)-half_line_width:waypoints(8,1)+half_line_width
    for j = waypoints(1,2)-half_line_width:waypoints(8,2)+half_line_width
        map(i,j) = 1;
    end
end

end
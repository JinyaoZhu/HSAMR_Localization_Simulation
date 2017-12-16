function v = measure_voltage(dist,params)
v = zeros(4,1);
MIN_DIST = params.MIN_IR_RANGE;
MAX_DIST = params.MAX_IR_RANGE;
if dist(1) <= MIN_DIST
    v(1) = 0.66*dist(1);
elseif dist(1) < MAX_DIST
    v(1) = 32.73/(dist(1)+4.92);
else
    v(1)=0;
end
    
if dist(2) <= MIN_DIST
    v(2) = 0.66*dist(2);
elseif dist(2) < MAX_DIST
    v(2) = 32.73/(dist(2)+4.92);
else
    v(2)=0;
end

if dist(3) <= MIN_DIST
    v(3) = 0.66*dist(3);
elseif dist(3) < MAX_DIST
    v(3) = 32.73/(dist(3)+4.92);
else
    v(3)=0;
end

if dist(4) <= MIN_DIST
    v(4) = 0.66*dist(4);
elseif dist(4) < MAX_DIST
    v(4) = 32.73/(dist(4)+4.92);
else
    v(4)=0;
end

end
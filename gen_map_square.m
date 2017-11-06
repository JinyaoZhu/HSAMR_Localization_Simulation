function map = gen_map_square(size)
lx = size(1);
ly = size(2);
map = zeros(lx,ly,'logical');
for i=1:lx
    map(i,1) = 1;
end
for i=1:lx
    map(i,ly) = 1;
end
for i=1:ly
    map(1,i) = 1;
end
for i=1:ly
    map(lx,i) = 1;
end
% map = uint8(map);
end
function map = gen_map_1(size)
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

a = 90+10;
b = 60+10;
c = 30+5;
d = 30+5;
e = 30;
g = 60+10;
h = 30+5;
j = 30;
m = 45;
n = 15+5;

l = h+j;

% walls
for i=1:e
    map(m,i) = 1;
end
for i=(m-c):m
    map(i,e) = 1;
end
for i=e:e+a
    map(m-c,i) = 1;
end
for i=m-c:m-c+b
    map(i,a+e) = 1;
end
for i=a+e-d:a+e
    map(m-c+b,i) = 1;
end
for i=1:e
    map(lx-l,i) = 1;
end
for i=lx-l:lx-l+h
    map(i,e) = 1;
end
for i=e:e+n
    map(lx-j,i) = 1;
end
for i=lx-j:lx
    map(i,e+n) = 1;
end
for i=lx-j:lx
    map(i,a+e-n) = 1;
end
for i=a+e-n:a+e
    map(lx-j,i) = 1;
end
for i=lx-j-g:lx-j
    map(i,a+e) = 1;
end
for i=a+e-d:a+e
    map(lx-j-g,i) = 1;
end

% obstacles
for i=m+1:m+1+7
    map(i,e) = 1;
end
for i=m+1:m+1+7
    map(i,e-15) = 1;
end
% for i=e-15:e
%     map(m+1,i) = 1;
% end
for i=e-15:e
    map(m+1+7,i) = 1;
end

for i=90+15:90+15+45
    map(i,e) = 1;
end
for i=90+15:90+15+45
    map(i,e-15) = 1;
end
for i=e-15:e
    map(90+15,i) = 1;
end
for i=e-15:e
    map(90+15+45,i) = 1;
end


for i=e+n:e+n+15
    map(lx-j,i) = 1;
end
for i=e+n:e+n+15
    map(lx-j+15,i) = 1;
end
for i=lx-j:lx-j+15
    map(i,e+n+15) = 1;
end

for i=m-c+b:m-c+b+15
    map(i,a+e-d) = 1;
end
for i=m-c+b:m-c+b+15
    map(i,a+e-d+15) = 1;
end
for i=a+e-d:a+e-d+15
    map(m-c+b+15,i) = 1;
end
end
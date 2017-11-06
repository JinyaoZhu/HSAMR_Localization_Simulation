function y = med_filter_31(x)
window_width = 31;
persistent buffer;
if isempty(buffer)
    buffer = x*ones(1,window_width);
end
persistent index;
if isempty(index)
    index = 1;
end
buffer(index) = x;
index = index+1;
if index > window_width
    index = 1;
end
y = median(buffer);
end
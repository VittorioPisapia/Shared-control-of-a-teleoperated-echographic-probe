clear all
clc

joy = vrjoystick(1);

while true
    [axes,buttons] = read(joy);
    r3=round(-atan2(axes(5),axes(4)),2) 
    disp(axes)
end
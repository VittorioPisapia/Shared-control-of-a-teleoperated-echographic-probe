clear all
clc

joy = vrjoystick(1);

while true
    [axes,buttons] = read(joy);
    disp(buttons);
end
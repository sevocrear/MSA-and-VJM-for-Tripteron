function [T] = dtrotx(angle)
T = [0, 0, 0, 0;
    0, -sin(angle), -cos(angle),0;
    0, cos(angle), -sin(angle),0;
    0,0,0,0];
end
function [T] = dtroty(angle)
T = [ -sin(angle), 0,  cos(angle), 0;
               0, 0,           0, 0;
     -cos(angle), 0, -sin(angle), 0;
               0, 0,           0, 0];
end
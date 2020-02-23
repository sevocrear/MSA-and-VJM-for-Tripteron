function [T] = dtrotz(angle)
T = [ -sin(angle), -cos(angle), 0, 0;
  cos(angle), -sin(angle), 0, 0;
           0,           0, 0, 0;
           0,           0, 0, 0];
end
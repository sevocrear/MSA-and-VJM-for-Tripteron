function [k11, k12, k22] = k_cylinder(E, G, d, L, S, Iy, Iz)
% The stiffness matrix K_theta for cylinder link
J = Iy + Iz;

%the stiffness matrices of the MSA model
k11 = [E*S/L 0                  0                 0           0                 0;    
    0           12*E*Iz/L^3  0                 0           0                 6*E*Iz/L^2;
    0           0                  12*E*Iy/L^3 0           -6*E*Iy/L^2 0;
    0           0                  0                 G*J/L 0                 0;
    0           0                  -6*E*Iy/L^2 0           4*E*Iy/L      0;
    0           6*E*Iz/L^2   0                 0           0                 4*E*Iz/L];

k12 = [-E*S/L 0                  0                 0           0                 0;
    0           -12*E*Iz/L^3  0                 0           0                 6*E*Iz/L^2;
    0           0                  -12*E*Iy/L^3 0           -6*E*Iy/L^2 0;
    0           0                  0                 -G*J/L 0                 0;
    0           0                  6*E*Iy/L^2 0           2*E*Iy/L      0;
    0           -6*E*Iz/L^2   0                 0           0                 2*E*Iz/L];

k22 = [E*S/L 0                  0                 0           0                 0;
    0           12*E*Iz/L^3  0                 0           0                 -6*E*Iz/L^2;
    0           0                  12*E*Iy/L^3 0           6*E*Iy/L^2 0;
    0           0                  0                 G*J/L 0                 0;
    0           0                  6*E*Iy/L^2 0           4*E*Iy/L      0;
    0           -6*E*Iz/L^2   0                 0           0                 4*E*Iz/L];

end

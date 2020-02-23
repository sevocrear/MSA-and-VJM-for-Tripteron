% Innopolis University
% Advanced Robotic Manipulation
% Homework 1
%
% Calculate robot stiffness matrix for 1st serial chain
%
% Using:
% [Kc] = VJM_lin_1(Tbase,Ttool,q0,q,t,L,l,d);
% Input: Tbase Ttool - transformations matrix of base and tool
%        q0 - active joint angle
%        q  - passive joint angle
%        t - virtual joint angle
%        L,l,d - robot parameters
% Output: Kc - 6*6 stiffness matrix

function [Kc] = VJM_part(Jq, Jt, link_length, materials_params)


k0 = materials_params(1); % Actuator stiff
%material and shape parameters
E = materials_params(2); %Young's modulus
G = materials_params(3); %shear modulus
d = materials_params(4); % Link's diameter

%for cylinder
S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;

L = link_length;

[k11, k12, k22] = k_cylinder(E, G, d, L, S, Iy, Iz);

% K theta matrix
Kt = [k0 zeros(1,12)
    zeros(6,1) k22 zeros(6,6)
    zeros(6,1) zeros(6,6) k22];

% Numerical solution
% SM=inv([Jt*inv(Kt)*Jt' Jq;  Jq' zeros(1,1)]);
% Kc=SM(1:6,1:6);

% Analytical solution
Kc0=inv(Jt*inv(Kt)*Jt');
% Kcq = inv(Jq'*inv(Kc0)*Jq)*Jq'*inv(Kc0);
% Kc = Kc0-Kc0*Jq*Kcq;

Kc = Kc0 - Kc0*Jq*inv(Jq'*Kc0*Jq)*Jq'*Kc0;
end


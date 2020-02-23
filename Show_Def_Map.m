%That's the main m-file of the VJM and MSA method's deflection map building. It gives the operator the
%diflection map under the torque and force vector F
% F - (Fx,Fy,Fz,Mx,My,Mz)

%Robot - Tripteron

%% Let's define parameters
clc;
clear;
format short;

% Force and Torque Vector
F=[0 0 100 0 0 0]';

% Configuration (1: elbow-down, 2: elbow-up)
configuration = 1;

% Material's parameters:
k0 = 1e6; % Actuator stiff
%material and shape parameters
E = 7.0e10; %Young's modulus
G = 2.55e10; %shear modulus
d = 0.15; % Link's diameter
materials_params = [k0, E, G, d]; 

link_length = 0.75; % the length of the links (suggested, they all are the same)

% Determination of the operation space
space_x = round(2*sqrt(3)*link_length/3,1);
space_y = round(2*sqrt(3)*link_length/3,1);
space_z = round(2*sqrt(3)*link_length/3,1);

% Number of points where to calculate deflection
n = 10; % Number of points = n^3
x = 0.01:space_x/n:space_x;
y = 0.01:space_y/n:space_y;
z = 0.01:space_z/n:space_z;

D_VJM = zeros(1,n^3); % the array for deflection (VJM)
D_MSA = zeros(1,n^3); % the array for deflection (MSA)

X = zeros(1,n^3); % The array for point's x-coordinate
Y = zeros(1,n^3); % The array for point's y-coordinate
Z = zeros(1,n^3); % The array for point's z-coordinate
l = 1;
%% For many points plot
for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            % The function to calculate stiffness Matrix Kc (VJM)
            [Kc_VJM,Kc1_VJM,Kc2_VJM,Kc3_VJM] = Kc_def([x(i),y(j), z(k)], space_x, space_y, space_z, link_length, false, configuration, materials_params);
            % The function to calculate stiffness Matrix Kc (MSA)
            [Kc_MSA, Kcx_MSA, Kcy_MSA, Kcz_MSA] = MSA_part([x(i), y(j), z(k)], space_x, space_y, space_z, link_length,  false, configuration, materials_params);
            % Vector of end-effector deflection dt = [p(1) p(2) p(3) o(1) o(2) o(3)],
            % where p - position, o - orientation parts.
            dt_VJM = Kc_VJM\F;
            D_VJM(l) = sqrt(dt_VJM(1)^2 + dt_VJM(2)^2 + dt_VJM(3)^2);
            dt_MSA = Kc_MSA\F;
            D_MSA(l) = sqrt(dt_MSA(1)^2 + dt_MSA(2)^2 + dt_MSA(3)^2);
            
            X(l) = x(i);
            Y(l) = y(j);
            Z(l) = z(k);
            l = l+1;
        end
    end    
end

 %%           
            
% % Plot Deflection Map in 4D (VJM)
plot4D(X',Y',Z',D_VJM',[],F,1)

% Plot Deflection Map in 4D (MSA and VJM (comparison))
% plot4D(X',Y',Z',D_VJM',D_MSA',F,2)


saveas(gcf,'Fz_VJM.png')
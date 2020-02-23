function [Kc, Kcx, Kcy, Kcz] = MSA_part(tool_pose, space_x, space_y, space_z, link_length,  plot, configuration, materials_params)
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
% Output: Kc - 6*6 stiffness matrix for each single chain


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
    theta = zeros(1,13);
    
  % Let's get the stiffness matrixes of the robot  
    [k11, k12, k22] = k_cylinder(E, G, d, L, S, Iy, Iz);
    
    %% Let's solve Inverse Kinematics for our Serial Chains of the Robot
    T_base1 = eye(4);
    q1 = IK(T_base1, tool_pose, link_length, plot, configuration, 1, ['x','y']);
    [T0z,R10z_frame, R21z_frame, R32z, R43z] = FK(T_base1, link_length, theta, q1, tool_pose(3));
    
    T_base2 = transl(0,space_y,0)*troty(pi/2)*trotz(pi);
    q2 = IK(T_base2, tool_pose, link_length, plot, configuration, 2, ['x rel1','y rel1']);
    [T0x,R10x_frame, R21x_frame, R32x, R43x] = FK(T_base2, link_length, theta, q2, tool_pose(1));
     
    T_base3 = transl(0,0,space_z)*trotx(-pi/2);
    q3 = IK(T_base3, tool_pose, link_length, plot, configuration, 3, ['x rel2','y rel2']);
    [T0y,R10y_frame, R21y_frame, R32y, R43y] = FK(T_base3, link_length, theta, q3, tool_pose(2));
    
    Qz_45 = [R10z_frame, zeros(3,3);
               zeros(3,3), R10z_frame];
    
    Qx_45 = [R10x_frame, zeros(3,3);
               zeros(3,3), R10x_frame];
    Qy_45 = [R10y_frame, zeros(3,3);
               zeros(3,3), R10y_frame];
           
    Qz_67 = [R21z_frame, zeros(3,3);
               zeros(3,3), R21z_frame];
    
    Qx_67 = [R21x_frame, zeros(3,3);
               zeros(3,3), R21x_frame];
    Qy_67 = [R21y_frame, zeros(3,3);
               zeros(3,3), R21y_frame];
           
    % Stiffness matrixes for first link for 3 chains       
    K11_45z = Qz_45*k11*Qz_45';
    K12_45z = Qz_45*k12*Qz_45';
    K21_45z = Qz_45*k12'*Qz_45';
    K22_45z = Qz_45*k22*Qz_45';
    
    K11_45x = Qx_45*k11*Qx_45';
    K12_45x = Qx_45*k12*Qx_45';
    K21_45x = Qx_45*k12'*Qx_45';
    K22_45x = Qx_45*k22*Qx_45';
    
    K11_45y = Qy_45*k11*Qy_45';
    K12_45y = Qy_45*k12*Qy_45';
    K21_45y = Qy_45*k12'*Qy_45';
    K22_45y = Qy_45*k22*Qy_45';
    
    % Stiffness matrixes for second link for 3 chains
    K11_67z = Qz_67*k11*Qz_67';
    K12_67z = Qz_67*k12*Qz_67';
    K21_67z = Qz_67*k12'*Qz_67';
    K22_67z = Qz_67*k22*Qz_67';
    
    K11_67x = Qx_67*k11*Qx_67';
    K12_67x = Qx_67*k12*Qx_67';
    K21_67x = Qx_67*k12'*Qx_67';
    K22_67x = Qx_67*k22*Qx_67';
    
    K11_67y = Qy_67*k11*Qy_67';
    K12_67y = Qy_67*k12*Qy_67';
    K21_67y = Qy_67*k12'*Qy_67';
    K22_67y = Qy_67*k22*Qy_67';
    
    % Calculate lambdas
    [lambdaez, lambdaey, lambdaex] = lambda_e();
    [lambdap34z, lambdap56z, lambdap78z,lambdap34y, lambdap56y, ...
    lambdap78y,lambdap34x, lambdap56x, lambdap78x] = lambda_p();
    
    [lambdar12z, lambdar34z, lambdar56z, lambdar78z, lambdar12y, lambdar34y, lambdar56y, lambdar78y, ...
    lambdar12x, lambdar34x, lambdar56x, lambdar78x] = lambda_r();

    %% Aggregation for Chain Z
    Dz = zeros(6,6);
    Bz = [
        zeros(30,6);
        -eye(6,6);
        zeros(66,6);
        ];
    %102x102
    Az = [
        zeros(6, 6*9), eye(6,6),zeros(6, 7*6); %1
        
        zeros(6,18), -eye(6,6),zeros(6,6*8), K11_45z, K12_45z, zeros(6,18); %2
        
        zeros(6,24), -eye(6,6), zeros(6,6*7), K21_45z, K22_45z, zeros(6,18); %3
        
        zeros(6,30), -eye(6,6), zeros(6,48), K11_67z, K12_67z, zeros(6,6); %4 row
        
        zeros(6,36), -eye(6,6), zeros(6,42), K21_67z, K22_67z, zeros(6,6); %5 row
        
        zeros(6, 16*6),eye(6,6); %6
        
        zeros(6,42), eye(6,6), eye(6,6), zeros(6,48); %7
        
        zeros(6, 60), eye(6,6), -eye(6,6), zeros(6,30);%8
        
        zeros(6,6), eye(6,6), eye(6,6), zeros(6, 6*14); %9
        
        zeros(5, 6*9), lambdar12z, -lambdar12z, zeros(5, 6*6); %10
        
        eye(6,6), eye(6,6), zeros(6,15*6); %11
        
        lambdaez, zeros(1, 6*8), lambdaez*k0, -lambdaez*k0, zeros(1,6*6); %12
        
        zeros(5, 11*6), lambdar34z, -lambdar34z, zeros(5,6*4); %13
        
        zeros(5, 12), lambdar34z, lambdar34z, zeros(5, 13*6); %14
        
        zeros(1,12), lambdap34z, zeros(1,14*6); %15
        
        zeros(1,18), lambdap34z, zeros(1, 6*13); %16
        
        zeros(5, 13*6), lambdar56z, -lambdar56z, zeros(5,12); %17
        
        zeros(5, 6*4), lambdar56z, lambdar56z, zeros(5, 11*6); % 18
        
        zeros(1, 4*6), lambdap56z, zeros(1, 12*6); %19
        
        zeros(1, 5*6), lambdap56z, zeros(1, 11*6); %20
        
        zeros(5, 15*6), lambdar78z, -lambdar78z; %21
        
        zeros(5, 6*6), lambdar78z, lambdar78z, zeros(5, 9*6);  %22
        
        zeros(1, 6*6), lambdap78z, zeros(1, 10*6);
        
        zeros(1, 7*6), lambdap78z, zeros(1, 9*6)]; 
    
        
    Cz = [zeros(6, 8*6), -eye(6,6), zeros(6,8*6)];    

    Kcz =  Dz - Cz*(Az\Bz);
    
    
    
    %% Aggregation for Chain Y
    Dy = zeros(6,6);
    By = [
        zeros(30,6);
        -eye(6,6);
        zeros(66,6);
        ];
    %102x102
    Ay = [
        zeros(6, 6*9), eye(6,6),zeros(6, 7*6); %1
        
        zeros(6,18), -eye(6,6),zeros(6,6*8), K11_45y, K12_45y, zeros(6,18); %2
        
        zeros(6,24), -eye(6,6), zeros(6,6*7), K21_45y, K22_45y, zeros(6,18); %3
        
        zeros(6,30), -eye(6,6), zeros(6,48), K11_67y, K12_67y, zeros(6,6); %4 row
        
        zeros(6,36), -eye(6,6), zeros(6,42), K21_67y, K22_67y, zeros(6,6); %5 row
        
        zeros(6, 16*6),eye(6,6); %6
        
        zeros(6,42), eye(6,6), eye(6,6), zeros(6,48); %7
        
        zeros(6, 60), eye(6,6), -eye(6,6), zeros(6,30);%8
        
        zeros(6,6), eye(6,6), eye(6,6), zeros(6, 6*14); %9
        
        zeros(5, 6*9), lambdar12y, -lambdar12y, zeros(5, 6*6); %10
        
        eye(6,6), eye(6,6), zeros(6,15*6); %11
        
        lambdaey, zeros(1, 6*8), lambdaey*k0, -lambdaey*k0, zeros(1,6*6); %12
        
        zeros(5, 11*6), lambdar34y, -lambdar34y, zeros(5,6*4); %13
        
        zeros(5, 12), lambdar34y, lambdar34y, zeros(5, 13*6); %14
        
        zeros(1,12), lambdap34y, zeros(1,14*6); %15
        
        zeros(1,18), lambdap34y, zeros(1, 6*13); %16
        
        zeros(5, 13*6), lambdar56y, -lambdar56y, zeros(5,12); %17
        
        zeros(5, 6*4), lambdar56y, lambdar56y, zeros(5, 11*6); % 18
        
        zeros(1, 4*6), lambdap56y, zeros(1, 12*6); %19
        
        zeros(1, 5*6), lambdap56y, zeros(1, 11*6); %20
        
        zeros(5, 15*6), lambdar78y, -lambdar78y; %21
        
        zeros(5, 6*6), lambdar78y, lambdar78y, zeros(5, 9*6);  %22
        
        zeros(1, 6*6), lambdap78y, zeros(1, 10*6);
        
        zeros(1, 7*6), lambdap78y, zeros(1, 9*6)]; 
    
        
    Cy = [zeros(6, 8*6), -eye(6,6), zeros(6,8*6)];    
    Kcy =  Dy - Cy*(Ay\By);
    
    %% Aggregation for Chain X
    Dx = zeros(6,6);
    Bx = [
        zeros(30,6);
        -eye(6,6);
        zeros(66,6)];
    %102x102
    Ax = [
        zeros(6, 6*9), eye(6,6),zeros(6, 7*6); %1
        
        zeros(6,18), -eye(6,6),zeros(6,6*8), K11_45x, K12_45x, zeros(6,18); %2
        
        zeros(6,24), -eye(6,6), zeros(6,6*7), K21_45x, K22_45x, zeros(6,18); %3
        
        zeros(6,30), -eye(6,6), zeros(6,48), K11_67x, K12_67x, zeros(6,6); %4 row
        
        zeros(6,36), -eye(6,6), zeros(6,42), K21_67x, K22_67x, zeros(6,6); %5 row
        
        zeros(6, 16*6),eye(6,6); %6
        
        zeros(6,42), eye(6,6), eye(6,6), zeros(6,48); %7
        
        zeros(6, 60), eye(6,6), -eye(6,6), zeros(6,30);%8
        
        zeros(6,6), eye(6,6), eye(6,6), zeros(6, 6*14); %9
        
        zeros(5, 6*9), lambdar12x, -lambdar12x, zeros(5, 6*6); %10
        
        eye(6,6), eye(6,6), zeros(6,15*6); %11
        
        lambdaex, zeros(1, 6*8), lambdaex*k0, -lambdaex*k0, zeros(1,6*6); %12
        
        zeros(5, 11*6), lambdar34x, -lambdar34x, zeros(5,6*4); %13
        
        zeros(5, 12), lambdar34x, lambdar34x, zeros(5, 13*6); %14
        
        zeros(1,12), lambdap34x, zeros(1,14*6); %15
        
        zeros(1,18), lambdap34x, zeros(1, 6*13); %16
        
        zeros(5, 13*6), lambdar56x, -lambdar56x, zeros(5,12); %17
        
        zeros(5, 6*4), lambdar56x, lambdar56x, zeros(5, 11*6); % 18
        
        zeros(1, 4*6), lambdap56x, zeros(1, 12*6); %19
        
        zeros(1, 5*6), lambdap56x, zeros(1, 11*6); %20
        
        zeros(5, 15*6), lambdar78x, -lambdar78x; %21
        
        zeros(5, 6*6), lambdar78x, lambdar78x, zeros(5, 9*6);  %22
        
        zeros(1, 6*6), lambdap78x, zeros(1, 10*6);
        
        zeros(1, 7*6), lambdap78x, zeros(1, 9*6)];  
    
        
    Cx = [zeros(6, 8*6), -eye(6,6), zeros(6,8*6)];    
    Kcx =  Dx - Cx*(Ax\Bx);
    %% Let's calculate Stiffness matrixes for our Serial Chains
    
    %% Let's calculate the overall Stiffness Matrix
    Kc = Kcx+Kcy+Kcz;
end


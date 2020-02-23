function [T,R10_frame, R21_frame, R32, R43] = FK(Tbase, link_length, theta, q, move)
%This is the Forward Kinematics solution for the 2D link manipulator in the
%plane
% INPUT: link_length - length of the link (the same for all)
        %theta - theta angles
        %q - q angles from IK solution
        %move - displacement of the actuator
        
% OUTPUT: T - transformation matrix from the base to the end-effector(tool)
    T_rot = Tbase;
    T_rot(1:3,4) = 0;
    T10_frame = Tbase*transl(0,0,move)*transl(0,0,theta(1))*trotz(q(1)); %1
    T21 = transl(link_length,0,0)*transl(theta(2),0,0) ... 
    *transl(0, theta(3),0)*transl(0,0,theta(4))*trotx(theta(5))*troty(theta(6))*trotz(theta(7))*trotz(q(2));%2
    T32 = transl(link_length,0,0)*transl(theta(8),0,0)...
    *transl(0, theta(9),0)*transl(0,0,theta(10))*trotx(theta(11))*troty(theta(12))*trotz(theta(13)); %3

    T43 = trotz(q(3))*inv(T_rot); %4
    
    T = T10_frame*T21*T32*T43;
    
    T21_frame = T10_frame*T21;
    
    R10_frame = T10_frame(1:3,1:3);
    R21_frame = T21_frame(1:3,1:3);
    R32 = T32(1:3,1:3);
    R43 = T43(1:3,1:3);
    
end


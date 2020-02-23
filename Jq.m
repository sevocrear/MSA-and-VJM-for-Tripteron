function [Jq] = Jq(Tbase, link_length, space_x, space_y, space_z, theta, q, move)
    % The function finds the Jacobian by q angles
    %INPUT:
    %   link_length - the length of the links (the same for all)
    %   (space_x, space_y, space_z) - the shape of the operation space
    %   theta - angles thetas of actuating joints 
    %   q - angles qs of passive joints
    % tool_pose - the position of the tool
    %OUTPUT:
    %   Jq - Jacobian by q
    
    [T0,R10, R21, R32, R43] = FK(Tbase, link_length, theta, q, move);
    T0(1:3,4) = [0;0;0];
    T0 = T0';

    T_rot = Tbase;
    T_rot(1:3,4) = 0;
    
    Td = Tbase*transl(0,0,move)*transl(0,0,theta(1))*dtrotz(q(1))*transl(link_length,0,0)*transl(theta(2),0,0) ... 
    *transl(0, theta(3),0)*transl(0,0,theta(4))*trotx(theta(5))*troty(theta(6))*trotz(theta(7)) ... 
    *trotz(q(2))*transl(link_length,0,0)*transl(theta(8),0,0) ... 
    *transl(0, theta(9),0)*transl(0,0,theta(10))*trotx(theta(11))*troty(theta(12))*trotz(theta(13)) ...
    *trotz(q(3))*inv(T_rot)*T0;

    J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

    Td = Tbase*transl(0,0,move)*transl(0,0,theta(1))*trotz(q(1))*transl(link_length,0,0)*transl(theta(2),0,0) ... 
    *transl(0, theta(3),0)*transl(0,0,theta(4))*trotx(theta(5))*troty(theta(6))*trotz(theta(7)) ... 
    *dtrotz(q(2))*transl(link_length,0,0)*transl(theta(8),0,0) ... 
    *transl(0, theta(9),0)*transl(0,0,theta(10))*trotx(theta(11))*troty(theta(12))*trotz(theta(13)) ...
    *trotz(q(3))*inv(T_rot)*T0;
    
    J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

     Td = Tbase*transl(0,0,move)*transl(0,0,theta(1))*trotz(q(1))*transl(link_length,0,0)*transl(theta(2),0,0) ... 
    *transl(0, theta(3),0)*transl(0,0,theta(4))*trotx(theta(5))*troty(theta(6))*trotz(theta(7)) ... 
    *trotz(q(2))*transl(link_length,0,0)*transl(theta(8),0,0) ... 
    *transl(0, theta(9),0)*transl(0,0,theta(10))*trotx(theta(11))*troty(theta(12))*trotz(theta(13)) ...
    *dtrotz(q(3))*inv(T_rot)*T0;
    
    J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';


    Jq = [J1 J2 J3];
end

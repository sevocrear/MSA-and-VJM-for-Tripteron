function [Kc,Kc1,Kc2,Kc3] = Kc_def(tool_pose, space_x, space_y, space_z, link_length,  plot, configuration, materials_params)
    %% Let's solve Inverse Kinematics for our Serial Chains of the Robot
    T_base1 = eye(4);
    q1 = IK(T_base1, tool_pose, link_length, plot, configuration, 1, ['x','y']);
    
    T_base2 = transl(0,space_y,0)*troty(pi/2)*trotz(pi);
    q2 = IK(T_base2, tool_pose, link_length, plot, configuration, 2, ['x rel1','y rel1']);
    
    T_base3 = transl(0,0,space_z)*trotx(-pi/2);
    q3 = IK(T_base3, tool_pose, link_length, plot, configuration, 3, ['x rel2','y rel2']);
    
    theta = zeros(1,13);
    %% Let's calculate Jacobians of our Serial Chains
    % Serial chain #1
    Jq1 = Jq(T_base1, link_length, space_x, space_y, space_z, theta, q1, tool_pose(3));
    Jt1 = Jt(T_base1, link_length, space_x, space_y, space_z, theta, q1, tool_pose(3));
    
    % Serial chain #2
    Jq2 = Jq(T_base2, link_length, space_x, space_y, space_z, theta, q2, tool_pose(1));
    Jt2 = Jt(T_base2, link_length, space_x, space_y, space_z, theta, q2, tool_pose(1));
    
    % Serial chain #3
    Jq3 = Jq(T_base3, link_length, space_x, space_y, space_z, theta, q3, tool_pose(2));
    Jt3 = Jt(T_base3, link_length, space_x, space_y, space_z, theta, q3, tool_pose(2));
    %% Let's calculate Stiffness matrixes for our Serial Chains

    Kc1 = VJM_part(Jq1, Jt1, link_length, materials_params);
    Kc2 = VJM_part(Jq2, Jt2, link_length, materials_params);
    Kc3 = VJM_part(Jq3, Jt3, link_length, materials_params);

    %% Let's calculate the overall Stiffness Matrix
    Kc = Kc1 + Kc2 + Kc3;
end    



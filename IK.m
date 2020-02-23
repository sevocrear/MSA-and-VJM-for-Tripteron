function [q] = IK(T_base, tool_pose, link_length, plot, configuration, fig_num, axes)
%This is the Inverse Kinematics solution for the 2D link manipulator in the
%plane
% INPUT: tool_pose (global) - desired point of the tool.
% T_base - coordinates of the robot's relative base
% plot - To draw or not (false, true)
% configuration (1,2) - enter the type of configuration (1- elbow-down, 2 -
% elbow-up)
% fig_num - creates the fig(fig_num)
% OUTPUT: desired angles of the joints: q = [q1 q2 q3]
    
    R_base = T_base(1:3,1:3);
    p_base = T_base(1:3,4);
    
    local_pose = R_base'*(tool_pose' - p_base);
% First configuration
    x = local_pose(1);
    y = local_pose(2);
    z = local_pose(3);
    
    a = link_length;
    b = link_length;
    c = sqrt((x)^2 + (y)^2);
    cos2 = (c^2 - a^2 - b^2)/(2*a*b);
    
    sin2 = sqrt(1-cos2^2);
    
    q2 = atan2(sin2, cos2);
    
    cosk = (c^2+a^2-b^2)/(2*a*c);
    sink = sqrt(1-cosk^2);
    
    k = atan2(sink, cosk);
    phi = atan2(y,x);
    
    q1 = phi - k;
    
    % Assume that at the end we have not platform but point.
    q3 = -q1 - q2;
    % Second configuration
    q11 = k + phi;
    q22 = -atan2(sin2, cos2);
    q33 = -q11 - q22;
    
    if configuration == 1
        q = [q1, q2, q3];
    elseif configuration == 2
        q = [q11, q22, q33];
    end
    if plot
        figure(fig_num);
        subplot(2,2,1)
        grid on
        line([0 0 + a*cos(q1)],[0 0 + a*sin(q1)],'LineWidth',8,'Color','r',... 
            'Marker','o','MarkerSize',10);
        
        line([0 + a*cos(q1) 0 + a*cos(q1)+b*cos(q1+q2)], [0+a*sin(q1) 0+a*sin(q1) + b*sin(q1+q2)],'LineWidth',8,'Color', ...
            'r','Marker','o','MarkerSize',10);
        
        line([x x+0.0001], [y y+0.0001],'LineWidth',8,'Color', ...
            'b','Marker','o','MarkerSize',10);
        xlabel(axes(1));
        ylabel(axes(2));
        legend('1st configuration');
    end
    
    if plot
        subplot(2,2,2)
        grid on
        line([0 0 + a*cos(q11)],[0 0+ a*sin(q11)],'LineWidth',8,'Color','r',... 
            'Marker','o','MarkerSize',10);
        
        line([0+a*cos(q11) 0+a*cos(q11)+b*cos(q11+q22)], [0+a*sin(q11) 0+a*sin(q11) + b*sin(q11+q22)],'LineWidth',8,'Color', ...
            'r','Marker','o','MarkerSize',10);
        
        line([x x+0.0001], [y y+0.0001],'LineWidth',8,'Color', ...
            'b','Marker','o','MarkerSize',10);
        xlabel(axes(1));
        ylabel(axes(2));
        legend('2nd configuration');
    end

end


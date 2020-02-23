function plot4D(x,y,z,deflection1,deflection2, F,subplots)
% This is the function that allows to plot smth in 4D
cla
if subplots == 1
    scatter3(x,y,z, length(x),deflection1,'filled')    % draw the scatter plot
    text = ['F=','[',num2str(F(1)),' ',num2str(F(2)),' ',num2str(F(3)),' ',num2str(F(4)),' ',...
        num2str(F(5)),' ',num2str(F(6)),']'];
    text = join(text);
    title(text,'FontSize',14);
    view(-31,14)
    xlabel('x-coordinate [m]')
    ylabel('y-coordinate [m]')
    zlabel('z-coordinate [m]')
    cb = colorbar;                                     % create and label the colorbar
    cb.Label.String = 'Deflection [m]';
elseif subplots == 2
    subplot(1,2,1);  
    
    scatter3(x,y,z, length(x),deflection1,'filled')    % draw the scatter plot
    text = ['F=','[',num2str(F(1)),' ',num2str(F(2)),' ',num2str(F(3)),' ',num2str(F(4)),' ',...
        num2str(F(5)),' ',num2str(F(6)),']'];
    text = join(text);
    title(text,'FontSize',14);
    view(-31,14)
    xlabel('x-coordinate [m]')
    ylabel('y-coordinate [m]')
    zlabel('z-coordinate [m]')
    cb = colorbar;                                     % create and label the colorbar
    cb.Label.String = 'Deflection [m]';
    
    subplot(1,2,2);  
    scatter3(x,y,z, length(x),deflection2,'filled')    % draw the scatter plot
    text = ['F=','[',num2str(F(1)),' ',num2str(F(2)),' ',num2str(F(3)),' ',num2str(F(4)),' ',...
        num2str(F(5)),' ',num2str(F(6)),']'];
    text = join(text);
    title(text,'FontSize',14);
    view(-31,14)
    xlabel('x-coordinate [m]')
    ylabel('y-coordinate [m]')
    zlabel('z-coordinate [m]')
    cb = colorbar;                                     % create and label the colorbar
    cb.Label.String = 'Deflection [m]';
end    

end
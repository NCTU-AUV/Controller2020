clc
clear
close force all
%%
A = serialport('COM4', 115200);
%% 
Quaternion = readline(A);
Quaternion = str2num(Quaternion); %#ok<*ST2NM>
RM = quat2rotm(Quaternion);
X = plot3([0 RM(1, 1)], [0 RM(2, 1)], [0 RM(3, 1)], 'r-');
axis equal
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
hold on
Y = plot3([0 RM(1, 2)], [0 RM(2, 2)], [0 RM(3, 2)], 'g-');
Z = plot3([0 RM(1, 3)], [0 RM(2, 3)], [0 RM(3, 3)], 'b-');
pause(0.1);
%%{
while 1
    if(A.NumBytesAvailable>20)
        read(A, A.NumBytesAvailable-20, 'char');
        readline(A);
        Quaternion = readline(A);
        Quaternion = str2num(Quaternion); %#ok<*ST2NM>
        RM = quat2rotm(Quaternion);
        X.XData = [0 RM(1, 1)]; X.YData = [0 RM(2, 1)]; X.ZData = [0 RM(3, 1)];
        Y.XData = [0 RM(1, 2)]; Y.YData = [0 RM(2, 2)]; Y.ZData = [0 RM(3, 2)];
        Z.XData = [0 RM(1, 3)]; Z.YData = [0 RM(2, 3)]; Z.ZData = [0 RM(3, 3)];
        pause(0.01);
    end
end
%}
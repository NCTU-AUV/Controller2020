clc
clear
close force all
%%
A = serialport('COM4', 115200); %對應到arduino的COM和Baud rate
%% 
Quaternion = readline(A); %取值
Quaternion = str2num(Quaternion); %#ok<*ST2NM> %string to num
RM = quat2rotm(Quaternion); %quaternion to rotation matrix

%畫圖
axis equal
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
hold on

%取值
X = plot3([0 RM(1, 1)], [0 RM(2, 1)], [0 RM(3, 1)], 'r-');
Y = plot3([0 RM(1, 2)], [0 RM(2, 2)], [0 RM(3, 2)], 'g-');
Z = plot3([0 RM(1, 3)], [0 RM(2, 3)], [0 RM(3, 3)], 'b-');

pause(0.1);
%%{
while 1
    if(A.NumBytesAvailable>20)
        read(A, A.NumBytesAvailable-20, 'char'); %將要取值的那行之前的所有資料
        %先read，不然會一直取到第一行
        readline(A); %將要取值的前一行取乾淨
        
        Quaternion = readline(A); %取值
        Quaternion = str2num(Quaternion); %#ok<*ST2NM> %string to num
        RM = quat2rotm(Quaternion); %Quaternion to rotation matrix
        
        %更新X軸座標值
        X.XData = [0 RM(1, 1)];
        X.YData = [0 RM(2, 1)]; 
        X.ZData = [0 RM(3, 1)];
        
        %更新Y軸座標值
        Y.XData = [0 RM(1, 2)]; 
        Y.YData = [0 RM(2, 2)]; 
        Y.ZData = [0 RM(3, 2)];
        
        %更新Z軸座標值
        Z.XData = [0 RM(1, 3)]; 
        Z.YData = [0 RM(2, 3)]; 
        Z.ZData = [0 RM(3, 3)];
        
        pause(0.01);
    end
end
%
}
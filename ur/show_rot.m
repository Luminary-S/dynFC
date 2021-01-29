%*****************************************
%******直角坐标系的转换及动画演示************
%*****************************************
% clear all;
% Iz_a=[0 0 1]';%原坐标系下的Z轴
% Iy_a=[0 1 0]';%原坐标系下的Y轴
% Ix_a=[1 0 0]';%原坐标系下的X轴
% %绘制原三维坐标系O-XYZ的XYZ轴
% for t=-10:0.1:10
%     plot3(Iz_a(1),Iz_a(2),t*Iz_a(3));%绘制Z轴
%     plot3(Iy_a(1),t*Iy_a(2),Iy_a(3));%绘制Y轴
%     plot3(t*Ix_a(1),Ix_a(2),Ix_a(3));%绘制X轴
%     hold on;
% end
% rx = 60
% ry = 60
% rz = 60
% r_x=deg2rad(60);%原坐标系绕X轴旋转60度
% r_y=deg2rad(60);%原坐标系绕Y轴旋转60度
% r_z=deg2rad(60);%原坐标系绕Z轴旋转60度
%坐标系绕X轴旋转的转换矩阵Tx
% Tx=[1        0          0;
%     0  cos(r_x)  sin(r_x);
%     0 -sin(r_x)  cos(r_x)];
% %坐标系绕Y轴旋转的转换矩阵Ty
% Ty=[cos(r_y) 0 -sin(r_y);
%     0        1         0;
%     0 sin(r_y)  cos(r_y)];
% %坐标系绕Z轴旋转的转换矩阵Tz
% Tz=[ cos(r_z) sin(r_z) 0;
%     -sin(r_z) cos(r_z) 0;
%     0         0        1]; 
% Iz_b=Tx*Iz_a;%绕X轴旋转后新坐标系下的Z轴
% Iy_b=Tx*Iy_a;%绕X轴旋转后新坐标系下的Y轴
% Ix_b=Tx*Ix_a;%绕X轴旋转后新坐标系下的X轴
% for t=-10:0.1:10
%     plot3(t*Iz_b(1),t*Iz_b(2),t*Iz_b(3),'r');%绘制Z轴
%     plot3(t*Iy_b(1),t*Iy_b(2),t*Iy_b(3),'r');%绘制Y轴
%     plot3(t*Ix_b(1),t*Ix_b(2),t*Ix_b(3),'r');%绘制X轴
%     hold on;
% end
% %********************************************
% %******动画演示坐标系绕X轴旋转过程**************
% %********************************************
% clear all;
% Iz_a=[0 0 1]';%原坐标系下的Z轴
% Iy_a=[0 1 0]';%原坐标系下的Y轴
% Ix_a=[1 0 0]';%原坐标系下的X轴
% for rx=0:5:60
%     Rx=deg2rad(rx);
%     %坐标系绕X轴旋转的转换矩阵Tx
%     Tx=[1        0        0;
%         0  cos(Rx)  sin(Rx);
%         0 -sin(Rx)  cos(Rx)];
%     Iz_b=Tx*Iz_a;%绕X轴旋转后新坐标系下的Z轴
%     Iy_b=Tx*Iy_a;%绕X轴旋转后新坐标系下的Y轴
%     Ix_b=Tx*Ix_a;%绕X轴旋转后新坐标系下的X轴
%     for t=-10:0.1:10
%         plot3(t*Iz_b(1),t*Iz_b(2),t*Iz_b(3),'r');%绘制Z轴
%         plot3(t*Iy_b(1),t*Iy_b(2),t*Iy_b(3),'b');%绘制Y轴
%         plot3(t*Ix_b(1),t*Ix_b(2),t*Ix_b(3),'g');%绘制X轴
%         hold on;
%     end
%     pause(0.1);
% end
% %************************************************
% %******动画演示坐标系绕Z-Y-X轴旋转过程**************
% %************************************************
% clear all;
% Iz_a=[0 0 1]';%原坐标系下的Z轴
% Iy_a=[0 1 0]';%原坐标系下的Y轴
% Ix_a=[1 0 0]';%原坐标系下的X轴
% for rx=0:5:60
%     Rx=deg2rad(rx);
%     %坐标系绕X轴旋转的转换矩阵Tx
%     Tx=[1        0        0;
%         0  cos(Rx)  sin(Rx);
%         0 -sin(Rx)  cos(Rx)];
%     %坐标系绕Y轴旋转的转换矩阵Ty
%     Ty=[cos(Rx) 0 -sin(Rx);
%         0        1         0;
%         0 sin(Rx)  cos(Rx)];
%     %坐标系绕Z轴旋转的转换矩阵Tz
%     Tz=[ cos(Rx) sin(Rx) 0;
%         -sin(Rx) cos(Rx) 0;
%         0         0        1]; 
%     Iz_b=Tx*Ty*Tz*Iz_a;%绕ZYX轴旋转后新坐标系下的Z轴
%     Iy_b=Tx*Ty*Tz*Iy_a;%绕ZYX轴旋转后新坐标系下的Y轴
%     Ix_b=Tx*Ty*Tz*Ix_a;%绕ZYX轴旋转后新坐标系下的X轴
%     for t=-10:0.1:10
%         plot3(t*Iz_b(1),t*Iz_b(2),t*Iz_b(3),'r');%绘制Z轴
%         plot3(t*Iy_b(1),t*Iy_b(2),t*Iy_b(3),'b');%绘制Y轴
%         plot3(t*Ix_b(1),t*Ix_b(2),t*Ix_b(3),'g');%绘制X轴
%         hold on;
%     end
%     hold off;
%     pause(0.1);
% end
%************************************************
%******动画演示坐标系分别绕Z-Y-X轴旋转过程**********
%************************************************
% clear all;
Iz_a=[0 0 1]';%原坐标系下的Z轴
Iy_a=[0 1 0]';%原坐标系下的Y轴
Ix_a=[1 0 0]';%原坐标系下的X轴
%坐标系先绕Z轴旋转
for rx=0:5:60
    Rx=deg2rad(rx);
    %坐标系绕Z轴旋转的转换矩阵Tz
    Tz=[ cos(Rx) sin(Rx) 0;
        -sin(Rx) cos(Rx) 0;
        0         0        1]; 
    Iz_b=Tz*Iz_a;%绕Z轴旋转后新坐标系下的Z轴
    Iy_b=Tz*Iy_a;%绕Z轴旋转后新坐标系下的Y轴
    Ix_b=Tz*Ix_a;%绕Z轴旋转后新坐标系下的X轴
    for t=-10:0.1:10
        plot3(t*Iz_b(1),t*Iz_b(2),t*Iz_b(3),'r');%绘制Z轴
        plot3(t*Iy_b(1),t*Iy_b(2),t*Iy_b(3),'b');%绘制Y轴
        plot3(t*Ix_b(1),t*Ix_b(2),t*Ix_b(3),'g');%绘制X轴
        hold on;
    end
    pause(0.1);
    hold off;
end
%坐标系再绕Y轴旋转
for rx=0:5:60
    Rx=deg2rad(rx);
    %坐标系绕Y轴旋转的转换矩阵Ty
    Ty=[cos(Rx) 0 -sin(Rx);
        0        1         0;
        0 sin(Rx)  cos(Rx)];
    Iz_c=Ty*Iz_b;%绕Z轴旋转后新坐标系下的Z轴
    Iy_c=Ty*Iy_b;%绕Z轴旋转后新坐标系下的Y轴
    Ix_c=Ty*Ix_b;%绕Z轴旋转后新坐标系下的X轴
    for t=-10:0.1:10
        plot3(t*Iz_c(1),t*Iz_c(2),t*Iz_c(3),'r');%绘制Z轴
        plot3(t*Iy_c(1),t*Iy_c(2),t*Iy_c(3),'b');%绘制Y轴
        plot3(t*Ix_c(1),t*Ix_c(2),t*Ix_c(3),'g');%绘制X轴
        hold on;
    end
    pause(0.1);
    hold off;
end
%最后坐标系绕X轴旋转
for rx=0:5:60
    Rx=deg2rad(rx);
    %坐标系绕X轴旋转的转换矩阵Tx
    Tx=[1        0        0;
        0  cos(Rx)  sin(Rx);
        0 -sin(Rx)  cos(Rx)];
    Iz_d=Tx*Iz_c;%绕Z轴旋转后新坐标系下的Z轴
    Iy_d=Tx*Iy_c;%绕Z轴旋转后新坐标系下的Y轴
    Ix_d=Tx*Ix_c;%绕Z轴旋转后新坐标系下的X轴
    for t=-10:0.1:10
        plot3(t*Iz_d(1),t*Iz_d(2),t*Iz_d(3),'r');%绘制Z轴
        plot3(t*Iy_d(1),t*Iy_d(2),t*Iy_d(3),'b');%绘制Y轴
        plot3(t*Ix_d(1),t*Ix_d(2),t*Ix_d(3),'g');%绘制X轴
        hold on;
    end
    pause(0.1);
    hold off;
end
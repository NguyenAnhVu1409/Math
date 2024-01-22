clc; clear all; close all;

t=0;
ts = 0.01;
LastCycle=0;

x_a   = 0; %[mm]
y_a   = 0;
z_a   = 0;

x_b   = 100; %[mm]
y_b   = 100;
z_b   = 100;

AB_MD   = sqrt(((x_b-x_a)^2)+((y_b-y_a)^2)+((z_b-z_a)^2));

%% Quy hoach quy dao LSPB
x_0           = 0;            %% trang thai cua diem dau
x_dot_0    = 0;
x_2dot_0  = 0;
x_F           = AB_MD;        %% trang thai cua diem cuoi
x_dot_F    = 0;
x_2dot_F  = 0;

% Magic = 0.1;
% tf_o = AB_MD/v_qd;
% tf    = (1+Magic)*tf_o;
% tb   = Magic*tf_o;
% v_qd  = (x_F-x_0)/(tf-tb);
% acc    = v_qd / tb;

v_qd  = 500;    %% [mm]/[s]
tb = 0.05;
acc_max = 10000;     %[mm/s]

tb_min = v_qd / acc_max;
if(tb < tb_min) 
    tb = tb_min;    
end
acc = v_qd / tb;

t_giutoc = (AB_MD - acc*tb*tb) / v_qd;
tf = t_giutoc + 2*tb;

% %% Quy hoach quy dao bac 5
% a0=x_0;
% a1=x_dot_0;
% a2=x_2dot_0/2;
% a3=(20*x_F - 20*x_0-(8*x_dot_F + 12*x_dot_0)*tf -(3*x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^3);
% a4=(30*x_0 - 30*x_F + (14*x_dot_F + 16*x_dot_0)*tf +(3*x_2dot_0 - 2*x_2dot_F)*tf^2)/(2*tf^4);
% a5=(12*x_F - 12*x_0 - (6*x_dot_F + 6*x_dot_0)*tf -(x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^5);

while(1)

%     S= a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
%     v = a1+ 2*a2*t+ 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
%     a = 2*a2+ 6*a3*t+ 12*a4*t^2 + 20*a5*t^3;
    
    if((t>=0)&&(t<=tb))
        S = x_0 + 0.5*acc*t*t;
        v = acc*t;
        a = acc;
    end
    if((t>=tb)&&(t<=(tf-tb)))
        t2 = t - tb;
        S=(x_0 + 0.5*acc*tb*tb) + v_qd*t2;
        v = v_qd;
        a = 0;
    end
    if((t>=(tf-tb))&&(t<=tf))
        t3 = t - tf + tb;
        a = - acc;
        v = v_qd - acc*t3;
%          S = x_F + acc*tf*t - 0.5*acc*tf*tf - 0.5*acc*t*t;
%          S= (x_0 + 0.5*acc*tb*tb) + v_qd*(tf-2*tb) + 0.5*acc*tb*tb - 0.5*acc*(tb-t3)*(tb-t3);
          S= x_F - 0.5*acc*(tb-t3)*(tb-t3);
    end
    
    P_x_TCP   = x_a + (x_b - x_a)*S/AB_MD;
    P_y_TCP   = y_a + (y_b - y_a)*S/AB_MD;
    P_z_TCP   = z_a + (z_b - z_a)*S/AB_MD;

    subplot(1,4,1);
    plot3(P_x_TCP,P_y_TCP,P_z_TCP,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([-100 150]);
    ylim([-100 150]);
    zlim([-100 150]);
    grid on;
    hold on;
    
    subplot(1,4,2);
    plot(t,S,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([0 tf]);
    ylim([x_0 x_F]);
    grid on;
    hold on;
    
    subplot(1,4,3);
    plot(t,v,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([0 tf]);
    ylim([-3*v_qd 3*v_qd]);
    grid on;
    hold on;
    
    subplot(1,4,4);
    plot(t,a,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([0 tf]);
    ylim([-acc acc]);
    grid on;
    hold on;
    
    if(LastCycle==1)
        break;
    end
    
    t=t+ts;
    if(t>tf)
         t=tf;
         LastCycle = 1;
    end
    pause(0.01);
end

sprintf("Simulation Done")
clc; clear all; close all;

t=0;
ts = 0.01;
LastCycle=0;

%% Start point
x_start   = 20; %[mm]
y_start   = 20;
z_start   = 20;
S0_0 = [x_start;     y_start;    z_start];

%% End point
x_end   = 55; %[mm]
y_end   = 55;
z_end   = 55;
E0_0 = [x_end;     y_end;    z_end];

%% Start point
x_a   = 25; %[mm]
y_a   = 25;
z_a   = 25;
A_0 = [x_a;     y_a;    z_a];
theta_z_0  = 0*pi/180;
theta_y_0  = 0*pi/180;
theta_x_0  = 90*pi/180;
%% End point
x_b   = 90; %[mm]
y_b   = 90;
z_b   = 90;
B_0 = [x_b;     y_b;    z_b];
theta_z_F  = 0*pi/180;
theta_y_F  = 0*pi/180;
theta_x_F  = -90*pi/180;

AB_MD   = sqrt(((x_end-x_start)^2)+((y_end-y_start)^2)+((z_end-z_start)^2));

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

v_qd  = 100;    %% [mm]/[s]
tb = 0.05;
acc_max = 10000;     %[mm/s]

tb_min = v_qd / acc_max;
if(tb < tb_min) 
    tb = tb_min;    
end
acc = v_qd / tb;

t_giutoc = (AB_MD - acc*tb*tb) / v_qd;
tf = t_giutoc + 2*tb;

%% Quy hoach quy dao bac 5
% a0=x_0;
% a1=x_dot_0;
% a2=x_2dot_0/2;
% a3=(20*x_F - 20*x_0-(8*x_dot_F + 12*x_dot_0)*tf -(3*x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^3);
% a4=(30*x_0 - 30*x_F + (14*x_dot_F + 16*x_dot_0)*tf +(3*x_2dot_0 - 2*x_2dot_F)*tf^2)/(2*tf^4);
% a5=(12*x_F - 12*x_0 - (6*x_dot_F + 6*x_dot_0)*tf -(x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^5);

%% He toa do G
OG_G = [0 ; 0 ; 0];
IG_G = [10 ; 0 ; 0];
JG_G = [0 ; 10 ; 0];
KG_G = [0 ; 0 ; 10];

%% He toa do trung gian - Input
HG0 = [1 0 0 10;
            0 1 0 10;
            0 0 1 10;
            0 0 0 1];
H0G = inv(HG0);
O0_G   = HG0 *  [OG_G;  1];
I0_G     = HG0 *  [IG_G;  1];
J0_G    = HG0 *  [JG_G;  1];
K0_G    = HG0 *  [KG_G;  1];

S0_G = HG0 * [S0_0; 1];
E0_G = HG0 * [E0_0; 1];

subplot(1,1,1);
%% Ve dua tren goc toa do toan cuc
plot3([0 10], [0 0], [0 0],'r','LineWidth',2);
grid on;
hold on;
axis([-10 100 -10 100 -10 100]);

plot3([0 0], [0 10], [0 0],'g','LineWidth',2);
plot3([0 0], [0 0], [0 10],'b','LineWidth',2);

%% Ve goc toa do 0
plot3([O0_G(1,1) I0_G(1,1)], [O0_G(2,1) I0_G(2,1)], [O0_G(3,1) I0_G(3,1)],'r','LineWidth',2);
grid on;
hold on;
axis([-10 100 -10 100 -10 100]);

plot3([O0_G(1,1) J0_G(1,1)], [O0_G(2,1) J0_G(2,1)], [O0_G(3,1) J0_G(3,1)],'g','LineWidth',2);
plot3([O0_G(1,1) K0_G(1,1)], [O0_G(2,1) K0_G(2,1)], [O0_G(3,1) K0_G(3,1)],'b','LineWidth',2);

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
    
    phantram = S/AB_MD;
    
    P_x_TCP_G   = S0_G(1,1) + (E0_G(1,1) - S0_G(1,1))*phantram;
    P_y_TCP_G   = S0_G(2,1) + (E0_G(2,1) - S0_G(2,1))*phantram;
    P_z_TCP_G   = S0_G(3,1) + (E0_G(3,1) - S0_G(3,1))*phantram;
    
    P_x_TCP_0   = S0_0(1,1) + (E0_0(1,1) - S0_0(1,1))*phantram;
    P_y_TCP_0   = S0_0(2,1) + (E0_0(2,1) - S0_0(2,1))*phantram;
    P_z_TCP_0   = S0_0(3,1) + (E0_0(3,1) - S0_0(3,1))*phantram;
    
    theta_z_TCP = theta_z_0 + (theta_z_F - theta_z_0)*phantram;
    theta_y_TCP = theta_y_0 + (theta_y_F - theta_y_0)*phantram;
    theta_x_TCP = theta_x_0 + (theta_x_F - theta_x_0)*phantram;

%     theta_z_TCP = ;
%     theta_y_TCP = ;
%     theta_x_TCP = ;

    subplot(1,1,1);
    plot3(P_x_TCP_G,P_y_TCP_G,P_z_TCP_G,'-o','LineWidth',1,'MarkerEdgeColor','b')
    axis([0   2*E0_G(1,1)     0     2*E0_G(2,1)    0     2*E0_G(3,1)]);
    grid on;
    hold on;

    H01 = [ cos(theta_z_TCP)*cos(theta_y_TCP)      cos(theta_z_TCP)*sin(theta_y_TCP)*sin(theta_x_TCP)-sin(theta_z_TCP)*cos(theta_x_TCP)        cos(theta_z_TCP)*sin(theta_y_TCP)*cos(theta_x_TCP)+sin(theta_z_TCP)*sin(theta_x_TCP)      P_x_TCP_0;
                sin(theta_z_TCP)*cos(theta_y_TCP)       sin(theta_z_TCP)*sin(theta_y_TCP)*sin(theta_x_TCP)+cos(theta_z_TCP)*cos(theta_x_TCP)       sin(theta_z_TCP)*sin(theta_y_TCP)*cos(theta_x_TCP)-cos(theta_z_TCP)*sin(theta_x_TCP)       P_y_TCP_0;
                -sin(theta_y_TCP)                                 cos(theta_y_TCP)*sin(theta_x_TCP)                                                                                     cos(theta_y_TCP)*cos(theta_x_TCP)                                                                                           P_z_TCP_0;
                0                                                          0                                                                                                                                      0                                                                                                                                                 1];
    
    HG1 = HG0*H01;
    
    I1_G = HG1*[IG_G; 1];
    J1_G = HG1*[JG_G; 1];
    K1_G = HG1*[KG_G; 1];
     
    plot3([HG1(1,4) I1_G(1,1)], [HG1(2,4) I1_G(2,1)], [HG1(3,4) I1_G(3,1)],'r','LineWidth',2);
    grid on;
    hold on;
    plot3([HG1(1,4) J1_G(1,1)],  [HG1(2,4) J1_G(2,1)],     [HG1(3,4) J1_G(3,1)],'g','LineWidth',2);
    plot3([HG1(1,4) K1_G(1,1)],  [HG1(2,4) K1_G(2,1)],    [HG1(3,4) K1_G(3,1)],'b','LineWidth',2);

%     subplot(1,4,2);
%     plot(t,S,'-o','LineWidth',1,'MarkerEdgeColor','b')
%     axis([-0.1 tf*1.1 x_0 x_F]);
%     grid on;
%     hold on;
%     
%     subplot(1,4,3);
%     plot(t,v,'-o','LineWidth',1,'MarkerEdgeColor','b')
%     axis([-0.1 tf*1.1 -0.1*v_qd 1.5*v_qd]);
%     grid on;
%     hold on;
%     
%     subplot(1,4,4);
%     plot(t,a,'-o','LineWidth',1,'MarkerEdgeColor','b')
%     axis([-0.1 tf*1.1 -1.1*acc 1.1*acc]);
%     grid on;
%     hold on;
    
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
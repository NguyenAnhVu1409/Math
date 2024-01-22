close all;clear all; clc;
%% Declare all global variable
% syms Px Py Pz
% syms i ii T l0 l1 l2 l3 l4 l5 l6 d
% syms z R x  D F G E  y x_o y_o z_o t t1 tf
% syms anpha anpha1 anpha2 anpha3 anpha4 min_anpha
% syms x_a y_a z_a x_b y_b z_b x_c y_c z_c
% syms PhanTram error_ahihi_1 error_ahihi_2
% syms theta1_dot theta2_dot theta3_dot d_dot V_tt_pre theta1_0 theta2_0 d_0
% syms v_qd v_o v_f q_o q_f t_tt t_c t_f t_gt
% syms A B C V Q O D X Y
% syms x_a y_a z_a x_b y_b z_b x_c y_c z_c x_v y_v z_v x_q y_q z_q x_o y_o z_o x_d y_d z_d R R_P R_2
% syms tb_1 tb_2 tb_3 tf_1 tf_2 tf_3 At Vt_pre
%% Declare all parameters of Robot
    l0=185;    
    l1=87;    %272-185
    l2=200;
    l3=200;
    l4=116;   %%122-6
%% Declare all initial value of any variable
theta1_pre=0;
theta2_pre=0;
d_pre=0;
theta1_dot=0*pi/180;
theta2_dot=0*pi/180;
theta3_dot=0*pi/180;
d_dot=0;
Vt_pre=0;
phantram=0;
phantram_pre=0;
theta1_dot_estimation       =  0;
theta2_dot_estimation      =   0;
d_dot_estimation              =   0;
theta3_dot_estimation     =   0;
Px_pre=0;
Py_pre=0;
Pz_pre=0;
WP=0;
%% Delare the parameters of All Motion
Error=0;             %% cho phep chuong trinh chay
t=0;                  %% thoi gian ban dau
WP=0;
%% Nhap toa do diem Home cho Robot ( don vi la milimet)
[x_a, y_a, z_a]=Point(400,0,156);
A=[x_a;y_a;z_a];
[theta1,theta2,d]=DHN(x_a,y_a,z_a,theta1_pre);
theta1_pre=theta1;
theta2_pre=theta2;
d_pre=d;
setappdata(0, 'theta1_old', theta1);
setappdata(0, 'theta2_old', theta2);
setappdata(0, 'd_old', d);
setappdata(0, 'Error_old', Error); 
setappdata(0, 'WP', WP);
%% Cai Dat ban dau chuan bi chay chuong trinh
if(Error==0)
    sprintf('The input position is correct - the program is running')
                MoveJ(400,0,150,150);
                wait('s',5);
                %% doi de  mo to hinh`
                MoveJ(0,400,16,150);  
                MoveJ(0,-400,156,150);
                wait('s',0.5);
                MoveL(400,0,16,150); 
                MoveL(0,400,156,150);
                wait('s',0.5);
                MoveJ(200,200,106,150);
                CircleMove(200,-200,106,350,0,106,150);
                CircleMove(200,200,106,350,0,106,150);
                wait('s',0.5);
                MoveJ(400,0,156,150); 
                MoveC(200,0,156,300,100,156,150);
                MoveC(200,0,156,300,-100,156,150);
                wait('s',0.5);
                MoveJ(0,-400,106,150); 
                MoveP(0,400,106,400,0,106,150,150);
                MoveP(0,-400,106,400,0,106,150,150);
                wait('s',0.5);
                MoveJ(400,0,156,150); 
             sprintf('The simulation completed')
end


%%
close all;
clear all; clc;
syms Px Py Pz
syms i ii T l0 l1 l2 l3 l4 l5 l6 d
syms z R x  D F G E  y x_o y_o z_o t t1 tf
syms anpha anpha1 anpha2 anpha3 anpha4 min_anpha
syms x_a y_a z_a x_b y_b z_b x_c y_c z_c
syms PhanTram error_ahihi_1 error_ahihi_2
syms theta1_dot theta2_dot theta3_dot d_dot V_tt_pre theta1_0 theta2_0 d_0
syms v_qd v_o v_f q_o q_f t_tt t_c t_f t_gt
syms A B C V Q O D X Y
syms x_a y_a z_a x_b y_b z_b x_c y_c z_c x_v y_v z_v x_q y_q z_q x_o y_o z_o x_d y_d z_d R R_P R_2
syms tb_1 tb_2 tb_3 tf_1 tf_2 tf_3 At Vt_pre
%%
t=0;                %% thoi gian ban dau
ts=0.05;            %% thoi gian lay mau
v_qd=300;           %% van toc cua chuyen dong mm/s
QD=2;               %% O: Move L , 1: Circle Move , 2: Move J, 3: Move P, 4: MoveC
R_P=250;            %% R: Ban kinh move P
%% thong so DH
l0=185;
l1=73;
l2=200;
l3=200;
l4=122;
%% Cac bien ban dau
theta1_pre=0;
theta2_pre=0;
theta1_dot=0*pi/180;
theta2_dot=0*pi/180;
theta3_dot=0*pi/180;
d_dot=0;
d_pre=0;
V_tt_pre=0;
phantram=0;
Vt_pre=0;
theta1_dot_estimation      =   0;
theta2_dot_estimation      =   0;
d_dot_estimation           =   0;
theta3_dot_estimation      =   0;
Px_pre=0;
Py_pre=0;
Pz_pre=0;
%% Nhap toa do ban dau A Robot ( don vi la milimet)
x_a=-160;        % 400 0 150
y_a=-238;
z_a=85;
%% Nhap toa do diem C cho Robot ( don vi la milimet)
x_c=300;        %250 0 150      %200 0 150
y_c=109;
z_c=85;
%% Nhap toa do diem B cho Robot ( don vi la milimet)
x_b=300;        %250 250 150        %300 100 150
y_b=100;
z_b=85;
%% Kiem tra toa do diem nhap vao co phu hop khong
run=1;
if(((x_a==x_b)&&(y_a==y_b)&&(z_a==z_b))||((x_a==x_c)&&(y_a==y_c)&&(z_a==z_c))||((x_c==x_b)&&(y_c==y_b)&&(z_c==z_b)))
    sprintf('The points is coincident Please input the new Points')
    run=0;
end
if(x_a^2+y_a^2>400*400)
    sprintf('1Out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
    run=0;
end
if(x_a^2+y_a^2<200*200)
    sprintf('The mechanic of robot can not reach the position - The A point is too close to the base ')
    run=0;
end
if((z_a<0)||(z_a>136))
    sprintf('The mechanic of robot can not reach the position - The z_a is out of range ')
    run=0;
end
if (x_b^2+y_b^2>400*400)
    sprintf('2Out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
    run=0;
end
if(x_b^2+y_b^2<200*200)
    sprintf('The mechanic of robot can not reach the position - The B point is too close to the base ')
    run=0;
end
if((z_b<0)||(z_b>136))
    sprintf('The mechanic of robot can not reach the position - The z_b is out of range ')
    run=0;
end
if(x_c^2+y_c^2>400*400)
    sprintf('3Out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
    run=0;
end
if(x_c^2+y_c^2<200*200)
    sprintf('The mechanic of robot can not reach the position - The C point is too close to the base ')
    run=0;
end
if((z_c<0)||(z_c>136))
    sprintf('The mechanic of robot can not reach the position - The z_c is out of range ')
    run=0;
end
%% Kiem tra diem C co thang hang voi AB hay khong
A=[x_a y_a z_a];
B=[x_b y_b z_b];
C=[x_c y_c z_c];
V=[x_v y_v z_v];
Q=[x_q y_q z_q];
O=[x_o y_o z_o];
D=[x_d y_d z_d];

P=(A+B)/2;

Vector_AB =B-A;
Vector_CP =P-C;
TCH=cross(Vector_CP,Vector_AB);
TS_KC=sqrt(TCH(1,1)*TCH(1,1)+TCH(1,2)*TCH(1,2)+TCH(1,3)*TCH(1,3));
MS_KC=sqrt(Vector_AB(1,1)*Vector_AB(1,1)+Vector_AB(1,2)*Vector_AB(1,2)+Vector_AB(1,3)*Vector_AB(1,3));
Distance_C_AB=TS_KC/MS_KC;
if(Distance_C_AB==0)
    sprintf('The position of C point is in line with AB - Please Input the new position of C')
    run=0;
end
%% Xet Diem C va Diem O cung bo hay khac bo
x_o =((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(y_a*z_b - y_b*z_a - y_a*z_c + y_c*z_a + y_b*z_c - y_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_b*y_a^2 - x_a*y_c^2 - x_c*y_a^2 + x_b*y_c^2 + x_b*z_a^2 - x_a*z_c^2 - x_c*z_a^2 + x_b*z_c^2 - x_a*y_a*y_b + x_a*y_a*y_c + x_a*y_b*y_c - 2*x_b*y_a*y_c + x_c*y_a*y_b + x_c*y_a*y_c - x_c*y_b*y_c - x_a*z_a*z_b + x_a*z_a*z_c + x_a*z_b*z_c - 2*x_b*z_a*z_c + x_c*z_a*z_b + x_c*z_a*z_c - x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_a*y_b^2 + x_a*y_c^2 - x_b*y_c^2 - x_c*y_b^2 + x_a*z_b^2 + x_a*z_c^2 - x_b*z_c^2 - x_c*z_b^2 - x_b*y_a*y_b - 2*x_a*y_b*y_c + x_b*y_a*y_c + x_c*y_a*y_b + x_b*y_b*y_c - x_c*y_a*y_c + x_c*y_b*y_c - x_b*z_a*z_b - 2*x_a*z_b*z_c + x_b*z_a*z_c + x_c*z_a*z_b + x_b*z_b*z_c - x_c*z_a*z_c + x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
y_o =(((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_a^2*y_b - x_a^2*y_c - x_c^2*y_a + x_c^2*y_b + y_b*z_a^2 - y_a*z_c^2 - y_c*z_a^2 + y_b*z_c^2 - x_a*x_b*y_a + x_a*x_c*y_a + x_a*x_b*y_c - 2*x_a*x_c*y_b + x_b*x_c*y_a + x_a*x_c*y_c - x_b*x_c*y_c - y_a*z_a*z_b + y_a*z_a*z_c + y_a*z_b*z_c - 2*y_b*z_a*z_c + y_c*z_a*z_b + y_c*z_a*z_c - y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) - ((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(x_a*z_b - x_b*z_a - x_a*z_c + x_c*z_a + x_b*z_c - x_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_b^2*y_a + x_c^2*y_a - x_b^2*y_c - x_c^2*y_b + y_a*z_b^2 + y_a*z_c^2 - y_b*z_c^2 - y_c*z_b^2 - x_a*x_b*y_b + x_a*x_b*y_c + x_a*x_c*y_b - 2*x_b*x_c*y_a - x_a*x_c*y_c + x_b*x_c*y_b + x_b*x_c*y_c - y_b*z_a*z_b - 2*y_a*z_b*z_c + y_b*z_a*z_c + y_c*z_a*z_b + y_b*z_b*z_c - y_c*z_a*z_c + y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
z_o =((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(x_a*y_b - x_b*y_a - x_a*y_c + x_c*y_a + x_b*y_c - x_c*y_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_a^2*z_b - x_a^2*z_c - x_c^2*z_a + x_c^2*z_b + y_a^2*z_b - y_a^2*z_c - y_c^2*z_a + y_c^2*z_b - x_a*x_b*z_a + x_a*x_c*z_a + x_a*x_b*z_c - 2*x_a*x_c*z_b + x_b*x_c*z_a + x_a*x_c*z_c - x_b*x_c*z_c - y_a*y_b*z_a + y_a*y_c*z_a + y_a*y_b*z_c - 2*y_a*y_c*z_b + y_b*y_c*z_a + y_a*y_c*z_c - y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_b^2*z_a + x_c^2*z_a - x_b^2*z_c - x_c^2*z_b + y_b^2*z_a + y_c^2*z_a - y_b^2*z_c - y_c^2*z_b - x_a*x_b*z_b + x_a*x_b*z_c + x_a*x_c*z_b - 2*x_b*x_c*z_a - x_a*x_c*z_c + x_b*x_c*z_b + x_b*x_c*z_c - y_a*y_b*z_b + y_a*y_b*z_c + y_a*y_c*z_b - 2*y_b*y_c*z_a - y_a*y_c*z_c + y_b*y_c*z_b + y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);

R=sqrt((x_b-x_o)^2+(y_b-y_o)^2+(z_b-z_o)^2);

x_p= (x_a+x_b) /2;
y_p= (y_a+y_b) /2+R;
z_p= (z_a+z_b) /2;

Ox1 = [x_a-x_o, y_a-y_o , z_a-z_o];
Oz1 = cross([x_a-x_o, y_a-y_o , z_a-z_o],[x_b-x_o, y_b-y_o , z_b-z_o]);
Oy1 = cross(Oz1,Ox1);
        
i1  = Ox1/sqrt(Ox1(1,1)^2 + Ox1(1,2)^2 + Ox1(1,3)^2);
j1  = Oy1/sqrt(Oy1(1,1)^2 + Oy1(1,2)^2 + Oy1(1,3)^2);
k1 = Oz1/sqrt(Oz1(1,1)^2 + Oz1(1,2)^2 + Oz1(1,3)^2);

i0  = [1 0 0];
j0  = [0 1 0];
k0 =  [0 0 1];

R10 = [ i0*i1', i0*j1', i0*k1';
    j0*i1', j0*j1', j0*k1';
    k0*i1', k0*j1', k0*k1'];

H10 = [R10, [x_o; y_o; z_o];
    0     0     0     1];

OP=[x_p-x_o, y_p-y_o, z_p-z_o];
APB_x_c=(x_p-x_o)*(x_c-x_b)+(y_p-y_o)*(y_c-y_b)+(z_p-z_o)*(z_c-z_b);
APB_x_o=(x_p-x_o)*(x_o-x_b)+(y_p-y_o)*(y_o-y_b)+(z_p-z_o)*(z_o-z_b);

OA   = [x_a-x_o, y_a-y_o, z_a-z_o];
OB   = [x_b-x_o, y_b-y_o, z_b-z_o];
OA_md= sqrt((x_a-x_o)*(x_a-x_o)+(y_a-y_o)*(y_a-y_o)+(z_a-z_o)*(z_a-z_o));
OB_md= sqrt((x_b-x_o)*(x_b-x_o)+(y_b-y_o)*(y_b-y_o)+(z_b-z_o)*(z_b-z_o));
BA   = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2);
TS   = OA_md*OA_md+OB_md*OB_md-BA*BA;
MS   = 2*OA_md*OB_md;
AOB  = acos(TS/MS);
K=APB_x_c*APB_x_o;
if(K>=0)
    AOB=-(2*pi-acos(TS/MS));        
end
%% Kiemtra quydao
if(QD==8)   %% Ktra Circle Move
    for ii=0:1:1000
        PhanTram=ii/1000;
        x1_AOB = R*cos(PhanTram*AOB);
        y1_AOB = R*sin(PhanTram*AOB);
        z1_AOB = 0;
        
        toa_do_1=H10*[ x1_AOB; y1_AOB; z1_AOB; 1];
        P_x1_AOB = toa_do_1(1,1);
        P_y1_AOB = toa_do_1(2,1);
        P_z1_AOB = toa_do_1(3,1);
        if(P_x1_AOB^2+P_y1_AOB^2>400*400)
            sprintf('12Out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
            run=0;
            break
        end
        if(P_x1_AOB^2+P_y1_AOB^2<200*200)
            sprintf('13Out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
            run=0;
            break
        end
        if((P_z1_AOB>136)||(P_z1_AOB<0))
            sprintf('14Out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
            run=0;
            break
        end
        
        c2= (P_x1_AOB*P_x1_AOB + P_y1_AOB*P_y1_AOB -l2*l2 - l3*l3)/(2*l2*l3);
        s2_1=sqrt(1-c2*c2);
        s2_2=-sqrt(1-c2*c2);
        theta2_1=atan2(s2_1,c2);
        theta2_2=atan2(s2_2,c2);
        c1_1=((c2*l3+l2)*P_x1_AOB+s2_1*l3*P_y1_AOB)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
        c1_2=((c2*l3+l2)*P_x1_AOB+s2_2*l3*P_y1_AOB)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
        s1_1=((c2*l3+l2)*P_y1_AOB-s2_1*l3*P_x1_AOB)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
        s1_2=((c2*l3+l2)*P_y1_AOB-s2_2*l3*P_x1_AOB)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
        theta1_1=atan2(s1_1,c1_1);
        theta1_2=atan2(s1_2,c1_2);
        d=l0+l1-P_z1_AOB;
        delta1_1=abs(theta1_1-theta1_pre);          % denta1 la sai so tuyet doi giua goc vua tinh theo cac cong thuc dong hoc nghich tren va goc theta1 cua chu ky tinh truoc do
        delta1_2=abs(theta1_2-theta1_pre);
        
        delta_min=min([delta1_1 delta1_2]);
        
        if(delta_min==delta1_1)
            theta1= theta1_1;
            theta2= theta2_1;
        end
        
        if(delta_min==delta1_2)
            theta1= theta1_2;
            theta2= theta2_2;
        end
        
        theta1_pre=theta1;
        theta2_pre=theta2;
        
        if((theta1*180/pi>120)||(theta1*180/pi<-120))
            run=0;
            sprintf('Theta1 is out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
        end
        if((theta2*180/pi>120)||(theta2*180/pi<-120))
            run=0;
            sprintf('Theta2 is out of range - The mechanic of robot can not reach the position - The program can not run correctly ')
        end
        if(run==0)
            break;
        end
    end
end
%% Cai Dat ban dau chuan bi chay chuong trinh
if(run==1)
    sprintf('the input position is correct - the program is running')
    Magic=0.5;
    if (QD==0)  %% MoveL
        BA   = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2);
        tf_o = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2)/v_qd;
        tf   = (1+Magic)*tf_o;
        q_o  = 0;                 %% trang thai cua diem dau
        q_f  = BA;               %% trang thai cua diem cuoi
        tb   = Magic*tf_o;
        vqd  = (q_o-q_f)/(tb-tf);
        a    = vqd/tb;
    end
    if (QD==1)  %% Circle Move
        x_o =((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(y_a*z_b - y_b*z_a - y_a*z_c + y_c*z_a + y_b*z_c - y_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_b*y_a^2 - x_a*y_c^2 - x_c*y_a^2 + x_b*y_c^2 + x_b*z_a^2 - x_a*z_c^2 - x_c*z_a^2 + x_b*z_c^2 - x_a*y_a*y_b + x_a*y_a*y_c + x_a*y_b*y_c - 2*x_b*y_a*y_c + x_c*y_a*y_b + x_c*y_a*y_c - x_c*y_b*y_c - x_a*z_a*z_b + x_a*z_a*z_c + x_a*z_b*z_c - 2*x_b*z_a*z_c + x_c*z_a*z_b + x_c*z_a*z_c - x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_a*y_b^2 + x_a*y_c^2 - x_b*y_c^2 - x_c*y_b^2 + x_a*z_b^2 + x_a*z_c^2 - x_b*z_c^2 - x_c*z_b^2 - x_b*y_a*y_b - 2*x_a*y_b*y_c + x_b*y_a*y_c + x_c*y_a*y_b + x_b*y_b*y_c - x_c*y_a*y_c + x_c*y_b*y_c - x_b*z_a*z_b - 2*x_a*z_b*z_c + x_b*z_a*z_c + x_c*z_a*z_b + x_b*z_b*z_c - x_c*z_a*z_c + x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
        y_o =(((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_a^2*y_b - x_a^2*y_c - x_c^2*y_a + x_c^2*y_b + y_b*z_a^2 - y_a*z_c^2 - y_c*z_a^2 + y_b*z_c^2 - x_a*x_b*y_a + x_a*x_c*y_a + x_a*x_b*y_c - 2*x_a*x_c*y_b + x_b*x_c*y_a + x_a*x_c*y_c - x_b*x_c*y_c - y_a*z_a*z_b + y_a*z_a*z_c + y_a*z_b*z_c - 2*y_b*z_a*z_c + y_c*z_a*z_b + y_c*z_a*z_c - y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) - ((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(x_a*z_b - x_b*z_a - x_a*z_c + x_c*z_a + x_b*z_c - x_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_b^2*y_a + x_c^2*y_a - x_b^2*y_c - x_c^2*y_b + y_a*z_b^2 + y_a*z_c^2 - y_b*z_c^2 - y_c*z_b^2 - x_a*x_b*y_b + x_a*x_b*y_c + x_a*x_c*y_b - 2*x_b*x_c*y_a - x_a*x_c*y_c + x_b*x_c*y_b + x_b*x_c*y_c - y_b*z_a*z_b - 2*y_a*z_b*z_c + y_b*z_a*z_c + y_c*z_a*z_b + y_b*z_b*z_c - y_c*z_a*z_c + y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
        z_o =((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(x_a*y_b - x_b*y_a - x_a*y_c + x_c*y_a + x_b*y_c - x_c*y_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_a^2*z_b - x_a^2*z_c - x_c^2*z_a + x_c^2*z_b + y_a^2*z_b - y_a^2*z_c - y_c^2*z_a + y_c^2*z_b - x_a*x_b*z_a + x_a*x_c*z_a + x_a*x_b*z_c - 2*x_a*x_c*z_b + x_b*x_c*z_a + x_a*x_c*z_c - x_b*x_c*z_c - y_a*y_b*z_a + y_a*y_c*z_a + y_a*y_b*z_c - 2*y_a*y_c*z_b + y_b*y_c*z_a + y_a*y_c*z_c - y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_b^2*z_a + x_c^2*z_a - x_b^2*z_c - x_c^2*z_b + y_b^2*z_a + y_c^2*z_a - y_b^2*z_c - y_c^2*z_b - x_a*x_b*z_b + x_a*x_b*z_c + x_a*x_c*z_b - 2*x_b*x_c*z_a - x_a*x_c*z_c + x_b*x_c*z_b + x_b*x_c*z_c - y_a*y_b*z_b + y_a*y_b*z_c + y_a*y_c*z_b - 2*y_b*y_c*z_a - y_a*y_c*z_c + y_b*y_c*z_b + y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);  
        
        i0  = [1 0 0];
        j0  = [0 1 0];
        k0  = [0 0 1];
        
        Ox1 = [x_a-x_o, y_a-y_o , z_a-z_o];
        Oz1 = cross([x_a-x_o, y_a-y_o , z_a-z_o],[x_b-x_o, y_b-y_o , z_b-z_o]);
        if(cross([x_a-x_o, y_a-y_o , z_a-z_o],[x_b-x_o, y_b-y_o , z_b-z_o])==0)
            Oz1 = cross([x_a-x_o, y_a-y_o , z_a-z_o],[x_c-x_o, y_c-y_o , z_c-z_o]);
        end       
        Oy1 = cross(Oz1,Ox1);
        
        i1  = Ox1/sqrt(Ox1(1,1)^2 + Ox1(1,2)^2 + Ox1(1,3)^2);
        j1  = Oy1/sqrt(Oy1(1,1)^2 + Oy1(1,2)^2 + Oy1(1,3)^2);
        k1  = Oz1/sqrt(Oz1(1,1)^2 + Oz1(1,2)^2 + Oz1(1,3)^2);
        
        R10 = [ i0*i1', i0*j1', i0*k1';
            j0*i1', j0*j1', j0*k1';
            k0*i1', k0*j1', k0*k1'];
        
        H10 = [R10, [x_o; y_o; z_o];
            0 0 0 1];
        
        OP=[x_p-x_o, y_p-y_o, z_p-z_o];
        APB_x_c=(x_p-x_o)*(x_c-x_b)+(y_p-y_o)*(y_c-y_b)+(z_p-z_o)*(z_c-z_b);
        APB_x_o=(x_p-x_o)*(x_o-x_b)+(y_p-y_o)*(y_o-y_b)+(z_p-z_o)*(z_o-z_b);

        OA   = [x_a-x_o, y_a-y_o, z_a-z_o];
        OB   = [x_b-x_o, y_b-y_o, z_b-z_o];
        OA_md= sqrt((x_a-x_o)*(x_a-x_o)+(y_a-y_o)*(y_a-y_o)+(z_a-z_o)*(z_a-z_o));
        OB_md= sqrt((x_b-x_o)*(x_b-x_o)+(y_b-y_o)*(y_b-y_o)+(z_b-z_o)*(z_b-z_o));
        BA   = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2);
        TS   = OA_md*OA_md+OB_md*OB_md-BA*BA;
        MS   = 2*OA_md*OB_md;
        AOB  = acos(TS/MS);
        K=APB_x_c*APB_x_o;
        if(K>0)
            AOB=-(2*pi-acos(TS/MS));        
        end
        
        R=sqrt((x_b-x_o)^2+(y_b-y_o)^2+(z_b-z_o)^2);
        Cung_AB = abs(AOB*(2*pi*R)/(2*pi));
        tf_o = Cung_AB/v_qd;
        tf   = (1+Magic)*tf_o;
        q_o  = 0;                %% trang thai cua diem dau
        q_f  = AOB;               %% trang thai cua diem cuoi
        tb   = Magic*tf_o;
        vqd  = (q_o-q_f)/(tb-tf);
        a    = vqd/tb;
    end
    if (QD==2)  %% MoveJ
        tf = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2)/v_qd;
        %% Inverted Kinematic Diem A
        c2= (x_a*x_a + y_a*y_a -l2*l2 - l3*l3)/(2*l2*l3);
        s2_1=sqrt(1-c2*c2);
        s2_2=-sqrt(1-c2*c2);
        theta2_1_1=atan2(s2_1,c2);
        theta2_2_1=atan2(s2_2,c2);
        c1_1=((c2*l3+l2)*x_a+s2_1*l3*y_a)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
        c1_2=((c2*l3+l2)*x_a+s2_2*l3*y_a)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
        s1_1=((c2*l3+l2)*y_a-s2_1*l3*x_a)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
        s1_2=((c2*l3+l2)*y_a-s2_2*l3*x_a)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
        theta1_1_1=atan2(s1_1,c1_1);
        theta1_2_1=atan2(s1_2,c1_2);
        
        d_1=l0+l1-z_a-l4;
        %% Chon nghiem Diem A
        delta1_1=abs(theta1_1_1-theta1_pre)        % denta1 la sai so tuyet doi giua goc vua tinh theo cac cong thuc dong hoc nghich tren va goc theta1 cua chu ky tinh truoc do
        delta1_2=abs(theta1_2_1-theta1_pre)
        
        delta_min=min([delta1_1 delta1_2])
        if(delta_min==delta1_1)
            theta1_1= theta1_1_1;
            theta2_1= theta2_1_1;
        else
            theta1_1= theta1_2_1;
            theta2_1= theta2_2_1;
        end
        %% Inverted Kinematic Diem B
        c2= (x_b*x_b + y_b*y_b -l2*l2 - l3*l3)/(2*l2*l3);
        s2_1=sqrt(1-c2*c2);
        s2_2=-sqrt(1-c2*c2);
        theta2_1_2=atan2(s2_1,c2);
        theta2_2_2=atan2(s2_2,c2);
        c1_1=((c2*l3+l2)*x_b+s2_1*l3*y_b)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
        c1_2=((c2*l3+l2)*x_b+s2_2*l3*y_b)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
        s1_1=((c2*l3+l2)*y_b-s2_1*l3*x_b)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
        s1_2=((c2*l3+l2)*y_b-s2_2*l3*x_b)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
        theta1_1_2=atan2(s1_1,c1_1);
        theta1_2_2=atan2(s1_2,c1_2);
        
        d_2=l0+l1-z_b-l4;
        %% Chon nghiem Diem B
        delta1_1=abs(theta1_1_2-theta1_pre)        % denta1 la sai so tuyet doi giua goc vua tinh theo cac cong thuc dong hoc nghich tren va goc theta1 cua chu ky tinh truoc do
        delta1_2=abs(theta1_2_2-theta1_pre)
        
        delta_min=min([delta1_1 delta1_2])
        if(delta_min==delta1_1)
            theta1_2= theta1_1_2;
            theta2_2= theta2_1_2;
        else
            theta1_2= theta1_2_2;
            theta2_2= theta2_2_2;
        end
        
        %Quy hoach bien phantram
        x_0       = 0;            %% trang thai cua diem dau
        x_dot_0   = 0;
        x_2dot_0  = 0;
        
        x_F       = 100;            %% trang thai cua diem cuoi
        x_dot_F   = 0;
        x_2dot_F  = 0;
        
        a0=x_0;
        a1=x_dot_0;
        a2=x_2dot_0;
        a3=(20*x_F - 20*x_0-(8*x_dot_F + 12*x_dot_0)*tf -(3*x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^3);
        a4=(30*x_0 - 30*x_F + (14*x_dot_F + 16*x_dot_0)*tf +(3*x_2dot_0 - 2*x_2dot_F)*tf^2)/(2*tf^4);
        a5=(12*x_F - 12*x_0 - (6*x_dot_F + 6*x_dot_0)*tf -(x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^5);
    end
    if (QD==3)  %% MoveP
        Magic=0.5;          %% That name just for fun !! There is no magic here it's just a variable to inverted scale the time of motion to fix the setpoint of speed. The maximum value of this variable is belong to the acceleration of joint actuator
        %% tien xu ly Move P
        CA=A-C;
        CB=C-B;
        n=cross(CA,CB);
        CA_md=sqrt((x_a-x_c)*(x_a-x_c)+(y_a-y_c)*(y_a-y_c)+(z_a-z_c)*(z_a-z_c));
        CB_md=sqrt((x_b-x_c)*(x_b-x_c)+(y_b-y_c)*(y_b-y_c)+(z_b-z_c)*(z_b-z_c));
        if(CA_md<=CB_md)
            R_P_max=CA_md
            if(R_P>CA_md)
                R_P=CA_md;
            end
        end
        if(CB_md<CA_md)
            R_P_max=CB_md
            if(R_P>CB_md)
                R_P=CB_md;
            end
        end
        V=C+(A-C)*(R_P/CA_md);
        x_v=V(1,1);
        y_v=V(1,2);
        z_v=V(1,3);
        Q=C+(B-C)*(R_P/CB_md);
        x_q=Q(1,1);
        y_q=Q(1,2);
        z_q=Q(1,3);
        % Tinh tam O
%         M_A=[CA;CB;n];
%         M_B=[CA*V'; CB*Q';n*C'];
%         Nghiem=inv(M_A)*M_B;
%         x_o=Nghiem(1,1);
%         y_o=Nghiem(2,1);
%         z_o=Nghiem(3,1);
        x_o =((conj(z_c)*((x_a - x_c)*(y_b - y_c) - (x_b - x_c)*(y_a - y_c)) - conj(y_c)*((x_a - x_c)*(z_b - z_c) - (x_b - x_c)*(z_a - z_c)) + conj(x_c)*((y_a - y_c)*(z_b - z_c) - (y_b - y_c)*(z_a - z_c)))*(y_a*z_b - y_b*z_a - y_a*z_c + y_c*z_a + y_b*z_c - y_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + ((conj(x_q)*(x_b - x_c) + conj(y_q)*(y_b - y_c) + conj(z_q)*(z_b - z_c))*(x_b*y_a^2 - x_a*y_c^2 - x_c*y_a^2 + x_b*y_c^2 + x_b*z_a^2 - x_a*z_c^2 - x_c*z_a^2 + x_b*z_c^2 - x_a*y_a*y_b + x_a*y_a*y_c + x_a*y_b*y_c - 2*x_b*y_a*y_c + x_c*y_a*y_b + x_c*y_a*y_c - x_c*y_b*y_c - x_a*z_a*z_b + x_a*z_a*z_c + x_a*z_b*z_c - 2*x_b*z_a*z_c + x_c*z_a*z_b + x_c*z_a*z_c - x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + ((conj(x_v)*(x_a - x_c) + conj(y_v)*(y_a - y_c) + conj(z_v)*(z_a - z_c))*(x_a*y_b^2 + x_a*y_c^2 - x_b*y_c^2 - x_c*y_b^2 + x_a*z_b^2 + x_a*z_c^2 - x_b*z_c^2 - x_c*z_b^2 - x_b*y_a*y_b - 2*x_a*y_b*y_c + x_b*y_a*y_c + x_c*y_a*y_b + x_b*y_b*y_c - x_c*y_a*y_c + x_c*y_b*y_c - x_b*z_a*z_b - 2*x_a*z_b*z_c + x_b*z_a*z_c + x_c*z_a*z_b + x_b*z_b*z_c - x_c*z_a*z_c + x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
        y_o =((conj(x_q)*(x_b - x_c) + conj(y_q)*(y_b - y_c) + conj(z_q)*(z_b - z_c))*(x_a^2*y_b - x_a^2*y_c - x_c^2*y_a + x_c^2*y_b + y_b*z_a^2 - y_a*z_c^2 - y_c*z_a^2 + y_b*z_c^2 - x_a*x_b*y_a + x_a*x_c*y_a + x_a*x_b*y_c - 2*x_a*x_c*y_b + x_b*x_c*y_a + x_a*x_c*y_c - x_b*x_c*y_c - y_a*z_a*z_b + y_a*z_a*z_c + y_a*z_b*z_c - 2*y_b*z_a*z_c + y_c*z_a*z_b + y_c*z_a*z_c - y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) - ((conj(z_c)*((x_a - x_c)*(y_b - y_c) - (x_b - x_c)*(y_a - y_c)) - conj(y_c)*((x_a - x_c)*(z_b - z_c) - (x_b - x_c)*(z_a - z_c)) + conj(x_c)*((y_a - y_c)*(z_b - z_c) - (y_b - y_c)*(z_a - z_c)))*(x_a*z_b - x_b*z_a - x_a*z_c + x_c*z_a + x_b*z_c - x_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + ((conj(x_v)*(x_a - x_c) + conj(y_v)*(y_a - y_c) + conj(z_v)*(z_a - z_c))*(x_b^2*y_a + x_c^2*y_a - x_b^2*y_c - x_c^2*y_b + y_a*z_b^2 + y_a*z_c^2 - y_b*z_c^2 - y_c*z_b^2 - x_a*x_b*y_b + x_a*x_b*y_c + x_a*x_c*y_b - 2*x_b*x_c*y_a - x_a*x_c*y_c + x_b*x_c*y_b + x_b*x_c*y_c - y_b*z_a*z_b - 2*y_a*z_b*z_c + y_b*z_a*z_c + y_c*z_a*z_b + y_b*z_b*z_c - y_c*z_a*z_c + y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
        z_o =((conj(z_c)*((x_a - x_c)*(y_b - y_c) - (x_b - x_c)*(y_a - y_c)) - conj(y_c)*((x_a - x_c)*(z_b - z_c) - (x_b - x_c)*(z_a - z_c)) + conj(x_c)*((y_a - y_c)*(z_b - z_c) - (y_b - y_c)*(z_a - z_c)))*(x_a*y_b - x_b*y_a - x_a*y_c + x_c*y_a + x_b*y_c - x_c*y_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + ((conj(x_q)*(x_b - x_c) + conj(y_q)*(y_b - y_c) + conj(z_q)*(z_b - z_c))*(x_a^2*z_b - x_a^2*z_c - x_c^2*z_a + x_c^2*z_b + y_a^2*z_b - y_a^2*z_c - y_c^2*z_a + y_c^2*z_b - x_a*x_b*z_a + x_a*x_c*z_a + x_a*x_b*z_c - 2*x_a*x_c*z_b + x_b*x_c*z_a + x_a*x_c*z_c - x_b*x_c*z_c - y_a*y_b*z_a + y_a*y_c*z_a + y_a*y_b*z_c - 2*y_a*y_c*z_b + y_b*y_c*z_a + y_a*y_c*z_c - y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + ((conj(x_v)*(x_a - x_c) + conj(y_v)*(y_a - y_c) + conj(z_v)*(z_a - z_c))*(x_b^2*z_a + x_c^2*z_a - x_b^2*z_c - x_c^2*z_b + y_b^2*z_a + y_c^2*z_a - y_b^2*z_c - y_c^2*z_b - x_a*x_b*z_b + x_a*x_b*z_c + x_a*x_c*z_b - 2*x_b*x_c*z_a - x_a*x_c*z_c + x_b*x_c*z_b + x_b*x_c*z_c - y_a*y_b*z_b + y_a*y_b*z_c + y_a*y_c*z_b - 2*y_b*y_c*z_a - y_a*y_c*z_c + y_b*y_c*z_b + y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
 
        O=[x_o y_o z_o];
        % Tinh goc AOB
        OV_md=sqrt((x_v-x_o)*(x_v-x_o)+(y_v-y_o)*(y_v-y_o)+(z_v-z_o)*(z_v-z_o));
        QO_md=sqrt((x_o-x_q)*(x_o-x_q)+(y_o-y_q)*(y_o-y_q)+(z_o-z_q)*(z_o-z_q));
        VQ_md=sqrt((x_q-x_v)*(x_q-x_v)+(y_q-y_v)*(y_q-y_v)+(z_q-z_v)*(z_q-z_v));
        TS =OV_md*OV_md+QO_md*QO_md-VQ_md*VQ_md;
        MS=2*OV_md*QO_md;
        VOQ=acos(TS/MS);
        VOQ_thuc=VOQ*180/pi;
        %% MoveP_L_1
        AV    = sqrt((x_v-x_a)^2+(y_v-y_a)^2+(z_v-z_a)^2);
        tf_1  = sqrt((x_v-x_a)^2+(y_v-y_a)^2+(z_v-z_a)^2)/v_qd;
        %% MoveP_C_2
        i0  = [1 0 0];
        j0  = [0 1 0];
        k0  = [0 0 1];
        
        Ox1 = [x_v-x_o, y_v-y_o , z_v-z_o];
        Oz1 = cross([x_v-x_o, y_v-y_o , z_v-z_o],[x_q-x_o, y_q-y_o , z_q-z_o]);
        Oy1 = cross(Oz1,Ox1);
        
        i1  = Ox1/sqrt(Ox1(1,1)^2 + Ox1(1,2)^2 + Ox1(1,3)^2);
        j1  = Oy1/sqrt(Oy1(1,1)^2 + Oy1(1,2)^2 + Oy1(1,3)^2);
        k1  = Oz1/sqrt(Oz1(1,1)^2 + Oz1(1,2)^2 + Oz1(1,3)^2);
        
        R10 = [ i0*i1', i0*j1', i0*k1';
            j0*i1', j0*j1', j0*k1';
            k0*i1', k0*j1', k0*k1'];
        
        H10 = [R10, [x_o; y_o; z_o];
            0 0 0 1];
        
        R_2=sqrt((x_v-x_o)^2+(y_v-y_o)^2+(z_v-z_o)^2); %OV
        Cung_VQ = abs(VOQ*(2*pi*R_2)/(2*pi));
        tf_2=Cung_VQ/v_qd;
        %% MoveP_L_3
        QB      = sqrt((x_b-x_q)^2+(y_b-y_q)^2+(z_b-z_q)^2);
        tf_3    = sqrt((x_b-x_q)^2+(y_b-y_q)^2+(z_b-z_q)^2)/v_qd;
        quangduongAB=AV+Cung_VQ+QB;
        phantram1= (100*AV)/(quangduongAB);
        phantram2=(100*(AV+Cung_VQ))/(quangduongAB);
        %%
        tf_o=(tf_1+tf_2+tf_3);
        tf=(1+Magic)*(tf_1+tf_2+tf_3);
        q_o = 0;                %% trang thai cua diem dau
        q_f = 100;               %% trang thai cua diem cuoi
        tb    = Magic*tf_o;
        vqd   = (q_o - q_f)/(tb - tf);
        a     = vqd/tb;
    end
    if (QD==4)  %% MoveC
        x_o= (x_a+x_b) /2;
        y_o= (y_a+y_b) /2;
        z_o= (z_a+z_b) /2;
        
        R=sqrt((x_b-x_o)^2+(y_b-y_o)^2+(z_b-z_o)^2);
        
        Ox1 = [x_a-x_o, y_a-y_o , z_a-z_o];
        Oz1 = cross([x_c-x_o, y_c-y_o , z_c-z_o],[x_a-x_o, y_a-y_o , z_a-z_o]);
        Oy1 = cross(Ox1,Oz1);

        i1  = Ox1/sqrt(Ox1(1,1)^2 + Ox1(1,2)^2 + Ox1(1,3)^2);
        j1  = Oy1/sqrt(Oy1(1,1)^2 + Oy1(1,2)^2 + Oy1(1,3)^2);
        k1 = Oz1/sqrt(Oz1(1,1)^2 + Oz1(1,2)^2 + Oz1(1,3)^2);

        i0  = [1 0 0];
        j0  = [0 1 0];
        k0 =  [0 0 1];

        R10 = [ i0*i1', i0*j1', i0*k1';
            j0*i1', j0*j1', j0*k1';
            k0*i1', k0*j1', k0*k1'];

        H10 = [R10, [x_o; y_o; z_o];
            0     0     0     1];
        
        AOB = 2*pi;
        Cung_AB = 2*pi*R;
        tf_o = Cung_AB/v_qd;
        tf   = (1+Magic)*tf_o;
        q_o  = 0;                %% trang thai cua diem dau
        q_f  = AOB;               %% trang thai cua diem cuoi
        tb   = Magic*tf_o;
        vqd  = (q_o-q_f)/(tb-tf);
        a    = vqd/tb;
    end
    %*********************************************************************************
    if(QD==0||QD==1||QD==2||QD==4) 
        for i=1:1:(tf/ts)
            if(QD==0)   %% MoveL
                t=t;
                if((t>=0)&&(t<=tb))
                    st = q_o + 0.5*a*t*t;
                end
                if((t>=tb)&&(t<=(tf-tb)))
                    st=((q_o+q_f-vqd*tf)/2)+vqd*t;
                end
                if((t>=(tf-tb))&&(t<=tf))
                    st = q_f-0.5*a*tf*tf+a*tf*t-0.5*a*t*t;
                    % st = q_f+0.5*a*tf*tf-a*tf*t+0.5*a*t*t;
                end
                Px = x_a + st*(x_b-x_a)/BA;
                Py = y_a + st*(y_b-y_a)/BA;
                Pz = z_a + st*(z_b-z_a)/BA;
            end
            if(QD==1)   %%Circle Move
                if((t>=0)&&(t<=tb))
                    Psi = q_o + 0.5*a*t*t;
                end
                if((t>=tb)&&(t<=(tf-tb)))
                    Psi=((q_o+q_f-vqd*tf)/2)+vqd*t;
                end
                if((t>=(tf-tb))&&(t<=tf))
                    Psi = q_f-0.5*a*tf*tf+a*tf*t-0.5*a*t*t;
                    % st = q_f+0.5*a*tf*tf-a*tf*t+0.5*a*t*t;
                end
                x1_t = R*cos(Psi);
                y1_t = R*sin(Psi);
                z1_t = 0;
                toa_do=H10*[ x1_t; y1_t; z1_t; 1];
                Px=toa_do(1,1);
                Py=toa_do(2,1);
                Pz=toa_do(3,1);
            end
            if(QD==2)   %%Move J
                phantram = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
                theta1 = theta1_1 + (phantram*(theta1_2-theta1_1))/100;
                theta2 = theta2_1 + (phantram*(theta2_2-theta2_1))/100;
                d = d_1 + (phantram*(d_2-d_1))/100;
            end
            if(QD==4)   %%Circle Move
                if((t>=0)&&(t<=tb))
                    Psi = q_o + 0.5*a*t*t;
                end
                if((t>=tb)&&(t<=(tf-tb)))
                    Psi=((q_o+q_f-vqd*tf)/2)+vqd*t;
                end
                if((t>=(tf-tb))&&(t<=tf))
                    Psi = q_f-0.5*a*tf*tf+a*tf*t-0.5*a*t*t;
                end  
                x1_t = R*cos(Psi);
                y1_t = R*sin(Psi);
                z1_t = 0;
                toa_do=H10*[ x1_t; y1_t; z1_t; 1];
                Px=toa_do(1,1);
                Py=toa_do(2,1);
                Pz=toa_do(3,1);
            end
            if(QD==0||QD==1||QD==4)
                %% Inverted Kinematic
                c2= (Px*Px + Py*Py -l2*l2 - l3*l3)/(2*l2*l3);
                s2_1=sqrt(1-c2*c2);
                s2_2=-sqrt(1-c2*c2);
                theta2_1=atan2(s2_1,c2);
                theta2_2=atan2(s2_2,c2);
                c1_1=((c2*l3+l2)*Px+s2_1*l3*Py)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
                c1_2=((c2*l3+l2)*Px+s2_2*l3*Py)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
                s1_1=((c2*l3+l2)*Py-s2_1*l3*Px)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
                s1_2=((c2*l3+l2)*Py-s2_2*l3*Px)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
                theta1_1=atan2(s1_1,c1_1);
                theta1_2=atan2(s1_2,c1_2);
                
                d=l0+l1-Pz-l4;
                %% Chon nghiem
                delta1_1=abs(theta1_1-theta1_pre);        % denta1 la sai so tuyet doi giua goc vua tinh theo cac cong thuc dong hoc nghich tren va goc theta1 cua chu ky tinh truoc do
                delta1_2=abs(theta1_2-theta1_pre);
                
                delta_min=min([delta1_1 delta1_2]);
                if(delta_min==delta1_1)
                    theta1= theta1_1;
                    theta2= theta2_1;
                else
                    theta1= theta1_2;
                    theta2= theta2_2;
                end %
            end
            %%
            theta1_dot=(theta1-theta1_pre)/ts;
            theta2_dot=(theta2-theta2_pre)/ts;
            d_dot=(d-d_pre)/ts;
            d_pre=d;
            vantoc_theta1=theta1_dot*180/pi;
            vantoc_theta2=theta2_dot*180/pi;
            
            theta1_pre=theta1;
            theta2_pre=theta2;
            
            J=[-l3*sin(theta1+theta2)-l2*sin(theta1)  -l3*sin(theta1+theta2)     0   0;
                l3*cos(theta1+theta2)+l2*cos(theta1)   l3*cos(theta1+theta2)     0   0;
                0                                      0                        -1   0;
                0                                      0                         0   1];
            D=[theta1_dot; theta2_dot; d_dot; theta3_dot];
            V=J*D;
            Vt=sqrt(V(1,1)*V(1,1)+V(2,1)*V(2,1)+V(3,1)*V(3,1));
            
            At=(Vt-Vt_pre)/ts;
            Vt_pre=Vt;
            
            EE_x=  l3*cos(theta1 + theta2) + l2*cos(theta1);
            EE_y=  l3*sin(theta1 + theta2) + l2*sin(theta1);
            EE_z=  l0 - d + l1 -l4;
            truc_2_x=200*cos(theta1);
            truc_2_y=200*sin(theta1);
            truc_2_z=258;
            
            subplot(1,5,1);
            plot3([0,0 , truc_2_x,EE_x, EE_x],[0,0 , truc_2_y,EE_y, EE_y],[0,258 , truc_2_z ,EE_z+d+l4, EE_z],'o-','LineWidth',2,'MarkerEdgeColor','b');
            grid on;
            hold on;
            xlim([-500 500]);
            ylim([-500 500]);
            zlim([0 300]);
            %% ve do thi
            if(QD==0||QD==2)
                plot3([x_a,x_b],[y_a,y_b],[z_a,z_b],'o-','LineWidth',2,'MarkerEdgeColor','r');
                hold off;
            end
            if(QD==1)
                plot3([x_b,x_c,x_a],[y_b,y_c,y_a],[z_b,z_c,z_a],'o-','LineWidth',2,'MarkerEdgeColor','r');
                hold on;
                plot3(x_o,y_o,z_o,'o-','LineWidth',2,'MarkerEdgeColor','r');
                hold off;
            end
            if(QD==4)
                plot3([x_b,x_a],[y_b,y_a],[z_b,z_a],'o-','LineWidth',2,'MarkerEdgeColor','r');
%                 hold on;
%                 plot3(x_o,y_o,z_o,'o-','LineWidth',2,'MarkerEdgeColor','r');
                hold off;
            end
            
            subplot(1,5,2);
            plot3(EE_x,EE_y,EE_z,'o-','LineWidth',2,'MarkerEdgeColor','r');
            hold on;
            grid on;
            xlim([-500 500]);
            ylim([-500 500]);
            zlim([0 300]);
            
            subplot(1,5,3);
            if(QD==0)
                plot(t,st*100/BA,'o-','LineWidth',2,'MarkerEdgeColor','b')
                xlim([-1 tf*1.5]);
                ylim([-1 120]);
                hold on;
                grid on;
            end
            if(QD==1||QD==4)
                plot(t,Psi*100/AOB,'o-','LineWidth',2,'MarkerEdgeColor','b')
                xlim([-1 tf*1.5]);
                ylim([-1 120]);
                hold on;
                grid on;
            end
            if(QD==2)
                plot(t,phantram,'o-','LineWidth',2,'MarkerEdgeColor','b')
                xlim([-1 tf*1.5]);
                ylim([-1 120]);
                hold on;
                grid on;
            end
            
            subplot(1,5,4);
            plot(t,Vt,'o-','LineWidth',2,'MarkerEdgeColor','b')
            xlim([-1 tf*1.5]);
            ylim([-1 700]);
            hold on;
            grid on;
            
            subplot(1,5,5);
            plot(t,At,'o-','LineWidth',2,'MarkerEdgeColor','b')
            xlim([-1 tf*1.5]);
            ylim([-500 500]);
            hold on;
            grid on;
            
            pause(0.0001);
            t=t+ts;
            if(t>tf)
                break;
            end
%             if(t<ts*4)
%                 pause(2);
%                 sprintf('Ohyeah')
%             end
        end
        %%clc;
        sprintf('the simulation completed')
    end
    %*********************************************************************************
    if(QD==3)   %%Move P     
        for i=1:1:(tf/ts)
            if((t>=0)&&(t<=tb))
                phantram = q_o+ 0.5*a*t*t;
            end
            if((t>=tb)&&(t<=(tf-tb)))
                phantram=((q_o+q_f-vqd*tf)/2)+vqd*t;
            end
            if((t>=(tf-tb))&&(t<=tf))
                phantram = q_f-0.5*a*tf*tf+a*tf*t-0.5*a*t*t;
            end
            
            if(phantram<=phantram1)
                phantramqd1=phantram*100/phantram1;
                Px = x_a + phantramqd1*(x_v-x_a)/100;
                Py = y_a + phantramqd1*(y_v-y_a)/100;
                Pz = z_a + phantramqd1*(z_v-z_a)/100;
            end         
            if(phantram>=phantram1&&phantram<=phantram2)
                phantramqd2=((phantram-phantram1)*100)/(phantram2-phantram1);
                Psi=(phantramqd2*VOQ)/100;
                x1_t = R_2*cos(Psi);
                y1_t = R_2*sin(Psi);
                z1_t = 0;
                toa_do=H10*[ x1_t; y1_t; z1_t; 1];
                Px=toa_do(1,1);
                Py=toa_do(2,1);
                Pz=toa_do(3,1);
            end
            if(phantram>=phantram2&&phantram<=100)
                phantramqd3=(100*(phantram-phantram2))/(100-phantram2);
                Px = x_q + phantramqd3*(x_b-x_q)/100;
                Py = y_q + phantramqd3*(y_b-y_q)/100;
                Pz = z_q + phantramqd3*(z_b-z_q)/100;
            end
            
            v_x_qhqd=(Px-Px_pre)/ts;
            v_y_qhqd=(Py-Py_pre)/ts;
            v_z_qhqd=(Pz-Pz_pre)/ts;
            Px_pre=Px;
            Py_pre=Py;
            Pz_pre=Pz;
            %% Inverted Kinematic
            c2= (Px*Px + Py*Py -l2*l2 - l3*l3)/(2*l2*l3);
            s2_1=sqrt(1-c2*c2);
            s2_2=-sqrt(1-c2*c2);
            theta2_1=atan2(s2_1,c2);
            theta2_2=atan2(s2_2,c2);
            c1_1=((c2*l3+l2)*Px+s2_1*l3*Py)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
            c1_2=((c2*l3+l2)*Px+s2_2*l3*Py)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
            s1_1=((c2*l3+l2)*Py-s2_1*l3*Px)/((c2*l3+l2)*(c2*l3+l2)+(s2_1*l3)*(s2_1*l3));
            s1_2=((c2*l3+l2)*Py-s2_2*l3*Px)/((c2*l3+l2)*(c2*l3+l2)+(s2_2*l3)*(s2_2*l3));
            theta1_1=atan2(s1_1,c1_1);
            theta1_2=atan2(s1_2,c1_2);
            
            d=l0+l1-Pz-l4;
            %% Chon nghiem
            delta1_1=abs(theta1_1-theta1_pre);        % denta1 la sai so tuyet doi giua goc vua tinh theo cac cong thuc dong hoc nghich tren va goc theta1 cua chu ky tinh truoc do
            delta1_2=abs(theta1_2-theta1_pre);
            
            delta_min=min([delta1_1 delta1_2]);
            if(delta_min==delta1_1)
                theta1= theta1_1;
                theta2= theta2_1;
            else
                theta1= theta1_2;
                theta2= theta2_2;
            end
            %% Jacobian and Inveted Jacobian
            theta1_dot=(theta1-theta1_pre)/ts;
            theta2_dot=(theta2-theta2_pre)/ts;
            d_dot=(d-d_pre)/ts;
            
            theta1_pre=theta1;
            theta2_pre=theta2;
            d_pre=d;
            
            J=[-l3*sin(theta1+theta2)-l2*sin(theta1)  -l3*sin(theta1+theta2)     0   0;
                l3*cos(theta1+theta2)+l2*cos(theta1)   l3*cos(theta1+theta2)     0   0;
                0                                      0                        -1   0;
                0                                      0                         0   1];
            D=[theta1_dot; theta2_dot; d_dot; theta3_dot];
            V=J*D;
            Vt=sqrt(V(1,1)*V(1,1)+V(2,1)*V(2,1)+V(3,1)*V(3,1));         %% V  real
            
            At=(Vt-Vt_pre)/ts;
            Vt_pre=Vt;
            
            JacobiNghich=inv(J);
            v_w_qhqd=0;
            V_QHQD=[v_x_qhqd;v_y_qhqd;v_z_qhqd;v_w_qhqd];
            D=JacobiNghich*V_QHQD;           %% van toc cua quy hoach quy dao
            theta1_dot_estimation      =   D(1,1);
            theta2_dot_estimation      =   D(2,1);
            d_dot_estimation           =   D(3,1);
            theta3_dot_estimation      =   D(4,1);
            
            %             if((t>0.2)&&(t<0.3))
            %                 theta1_dot=theta1_dot+theta1_dot*0.21;
            %                 d_dot=d_dot+d_dot*0.1;
            %             end
            
            gamma=0.2;
            if(t>(10*ts))
                if(theta1_dot>5*pi/180)
                    if((theta1_dot>=(theta1_dot_estimation+theta1_dot_estimation*gamma)) || (theta1_dot<=(theta1_dot_estimation-theta1_dot_estimation*gamma)))
                        sprintf('Warning The joint 1 has impact - Please check out the system ')
                        break;
                    end
                end
                if(theta1_dot<-5*pi/180)
                    if((theta1_dot<=(theta1_dot_estimation+theta1_dot_estimation*gamma)) || (theta1_dot>=(theta1_dot_estimation-theta1_dot_estimation*gamma)))
                        sprintf('Warning The joint 1 has impact - Please check out the system ')
                        break;
                    end
                end
                
                if(theta2_dot>5*pi/180)
                    if((theta2_dot>=(theta2_dot_estimation+theta2_dot_estimation*gamma)) || (theta2_dot<=(theta2_dot_estimation-theta2_dot_estimation*gamma)))
                        sprintf('Warning The joint 2 has impact - Please check out the system ')
                        break;
                    end
                end
                if(theta2_dot<-5*pi/180)
                    if((theta2_dot<=(theta2_dot_estimation+theta2_dot_estimation*gamma)) || (theta2_dot>=(theta2_dot_estimation-theta2_dot_estimation*gamma)))
                        sprintf('Warning The joint 2 has impact - Please check out the system ')
                        break;
                    end
                end
                if(d_dot>5)
                    if((d_dot>=(d_dot_estimation+d_dot_estimation*gamma)) || (d_dot<=(d_dot_estimation-d_dot_estimation*gamma)))
                        sprintf('Warning The joint 3 has impact - Please check out the system ')
                        break;
                    end
                end
                if(d_dot<-5)
                    if((d_dot<=(d_dot_estimation+d_dot_estimation*gamma)) || (d_dot>=(d_dot_estimation-d_dot_estimation*gamma)))
                        sprintf('Warning The joint 3 has impact - Please check out the system ')
                        break;
                    end
                end
            end
            
            EE_x=  l3*cos(theta1 + theta2) + l2*cos(theta1);
            EE_y=  l3*sin(theta1 + theta2) + l2*sin(theta1);
            EE_z=  l0 - d + l1 -l4;
            truc_2_x=200*cos(theta1);
            truc_2_y=200*sin(theta1);
            truc_2_z=258;
            subplot(1,4,1);
            plot3([0,0 , truc_2_x,EE_x, EE_x],[0,0 , truc_2_y,EE_y, EE_y],[0,258 , truc_2_z ,EE_z+d+l4, EE_z],'o-','LineWidth',2,'MarkerEdgeColor','b');
            grid on;
            hold on;
            xlim([-500 500]);
            ylim([-500 500]);
            zlim([0 300]);
            
            %% ve do thi
            plot3([x_b,x_c,x_a],[y_b,y_c,y_a],[z_b,z_c,z_a],'o-','LineWidth',2,'MarkerEdgeColor','r');
            hold off;
            
            subplot(1,4,2);
            plot3(EE_x,EE_y,EE_z,'o-','LineWidth',2,'MarkerEdgeColor','r');
            hold on;
            grid on;
            xlim([-500 500]);
            ylim([-500 500]);
            zlim([0 300]);
            
            subplot(1,4,3);
            plot(t,Vt,'o-','LineWidth',2,'MarkerEdgeColor','b')
            xlim([-1 tf*1.5]);
            ylim([-1 700]);
            hold on;
            grid on;
            
            subplot(1,4,4);
            plot(t,At,'o-','LineWidth',2,'MarkerEdgeColor','b')
            xlim([-1 tf*1.5]);
            ylim([-100 100]);
            hold on;
            grid on;
            
            pause(0.001);
            t=t+ts;
            if(t>(tf))
                break;
            end
            if(t<ts*4)
                pause(2);
                sprintf('Ohyeah')
            end
        end
        %clc;
        sprintf('the simulation completed')
    end
end




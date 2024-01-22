clc; clear all; close all;

syms x_a y_a z_a x_b y_b z_b x_c y_c z_c

% % %% Toa do diem dau
% x_a   = 0;
% y_a   = 1;
%  z_a   = 0;
% % %% Toa do diem den
% x_b   = 0;
% y_b   = -1;
%  z_b   = 0;
% % %% Toa do diem trung gian
%  x_c   =  0;
%  y_c   =  0;
%  z_c   =  1;
%% Tim toa do diem M, M la trung diem AC
x_m  = (x_a + x_c)/2;
y_m  = (y_a + y_c)/2; 
z_m  = (z_a + z_c)/2;
%% Tim toa do diem N, N la trung diem BC
x_n  = (x_b + x_c)/2;
y_n  = (y_b + y_c)/2;
z_n  = (z_b + z_c)/2;
%% Tim vector CA
CA = [x_a - x_c     y_a - y_c        z_a - z_c];
%% Tim vector CB
CB = [x_b - x_c     y_b - y_c        z_b - z_c];
%% Tim vector phap tuyen cua mat phang gamma - n
n = cross(CA,CB);

M_A = [     (x_a - x_c)   (y_a - y_c)   (z_a - z_c);
                  (x_b - x_c)   (y_b - y_c)   (z_b - z_c)
                  n(1,1)            n(1,2)          n(1,3)];
              
X1 = x_m*(x_a - x_c) + y_m*(y_a - y_c) + z_m*(z_a - z_c);
X2 = x_n*(x_b - x_c) + y_n*(y_b - y_c) + z_n*(z_b - z_c);
X3 = x_c*n(1,1) + y_c*n(1,2) + z_c*n(1,3) ;

M_B = [X1;X2;X3];

O = inv(M_A)*M_B

% O = simplify(inv(M_A)*M_B)
% sprintf("Simulation Done")
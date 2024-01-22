clear all; close all; clc;
syms    tf     a0 a1 a2 a3 a4 a5
syms   q0    q0_dot   q0_2dot 
syms   qf     qf_dot     qf_2dot

M_A = [ 1 0 0 0 0 0;
            0 1 0 0 0 0;
            0 0 2 0 0 0;
            1 tf tf^2 tf^3 tf^4 tf^5;
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 6*tf 12*tf^2 20*tf^3;
            ];

M_B = [a0; 
            a1; 
            a2; 
            a3; 
            a4; 
            a5];
        
M_C = [q0;
            q0_dot;
            q0_2dot;
            qf ;
            qf_dot;
            qf_2dot;
            ];
        
M_B = simplify(inv(M_A)*M_C)
        
        
        



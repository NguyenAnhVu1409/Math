clear all; clc; 
ts = 0.001;
t = 0;
LastCycle=0;

q_0           = 0;            %% trang thai cua diem dau
q_dot_0    = 0;
q_2dot_0  = 0;

q_F           = 100;        %% trang thai cua diem cuoi
q_dot_F    = 0;
q_2dot_F  = 0;

v_qd  = 100;  % [%]/[s]
tf = abs(q_F - q_0)/v_qd;

a0=q_0;
a1=q_dot_0;
a2=q_2dot_0/2;
a3=(20*q_F - 20*q_0-(8*q_dot_F + 12*q_dot_0)*tf -(3*q_2dot_0 - q_2dot_F)*tf^2)/(2*tf^3);
a4=(30*q_0 - 30*q_F + (14*q_dot_F + 16*q_dot_0)*tf +(3*q_2dot_0 - 2*q_2dot_F)*tf^2)/(2*tf^4);
a5=(12*q_F - 12*q_0 - (6*q_dot_F + 6*q_dot_0)*tf -(q_2dot_0 - q_2dot_F)*tf^2)/(2*tf^5);
%% Quy hoach quy dao bac 5
while(1)
    phantram= a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    
    if(LastCycle==1)
        break;
    end
    t = t + ts;
    pause(ts);
         t=tf;
    if(t>tf)
         LastCycle = 1;
    end
end

sprintf("Simulation Done")

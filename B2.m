clc; clear all; close all;

t=0;
ts = 0.01;
LastCycle=0;
x_a   = 40; %[mm]
y_a   = 40;
z_a   = 40;  

x_b   = 100; %[mm]
y_b   = 100;
z_b   = 100;

AB_MD   = sqrt(((x_b-x_a)^2)+((y_b-y_a)^2)+((z_b-z_a)^2));

v_qd  = 100; %% [mm]/[s]

tf = AB_MD/v_qd;

%% Quy hoach quy dao bac 5
x_0           = 10;            %% trang thai cua diem dau
x_dot_0    = -100;
x_2dot_0  = 10;

x_F           = 100;        %% trang thai cua diem cuoi
x_dot_F    = -100;
x_2dot_F  = 10;

a0=x_0;
a1=x_dot_0;
a2=x_2dot_0/2;
a3=(20*x_F - 20*x_0-(8*x_dot_F + 12*x_dot_0)*tf -(3*x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^3);
a4=(30*x_0 - 30*x_F + (14*x_dot_F + 16*x_dot_0)*tf +(3*x_2dot_0 - 2*x_2dot_F)*tf^2)/(2*tf^4);
a5=(12*x_F - 12*x_0 - (6*x_dot_F + 6*x_dot_0)*tf -(x_2dot_0 - x_2dot_F)*tf^2)/(2*tf^5);

phantram= a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;

plot3([x_a x_b],[y_a y_b],[z_a z_b],'-o','LineWidth',2,'MarkerEdgeColor','b')
grid on;
hold on;

while(1)
    
    x_phantram =  a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
    v_phantram =  a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
    a_phantram =  2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
    
    x_TCP   = x_a + (x_b - x_a)*x_phantram/100;
    y_TCP   = y_a + (y_b - y_a)*x_phantram/100;
    z_TCP   = z_a + (z_b - z_a)*x_phantram/100;
    
    subplot(1,4,1);
    xlim([-100 150]);
    ylim([-100 150]);
    zlim([-100   150]);
    plot3(x_TCP,y_TCP,z_TCP,'-o','LineWidth',1,'MarkerEdgeColor','r')
    grid on;
    hold on;
    
    subplot(1,4,2);
    xlim([0 tf]);
    ylim([-20 150]);
    plot(t,x_phantram,'-o','LineWidth',1,'MarkerEdgeColor','b')
    grid on;
    hold on;
    
    subplot(1,4,3);
    xlim([0 tf]);
    plot(t,v_phantram,'-o','LineWidth',1,'MarkerEdgeColor','b')
    grid on;
    hold on;
    
    subplot(1,4,4);
    xlim([0 tf]);
    plot(t,a_phantram,'-o','LineWidth',1,'MarkerEdgeColor','b')
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
    pause(ts);
end

sprintf("Simulation Done")
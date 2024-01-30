clear all; clc;
t   = 0;
ts = 0.01;
LastCycle   =0;
%% khai báo các biến cần thiết ban đầu
mode = 0;
%% tọa độ điểm đầu
x_a   = 0; %[mm]
y_a   = 0;
z_a   = 0;
%% tọa độ điểm cuối
x_b   = 1000; %[mm]
y_b   = 0;
z_b   = 0;
%% nội quy quản đường đi được
 
AB_MD   = sqrt(((x_b-x_a)^2)+((y_b-y_a)^2)+((z_b-z_a)^2));
 
%% khai báo giá trị ban đầu
t               = 0;
x_0          = 0;                       %% trạng thái điểm đầu
x_dot_0   = 0;            
x_2dot_0 = 0;
v_0          = x_dot_0;            %% vận tốc điểm đầu
a_0          = x_2dot_0;          %% gia tốc điểm đầu

x_F             = 100;                 %% trạng thái điểm cuối
x_dot_F      = 0;           
x_2dot_F    = 0;
v_f              = x_dot_F;             %% vận tốc điểm cuối   
a_f              = x_2dot_F;           %% gia tốc điểm cuối

v_qd           = 500;                    %% [mm]/[s] %% vận tốc quỹ đạo             
t_tb             = 0.1;                      %% thời gian tăng giảm tốc
acc_max     = 10000;                %% [mm/s] gia tốc max của phần cứng
t_min          = v_qd / acc_max; %% [mm/s] thời gian tăng tốc thấp nhất

if(t_tb < t_min)          %% giới hạn thời gian tăng tốc
    t_tb = t_min;
end
a_giutoc = 0;             %% gia tốc trong quá trình giữ vận tốc
j_giutoc = 0;             %% jerk trong quá tình giữ vận tốc
phan_tram_j = 0.2;        %% nhập phần trăm tăng giảm tốc của gia tốc, dựa vào thời gian tăng giảm tốc t_tb

%% nội suy gia tốc tăng giảm từ vận tốc quỹ đạo và phần trăm thời tian tăng giảm gia tốc
a_up = (2*v_0 + 2*a_0*t_tb + j_giutoc*t_tb^2 - 2*j_giutoc*phan_tram_j*t_tb^2 - 2*a_0*phan_tram_j*t_tb - 2 * v_qd)/(2*t_tb*(phan_tram_j - 1));     %% nội quy gia tốc tăng
a_dw =  - a_up;           %% nội quy gia tốc giảm
 
%% nội suy các giá trị 
S_up =  v_0 * t_tb + 0.5 * a_up * t_tb * t_tb;   %% nội suy quãng đường đi được trong thời gian gia tốc tăng lên a_up
S_dw =  v_qd * t_tb + 0.5 * a_dw * t_tb * t_tb;  %% nội suy quãng đường đi được trong thời gian gia tốc giảm về 0
v_qd_max = x_F/(0.5*t_tb + t_tb - 0.5*t_tb);   %% nội suy vận tốc quỹ đạo max theo thời gian tăng giảm cố định
S_giutoc = x_F - (S_up + S_dw);                %% nội suy quãng đường trong thời gian giữ gia tốc
t_giutoc = S_giutoc / v_qd;                      %% nội suy thời gian giữ gia tốc
tf = t_giutoc + t_tb + t_tb;                     %% nội suy thời gian kết thúc quỹ đạo

%% giới hạn vân tốc
if (v_qd > v_qd_max )                            %% nếu vận tốc quỹ đạo lớn hơn vận tốc Max thì
    v_qd = v_qd_max;                             %% vận tốc quỹ đạo bằng vận tốc max
    a_up = (v_qd - v_0) / t_tb;                  %% và tính toán lại các giá trị mà có liên quan đến Vân tốc quỹ đạo
    a_dw = (v_f - v_qd) / t_tb;
    a_giutoc = 0;
    S_up =  v_0 * t_tb + 0.5 * a_up * t_tb * t_tb;
    S_dw =  v_qd * t_tb + 0.5 * a_dw * t_tb * t_tb;
    S_giutoc = x_F - (S_up + S_dw);
    t_giutoc = S_giutoc / v_qd;
    tf = t_giutoc + t_tb + t_tb;
end
%% Giới hạn thời gian tăng giảm tốc
if (t_tb > tf/2)                                 %% nếu thời gian tăng giảm tốc lớn hơn 1/2 thơi gian tổng quảng đường  (tf)   
    t_tb = tf/2;                                 %% thì thời gian tăng giảm tốc bằng thời gian 1/2 thời gian quản đường (tf)
    a_up = (v_qd - v_0) / t_tb;
    a_dw = (v_f - v_qd) / t_tb;
    a_giutoc = 0;
    S_up =  v_0 * t_tb + 0.5 * a_up * t_tb * t_tb;
    S_dw =  v_qd * t_tb + 0.5 * a_dw * t_tb * t_tb;
    S_giutoc = x_F - (S_up + S_dw);
    t_giutoc = S_giutoc / v_qd;
    tf = t_giutoc + t_tb + t_tb;
    end
%% jerk (độ giật gia tốc) (a_dot)
t_j_max = t_tb * phan_tram_j;                   %% nội suy thời gian tăng giảm j từ phần trăm j tăng giảm và thời gian tăng giảm tốc
t_j_max_giu = t_tb - (t_j_max + t_j_max);       %% nội suy thời gian giữ j
j_up_up = a_up / t_j_max;                       %% nội j tăng trong quá trình gia tốc tăng (a_up)
j_up_dw = - a_up / t_j_max;                     %% nội j giảm trong quá trình gia tốc tăng (a_up)
j_dw_up = - a_dw / t_j_max;                     %% nội j giảm trong quá trình gia tốc giảm (a_dw)
j_dw_dw = a_dw / t_j_max;                       %% nội j giảm trong quá trình gia tốc giảm (a_dw)
%% chia thời gian chuyển giai đoạn
t_gd1 = t_j_max;
t_gd2 = t_tb - t_j_max;
t_gd3 = t_tb;
t_gd4 = t_giutoc + t_tb;
t_gd5 = t_gd4 + t_j_max;
t_gd6 = tf - t_j_max;
t_gd7 = tf;
%% Nội suy S, V, A, các giai doạn
% Công thức nội suy các giai đoạn sẽ giống với các công thức https://physics.info/kinematics-calculus/
% trạng thái ở giai đoạn 1
v_gd1 = v_0 + (a_0 * (t_tb * phan_tram_j)) + (0.5 * (a_up / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j));
a_gd1 = a_0 + (a_up / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j);
S_gd1 = x_0 + (v_0 * (t_tb * phan_tram_j)) + (0.5 * a_0 * (t_tb * phan_tram_j) * (t_tb * phan_tram_j)) + ((1/6) * (a_up / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j)) ;
% trạng thái ở giai đoạn 2
v_gd2 = v_gd1 + (a_gd1 * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) + (0.5 * j_giutoc * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))));
a_gd2 = a_up + j_giutoc * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)));
S_gd2 = S_gd1 + (v_gd1 * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) + (0.5 * a_gd1 * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) + ((1/6) * j_giutoc * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) ;
% trạng thái ở giai đoạn 3
v_gd3 = v_gd2 + (a_gd2 * (t_tb * phan_tram_j)) + (0.5 * (-a_up / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j));
a_gd3 = a_up + (-a_up / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j);
S_gd3 = S_gd2 + (v_gd2 * (t_tb * phan_tram_j)) + (0.5 * a_gd2 * (t_tb * phan_tram_j) * (t_tb * phan_tram_j)) + ((1/6) * (-a_up / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j)) ;
% trạng thái ở giai đoạn 4
v_gd4 = v_gd3 + (a_gd3 * t_giutoc) + (0.5 * j_giutoc * t_giutoc * t_giutoc) ;
a_gd4 = a_giutoc + j_giutoc * t_giutoc ;
S_gd4 = S_gd3 + (v_gd3 * t_giutoc) + (0.5 * a_gd3 * t_giutoc * t_giutoc) + ((1/6) * j_giutoc * t_giutoc * t_giutoc * t_giutoc) ;
% trạng thái ở giai đoạn 5
v_gd5 = v_gd4 + (a_gd4 * (t_tb * phan_tram_j)) + (0.5 * (a_dw / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j)) ;
a_gd5 = a_giutoc + (a_dw / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j) ;
S_gd5 = S_gd4 + (v_gd4 * (t_tb * phan_tram_j)) + (0.5 * a_gd4 * (t_tb * phan_tram_j) * (t_tb * phan_tram_j)) + ((1/6) * (a_dw / (t_tb * phan_tram_j)) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j) * (t_tb * phan_tram_j)) ;
% trạng thái ở giai đoạn 6
v_gd6 = v_gd5 + (a_gd5 * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) + (0.5 * j_giutoc * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) ;
a_gd6 = a_dw + j_giutoc * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) ;
S_gd6 = S_gd5 + (v_gd5 * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) + (0.5 * a_gd5 * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j)))) + ((1/6) * j_giutoc * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))) * (t_tb - ((t_tb * phan_tram_j) + (t_tb * phan_tram_j))));
% trạng thái ở giai đoạn 7
a_gd7 = a_dw + j_dw_up * t_gd7 ;
v_gd7 = v_gd6 + (a_gd6 * t_gd7) + (0.5 * j_dw_up * t_gd7 * t_gd7) ;
S_gd7 = S_gd6 + (v_gd6 * t_gd7) + (0.5 * a_gd6 * t_gd7 * t_gd7) + ((1/6) * j_dw_up * t_gd7 * t_gd7 * t_gd7) ;
%% Xử lý trong While
while (true)
    % bởi vì code sẽ dùng switch case nên ta phải chia điều kiện các giai
    % đoạn để chạy switch case
    
    if (t <= t_gd1)                                         %% giai đoạn 1  
        t1 = t;
        j = j_up_up ;
        a = a_0 + j_up_up * t1 ;
        v = v_0 + (a_0 * t1) + (0.5 * j_up_up * t1 * t1);
        S = x_0 + (v_0 * t1) + (0.5 * a_0 * t1 * t1) + ((1/6) * j_up_up * t1 * t1 * t1) ;
    else
        if (t <= t_gd2 )                                    %% giai đoạn 2
            t2 = t - t_j_max ;
             j = j_giutoc ;
            a = a_up + j_giutoc * t2 ;
            v = v_gd1 + (a_gd1 * t2) + (0.5 * j_giutoc * t2 * t2);
            S = S_gd1 + (v_gd1 * t2) + (0.5 * a_gd1 * t2 * t2) + ((1/6) * j_giutoc * t2 * t2 * t2) ;
        else
            if (t <= t_gd3 )                                %% giai đoạn 3
                t3 = t - t_j_max - t_j_max_giu ;
                 j = j_up_dw ;
                a = a_up + j_up_dw * t3;
                v = v_gd2 + (a_gd2 * t3) + (0.5 * j_up_dw * t3 * t3);
                S = S_gd2 + (v_gd2 * t3) + (0.5 * a_gd2 * t3 * t3) + ((1/6) * j_up_dw * t3 * t3 * t3) ;
            else
                if (t <= t_gd4 )                            %% giai đoạn 4
                    t4 = t - t_tb;
                     j = j_giutoc ;
                    a = a_giutoc + j_giutoc * t4 ;
                    v = v_gd3 + (a_gd3 * t4) + (0.5 * j_giutoc * t4 * t4) ;
                    S = S_gd3 + (v_gd3 * t4) + (0.5 * a_gd3 * t4 * t4) + ((1/6) * j_giutoc * t4 * t4 * t4) ;
                else
                    if (t <= t_gd5 )                        %% giai đoạn 5
                        t5 = t - t_tb - t_giutoc;
                        j = j_dw_dw ;
                        a = a_giutoc + j_dw_dw * t5 ;
                        v = v_gd4 + (a_gd4 * t5) + (0.5 * j_dw_dw * t5 * t5) ;
                        S = S_gd4 + (v_gd4 * t5) + (0.5 * a_gd4 * t5 * t5) + ((1/6) * j_dw_dw * t5 * t5 * t5) ;
                    else
                        if (t <= t_gd6 )                    %% giai đoạn 6
                            t6 = t - t_tb - t_giutoc - t_j_max;
                            j = j_giutoc ;
                            a = a_dw + j_giutoc * t6 ;
                            v = v_gd5 + (a_gd5 * t6) + (0.5 * j_giutoc * t6 * t6) ;
                            S = S_gd5 + (v_gd5 * t6) + (0.5 * a_gd5 * t6 * t6) + ((1/6) * j_giutoc * t6 * t6 * t6) ; 
                        else
                            if (t <= t_gd7 )                %% giai đoạn 7
                                t7 = t - t_tb - t_giutoc - t_j_max - t_j_max_giu;
                                 j = j_dw_up ;
                                a = a_dw + j_dw_up * t7 ;
                                v = v_gd6 + (a_gd6 * t7) + (0.5 * j_dw_up * t7 * t7) ;
                                S = S_gd6 + (v_gd6 * t7) + (0.5 * a_gd6 * t7 * t7) + ((1/6) * j_dw_up * t7 * t7 * t7) ; 
                            end
                        end
                    end
                end
            end
        end
    end
            % nội suy tọa độ của đầu TCP
            P_x_TCP   = x_a + (x_b - x_a)*S/100;
            P_y_TCP   = y_a + (y_b - y_a)*S/100;
            P_z_TCP   = z_a + (z_b - z_a)*S/100;
            
            % vẽ đồ thị 
            subplot(5,1,1);
            plot(t,j,'-o','LineWidth',1,'MarkerEdgeColor','b')
            xlim([0 tf]);
            title('J');
            grid on;
            hold on;
             
            subplot(5,1,2);
            plot(t,a,'-o','LineWidth',1,'MarkerEdgeColor','b');
            xlim([0 tf]);
            title('A');
            grid on;
            hold on;
            
            subplot(5,1,3);
            plot(t,v,'-o','LineWidth',1,'MarkerEdgeColor','b')
            xlim([0 tf]);
            title('V');
            grid on;
            hold on;
            
            subplot(5,1,4);
            plot(t,S,'-o','LineWidth',1,'MarkerEdgeColor','b')
            xlim([0 tf]);
            title('S');
            grid on;
            hold on;
            
            subplot(5,1,5);
            plot3(P_x_TCP,P_y_TCP,P_z_TCP,'-o','LineWidth',1,'MarkerEdgeColor','b')
%             xlim([0 400]);
%             ylim([0 400]);
%             zlim([0 400]);
            view(3);
            axis equal;
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
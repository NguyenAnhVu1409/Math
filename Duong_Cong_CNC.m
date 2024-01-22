clc; clear all; close all;
LastCycle=0;
x_a   = 40;
y_a   = 0;
z_a   = 0;  
x_b   = 0;
y_b   = 40;
z_b   = 0;
R     = -30;

v_qd  = 5000; %% [mm]/[s]

pt_x  = 0;
pt_y  = 0;              %% This is parameter of normal line
pt_z  = 1;

chieu_cung=0;    %% 1 is anti clockwise base on normal Line and vice versa

i_base  = [1;0;0];
j_base  = [0;1;0];
k_base  = [0;0;1];

%%tim toa do tam O
pt      = [pt_x ; pt_y ; pt_z];
pt_MD   = sqrt((pt_x^2)+(pt_y^2)+(pt_z^2));
AB_base = [x_b-x_a ; y_b-y_a ; z_b - z_a];
AB_MD   = sqrt(((x_b-x_a)^2)+((y_b-y_a)^2)+((z_b-z_a)^2));


if((abs(R)<(AB_MD/2))||((pt_x==0)&&(pt_y==0)&&(pt_z==0)))
    error_wp=1;
else
    error_wp=0;
end



k_A_base = pt/pt_MD;
i_A_base = AB_base/AB_MD;
j_A_base = cross(k_A_base,i_A_base);

R_Base_A   = [  i_base'*i_A_base  i_base'*j_A_base  i_base'*k_A_base;
                j_base'*i_A_base  j_base'*j_A_base  j_base'*k_A_base;
                k_base'*i_A_base  k_base'*j_A_base  k_base'*k_A_base];
H_Base_A   = [  R_Base_A(1,1) R_Base_A(1,2) R_Base_A(1,3) x_a;
                R_Base_A(2,1) R_Base_A(2,2) R_Base_A(2,3) y_a;
                R_Base_A(3,1) R_Base_A(3,2) R_Base_A(3,3) z_a;
                0             0             0             1  ];

if(error_wp==0)
   if    (chieu_cung==1)&&(R>0)
            phi   = -acosd((AB_MD)/(abs(2*R))); %%%%sind giá trị độ
            H_A_O = [ cosd(-180-phi) -sind(-180-phi)      0    AB_MD/2   ;
                      sind(-180-phi)  cosd(-180-phi)      0    -sind(phi)*R;
                      0               0                   1    0         ;
                      0               0                   0    1         ];
   elseif(chieu_cung==1)&&(R<0)
            phi   = acosd((AB_MD)/(abs(2*R))); %%%%sind giá trị độ
            H_A_O = [ cosd(-180-phi) -sind(-180-phi)      0    AB_MD/2   ;
                      sind(-180-phi)  cosd(-180-phi)      0    sind(phi)*R;
                      0               0                   1    0         ;
                      0               0                   0    1         ];

   elseif(chieu_cung==0)&&(R>0)
            phi   = acosd((AB_MD)/(abs(2*R)));
            H_A_O = [ cosd(-180-phi) -sind(-180-phi)      0    AB_MD/2   ;
                      sind(-180-phi)  cosd(-180-phi)      0    -sind(phi)*R;
                      0               0                   1    0         ;
                      0               0                   0    1         ];
   elseif(chieu_cung==0)&&(R<0)
            phi   = -acosd((AB_MD)/(abs(2*R)));
            H_A_O = [ cosd(-180-phi) -sind(-180-phi)      0    AB_MD/2   ;
                      sind(-180-phi)  cosd(-180-phi)      0    sind(phi)*R;
                      0               0                   1    0         ;
                      0               0                   0    1         ];
   else
            phi   = 0;
   end 
end            
H_Base_O = H_Base_A*H_A_O;

x_o = H_Base_O(1,4);
y_o = H_Base_O(2,4);
z_o = H_Base_O(3,4);

O_Base = [x_o;y_o;z_o]

OA_MD = sqrt((x_a-x_o)*(x_a-x_o)+(y_a-y_o)*(y_a-y_o)+(z_a-z_o)*(z_a-z_o));
OB_MD = sqrt((x_b-x_o)*(x_b-x_o)+(y_b-y_o)*(y_b-y_o)+(z_b-z_o)*(z_b-z_o)); 
Tu   = OA_MD*OA_MD+OB_MD*OB_MD-AB_MD*AB_MD;
Mau   = 2*OA_MD*OB_MD;


if      (Mau>0)&&(chieu_cung==1)&&(R>0)
            AOB  = acosd(Tu/Mau);
elseif  (Mau>0)&&(chieu_cung==1)&&(R<0)
            AOB  = (360)- acosd(Tu/Mau);            
elseif  (Mau>0)&&(chieu_cung==0)&&(R>0)
            AOB  = -acosd(Tu/Mau);
elseif  (Mau>0)&&(chieu_cung==0)&&(R<0)
            AOB  = -360+acosd(Tu/Mau);
else
            AOB  = 0;
end

%%

cung_AB = ((180*abs(R)*abs(AOB))/180);

tf_o = cung_AB/v_qd;
tb   = 0.1;    
tf   = tb + tf_o;
ts   = 0.01;

q_o  = 0;
q_f  = cung_AB;
vqd  = (q_f-q_o)/(tf-tb);
a    = vqd/tb;
t    = 0;

plot3([x_a x_b],[y_a y_b],[z_a z_b],'-o','LineWidth',2,'MarkerEdgeColor','b')
grid on;
hold on;
while(1)
    
    if((t>=0)&&(t<=tb))
            cung_AB_s = (0.5*a*t*t);
    elseif((t>=tb)&&(t<=(tf-tb)))
            cung_AB_s=((0.5*a*tb*tb)+(vqd*(t-tb)));
    elseif((t>=(tf-tb))&&(t<=tf))
            cung_AB_s = (cung_AB+(a*tf*t)-(0.5*a*tf*tf)-(0.5*a*t*t));
    else
            cung_AB_s=0;
    end


    if(chieu_cung==1)
        AOB_s = (cung_AB_s*360)/(360*abs(R));
    else
        AOB_s = -(cung_AB_s*360)/(360*abs(R));
    end 

    x_EE_O = (abs(R))*cosd(AOB_s);
    y_EE_O = (abs(R))*sind(AOB_s);
    z_EE_O = 0;
    EE_Base_TT = H_Base_O*[ x_EE_O; y_EE_O; z_EE_O; 1];
    xlim([-100 100]);
    ylim([-100 100]);
    zlim([-1 1]);
    plot3(EE_Base_TT(1,1),EE_Base_TT(2,1),EE_Base_TT(3,1),'-o','LineWidth',1,'MarkerEdgeColor','r')
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
    
    pause(0.0000001);
end
EE_Base_TT = H_Base_O*[ x_EE_O; y_EE_O; z_EE_O; 1]
sprintf("Simulation Done")
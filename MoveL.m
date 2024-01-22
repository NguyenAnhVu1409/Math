function MoveL(x_b,y_b,z_b,v_qd)
    %% Lay bien khop hien tai
    theta1=getappdata(0,'theta1_old');
    theta2=getappdata(0,'theta2_old');
    d=getappdata(0,'d_old');
    Error_old=getappdata(0,'Error_old');
    WP=getappdata(0,'WP');
    WP=WP+1;
    setappdata(0, 'WP', WP);
    if(Error_old==1)
        return;
    end
    %% DHT de lay vi tri dau cong tac hien tai
    [x_a,y_a,z_a]=DHT(theta1,theta2,d);
    C=[0;0;0];
    A=[x_a;y_a;z_a];
    B=[x_b;y_b;z_b];
    R=0;
    %% Kiem tra Diem nhap da phu hop chua
    Error=CheckPoint_ABC(A,B,C,0,R);
    if(Error==1) 
        sprintf('Bị lỗi ở quá trình thứ %d',WP)
        setappdata(0, 'Error_old', Error); 
        return;
    else   
        sprintf('Quỹ đạo của lệnh MoveL ở quá trình thứ %d thỏa',WP)
    end

    ts=0.05;
    gamma=2;
    Percent_tt=0.2;
    
    count=0;
    l0=185;    
    l1=87;    %272-185
    l2=200;
    l3=200;
    l4=116;   %%122-6
    t=0;
    theta1_pre=theta1;
    theta2_pre=theta2;
    d_pre=d;
    Vt_pre=0;
    Px_pre=x_a;
    Py_pre=y_a;
    Pz_pre=z_a;
  
    BA   = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2);
    tf_o = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2)/v_qd;
    tf   = (1+Percent_tt)*tf_o;
    q_o  = 0;                 %% trang thai cua diem dau
    q_f  = BA;               %% trang thai cua diem cuoi
    tb   = Percent_tt*tf_o;
    vqd  = (q_o-q_f)/(tb-tf);
    a    = vqd/tb;
    for i=1:1:((tf/ts)+2)
        st=LSPB(tb,tf,q_o,q_f,vqd,a,t);

        Px = x_a + st*(x_b-x_a)/BA;
        Py = y_a + st*(y_b-y_a)/BA;
        Pz = z_a + st*(z_b-z_a)/BA;
        
        v_x_qhqd=(Px-Px_pre)/ts;
        v_y_qhqd=(Py-Py_pre)/ts;
        v_z_qhqd=(Pz-Pz_pre)/ts;
        Px_pre=Px;
        Py_pre=Py;
        Pz_pre=Pz;

        [theta1,theta2,d]=DHN(Px,Py,Pz,theta1_pre);

        EE_x=  l3*cos(theta1 + theta2) + l2*cos(theta1);
        EE_y=  l3*sin(theta1 + theta2) + l2*sin(theta1);
        EE_z=  l0 - d + l1 -l4;
        truc_2_x=200*cos(theta1);
        truc_2_y=200*sin(theta1);
        truc_2_z=272;    
        
        theta1_dot=(theta1-theta1_pre)/ts;
        theta2_dot=(theta2-theta2_pre)/ts;
        theta3_dot=0;
        d_dot=(d-d_pre)/ts;
        D=[theta1_dot; theta2_dot; d_dot; theta3_dot];

        theta1_pre=theta1;
        theta2_pre=theta2;
        d_pre=d;
        Vt=JACOBI_Thuan(theta1,theta2,D);
        
        At=(Vt-Vt_pre)/ts;
        Vt_pre=Vt;
%         %% Gia lap va cham
%         if((t>0.5)&&(t<0.6))
%             theta1_dot=theta1_dot+theta1_dot*0.05;
%             theta2_dot=theta2_dot+theta2_dot*0.05;
%             d_dot=d_dot+d_dot*0.5;
%         end
    
        %% Xu ly Jacobi nghich
        [theta1_dot_estimation,    theta2_dot_estimation,    d_dot_estimation]     =   Jacobi_Nghich(v_x_qhqd,v_y_qhqd,v_z_qhqd,theta1,theta2);
        %% Check dieu kien va dap
            if((t>(20*ts))&&(t<(tf-ts)))
                if(theta1_dot>30*pi/180)
                    if((theta1_dot>=(theta1_dot_estimation+theta1_dot_estimation*gamma)) || (theta1_dot<=(theta1_dot_estimation-theta1_dot_estimation*gamma)))
                        sprintf('Warning The joint 1 has impact - Please check out the system ')
                        Error=1;
                        setappdata(0, 'Error_old', Error);
                        return;
                    end
                end
                if(theta1_dot<-30*pi/180)
                    if((theta1_dot<=(theta1_dot_estimation+theta1_dot_estimation*gamma)) || (theta1_dot>=(theta1_dot_estimation-theta1_dot_estimation*gamma)))
                        sprintf('Warning The joint 1 has impact - Please check out the system ')
                        Error=1;
                        setappdata(0, 'Error_old', Error);
                        return;
                    end
                end
                
                if(theta2_dot>30*pi/180)
                    if((theta2_dot>=(theta2_dot_estimation+theta2_dot_estimation*gamma)) || (theta2_dot<=(theta2_dot_estimation-theta2_dot_estimation*gamma)))
                        sprintf('Warning The joint 2 has impact - Please check out the system ')
                        Error=1;
                        setappdata(0, 'Error_old', Error);
                        return;
                    end
                end
                if(theta2_dot<-30*pi/180)
                    if((theta2_dot<=(theta2_dot_estimation+theta2_dot_estimation*gamma)) || (theta2_dot>=(theta2_dot_estimation-theta2_dot_estimation*gamma)))
                        sprintf('Warning The joint 2 has impact - Please check out the system ')
                        Error=1;
                        setappdata(0, 'Error_old', Error);
                        return;
                    end
                end
                if(d_dot>30)
                    if((d_dot>=(d_dot_estimation+d_dot_estimation*gamma)) || (d_dot<=(d_dot_estimation-d_dot_estimation*gamma)))
                        sprintf('Warning The joint 3 has impact - Please check out the system ')
                        Error=1;
                        setappdata(0, 'Error_old', Error);
                        return;
                    end
                end
                if(d_dot<-30)
                    if((d_dot<=(d_dot_estimation+d_dot_estimation*gamma)) || (d_dot>=(d_dot_estimation-d_dot_estimation*gamma)))
                        sprintf('Warning The joint 3 has impact - Please check out the system ')
                        Error=1;
                        setappdata(0, 'Error_old', Error);
                        return;
                    end
                end
            end
        %% Ve do thi mo phong
        subplot(1,4,1);
        plot3([0,0 , truc_2_x,EE_x, EE_x],[0,0 , truc_2_y,EE_y, EE_y],[0,272 , truc_2_z ,EE_z+d+l4, EE_z],'o-','LineWidth',2,'MarkerEdgeColor','b');
        grid on;
        hold on;
        xlim([-500 500]);
        ylim([-500 500]);
        zlim([0 300]);
        
        plot3([x_a,x_b],[y_a,y_b],[z_a,z_b],'o-','LineWidth',2,'MarkerEdgeColor','r');
        hold off;
        
        subplot(1,4,2);
        plot3(EE_x,EE_y,EE_z,'.','LineWidth',20,'MarkerEdgeColor','r');
        hold on;
        grid on;
        xlim([-500 500]);
        ylim([-500 500]);
        zlim([0 300]);

        subplot(1,4,3);
        AB = sqrt((x_b-x_a)^2+(y_b-y_a)^2+(z_b-z_a)^2);
        plot(t,st*100/AB,'.','LineWidth',20,'MarkerEdgeColor','b')
        xlim([-1   tf*1.5]);
        ylim([-1 120]);
        hold on;
        grid on;
        
        subplot(1,4,4);
        plot(t,Vt,'.','LineWidth',20,'MarkerEdgeColor','b')
        xlim([-1 tf*1.5]);
        ylim([-1 800]);
        hold on;
        grid on;

%         subplot(1,5,5);
%         plot(t,At,'.','LineWidth',20,'MarkerEdgeColor','b')
%         xlim([-1 tf*1.5]);
%         ylim([-500 500]);
%         hold on;
%         grid on;
        %% Cong bien lay mau ts de mo phong roi rac hoa robot
        pause(0.0001);
        t=t+ts;  
        %% Xu ly bien truoc khi dung quy dao nay
        if(count~=0)
            count=0;
%             WP=WP+1;
            setappdata(0, 'theta1_old', theta1);
            setappdata(0, 'theta2_old', theta2);
            setappdata(0, 'd_old', d);
            setappdata(0, 'Error_old', Error);
            subplot(1,4,2);
            hold off;
            subplot(1,4,3);
            hold off;
            subplot(1,4,4);
            hold off;
%             subplot(1,5,5);
%             hold off;
            break;
        end
        %% Check dieu kien dung
        if(t>tf)
            setappdata(0, 'theta1_old', theta1);
            setappdata(0, 'theta2_old', theta2);
            setappdata(0, 'd_old', d);
            setappdata(0, 'Error_old', Error);
             t=tf;
             count=count+1;
        end
        %% Check dieu kien de kip nhan nut phong to
        if(WP==0)
            if(t<ts*3)
                pause(2);
            end
        end
    end
end

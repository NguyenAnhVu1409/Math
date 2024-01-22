function MoveP(x_b,y_b,z_b,x_c,y_c,z_c,R,v_qd)
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
    C=[x_c;y_c;z_c];
    A=[x_a;y_a;z_a];
    B=[x_b;y_b;z_b];
    %% Kiem tra Diem nhap da phu hop chua
    Error=CheckPoint_ABC(A,B,C,3,R);
    if(Error==1) 
        sprintf('Bị lỗi ở quá trình thứ %d',WP)
        setappdata(0, 'Error_old', Error); 
        return;
    else   
        sprintf('Quỹ đạo của lệnh MoveP ở quá trình thứ %d thỏa',WP)
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
        %% tien xu ly Move P
        CA=A-C;
        CB=C-B;
        n=cross(CA,CB);
        CA_md=sqrt((x_a-x_c)*(x_a-x_c)+(y_a-y_c)*(y_a-y_c)+(z_a-z_c)*(z_a-z_c));
        CB_md=sqrt((x_b-x_c)*(x_b-x_c)+(y_b-y_c)*(y_b-y_c)+(z_b-z_c)*(z_b-z_c));
        if(CA_md<=CB_md)
            R_max=fix(CA_md)
            if(R>CA_md)
                R=fix(CA_md);
            end
        end
        if(CB_md<CA_md)
            R_max=fix(CB_md)
            if(R>CB_md)
                R=fix(CB_md);
            end
        end
        V=C+(A-C)*(R/CA_md);
        x_v=V(1,1);
        y_v=V(2,1);
        z_v=V(3,1);
        Q=C+(B-C)*(R/CB_md);
        x_q=Q(1,1);
        y_q=Q(2,1);
        z_q=Q(3,1);
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
        tf=(1+Percent_tt)*(tf_1+tf_2+tf_3);
        q_o = 0;                %% trang thai cua diem dau
        q_f = 100;               %% trang thai cua diem cuoi
        tb    = Percent_tt*tf_o;
        vqd   = (q_o - q_f)/(tb - tf);
        a     = vqd/tb;
        
        %% Mo Phong MoveP
        for i=1:1:((tf/ts)+3)
            if((t>=0)&&(t<=tb))
                phantram = q_o+ 0.5*a*t*t;
            end
            if((t>=tb)&&(t<=(tf-tb)))
                phantram=((q_o+q_f-vqd*tf)/2)+vqd*t;
            end
            if((t>=(tf-tb))&&(t<=tf))
                phantram = q_f-0.5*a*tf*tf+a*tf*t-0.5*a*t*t;
            end

            if(phantram<phantram1)
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
            if(phantram>phantram2&&phantram<=100)
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

            plot3([x_b,x_c,x_a],[y_b,y_c,y_a],[z_b,z_c,z_a],'o-','LineWidth',2,'MarkerEdgeColor','r');
            hold off;

            subplot(1,4,2);
            plot3(EE_x,EE_y,EE_z,'.','LineWidth',20,'MarkerEdgeColor','r');
            hold on;
            grid on;
            xlim([-500 500]);
            ylim([-500 500]);
            zlim([0 300]);

            subplot(1,4,3);
            plot(t,phantram,'.','LineWidth',20,'MarkerEdgeColor','b')
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

%             subplot(1,5,5);
%             plot(t,At,'.','LineWidth',20,'MarkerEdgeColor','b')
%             xlim([-1 tf*1.5]);
%             ylim([-500 500]);
%             hold on;
%             grid on;
            %% Cong bien lay mau ts de mo phong roi rac hoa robot
            pause(0.0001);
            t=t+ts;  
            %% Xu ly bien truoc khi dung quy dao nay
            if(count~=0)
                count=0;
%                 WP=WP+1;
                setappdata(0, 'theta1_old', theta1);
                setappdata(0, 'theta2_old', theta2);
                setappdata(0, 'd_old', d);
                setappdata(0, 'Error_old', Error);
                setappdata(0, 'WP', WP);
                subplot(1,4,2);
                hold off;
                subplot(1,4,3);
                hold off;
                subplot(1,4,4);
                hold off;
%                 subplot(1,5,5);
%                 hold off;
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

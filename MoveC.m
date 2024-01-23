function MoveC(x_b,y_b,z_b,x_c,y_c,z_c,v_qd)
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
    R=0;
    %% Kiem tra Diem nhap da phu hop chua
    Error=CheckPoint_ABC(A,B,C,4,R);
    if(Error==1) 
        sprintf('Bị lỗi ở quá trình thứ %d',WP)
        setappdata(0, 'Error_old', Error); 
        return;
    else   
        sprintf('Quỹ đạo của lệnh MoveC ở quá trình thứ %d thỏa',WP)
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
    
    x_o =((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(y_a*z_b - y_b*z_a - y_a*z_c + y_c*z_a + y_b*z_c - y_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_b*y_a^2 - x_a*y_c^2 - x_c*y_a^2 + x_b*y_c^2 + x_b*z_a^2 - x_a*z_c^2 - x_c*z_a^2 + x_b*z_c^2 - x_a*y_a*y_b + x_a*y_a*y_c + x_a*y_b*y_c - 2*x_b*y_a*y_c + x_c*y_a*y_b + x_c*y_a*y_c - x_c*y_b*y_c - x_a*z_a*z_b + x_a*z_a*z_c + x_a*z_b*z_c - 2*x_b*z_a*z_c + x_c*z_a*z_b + x_c*z_a*z_c - x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_a*y_b^2 + x_a*y_c^2 - x_b*y_c^2 - x_c*y_b^2 + x_a*z_b^2 + x_a*z_c^2 - x_b*z_c^2 - x_c*z_b^2 - x_b*y_a*y_b - 2*x_a*y_b*y_c + x_b*y_a*y_c + x_c*y_a*y_b + x_b*y_b*y_c - x_c*y_a*y_c + x_c*y_b*y_c - x_b*z_a*z_b - 2*x_a*z_b*z_c + x_b*z_a*z_c + x_c*z_a*z_b + x_b*z_b*z_c - x_c*z_a*z_c + x_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
    y_o =(((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_a^2*y_b - x_a^2*y_c - x_c^2*y_a + x_c^2*y_b + y_b*z_a^2 - y_a*z_c^2 - y_c*z_a^2 + y_b*z_c^2 - x_a*x_b*y_a + x_a*x_c*y_a + x_a*x_b*y_c - 2*x_a*x_c*y_b + x_b*x_c*y_a + x_a*x_c*y_c - x_b*x_c*y_c - y_a*z_a*z_b + y_a*z_a*z_c + y_a*z_b*z_c - 2*y_b*z_a*z_c + y_c*z_a*z_b + y_c*z_a*z_c - y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) - ((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(x_a*z_b - x_b*z_a - x_a*z_c + x_c*z_a + x_b*z_c - x_c*z_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_b^2*y_a + x_c^2*y_a - x_b^2*y_c - x_c^2*y_b + y_a*z_b^2 + y_a*z_c^2 - y_b*z_c^2 - y_c*z_b^2 - x_a*x_b*y_b + x_a*x_b*y_c + x_a*x_c*y_b - 2*x_b*x_c*y_a - x_a*x_c*y_c + x_b*x_c*y_b + x_b*x_c*y_c - y_b*z_a*z_b - 2*y_a*z_b*z_c + y_b*z_a*z_c + y_c*z_a*z_b + y_b*z_b*z_c - y_c*z_a*z_c + y_c*z_b*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);
    z_o =((((z_a)/2 + (z_b)/2)*((x_a - x_b)*(y_a - y_c) - (x_a - x_c)*(y_a - y_b)) - ((y_a)/2 + (y_b)/2)*((x_a - x_b)*(z_a - z_c) - (x_a - x_c)*(z_a - z_b)) + ((x_a)/2 + (x_b)/2)*((y_a - y_b)*(z_a - z_c) - (y_a - y_c)*(z_a - z_b)))*(x_a*y_b - x_b*y_a - x_a*y_c + x_c*y_a + x_b*y_c - x_c*y_b))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_b - x_c)*((x_b)/2 + (x_c)/2) + (y_b - y_c)*((y_b)/2 + (y_c)/2) + (z_b - z_c)*((z_b)/2 + (z_c)/2))*(x_a^2*z_b - x_a^2*z_c - x_c^2*z_a + x_c^2*z_b + y_a^2*z_b - y_a^2*z_c - y_c^2*z_a + y_c^2*z_b - x_a*x_b*z_a + x_a*x_c*z_a + x_a*x_b*z_c - 2*x_a*x_c*z_b + x_b*x_c*z_a + x_a*x_c*z_c - x_b*x_c*z_c - y_a*y_b*z_a + y_a*y_c*z_a + y_a*y_b*z_c - 2*y_a*y_c*z_b + y_b*y_c*z_a + y_a*y_c*z_c - y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2) + (((x_a - x_c)*((x_a)/2 + (x_c)/2) + (y_a - y_c)*((y_a)/2 + (y_c)/2) + (z_a - z_c)*((z_a)/2 + (z_c)/2))*(x_b^2*z_a + x_c^2*z_a - x_b^2*z_c - x_c^2*z_b + y_b^2*z_a + y_c^2*z_a - y_b^2*z_c - y_c^2*z_b - x_a*x_b*z_b + x_a*x_b*z_c + x_a*x_c*z_b - 2*x_b*x_c*z_a - x_a*x_c*z_c + x_b*x_c*z_b + x_b*x_c*z_c - y_a*y_b*z_b + y_a*y_b*z_c + y_a*y_c*z_b - 2*y_b*y_c*z_a - y_a*y_c*z_c + y_b*y_c*z_b + y_b*y_c*z_c))/(x_a^2*y_b^2 - 2*x_a^2*y_b*y_c + x_a^2*y_c^2 + x_a^2*z_b^2 - 2*x_a^2*z_b*z_c + x_a^2*z_c^2 - 2*x_a*x_b*y_a*y_b + 2*x_a*x_b*y_a*y_c + 2*x_a*x_b*y_b*y_c - 2*x_a*x_b*y_c^2 - 2*x_a*x_b*z_a*z_b + 2*x_a*x_b*z_a*z_c + 2*x_a*x_b*z_b*z_c - 2*x_a*x_b*z_c^2 + 2*x_a*x_c*y_a*y_b - 2*x_a*x_c*y_a*y_c - 2*x_a*x_c*y_b^2 + 2*x_a*x_c*y_b*y_c + 2*x_a*x_c*z_a*z_b - 2*x_a*x_c*z_a*z_c - 2*x_a*x_c*z_b^2 + 2*x_a*x_c*z_b*z_c + x_b^2*y_a^2 - 2*x_b^2*y_a*y_c + x_b^2*y_c^2 + x_b^2*z_a^2 - 2*x_b^2*z_a*z_c + x_b^2*z_c^2 - 2*x_b*x_c*y_a^2 + 2*x_b*x_c*y_a*y_b + 2*x_b*x_c*y_a*y_c - 2*x_b*x_c*y_b*y_c - 2*x_b*x_c*z_a^2 + 2*x_b*x_c*z_a*z_b + 2*x_b*x_c*z_a*z_c - 2*x_b*x_c*z_b*z_c + x_c^2*y_a^2 - 2*x_c^2*y_a*y_b + x_c^2*y_b^2 + x_c^2*z_a^2 - 2*x_c^2*z_a*z_b + x_c^2*z_b^2 + y_a^2*z_b^2 - 2*y_a^2*z_b*z_c + y_a^2*z_c^2 - 2*y_a*y_b*z_a*z_b + 2*y_a*y_b*z_a*z_c + 2*y_a*y_b*z_b*z_c - 2*y_a*y_b*z_c^2 + 2*y_a*y_c*z_a*z_b - 2*y_a*y_c*z_a*z_c - 2*y_a*y_c*z_b^2 + 2*y_a*y_c*z_b*z_c + y_b^2*z_a^2 - 2*y_b^2*z_a*z_c + y_b^2*z_c^2 - 2*y_b*y_c*z_a^2 + 2*y_b*y_c*z_a*z_b + 2*y_b*y_c*z_a*z_c - 2*y_b*y_c*z_b*z_c + y_c^2*z_a^2 - 2*y_c^2*z_a*z_b + y_c^2*z_b^2);  
    
    i0  = [1 0 0];
    j0  = [0 1 0];
    k0  = [0 0 1];
    
    Ox1 = [x_a-x_o, y_a-y_o , z_a-z_o];
    Oz1 = cross([x_a-x_o, y_a-y_o , z_a-z_o],[x_b-x_o, y_b-y_o , z_b-z_o]);
    if(fix(cross([x_a-x_o, y_a-y_o , z_a-z_o],[x_b-x_o, y_b-y_o , z_b-z_o]))==0)
        Oz1 = cross([x_a-x_o, y_a-y_o , z_a-z_o],[x_c-x_o, y_c-y_o , z_c-z_o]);
    end
    Oy1 = cross(Oz1,Ox1);
    
    i1  = Ox1/sqrt(Ox1(1,1)^2 + Ox1(1,2)^2 + Ox1(1,3)^2);
    j1  = Oy1/sqrt(Oy1(1,1)^2 + Oy1(1,2)^2 + Oy1(1,3)^2);
    k1  = Oz1/sqrt(Oz1(1,1)^2 + Oz1(1,2)^2 + Oz1(1,3)^2);

    R10 = [    i0*i1', i0*j1', i0*k1';
                    j0*i1', j0*j1', j0*k1';
                    k0*i1', k0*j1', k0*k1'];

    H10 = [R10, [x_o; y_o; z_o];
        0 0 0 1];
    
    AOB  = 2*pi;

    R=sqrt((x_b-x_o)^2+(y_b-y_o)^2+(z_b-z_o)^2);
    Cung_AB = abs(AOB*(2*pi*R)/(2*pi));
    tf_o = Cung_AB/v_qd;
    tf   = (1+Percent_tt)*tf_o;
    q_o  = 0;                %% trang thai cua diem dau
    q_f  = AOB;               %% trang thai cua diem cuoi
    tb   = Percent_tt*tf_o;
    vqd  = (q_o-q_f)/(tb-tf);
    a    = vqd/tb;
    
    for i=1:1:((tf/ts)+3)
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
        plot(t,Psi*100/AOB,'.','LineWidth',20,'MarkerEdgeColor','b')
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
        %% Xu ly bien truoc khi dung quy dao nay
        if(count~=0)
%             WP=WP+1;
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
%             subplot(1,5,5);
%             hold off;
            break;
        end
        %% Check dieu kien dung
        t=t+ts;
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
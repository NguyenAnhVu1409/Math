 function [st]=LSPB(tb,tf,q_o,q_f,vqd,a,t)   
    if((t>=0)&&(t<=tb))
        st = q_o + 0.5*a*t*t;
    end
    if((t>=tb)&&(t<=(tf-tb)))
        st=(q_o + 0.5*a*tb*tb)+vqd*(t-tb);
    end
    if((t>=(tf-tb))&&(t<=tf))
        st = q_f+a*tf*t-0.5*a*tf*tf-0.5*a*t*t;
    end  
 end
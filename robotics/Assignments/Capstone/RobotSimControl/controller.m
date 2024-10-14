function u = controller(params, t, phi, phidot)  
    persistent newstate;
    persistent time;
    
    kp=100;
    kd=10;
    ki=1000;

    if isempty(newstate)
        newstate = 0
    end
    
    if isempty(time)
        time = 0
    end

    dt = t - time;
    time = t;
    
    newstate=newstate+(0-phi*dt);
    
    u=-1*(kp*(0-phi) + kd*(0-phidot) + ki*(newstate));
end


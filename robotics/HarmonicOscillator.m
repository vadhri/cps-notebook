% demonstrate a harmonic oscillator. 

function ode_example() 
    X0 = [1,0];
    tspan = [0, 10];
    [t,X] = ode45(@shosc, tspan, X0);
    clf

    hold all 
    plot(t,X(:,1))
    plot(t,X(:,2))
    hold off

end 

function Xd= shosc(t,X)
    x = X(1);
    xd = X(2);
    Xd = [xd; -x];
end 

ode_example();
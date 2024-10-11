function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
n=size(x,1);

x=[x ones(n,1)];
X=[X ones(n,1)];

b=K\x';
x=b';

A=[];

for i=1:n
    a=[zeros(1,4) -X(i,:) x(i,2)*X(i,:); 
        X(i,:) zeros(1,4) -x(i,1)*X(i,:);
        -x(i,2)*X(i,:) x(i,1)*X(i,:) zeros(1,4)];
    A=[A;a];
end

[u,d,v]=svd(A);
P=reshape(v(:,end),4,3)';

R=P(:,1:3);
t=P(:,end);

[u,d,v]=svd(R);

t=t/d(1,1);
R=u*v';

if det(u*v') <= 0
   R=-R;
   t=-t;

end

C=-R'*t;

end







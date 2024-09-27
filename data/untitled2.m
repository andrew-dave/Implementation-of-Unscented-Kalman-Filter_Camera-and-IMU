n=27;
XAug = zeros(n,2*n+1);
XAug(:,1) = uAug;

%Computing the sigma points for n iterations
for i = 2:n+1
    XAug(:,i) = uAug + sqrt(n + lambda)*root_PAug(:,i-1);
    XAug(:,i+n) = uAug - sqrt(n + lambda)*root_PAug(:,i-1);
end
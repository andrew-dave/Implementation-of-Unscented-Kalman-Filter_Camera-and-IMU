a = 1.5;
c = 0.5;
h = 2.75; 

x = 0:pi/10000:pi/2;  % Define the range of x values

y = zeros(size(x*10));  % Initialize y as a vector of zeros

for i = 1:numel(x)
    y(i) = (a * (sqrt((abs(1-x(i)) + 1 - x(i)) / 2)+0.25))* exp(-(1-x(i))^2) + ...
           (c / 40) * ( abs(1 - 16 * (5*x(i)-h)^4) + abs(1 - 512*(5*x(i)-h)^4) + 2 - 528*(5*x(i)-h)^4);
end
%y = ((a*(sqrt((abs(1-x)+ 1-x)/2))*exp(-(1-x).^2))) + (c/40)*( abs(1 - 16*( (5*x-h).^4 )) + abs(1-512*((5*x-h).^4)) + 2 - 528*((5*x-h).^4) );

plot(y, x);

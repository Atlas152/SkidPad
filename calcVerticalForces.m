function FzTyre = calcVerticalForces(car)

%Front Left
fz11 = (car.m.*car.g.*car.a2./(2.*car.l)) - (car.eta1.*car.m.*car.ay);

%Front Right
fz12 = (car.m.*car.g.*car.a2./(2.*car.l)) + (car.eta1.*car.m.*car.ay);

%Rear Left
fz21 = (car.m.*car.g.*car.a1./(2.*car.l)) - (car.eta2.*car.m.*car.ay);

%Rear Right
fz22 = (car.m.*car.g.*car.a1./(2.*car.l)) - (car.eta2.*car.m.*car.ay);

end
function FzTyre = calcVerticalForces(car, ay)

%Front Left
fz11 = 1./2(car.m.*car.g.*car.a2./car.l) - (car.eta1.*car.m.*car.ay);

%Front Right
fz12 = 1./2(car.m.*car.g.*car.a2./car.l) + (car.eta1.*car.m.*car.ay);

%Rear Left
fz21 = 1./2(car.m.*car.g.*car.a1./car.l) - (car.eta2.*car.m.*car.ay);

%Rear Right
fz22 = 1./2(car.m.*car.g.*car.a1./car.l) - (car.eta2.*car.m.*car.ay);

end
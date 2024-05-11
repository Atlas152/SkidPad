function  car = calcDerivedParams(car)

%% Dimensions
car.l = car.a1 + car.a2;

%% Static Weights
car.Fz0 = 1/2.*(car.m.*car.g./car.l).*[car.a2; car.a2; car.a1; car.a1];

%% Suspension Geometry
% Wheel Rates
car.kWheelF = car.KSpringF ./ car.mSpringF.^2;
car.kWheelR = car.KSpringR ./ car.mSpringR.^2;

% Axle Heave Rates
% Front
car.kzs1 = car.kWheelF * 2;                                                 %front suspension heave rate
car.kzp1 = car.kp1 * 2;                                                     %front tyre heave rate
car.kz1 = (car.kzs1 * car.kzp1) ./ (car.kzs1 + car.kzp1);                   %front axle heave rate

% Rear
car.kzs2 = car.kWheelR * 2;                                                 %rear suspension heave rate
car.kzp2 = car.kp2 * 2;                                                     %rear tyre heave rate
car.kz2 = (car.kzs2 * car.kzp2) ./ (car.kzs2 + car.kzp2);                   %rear axle heave rate

% Axle Roll Rates
% Front
% 1) Suspension Roll Rates
car.krSpringF = car;
car.krARBF = car;
car.krs1 = car;
% 2) Tyre Roll Rates
car.krp1 = car.kp1 .* car.t1.^2 ./ 2;
% 3) Total Front Roll Stiffness
car.kr1 = (car.krs1 .* car.krp1) ./ (car.krs1 + car.krp1);

% Rear
% 1) Suspension Roll Rates
car.krSpringR = car;
car.krARBR = car;
car.krs2 = car;
% 2) Tyre Roll Rates
car.krp2 = car.kp2 .* car.t2.^2 ./2;
% 3) Total Rear Roll Stiffness
car.kr2 = (car.krs2 .* car.krp2) ./ (car.krs2 + car.krp2);

% Car Roll Stiffness
car.kr = car.kr1 + car.kr2;

%% Load Transfer Coefficients
%niu: deltaFi = niui*m*ay
car.niu1 = (car.kr1.*car.kr2)./(car.t1.*car.kr).*((car.h-car.q)./car.kr2 + (car.a2.*car.q1)./(car.l.*car.krs1) + (car.a2.*car.q1)./(car.l.*car.krs2) + (car.a2.*car.q1 + car.a1.*car.q2)./(car.l.*car.krp2));
car.niu2 = (car.kr1.*car.kr2)./(car.t2.*car.kr).*((car.h-car.q)./car.kr1 + (car.a1.*car.q2)./(car.l.*car.krs1) + (car.a1.*car.q2)./(car.l.*car.krs2) + (car.a2.*car.q1 + car.a1.*car.q2)./(car.l.*car.krp1));
%% Roll Angle Coefficients
%rhos: relation between suspension roll angle and lateral acceleration
car.rhos1 = (car.kr1.*car.kr2)./(car.krs1.*car.kr).*((car.h-car.q)./car.kr2 - (car.a2.*car.q1)./(car.l.*car.krp1) + (car.a1.*car.q2)./(car.l.*car.krp2));
car.rhos2 = (car.kr1.*car.kr2)./(car.krs2.*car.kr).*((car.h-car.q)./car.kr1 - (car.a1.*car.q2)./(car.l.*car.krp2) + (car.a2.*car.q1)./(car.l.*car.krp1));

%rhop: relation between tyre roll angle and lateral acceleration
car.eta1 = (car.kr1.*(car.h-car.q)./car.kr + (car.a2.*car.q1)./car.l)./car.t1;
car.eta2 = (car.kr2.*(car.h-car.q)./car.kr + (car.a1.*car.q2)./car.l)./car.t2;

car.rhop1 = (car.eta1.*car.t1)./car.krp1;
car.rhop2 = (car.eta2.*car.t2)./car.krp2;

%% Camber Gain Coefficients
% Camber gain with heave (suspension)
car.dCamberdz1 = -1./car.c1;
car.dCamberdz2 = -1./car.c2;
% Camber gain with roll
% Suspension Roll
car.dCamberdrs1 = -(car.t1./2 - car.c1)./car.c1;
car.dCamberdrs2 = -(car.t2./2 - car.c2)./car.c1;
% Tyre Roll
car.dCamberdrp1 = 1;
car.dCamberdrp2 = 1;
%chi: relation between camber variation due to roll and lateral
%acceleration (dGammai = chii*m*ay)
car.chi1 = (-(car.t1./2 - car.c1)./car.c1).*car.rhos1 + car.rhop1;
car.chi2 = (-(car.t2./2 - car.c2)./car.c2).*car.rhos2 + car.rhop2;

end
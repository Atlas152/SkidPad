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
car.niu1 = (car.kr1.*car.kr2)./(car.t1.*car.kr).*((car.h-car.q)./car.kr2 + (car.a2.*car.q1)./(car.l.*car.krs1) + (car.a2.*car.q1)./(car.l.*car.krs2) + (car.a2.*car.q1 + car.a1.*car.q2)./(car.l.*car.krp2));
car.niu2 = (car.kr1.*car.kr2)./(car.t2.*car.kr).*((car.h-car.q)./car.kr1 + (car.a1.*car.q2)./(car.l.*car.krs1) + (car.a1.*car.q2)./(car.l.*car.krs2) + (car.a2.*car.q1 + car.a1.*car.q2)./(car.l.*car.krp1));
%% Roll Angle Coefficients

%% Camber Gain Coefficients
% Camber gain with heave (suspension)
car.dCamberdz1 = -1./car.c1;
car.dCamberdz2 = -1./car.c2;
% Camber gain with roll
% Suspension Roll
car.dCamberdrs1 = -(car.t1./2 - car.c1);
car.dCamberdrs2 = -(car.t2./2 - car.c2);
% Tyre Roll
car.dCamberdrp1 = 1;
car.dCamberdrs2 = 1;

end
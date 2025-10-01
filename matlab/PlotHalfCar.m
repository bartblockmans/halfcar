function [] = PlotHalfCar(q, P)

% Unpack
X = q(1); % Absolute X position in world axis frame of the chassis
Y = q(2); % Absolute Y position in world axis frame of the chassis
theta = q(3); % Pitch angle of the chassis
zF = q(4); % Absolute Y position in world axis frame of unsprung mass
zR = q(5); % Absolute Y position in world axis frame of unsprung mass

% Unit vector
e = [cos(theta); sin(theta)];

% Compute coordinates
% -------------------------------------------------------------------------

% Chassis pickup points 
pRx = X-P.lr * e(1);
pFx = X+P.lf * e(1);
pRy = Y-P.lr * e(2);
pFy = Y+P.lf * e(2);

% Unsprung mass coordinates
uFy = zF;
uFx = pFx + (pFy - zF) * tan(theta);
uRy = zR;
uRx = pRx + (pRy - zR) * tan(theta);

% Ground coordinates
rRx = uRx + P.LtR * e(2);
rFx = uFx + P.LtF * e(2);
rRy = uRy - P.LtR * e(1);
rFy = uFy - P.LtF * e(1);

% Plot
% -------------------------------------------------------------------------

% Plot chassis
PlotRectangle(pRx, pRy, pFx, pFy,P.hc,[0 0 0],[0.5 0.5 0.5], 1);

% Plot circles at connection point
PlotCircle(pRx,pRy,0.025,0,2*pi,[0 0 0],1,0,0,100);
PlotCircle(pFx,pFy,0.025,0,2*pi,[0 0 0],1,0,0,100);

% Plot unsprung masses
PlotCircle(uRx,uRy,P.hu/2,0,2*pi,[0.5 0.5 0.5],1,0,0,100);
PlotCircle(uFx,uFy,P.hu/2,0,2*pi,[0.5 0.5 0.5],1,0,0,100);

% Plot center unsprung mass
PlotCircle(uRx,uRy,0.025,0,2*pi,[0 0 0],1,0,0,100);
PlotCircle(uFx,uFy,0.025,0,2*pi,[0 0 0],1,0,0,100);

% Plot ground points
PlotCircle(rRx,rRy,0.025,0,2*pi,[0 0 0],1,0,0,100);
PlotCircle(rFx,rFy,0.025,0,2*pi,[0 0 0],1,0,0,100);

% Plot springs
PlotFullSpring(pRx, pRy, uRx, uRy, 0.1, 1/2, 5, [0 0 0], 1);
PlotFullSpring(pFx, pFy, uFx, uFy, 0.1, 1/2, 5, [0 0 0], 1);
PlotFullSpring(uRx, uRy, rRx, rRy, 0.1, 1/2, 4, [0 0 0], 1);
PlotFullSpring(uFx, uFy, rFx, rFy, 0.1, 1/2, 4, [0 0 0], 1);

% Plot wheel
PlotCircle(uRx, uRy, P.LtR, 0, 2*pi, [], 1, 0, 0, 100);
PlotCircle(uFx, uFy, P.LtF, 0, 2*pi, [], 1, 0, 0, 100);
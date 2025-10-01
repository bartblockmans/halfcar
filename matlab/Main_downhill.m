% Main_downhill
% -------------------------------------------------------------------------

%% ========================================================================
%                       INPUT PARAMETERS BICYCLE
% =========================================================================

% Initialize parameter struct
P = struct();

% Mass & inertia
P.ms  = 80;         % sprung mass [kg]
P.m_split_f = 0.4;  % Mass split front wheel [-]
P.Is  = 24;         % chassis pitch inertia [kg*m^2]
P.muf = 3.5;        % front unsprung [kg]
P.mur = 4.0;        % rear  unsprung [kg]

% Aerodynamics & rolling resistance
P.CdA = 0.350;      % CdA coefficient [-]
P.rho = 1.225;      % Air density [kg/m³]
P.Crr = 0.010;      % Rolling resistance coefficient

% Max torque [Nm]
P.Tmax = 80; 

% Chassis geometry (COM to pick-ups along chassis centerline)
P.lf  = 0.60;       % [m]
P.lr  = 0.60;       % [m]
P.L0 = P.lf + P.lr;

% Free lengths along local road normal
P.LtF = 0.35;       % tyre free length front (visual radius) [m]
P.LtR = 0.35;       % tyre free length rear  [m]
P.LsF = 0.45;       % suspension free length front [m]
P.LsR = 0.45;       % suspension free length rear  [m]

% Height chassis & unsprung mass (only for plotting)
P.hc = 0.15; 
P.hu = 0.15;

% Suspension stiffness (Kelvin–Voigt along local normal)
P.ksf = 18e3;  % front
P.ksr = 10e3;  % rear

% Tyre stiffness (Kelvin–Voigt normal)
P.ktf = 120e3;  % front
P.ktr = 140e3;  % rear

% Compute damping based on damping ratio's
P.zeta_body_F   = 0.25;   % target damping ratio for body mode (front)
P.zeta_wheel_F  = 0.50;   % target damping ratio for wheel-hop mode (front)
P.zeta_body_R   = 0.25;   % target damping ratio for body mode (rear)
P.zeta_wheel_R  = 0.50;   % target damping ratio for wheel-hop mode (rear)
[csf, csr, ctf, ctr] = ComputeDamping(P);
P.csf = csf;
P.csr = csr;
P.ctf = ctf;
P.ctr = ctr;

% Gravity acceleration [m/s²]
P.g = -9.81;

% Include n_dot (curvature) in forces? Usually small; keep off for stability.
P.use_n_dot = false;

% Contact parameters
P.contact = 'unilateral'; % 'unilateral' (default) or 'bilateral'
P.contact_method = 'circle'; % 'point' or 'circle'

% Initial horizontal position with rear tyre at X = 0
q = zeros(5,1);
q(1) = P.lf;
q(2) = 0.5 * (P.LtF + P.LsF + P.LtR + P.LsR);

% Plot half-car
figure; hold on; axis equal; box on;
PlotHalfCar(q, P);

%% ========================================================================
%                            ROAD DEFINITION
% =========================================================================

% Load course
[x_course, z_course, ~, course_info] = SelectCourse('course','extreme');

% Build spline once and derivatives
pp_h   = pchip(x_course, z_course);
pp_hp  = fnder(pp_h);      % dh/dx
pp_hpp = fnder(pp_hp);     % d2h/dx2

% Store in road struct
road.x_course = x_course;
road.z_course = z_course;
road.course_info = course_info;
road.h   = @(x) ppval(pp_h,   x);
road.hp  = @(x) ppval(pp_hp,  x);
road.hpp = @(x) ppval(pp_hpp, x);

% Optional slope clamp (improves robustness near ends)
s_max = 2.0;
road.hpC  = @(x) max(min(road.hp(x), s_max), -s_max);
road.hppC = @(x) road.hpp(x);

% Plot road
figure; hold on; axis equal; box on;
plot(x_course, road.h(x_course));
fill([x_course, 0, 0], [road.h(x_course), road.h(x_course(end)), road.h(0)], [0.5 0.5 0.5])

%% ========================================================================
%                             PACING PLAN
% =========================================================================
% Propulsive power should only be applied along flat or curved sections 

% Define non-zero propulsive power in function of distance
P.power = [];

% Add to road 
[road.P, road.T_max] = PowerData(P, road);

%% ========================================================================
%                         INITIAL SPEED ALONG ROAD
% =========================================================================

% Initial forward velocity in m/s (set slightly positive in case of
% starting from flat section)
U0 = 10;

% Initial position, road derivative & tangent
X0 = P.lr; 
s0 = road.hpC(X0);
t0 = [1; s0] / sqrt(1 + s0^2);

% Initial velocity in world axis frame
v0_world = U0 * t0;

% Align chassis with local slope
th0 = atan(s0);

%% ========================================================================
%                               STATIC SAG
% =========================================================================

% Compute static sag on flat coarse
[zuf0, zur0] = static_sag(P, X0, th0, road);

% Inclination angle
P.theta_sag = -atan((zur0 - zuf0)/(P.lf + P.lr));

% Compute static sag on actual coarse
[zuf0, zur0, Y0, etaF, etaR] = static_sag(P, X0, th0, road);

% Initial state
q0 = [ X0;  Y0;  th0;  zuf0;  zur0 ];
v0 = [ v0_world(1); v0_world(2); 0; 0; 0 ];
y0 = [ q0; v0 ];

% Plot half-car
figure; hold on; axis equal; box on;
PlotHalfCar(q0, P);
plot(linspace(X0-2, X0+2, 100), road.h(linspace(X0-2, X0+2, 100)), 'k');

% Debug: print initial conditions
fprintf('Initial conditions:\n');
fprintf('q0 = [%.6f, %.6f, %.6f, %.6f, %.6f]\n', q0);
fprintf('v0 = [%.6f, %.6f, %.6f, %.6f, %.6f]\n', v0);
fprintf('Contact mode: %s\n', P.contact);
fprintf('Front suspension compression: %.2f mm\n', etaF*1000);
fprintf('Rear suspension compression: %.2f mm\n', etaR*1000);

%% ========================================================================
%                    INTEGRATE EQUATIONS OF MOTION
% =========================================================================

% Simulation time [s]
Tend = Inf;

% Solver
solver = 'ode15s';

% Integrate equations of motion
[t, y] = IntegrateEOMs(y0, P, road, Tend, solver);

% Recompute derived quantities for plotting & animation
results = ComputeResults(t, y, P, road);

% Unpack
X=y(:,1); Y=y(:,2); th=y(:,3); zuf=y(:,4); zur=y(:,5);
Xd=y(:,6); Yd=y(:,7); thd=y(:,8); zdf=y(:,9); zdr=y(:,10);
speed = hypot(Xd, Yd);

%% ========================================================================
%                           PLOT RESULTS
% =========================================================================

figure('Name','States (ode15s)','Position',[80,80,1400,800]);
namesQ = {'X','Y','\theta','z_{uf}','z_{ur}'};
namesV = {'Xdot','Ydot','thetadot','z_{uf}dot','z_{ur}dot'};
for i=1:5
    subplot(2,5,i);
    plot(t, y(:,i), 'LineWidth',1.4); grid on; xlabel('t [s]'); ylabel(namesQ{i});
end
for i=1:5
    subplot(2,5,5+i);
    plot(t, y(:,5+i), 'LineWidth',1.4); grid on; xlabel('t [s]'); ylabel(namesV{i});
end
sgtitle(sprintf('5-DOF model with ode15s (%s contact)', P.contact), 'FontWeight','bold');

% Second figure: Suspension and tire dynamics
figure('Name','Suspension & Tire Dynamics','Position',[100,100,1200,600]);

% Top left: Suspension deflections
subplot(2,2,1);
plot(results.time, results.etaF, 'LineWidth',1.4, 'DisplayName','Front'); hold on;
plot(results.time, results.etaR, 'LineWidth',1.4, 'DisplayName','Rear');
grid on; xlabel('t [s]'); ylabel('Suspension deflection [m]');
title('Suspension Deflections'); legend('Location','best');

% Bottom left: Suspension forces
subplot(2,2,3);
plot(results.time, results.FsF, 'LineWidth',1.4, 'DisplayName','Front'); hold on;
plot(results.time, results.FsR, 'LineWidth',1.4, 'DisplayName','Rear');
grid on; xlabel('t [s]'); ylabel('Suspension force [N]');
title('Suspension Forces'); legend('Location','best');

% Top right: Tire compressions
subplot(2,2,2);
plot(results.time, results.deltaF, 'LineWidth',1.4, 'DisplayName','Front'); hold on;
plot(results.time, results.deltaR, 'LineWidth',1.4, 'DisplayName','Rear');
grid on; xlabel('t [s]'); ylabel('Tire compression [m]');
title('Tire Compressions'); legend('Location','best');

% Bottom right: Tire forces
subplot(2,2,4);
plot(results.time, results.Nf, 'LineWidth',1.4, 'DisplayName','Front'); hold on;
plot(results.time, results.Nr, 'LineWidth',1.4, 'DisplayName','Rear');
grid on; xlabel('t [s]'); ylabel('Tire force [N]');
title('Tire Forces'); legend('Location','best');

sgtitle(sprintf('Suspension & Tire Dynamics (%s contact)', P.contact), 'FontWeight','bold');

%% ========================================================================
%                           ANIMATE RESULTS
% =========================================================================

% Number of frames per second
fps = 24;

% Animate results
animate_results(t, y, P, road, fps, 2, 1);


function [csf, csr, ctf, ctr] = ComputeDamping(P)
% ComputeDamping
%   [csf, csr, ctf, ctr] = ComputeDamping(P)
% Computes suspension (csf, csr) and tyre (ctf, ctr) damping constants
% for a half-car modeled as two independent quarter-cars (front/rear).
%
% Inputs
%   P : downhill MTB parameters
%
% Outputs
%   csf, csr : suspension damping front/rear [Ns/m]
%   ctf, ctr : tyre damping front/rear [Ns/m]
%
% Method
%   For each axle, we form the undamped 2-DOF quarter-car (Ms, Mu, Ks, Kt),
%   compute undamped modes (body and wheel-hop), then solve a 2x2 system so
%   the modal damping ratios match the targets below.
%

% Get parameters
ksf = P.ksf;                        % suspension stiffness front [N/m]  (wheel-normal)
ksr = P.ksr;                        % suspension stiffness rear [N/m]   (wheel-normal)
ktf = P.ktf;                        % tyre stiffness front [N/m]        (wheel-normal)
ktr = P.ktr;                        % tyre stiffness rear [N/m]         (wheel-normal)
mc = P.ms;                          % chassis (sprung) mass total [kg]
muf = P.muf;                        % front unsprung mass [kg]
mur = P.mur;                        % rear unsprung mass [kg]
front_split   = P.m_split_f;        % mass split front
zeta_body_F   = P.zeta_body_F;      % target damping ratio for body mode (front)
zeta_wheel_F  = P.zeta_wheel_F;     % target damping ratio for wheel-hop mode (front)
zeta_body_R   = P.zeta_body_R;      % target damping ratio for body mode (rear)
zeta_wheel_R  = P.zeta_wheel_R;     % target damping ratio for wheel-hop mode (rear)

% Effective sprung masses per axle
msf = mc * front_split;
msr = mc * (1 - front_split);

% Front axle
[csf, ctf] = quarter_car_c_from_zetas(msf, muf, ksf, ktf, zeta_body_F, zeta_wheel_F);

% Rear axle
[csr, ctr] = quarter_car_c_from_zetas(msr, mur, ksr, ktr, zeta_body_R, zeta_wheel_R);

% Optional: print a quick summary and verify achieved zetas
% (comment out if you prefer a silent function)
[zF, wF] = quarter_car_zeta_from_c(msf, muf, ksf, ktf, csf, ctf);
[zR, wR] = quarter_car_zeta_from_c(msr, mur, ksr, ktr, csr, ctr);
fprintf('[ComputeDamping] Front:  cs=%.1f Ns/m, ct=%.1f Ns/m | zeta=[%.3f, %.3f], f=[%.2f, %.2f] Hz\n',...
    csf, ctf, zF(1), zF(2), wF(1)/(2*pi), wF(2)/(2*pi));
fprintf('[ComputeDamping] Rear:   cs=%.1f Ns/m, ct=%.1f Ns/m | zeta=[%.3f, %.3f], f=[%.2f, %.2f] Hz\n',...
    csr, ctr, zR(1), zR(2), wR(1)/(2*pi), wR(2)/(2*pi));
end

% ========================= Helpers (local) ================================

function [cs, ct] = quarter_car_c_from_zetas(ms, mu, ks, kt, zeta_body, zeta_wheel)
% Solve for (cs, ct) to match the two target modal damping ratios.

[M, K] = mk_mk(ms, mu, ks, kt);

% Undamped modes: K*phi = w^2 * M * phi
[Phi, Om2] = eig(K, M);
omega = sqrt(diag(Om2));
% Sort ascending (mode 1 = body, mode 2 = wheel-hop)
[omega, idx] = sort(omega); Phi = Phi(:,idx);

% Modal masses
m1 = Phi(:,1).'*M*Phi(:,1);
m2 = Phi(:,2).'*M*Phi(:,2);

% For C = [ cs -cs; -cs cs+ct ], modal damping is:
%   d_i = phi_i^T C phi_i = cs*(phi_s - phi_u)^2 + ct*(phi_u)^2
a1 = (Phi(1,1) - Phi(2,1))^2;  b1 = (Phi(2,1))^2;
a2 = (Phi(1,2) - Phi(2,2))^2;  b2 = (Phi(2,2))^2;

% Targets: d_i = 2*zeta_i*omega_i*m_i
d1 = 2*zeta_body*omega(1)*m1;
d2 = 2*zeta_wheel*omega(2)*m2;

A   = [a1 b1; a2 b2];
rhs = [d1; d2];

x = A \ rhs;
cs = x(1);  ct = x(2);

% Clip tiny negatives due to round-off; warn on strong negatives
if cs < 0 && cs > -1e-9, cs = 0; end
if ct < 0 && ct > -1e-9, ct = 0; end
if cs < 0 || ct < 0
    warning('quarter_car_c_from_zetas:NegativeC',...
        'Computed negative damping (cs=%.2f, ct=%.2f). Check targets/masses/stiffness.', cs, ct);
end
end

function [zeta, omega] = quarter_car_zeta_from_c(ms, mu, ks, kt, cs, ct)
% Modal damping ratios for given (cs, ct)
[M, K] = mk_mk(ms, mu, ks, kt);
C = [ cs  -cs ;
     -cs  cs+ct ];

[Phi, Om2] = eig(K, M);
omega = sqrt(diag(Om2));
[omega, idx] = sort(omega); Phi = Phi(:,idx);

m1 = Phi(:,1).'*M*Phi(:,1);
m2 = Phi(:,2).'*M*Phi(:,2);

d1 = Phi(:,1).'*C*Phi(:,1);
d2 = Phi(:,2).'*C*Phi(:,2);

zeta = [ d1/(2*omega(1)*m1) ; d2/(2*omega(2)*m2) ];
end

function [M, K] = mk_mk(ms, mu, ks, kt)
% 2-DOF quarter-car mass & stiffness
M = diag([ms, mu]);
K = [ ks,   -ks ;
     -ks, ks+kt ];
end

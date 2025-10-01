function [dydt, output] = ComputeAccelerations(t, y, P, road, debug, return_output)

% Check input arguments
if nargin < 5; debug = 0; end
if nargin < 6; return_output = 0; end

% Unpack state
X=y(1); Y=y(2); th=y(3); zuf=y(4); zur=y(5);
Xd=y(6); Yd=y(7); thd=y(8); zdf=y(9); zdr=y(10);

% Plot vehicle
if debug
    figure; hold on; axis equal; box on;
    PlotHalfCar(y(1:5), P);
    plot(linspace(X-2, X+2, 100), road.h(linspace(X-2, X+2, 100)), 'k');
end

% Check for NaN or Inf values
if any(isnan(y)) || any(isinf(y))
    error('NaN or Inf detected in state vector');
end

% Body axes
u      = [cos(th); sin(th)];
u_perp = [-sin(th); cos(th)];

% Normal direction to the chassis
n_chassis = [-sin(th); cos(th)];
nd_chassis = [-cos(th); -sin(th)] * thd;

% Pickups (world) and velocities
pF = [X;Y] + P.lf*u;
pR = [X;Y] - P.lr*u;
vF = [Xd;Yd] + P.lf*thd*u_perp;
vR = [Xd;Yd] - P.lr*thd*u_perp;
if debug
    scatter(pF(1), pF(2),'filled');
    scatter(pR(1), pR(2),'filled');
end

% Intersection with road y = road.h(x)
tF_param = find_road_intersection(pF, n_chassis, road);
tR_param = find_road_intersection(pR, n_chassis, road); 

% Road contact points
rF = pF + tF_param * n_chassis;
rR = pR + tR_param * n_chassis; 
if debug
    scatter(rF(1), rF(2),'filled');
    scatter(rR(1), rR(2),'filled');
end

% Road normals & tangents
sF = road.hpC(rF(1));  sR = road.hpC(rR(1));
dF = sqrt(1+sF^2);  dR = sqrt(1+sR^2);
nF = [-sF; 1]/dF;   nR = [-sR; 1]/dR;
tF = [1; sF]/dF;    tR = [1; sR]/dR;

% Road velocities at contact points
rFd = [Xd; sF*Xd];
rRd = [Xd; sR*Xd];

% n_dot (optional)
if P.use_n_dot
    sFp = road.hppC(rF(1));
    sRp = road.hppC(rR(1));
    nFd = -(sFp*rFd(1))/(dF^3) * [1; sF];
    nRd = -(sRp*rRd(1))/(dR^3) * [1; sR];
else
    nFd = [0;0]; nRd = [0;0];
end

% Suspension end points
suspF = pF - n_chassis * P.LsF;
suspR = pR - n_chassis * P.LsR;
if debug
    scatter(suspF(1), suspF(2),'filled');
    scatter(suspR(1), suspR(2),'filled');
end

% Unsprung masses and velocities
uFy = zuf; 
uFx = pF(1) + (pF(2) - zuf) * tan(th);
uRy = zur; 
uRx = pR(1) + (pR(2) - zur) * tan(th);
uF = [uFx; uFy]; 
uR = [uRx; uRy];
vUFy = zdf;
vUFx = vF(1) + (vF(2) - zdf) * tan(th) + (pF(2) - zuf) * sec(th)^2 * thd;
vURy = zdr;
vURx = vR(1) + (vR(2) - zdr) * tan(th) + (pR(2) - zur) * sec(th)^2 * thd;
vUF = [vUFx; vUFy];
vUR = [vURx; vURy];
if debug
    scatter(uF(1), uF(2),'filled');
    scatter(uR(1), uR(2),'filled');
end

% Wheel contact points from unsprung masses
cF = uF - P.LtF * n_chassis;
cR = uR - P.LtR * n_chassis;
vCF = vUF - P.LtF * nd_chassis;
vCR = vUR - P.LtR * nd_chassis;
if debug
    scatter(cF(1), cF(2),'filled');
    scatter(cR(1), cR(2),'filled');
end

% Suspension deflection
etaF  = n_chassis.'*(uF - suspF);
etaR  = n_chassis.'*(uR - suspR);
detaF = n_chassis.'*(vUF - vF) + nd_chassis.'*(uF - suspF);
detaR = n_chassis.'*(vUR - vR) + nd_chassis.'*(uR - suspR);

% Suspension forces
FsF = P.ksf*etaF + P.csf*detaF;
FsR = P.ksr*etaR + P.csr*detaR;

% Switch between contact detection mode
if strcmp(P.contact_method, 'point')

    % Tyre compression & rate
    deltaF = nF.'*(rF - cF);
    deltaR = nR.'*(rR - cR);
    deldF = nF.'*(rFd - vCF) + nFd.'*(rF - cF);
    deldR = nR.'*(rRd - vCR) + nRd.'*(rR - cR);

    % Tyre forces (bilateral/unilateral with gated damper)
    if strcmpi(P.contact,'bilateral')
        Nf = P.ktf*deltaF + P.ctf*deldF;
        Nr = P.ktr*deltaR + P.ctr*deldR;
    else
        if deltaF > 0
            Nf = max(0,P.ktf*deltaF + P.ctf*deldF);
        else
            Nf = 0; deltaF = 0;
        end
        if deltaR > 0
            Nr = max(0,P.ktr*deltaR + P.ctr*deldR);
        else
            Nr = 0; deltaR = 0;
        end
    end

elseif strcmp(P.contact_method, 'circle') % only unilateral!

    % Front tyre
    % -----------------------------------------------------------------

    % Relevant road section
    xf_road = linspace(cF(1)-0.5,cF(1)+0.5,100);

    % Find intersection of front tyre with road
    [xf_contact, yf_contact, deltaF, nf_contact, contact_state_f] = ...
        circleCurveContact(uF(1), uF(2), P.LtF, xf_road, road.h(xf_road));

    % Find deltaF
    deldF = nF.'*(rFd - vUF);

    % Plot contact point
    if debug
        scatter(xf_contact, yf_contact, 'filled');
    end

    % Compute force
    if contact_state_f
        Nf = max(0,(P.ktf*deltaF + P.ctf*deldF)*nf_contact(2));
    else
        Nf = 0; deltaF = 0; 
    end

    % Rear tyre
    % -----------------------------------------------------------------

    % Relevant road section
    xr_road = linspace(cR(1)-0.5,cR(1)+0.5,100);

    % Find intersection of front tyre with road
    [xr_contact, yr_contact, deltaR, nr_contact, contact_state_r] = ...
        circleCurveContact(uR(1), uR(2), P.LtR, xr_road, road.h(xr_road));

    % Find deltaF
    deldR = nR.'*(rRd - vUR);

    % Plot contact point
    if debug
        scatter(xr_contact, yr_contact, 'filled');
    end

    % Compute force
    if contact_state_r
        Nr = max(0,(P.ktr*deltaR + P.ctr*deldR)*nr_contact(2));
    else
        Nr = 0; deltaR = 0;
    end

end

% Rolling resistance
RrF = 0;
RrR = 0;
if strcmpi(P.contact,'bilateral') || deltaF > 0
    RrF = -sign(Xd) * P.Crr * Nf;
end
if strcmpi(P.contact,'bilateral') || deltaR > 0
    RrR = -sign(Xd) * P.Crr * Nr;
end

% Aerodynamic resistance
Raero = [0; 0];
V_mag = sqrt(Xd^2 + Yd^2);
if V_mag > 0
    V_unit = [Xd; Yd] / V_mag;
    Raero = -0.5 * P.rho * (V_mag^2) * P.CdA * V_unit;
end

% Propulsive power
% -------------------------------------------------------------------------

% Are both wheels in contact with the road?
if contact_state_r && contact_state_f

    % Propulsive power
    Pprop = interp1(road.x_course, road.P, X);
    
    % Max torque
    Tmax = interp1(road.x_course, road.T_max, X);
    
    % Convert to propulsive force
    Fprop = max(0, min(Pprop / Xd, Tmax/P.LtR));

else
    Fprop = 0;
end

% Accelerations
% -------------------------------------------------------------------------

% CHASSIS accelerations (sum of forces on chassis)
Xdd = (FsF*n_chassis(1) + FsR*n_chassis(1) + RrF*tF(1) + RrR*tR(1) + Raero(1) + Fprop * tR(1)) / P.ms;
Ydd = (FsF*n_chassis(2) + FsR*n_chassis(2) + RrF*tF(2) + RrR*tR(2) + Raero(2) + Fprop * tR(2) + P.ms*P.g*n_chassis(2)) / P.ms;

% CHASSIS pitch acceleration (moments about COM)
cross2 = @(a,b) a(1)*b(2) - a(2)*b(1);
Mz = cross2(pF-[X;Y], FsF*n_chassis + RrF*tF) + cross2(pR-[X;Y], FsR*n_chassis + RrR*tR);

% Add viscous damping to pitch angle for stability
% Default damping coefficient (can be overridden in P structure)
C_pitch = 50;  % N⋅m⋅s/rad (viscous damping coefficient)
if isfield(P, 'C_pitch'); C_pitch = P.C_pitch; end

% Pitch equation with viscous damping
thdd = (Mz - C_pitch * thd) / P.Is;

% UNSPRUNG accelerations (normal DOFs)
zddf = (-FsF*(n_chassis.' * nF) + Nf + P.muf*P.g*nF(2) ) / P.muf;
zddr = (-FsR*(n_chassis.' * nR) + Nr + P.mur*P.g*nR(2) ) / P.mur;

% Pack derivatives
dydt = [ Xd; Yd; thd; zdf; zdr; Xdd; Ydd; thdd; zddf; zddr ];

% Store output
if return_output

    output.X = X;
    output.Y = Y;
    output.th = th;
    output.zuf = zuf; 
    output.zur = zur;
    output.Xd = Xd;
    output.Yd = Yd; 
    output.thd = thd; 
    output.zdf = zdf;
    output.zdr = zdr;
    output.u = u;
    output.u_perp = u_perp;
    output.n_chassis = n_chassis;
    output.nd_chassis = nd_chassis;
    output.pF = pF;
    output.pR = pR;
    output.vF = vF;
    output.vR = vR;
    output.rF = rF;
    output.rR = rR;
    output.sF = sF;
    output.sR = sR;
    output.dF = dF;
    output.dR = dR;
    output.nF = nF;
    output.nR = nR;
    output.tF = tF;
    output.tR = tR;
    output.suspF = suspF;
    output.suspR = suspR;
    output.uFx = uFx;
    output.uFy = uFy;
    output.uRx = uRx;
    output.uRy = uRy;
    output.vUFx = vUFx;
    output.vUFy = vUFy;
    output.vURx = vURx;
    output.vURy = vURy;
    output.cF = cF;
    output.cR = cR;
    output.vCF = vCF;
    output.vCR = vCR;
    output.etaF = etaF;
    output.etaR = etaR;
    output.detaF = detaF;
    output.detaR = detaR;
    output.FsF = FsF;
    output.FsR = FsR;
    output.deltaF = deltaF;
    output.deltaR = deltaR;
    output.deldF = deldF;
    output.deldR = deldR;
    output.Nf = Nf;
    output.Nr = Nr;
    output.RrF = RrF;
    output.RrR = RrR;
    output.Raero = Raero;
    output.Fprop = Fprop;
    
else
    output = []; 
end

end

% Helper function to find intersection of line with road surface
function t = find_road_intersection(p, n_chassis, road)
    % Line: p + t * n_chassis
    % Road: y = road.h(x)
    % Solve: p(2) + t * n_chassis(2) = road.h(p(1) + t * n_chassis(1))
    
    % Define function to solve: f(t) = 0
    f = @(t) p(2) + t * n_chassis(2) - road.h(p(1) + t * n_chassis(1));
    
    % Initial guess
    t = 0;
    
    % Newton-Raphson iteration
    for k = 1:20
        % Evaluate function and derivative
        ft = f(t);
        if abs(ft) < 1e-12
            break;
        end
        
        % Numerical derivative
        dt = 1e-6;
        ft_plus = f(t + dt);
        ft_minus = f(t - dt);
        dfdt = (ft_plus - ft_minus) / (2 * dt);
        
        % Avoid division by zero
        if abs(dfdt) < 1e-12
            t = t - ft * 0.1; % Simple step if derivative is too small
        else
            t = t - ft / dfdt; % Newton step
        end
        
        % Prevent unrealistic values
        if abs(t) > 1000
            t = 0;
            break;
        end
    end
end
function [zuf0, zur0, Y0, etaF, etaR, deltaF, deltaR] = static_sag(P, X0, th0, road)

    % Static equilibrium for rigid chassis on spring-mass-spring systems
    % Ground → Spring1 (tire) → Unsprung Mass → Spring2 (suspension) → Chassis
    
    % Road normal at X0
    s0  = road.hpC(X0);  d0 = sqrt(1+s0^2);
    n0  = [-s0; 1]/d0;   ny0 = n0(2);
    u0  = [cos(th0); sin(th0)];

    % Gravity component normal to road
    g_normal = P.g * ny0;
    
    % Load split for chassis (beam theory)
    L = P.lf + P.lr;
    Fsf_ch = (P.lr / L) * P.ms * g_normal;  % Chassis load on front spring
    Fsr_ch = (P.lf / L) * P.ms * g_normal;  % Chassis load on rear spring

    % Suspension spring deflections
    etaF = abs(Fsf_ch) / P.ksf;
    etaR = abs(Fsr_ch) / P.ksr;

    % Tire compressions
    deltaF = abs(P.muf * g_normal - P.ksf * etaF) / P.ktf;
    deltaR = abs(P.mur * g_normal - P.ksr * etaR) / P.ktr;

    % Pick-up positions
    pFx = X0 + P.lf * u0(1);
    pRx = X0 - P.lr * u0(1);

    % Total length
    Lf = P.LsF + P.LtF - etaF - deltaF;
    Lr = P.LsR + P.LtR - etaR - deltaR;

    % Abcissa
    rFx = pFx + Lf * u0(2);
    rRx = pRx + Lr * u0(2);

    % Absolute y position of the unsprung mass
    zuf0 = road.h(rFx) + (P.LtF - deltaF) * n0(2);
    zur0 = road.h(rRx) + (P.LtR - deltaR) * n0(2);

    % Chassis position (from spring deflections)
    % Chassis center is at weighted average of suspension deflections
    Y0 = (P.lf/L) * (zuf0 + (P.LsF - etaF) * n0(2)) + ...
         (P.lr/L) * (zur0 + (P.LsR - etaR) * n0(2));

end

function results = ComputeResults(t, y, P, road)

% Display
disp('Post-processing results for plotting & animation...')

% Downsample y & t
% -------------------------------------------------------------------------

% Print interval
t_print = 1e-2;

% Number of time steps
nt = ceil(t(end) / t_print);

% Time vector
time = linspace(0, nt * t_print, nt + 1);

% Downsample y
y = interp1(t,y,time,"linear","extrap");

% Downsample t
t = time;

% Get results
% -------------------------------------------------------------------------

% Results time
results.time = t;

% Initialize
results.uRx = zeros(length(t),1);
results.uRy = zeros(length(t),1);
results.etaF = zeros(length(t),1);
results.etaR = zeros(length(t),1);
results.FsF = zeros(length(t),1);
results.FsR = zeros(length(t),1);
results.deltaF = zeros(length(t),1);
results.deltaR = zeros(length(t),1);
results.Nf = zeros(length(t),1);
results.Nr = zeros(length(t),1);

% Loop over all time steps
for i = 1 : length(t)
   

    % Compute aux variables
    [~, output] = ComputeAccelerations(t(i), y(i,:), P, road, 0, 1);

    % Store
    results.uRx(i) = output.uRx;
    results.uRy(i) = output.uRy;
    results.etaF(i) = output.etaF;
    results.etaR(i) = output.etaR;
    results.FsF(i) = output.FsF;
    results.FsR(i) = output.FsR;
    results.deltaF(i) = output.deltaF;
    results.deltaR(i) = output.deltaR;
    results.Nf(i) = output.Nf;
    results.Nr(i) = output.Nr;

end

% Display
disp('Post-processing completed.')
disp(' ');

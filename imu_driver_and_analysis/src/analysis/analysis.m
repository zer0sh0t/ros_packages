load('Allan_Variance', 'gyro_z', 'Fs')
t0 = 1/Fs;
theta = cumsum(gyro_z, 1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));
adev = sqrt(avar);

slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);

slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);

slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')

% loglog(tau, avar)

title('Allan Deviation with Noise Parameters of Angular Velocity about Z')
% title('Allan Variance of Angular Velocity about Z')

xlabel('\tau (sec)')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)^2$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')

% legend('$\sigma (mg)^2$', '$\sigma_N ((mg)/\sqrt{Hz})$', ...
%    '$\sigma_K ((mg)\sqrt{Hz})$', '$\sigma_B (mg)$', 'Interpreter', 'latex')

text(tauParams, params, {'N', 'K', 'B'})
grid on
axis equal

load('SimulatedSingleAxisGyroscope', 'omegaSim')

[avarSim, tauSim] = allanvar(omegaSim, 'octave', Fs);
adevSim = sqrt(avarSim);
adevSim = mean(adevSim, 2); % Use the mean of the simulations.

figure
loglog(tau, adev, tauSim, adevSim, '--')
title('Allan Deviation of HW and Simulation of Angular Velocity about Z')
xlabel('\tau (sec)');
ylabel('\sigma(\tau)')
legend('HW', 'SIM')
grid on
axis equal\
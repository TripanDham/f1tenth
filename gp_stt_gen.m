%% Train GP to fit data and get \mu and \rho

delete 'STT.csv'

clc;
clear;
clf;

A = readmatrix('pose_data.csv');
nt = 1;
A = A(1:nt:end,:);
A = A(800:1100, :);
td = A(:,1);
td = td - td(1);
n = length(td);
x = A(:,2);
y = A(:,3);
th = A(:,4);
th = unwrap(th);

tg = td;
xg = x;
yg = y;

d = 0.2;
for i = 1
    tg = [tg; flip(td)];
    xg = [xg; flip(x) + 2*d*rand(n,1)-d];
    yg = [yg; flip(y) + 2*d*rand(n,1)-d];
end

%% GP training     
sigma0 = 0.2;
kparams0 = [3.5, 6.2];

% GPR for x
gprMdlx = fitrgp(tg, xg, 'KernelFunction', 'squaredexponential', ...
    'KernelParameters', kparams0, 'Sigma', sigma0);
xpred = resubPredict(gprMdlx);

% GPR for y
gprMdly = fitrgp(tg, yg, 'KernelFunction', 'squaredexponential', ...
    'KernelParameters', kparams0, 'Sigma', sigma0);
ypred = resubPredict(gprMdly);

%% Plot results
subplot(2,2,1)
plot(tg, xg, 'k.'); hold on;
plot(tg, xpred, 'g', 'LineWidth', 1.5);
xlabel('t');
ylabel('x');
% legend('Data','GPR predictions');

subplot(2,2,2)
plot(tg, yg, 'k.'); hold on;
plot(tg, ypred, 'g', 'LineWidth', 1.5);
xlabel('t');
ylabel('y');
% legend('Data','GPR predictions');

subplot(2,2,3)
plot(xg, yg, 'k.'); hold on;
plot(xpred, ypred, 'g', 'LineWidth', 1.5);
xlabel('x');
ylabel('y');
% legend('Data','GPR predictions');

%% Kernel Information for X
sigma = gprMdlx.KernelInformation.KernelParameters;
sigmaL = sigma(1);
sigmaF = sigma(2);
N = length(xg);

% Covariance matrix K for X
K = zeros(N, N);
for i = 1:N
    for j = 1:N
        K(i, j) = sigmaF^2 * exp( -0.5*(tg(i)-tg(j))^2 / sigmaL^2 );
    end
end

I = eye(N);
rhoF = 2;

Nt = length(td);
mu_t = zeros(Nt, 1);
rho_t = zeros(Nt, 1);
for i = 1:Nt
    % Covariance Vector
    kb_t = kbar(tg, td(i), sigmaL, sigmaF);
    ktt = sigmaF^2 * exp( -0.5*(td(i)-td(i))^2 / sigmaL^2 );

    mu_t(i) = kb_t' * inv(K + rhoF^2 * I) * xg;
    rho_t(i) = sqrt(ktt - kb_t' * inv(K + rhoF^2 * I) * kb_t);
end

subplot(2,2,1)
plot(td, mu_t + rho_t, 'r-', 'LineWidth', 2); hold on;
plot(td, mu_t - rho_t, 'b-', 'LineWidth', 2); hold on;

STT = [td, mu_t - rho_t, mu_t + rho_t];
% writematrix(STT_X, 'STT_X.csv')

%% Kernel Information for Y
sigma = gprMdly.KernelInformation.KernelParameters;
sigmaL = sigma(1);
sigmaF = sigma(2);
N = length(xg);

% Covariance matrix K for Y
K = zeros(N, N);
for i = 1:N
    for j = 1:N
        K(i, j) = sigmaF^2 * exp( -0.5*(tg(i)-tg(j))^2 / sigmaL^2 );
    end
end

I = eye(N);
rhoF = 1;

Nt = length(td);
mu_t = zeros(Nt, 1);
rho_t = zeros(Nt, 1);
for i = 1:Nt
    % Covariance Vector
    kb_t = kbar(tg, td(i), sigmaL, sigmaF);
    ktt = sigmaF^2 * exp( -0.5*(td(i)-td(i))^2 / sigmaL^2 );

    mu_t(i) = kb_t' * inv(K + rhoF^2 * I) * yg;
    rho_t(i) = sqrt(ktt - kb_t' * inv(K + rhoF^2 * I) * kb_t);
end

subplot(2,2,2)
plot(td, mu_t + rho_t, 'r-', 'LineWidth', 2); hold on;
plot(td, mu_t - rho_t, 'b-', 'LineWidth', 2); hold on;

STT = [STT, mu_t - rho_t, mu_t + rho_t];
writematrix(STT, 'STT.csv')

%% Covariance vector function
function kb = kbar(tg, t, sigmaL, sigmaF)
    N = length(tg);
    kb = zeros(N, 1);
    for i = 1:N
        kb(i) = sigmaF^2 * exp( -0.5*(tg(i)-t)^2 / sigmaL^2 );
    end
end

%% plot_pid.m  --  plot micromouse PID telemetry captured over UART
%  Log is comma-separated plain text, one header row:  sp,pvL,pvR,mvL,mvR
%  Capture with TeraTerm:  File > Log, "Plain text" on, "Timestamp" off.

file        = 'pidlog.txt';   % <-- whatever TeraTerm saved (extension irrelevant)
print_every = 20;             % MOTION_TEST_PRINT_EVERY in firmware
loop_hz     = 1000;           % control loop rate (Hz)

dt = print_every / loop_hz;              % s per logged sample (0.02 s @ 50 Hz)

%% Load ------------------------------------------------------------------
raw = readmatrix(file);                  % delimiter auto-detected; header row skipped
raw = raw(all(~isnan(raw), 2), :);       % drop any garbled / partial lines
if isempty(raw)
    error('No numeric rows in %s -- check the file and that timestamps are off.', file);
end

t   = (0:size(raw,1)-1)' * dt;
sp  = raw(:,1);
pvL = raw(:,2);
pvR = raw(:,3);
mvL = raw(:,4);
mvR = raw(:,5);

%% Plot ------------------------------------------------------------------
figure('Name','PID telemetry','Color','w');

ax1 = subplot(2,1,1);
plot(t, sp,  'k--', 'LineWidth',1.2, 'DisplayName','SP');       hold on
plot(t, pvL, 'b',   'LineWidth',1.0, 'DisplayName','PV left');
plot(t, pvR, 'r',   'LineWidth',1.0, 'DisplayName','PV right');
ylabel('speed (CPS)'); title('Setpoint vs. actual');
legend('Location','southeast'); grid on

ax2 = subplot(2,1,2);
plot(t, mvL, 'b', 'LineWidth',1.0, 'DisplayName','MV left');    hold on
plot(t, mvR, 'r', 'LineWidth',1.0, 'DisplayName','MV right');
yline( 100, ':', 'sat +'); yline(-100, ':', 'sat -');
ylabel('output (% duty)'); xlabel('time (s)'); title('Controller effort');
legend('Location','southeast'); grid on

linkaxes([ax1 ax2], 'x');                % shared time axis: zoom one, both follow

%% Quick tuning metrics  (steady state = last 20% of the run) ------------
n  = size(raw,1);
ss = max(1, round(0.8*n)) : n;
fprintf('\n--- steady-state (last %d samples) ---\n', numel(ss));
fprintf('        SP = %6.0f CPS\n', mean(sp(ss)));
fprintf(' left   PV = %6.0f   err = %+5.0f   MV = %5.1f%%\n', ...
        mean(pvL(ss)), mean(sp(ss))-mean(pvL(ss)), mean(mvL(ss)));
fprintf(' right  PV = %6.0f   err = %+5.0f   MV = %5.1f%%\n', ...
        mean(pvR(ss)), mean(sp(ss))-mean(pvR(ss)), mean(mvR(ss)));
if mean(sp) > 0
    fprintf(' overshoot: L = %+4.1f%%   R = %+4.1f%%\n', ...
        (max(pvL)/mean(sp(ss))-1)*100, (max(pvR)/mean(sp(ss))-1)*100);
end
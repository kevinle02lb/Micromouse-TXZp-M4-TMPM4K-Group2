%% capture_pid.m  --  capture micromouse PID telemetry from UART into a CSV
%  Reads the comma separated stream emitted by MotionTest_StreamRow in
%  ModuleTest.c, writes every row to a CSV file as it arrives, and plots
%  setpoint, process variable and controller output while the run proceeds.
%  The CSV it produces is the input format plot_pid.m already reads.

port        = "COM5";         % <-- serial port, see serialportlist
duration_s  = 15;             % capture length
print_every = 20;             % MOTION_TEST_PRINT_EVERY in firmware
loop_hz     = 1000;           % control loop rate (Hz)

baud     = 115200;            % UART_Init: 8-N-1, 115191 Bd
expected = ["sp" "pvL" "pvR" "mvL" "mvR"];
dt       = print_every / loop_hz;
outfile  = sprintf('pidlog_%s.csv', string(datetime("now"), "yyyyMMdd_HHmmss"));

%% Open the port --------------------------------------------------------
s = serialport(port, baud);
configureTerminator(s, "CR/LF");        % UART_CRLF sends CR then LF
s.Timeout = 3;
closePort = onCleanup(@() delete(s));   % releases COM even on error or Ctrl-C
flush(s);

fprintf('%s at %d Bd. Waiting for the header row, press RESET on the robot.\n', ...
        port, baud);

%% Wait for the header --------------------------------------------------
%  A line whose fields are not all numeric is the header. Numeric lines seen
%  before it are the remains of a stream already in progress and are dropped.
header = strings(0);
while isempty(header)
    line = readline(s);
    if ismissing(line)
        error('No data on %s. Check the port, the wiring, and that no terminal holds it.', port);
    end
    fields = split(strtrim(line), ",");
    if any(isnan(str2double(fields)))
        header = fields';
    end
end

if ~isequal(header, expected)
    error('Header is "%s", expected "%s". Flash a MOTION_TEST build.', ...
          join(header, ","), join(expected, ","));
end

%% Prepare the CSV and the figure ---------------------------------------
fid = fopen(outfile, 'w');
closeFile = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', join(header, ","));

fig = figure('Name', 'PID telemetry (live)', 'Color', 'w');

ax1 = subplot(2,1,1);
hold(ax1, 'on'); grid(ax1, 'on');
hSP  = animatedline(ax1, 'Color','k', 'LineStyle','--', 'LineWidth',1.2, 'DisplayName','SP');
hPVL = animatedline(ax1, 'Color','b', 'LineWidth',1.0, 'DisplayName','PV left');
hPVR = animatedline(ax1, 'Color','r', 'LineWidth',1.0, 'DisplayName','PV right');
ylabel(ax1, 'speed (CPS)'); title(ax1, 'Setpoint vs. actual');
legend(ax1, 'Location', 'southeast');

ax2 = subplot(2,1,2);
hold(ax2, 'on'); grid(ax2, 'on');
hMVL = animatedline(ax2, 'Color','b', 'LineWidth',1.0, 'DisplayName','MV left');
hMVR = animatedline(ax2, 'Color','r', 'LineWidth',1.0, 'DisplayName','MV right');
yline(ax2,  100, ':', 'sat +');
yline(ax2, -100, ':', 'sat -');
ylabel(ax2, 'output (% duty)'); xlabel(ax2, 'time (s)');
title(ax2, 'Controller effort');
legend(ax2, 'Location', 'southeast');

linkaxes([ax1 ax2], 'x');
xlim(ax1, [0 duration_s]);

%% Stream ---------------------------------------------------------------
raw = zeros(0, numel(expected));
k   = 0;
t0  = tic;

while toc(t0) < duration_s && isvalid(fig)
    line = readline(s);
    if ismissing(line)
        break                                    % transmitter stopped
    end

    values = str2double(split(strtrim(line), ","))';
    if numel(values) ~= numel(expected) || any(isnan(values))
        continue                                 % partial or garbled row
    end

    k = k + 1;
    raw(k,:) = values;
    fprintf(fid, '%g,%g,%g,%g,%g\n', values);    % flushed row by row

    t = (k - 1) * dt;
    addpoints(hSP,  t, values(1));
    addpoints(hPVL, t, values(2));
    addpoints(hPVR, t, values(3));
    addpoints(hMVL, t, values(4));
    addpoints(hMVR, t, values(5));

    if mod(k, 10) == 0
        drawnow limitrate
    end
end

drawnow
clear closeFile closePort                        % close the file, release COM

if k == 0
    error('Header received but no data rows followed.');
end
fprintf('%d rows captured to %s\n', k, outfile);

%% Quick tuning metrics  (steady state = last 20% of the run) ------------
sp  = raw(:,1);  pvL = raw(:,2);  pvR = raw(:,3);
mvL = raw(:,4);  mvR = raw(:,5);

ss = max(1, round(0.8*k)) : k;
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

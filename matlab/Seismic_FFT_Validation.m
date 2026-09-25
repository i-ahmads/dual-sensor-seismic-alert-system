%% FFT Detection Pipeline Validation (mirrors ESP32 firmware)
clear; clc; close all;

%% Firmware-matching parameters
fs = 100;                   % Hz, ADC sample rate
N  = 256;                   % samples per buffer
Tbuf = N/fs;                 % 2.56 s per buffer
df = fs/N;                   % 0.390625 Hz/bin

k_low  = ceil(1/df);         % bin 3  -> 1.17 Hz
k_high = floor(10/df);       % bin 25 -> 9.77 Hz
band_bins = k_low:k_high;    % 23 bins, matches Subsystem 3.4 spec

fprintf('Bin spacing df = %.6f Hz\n', df);
fprintf('Seismic band bins: %d to %d (%d bins, %.3f-%.3f Hz)\n', ...
    k_low, k_high, numel(band_bins), k_low*df, k_high*df);

t = (0:N-1)'/fs;   % 2.56 s time vector, one buffer

%% Hamming window (moved up: needed by earthquake window-selector below) ----
n = (0:N-1)';
w = 0.54 - 0.46*cos(2*pi*n/(N-1));

%% Signal library 
signals = struct();

% 1. Real earthquake (ObsPy BW.RJOB) - robust reader, selects window by IN-BAND energy
try
    raw = readmatrix('quake_signal.txt');

    if size(raw,2) >= 2
        tcol = raw(:,1);
        vcol = raw(:,2);
        dt = diff(tcol);
        if std(dt) < 0.05*mean(dt)   % looks like a time axis
            quakeFull = vcol;
        else                          % columns swapped in the export
            quakeFull = tcol;
        end
    else
        quakeFull = raw(:,1);
    end

    quakeFull = quakeFull(:);
    Nfull = numel(quakeFull);

    if Nfull < N
        error('quake_signal.txt has only %d samples, need at least %d', Nfull, N);
    end

    fprintf('quakeFull range: min=%.4g, max=%.4g\n', min(quakeFull), max(quakeFull));

    % slide a length-N window across the record, keep the one with max IN-BAND energy
    bestBandE = -inf; bestStart = 1;
    for s = 1:(Nfull-N+1)
        seg = quakeFull(s:s+N-1) .* w;
        Yseg = fft(seg, N);
        magSeg = abs(Yseg(1:N/2+1));
        E = sum(magSeg(band_bins+1).^2);
        if E > bestBandE
            bestBandE = E; bestStart = s;
        end
    end

    signals.Earthquake = quakeFull(bestStart:bestStart+N-1);
    fprintf('Selected by band energy: samples %d-%d (of %d), in-band energy = %.4g\n', ...
        bestStart, bestStart+N-1, Nfull, bestBandE);

catch ME
    warning('quake_signal.txt read failed (%s) - using placeholder burst instead', ME.message);
    signals.Earthquake = 0.4*exp(-((t-1.2)/0.5).^2) .* ...
        (sin(2*pi*3*t) + 0.6*sin(2*pi*6*t) + 0.3*sin(2*pi*8*t));
end

% 2. Construction / pile driving - Eq. 8
signals.Construction_10Hz = 0.105*sin(2*pi*10*t);

% 3. Door slam - Eq. 10
tau = 0.05;
signals.DoorSlam_20Hz = 0.334*exp(-t/tau).*sin(2*pi*20*t);

% 4. Train - Eq. 11
signals.Train_15Hz = 0.00153*sin(2*pi*15.18*t);

% 5. Body massager - out-of-band check
signals.Massager_53Hz = 0.05*sin(2*pi*53*t);

%%  Run FFT + band-energy on every signal 
names = fieldnames(signals);
figure('Name','FFT Detection Pipeline - Signal Comparison');

for i = 1:numel(names)
    y  = signals.(names{i});
    yw = y .* w;
    Y  = fft(yw, N);
    mag = abs(Y(1:N/2+1));
    f   = (0:N/2)*df;

    bandEnergy = sum(mag(band_bins+1).^2);

    subplot(numel(names),2,2*i-1);
    plot(t, y); title([strrep(names{i},'_',' ') ' - time domain']);
    xlabel('t (s)'); ylabel('accel (g)'); grid on;

    subplot(numel(names),2,2*i);
    stem(f, mag, 'filled'); hold on;
    xline(1,'r--'); xline(10,'r--');
    title(sprintf('FFT | band energy = %.4g', bandEnergy));
    xlabel('f (Hz)'); ylabel('|Y(f)|'); xlim([0 25]); grid on;

    fprintf('%-20s band energy (bins %d-%d) = %.6g\n', ...
        names{i}, k_low, k_high, bandEnergy);
end

%% ---- Summary table ----
fprintf('BAND ENERGY SUMMARY');
resultNames = fieldnames(signals);
resultEnergies = zeros(numel(resultNames),1);

for i = 1:numel(resultNames)
    y  = signals.(resultNames{i});
    yw = y .* w;
    Y  = fft(yw, N);
    mag = abs(Y(1:N/2+1));
    resultEnergies(i) = sum(mag(band_bins+1).^2);
end

T = table(resultNames, resultEnergies, ...
    'VariableNames', {'Signal','BandEnergy'});
disp(T)

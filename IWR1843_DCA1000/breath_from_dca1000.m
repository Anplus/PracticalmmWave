clear dca; close all; clc;

% ----------------- Connect to TI Radar board + DCA1000EVM -----------------
dca = dca1000("IWR1843BOOST");

% Sampling rate (Hz) and FMCW slope (Hz/s) from device properties
fs         = dca.ADCSampleRate * 1e3;     % kHz -> Hz
sweepSlope = dca.SweepSlope     * 1e12;   % GHz/us -> Hz/s
fc         = dca.CenterFrequency * 1e9;   % GHz -> Hz
lambda     = physconst('LightSpeed')/fc;  % wavelength

% Number of ADC samples (range-FFT length)
nr = dca.SamplesPerChirp;

% Range response object (FFT in fast-time)
rangeresp = phased.RangeResponse( ...
    "RangeMethod","FFT", ...
    "RangeFFTLengthSource","Property", ...
    "RangeFFTLength",nr, ...
    "SampleRate",fs, ...
    "SweepSlope",sweepSlope, ...
    "ReferenceRangeCentered",false);

% Prime the stream once (first call configures HW and can be slower)
iqData = dca();  %#ok<NASGU>

% ----------------- Breathing parameters -----------------
targetRange  = 1.00;           % meters (subject is ~1 m away)
rangeLockWin = 0.5;            % +/- meters search window around 1 m
respBandHz   = [0.1 0.5];      % 6–30 BPM
estWindowSec = 20;             % rolling window for estimation
printEverySec = 2;             % console print interval

% ----------------- Visualization -----------------
figure('Name','Breath Sensing'); 

ax1 = subplot(2,1,1); % Top subplot: displacement
grid(ax1,'on'); 
xlabel(ax1,'Time (s)'); 
ylabel(ax1,'Displacement (mm)'); 
title(ax1,'Breath displacement over time');

ax2 = subplot(2,1,2); % Bottom subplot: spectrum
grid(ax2,'on'); 
xlabel(ax2,'Frequency (Hz)'); 
ylabel(ax2,'PSD'); 
title(ax2,'Respiration Spectrum');

% ----------------- Buffers for slow-time -----------------
t0 = tic;
tFrame = [];               % wall-clock timestamp per frame (s)
phaseTrack = [];           % phase at locked bin (radians)
dispTrack  = [];           % displacement (meters)
lastPrint = 0;

% ----------------- Main loop -----------------
stopTime = 100; % seconds
while toc(t0) < stopTime
    % Frame: [numSamples x numRx x numChirps]
    cube = dca();

    % Use RX1 for simplicity; you can average across RX for SNR if desired:
    iq = squeeze(cube(:,1,:));     % [nr x nChirp]

    % Range FFT (complex), get range grid as well
    [Rresp, Rgrid] = rangeresp(iq);   % Rresp: [nr x nChirp] complex range profiles

    % Magnitude range profile (avg over chirps) for robust locking
    magR = mean(abs(Rresp), 2);

    % Lock the range bin near 1 m (search within a +/- window)
    mask = abs(Rgrid - targetRange) <= rangeLockWin;
    if ~any(mask)
        [~, lockIdx] = max(magR);
    else
        [~, rel] = max(magR .* mask);
        lockIdx = rel;
    end
    lockedRange = Rgrid(lockIdx);

    % Coherently average the complex bin across chirps -> one complex sample per frame
    sFrame = mean(Rresp(lockIdx, :), 2);   % complex scalar

    % Append wall-clock timestamp and phase
    tFrame(end+1,1) = toc(t0); %#ok<SAGROW>
    phaseTrack(end+1,1) = angle(sFrame); %#ok<SAGROW>

    % Unwrap phase to displacement (two-way path: λ / (4π))
    phUnw = unwrap(phaseTrack);
    dispTrack = (lambda/(4*pi)) * (phUnw - mean(phUnw));  % meters, zero-mean

    % ----------------- CLEAN DISPLAY FILTER (added) -----------------
    % Compute slow-time rate from timestamps
    dispPlot = dispTrack;   % default
    if numel(tFrame) >= 5
        fs_slow_plot = 1/median(diff(tFrame));            % frames per second
        try
            % Zero-phase band-pass 0.1–0.5 Hz to show only breathing motion
            dplot = designfilt('lowpassiir','FilterOrder',4, ...
                    'HalfPowerFrequency',0.6, ...
                    'SampleRate',fs_slow_plot);
            dispPlot = filtfilt(dplot, dispTrack);
        catch
            % Fallback: short moving-average smoothing if designfilt unavailable
            w = max(3, round(0.5*fs_slow_plot));          % ~0.5 s window
            dispPlot = movmean(dispTrack, w);
        end
    end

    % --- Update displacement subplot (filtered curve) ---
    plot(ax1, tFrame - tFrame(1), dispPlot*1e3, 'LineWidth', 1.4);
    xlim(ax1, [0, max(60, tFrame(end)-tFrame(1))]);  % show up to full duration (>=60 s)
    title(ax1, sprintf('Breath displacement (filtered) — locked at %.2f m', lockedRange));

    % ----------------- Estimate respiration rate (BPM) -----------------
    % Keep a rolling window
    keep = tFrame >= (tFrame(end) - estWindowSec - 1);
    tWin = tFrame(keep);
    dWin = dispTrack(keep);

    if numel(tWin) > 20
        % Estimate sample rate of the slow-time (frames per second)
        fs_slow = 1/median(diff(tWin));

        % Band-pass 0.1–0.5 Hz to isolate respiration
        try
            d = designfilt('bandpassiir','FilterOrder',4, ...
                'HalfPowerFrequency1',respBandHz(1),'HalfPowerFrequency2',respBandHz(2), ...
                'SampleRate',fs_slow);
            x = filtfilt(d, dWin);
        catch
            % Fallback: detrend if DSP System Toolbox is unavailable
            x = detrend(dWin);
        end

        % --- Robust PSD (Welch) with short-buffer handling ---
        npts = numel(x);
        if npts < 8
            f_resp = NaN; bpm = NaN; Pxx = []; f = [];
        else
            L = round(4*fs_slow);                 % target segment length (samples)
            L = max(16, min(L, npts));            % clip to [16, npts]
            if mod(L,2)==1, L = L-1; end          % make even
            noverlap = floor(0.5*L);              % 50% overlap
            welchWin = hamming(L,'periodic');

            nfft = max(256, 2^nextpow2(min(npts, 4096)));

            if npts < L
                [Pxx, f] = periodogram(x, hamming(npts,'periodic'), nfft, fs_slow);
            else
                [Pxx, f] = pwelch(x, welchWin, noverlap, nfft, fs_slow);
            end

            % Find resp peak in 0.1–0.5 Hz
            band = (f >= respBandHz(1) & f <= respBandHz(2));
            if any(band)
                [~, pk] = max(Pxx(band));
                f_resp = f(find(band,1,'first') + pk - 1);
                bpm = 60*f_resp;
            else
                f_resp = NaN; bpm = NaN;
            end
        end

        % --- Update spectrum subplot ---
        cla(ax2);
        if ~isempty(Pxx)
            plot(ax2, f, Pxx, 'LineWidth', 1.2); 
            xlim(ax2, [0, 1.0]);
            if ~isnan(f_resp)
                yl = ylim(ax2); 
                hold(ax2,'on'); 
                plot(ax2,[f_resp f_resp], yl, '--r'); 
                hold(ax2,'off');
                title(ax2, sprintf('Resp peak: %.2f Hz (%.1f BPM)', f_resp, bpm));
            else
                title(ax2,'Resp peak: N/A');
            end
        else
            title(ax2,'Collecting data…');
        end

        % Console update every few seconds
        if (tFrame(end) - lastPrint) > printEverySec && ~isnan(bpm)
            amp_mm = (max(x) - min(x))*1e3; % peak-to-peak over window
            fprintf('[LIVE] Respiration: %.1f BPM (%.2f Hz), Amp ≈ %.1f mm @ R≈%.2f m\n', ...
                bpm, f_resp, amp_mm, lockedRange);
            lastPrint = tFrame(end);
        end
    end

    drawnow limitrate;
end

% Optional: release streaming when done
try dca.release; catch, end

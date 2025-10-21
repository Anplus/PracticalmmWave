clear dca
% Create connection to TI Radar board and DCA1000EVM Capture card
dca = dca1000("IWR1843BOOST");

% Define a variable to set the sampling rate in Hz for the
% phased.RangeResponse object. Because the dca1000 object provides the
% sampling rate in kHz, convert this rate to Hz.
fs = dca.ADCSampleRate*1e3;
% Define a variable to set the FMCW sweep slope in Hz/s for the
% phased.RangeResponse object. Because the dca1000 object provides the
% sweep slope in GHz/us, convert this sweep slope to Hz/s.
sweepSlope = dca.SweepSlope * 1e12;
% Define a variable to set the number of range samples
nr = dca.SamplesPerChirp;
% Create phased.RangeResponse System object that performs range filtering
% on fast-time (range) data, using an FFT-based algorithm
rangeresp = phased.RangeResponse(RangeMethod = 'FFT',...
                                 RangeFFTLengthSource = 'Property',...
                                 RangeFFTLength = nr, ...
                                 SampleRate = fs, ...
                                 SweepSlope = sweepSlope, ...
                                 ReferenceRangeCentered = false);
% The first call of the dca1000 object may take longer due to the
% configuration of the radar and the DCA1000EVM. To exclude the configuration
% time from the loop's duration, make the first call to the dca1000 object
% before entering the loop.
iqData = dca();
% Specify the duration in seconds for which the loop should run
stopTime = 100;
% Start the stopwatch timer
ts = tic;
% Execute the loop until the stopTime specified is reached
while (toc(ts)<stopTime)
% Capture the ADC data (IQ data) from TI Radar board and DCA1000EVM
iqData = dca();
% Get the data from first receiver antenna
iqData = squeeze(iqData(:,1,:));
% Plot the range response corresponding to the the input signal, iqData.
plotResponse(rangeresp,iqData);
% Update figures
drawnow limitrate;
end
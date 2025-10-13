% Audio to Image Conversion

clc; clear all; close all;

DatasetPath = 'D:\SER\RAVDESS\Anger';

ads = audioDatastore(DatasetPath, 'IncludeSubfolders', true, 'LabelSource', 'foldernames');

Fs = 16000; % Target Sampling Frequency
%%

% Visualization

[audio_signal, fs] = audioread(ads.Files{21}); % read with original sampling frequency
[Numer, Denom] = rat(Fs/fs);
audio_signal = resample(audio_signal(:,1),Numer,Denom); % resample with 16 kHz sampling frequency
audio_signal = audio_signal-mean(audio_signal); % mean removal

signal_length = length(audio_signal);
fb = cwtfilterbank('SignalLength', signal_length,'Wavelet', 'amor', 'VoicesPerOctave', 12, 'SamplingFrequency', Fs);

ts = [0:(signal_length-1)]/Fs;
fs = Fs*(0:(signal_length/2-1))/signal_length;

[p,frq,t] = pspectrum(audio_signal, Fs, 'spectrogram', 'TimeResolution', 0.05);
frq = hz2mel(frq);

figure;
pcolor(t,frq,p)
xlabel('Time')
ylabel('Frequency')
title('PSpectrum with Mel Scale')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/128])


[pm,frm,tm] = melSpectrogram(audio_signal,Fs,'Window',hann(512,'periodic'),'OverlapLength',440,'FFTLength',1024,'NumBands',256);
frm = hz2mel(frm);

figure;
pcolor(tm,frm,pm)
xlabel('Time')
ylabel('Frequency')
title('Mel-Spectrogram')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/128])


[cfs,frs] = wt(fb,audio_signal);
frs = hz2mel(frs);

figure;
pcolor(ts,frs,abs(cfs))
xlabel('Time')
ylabel('Frequency')
title('CWT with Mel Scale')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/8])


[coeffs, delta, deltaDelta, loc] = mfcc(audio_signal, Fs, 'NumCoeffs', 40);

figure;
imagesc(loc, 1:40, coeffs.')
axis xy
title('MFCC Coefficients')
xlabel('Time (s)')
ylabel('Coefficient Index')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/4])

figure;
imagesc(loc, 1:40, delta.')
axis xy
title('Delta Coefficients')
xlabel('Time (s)')
ylabel('Coefficient Index')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/8])

figure;
imagesc(loc, 1:40, deltaDelta.')
axis xy
title('Delta-Delta Coefficients')
xlabel('Time (s)')
ylabel('Coefficient Index')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/16])

[cq, fq] = cqt(audio_signal, 'SamplingFrequency', Fs);
fq = hz2mel(fq);
szq = size(cq, 2);
tq = [0:szq-1];

figure;
pcolor(tq,fq,abs(cq))
xlabel('Time')
ylabel('Frequency')
title('CQT with Mel Scale')
shading interp
axis tight
ylim([1 fq(length(fq)/2+1)]);
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/8])

[s, f, t] = stft(audio_signal, Fs, FrequencyRange='onesided', Window=hann(512,'periodic'), OverlapLength=440, FFTLength=1024);
f = hz2mel(f);

figure;
pcolor(t,f,abs(s))
xlabel('Time')
ylabel('Frequency')
title('STFT with Mel Scale')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/8])

%{
[sp, fp, tp] = spectrogram(audio_signal, hann(512,'periodic'), 440, 1024, Fs, 'power', 'yaxis');
fp = hz2mel(fp);

figure;
pcolor(tp,fp,abs(sp))
xlabel('Time')
ylabel('Frequency')
title('Spectrogram with Mel Scale')
shading interp
axis tight
colormap(gray(256))
limy = caxis;
caxis([0 limy(2)/4]) 



%%

% Audio to Image

for i = 1:length(ads.Files); 
            [audio_signal, fs] = audioread(ads.Files{i}); % read with original sampling frequency
            [Numer, Denom] = rat(Fs/fs);
            audio_signal = resample(audio_signal(:,1),Numer,Denom); % resample with 16 kHz sampling frequency
            audio_signal = audio_signal-mean(audio_signal); % mean removal
            f = figure('visible','off');
            [p,frq,t] = pspectrum(audio_signal, Fs, 'spectrogram', 'TimeResolution', 0.05);
            frq = hz2mel(frq);
            pcolor(t,frq,p)
            shading interp
            axis tight
            colormap(parula(256))
            limy = caxis;
            caxis([0 limy(2)/128])
            axis off
            title ('')
            box off
            colorbar off
            filenameindex = i;
            filename = strcat(sprintf('%d.jpg', filenameindex));
            exportgraphics(f, filename);
end
%}
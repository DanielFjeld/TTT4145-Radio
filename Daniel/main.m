%%coments
% %% will give a new line to split up code
% clear all, will clear persistant variables wich may be stuck after last compile
clear all;

%% Parameters
resend = 10;
Message = 'Hello';
Number_size = 8; %int8_t
Number = 69; %number to be sent

Simulate = false;
RX_LOOP = true;
TX_LOOP = true;
TX_AWGN = false;

MessageLength = strlength(Message); 
SamplesPerSymbol = 12;



%% Setup

% Setup Receiver
rx = sdrrx('Pluto','OutputDataType','double','SamplesPerFrame',2^15);
rx.CenterFrequency = 916e6;
rx.BasebandSampleRate = 400000;
rx.SamplesPerFrame = 11226;
% Setup Transmitter
tx = sdrtx('Pluto','Gain',0);
tx.CenterFrequency = 916e6;
tx.Gain = 0;
tx.BasebandSampleRate = 400000;
rx.SamplesPerFrame = 11226;

%% Constellation diagrams
constDiagram1 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',3000,'Title','txData');
constDiagram2 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','filteredData');
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',30000,'Title','agcData');
constDiagram4 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',30000);
constDiagram5 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',30000);

%% Channel
channel = comm.AWGNChannel('EbNo',20,'BitsPerSymbol',2);
pfo = comm.PhaseFrequencyOffset( ...
    'PhaseOffset',180, ...
    'FrequencyOffset',1e3, ...
    'SampleRate',1e6);

%% Instantiate communication toolbox blocks
qpskmod = comm.QPSKModulator('BitInput',true);
qpskdemod = comm.QPSKDemodulator('BitOutput',true);

pMeanFreqOff = 0;
pCnt = 0;
pCoarseFreqEstimator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","Correlation-based",MaximumFrequencyOffset=6e3,SampleRate=200000);
pCoarseFreqCompensator = comm.PhaseFrequencyOffset("PhaseOffset",0,"FrequencyOffsetSource","Input port","SampleRate",200000);
symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01,DetectorGain=5.4,Modulation="PAM/PSK/QAM");
carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);

txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',2,'RolloffFactor',0.5,'FilterSpanInSymbols',10);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',2,RolloffFactor=0.5,FilterSpanInSymbols=10,DecimationFactor=2);

%txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',SamplesPerSymbol,'RolloffFactor',0.5);
%rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',SamplesPerSymbol,'DecimationFactor',SamplesPerSymbol,RolloffFactor=0.5);


coarseFrequencyCompensator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","Correlation-based",MaximumFrequencyOffset=6e3,SampleRate=200000);
%symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);
%carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);

%errorRate = comm.ErrorRate('ReceiveDelay',2);
agc = comm.AGC();
agc.DesiredOutputPower = 2;
agc.AveragingLength = 50;
agc.MaxPowerGain = 60;

%% create frame
barker = comm.BarkerCode("Length",13,SamplesPerFrame=13);
bar = barker();
Preamble = [bar; bar;];

% MessageBits

msgSet = zeros(resend * MessageLength, 1); 
for msgCnt = 0 : resend-1
    msgSet(msgCnt * MessageLength + (1 : MessageLength)) = ...
        sprintf('%s', Message);
end
seq = barker();
seq = max(0, seq); %%make barker bits between 0 and 1 
%seq = int2bit(seq, 2);

MsgTxOut = msgSet;

%% CRC Generation
CRCtxIn = MsgTxOut;
crcGen = comm.CRCGenerator('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
CRCtxBits = [int2bit(CRCtxIn, 7); int2bit(Number, Number_size);]; %CRC frame
CRCtxOut = crcGen(CRCtxBits);

%% hamming encoding
HammingTxIn = CRCtxOut;
k = size(HammingTxIn, 1);
r = ceil(log2(k));
% Adjust r until the condition is met: 2^r >= k + r + 1
while 2^r < k + r + 1
    r = r + 1;
end
% Calculate the total codeword length n
n = 2^r - 1;
k = n - r;

HammingTxOut = encode(HammingTxIn, n, k, 'hamming/binary');

%% Trells encoding, Veterbi

TrellisTxIn = [HammingTxOut; zeros(1,1);];
trellis = poly2trellis([4 3],[4 5 17;7 4 2]);
tbdepth = 5 * 3; % A common practice for traceback depth

TrellisTxOut = convenc(TrellisTxIn,trellis);

%%modulate, add noise and demodulate
%modulatedSignal = qpskmod(code); % QPSK modulation
%SNR = 5; % Signal-to-noise ratio in dB
%receivedSignal = awgn(modulatedSignal, SNR, 'measured');
%noiseVar = 1;
%softBits = qpskdemod(receivedSignal);

%% frame 
frameTxIn = TrellisTxOut;
if(mod(size(frameTxIn,1),2) == 1)%must be integer multiple of bits per symbol (2)
    MessageBits = [frameTxIn; zeros(1, 1);]; %need to add a zero at the end
else
    MessageBits = frameTxIn; 
end

frameTxOut = MessageBits; %frame

%% modululate from real to imaginary numbers and add preamble
modulateTxIn = frameTxOut;
msg = qpskmod(modulateTxIn);
ImPreamble = Preamble+Preamble*i;
padding = zeros(200, 1);
modSig = [padding; ImPreamble; msg; padding;]; %make signal imaginary

txData = txfilter(modSig); %lp filter (make transitions smooth)

rxSig = 0;

%% transmitt data
%transmitt on Adalm pluto

if(Simulate == false)
    rxSig = txData;
    if(TX_AWGN)
    offsetData = pfo(txData);
    rxSig = channel(offsetData);
    end
    tx.transmitRepeat(rxSig); %transmitt the data
    
%simulate
else
    offsetData = pfo(txData);
    rxSig = channel(offsetData);
end

%% ---- Receiver ----
while(RX_LOOP)
%agcData = agc(rx());
if(Simulate == false)
    filteredData = rxfilter(rx());
else
    filteredData = rxfilter(rxSig);
end

[~, freqOffsetEst] = pCoarseFreqEstimator(filteredData);   % Coarse frequency offset estimation
            % average coarse frequency offset estimate, so that carrier
            % sync is able to lock/converge
            freqOffsetEst = (freqOffsetEst + pCnt * pMeanFreqOff)/(pCnt+1);
            pCnt = pCnt + 1;            % update state
            pMeanFreqOff = freqOffsetEst;
            
coarseFreq = pCoarseFreqCompensator(filteredData,-freqOffsetEst);     
%coarseFreq = coarseFrequencyCompensator(filteredData); %frequency correction

%synchronizedSymbol = symbolSynchronizer(coarseFreq);
synchronizedCarrier = carrierSynchronizer(coarseFreq); %phase correction
%synchronizedCarrier = carrierSynchronizer(synchronizedSymbol); %phase correction

ImRxOut = synchronizedCarrier;

%find start of frame

%% detect frame start and phase offset
corr = xcorr(ImRxOut, ImPreamble); %%do correlation
L = length(corr);
[v,i] = max(corr); %i = start of index

%get amplitude and angle of correlation
amp = abs(v)
theta = atan2(imag(v),real(v))*180/pi

%apply offset
phaseOffset = comm.PhaseFrequencyOffset(PhaseOffset=-theta);


preambleIndex = (i-(L+1)/2)+1 %find start of preamble
phaseTxOut = ImRxOut(preambleIndex:end); %set start of data
phaseTxOut = phaseOffset(phaseTxOut);

rxOut = qpskdemod(phaseTxOut(27:end)); %generate bits from const diagram
rxOut = [rxOut(1:end); zeros(1000,1)];

EOF = size(TrellisTxOut, 1);
FrameDetectOut = rxOut(1:EOF);

%% Trells decoding, Veterbi
TrellsRxIn = FrameDetectOut;
TrellsRxOut = vitdec(TrellsRxIn, trellis, tbdepth, 'trunc', 'hard'); % The last parameter specifies the number of soft decision bits
%% hamming decoding
%%introduse error
%errLoc = randerr(1,n);
%HammingDecData = mod(HammingDecData + errLoc',2);

%calculation
HammingRxIn = TrellsRxOut(1:size(HammingTxOut, 1)); %get data

DetectedRxData = decode(HammingRxIn,n,k,'hamming/binary');
HammingRxOut = DetectedRxData(1:size(CRCtxOut, 1));

%% CRC check
CRCrxIn = HammingRxOut;
% Create a CRC detector with the same settings as the generator
crcDet = comm.CRCDetector('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
% Check the received data for CRC errors
[detectedData, errFlag] = crcDet(CRCrxIn);

%% reshape bits
% Extract the message bits after the Barker codes
reshapeRxIn = detectedData;
endOfMessage = MessageLength*7*resend;
messageBits = reshapeRxIn(1:endOfMessage);

% Reshape the message bits into 7-bit rows, assuming the total number of message bits is divisible by 7
% This might need adjustment based on how the bits are packed and the total length
messageBitsReshaped = reshape(messageBits, 7, [])'; %can be printed if you remove ; and add '

% Convert each 7-bit group to a character
decodedMessage = char(bin2dec(num2str(messageBitsReshaped)));  %can be printed if you remove ; and add '

%%Extract number
number_index_start = endOfMessage+1;
number_index_stop = number_index_start+Number_size-1;
rx_number_bits = DetectedRxData(number_index_start:number_index_stop);
rx_number = bit2int(rx_number_bits,Number_size);

%% ---- Error calculation ----
if(isequal(TrellsRxOut ,TrellisTxIn))
    disp('Trellis OK');
else
    disp('Trellis not equeal to input');
end
errors = biterr(TrellisTxOut, TrellsRxIn, [], 'column-wise');
errorRate = errors/size(TrellisTxOut, 1);
formatSpec = 'Before  Trells, error on %2d out of %2d bits = %.5f\n';
fprintf(formatSpec,errors, size(TrellisTxOut, 1), errorRate);

errors = biterr(TrellisTxIn, TrellsRxOut, [], 'column-wise');
errorRate = errors/size(TrellisTxIn, 1);
formatSpec = 'After   Trells, error on %2d out of %2d bits = %.5f\n\n';
fprintf(formatSpec,errors, size(TrellisTxIn, 1), errorRate);

%------------------------------------------------------
if(isequal(HammingTxIn ,HammingRxOut))
    disp('Hamming OK');
else
    disp('Hamming not equeal to input');
end
errors = biterr(HammingRxIn, HammingTxOut, [], 'column-wise');
errorRate = errors/size(HammingTxOut, 1);
formatSpec = 'Before Hamming, error on %2d out of %2d bits = %.5f\n';
fprintf(formatSpec,errors, size(HammingTxOut, 1), errorRate);

errors = biterr(HammingRxOut, HammingTxIn, [], 'column-wise');
errorRate = errors/size(HammingRxOut, 1);
formatSpec = 'After  Hamming, error on %2d out of %2d bits = %.5f\n\n';
fprintf(formatSpec,errors, size(HammingRxOut, 1), errorRate);

%------------------------------------------------------
if(errFlag)
    fprintf('CRC ERROR\n\n');
else
    fprintf('CRC OK\n\n');
end

%% Print data recived
formatSpec = '%s%d\n';
fprintf(formatSpec, decodedMessage, rx_number);

%% print diagrams
%constDiagram1(txData)
%constDiagram2(filteredData)
%constDiagram3(coarseFreq)
constDiagram4(synchronizedCarrier)
%constDiagram5(synchronizedSymbol) %dont know what this is

if(TX_LOOP)
else
    release(tx);
end

end
release(rx);

%% creating functions test
[y, x] = test(reshapeRxIn, MessageLength*resend, Number_size);
formatSpec = '%s%d\n';
%fprintf(formatSpec, y, x);

function [y, x] = test(reshapeRxIn, MessageLength, Number_size)
% Extract the message bits after the Barker codes
endOfMessage = MessageLength*7;
messageBits = reshapeRxIn(1:endOfMessage);

% Reshape the message bits into 7-bit rows, assuming the total number of message bits is divisible by 7
% This might need adjustment based on how the bits are packed and the total length
messageBitsReshaped = reshape(messageBits, 7, [])'; %can be printed if you remove ; and add '

% Convert each 7-bit group to a character
decodedMessage = char(bin2dec(num2str(messageBitsReshaped)));  %can be printed if you remove ; and add '

%%Extract number
number_index_start = endOfMessage+1;
number_index_stop = number_index_start+Number_size-1;
rx_number_bits = reshapeRxIn(number_index_start:number_index_stop);
rx_number = bit2int(rx_number_bits,Number_size);
y = decodedMessage;
x = rx_number;

end
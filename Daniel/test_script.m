%%coments
% %% will give a new line to split up code
% clear all, will clear persistant variables wich may be stuck after last compile
clear all;

%% Parameters
Message = 'Hello ';
Number_size = 8; %int8_t
Number = 69; %number to be sent

Simulate = true;

MessageLength = strlength(Message); 
SamplesPerSymbol = 12;



%% Setup Receiver
rx = sdrrx('Pluto','OutputDataType','double','SamplesPerFrame',2^15);
rx.CenterFrequency = 916e6;
rx.BasebandSampleRate = 400000;
rx.SamplesPerFrame = 11226;
% Setup Transmitter
tx = sdrtx('Pluto','Gain', 0);
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
channel = comm.AWGNChannel('EbNo',5,'BitsPerSymbol',2);
pfo = comm.PhaseFrequencyOffset( ...
    'PhaseOffset',40, ...
    'FrequencyOffset',1e3, ...
    'SampleRate',1e6);

%% Instantiate communication toolbox blocks
qpskmod = comm.QPSKModulator('BitInput',true);
qpskdemod = comm.QPSKDemodulator('BitOutput',true);
coarseFrequencyCompensator = comm.CoarseFrequencyCompensator("Modulation","QPSK","SamplesPerSymbol",SamplesPerSymbol);
symbolSynchronizer = comm.SymbolSynchronizer("Modulation","PAM/PSK/QAM","SamplesPerSymbol",SamplesPerSymbol);
carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","SamplesPerSymbol",SamplesPerSymbol);
txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',SamplesPerSymbol,'RolloffFactor',0.5);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',SamplesPerSymbol, ...
    'DecimationFactor',12,RolloffFactor=0.5);
errorRate = comm.ErrorRate('ReceiveDelay',2);
agc = comm.AGC();
agc.DesiredOutputPower = 2;
agc.AveragingLength = 50;
agc.MaxPowerGain = 60;

%% create frame
barker = comm.BarkerCode("Length",13,SamplesPerFrame=13);
bar = barker();
Preamble = [bar; bar;];

% MessageBits
resend = 1;
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
    MessageBits = [seq; seq; frameTxIn; zeros(1, 1);]; %need to add a zero at the end
else
    MessageBits = [seq; seq; frameTxIn;]; 
end

frameTxOut = MessageBits; %frame

%% modululate from real to imaginary numbers
modulateTxIn = frameTxOut;
msg = qpskmod(modulateTxIn);
ImPreamble = Preamble+Preamble*i
padding = zeros(160, 1);
modSig = [padding; ImPreamble; msg; padding;]; %make signal imaginary

txData = txfilter(modSig); %lp filter (make transitions smooth)

rxSig = 0;

%% transmitt data
%transmitt on Adalm pluto
if(Simulate == false)
    tx.transmitRepeat(txData); %transmitt the data
    
%simulate
else
    offsetData = pfo(txData);
    rxSig = channel(offsetData);
end

%% ---- Receiver ----

%agcData = agc(rx());
if(Simulate == false)
    filteredData = rxfilter(rx());
else
    filteredData = rxfilter(rxSig);
end

coarseFreq = coarseFrequencyCompensator(filteredData); %frequency correction
synchronizedCarrier = carrierSynchronizer(coarseFreq); %phase correction
synchronizedSymbol = symbolSynchronizer(coarseFreq);

ImRxOut = synchronizedCarrier;
rxOut = qpskdemod(ImRxOut); %generate bits from const diagram
%find start of frame

%% detect frame start
FrameDetectIn = rxOut;
bar = barker();
barkerPreamble = [seq; seq;];
% Perform cross-correlation between the noisy signal and the Barker sequence
corr = xcorr(ImRxOut, ImPreamble);

PreambleDetector = comm.PreambleDetector(barkerPreamble,"Input","Bit");
preambleIndex = PreambleDetector(FrameDetectIn)

L = length(corr)
[v,i] = max(corr) %i = start of index
preambleIndex = i*2-L + 77%L-i + 74 %i-(L+1)/2 %dont know if this is correct %want 298


sizeOfBarker = 13*2;
startOfFrame = preambleIndex+1;

endOfFrame = startOfFrame+size(CRCtxOut, 1)-1;

EOF = startOfFrame+size(TrellisTxOut, 1)-1;
FrameDetectOut = FrameDetectIn(startOfFrame:EOF);

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
%constDiagram4(synchronizedCarrier)
%constDiagram5(synchronizedSymbol) %dont know what this is

release(tx);
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

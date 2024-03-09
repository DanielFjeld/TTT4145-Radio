%%coments
% %% will give a new line to split up code

% clear all, will clear persistant variables wich may be stuck after last
% compile

%

clear all;

%% Parameters


Message = 'test';
Number_size = 8; %int8_t
Number = [69]; %number to be sent

Simulate = true;

MessageLength = strlength(Message); 
SamplesPerSymbol = 12;

%% Setup Receiver
rx = sdrrx('Pluto','OutputDataType','double','SamplesPerFrame',2^15);
rx.CenterFrequency = 916e6;
rx.BasebandSampleRate = 400000;
rx.SamplesPerFrame = 11226;
% Setup Transmitter
tx = sdrtx('Pluto','Gain', -0);
tx.CenterFrequency = 916e6;
tx.Gain = 0;
tx.BasebandSampleRate = 400000;
rx.SamplesPerFrame = 11226;

%% Constellation diagrams
constDiagram1 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','txData');
constDiagram2 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','filteredData');
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','agcData');
constDiagram4 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',10000);
constDiagram5 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',10);

%% Channel
channel = comm.AWGNChannel('EbNo',10,'BitsPerSymbol',2);
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



%% CRC Generation
crcGen = comm.CRCGenerator('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
CRCtxBits = [int2bit(msgSet, 7); int2bit(Number, Number_size);]; %CRC frame
CRCtxData = crcGen(CRCtxBits)

%% hamming encoding

k = size(CRCtxData, 1)
r = ceil(log2(k))
% Adjust r until the condition is met: 2^r >= k + r + 1
while 2^r < k + r + 1
    r = r + 1;
end
% Calculate the total codeword length n
n = 2^r - 1
k = n - r

HammingCode = encode(CRCtxData, n, k, 'hamming/binary');



%% frame 
zero = zeros(1000, 1);
if(mod(size(HammingCode,1),2) == 1)%must be integer multiple of bits per symbol (2)
    MessageBits = [seq; seq; HammingCode; zeros(1, 1);]; %need to add a zero at the end
else
    MessageBits = [seq; seq; HammingCode;]; 
end


Zero_padding = [zero; MessageBits; zero;]; %frame

%% modululate from real to imaginary numbers

modSig = qpskmod(Zero_padding); %make signal imaginary
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
synchronizedSymbol = symbolSynchronizer(synchronizedCarrier);
rxData = qpskdemod(synchronizedCarrier) %generate bits from const diagram
%find start of frame

%% detect frame start
bar = barker();
barkerPreamble = [bar; bar;];
% Perform cross-correlation between the noisy signal and the Barker sequence
corr = xcorr(rxData, barkerPreamble);
L = length(corr)
[v,i] = max(corr) %i = start of index

index = i-(L+1)/2 %dont know if this is correct


sizeOfBarker = 13*2;
startOfFrame = index+sizeOfBarker+1;

endOfFrame = startOfFrame+size(CRCtxData, 1)-1;

Hamming_EOF = startOfFrame+size(HammingCode, 1)-1;
HammingDecData = rxData(startOfFrame:Hamming_EOF);

%% hamming decoding
%%introduse error
%errLoc = randerr(1,n);
%HammingDecData = mod(HammingDecData + errLoc',2);

%calculation
size(HammingCode, 1);
HammingRxData = HammingDecData(1:size(HammingCode, 1)); %get data

DetectedRxData = decode(HammingCode,n,k,'hamming/binary');
DetectedRxData = DetectedRxData(1:size(CRCtxData, 1));

%% CRC check
% Create a CRC detector with the same settings as the generator
crcDet = comm.CRCDetector('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
% Check the received data for CRC errors
[detectedData, errFlag] = crcDet(DetectedRxData);

if(errFlag)
    disp('CRC ERROR');
else
    disp('CRC OK');
end

%% reshape bits
% Extract the message bits after the Barker codes
endOfMessage = MessageLength*7*resend;
messageBits = DetectedRxData(1:endOfMessage);

% Reshape the message bits into 7-bit rows, assuming the total number of message bits is divisible by 7
% This might need adjustment based on how the bits are packed and the total length
messageBitsReshaped = reshape(messageBits, 7, [])'; %can be printed if you remove ; and add '

% Convert each 7-bit group to a character
decodedMessage = char(bin2dec(num2str(messageBitsReshaped)));  %can be printed if you remove ; and add '

%% Extract number
number_index_start = endOfMessage+1;
number_index_stop = number_index_start+Number_size-1;
rx_number_bits = DetectedRxData(number_index_start:number_index_stop);
rx_number = bit2int(rx_number_bits,Number_size);

%% ---- Error calculation ----
errors = biterr(HammingRxData, HammingCode, [], 'column-wise');
errorRate = errors/size(HammingCode, 1);
formatSpec = 'Before Hamming, error on %2d out of %2d bits = %.5f\n';
fprintf(formatSpec,errors, size(HammingCode, 1), errorRate);

errors = biterr(CRCtxData, DetectedRxData, [], 'column-wise');
errorRate = errors/size(DetectedRxData, 1);
formatSpec = 'After  Hamming, error on %2d out of %2d bits = %.5f\n';
fprintf(formatSpec,errors, size(DetectedRxData, 1), errorRate);

%% Print data recived
formatSpec = '%s %d\n';
fprintf(formatSpec, decodedMessage, rx_number);

%% print diagrams
%constDiagram1(txData)
%constDiagram2(filteredData)
%constDiagram3(coarseFreq)
%constDiagram4(synchronizedCarrier)
%constDiagram5(synchronizedSymbol)
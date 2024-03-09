% Parameters
Message = "Hello world";
MessageLength = strlength(Message) + 5;
SamplesPerSymbol = 12;

% Setup Receiver
rx = sdrrx('Pluto','OutputDataType','double','SamplesPerFrame',2^15);
rx.CenterFrequency = 916e6;
rx.BasebandSampleRate = 400000;
rx.SamplesPerFrame = 11226;
% Setup Transmitter
tx = sdrtx('Pluto','Gain',-30);
tx.CenterFrequency = 916e6;
tx.Gain = 0;
tx.BasebandSampleRate = 400000;
rx.SamplesPerFrame = 11226;

% Constellation diagrams
constDiagram1 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','txData');
constDiagram2 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','filteredData');
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','agcData');
constDiagram4 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100);
constDiagram5 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100);

% Channel
channel = comm.AWGNChannel('EbNo',10,'BitsPerSymbol',2);
pfo = comm.PhaseFrequencyOffset( ...
    'PhaseOffset',45, ...
    'FrequencyOffset',1e4, ...
    'SampleRate',1e6);

% Instantiate communication toolbox blocks
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

barker = comm.BarkerCode("Length",13,SamplesPerFrame=13);


% Barker code

% Bit generation
msgSet = zeros(100 * MessageLength, 1); 
for msgCnt = 0 : 99
    msgSet(msgCnt * MessageLength + (1 : MessageLength)) = ...
        sprintf('%s %03d\n', Message, msgCnt);
end
seq = barker();
seq
MessageBits = [seq, int2bit(msgSet, 7)];


% ---- Sender ----
modSig = qpskmod(MessageBits);
txData = txfilter(modSig);
tx.transmitRepeat(txData);

% ---- Channel ----
% offsetData = pfo(txData);
% rxSig = channel(offsetData);


% ---- Receiver ----

%agcData = agc(rx());
filteredData = rxfilter(rx());
compensatedData = coarseFrequencyCompensator(filteredData);
synchronizedSymbol = symbolSynchronizer(compensatedData);
synchronizedCarrier = carrierSynchronizer(synchronizedSymbol);

rxData = qpskdemod(synchronizedCarrier);

% ---- Error calculation ----
% errorStats = errorRate(MessageBits,rxData);
% fprintf("Errors: %d\n", errorStats(1));

% ---- Data decoding ----
charSet = int8(bit2int(rxData, 1));
%fprintf('%s', char(charSet));

constDiagram1(txData)
constDiagram2(filteredData)
%constDiagram3(agcData)
constDiagram4(compensatedData)


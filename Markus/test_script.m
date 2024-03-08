% Parameters
Message = "Hello world";
MessageLength = strlength(Message) + 5;
SamplesPerSymbol = 12;

% Constellation diagrams
constDiagram1 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100);
constDiagram2 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100);
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100);
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

txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',SamplesPerSymbol);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',SamplesPerSymbol, ...
    'DecimationFactor',12);
errorRate = comm.ErrorRate('ReceiveDelay',2);


% Bit generation
msgSet = zeros(100 * MessageLength, 1); 
for msgCnt = 0 : 99
    msgSet(msgCnt * MessageLength + (1 : MessageLength)) = ...
        sprintf('%s %03d\n', Message, msgCnt);
end
MessageBits = int2bit(msgSet, 7);

% ---- Sender ----
modSig = qpskmod(MessageBits);
txData = txfilter(modSig);


% ---- Channel ----
offsetData = pfo(txData);
rxSig = channel(offsetData);

% ---- Receiver ----
filteredData = rxfilter(rxSig);
compensatedData = coarseFrequencyCompensator(filteredData);
synchronizedSymbol = symbolSynchronizer(compensatedData);
synchronizedCarrier = carrierSynchronizer(synchronizedSymbol);

rxData = qpskdemod(synchronizedCarrier);

% ---- Error calculation ----
% errorStats = errorRate(MessageBits,rxData);
fprintf("Errors: %d\n", errorStats(1));

% ---- Data decoding ----
charSet = int8(bit2int(rxData, 1));
%fprintf('%s', char(charSet));

constDiagram1(txData)
constDiagram2(filteredData)


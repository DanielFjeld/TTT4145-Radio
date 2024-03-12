% Parameters
Message = "Hello world";
MessageLength = strlength(Message) + 5;
SamplesPerSymbol = 2;

% Setup Receiver
%rx = sdrrx('Pluto','OutputDataType','double','SamplesPerFrame',2^15);
%rx.CenterFrequency = 916e6;
%rx.BasebandSampleRate = 400000;
%rx.SamplesPerFrame = 11226;
% Setup Transmitter
%tx = sdrtx('Pluto','Gain',0);
%tx.CenterFrequency = 916e6;
%tx.Gain = 0;
%tx.BasebandSampleRate = 400000;
%rx.SamplesPerFrame = 11226;

% Constellation diagrams
constDiagram1 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','txData');
constDiagram2 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','agcData');
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','filteredData');
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','compensatedData');
constDiagram4 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','synchronizedSymbol');
constDiagram5 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','synchronizedCarrier');
constDiagram6 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','filteredData');

% Channel
channel = comm.AWGNChannel('EbNo',10,'BitsPerSymbol',2);
pfo = comm.PhaseFrequencyOffset( ...
    'PhaseOffset',45, ...
    'FrequencyOffset',1e4, ...
    'SampleRate',1e6);

% Instantiate communication toolbox blocks
qpskmod = comm.QPSKModulator('BitInput',true);
qpskdemod = comm.QPSKDemodulator('BitOutput',true);
coarseFrequencyCompensator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","Correlation-based",MaximumFrequencyOffset=6e3,SampleRate=200000);
symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);
carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);

txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',3,'RolloffFactor',0.5,'FilterSpanInSymbols',16);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',3, ...
           RolloffFactor=0.5,FilterSpanInSymbols=16,DecimationFactor=1);
errorRate = comm.ErrorRate('ReceiveDelay',2);
agc = comm.AGC();
agc.DesiredOutputPower = 2;
agc.AveragingLength = 50;
agc.MaxPowerGain = 20;

%barker = comm.BarkerCode("Length",13,SamplesPerFrame=13);


% Barker code

% Bit generation
msgSet = zeros(100 * MessageLength, 1);
for msgCnt = 0 : 99
    msgSet(msgCnt * MessageLength + (1 : MessageLength)) = ...
        sprintf('%s %03d\n', Message, msgCnt);
end
barkerSeq = [0;0;0;0;0;1;1;0;0;1;0;1;0];
barkerCode = [barkerSeq; barkerSeq];
%msgtest = int2bit(msgSet, 7);
MessageBits = [zeros(1000, 1) ; barkerCode ; int2bit(msgSet, 7)];


% ---- Sender ----
modSig = qpskmod(MessageBits);
padding = zeros(100, 1);
padSig = [padding; modSig; padding;]; %make signal imaginary
txData = txfilter(modSig);

%tx.transmitRepeat(txData);

% ---- Channel ----
offsetData = pfo(txData);
rxSig = channel(offsetData);


% ---- Receiver ----

%agcData = agc(rx());
filteredData = rxfilter(rxSig);
compensatedData = coarseFrequencyCompensator(filteredData);
synchronizedSymbol = symbolSynchronizer(compensatedData);
synchronizedCarrier = carrierSynchronizer(synchronizedSymbol);

rxData = qpskdemod(synchronizedCarrier);


%% ---- Error calculation ----
%errorStats = errorRate(MessageBits,rxData);
%fprintf("Errors: %d\n", errorStats(1));

%% ---- Data decoding ----
%charSet = int8(bit2int(rxData, 1));
%fprintf('%s', char(charSet));

PreambleDetector = comm.PreambleDetector(barkerSeq,"Input","Bit");
preambleIndex = PreambleDetector(rxData)+14

temp = rxData([preambleIndex:(preambleIndex+5*(16*7)-1)]);

preambleIndex = PreambleDetector(rxData);
disp("Preamble Idex:")
disp(preambleIndex); % Check preamble index value
disp("rxData length:")
disp(length(rxData)); % Check length of rxData
disp("Index of temp:")
disp(preambleIndex + 5*(16*7)-1); % Check end index for temp


result = char(bit2int(temp,7));
strings_2D = reshape(result, 16, [])';
% Convert the 2D matrix into a cell array of strings
strings_cell = cellstr(strings_2D);

disp(strings_cell);

%constDiagram1(txData)
%constDiagram2(agcData)
%constDiagram3(filteredData)
%constDiagram4(compensatedData)
%constDiagram5(synchronizedSymbol)
%constDiagram5(synchronizedCarrier)

%release(tx);
%release(rx);
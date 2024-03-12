% Parameters
Message = "Hello world";
MessageLength = strlength(Message) + 5;
SamplesPerSymbol = 2;
pCnt = 0;
pMeanFreqOff = 0;

numReceives = 0;
numSuccessfull = 0;

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

% Constellation diagrams
constDiagram1 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','txData');
constDiagram2 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','agcData');
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','filteredData');
constDiagram4 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','compensatedData');
constDiagram5 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','synchronizedSymbol');
constDiagram6 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',100,'Title','synchronizedCarrier');

% Channel
channel = comm.AWGNChannel('EbNo',10,'BitsPerSymbol',2);
pfo = comm.PhaseFrequencyOffset( ...
    'PhaseOffset',45, ...
    'FrequencyOffset',1e4, ...
    'SampleRate',1e6);

% Instantiate communication toolbox blocks
qpskmod = comm.QPSKModulator('BitInput',true);
qpskdemod = comm.QPSKDemodulator('BitOutput',true);
pCoarseFreqEstimator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","Correlation-based",MaximumFrequencyOffset=6e3,SampleRate=200000);
pCoarseFreqCompensator = comm.PhaseFrequencyOffset("PhaseOffset",0,"FrequencyOffsetSource","Input port","SampleRate",200000);
symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01,DetectorGain=5.4,Modulation="PAM/PSK/QAM");
carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);

txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',2,'RolloffFactor',0.5,'FilterSpanInSymbols',10);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',2, ...
           RolloffFactor=0.5,FilterSpanInSymbols=10,DecimationFactor=1);
errorRate = comm.ErrorRate('ReceiveDelay',2);
agc = comm.AGC();
agc.DesiredOutputPower = 2;
agc.AveragingLength = 50;
agc.MaxPowerGain = 20;

barker = comm.BarkerCode("Length",13,SamplesPerFrame=13);

pMeanFreqOff = 0;
pCnt = 0;

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

% tx.transmitRepeat(txData);


% ---- Channel ----
% offsetData = pfo(txData);
% rxSig = channel(offsetData);


% ---- Receiver ----
while(1)
agcData = agc(rx());
filteredData = rxfilter(agcData);
[~, freqOffsetEst] = pCoarseFreqEstimator(filteredData);   % Coarse frequency offset estimation
            % average coarse frequency offset estimate, so that carrier
            % sync is able to lock/converge
            freqOffsetEst = (freqOffsetEst + pCnt * pMeanFreqOff)/(pCnt+1);
            pCnt = pCnt + 1;            % update state
            pMeanFreqOff = freqOffsetEst;
            
            coarseCompSignal = pCoarseFreqCompensator(filteredData,...
                -freqOffsetEst);                                
synchronizedSymbol = symbolSynchronizer(coarseCompSignal);
synchronizedCarrier = carrierSynchronizer(synchronizedSymbol);

rxData = qpskdemod(synchronizedCarrier);

%% ---- Error calculation ----
%errorStats = errorRate(MessageBits,rxData);
%fprintf("Errors: %d\n", errorStats(1));

%% ---- Data decoding ----
%charSet = int8(bit2int(rxData, 1));
%fprintf('%s', char(charSet));
%constDiagram1(txData)
%constDiagram2(agcData)
%constDiagram3(filteredData)
%onstDiagram4(coarseCompSignal)
%constDiagram5(synchronizedSymbol)
constDiagram6(synchronizedCarrier)
%eyediagram(txData, 10)
%eyediagram(agcData, 10)
%eyediagram(filteredData, 10)
%eyediagram(coarseCompSignal, 10)
%eyediagram(synchronizedSymbol, 10)
%eyediagram(synchronizedCarrier, 10)

pModulatedHeader = sqrt(2)/2 * (-1-1i) * [+1; +1; +1; +1; +1; -1; -1; +1; +1; -1; +1; -1; +1];
pMod = [pModulatedHeader ; pModulatedHeader];
PreambleDetector = comm.PreambleDetector(barkerSeq,"Input","Bit");

preambleIndex = PreambleDetector(rxData)+14;
numReceives = numReceives + 1;
rxPadData = [rxData ; zeros(10000, 1)];
if (numel(preambleIndex)==2)
    if ((preambleIndex(1)+13) == preambleIndex(2))
            numSuccessfull = numSuccessfull + 1;
            temp = rxPadData([preambleIndex:(preambleIndex+5*(16*7)-1)]);
            result = char(bit2int(temp,7));
            strings_2D = reshape(result, 16, [])';
            % Convert the 2D matrix into a cell array of strings
            strings_cell = cellstr(strings_2D);
            disp(strings_cell);
    else
        %disp("Not found")
    end
else
    %disp("Empty")
end
er = numSuccessfull/numReceives;
fprintf("Received: %d \t Successful: %d \t Rate: %f \n", numReceives, numSuccessfull, er)
end
%constDiagram1(txData)
%constDiagram2(agcData)
%constDiagram3(filteredData)
%constDiagram4(compensatedData)
%constDiagram5(synchronizedSymbol)
%constDiagram6(synchronizedCarrier)


%release(tx);
%release(rx);

%% Comments
% %% will give a new line to split up code
% Clear all will clear persistent variables which may be stuck after the last compile
clear all;

NODE = 0;
TXID = 1;
RXID = 2;

%% Parameters
resend = 1;
if(NODE)
    Message = ' ACK ';
else
    Message = 'ABCDE';
end
Number_size = 7; % int8_t
% Number = 69; % number to be sent

Simulate = false;
RX_LOOP = true;
TX_LOOP = false;

MessageLength = strlength(Message); 
SamplesPerSymbol = 12;

%% Setup
count = 0;
count2 = 0;
CRCok = 0;

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
pCoarseFreqEstimator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","FFT-based",SampleRate=400000);
pCoarseFreqCompensator = comm.PhaseFrequencyOffset("PhaseOffset",0,"FrequencyOffsetSource","Input port","SampleRate",400000);
symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01,DetectorGain=5.4,Modulation="PAM/PSK/QAM");
carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);

txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',2,'RolloffFactor',0.5,'FilterSpanInSymbols',10);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',2,RolloffFactor=0.5,FilterSpanInSymbols=10,DecimationFactor=1);

%txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',SamplesPerSymbol,'RolloffFactor',0.5);
%rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',SamplesPerSymbol,'DecimationFactor',SamplesPerSymbol,RolloffFactor=0.5);

coarseFrequencyCompensator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","Correlation-based",MaximumFrequencyOffset=6e3,SampleRate=200000);
%symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);
%carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);

%errorRate = comm.ErrorRate('ReceiveDelay',2);
agc = comm.AGC();
agc.DesiredOutputPower = 1;
agc.AveragingLength = 50;
agc.MaxPowerGain = 60;

%% Create frame
barker = comm.BarkerCode("Length",13,SamplesPerFrame=13);
bar = barker();
Preamble = [bar; bar;];

% MessageBits
msgSet = zeros(resend * MessageLength, 1); 
for msgCnt = 0 : resend-1
    msgSet(msgCnt * MessageLength + (1 : MessageLength)) = ...
        sprintf('%s', Message);
end

MsgTxOut = msgSet;
MsgTxOut = [int2bit(TXID, Number_size);int2bit(MsgTxOut, 7);];

%% CRC Generation
CRCtxIn = MsgTxOut;
crcGen = comm.CRCGenerator('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
CRCtxOut = crcGen(CRCtxIn);

%% Hamming encoding
HammingTxIn = CRCtxOut;
k = size(HammingTxIn, 1);
r = ceil(log2(k));
while 2^r < k + r + 1
    r = r + 1;
end
n = 2^r - 1;
k = n - r;

HammingTxOut = encode(HammingTxIn, n, k, 'hamming/binary');

%% Trellis encoding, Viterbi
TrellisTxIn = [HammingTxOut; zeros(1,1);];
trellis = poly2trellis([4 3],[4 5 17;7 4 2]);
tbdepth = 5 * 3; 

TrellisTxOut = convenc(TrellisTxIn,trellis);
TrellisTxOut = TrellisTxIn; %%BYPASS

%% Modulate, add noise, and demodulate
modulateTxIn = [CRCtxOut;zeros(64, 1);]; %%ScramblerTXout;

if(mod(size(modulateTxIn,1),2) == 1)
    modulateTxIn = [modulateTxIn; zeros(1, 1)]; 
end

msg = qpskmod(modulateTxIn);
msg = msg*sqrt(2);
ImPreamble = Preamble+Preamble*i;
padding = zeros(14000, 1);
modSig = [msg; ImPreamble; msg; msg;]; 

txData = txfilter(modSig); 

rxSig = 0;

%% Transmit data
if(Simulate == false && TX_LOOP)
    while(1)
        tx(txData);
        tic
        while(toc < 1)
        end
    end
else
    offsetData = pfo(txData);
    rxSig = channel(offsetData);
end

%% Receiver
ack = 0;
while(RX_LOOP)
    if(NODE == 0)
        if(toc > 0.05 && ack == 0)
            tic  
            tx(txData);
            count = count + 1;
            count2 = count2 + 1;
        end
        if(toc > 0.1 && ack == 1)
            ack = 0;
            tic  
            tx(txData);
            count2 = 1;
        end
    end
    if(NODE == 1 && ack == 1)
        count = count + 1;
        count2 = count2 + 1;
        ack = 0;
        tx(txData);
    end

    if(Simulate == false)
        filteredData = rxfilter(rx());
    else
        filteredData = rxfilter(rxSig);
    end

    [~, freqOffsetEst] = pCoarseFreqEstimator(filteredData);
    freqOffsetEst = (freqOffsetEst + pCnt * pMeanFreqOff)/(pCnt+1);
    pMeanFreqOff = freqOffsetEst;

    coarseFreq = pCoarseFreqCompensator(filteredData,-freqOffsetEst);     
    synchronizedSymbol = symbolSynchronizer(coarseFreq);
    synchronizedCarrier = carrierSynchronizer(synchronizedSymbol); 

    ImRxOut = synchronizedCarrier;

    corr = xcorr(ImRxOut, ImPreamble); 
    L = length(corr);
    [v,i] = max(corr); 
    amp = abs(v);
    theta = atan2(imag(v),real(v))*180/pi;

    phaseOffset = comm.PhaseFrequencyOffset(PhaseOffset=-theta);
    preambleIndex = (i-(L+1)/2)+1 + 26; 
    if(size(ImRxOut, 1) - preambleIndex > 0)
        phaseTxOut = ImRxOut(preambleIndex:end); 
        phaseTxOut = phaseOffset(phaseTxOut);
        rxOutTemp = qpskdemod(phaseTxOut(1:end)); 
        
    else
        rxOutTemp = [zeros(1,1)];
    end
    rxOut = [rxOutTemp(1:end); zeros(size(TrellisTxOut, 1),1)];

    EOF = size(TrellisTxOut, 1);
    FrameDetectOut = rxOut(1:EOF);

    ScramblerRXin = FrameDetectOut;

    descrambler = comm.Descrambler(ScramblerBase, ...
                    ScramblerPolynomial, ScramblerInitialConditions);
    ScramblerRXout = descrambler(ScramblerRXin);

    TrellsRxIn = ScramblerRXout;
    TrellsRxOut = TrellsRxIn; 

    HammingRxIn = TrellsRxOut(1:size(HammingTxOut, 1)); 
    DetectedRxData = decode(HammingRxIn,n,k,'hamming/binary');
    HammingRxOut = DetectedRxData(1:size(CRCtxOut, 1));

    CRCrxIn = FrameDetectOut(1:size(CRCtxOut, 1)); 
    crcDet = comm.CRCDetector('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
    [detectedData, errFlag] = crcDet(CRCrxIn);

    reshapeRxIn = FrameDetectOut; 

    [y, x] = test(reshapeRxIn, MessageLength*resend, Number_size);
    formatSpec = '%s%d\n';
    fprintf(formatSpec, y, x);

    if(RX_LOOP)
        if(amp > 5 && size(rxOutTemp, 1) > EOF && rx_number == RXID)
            if(errFlag)
            else
                CRCok = CRCok +1;
            end
            rate = 1/count2;
            rate2 = CRCok/count;

            formatSpec = '%s%d count%d   CRCerror%d   CRCok%d amp%d success %f success2 %f\n';
            fprintf(formatSpec, decodedMessage, rx_number, count, count-CRCok, CRCok, amp, rate, rate2);
            if(errFlag == 0 && rx_number == RXID)
                ack = 1;
            end

        end
    else
        if(isequal(TrellsRxOut ,TrellisTxIn))
            disp('Trellis OK');
        else
            disp('Trellis not equal to input');
        end
        errors = biterr(TrellisTxOut, TrellsRxIn, [], 'column-wise');
        errorRate = errors/size(TrellisTxOut, 1);
        formatSpec = 'Before  Trells, error on %2d out of %2d bits = %.5f\n';
        fprintf(formatSpec,errors, size(TrellisTxOut, 1), errorRate);

        errors = biterr(TrellisTxIn, TrellsRxOut, [], 'column-wise');
        errorRate = errors/size(TrellisTxIn, 1);
        formatSpec = 'After   Trells, error on %2d out of %2d bits = %.5f\n\n';
        fprintf(formatSpec,errors, size(TrellisTxIn, 1), errorRate);

        if(isequal(HammingTxIn ,HammingRxOut))
            disp('Hamming OK');
        else
            disp('Hamming not equal to input');
        end
        errors = biterr(HammingRxIn, HammingTxOut, [], 'column-wise');
        errorRate = errors/size(HammingTxOut, 1);
        formatSpec = 'Before Hamming, error on %2d out of %2d bits = %.5f\n';
        fprintf(formatSpec,errors, size(HammingTxOut, 1), errorRate);

        errors = biterr(HammingRxOut, HammingTxIn, [], 'column-wise');
        errorRate = errors/size(HammingRxOut, 1);
        formatSpec = 'After  Hamming, error on %2d out of %2d bits = %.5f\n\n';
        fprintf(formatSpec,errors, size(HammingRxOut, 1), errorRate);

        if(errFlag)
            fprintf('CRC ERROR\n\n');
        else
            fprintf('CRC OK\n\n');
        end

        formatSpec = '%s%d\n';
        fprintf(formatSpec, decodedMessage, rx_number);
    end

end
release(rx);

function [y, x] = test(reshapeRxIn, MessageLength, Number_size)
    endOfMessage = MessageLength*7;
    messageBits = reshapeRxIn(1:endOfMessage);
    messageBitsReshaped = reshape(messageBits, 7, [])'; 
    decodedMessage = char(bin2dec(num2str(messageBitsReshaped)));  
    number_index_start = endOfMessage+1;
    number_index_stop = number_index_start+Number_size-1;
    rx_number_bits = reshapeRxIn(number_index_start:number_index_stop);
    rx_number = bit2int(rx_number_bits,Number_size);
    y = decodedMessage;
    x = rx_number;
end

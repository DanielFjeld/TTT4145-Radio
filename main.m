%%coments
% %% will give a new line to split up code
% clear all, will clear persistant variables wich may be stuck after last compile

% BER one way
% package loss one way
% PER two way

% 

clear all;

Message = '*';


NODE = 1;
continous = 1;
barker_test = 0;
BER_test = 0;
eye_test = 0;


sps = 2;
passband = 200000*sps;


%% Parameters
freq_offset = 0;

if(NODE)
    TXID = 2;
    RXID = 1;
    freq_offset = -750;
else
    TXID = 1;
    RXID = 2;
    freq_offset = +1500;
end





resend = 1;
if(NODE)
    Message = 'C';
else
    Message = 'A';
end

message_ID = 0;
Message_ID_size = 2;
rx_last_val = 0;

Number_size = 7; %int8_t
%Number = 69; %number to be sent

Simulate = false;
RX_LOOP = true;
TX_LOOP = false;

MessageLength = strlength(Message); 
SamplesPerSymbol = 8;


%% Setup

count = 0;
count2 = 0;
count4 = 0;
count5 = 0;
CRCok = 0;
starting_continous = 1;
barker_send_count = 100;

BER_acc = 0;
BER_bits = 0;
BER = 0;

n = 40; % However many numbers you want.
padding = randi([0, 1], [n, 1]);
%padding = zeros(7000, 1);


%frequenzy = 916MHz

Freq1 = 916.3e6;
Freq2 = 917.5e6;

% Setup Receiver
rx = sdrrx('Pluto','OutputDataType','double','SamplesPerFrame',2^15);
if(NODE)
 rx.CenterFrequency = Freq1;
else 
 rx.CenterFrequency = Freq2;
end



rx.BasebandSampleRate = passband;
rx.SamplesPerFrame = 20000;
% Setup Transmitter
tx = sdrtx('Pluto','Gain',0);
if(NODE)
 tx.CenterFrequency = Freq2;
else 
 tx.CenterFrequency = Freq1;
end

tx.Gain = 0;
tx.BasebandSampleRate = passband;

%% Constellation diagrams
constDiagram1 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',3000,'Title','filteredData');
constDiagram2 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',3000,'Title','coarseFreq');
constDiagram3 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',3000,'Title','phaseTxOut');
constDiagram4 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',3000, 'Title','synchronizedSymbol');
constDiagram5 = comm.ConstellationDiagram('SamplesPerSymbol',SamplesPerSymbol, ...
    'SymbolsToDisplaySource','Property','SymbolsToDisplay',3000, 'Title','synchronizedCarrier');



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
pCoarseFreqEstimator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","Correlation-based",SampleRate=80000);
pCoarseFreqCompensator = comm.PhaseFrequencyOffset("PhaseOffset",0,"FrequencyOffsetSource","Input port","SampleRate",80000*sps);
symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=sps,DampingFactor=1,NormalizedLoopBandwidth=0.01,DetectorGain=2.7,Modulation="PAM/PSK/QAM");
carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=1,DampingFactor=1,NormalizedLoopBandwidth=0.01);

txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',sps,'RolloffFactor',0.5,'FilterSpanInSymbols',10);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',sps,RolloffFactor=0.5,FilterSpanInSymbols=10,DecimationFactor=1);

%txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol',SamplesPerSymbol,'RolloffFactor',0.5);
%rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol',SamplesPerSymbol,'DecimationFactor',SamplesPerSymbol,RolloffFactor=0.5);


%coarseFrequencyCompensator = comm.CoarseFrequencyCompensator("Modulation","QPSK","Algorithm","Correlation-based",MaximumFrequencyOffset=6e3,SampleRate=200000);
%symbolSynchronizer = comm.SymbolSynchronizer("TimingErrorDetector","Gardner (non-data-aided)",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);
%carrierSynchronizer = comm.CarrierSynchronizer("Modulation","QPSK","ModulationPhaseOffset","Auto",SamplesPerSymbol=2,DampingFactor=1,NormalizedLoopBandwidth=0.01);

%errorRate = comm.ErrorRate('ReceiveDelay',2);
agc = comm.AGC();
agc.DesiredOutputPower = 1;
agc.AveragingLength = 50;
agc.MaxPowerGain = 60;




%% create frame
barker = comm.BarkerCode("Length",13,SamplesPerFrame=13);
bar = barker();
Preamble = [bar; bar;];


ImPreamble = Preamble/sqrt(2)+Preamble*i/sqrt(2);

ack = 1;
transmitt_message = 0;
rx_message_id = 0;
message_index = 0;
message_index_size = 0;
while(RX_LOOP)

 if(NODE == 0)
    if(toc > 0.05) %&& ack == 1
      if(ack || (barker_test && count >= barker_send_count))
          while(~message_index || (barker_test && count >= barker_send_count))
              if(~barker_test)
                  count = 0;
                  CRCok = 0;
                  prompt = "Input:";
                  text_input = input(prompt, "s");
                  message_index = strlength(text_input)+1;
                  message_index_size = message_index;
                  text_input_char_arr = [char(text_input), char(10)];
              else
                  message_index = 1;
                  count = 0;
                  count4 = 0;
                  prompt = "Input:";
                  text_input = input(prompt, "s");
              end

          end
          if(barker_test)
              Message = 'U';
              message_ID = 0;
          else
              Message = text_input_char_arr(message_index_size - message_index + 1);
              message_index = message_index - 1;
              if(message_index < 0)
                  message_ID = 0;
              end
              message_ID = message_ID + 1;
              if(message_ID == 4)
                  message_ID = 0;
              end
          
          end
      end
      ack = 0;
      tic  
      transmitt_message = 1;
      count2 = 1;
      count = count + 1;
    end
end
if(NODE == 1 && ack == 1)
    count = count + 1;
    ack = 0;
    transmitt_message = 1;
    message_ID = rx_message_id;
end

%% MessageBits

msgSet = zeros(resend * MessageLength, 1); 
for msgCnt = 0 : resend-1
    msgSet(msgCnt * MessageLength + (1 : MessageLength)) = ...
        sprintf('%s', Message);
end
%seq = barker();
%seq = max(0, seq); %%make barker bits between 0 and 1 
%seq = int2bit(seq, 2);

MsgTxOut = msgSet;
MsgTxOut = [int2bit(TXID, Number_size);int2bit(message_ID, Message_ID_size);int2bit(MsgTxOut, 8);];

%% CRC Generation
CRCtxIn = MsgTxOut;
crcGen = comm.CRCGenerator('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
CRCtxOut = crcGen(CRCtxIn);

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
TrellisTxOut = TrellisTxIn; %%BYPASS


%% frame 
frameTxIn = TrellisTxOut;
if(mod(size(frameTxIn,1),2) == 1)%must be integer multiple of bits per symbol (2)
    MessageBits = [frameTxIn; zeros(1, 1);]; %need to add a zero at the end
else
    MessageBits = frameTxIn; 
end

frameTxOut = MessageBits; %frame



%% modululate from real to imaginary numbers and add preamble
%padding = zeros(14000, 1);

modulateTxIn = [CRCtxOut;]; %%ScramblerTXout;

if(mod(size(modulateTxIn,1),2) == 1)%must be integer multiple of bits per symbol (2)
    modulateTxIn = [modulateTxIn; zeros(1, 1);]; %need to add a zero at the end
end
trail = qpskmod(padding);
msg = qpskmod(modulateTxIn);
if(eye_test)
    msg = qpskmod([0;0;1;0;0;0;1;1;0;1;1;1;0;0;1;0;0;1;0;0;1;0;0;0;1;1;0;1;1;1;0;0;1;0;0;1;0;0;1;0;0;0;1;1;0;1;1;1;0;0;1;0;0;1]);
end
msg = msg; %*sqrt(2);


modSig = [trail; ImPreamble; msg; trail;]; %make signal imaginary



txData = txfilter(modSig); %lp filter (make transitions smooth)

if(transmitt_message && (~continous | NODE | barker_test))
   transmitt_message = 0;
   tx(txData);
end
if(continous && starting_continous && ~NODE && ~barker_test)
    starting_continous = 0;
    tx.transmitRepeat(txData)
end


%% ---- Receiver ----

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
            
%coarseFreq = pCoarseFreqCompensator(filteredData,-freqOffsetEst);  

coarseFreq = pCoarseFreqCompensator(filteredData,-pMeanFreqOff);     

synchronizedSymbol = symbolSynchronizer(coarseFreq);
%synchronizedCarrier = carrierSynchronizer(coarseFreq); %phase correction
synchronizedCarrier = carrierSynchronizer(synchronizedSymbol); %phase correction

ImRxOut = synchronizedCarrier;

%find start of frame

%% detect frame start and phase offset
corr = xcorr(ImRxOut, ImPreamble); %%do correlation
L = length(corr);
[v,i] = max(corr); %i = start of index

%get amplitude and angle of correlation
amp = abs(v);
theta = atan2(imag(v),real(v))*180/pi;

%apply offset
phaseOffset = comm.PhaseFrequencyOffset(PhaseOffset=-theta);


preambleIndex = (i-(L+1)/2)+1 + 26; %find start of preamble
if(size(ImRxOut, 1) - preambleIndex > 0)
    phaseTxOut = ImRxOut(preambleIndex:end); %set start of data
    phaseTxOut = phaseOffset(phaseTxOut);
    rxOutTemp = qpskdemod(phaseTxOut(1:end)); %generate bits from const diagram
    
else
    rxOutTemp = [zeros(1,1)];
end
rxOut = [rxOutTemp(1:end); zeros(size(TrellisTxOut, 1),1)];


EOF = size(TrellisTxOut, 1);
FrameDetectOut = rxOut(1:EOF);


%% Trells decoding, Veterbi
TrellsRxIn = FrameDetectOut;
%TrellsRxOut = vitdec(TrellsRxIn, trellis, tbdepth, 'trunc', 'hard'); % The last parameter specifies the number of soft decision bits
TrellsRxOut = TrellsRxIn; %BYPASS

%% hamming decoding
%%introduse error
%errLoc = randerr(1,n);
%HammingDecData = mod(HammingDecData + errLoc',2);

%calculation
HammingRxIn = TrellsRxOut(1:size(HammingTxOut, 1)); %get data

DetectedRxData = decode(HammingRxIn,n,k,'hamming/binary');
HammingRxOut = DetectedRxData(1:size(CRCtxOut, 1));

%% CRC check
CRCrxIn = FrameDetectOut(1:size(CRCtxOut, 1)); %HammingRxOut;
% Create a CRC detector with the same settings as the generator
crcDet = comm.CRCDetector('Polynomial', 'z^8 + z^2 + z + 1', 'InitialConditions', 1, 'DirectMethod', true, 'FinalXOR', 1);
% Check the received data for CRC errors
[detectedData, errFlag] = crcDet(CRCrxIn);

%% reshape bits
reshapeRxIn = FrameDetectOut; %detectedData;
%%Extract number
rx_number_bits = reshapeRxIn(1:7);
rx_number = bit2int(rx_number_bits,Number_size);

rx_message_id_bits = reshapeRxIn(8:9);
rx_message_id = bit2int(rx_message_id_bits,Message_ID_size);

% Extract the message bits after the Barker codes
reshapeRxIn = FrameDetectOut(10:end); %detectedData;
endOfMessage = MessageLength*8*resend;
messageBits = reshapeRxIn(1:endOfMessage);

% Reshape the message bits into 7-bit rows, assuming the total number of message bits is divisible by 7
% This might need adjustment based on how the bits are packed and the total length
messageBitsReshaped = reshape(messageBits, 8, [])'; %can be printed if you remove ; and add '

% Convert each 7-bit group to a character
decodedMessage = char(bin2dec(num2str(messageBitsReshaped)));  %can be printed if you remove ; and add '



%% ---- Error calculation ---- ack = 1;
if(RX_LOOP)
    if(amp > 5)
        count2 = count2 + 1; 
        count4 = count4 + 1;        
        if(barker_test)
            ack = 1;
            if(~NODE)
                PER = count4/count;
                formatSpec = 'sent:%d recived:%d PER:%f\n';
                fprintf(formatSpec, count, count4, PER);
            else
                formatSpec = 'Count%d\n';
                fprintf(formatSpec, count);
            end
        end
    end
    if(amp > 5 && size(rxOutTemp, 1) > EOF && ~barker_test)
        if(NODE) %BER test
            test = [0;0;0;0;0;0;1;0;1;0;1;1;0;1;0;0;0;0;0;0;0;1;0;0;0];
            [number,ratio] = biterr(test,CRCrxIn);
            count5 = count5+1;
            BER_bits = BER_bits + number;
            BER = BER + ratio;
            BER_acc = BER/count5;
        end
        
        if(errFlag)
        else
            CRCok = CRCok +1;
        end
        rate2 = CRCok/count;
        rate3 = CRCok/count2;
        

        if(NODE)
            
            if(errFlag == 0 && rx_number == RXID && rx_message_id ~= rx_last_val && ~continous)
                rx_last_val = rx_message_id;
                formatSpec = '%s';
                fprintf(formatSpec, decodedMessage);
                
            end
            if(continous)
                rx_last_val = rx_message_id;
                formatSpec = '%s PER:%f count%d success:%d failed:%d RX:%d freq:%d BER: %f BER bits:%d\n';
                fprintf(formatSpec, decodedMessage, 1-rate3, count2, CRCok, count2-CRCok, rx_number, freqOffsetEst, BER_acc, BER_bits);
            end
            if(errFlag == 0 && rx_number == RXID)
                ack = 1;
            end
            if(eye_test)
                eyediagram(coarseFreq(1+preambleIndex:25*sps+preambleIndex), 2*sps)
            end
        end
        if(~NODE && errFlag == 0 && rx_number == RXID && (rx_message_id ~= rx_last_val || continous))
            rx_last_val = rx_message_id;
            formatSpec = 'count:%d   Failed:%d   Success:%d amp:%d success_rate:%f M_ID:%d BER:%d\n';
            fprintf(formatSpec, count, count-CRCok, CRCok, amp, rate2, rx_message_id, 1);
            if(errFlag == 0 && rx_number == RXID && rx_message_id == message_ID && ~BER_test)
                ack = 1;
            end
        end
    end
else

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
end
%% print diagrams
%constDiagram1(filteredData*5)
%constDiagram2(coarseFreq*5)
%constDiagram3(phaseTxOut*5)
%constDiagram4(synchronizedSymbol*5)
%constDiagram5(synchronizedCarrier*5) %dont know what this is

if(TX_LOOP)
else
 %   release(tx);
end

end
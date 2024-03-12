%% QPSK Transmitter and Receiver
% This example shows  the implementation of a QPSK transmitter and receiver
% with MATLAB(R). In particular, this example illustrates methods to
% address real-world wireless communications issues like carrier frequency
% and phase offset, timing recovery and frame synchronization. For the
% Simulink(R) implementation of the same system, refer to the
% <docid:comm_ug#example-commqpsktxrx QPSK Transmitter and Receiver in
% Simulink> example.

%   Copyright 2011-2023 The MathWorks, Inc.

%% Introduction
% The transmitted QPSK data undergoes impairments that simulate the effects
% of wireless transmission such as addition of Additive White Gaussian
% Noise (AWGN), introduction of carrier frequency and phase offset, and
% timing drift. To cope with these impairments, this example provides a
% reference design of a practical digital receiver. The receiver includes
% correlation-based coarse frequency compensation, PLL-based fine frequency
% compensation, PLL-based symbol timing recovery, frame synchronization,
% and phase ambiguity resolution.
%
% This example serves two main purposes:
%
% * To model a general wireless communication system that is able to
% successfully recover a message, which was corrupted by various simulated
% channel impairments.
%
% * To illustrate the use of key Communications Toolbox(TM) synchronization
% components including coarse and fine carrier frequency compensation,
% closed-loop timing recovery with bit stuffing and stripping, frame
% synchronization and carrier phase ambiguity resolution.

%% Initialization
% The
% <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','commqpsktxrx_init.m') commqpsktxrx_init.m>
% script initializes simulation parameters and generates the structure
% prmQPSKTxRx.
clear all;
prmQPSKTxRx = commqpsktxrx_init %#ok<*NOPTS> % QPSK system parameters

useScopes = true;          % true if scopes are to be used
printReceivedData = true; % true if the received data is to be printed
compileIt = false;         % true if code is to be compiled
useCodegen = false;        % true to run the generated mex file

%% Code Architecture for the System Under Test
% This example models a digital communication system using QPSK modulation.
% The function
% <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','runQPSKSystemUnderTest.m') runQPSKSystemUnderTest.m>
% models this communication environment. The QPSK transceiver model in this
% script is divided into the following four main components.
%
% 1) <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKTransmitter.m') QPSKTransmitter.m>:
% generates the bit stream and then encodes, modulates and filters it.
%
% 2) <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKChannel.m') QPSKChannel.m>:
% models the channel with carrier offset, timing offset, and AWGN.
%
% 3) <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKReceiver.m') QPSKReceiver.m>:
% models the receiver, including components for phase recovery, timing
% recovery, decoding, demodulation, etc.
%
% 4) <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKScopes.m') QPSKScopes.m>:
% optionally visualizes the signal using time scopes, frequency scopes, and
% constellation diagrams.
%
% Each component is modeled using a System object. To see the construction
% of the four main System object components, refer to
% <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','runQPSKSystemUnderTest.m') runQPSKSystemUnderTest.m>.

%% Description of the Individual Components
% *Transmitter*
%
% This component generates a message using ASCII characters, converts the
% characters to bits, and prepends a Barker code for receiver frame
% synchronization. This data is then modulated using QPSK and filtered with
% a square root raised cosine filter.
%
% *Channel*
%
% This component simulates the effects of over-the-air transmission. It
% degrades the transmitted signal with both phase and frequency offset, a
% time-varying delay to mimic clock skew between transmitter and receiver,
% and AWGN.
%
% *Receiver*
%
% This component regenerates the original transmitted message. It is
% divided into six subcomponents.
%
% 1) Automatic Gain Control: Sets its output power to a level ensuring that
% the equivalent gains of the phase and timing error detectors keep
% constant over time. The AGC is placed before the *Raised Cosine Receive
% Filter* so that the signal amplitude can be measured with an oversampling
% factor of two. This process improves the accuracy of the estimate.
%
% 2) Coarse frequency compensation: Uses a correlation-based algorithm to
% roughly estimate the frequency offset and then compensate for it. The
% estimated coarse frequency offset is averaged so that fine frequency
% compensation is allowed to lock/converge. Hence, the coarse frequency
% offset is estimated using a *comm.CoarseFrequencyCompensator* System
% object and an averaging formula; the compensation is performed using a
% *comm.PhaseFrequencyOffset* System object.
%
% 3) Timing recovery: Performs timing recovery with closed-loop scalar
% processing to overcome the effects of delay introduced by the channel,
% using a *comm.SymbolSynchronizer* System object. The object implements a
% PLL to correct the symbol timing error in the received signal. The
% rotationally-invariant Gardner timing error detector is chosen for the
% object in this example; thus, timing recovery can precede fine frequency
% compensation. The input to the object is a fixed-length frame of samples.
% The output of the object is a frame of symbols whose length can vary due
% to bit stuffing and stripping, depending on actual channel delays.
%
% 4) Fine frequency compensation: Performs closed-loop scalar processing
% and compensates for the frequency offset accurately, using a
% *comm.CarrierSynchronizer* System object. The object implements a
% phase-locked loop (PLL) to track the residual frequency offset and the
% phase offset in the input signal.
%
% 5) Frame Synchronization: Detects the location of the known Barker code
% in the input and performs frame synchronization using a
% *FrameSynchronizer* System object. It also converts the variable-length
% symbol inputs into fixed-length outputs. The object has a secondary
% output that is a boolean scalar indicating if the first frame output is
% valid.
%
% 6) Data decoder: Performs phase ambiguity resolution and demodulation.
% Also, the data decoder compares the regenerated message with the
% transmitted one and calculates the BER.
%
% *Scopes*
%
% This component provides optional visualization to plot:
%
% * A spectrum scope depicting the received signal before and after square
% root raised cosine filtering,
% * Constellation diagrams showing the received signal after receiver
% filtering, after timing recovery and then after fine frequency
% compensation.
%
% For more information about the system components, refer to the
% <docid:comm_ug#example-commqpsktxrx> Simulink example.

%% System Under Test
% The main loop in the system under test script processes the data
% frame-by-frame. Set the MATLAB variable |compileIt| to true in order to
% generate code. This can be accomplished by using the *codegen* command
% provided by the MATLAB Coder(TM) product. The *codegen* command
% translates MATLAB(R) functions to a MEX file, producing code for
% accelerated execution. The generated C code runs several times faster
% than the original MATLAB code. For this example, set |useCodegen| to true
% to use the code generated by *codegen* instead of the MATLAB code.
%
% The inner loop of
% <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','runQPSKSystemUnderTest.m') runQPSKSystemUnderTest.m>
% uses the four System objects previously mentioned. In this file, there is
% a for-loop around the system under test to process one frame at a time.
%
%  for count = 1:prmQPSKTxRx.FrameCount
%      transmittedSignal = qpskTx();
%      rcvdSignal = qpskChan(transmittedSignal, count);
%      [RCRxSignal, timingRecSignal, freqRecSignal, BER] = qpskRx(rcvdSignal); % Receiver
%      if useScopes
%        runQPSKScopes(qpskScopes, rcvdSignal, RCRxSignal, timingRecSignal,
%        freqRecSignal); % Plots all the scopes
%      end
%  end

%% Execution and Results
% To run the System Under Test script and obtain BER values for the
% simulated QPSK communication, the following code is executed. When you
% run the simulations, it displays the bit error rate data, and some
% graphical results. The displayed scopes are constellation diagrams of the
% *Raised Cosine Receive Filter* output, the *Symbol Synchronizer* output,
% and the *Fine Frequency Compensation* output, and the power spectrum of
% the *Raised Cosine Receive Filter* output.

if compileIt
    codegen -report runQPSKSystemUnderTest.m -args {coder.Constant(prmQPSKTxRx),coder.Constant(useScopes),coder.Constant(printReceivedData)} %#ok
end
if useCodegen
    BER = runQPSKSystemUnderTest_mex(prmQPSKTxRx,useScopes,printReceivedData);
else
    BER = runQPSKSystemUnderTest(prmQPSKTxRx,useScopes,printReceivedData);
end
fprintf('Error rate = %f.\n',BER(1));
fprintf('Number of detected errors = %d.\n',BER(2));
fprintf('Total number of compared samples = %d.\n',BER(3));

%% Alternate Execution Options
% As already mentioned in the section *System Under Test*, by using the
% variables at the beginning of the example, it is possible to interact
% with the code to explore different aspects of System objects and coding
% options.
%
% By default, the variables |useScopes| and |printReceivedData| are set to
% true and false, respectively. The |useScopes| variable enables MATLAB
% scopes to be opened during the example execution. Using the scopes, you
% can see how the simulated subcomponents behave and also obtain a better
% understanding of how the system functions in simulation time. When you
% set this variable to false, the scopes will not open during the example
% execution. When you set |printReceivedData| to true, you can also see the
% decoded received packets printed in the command window. The other two
% variables, |compileIt| and |useCodegen|, are related to speed performance
% and can be used to analyze design tradeoffs.
%
% When you set |compileIt| to true, this example script will use MATLAB
% Coder(TM) capabilities to compile the script runQPSKSystemUnderTest for
% accelerated execution. This command will create a MEX file
% (runQPSKSystemUnderTest_mex) and save it in the current folder. Once you
% set |useCodegen| to true to run the mex file, the example is able to run
% the system implemented in MATLAB much faster. This feature is essential
% for implementation of real-time systems and is an important simulation
% tool. To maximize simulation speed, set |useScopes| to false and
% |useCodegen| to true to run the mex file.
%
% For other exploration options, refer to the
% <docid:comm_ug#example-commqpsktxrx QPSK Transmitter and Receiver in
% Simulink> example.
%
%% Summary
% This example simulates digital communication over an AWGN channel. It
% shows how to model several parts of the QPSK system such as modulation,
% frequency and phase recovery, timing recovery, and frame synchronization.
% It measures the system performance by calculating BER. It also shows that
% the generated C code runs several times faster than the original MATLAB
% code.

%% Appendix
% This example uses the following script and helper functions:
%
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','runQPSKSystemUnderTest.m') runQPSKSystemUnderTest.m>
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKTransmitter.m') QPSKTransmitter.m>
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKChannel.m') QPSKChannel.m>
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKReceiver.m') QPSKReceiver.m>
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKScopes.m') QPSKScopes.m>
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKBitsGenerator.m') QPSKBitsGenerator.m>
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','QPSKDataDecoder.m') QPSKDataDecoder.m>
% * <matlab:openExample('comm/QPSKTransmitterAndReceiverExample','supportingFile','FrameSynchronizer.m') FrameSynchronizer.m>
%
%% References
% 1. Rice, Michael. _Digital Communications - A Discrete-Time Approach_.
% 1st ed. New York, NY: Prentice Hall, 2008.

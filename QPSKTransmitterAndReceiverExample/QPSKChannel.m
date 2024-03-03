classdef QPSKChannel < matlab.System 
%#codegen
    
%   Copyright 2012-2021 The MathWorks, Inc.
   
    
    properties (Nontunable)
        FrameSize = 16;
        DelayType = 'Triangle';
        RaisedCosineFilterSpan = 10;
        PhaseOffset = 47;
        SignalPower = 0.25;
        InterpolationFactor = 4;
        EbNo = 7;
        BitsPerSymbol = 2;
        FrequencyOffset = 5000;
        SampleRate = 200000;
        DelayStepSize = 0.05;
        DelayMaximum = 8;
        DelayMinimum = 0.1;
    end
    
    properties (Access=private)
        pPhaseFreqOffset
        pVariableTimeDelay
        pSNR    % in linear units
    end
    
    properties (Nontunable, Access=private)
        pFrameSize
    end
    
    methods
        function obj = QPSKChannel(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj, transmittedSignal, ~)
            
            obj.pFrameSize = length(transmittedSignal);
            
            obj.pPhaseFreqOffset = comm.PhaseFrequencyOffset( ...
                'PhaseOffset',              obj.PhaseOffset, ...
                'FrequencyOffset',          obj.FrequencyOffset, ...
                'SampleRate',               obj.SampleRate);
            obj.pVariableTimeDelay = dsp.VariableFractionalDelay( ...
                'MaximumDelay',             obj.pFrameSize*obj.InterpolationFactor, ...
                'OutputDataType',           'Same as first input');
            obj.pSNR = 10^(convertSNR(obj.EbNo,'ebno', ...
              'BitsPerSymbol',obj.BitsPerSymbol,...
              'SamplesPerSymbol',obj.InterpolationFactor)/10);
         end
        
        
        function corruptSignal = stepImpl(obj, TxSignal, count)
            
            % Calculates the delay 
            if strcmp(obj.DelayType,'Ramp')
                delay = ...
                    min(((count - 1) * obj.DelayStepSize + obj.DelayMinimum), ...
                    (obj.pFrameSize-obj.RaisedCosineFilterSpan) ...
                    *obj.InterpolationFactor); % Variable delay taking the form of a ramp
            else
                % Variable delay taking the shape of a triangle
                index = mod(count-1,2*obj.DelayMaximum/obj.DelayStepSize);
                if index <= obj.DelayMaximum/obj.DelayStepSize
                    delay = index * obj.DelayStepSize;
                else
                    delay = 2*obj.DelayMaximum - index * obj.DelayStepSize;
                end
            end
            
            delay = cast(delay, 'like', real(TxSignal));
            
            % Signal undergoes phase/frequency offset
            rotatedSignal = obj.pPhaseFreqOffset(TxSignal);
            
            % Delayed signal
            delayedSignal = obj.pVariableTimeDelay(rotatedSignal, delay);
            
            % Signal passing through AWGN channel
            corruptSignal = awgn(delayedSignal,obj.pSNR,obj.SignalPower,'linear');
        end
        
        function resetImpl(obj)
            reset(obj.pPhaseFreqOffset);
            reset(obj.pVariableTimeDelay);            
        end
        
        function releaseImpl(obj)
            release(obj.pPhaseFreqOffset);
            release(obj.pVariableTimeDelay);            
        end
        
        function N = getNumInputsImpl(~)
            N = 2; 
        end
    end
end


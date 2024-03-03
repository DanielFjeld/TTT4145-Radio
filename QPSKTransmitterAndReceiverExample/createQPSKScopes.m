function hScopes = createQPSKScopes(sampleRate)

persistent hScope
if isempty(hScope)
    hScope = QPSKScopes('SampleRate', sampleRate);
end
hScopes = hScope;

end


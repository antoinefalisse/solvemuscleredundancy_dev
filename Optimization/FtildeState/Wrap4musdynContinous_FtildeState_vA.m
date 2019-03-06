function phaseout = Wrap4musdynContinous_FtildeState_vA(input)

persistent splinestruct

if isempty(splinestruct) || size(splinestruct.MA,1) ~= length(input.phase.time) 
    if ~isfield(input.auxdata,'EMGconstr') || input.auxdata.EMGconstr==0
        splinestruct = SplineInputData(input.phase.time,input);
    else
        splinestruct = SplineInputData_EMG(input.phase.time,input);
    end
end

input.auxdata.splinestruct = splinestruct;

if ~isfield(input.auxdata,'EMGconstr') || input.auxdata.EMGconstr==0
    phaseout = musdynContinous_FtildeState_vA(input);
else
     phaseout = musdynContinous_FtildeState_vA_EMG(input);
end
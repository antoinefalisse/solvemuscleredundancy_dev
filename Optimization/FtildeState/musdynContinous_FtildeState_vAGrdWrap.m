function phaseout = musdynContinous_FtildeState_vAGrdWrap(input)

persistent splinestruct

if isempty(splinestruct) || size(splinestruct.MA,1) ~= length(input.phase.time.f)   
    if ~isfield(input.auxdata,'EMGconstr') || input.auxdata.EMGconstr==0
        splinestruct = SplineInputData(input.phase.time.f,input);
    else
        splinestruct = SplineInputData_EMG(input.phase.time.f,input);
    end
end

input.auxdata.splinestruct = splinestruct;

if ~isfield(input.auxdata,'EMGconstr') || input.auxdata.EMGconstr==0
    phaseout = musdynContinous_FtildeState_vAADiGatorGrd(input);
else
    phaseout = musdynContinous_FtildeState_vA_EMGADiGatorGrd(input);
end
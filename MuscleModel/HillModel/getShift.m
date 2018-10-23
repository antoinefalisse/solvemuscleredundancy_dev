% This script returns the value used to shift the tendon force-length curve
% when changing the tendon stiffness. 
% With the standard stiffness (35), the shift is 0. For a different
% stiffness, the curve is shifted so that the normalized tendon force is
% the same as with the standard stiffness when the normalized tendon length
% is 1.

function shift = getShift(kT)

kT35 = 35;
shift = 0;
lTtilda = 1;
fse = (exp(kT35.*(lTtilda - 0.995)))/5 - 0.25 + shift; 
fse_kt35 = fse;

fse = (exp(kT.*(lTtilda - 0.995)))/5 - 0.25 + shift; 
fse_kt = fse;

shift = fse_kt35-fse_kt;
fse = (exp(kT.*(lTtilda - 0.995)))/5 - 0.25 + shift;

if sum(fse ~= fse_kt35) ~= 0
    shift = NaN;
    warning('Error in shift tendon force-length curve');
end

end


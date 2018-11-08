% This script adjusts the tendon slack length as a function of the tendon
% compliance to maintain the tendon strain

function lTs_adj = getlTs(lTs,kT)
% 1) lT when fse is 0.5 with standard tendon stiffness (35)
fse = 0.5;
shift_35 = 0;
Atendon_35 = 35;
lTtilde_35 = log(5*(fse + 0.25 - shift_35))./Atendon_35 + 0.995;
lT_35 = lTtilde_35.*lTs;

% 2) get shift for adjusted tendon stiffness
shift = getShift(kT);

% 3) adjust tendon slack length to keep the same tendon length when the
% normalized tendon force is 0.5
lTtilde = log(5*(fse + 0.25 - shift))./kT + 0.995;
lTs_adj = lT_35./lTtilde;
end

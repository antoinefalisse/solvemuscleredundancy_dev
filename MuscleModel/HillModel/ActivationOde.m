function xdot = ActivationOde(t, x, atime, EMG)

e = interp1(atime, EMG, t);

b = 100;

tact = 0.015;
tdeact = 0.06;

d1 = 1./(tact*(0.5+1.5*x));
d2 = (0.5+1.5*x)/tdeact;
f = 0.5*tanh(b*(e-x));
xdot = (d1.*(f+0.5) + d2.*(-f+0.5)).*(e-x);
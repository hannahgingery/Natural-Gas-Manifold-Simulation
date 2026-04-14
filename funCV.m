function out  = funCV(in,flag)
%If flag is true,
%   in = stem position (0 to 1)
%   out = Cv
%else
%   in = Cv
%   out = stem position

sd = [0:0.1:1]';
cvd = [0; 0.84; 1.49; 6.68; 12.3; 17.3; 22.1; 26.7; 30.9; 34.4; 36.1];

if flag %stem is input, cv is output
    if any(in<(-eps) | in>(1+eps)); error('stem position must be between 0 and 1'); end
    out = interp1(sd,cvd,in,'linear');
else
    if any(in>(max(cvd)+eps) | in<-eps); error('Cv range error'); end
    out = interp1(cvd,sd,in,'linear');
end

%Cv = (5e-7) .* u.^4 - 0.0002.*u.^3 + 0.0179.*u.^2 - 0.1729.*u +0.1748;

%0.0000005.*u*exp(4)-0.0002.*u*exp(3)+0.0179.*u.*exp(2)-0.1729.*u+0.1748;
end

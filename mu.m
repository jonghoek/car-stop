function[o,sOpt]=mu(s)
%adaptive optimal slip ratio estimator for effective braking on a
%non-uniform condition road
global dry wet snow ice
%dry


if dry==1
    c1=1.25;
    c2=23.99;
    c3=0.52;
end

if wet==1
    %wet
    c1=0.857;
    c2=33.82;
    c3=0.34;
end

if snow==1
    %snow
    c1=0.194;
    c2=94.12;
    c3=0.06;
end

if ice==1
    c1=0.05;
    c2=306;
    c3=0;
end
o=c1*(1-exp(-c2*s))-c3*s;
sOpt=-log(c3/c1/c2)/c2;



if ice==1
    sOpt=0.01;
end
% if s<0.1
%     o=s*7.5;
% else
%     slope=(0.35-0.75)/(1-0.1);
%     o=slope*(s-0.1)+0.75;
% end
% if o>1
%     o=1
% end
% if o<0
%     o=0.1;
% end

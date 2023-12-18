close all
clear all

global dry wet snow ice

kJ=20;%pid gain search interval
Tb_max=10^4;
Tb_min=0;
d2r=pi/180;
kmHourToMeterSec=1000/3600;
rho=1.225;%air density
weight=2045;%directed adaptive neural control of antilock braking systems incorporated with passive suspension dynamics
Cd=0.45;%drag coefficient
Cr=0.02;%car tires on gravel
%Cr=0.05;%car tire on solid sand, gravel loose worn, soil medium hard
dt=0.001;
R=0.5;%tire wheel radius: directed adaptive neural control of antilock braking systems incorporated with passive suspension dynamics
T=1000;
Rg=1;
J=1.5;%: directed adaptive neural control of antilock braking systems incorporated with passive suspension dynamics
g=9.8;
B=0.08;%wheel bearing friction coefficient:: directed adaptive neural control of antilock braking systems incorporated with passive suspension dynamics
%useSafetyControl=0;
vInit=30;

%% parameters
VolMax=1000;%max voltage.
PID=1;
scn=2;
control=1;%control:0 when optimal w0 is used
useFuzz=0;
slantNoise=1;

%% plot
muSetplot=[];
sSetPlot=[];
mu2Setplot=[];

for s=0:0.01:1
    dry=1;wet=0;snow=0;
    m=mu(s);
    muSetplot=[muSetplot m]
    sSetPlot=[sSetPlot s];
    dry=0;wet=1;snow=0;
    m=mu(s);
    mu2Setplot=[mu2Setplot m]

end
plot(sSetPlot,muSetplot)
hold on
plot(sSetPlot,mu2Setplot)
xlabel('S')
ylabel('\mu(S)')
legend('dry road','wet road')

% 
% 
% dry=1;
% wet=0;
% snow=0;
% ice=0;
% 
% theta=-20*d2r;%road slant angle (minus angle-> steep road)
k1=400;
K=0.8;%braking gain: directed adaptive neural control of antilock braking systems incorporated with passive suspension dynamics
%%
%w=wE*Rg;

ID=0;



dmin=inf;

for k1=51%51
    ID=ID+1;
    ID2=0;
    for k2=51%1:kJ:51
        ID2=ID2+1;
        v=vInit;





        wDot=0;
        wDotPrev=0;
        wDDot=0;
        Dist=0;
        wDotMax=1;


        w=v/R;
        wPrev=w;
        dist=0;
        alpha=1;
        S_o=0.16;%: directed adaptive neural control of antilock braking systems incorporated with passive suspension dynamics
        sSet{ID}=[];
        muSet{ID}=[];
        wSet{ID}=[];
        pSet{ID}=[];
        eSet{ID}=[];
        vSet{ID}=[];
        Dset{ID}=[] ;
        tSet{ID}=[];
        wDotSet{ID}=[];
        wSet{ID}=[];
        wOset{ID}=[];
        thetaSet{ID}=[];
        errorInt=0;
        for t=0:dt:T

            if scn==1
                theta=rand*slantNoise*d2r;
                dry=1;wet=0;snow=0;
            elseif scn==2
                theta=(-20+rand*slantNoise)*d2r;
                dry=1;wet=0;snow=0;
            else
                dry=0;wet=1;snow=0;
                if dist<50
                    theta=-30*d2r+rand*slantNoise*d2r
                else
                    theta=-20*d2r+rand*slantNoise*d2r
                end
            end
            
            S=1-R*w/v;
            % if S<0
            %     S=0;
            % elseif S>1
            %     S=1;
            % end

            [mu_S,S_opt]=mu(S);

            S_o=S_opt;
            w_o=(1-S_o)*v/R;%optimal angular velocity
            %         ds=0.001;
            %         if S_opt>S
            %             S_o=S+ds
            %         else
            %             S_o=S-ds
            %         end

            %         if mu_S<0
            %             mu_S=0;
            %         end
            Fx=mu_S*weight*g*cos(theta);
            resist=0.5*rho*Cd*v^2+weight*g*sin(theta)+Fx;

            if v<=.1
                disp('zero speed')
                break;
            end
            v
            vDot=-resist/weight;

            %%
            v=v+vDot*dt;

            dist=dist+v*dt;

            if dist>1000
                break;
            end
            %%
            %wDot<=vDot/R;


            %if mu_S<0
            %    w_o=w
            %end

            if control==0 %&& PID==0
                w=w_o;%w+wDot*dt;
                wDot=(w-wPrev)/dt;
                wDDot=(wDot-wDotPrev)/dt;
            elseif PID==1
                error=w_o-w;

                errorInt=errorInt+error;
                Va=k1*error+k2*errorInt;
                if Va<-VolMax
                    Va=-VolMax;
                end
                if Va>VolMax
                    Va=VolMax;
                end
                wDDot=-10*w-200*wDot+36/50*Va;
                wDot=wDot+wDDot*dt;
                w=w+wDot*dt;
            else
                %%
                x=w-w_o;
                %error=S-S_o;


                sig=sign(x);%exp(x)/(1+exp(x));%sigmoid(w-w_o);
                Va=(50*wDDot+500*w-10^4*k1*(sig))/36;
                if Va<-VolMax
                    Va=-VolMax;
                end
                if Va>VolMax
                    Va=VolMax;
                end
                %if Va<0: reverse rotation possible
                %    Va=0
                %end
                wDDot=-10*w-200*wDot+36/50*Va;
                wDot=wDot+wDDot*dt;
                w=w+wDot*dt;
            end
            %wDot=vDot/R;


            wPrev=w;
            wDotPrev=wDot;


            

            wOset{ID}=[wOset{ID} w_o];

            %   pSet=[pSet pF];
            muSet{ID}=[muSet{ID} mu_S];
            sSet{ID}=[sSet{ID} S];
            tSet{ID}=[tSet{ID} t];
            vSet{ID}=[vSet{ID} v];
            wSet{ID}=[wSet{ID} w];
            thetaSet{ID}=[thetaSet{ID} theta];


            % eSet=[eSet error];
        end

        Dset{ID}{ID2}=dist;
        if Dset{ID}{ID2}<dmin
            IDopt=ID;
            ID2opt=ID2;
            dmin=Dset{ID}{ID2};
        end


    end
end

dmin
IDopt
ID2opt

k1=1+IDopt*kJ;
k2=1+ID2opt*kJ;
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%
%%%%%%

%%%%%%

v=vInit;%50*kmHourToMeterSec;





wDot=0;
wDotPrev=0;
wDDot=0;
Dist=0;
wDotMax=1;
ID=1;

w=v/R;
wPrev=w;
dist=0;
alpha=1;
S_o=0.16;%: directed adaptive neural control of antilock braking systems incorporated with passive suspension dynamics
sSet{ID}=[];
VaSet{ID}=[];
muSet{ID}=[];
wSet{ID}=[];
pSet{ID}=[];
eSet{ID}=[];
vSet{ID}=[];
dSet{ID}=[] ;
tSet{ID}=[];
wDotSet{ID}=[];
wSet{ID}=[];
wOset{ID}=[];
thetaSet{ID}=[];
errorInt=0;
for t=0:dt:T

    if scn==1
        theta=rand*slantNoise*d2r;
        dry=1;wet=0;snow=0;
    elseif scn==2
        theta=(-20+rand*slantNoise)*d2r;
        dry=1;wet=0;snow=0;
    else
        dry=0;wet=1;snow=0;
        if dist<50
            theta=-30*d2r+rand*slantNoise*d2r
        else
            theta=-20*d2r+rand*slantNoise*d2r
        end
    end
    %%
    S=1-R*w/v;
    % if S<0
    %     S=0;
    % elseif S>1
    %     S=1;
    % end


    [mu_S,S_opt]=mu(S);

    S_o=S_opt;
    w_o=(1-S_o)*v/R;%optimal angular velocity
    %         ds=0.001;
    %         if S_opt>S
    %             S_o=S+ds
    %         else
    %             S_o=S-ds
    %         end

    %         if mu_S<0
    %             mu_S=0;
    %         end
    Fx=mu_S*weight*g*cos(theta);
    resist=0.5*rho*Cd*v^2+weight*g*sin(theta)+Fx;

    if v<=.1
        disp('zero speed')
        break;
    end
    v
    vDot=-resist/weight
    %wDot<=vDot/R;


    %if mu_S<0
    %    w_o=w
    %end

    if control==0 %&& PID==0
        w=w_o;%w+wDot*dt;
        wDot=(w-wPrev)/dt;
        wDDot=(wDot-wDotPrev)/dt;
        Va=0;
    elseif PID==1
        error=w_o-w;

        errorInt=errorInt+error;
        Va=k1*error+k2*errorInt;
        if Va<-VolMax
            Va=-VolMax;
        end
        if Va>VolMax
            Va=VolMax;
        end
        wDDot=-10*w-200*wDot+36/50*Va;
        wDot=wDot+wDDot*dt;
        w=w+wDot*dt;
    else
        %%
        x=w-w_o;
        %error=S-S_o;


        sig=sign(x);%exp(x)/(1+exp(x));%sigmoid(w-w_o);
        Va=(50*wDDot+500*w-10^4*k1*(sig))/36;
        if Va<-VolMax
            Va=-VolMax;
        end
        if Va>VolMax
            Va=VolMax;
        end
        %if Va<0: reverse rotation possible
        %    Va=0
        %end
        wDDot=-10*w-200*wDot+36/50*Va;
        wDot=wDot+wDDot*dt;
        w=w+wDot*dt;
    end
    %wDot=vDot/R;


    wPrev=w;
    wDotPrev=wDot;


    %%
    v=v+vDot*dt;

    dist=dist+v*dt;

    if dist>1000
        break;
    end

    wOset{ID}=[wOset{ID} w_o];
    dSet{ID}=[dSet{ID} dist];
    %   pSet=[pSet pF];
    muSet{ID}=[muSet{ID} mu_S];
    sSet{ID}=[sSet{ID} S];
    tSet{ID}=[tSet{ID} t];
    vSet{ID}=[vSet{ID} v];
    wSet{ID}=[wSet{ID} w];
    thetaSet{ID}=[thetaSet{ID} theta];
    VaSet{ID}=[VaSet{ID} Va];


    % eSet=[eSet error];
end


%%%%
figure

for ID=1%:length(Dset):length(Dset)
    hold on
    plot(tSet{ID},thetaSet{ID}*180/pi)
    hold on
    grid on
    xlabel('t(sec)')
    ylabel('\theta(degrees)')
end



figure

for ID=1%:length(Dset)
    hold on
    plot(tSet{ID},wOset{ID})
    hold on
    grid on
    xlabel('t(sec)')
    ylabel('wO')
end

figure
for ID=1%:length(Dset):length(Dset)
    hold on
    plot(tSet{ID},muSet{ID})
    hold on
    grid on
    xlabel('t(sec)')
    ylabel('muS')
end


figure
for ID=1%:length(Dset):length(Dset)
    hold on
    plot(tSet{ID},VaSet{ID})
    hold on
    grid on
    xlabel('t(sec)')
    ylabel('Va(V)')
end

figure

for ID=1%:length(Dset):length(Dset)

    S_oSet=ones(1,length(sSet{ID}))*S_o
    plot(tSet{ID},sSet{ID})
    hold on
    plot(tSet{ID},S_oSet,'ro')
    hold on
    legend('SlipRatio','optimalSlipRatio')
    grid on
    xlabel('t(sec)')
    ylabel('slip ratio(S)')
end


figure
for ID=1%:length(Dset):length(Dset)

    subplot(4,1,1)
    hold on
    plot(tSet{ID},vSet{ID})
    hold on
    grid on
    xlabel('t(sec)')
    ylabel('speed(m/s)')
    title('a')

    subplot(4,1,2)
    hold on
    plot(tSet{ID},dSet{ID})
    hold on
    grid on
    xlabel('t(sec)')
    ylabel('d(m)')
    title('b')

    subplot(4,1,3)
    hold on
    plot(tSet{ID},wSet{ID})
    hold on
    plot(tSet{ID},wOset{ID})
    grid on
    xlabel('t(sec)')
    ylabel('w(rad/s)')
    legend('w','w^o')
    title('c')

    subplot(4,1,4)
    hold on
    plot(tSet{ID},thetaSet{ID}*180/pi)
    grid on
    xlabel('t(sec)')
    ylabel('\theta(deg)')
    legend('\theta')
    title('d')

end

k1opt=k1
k2opt=k2

dist
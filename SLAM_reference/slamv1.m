clc; clear all; close all;

lmx = [0 5 -9 12];
lmy = [-6 2 16 5];
numlm = numel(lmx);
nump = 500;
idxp = 1:1:nump;
time1 = 0:0.01:30;
chi = 0:2*pi/nump:2*pi-2*pi/nump;
px = 11; py = 0;
stdn = 0.5;
ran_tru = range_v1(px,py,lmx,lmy,stdn);

p_x = px+ran_tru*cos(chi);
p_y = py+ran_tru*sin(chi);
p_x1 = p_x;
p_y1 = p_y;

pp1 = plot(px,py,'ro');
hold on;
pp2 = plot(p_x(1,:),p_y(1,:),'b.');
pp3 = plot(p_x(2,:),p_y(2,:),'m.');
pp4 = plot(p_x(3,:),p_y(3,:),'c.');
pp5 = plot(p_x(4,:),p_y(4,:),'g.');
pp6 = plot(lmx,lmy,'dk');
axis([-21 21 -21 21]);

con = 2*stdn^2;
r = 11;

for i = 1:1:numel(time1)
    %true states 
    px = r*cos(time1(i));
    py = r*sin(time1(i));
    ran_tru = range_v1(px,py,lmx,lmy,stdn);
    for j = 1:1:numlm
        x_p = p_x(j,:);%+0.1*randn(1,nump);
        y_p = p_y(j,:);%+0.1*randn(1,nump);
        ran_est = range_v1(px,py,x_p,y_p,stdn);
        err = (ran_tru(j) - ran_est).^2;
        nump_sample = 0;
        idxrep = [];
        idxrem = [];
        idxmean = [];
        for k = 1:1:numel(err)
            if err(k)>5*exp(-i/2)+0.025
                nump_sample = nump_sample+1;
                idxrep = [idxrep k];
            end
            if err(k)>0.1
                idxrem = [idxrem k];
            end
        end
        %wt = 1./err;
        wt = sqrt(1/(con*pi))*exp(-err/con);
        n_wt = wt/sum(wt);
        newidx = datasample(idxp,nump_sample,'Replace',true,'Weights',n_wt);
        p_x(j,idxrep) = x_p(newidx);%+0.001*rand(1,nump);
        p_y(j,idxrep) = y_p(newidx);%+0.001*rand(1,nump);
        %p_x(j,idxrem) = p_x(j,idxrem)+(0.5*exp(-i/5)+0.02)*randn(1,numel(idxrem));
        %p_y(j,idxrem) = p_y(j,idxrem)+(0.5*exp(-i/5)+0.02)*randn(1,numel(idxrem));
        p_x(j,idxrem) = p_x(j,idxrem)+(0.5*exp(-i/2)+0.02)*randn(1,numel(idxrem));
        p_y(j,idxrem) = p_y(j,idxrem)+(0.5*exp(-i/2)+0.02)*randn(1,numel(idxrem));
        px_mean(i,j) = sum(p_x(j,:))/nump;
        py_mean(i,j) = sum(p_y(j,:))/nump;
        px_std(i,j) = std(p_x(j,:));
        py_std(i,j) = std(p_y(j,:));
    end
    set(pp1,'xData',px,'yData',py);
    set(pp2,'xData',p_x(1,:),'yData',p_y(1,:));
    set(pp3,'xData',p_x(2,:),'yData',p_y(2,:));
    set(pp4,'xData',p_x(3,:),'yData',p_y(3,:));
    set(pp5,'xData',p_x(4,:),'yData',p_y(4,:));
    %pause(0.001);
    
end


nump1 = 1000;
idxp1 = 1:1:nump1;
time1 = 30:0.01:50;




for i = 1:1:numel(time1)
    px = r*cos(time1(i)); py = r*sin(time1(i));
    ran_tru = range_v1(px,py,lmx,lmy,stdn);
    for j = 1:1:numlm
        x_p = p_x(j,:);%+0.1*randn(1,nump);
        y_p = p_y(j,:);%+0.1*randn(1,nump);
        ran_est = range_v1(px,py,x_p,y_p,stdn);
        err = (ran_tru(j) - ran_est).^2;
        nump_sample = 0;
        idxrep = [];
        idxrem = [];
        idxmean = [];
        for k = 1:1:numel(err)
            if err(k)>5*exp(-i/2)+0.025
                nump_sample = nump_sample+1;
                idxrep = [idxrep k];
            end
            if err(k)>0.1
                idxrem = [idxrem k];
            end
        end
        %wt = 1./err;
        wt = sqrt(1/(con*pi))*exp(-err/con);
        n_wt = wt/sum(wt);
        newidx = datasample(idxp,nump_sample,'Replace',true,'Weights',n_wt);
        p_x(j,idxrep) = x_p(newidx);%+0.001*rand(1,nump);
        p_y(j,idxrep) = y_p(newidx);%+0.001*rand(1,nump);
        %p_x(j,idxrem) = p_x(j,idxrem)+(0.5*exp(-i/5)+0.02)*randn(1,numel(idxrem));
        %p_y(j,idxrem) = p_y(j,idxrem)+(0.5*exp(-i/5)+0.02)*randn(1,numel(idxrem));
        p_x(j,idxrem) = p_x(j,idxrem)+(0.5*exp(-i/2)+0.02)*randn(1,numel(idxrem));
        p_y(j,idxrem) = p_y(j,idxrem)+(0.5*exp(-i/2)+0.02)*randn(1,numel(idxrem));
        px_mean(i,j) = sum(p_x(j,:))/nump;
        py_mean(i,j) = sum(p_y(j,:))/nump;
        px_std(i,j) = std(p_x(j,:));
        py_std(i,j) = std(p_y(j,:));
    end
    set(pp1,'xData',px,'yData',py);
    set(pp2,'xData',p_x(1,:),'yData',p_y(1,:));
    set(pp3,'xData',p_x(2,:),'yData',p_y(2,:));
    set(pp4,'xData',p_x(3,:),'yData',p_y(3,:));
    set(pp5,'xData',p_x(4,:),'yData',p_y(4,:));
    %pause(0.001);
    
end


nump1 = 1000;
idxp1 = 1:1:nump1;
time1 = 0:0.01:6;

r = 11;
%px = 0; py = 0;
stdn = 0.5;
cov = 2*stdn^2;

p_x1 = px+50*stdn*randn(nump1,1); p_y1 = py+50*stdn*randn(nump1,1);
wt = zeros(nump1,1);

pp1 = plot(px,py,'ro');
hold on;
qq2 = plot(p_x1,p_y1,'b.');
%axis([-18 18 -18 18]);
qq3 = plot(lmx,lmy,'dk');

for i = 1:1:numel(time1)
    px = px + 0.2*time1(i) + (1/2)*(sin(2*pi*i))*(time1(i)^2); py = py+ 0.1*time1(i);
    ran_tru = range_v1(px,py,lmx,lmy,stdn);
    p_x1 = p_x1+0.2*time1(i)+(1/2)*(sin(2*pi*i))*(time1(i)^2); p_y1 = p_y1+0.1*time1(i);
    ran_est = range_v1(p_x1,p_y1,lmx,lmy,stdn);
        
    for j = 1:1:numlm
        err = (ran_tru(j)-ran_est(j,:)').^2;
        wt = wt+(1/sqrt(pi*cov))*exp(-err/cov);
    end
    
    wtn = wt./sum(wt);
    set(pp1,'xData',px,'yData',py);
    set(qq2,'xData',p_x1,'yData',p_y1);
    newidx = datasample(idxp1,nump1,'Replace',true,'Weights',wtn);
    p_x1 = p_x1(newidx)+(0.1*exp(-time1(i))+0.01)*randn(nump1,1);
    p_y1 = p_y1(newidx)+(0.1*exp(-time1(i))+0.01)*randn(nump1,1);
    
    pxmean(i) = sum(p_x1)/nump1;
    pymean(i) = sum(p_y1)/nump1;
    pxstdp(i) = px+3*std(p_x1);pxstdn(i) = px-3*std(p_x1);
    pystdp(i) = py+3*std(p_y1);pystdn(i) = py-3*std(p_y1);
    
    posx(i) = px; posy(i) = py;
    
    pause(0.02);
    
    %p_x = p_x+0.2*t(i)+(1/2)*(t(i)^2); p_y = p_y+0.1*t(i);
end
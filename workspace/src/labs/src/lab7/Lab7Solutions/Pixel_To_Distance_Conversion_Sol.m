%% Author: Tony Zheng
clear
clc
close all

%% Mapping a forward distance (with no lateral movement) to a number of y_newPixels
%xI = 1.5,2ft,2.5ft,3ft,3.5ft,4ft,4.5ft,5ft, yI=0
f = @(xy) [xy(:,1)-320,480-xy(:,2)];
xpyp=[322,424;
    320,358;
    319,324;
    318,303;
    318,288;
    316,278;
    316,270;
    316,263];
xpyp=f(xpyp);
xi = [1.5;2;2.5;3;3.5;4;4.5;5];
xI_to_ynp_eq = polyfit([1.2;xi],[0;xpyp(:,2)],4);
xI_to_ynp_vals =  polyval(xI_to_ynp_eq,0:.1:5);
plot(xi,xpyp(:,2),[0:.1:5],xI_to_ynp_vals,'--')
xlabel('x_{Inertial} = 1 to 5 ft, y_{Inertial} = 0 ft')
ylabel('y_{newPixel}')
title('x_{Inertial} vs y_{newPixel} with a fixed 0 lateral distance')


%% Flipped axis to get the equation in reverse
figure
ynp_to_xI_eq = polyfit([0;xpyp(:,2)],[1.2;xi],4);
ynp_to_xI_vals = polyval(ynp_to_xI_eq,[0:220]);
plot(xpyp(:,2),xi,[0:220],ynp_to_xI_vals,'--')
ylabel('x_{Inertial} = 1 to 5 ft, y_{Inertial} = 0 ft')
xlabel('y_{newPixel}')
legend('4th order polynomial','Location','Best')
title('FLIPPED:y_{newPixel} vs x_{Inertial} with a fixed 0 lateral distance')

%% Plotting the objects at varying y_Inertial for different x_Inertial 
% xi = 2, yi by 0.25ft
x2 = [320,358;
      386,355
      452,355;
      522,354;
      597,355;];
x2 = f(x2);
x2yI = [0;0.25;0.5;0.75;1];

%yi = .5,.75,1,1.5
x3 = [318,303;
    399,301;
    440,300;
    481,299;
    569,297;]
x3yI = [0;0.5;0.75;1;1.5];
x3=f(x3);
    %.5s
x4 = [316,278;
    374,273;
    432,272;
    491,271;
    555,271;]
x4yI = [0;0.5;1;1.5;2];
x4=f(x4);
x5 = [316,263;
    360,259;
    406,259;
    452,259;
    495,257];
x5yI = [0;0.5;1;1.5;2];
x5=f(x5);
figure
plot(x2(:,1),x2(:,2),x3(:,1),x3(:,2),x4(:,1),x4(:,2),x5(:,1),x5(:,2))
xlabel('x_{newPixel}')
ylabel('y_{newPixel}')  
title('Pixels only: x_{newPixel} vs y_{newPixel}')

%% Changing the previous plot to have the given y_Inertial rather than x_newPixel for the x axis
figure
plot(x2yI,x2(:,2),x3yI,x3(:,2),x4yI,x4(:,2),x5yI,x5(:,2))
xlabel('-y_{Inertial}')
ylabel('y_{newPixel}')  
title('Lateral motion: y_{Inertial} vs y_{newPixel}')

%% Offset each set of measurements at different x_Inertials so that the y_newPixel starts at 0 rather than the given y_newPixel (subtract each row by the first value)
yntrunc = @(xy) [xy(:,2)-xy(1,2)];
y2 = yntrunc(x2);
y3 = yntrunc(x3);
y4 = yntrunc(x4);
y5 = yntrunc(x5);

figure
plot(x2yI,y2,'r',x3yI,y3,'g',x4yI,y4,'b',x5yI,y5,'c')
xlabel('-y_{Inertial}')
ylabel('y_{newPixel}') 
%legend('x_I = 2ft','x_I = 3ft','x_I = 4ft','x_I = 5ft')
title('Lateral motion (pixel start at 0): y_{Inertial} vs y_{newPixel}')

%% Find a polynomial fit for all those data points
m_yiyp=@(yi,yp) polyfit(yi,yp,3);

 m_yiyp2 = m_yiyp(x2yI,y2)
 m_yiyp3 = m_yiyp(x3yI,y3)
 m_yiyp4 = m_yiyp(x4yI,y4)
 m_yiyp5 = m_yiyp(x5yI,y5)
 
 x2m_yiyp = polyval(m_yiyp2,[0:0.1:x2yI(end)]);
 x3m_yiyp = polyval(m_yiyp3,[0:0.1:x3yI(end)]);
 x4m_yiyp = polyval(m_yiyp4,[0:0.1:x4yI(end)]);
 x5m_yiyp = polyval(m_yiyp5,[0:0.1:x5yI(end)]);
 
mall = m_yiyp([[0:0.1:x2yI(end)]';[0:0.1:x3yI(end)]';[0:0.1:x4yI(end)]';[0:0.1:x5yI(end)]'],[x2m_yiyp';x3m_yiyp';x4m_yiyp';x5m_yiyp']);

xyall = polyval(mall,[[0:0.1:x2yI(end)]';[0:0.1:x3yI(end)]';[0:0.1:x4yI(end)]';[0:0.1:x5yI(end)]']);
xxxyall = [[0:0.1:x2yI(end)]';[0:0.1:x3yI(end)]';[0:0.1:x4yI(end)]';[0:0.1:x5yI(end)]'];

hold on
plot(xxxyall,xyall,'black*',[0:0.1:x2yI(end)],x2m_yiyp,'r--',[0:0.1:x3yI(end)],x3m_yiyp,'g--',[0:0.1:x4yI(end)],x4m_yiyp,'b--',[0:0.1:x5yI(end)],x5m_yiyp,'c--')
legend('x_I = 2ft','x_I = 3ft','x_I = 4ft','x_I = 5ft','3rd order poly','Location','best')

%% Plot the varying lateral distance y_Inertial vs. x_newPixel

figure
plot(x2yI,x2(:,1),x3yI,x3(:,1),x4yI,x4(:,1),x5yI,x5(:,1))
xlabel('-y_{Inertial}')
ylabel('x_{newPixel}')  

%% Calculate the slope of the line for each x_Inertial
 m=@(yi,xn) polyfit(yi,xn,1);
 m2 = m(x2yI,x2(:,1))
 m3 = m(x3yI,x3(:,1))
 m4 = m(x4yI,x4(:,1))
 m5 = m(x5yI,x5(:,1))
 
 x2m = polyval(m2,x2yI);
 x3m = polyval(m3,x3yI);
 x4m = polyval(m4,x4yI);
 x5m = polyval(m5,x5yI);
 
%% Taking a polyfit of each slope vs x_inertial
mlist = [2,m2(1);
    3,m3(1);
    4,m4(1);
    5,m5(1)];
mpoly = polyfit(mlist(:,1),mlist(:,2),3);
mvals = polyval(mpoly,mlist(:,1));

%% Taking a polyfit of each offset vs x_inertial
blist = [2,m2(2);
    3,m3(2);
    4,m4(2);
    5,m5(2)];
bpoly = polyfit(blist(:,1),blist(:,2),3);
bvals = polyval(bpoly,blist(:,1));

%plot(mlist(:,1),mlist(:,2),mlist(:,1),mvals)
% 3rd order fit between the xI and m

%% Example of an arbitrary x_inertial to see if it lines up with my expectations
yis = 0:.2:2;
xnpis = polyval(mpoly,2.5)*yis+polyval(bpoly,2.5);
hold on
plot(yis,xnpis)

%% Plotting the given x_inertials with my slope equation
hold on
plot(x2yI,x2m,'--',x3yI,x3m,'--',x4yI,x4m,'--',x5yI,x5m,'--')
title('Lateral motion: y_{Inertial} vs x_{newPixel} at varying forward distance')
legend('x_I = 2ft','x_I = 3ft','x_I = 4ft','x_I = 5ft','Test: x_I = 2.5ft','Location','best')

%% Plot slope vs x_inertial
figure
plot(mlist(:,1),mlist(:,2))
title('Slope vs. x_{Inertial}')
xlabel('x_{Inertial}')
ylabel('Slope (x_{newPixel}/y_{Inertial})')

%% Plot offset B vs x_inertial
figure
plot(blist(:,1),blist(:,2))
title('Offset b vs. x_{Inertial}')
xlabel('x_{Inertial}')
ylabel('b (x_{newPixel}/y_{Inertial})')

%% Plotting the original points (Given x_inertial and y_inertial, when I run them through my equations, do they match the measured pixel positions?)
figure
calc_ynp_to_xI = @(ynp) polyval(ynp_to_xI_eq,ynp);
calc_xnp_to_yI = @(xI,xnp) (xnp-polyval(bpoly,xI))./polyval(mpoly,xI);

x2I = calc_ynp_to_xI(x2(:,2))
y2I = calc_xnp_to_yI(x2I,x2(:,1))
x2xI = 2*ones(length(x2yI),1);

x3I = calc_ynp_to_xI(x3(:,2))
y3I = calc_xnp_to_yI(x3I,x3(:,1))
x3xI = 3*ones(length(x3yI),1);

x4I = calc_ynp_to_xI(x4(:,2))
y4I = calc_xnp_to_yI(x4I,x4(:,1))
x4xI = 4*ones(length(x4yI),1);

x5I = calc_ynp_to_xI(x5(:,2))
y5I = calc_xnp_to_yI(x5I,x5(:,1))
x5xI = 5*ones(length(x5yI),1);

plot(x2yI,x2xI,'o',y2I,x2I,'x')
hold on 
plot(x3yI,x3xI,'o',y3I,x3I,'x')
plot(x4yI,x4xI,'o',y4I,x4I,'x')
plot(x5yI,x5xI,'o',y5I,x5I,'x')
ylim([0 6])
ylabel('x_{Inertial}')
xlabel('-y_{Inertial}')
title('Skewed')

%% Skew Correction
calc_yI_yp_skew = @(yI) polyval(mall,yI);

yp2skew = calc_yI_yp_skew(y2I)
x2Iskew = calc_ynp_to_xI(x2(:,2)-yp2skew)
y2Iskew = calc_xnp_to_yI(x2Iskew,x2(:,1))

yp3skew = calc_yI_yp_skew(y3I)
x3Iskew = calc_ynp_to_xI(x3(:,2)-yp3skew)
y3Iskew = calc_xnp_to_yI(x3Iskew,x3(:,1))

yp4skew = calc_yI_yp_skew(y4I)
x4Iskew = calc_ynp_to_xI(x4(:,2)-yp4skew)
y4Iskew = calc_xnp_to_yI(x4Iskew,x4(:,1))

yp5skew = calc_yI_yp_skew(y5I)
x5Iskew = calc_ynp_to_xI(x5(:,2)-yp5skew)
y5Iskew = calc_xnp_to_yI(x5Iskew,x5(:,1))

figure
plot(x2yI,x2xI,'o',y2Iskew,x2Iskew,'x')
hold on 
plot(x3yI,x3xI,'o',y3Iskew,x3Iskew,'x')
plot(x4yI,x4xI,'o',y4Iskew,x4Iskew,'x')
plot(x5yI,x5xI,'o',y5Iskew,x5Iskew,'x')
ylim([0 6])
ylabel('x_{Inertial}')
xlabel('-y_{Inertial}')
title('Corrected')
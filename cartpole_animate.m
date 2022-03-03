% Asutay Ozmen, March 2020
% modified to fit ECE147B cart-pole model, February 2021
function cartpole_animate(t_ode,X_ode,f,label)
% Inputs:
% t_ode: time vector
% X_ode: Matrix containing the states over time [x,dx,theta,dtheta]
% f: figure number that you want to animate the cartpole in to avoid
% animating over existing figures
% t_ode and X_ode are assumed each to be column vectors or consisting of
% column vectors (i.e size of t_ode would be 1000x1 and size of X_ode would be 1000x4) 
% if you'd like to input row vectors then please uncomment the following lines (lines 12 and 13)
%X_ode = X_ode';
%t_ode = t_ode';
if ~exist('f','var')
     f = 45; 
end

dt = .07; %to increase motion resolution decrease dt but will be a slower animation, for faster animation increase dt
tlist = min(t_ode):dt:max(t_ode);
Xlist = zeros(4,length(tlist));
for i=1:4
    Xlist(i,:) = interp1(t_ode,X_ode(:,i),tlist);
end
dim = [.7 .7 .2 .2];
for n=1:length(tlist)
    figure(f);
    delete(findall(gcf,'type','annotation'));
    str=sprintf('t = %.2f',tlist(n));
    annotation('textbox',dim,'String',str,'FitBoxToText','on');
    cartpole_draw(Xlist(:,n),f,label); plot(Xlist(1,1:n)+sin(Xlist(3,1:n)),cos(Xlist(3,1:n)));
    axis([-(max(abs(Xlist(1,1:end)+max(sin(Xlist(3,1:end)))))+.25) max(abs(Xlist(1,1:end)+max(sin(Xlist(3,1:end)))))+.25 -1.5 1.5]); 
    drawnow limitrate; %pause(0.001);
end

function cartpole_draw(X,f,label)
x=X(1); q1 = X(3);
L1 = 1;
th = .5*.03;                    % half-THickness of arm

avals = pi*[0:.05:1];
x1 = [0 L1 L1+th*cos(avals-pi/2) L1 0 th*cos(avals+pi/2)];
y1 = [-th -th th*sin(avals-pi/2) th th th*sin(avals+pi/2)];
r1 = (x1.^2 + y1.^2).^.5;
a1 = atan2(y1,x1);
x1draw = x + r1.*sin(a1+q1);    % x pts to plot, for Link 1
y1draw = r1.*cos(a1+q1);        % y pts to plot, for Link 1
x1end = x+L1*sin(q1);           % "elbow" at end of Link 1, x
y1end = L1*cos(q1);             % "elbow" at end of Link 1, y

% cart
x2 = [x-.3 x+.3 x+.3 x-.3];
y2 = [-.2 -.2 .2 .2 ];

%draw the cart-pole
figure(f); 

delete(gca);
p1 = patch(x1draw,y1draw,'b','FaceAlpha',.3); hold on                                                               %pole
c1 = rectangle('Position',[x-.25 -.15 .5 .3],'Curvature',0.2, 'FaceColor', [1, 0, 0, .3]);                          %cart
% m1 = rectangle('Position', [x+L1*sin(q1)-.075 L1*cos(q1)-.075 .15 .15],'Curvature',1, 'FaceColor', [0, 1, 1, .3]);  %mass
yline(-.155);
xline(0,'-.');
axis equal; 
title(label)

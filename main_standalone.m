
%% ARS4 Project - Master 2 University of Technology of COMPIEGNE 
% Fall 2021

% Done by : 
% % Mohamad HAMMOUD 
% % Fadel TARHINI

% Supervised by : M.Philippe Bonnifait
% Date : 24/01/2022

%% Part 1 : Standalone EKF Perception

%%
clear;
clc;

%%
load 'ARS4-Project.mat' % Time, Blue, Grey, White

k_start = 1;
k_end = size(time,1);
steps = k_end - k_start + 1;
t = time(k_start);
risk = 0.01;
live_display = true;

% % Initialize States (Blue, Grey, White as seen by Blue, Grey and White)
Xbb = zeros(5,1); Xbg = zeros(5,1); Xbw = zeros(5,1);
Pbb = ones(5,5)*100; Pbg = ones(5,5)*100; Pbw = ones(5,5)*100;
Xbb(:) = blue.X_real(:,k_start);

Xgg = zeros(5,1); Xgb = zeros(5,1); Xgw = zeros(5,1);
Pgg = ones(5,5)*100; Pgb = ones(5,5)*100; Pgw = ones(5,5)*100;
Xgg(:) = grey.X_real(:,k_start);

Xww = zeros(5,1); Xwb = zeros(5,1); Xwg = zeros(5,1);
Pww = ones(5,5)*100; Pwb = ones(5,5)*100; Pwg = ones(5,5)*100;
Xww(:) = white.X_real(:,k_start);

Qego = diag([0.05, 0.05, 0.02, 0.05, 0.01]); % Q small for for the ego-state evolution
Qoth = diag([0.05, 0.05, 0.1, 0.5, 0.1]); % Q larger for the prediction of other

% % Initialize Storage
Xbbs = zeros(5, steps); Xbgs = zeros(5, steps); Xbws = zeros(5, steps);
Pbbs = zeros(5, steps); Pbgs = zeros(5, steps); Pbws = zeros(5, steps);
Xggs = zeros(5, steps); Xgbs = zeros(5, steps); Xgws = zeros(5, steps);
Pggs = zeros(5, steps); Pgbs = zeros(5, steps); Pgws = zeros(5, steps);
Xwws = zeros(5, steps); Xwbs = zeros(5, steps); Xwgs = zeros(5, steps);
Pwws = zeros(5, steps); Pwbs = zeros(5, steps); Pwgs = zeros(5, steps);
Ybg = zeros(5,1); Ybw = zeros(5,1); Ygb = zeros(5,1); Ygw = zeros(5,1); Ywb = zeros(5,1); Ywg = zeros(5,1);
Rbg = ones(5,1); Rbw = ones(5,1); Rgb = ones(5,1); Rgw = ones(5,1); Rwb = ones(5,1); Rwg = ones(5,5);

%% Main Loop

for k = k_start:k_end
  
  dt = time(k)-t;
  t = time(k);

% %   Prediction 
  [Xbb, Pbb] = KF_predict(Xbb, Pbb, dt, Qego, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xbg, Pbg] = KF_predict(Xbg, Pbg, dt, Qoth, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xbw, Pbw] = KF_predict(Xbw, Pbw, dt, Qoth, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xgg, Pgg] = KF_predict(Xgg, Pgg, dt, Qego, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xgb, Pgb] = KF_predict(Xgb, Pgb, dt, Qoth, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xgw, Pgw] = KF_predict(Xgw, Pgw, dt, Qoth, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xww, Pww] = KF_predict(Xww, Pww, dt, Qego, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xwb, Pwb] = KF_predict(Xwb, Pwb, dt, Qoth, @f_coordinated_turn, @fJ_coordinated_turn);
  [Xwg, Pwg] = KF_predict(Xwg, Pwg, dt, Qoth, @f_coordinated_turn, @fJ_coordinated_turn);

% %   Update ego (ego : refers to the considered vehicle doing processing about itself)
  if ~isnan(blue.Y_blue(:,k))
    [Xbb, Pbb] = KF_update(Xbb, Pbb, blue.Y_blue(:,k), blue.R_blue, @g_gnss, @gC_gnss);
  end
  
  if ~isnan(grey.Y_grey(:,k))
     [Xgg, Pgg] = KF_update(Xgg, Pgg, grey.Y_grey(:,k), grey.R_grey, @g_gnss, @gC_gnss);
  end
  
  if ~isnan(white.Y_white(:,k))
     [Xww, Pww] = KF_update(Xww, Pww, white.Y_white(:,k), white.R_white, @g_gnss, @gC_gnss);
  end
  
% %  Update other
  if ~isnan(blue.Y_grey(:,k)) % Sometimes a vehicle will not be observed for a few steps (a NaN, Not a Number, denotes a...
                             ...missing value). The estimation should be able to maintain itself until the vehicle is observed again.
    [Ybg, Rbg] = transform_lidar(Xbb, Pbb, blue.Y_grey(:,k), blue.R_grey);
    [Xbg, Pbg] = KF_update(Xbg, Pbg, Ybg, Rbg, @g_lidar, @gC_lidar);
  end
  
  if ~isnan(blue.Y_white(:,k))
    [Ybw, Rbw] = transform_lidar(Xbb, Pbb, blue.Y_white(:,k), blue.R_white);
    [Xbw, Pbw] = KF_update(Xbw, Pbw, Ybw, Rbw, @g_lidar, @gC_lidar);
  end
  
  if ~isnan(grey.Y_blue(:,k))
    [Ygb, Rgb] = transform_lidar(Xgg, Pgg, grey.Y_blue(:,k), grey.R_blue);
    [Xgb, Pgb] = KF_update(Xgb, Pgb, Ygb, Rgb, @g_lidar, @gC_lidar);
  end
  
  if ~isnan(grey.Y_white(:,k))
    [Ygw, Rgw] = transform_lidar(Xgg, Pgg, grey.Y_white(:,k), grey.R_white);
    [Xgw, Pgw] = KF_update(Xgw, Pgw, Ygw, Rgw, @g_lidar, @gC_lidar);
  end
  
  if ~isnan(white.Y_blue(:,k))
    [Ywb, Rwb] = transform_lidar(Xww, Pww, white.Y_blue(:,k), white.R_blue);
    [Xwb, Pwb] = KF_update(Xwb, Pwb, Ywb, Rwb, @g_lidar, @gC_lidar);
  end
  if ~isnan(white.Y_grey(:,k))
    [Ywg, Rwg] = transform_lidar(Xww, Pww, white.Y_grey(:,k), white.R_grey);
    [Xwg, Pwg] = KF_update(Xwg, Pwg, Ywg, Rwg, @g_lidar, @gC_lidar);
  end
  
 %% Storage
 
  Xbbs(:,k) = Xbb;
  Xbbs(3,k) = prepareNormalizedAngleDiff(Xbbs(3,k), blue.X_real(3,k));
  Pbbs(:,k) = diag(Pbb);
  Xbgs(:,k) = Xbg;
  Xbgs(3,k) = prepareNormalizedAngleDiff(Xbgs(3,k), grey.X_real(3,k));
  Pbgs(:,k) = diag(Pbg);
  Xbws(:,k) = Xbw;
  Xbws(3,k) = prepareNormalizedAngleDiff(Xbws(3,k), white.X_real(3,k));
  Pbws(:,k) = diag(Pbw);

  Xggs(:,k) = Xgg;
  Xggs(3,k) = prepareNormalizedAngleDiff(Xggs(3,k), grey.X_real(3,k));
  Pggs(:,k) = diag(Pgg);
  Xgbs(:,k) = Xgb;
  Xgbs(3,k) = prepareNormalizedAngleDiff(Xgbs(3,k), blue.X_real(3,k));
  Pgbs(:,k) = diag(Pgb);
  Xgws(:,k) = Xgw;
  Xgws(3,k) = prepareNormalizedAngleDiff(Xgws(3,k), white.X_real(3,k));
  Pgws(:,k) = diag(Pgw);

  Xwws(:,k) = Xww;
  Xwws(3,k) = prepareNormalizedAngleDiff(Xwws(3,k), white.X_real(3,k));
  Pwws(:,k) = diag(Pww);
  Xwbs(:,k) = Xwb;
  Xwbs(3,k) = prepareNormalizedAngleDiff(Xwbs(3,k), blue.X_real(3,k));
  Pwbs(:,k) = diag(Pwb);
  Xwgs(:,k) = Xwg;
  Xwgs(3,k) = prepareNormalizedAngleDiff(Xwgs(3,k), grey.X_real(3,k));
  Pwgs(:,k) = diag(Pwg);

  
  % % Display example for Grey
  if live_display
    figure(1); clf; hold on;
    displayPos(Xgb, 1, 'b');
    displayCov(Xgb, Pgb, 0.95, 'b');
    displayPos(Xgg, 1, '+g');
    displayCov(Xgg, Pgg, 0.95, 'g');
    displayPos(Xgw, 1, '+r');
    displayCov(Xgw, Pgw, 0.95, 'r');

    displayPos(blue.X_real(1:3,k), 1, 'ok');
    displayPos(grey.X_real(1:3,k), 1, 'ok');
    displayPos(white.X_real(1:3,k), 1, 'ok');
    hold off;
    axis equal;
    xlabel('x (m)'); ylabel('y (m)'); title('Three cars around a roundabout');
    xlim([-50, 50]); ylim([-50, 50]);

    pause(dt);
  end
end

%% Display the estimation errors

labels = ['x' 'y' 'h' 'v' 'w'];

figure(2); clf;
for k = 1:5
  subplot(5,3,(k-1)*3+1); hold on;
  plot(Xbbs(k,:) - blue.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pbbs(k,:)), 'r'); plot(-3 * sqrt(Pbbs(k,:)), 'r');
  ylabel(labels(k));

  subplot(5,3,(k-1)*3+2); hold on;
  plot(Xbgs(k,:) - grey.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pbgs(k,:)), 'r'); plot(-3 * sqrt(Pbgs(k,:)), 'r');
  ylabel(labels(k));

  subplot(5,3,(k-1)*3+3); hold on;
  plot(Xbws(k,:) - white.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pbws(k,:)), 'r'); plot(-3 * sqrt(Pbws(k,:)), 'r');
  ylabel(labels(k));
end
axes('visible', 'off', 'title', 'Errors of Blue, Grey and White by Blue');

figure(3); clf;
for k = 1:5
  subplot(5,3,(k-1)*3+1); hold on;
  plot(Xgbs(k,:) - blue.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pgbs(k,:)), 'r'); plot(-3 * sqrt(Pgbs(k,:)), 'r');
  ylabel(labels(k));

  subplot(5,3,(k-1)*3+2); hold on;
  plot(Xggs(k,:) - grey.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pggs(k,:)), 'r'); plot(-3 * sqrt(Pggs(k,:)), 'r');
  ylabel(labels(k));

  subplot(5,3,(k-1)*3+3); hold on;
  plot(Xgws(k,:) - white.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pgws(k,:)), 'r'); plot(-3 * sqrt(Pgws(k,:)), 'r');
  ylabel(labels(k));
end
axes('visible', 'off', 'title', 'Errors of Blue, Grey and White by Grey');

figure(4); clf;
for k = 1:5
  subplot(5,3,(k-1)*3+1); hold on;
  plot(Xwbs(k,:) - blue.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pwbs(k,:)), 'r'); plot(-3 * sqrt(Pwbs(k,:)), 'r');
  ylabel(labels(k));

  subplot(5,3,(k-1)*3+2); hold on;
  plot(Xwgs(k,:) - grey.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pwgs(k,:)), 'r'); plot(-3 * sqrt(Pwgs(k,:)), 'r');
  ylabel(labels(k));

  subplot(5,3,(k-1)*3+3); hold on;
  plot(Xwws(k,:) - white.X_real(k,k_start:k_end), 'b');
  plot(3 * sqrt(Pwws(k,:)), 'r'); plot(-3 * sqrt(Pwws(k,:)), 'r');
  ylabel(labels(k));
end
axes('visible', 'off', 'title', 'Errors of Blue, Grey and White by White');

hold off;

%% Some functions (Evolution, Observation, Frames Transformation)
% Define evolution and observation models

function [X, P] = f_coordinated_turn(X, dt) 
% Function evaluated on X, dt to evolve X
    X(1) = X(1) + dt*X(4)*cos(X(3));
    X(2) = X(2) + dt*X(4)*sin(X(3));
    X(3) = X(3) + dt*X(5);
    X(4) = X(4);
    X(5) = X(5);
  
end

function [J] = fJ_coordinated_turn(X, dt)
% Function evaluated on X, dt to provide the evolution model matrix
  J = [ 1 0 -dt*X(4)*sin(X(3)) dt*cos(X(3)) 0;
        0 1 dt*X(4)*cos(X(3)) dt*sin(X(3)) 0;
        0 0 1 0 dt ;
        0 0 0 1 0 ;
        0 0 0 0 1 ];
end

function [Yhat] = g_gnss(X, Y)
% Function evaluated on X, Y to provide a "theoretical" measurement
  Yhat = eye(5)*X;
end

function [C] = gC_gnss(X, Y)
% Function evaluated on X, Y to provide the observation model matrix
      C = eye(5); % Observation matrix
end

function [Yhat, R] = g_lidar(X, Y)
% Observation Model (by Lidar)
  Yhat = [1 0 0 0 0;
          0 1 0 0 0]*X;
end

function [C] = gC_lidar(X, Y)
% Observation matrix by (Lidar)
       C = [1 0 0 0 0;
            0 1 0 0 0]; % Observation matrix
end

function [Ytf, Rtf] = transform_lidar(X, P, Y, R)
% Frame Transformation of the relative position into R0 (using oplus function).
  [Ytf, Rtf] = oplus(X(1:3,1),Y,P(1:3,1:3),R);
end
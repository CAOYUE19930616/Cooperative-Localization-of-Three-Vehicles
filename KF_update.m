function [X, P] = KF_update(X, P, Y, R, g, gC)
  % X: Filtered state
  % P: Covariance associated to X
  % Y: Observation vector
  % R: Covariance associated to Y
  % g: Function evaluated on X, Y to provide a "theoretical" measurement
  % gC: Function evaluated on X, Y to provide the observation model matrix

  C = gC(X, Y); % Observation Model Matrix
  Yhat = g(X, Y);

  K = P*C'*inv(C*P*C' + R);
  X = X + K*(Y-Yhat);
  P = (eye(5)-K*C)*P*(eye(5)-K*C)' + K*R*K';
  
end

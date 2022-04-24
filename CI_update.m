function [X, P] = CI_update(X, P, Y, R, g, gC)
  % X: Filtered state
  % P: Covariance associated to X
  % Y: Observation vector
  % R: Covariance associated to Y
  % g: Function evaluated on X, Y to provide a "theoretical" measurement
  % gC: Function evaluated on X, Y to provide the observation model matrix

  Pinv = inv(P);
  Rinv = inv(R);
  
  C = gC(X, Y);
  Yhat = g(X, Y);

  f = inline("1/det(w*Pinv + (1-w)*C'*Rinv*C)", 'w', 'Pinv', 'Rinv', 'C');
  w = fminsearch(f, 0.5, optimset('Display','off'), Pinv, Rinv, C);
  w = max(0, min(1, w));

  Pi = w * Pinv + (1-w)*Rinv ; 
  P = inv(Pi) ;
  K = P * inv(P + R) ;
  X = P*(w*Pinv*X + (1-w)*C'*Rinv*Y) ;
end


function [X, P] = KF_predict(X, P, dt, Q, f, fJ)
  % X: Filtered state
  % P: Covariance associated to X
  % dt: Time ellapsed since last prediction
  % f: Function evaluated on X, dt to evolve X
  % fg: Function evaluated on X, dt to provide the evolution model matrix

  J = fJ(X, dt) ; % Jacobian of f
  
  X = f(X, dt) ; 
  
  P = J * P * J' + Q ;

end

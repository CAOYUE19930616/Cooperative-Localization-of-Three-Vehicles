function [pos_res, cov_res] = oplus(pos_a, pos_b, cov_a, cov_b, cov_a_b, cov_b_a)
  % if cov_a cov_b, cov_a_b and cov_b_a are specified
  if(nargin > 4)
    cov = [cov_a, cov_a_b; cov_b_a, cov_b];
  % if cov_a and cov_b are specified
  elseif(nargin > 2)
    cov = [cov_a, zeros(size(pos_a,1),size(pos_b,1)); zeros(size(pos_b,1),size(pos_a,1)), cov_b];
  else
    cov = nan;
  end

  % if pos_b is a "position"
  if(size(pos_b, 1) < 3)
    R = [ cos(pos_a(3))  -sin(pos_a(3)) ;
          sin(pos_a(3))   cos(pos_a(3)) ; ];
    
    position_a = [ pos_a(1);
                   pos_a(2); ];
    
    pos_res = position_a + R * pos_b;
    
    % Jacobian Matrix
    J = [ 1   0   -(pos_res(2)-pos_a(2))  cos(pos_a(3))   -sin(pos_a(3))  ;
          0   1   (pos_res(1)-pos_a(1))  sin(pos_a(3))    cos(pos_a(3))  ; ];
      

    cov_res = J * cov * J' ;

  
% if pos_b is a "pose"
  else
    R = [ cos(pos_a(3)) -sin(pos_a(3)) 0 ;
          sin(pos_a(3)) cos(pos_a(3))  0 ;
                0            0         1 ;];
    
    pos_res = pos_a + R * pos_b ;

    % Jacobian Matrix
    J = [ 1   0   -(pos_res(2)-pos_a(2))  cos(pos_a(3))   -sin(pos_a(3))   0 ;
          0   1   -(pos_res(1)-pos_a(1))  sin(pos_a(3))    cos(pos_a(3))   0 ;
          0   0              1                 0                 0         1 ; ];              
  
    cov_res = J * cov * J' ;
 end

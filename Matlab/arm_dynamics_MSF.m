function dX = arm_dynamics_MSF(t, X, params)
% simulating the arm dynamic (modal form). The model only uses 2 of the
% joint variables (PMA 2 and PMA 3). Please refer the [1], and [2] for more
% informations.
% 
% Inputs:
%   t : time
%   X : length and length velocity [4x1]
%       [l2, l3, dl2, dl3]
%   params : parameters for matrices (structure)

    

    l = zeros(3,1); % l has to be (3x1) because in the function it is a vector
    l(2:3) = X(1:2);

    dp = zeros(2,1); % this is (2x1). because we don't use it in functions
    dp(1:2) = X(3:4);

    m = params.m; 
    L = params.L; 
    r = params.r; 
    dl2 = dp(1); 
    dl3 = dp(2); 
    k = params.k; 
    g = params.g; 
    d = params.d; 
    tau = params.tau;

    % Dynamics
    M = M_jointspace_MSF(l, m, L, r);
    C = C_jointspace_MSF(l, m, L, r, dl2, dl3);
    D = d*[dl2; dl3];
    G = G_and_K_jointspace_MSF(l, m, L, r, k, g);

    ddl = M \ (tau - C*dp - D - G);

    if any(isnan(ddl), 'all') || any(isinf(ddl), 'all')
        ddl = [ 0.0; 0.0];
    end

    dX = [dp; ddl];


end

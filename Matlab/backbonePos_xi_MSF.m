function P = backbonePos_xi_MSF(l, L, r, xi)
% The backbone point at xi
% Inputs:
%   l  : lenght change [1x3]
%   L  : neutral length of the backbone [3x1]
%   r  : radial offset of the PMA from the centerline [constant]
%   xi : selection parameter on the backbone [constant {0-1}]
% 
% Output:
%   P  : position vector [3x1]

arguments
    l (1,3) double 
    L (1,3) double {mustBeGreaterThanOrEqual(L, 0)} = [0, 0.278, 0]
    r (1,1) double {mustBePositive} = 0.013
    xi (1,1) double {mustBeBetween(xi,0,1)} = 1
end

P = [(l(2) + l(3)) * (0.2e1 / 0.1148175e7 * xi ^ 8 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 4 - r ^ 2 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 3 * xi ^ 6 / 0.8505e4 + 0.2e1 / 0.405e3 * r ^ 4 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 2 * xi ^ 4 - r ^ 6 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) * xi ^ 2 / 0.9e1 + r ^ 8) * L(2) * xi ^ 2 / r ^ 9 / 0.2e1 -(0.2e1 / 0.1148175e7 * xi ^ 8 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 4 - r ^ 2 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 3 * xi ^ 6 / 0.8505e4 + 0.2e1 / 0.405e3 * r ^ 4 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 2 * xi ^ 4 - r ^ 6 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) * xi ^ 2 / 0.9e1 + r ^ 8) * L(2) * sqrt(0.3e1) * (-l(3) + l(2)) * xi ^ 2 / r ^ 9 / 0.6e1 (0.2e1 / 0.229635e6 * xi ^ 8 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 4 - 0.4e1 / 0.8505e4 * r ^ 2 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 3 * xi ^ 6 + 0.2e1 / 0.135e3 * r ^ 4 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) ^ 2 * xi ^ 4 - 0.2e1 / 0.9e1 * r ^ 6 * (l(2) ^ 2 + l(3) * l(2) + l(3) ^ 2) * xi ^ 2 + r ^ 8) * L(2) * xi / r ^ 8];

P = reshape(P, [3,1]);

end

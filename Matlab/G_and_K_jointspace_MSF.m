function G = G_and_K_jointspace_MSF(l, m, L, r, k, g)
% Generalized force matrix and stiffness matrix in jointspace (modal form)
% 
% Inputs:
%   l  : lenght change [1x3]
%   m  : mass of the module [constant]
%   L  : neutral length of the backbone [1x3]
%   r  : radial offset of the PMA from the centerline [constant]
%   k  : stiffness matrix [1x3]
% 
% Output:
%   G  : Gravity matrix [2x1]

arguments (Input)
    l (1,3) double 
    m (1,1) double {mustBePositive} = 0.1
    L (1,3) double {mustBeGreaterThanOrEqual(L, 0)} = [0, 0.278, 0]
    r (1,1) double {mustBePositive} = 0.013
    k (1,3) double {mustBeGreaterThanOrEqual(k, 0)} = [0, 2.2e3,2.2e3]
    g (1,1) double {mustBePositive} = 9.81
end

arguments(Output)
    G (2,1) double
end

G = [((0.6967578984e-5 * l(2) ^ 7 + 0.2438652644e-4 * l(3) * l(2) ^ 6 + (-0.3527336861e-3 * r ^ 2 + 0.5225684238e-4 * l(3) ^ 2) * l(2) ^ 5 + (-0.8818342152e-3 * r ^ 2 * l(3) + 0.6967578984e-4 * l(3) ^ 3) * l(2) ^ 4 + (0.987654321e-2 * r ^ 4 - 0.1410934744e-2 * r ^ 2 * l(3) ^ 2 + 0.6619200035e-4 * l(3) ^ 4) * l(2) ^ 3 + (0.1481481481e-1 * r ^ 4 * l(3) - 0.1234567901e-2 * r ^ 2 * l(3) ^ 3 + 0.418054739e-4 * l(3) ^ 5) * l(2) ^ 2 + (-0.1111111111e0 * r ^ 6 + 0.1481481481e-1 * r ^ 4 * l(3) ^ 2 - 0.7054673721e-3 * r ^ 2 * l(3) ^ 4 + 0.1741894746e-4 * l(3) ^ 6) * l(2) - 0.5555555556e-1 * l(3) * r ^ 6 + 0.4938271605e-2 * l(3) ^ 3 * r ^ 4 - 0.176366843e-3 * l(3) ^ 5 * r ^ 2 + 0.3483789492e-5 * l(3) ^ 7) * m * g * L(2) + l(2) * k(2) * r ^ 8) / r ^ 8; ((0.6967578984e-5 * l(3) ^ 7 + 0.2438652644e-4 * l(2) * l(3) ^ 6 + (-0.3527336861e-3 * r ^ 2 + 0.5225684238e-4 * l(2) ^ 2) * l(3) ^ 5 + (-0.8818342152e-3 * r ^ 2 * l(2) + 0.6967578984e-4 * l(2) ^ 3) * l(3) ^ 4 + (0.987654321e-2 * r ^ 4 - 0.1410934744e-2 * r ^ 2 * l(2) ^ 2 + 0.6619200035e-4 * l(2) ^ 4) * l(3) ^ 3 + (0.1481481481e-1 * r ^ 4 * l(2) - 0.1234567901e-2 * r ^ 2 * l(2) ^ 3 + 0.418054739e-4 * l(2) ^ 5) * l(3) ^ 2 + (-0.1111111111e0 * r ^ 6 + 0.1481481481e-1 * r ^ 4 * l(2) ^ 2 - 0.7054673721e-3 * r ^ 2 * l(2) ^ 4 + 0.1741894746e-4 * l(2) ^ 6) * l(3) - 0.5555555556e-1 * r ^ 6 * l(2) + 0.4938271605e-2 * r ^ 4 * l(2) ^ 3 - 0.176366843e-3 * r ^ 2 * l(2) ^ 5 + 0.3483789492e-5 * l(2) ^ 7) * m * g * L(2) + l(3) * k(3) * r ^ 8) / r ^ 8;];

G = reshape(G, [2,1]);

end
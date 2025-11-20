% This is the MATLAB script that run the dynamic simulation of a soft
% continuum arm. The continuum arm is hybrid design which contains both
% soft and rigid elements to achieve deformation and the structural
% rigidity. Please refer following research articles for more informations.
% 
%  Research Articles:
%   [1]   Arachchige, Dimuthu DK, Yue Chen, Ian D. Walker, and Isuru S. Godage. 
%       "A novel variable stiffness soft robotic gripper." In 2021 IEEE 17th 
%       International Conference on Automation Science and Engineering (CASE), 
%       pp. 2222-2227. IEEE, 2021.
%   [2]   Arachchige, Dimuthu DK, and Isuru S. Godage. "Hybrid soft robots 
%       incorporating soft and stiff elements." In 2022 IEEE 5th international 
%       conference on soft robotics (RoboSoft), pp. 267-272. IEEE, 2022.
% 
% The dynamic model is derived as presented in the following article
%   [3]   Godage, Isuru S., David T. Branson, Emanuele Guglielmino, 
%       Gustavo A. Medrano-Cerda, and Darwin G. Caldwell. "Dynamics for biomimetic 
%       continuum arms: A modal approach." In 2011 IEEE International Conference on 
%       Robotics and Biomimetics, pp. 104-109. IEEE, 2011.
% 
% The dynamic model is derived for joint-space variables and the input to
% the system is pressure. The special case of this in-extensible soft
% contiuum arm is that, out of 3 actuator degrees, only 2 are availble for
% kinematic degress (workspace of the arm is spehical surface). The
% remaining degree is for the stiffness control.
% 
% The equation of motion is in the form of:
%       M(l)*l_ddot + C(l, l_dot)*l_dot + G(l) = P*A
%           
%   l       - joint variables (lenght of each actuator) (m) [2x1]
%   l_dot   - joint speed (m/s) [2x1]
%   l_ddot  - joint acceleration (m/s^2) [2x1]
%   P       - Pressures (bar) [2x1]
%   A       - Area of the actuator (m^2) [constant]
% 
% NOTE: The modal form is used to remove the singularities [3].
% 
% The parameters of the continuum arm is defined with ISO units. The most
% of the parameters are for pneumatic muscle actuators (PMAs). The other
% paramerters are for the in-extensible backbone




clear   % clearing the workspace for 

params.m = 0.1;     % mass of the module in kg
params.L = [0, 0.278, 0];   % module backbone length in m
params.A = pi*(0.013/2)^2;  % area of the pneumatic fitting in m^2
params.r = 0.013;   % radial offset from the backbone in m
params.k = [0, 2.2e3, 2.2e3];   % Stiffness of each PMAs
params.g = 9.81;    % the gravity acceleration
params.d = diag([30 30]);   % damping coeeficient of the PMAs
params.tau = [params.A*0e5; params.A*0e5];  % input force

% The 'xi' define the point on the backbone. The value range [0, 1] and
% 0-being the base of the continuum arm and the 1-being the tip of the arm.
% This parameter is used to draw the continuum arm backbone.
xi = linspace(0,1,20);

% Initial state
l0  = [0.0001; -0.02];      % initial joint coords
dl0 = [0.0; 0.0];           % initial joint velocities
X0  = [l0; dl0];            % concatinating the initial conditions

% Integrate
tspan = [0 10];

% Stiffer ODE solver is used as the matrices are tend to go ill-condition
[t, X] = ode15s(@(t,X) arm_dynamics_MSF(t,X,params), tspan, X0);


% drawing the arm's backbone
tip_pos = zeros(length(t), length(xi), 3);
l = zeros(1,3);

figure(1); clf
ax = axes;
h_backbone = plot3(NaN,NaN,NaN,'LineWidth',2); hold on
h_tip      = plot3(NaN,NaN,NaN,'o','MarkerFaceColor',[.8 .2 .2],'MarkerEdgeColor','none');
xlabel('X'); ylabel('Y'); zlabel('Z');
rotate3d 'on'; 
ht = title(ax, 'Continuum arm backbone (current frame) and tip');
grid on; axis equal

for k = 1:length(t)
    l(2:3) = X(k,1:2)';  % update joint variables from ODE solutions

    % fill positions along the backbone for all xi
    for j = 1:length(xi)
        % backbonePos_xi_MSF output the cartisian position of the backbone
        % at the point 'xi'
        pos = backbonePos_xi_MSF(l, params.L, params.r, xi(j));
        tip_pos(k,j,1) = pos(1);
        tip_pos(k,j,2) = pos(2);
        tip_pos(k,j,3) = pos(3);
    end

    % extract this frame's curve
    Xs = squeeze(tip_pos(k,:,1));
    Ys = squeeze(tip_pos(k,:,2));
    Zs = squeeze(tip_pos(k,:,3));

    % update plot
    set(h_backbone,'XData',Xs,'YData',Ys,'ZData',Zs);
    set(h_tip,'XData',Xs(end),'YData',Ys(end),'ZData',Zs(end));  % tip = xi(end)
    xlim([-0.3 0.3]); ylim([-0.3 0.3]); zlim([-0.1 0.3]); 
    set(ht, 'String', sprintf('Continuum arm — frame k = %d / %d   t = %.3f s', ...
                          k, numel(t), t(k)));
    drawnow
    pause(0.001);
end


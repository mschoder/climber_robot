function derive_everything() 
name = 'climber';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t y dy ddy th1 dth1 ddth1 th2 dth2 ddth2 tau1 tau2...
    Fy_foot Fx_foot Fy_hand Fx_hand ...
    g stance_width rope_diam c_fric_hand c_fric_foot delta ...        
    L_AB L_BC L_CD L_DE L_CE L_EG L_DF L_FG L_FH L_GH ...    
    c_AB c_BC c_CE c_EG c_DF c_FH ...                            
    m_AB m_BC m_CE m_EG m_DF m_FH ...                            
    m_tibloc m_motor ...                                             
    I_AB I_BC I_CE I_EG I_DF I_FH I_motor I_rm1 I_rm2 Nm1 Nm2 real
syms phi_sym real  % temporary angle var until we can solve numerically (see diagram)

% Group them for later use.
q   = [y; th1; th2];        % generalized coordinates
dq  = [dy; dth1; dth2];     % first time derivatives
ddq = [ddy; ddth1; ddth2];  % second time derivatives
u   = [tau1, tau2];         % control forces and moments
Fc  = [Fx_foot, Fy_foot, Fx_hand, Fy_hand]; % constraint forces and moments

% Parameters
p = [g; stance_width; rope_diam; c_fric_hand; c_fric_foot; delta; ...  % 
    L_AB; L_BC; L_CD; L_DE; L_CE; L_EG; L_DF; L_FG; L_FH; L_GH; ...    % 
    c_AB; c_BC; c_CE; c_EG; c_DF; c_FH; ...                            % 
    m_AB; m_BC; m_CE; m_EG; m_DF; m_FH; ...                            % 
    m_tibloc; m_motor; ...                                             % 
    I_AB; I_BC; I_CE; I_EG; I_DF; I_FH; I_motor; ...                   %
    I_rm1; I_rm2; Nm1; Nm2];                                           % 

%%% Calculate important vectors and their time derivatives.

% Define fundamental unit vectors.  The first element should be the
% horizontal (+x cartesian) component, the second should be the vertical (+y
% cartesian) component, and the third should be right-handed orthogonal.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

% Define other unit vectors for use in defining other vectors.
e1hat = @(th) sin(th)*ihat + cos(th)*jhat;
e2hat = @(th) -sin(th)*ihat + cos(th)*jhat;

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
psi = pi - delta - phi_sym;
rH = y*jhat;
rF = rH + L_FH * e1hat(th1 + phi_sym);
rG = rH + L_GH * e1hat(th1 + phi_sym);
rD = rF + L_DF * e1hat(th1 + th2 + phi_sym);
rE = rG + L_EG * e1hat(th1 + th2 + phi_sym);
rC = rE + L_CE * e1hat(th1 + phi_sym);
rB = rC + L_BC * e1hat(phi_sym);
rA = rB + L_AB * e2hat(psi);

% Solve for psi -- quadrilateral with 3 known sides and 2 known angles
% take second solution after verifying -- TODO, might be a cleaner way
phi_solved = solve(rA(1) == 0, phi_sym, 'Real', true);  
phi = simplify(phi_solved(2));

% substitute in for all of the key vectors already defined
psi = pi - delta - phi;
rF = rH + L_FH * e1hat(th1 + phi);
rG = rH + L_GH * e1hat(th1 + phi);
rD = rF + L_DF * e1hat(th1 + th2 + phi);
rE = rG + L_EG * e1hat(th1 + th2 + phi);
rC = rE + L_CE * e1hat(th1 + phi);
rB = rC + L_BC * e1hat(phi);
rA = rB + L_AB * e2hat(psi);

% derivative of phi for convenient use
dphi = ddt(phi);

% Define COMs
rcmFH = rH + c_FH * e1hat(th1 + phi);
rcmDF = rF + c_DF * e1hat(th1 + th2 + phi);
rcmEG = rG + c_EG * e1hat(th1 + th2 + phi);
rcmCE = rE + c_CE * e1hat(th1 + phi);
rcmBC = rC + c_BC * e1hat(phi);
rcmAB = rB + c_AB * e2hat(psi);

% Take time derivatives of vectors for kinetic energy terms
drH = ddt(rH);
drF = ddt(rF);
drG = ddt(rG);
drD = ddt(rD);
drE = ddt(rE);
drC = ddt(rC);
drB = ddt(rB);
drA = ddt(rA);

drcmFH = ddt(rcmFH);
drcmDF = ddt(rcmDF);
drcmEG = ddt(rcmEG);
drcmCE = ddt(rcmCE);
drcmBC = ddt(rcmBC);
drcmAB = ddt(rcmAB);

keypoints = [rA rB rC rD rE rF rG rH];

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 

% Define kinetic energies. See Lecture 6 formula for kinetic energy
% of a rigid body. Negative bc defined via RHR
omegaFH = -dphi - dth1;
omegaDF = -dphi - dth1 - dth2;
omegaEG = -dphi - dth1 - dth2;
omegaCE = -dphi - dth1;
omegaBC = -dphi;
omegaAB = -dphi;

% Kinetic energy
T_FH = (1/2) * m_FH * dot(drcmFH, drcmFH) + (1/2) * I_FH * omegaFH^2;
T_DF = (1/2) * m_DF * dot(drcmDF, drcmDF) + (1/2) * I_DF * omegaDF^2;
T_EG = (1/2) * m_EG * dot(drcmEG, drcmEG) + (1/2) * I_EG * omegaEG^2;
T_CE = (1/2) * m_CE * dot(drcmCE, drcmCE) + (1/2) * I_CE * omegaCE^2;
T_BC = (1/2) * m_BC * dot(drcmBC, drcmBC) + (1/2) * I_BC * omegaBC^2;
T_AB = (1/2) * m_AB * dot(drcmAB, drcmAB) + (1/2) * I_AB * omegaAB^2;
T_m1 = (1/2) * m_motor * dot(drC, drC) + (1/2) * I_motor * omegaBC^2;
T_m2 = (1/2) * m_motor * dot(drE, drE) + (1/2) * I_motor * omegaCE^2;
T_r1 = (1/2) * I_rm1 * (dphi + Nm1*dth1)^2;
T_r2 = (1/2) * I_rm2 * (dphi + dth1 + Nm2*dth2)^2;
% TODO - check angle signs RHR above ??

% Define potential energies. See Lecture 6 formulas for gravitational 
% potential energy of rigid bodies and elastic potential energies of
% energy storage elements.
V_FH = m_FH * g * dot(rcmFH, -jhat);
V_DF = m_DF * g * dot(rcmDF, -jhat);
V_EG = m_EG * g * dot(rcmEG, -jhat);
V_CE = m_CE * g * dot(rcmCE, -jhat);
V_BC = m_BC * g * dot(rcmBC, -jhat);
V_AB = m_AB * g * dot(rcmAB, -jhat);
V_m1 = m_motor * g * dot(rC, -jhat);
V_m2 = m_motor * g * dot(rE, -jhat);


% Sum KE and PE terms
T = T_FH + T_DF + T_EG + T_CE + T_BC + T_AB + T_r1 + T_r2 + T_m1 + T_m2;
V = V_FH + V_DF + V_EG + V_CE + V_BC + V_AB + V_m1 + V_m2;

% Define contributions to generalized forces.  See Lecture 6 formulas for
% contributions to generalized forces.
QF = F2Q(Fy_foot*jhat,rH);              % y-dir contact force at foot
Qtau1 = M2Q(-tau1*khat, omegaBC*khat);  % motor 1 @ point C
Qtau2 = M2Q(-tau2*khat, omegaCE*khat);  % motor 2 @ point E

Qtau = Qtau1 + Qtau2;

% Sum kinetic energy terms, potential energy terms, and generalized force
% contributions.
Q = QF + Qtau;

% Calculate rCOM, the location of the center of mass
rCOM = (m_AB*rcmAB + m_BC*rcmBC + m_CE*rcmCE + m_EG*rcmEG + m_DF*rcmDF + m_FH*rcmFH ...
        + m_motor*rC + m_motor*rE)/...
        (m_AB + m_BC + m_CE + m_EG + m_DF + m_FH + 2*m_motor);


% Assemble C, the set of constraints
C  = y;  % When y = 0, the constraint is satisfied because foot is on the ground
dC = ddt(C);

%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

size(eom)

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;

%%

%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:3,1) = q;  
z(4:6,1) = dq;

%%

% Write functions to a separate folder because we don't usually have to see them
directory = './AutoDerived/';  % changed to run from parent dir
% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});

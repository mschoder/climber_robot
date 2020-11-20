function derive_everything_v3() 
name = 'climber';
%%% Kinematics to test out fixing theta 1

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t y dy ddy th1 dth1 ddth1 th2 dth2 ddth2 tau1 tau2...
    g stance_width rope_diam c_fric_hand c_fric_foot delta ...        
    L_AB L_BC L_CD L_DE L_CE L_EG L_DF L_FG L_FH L_GH ...    
    c_AB c_BC c_CE c_EG c_DF c_FH ...                            
    m_AB m_BC m_CE m_EG m_DF m_FH ...                            
    m_tibloc m_motor ...                                             
    I_AB I_BC I_CE I_EG I_DF I_FH I_motor I_rm1 I_rm2 Nm1 Nm2 real
syms gam dgam ddgam real

% Group them for later use.
q   = [y; th2; gam];         % generalized coordinates
dq  = [dy; dth2; dgam];      % first time derivatives
ddq = [ddy; ddth2; ddgam];   % second time derivatives
u   = [tau2];                % control forces and moments

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
rH = y*jhat;
rF = rH + L_FH * e1hat(-gam);
rG = rH + L_GH * e1hat(-gam);
rD = rF + L_DF * e1hat(-gam - th2);
rE = rG + L_EG * e1hat(-gam - th2);
rC = rE + L_CE * e1hat(-gam);
rB = rC + L_BC * e1hat(-gam);
rA = rB + L_AB * e2hat(pi - delta + gam);

% Solve for gam given fixed rA(x) = 0 constraint; for use externally
gam_solved = solve(rA(1) == 0, gam, 'Real', true);
gam_solved = simplify(gam_solved(2));   % second soln is correct (trial & error)
dgam_solved = simplify(ddt(gam_solved));

% Define COMs
rcmFH = rH + c_FH * e1hat(-gam);
rcmDF = rF + c_DF * e1hat(-gam - th2);
rcmEG = rG + c_EG * e1hat(-gam - th2);
rcmCE = rE + c_CE * e1hat(-gam);
rcmBC = rC + c_BC * e1hat(-gam);
rcmAB = rB + c_AB * e2hat(pi - delta + gam);

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
dkeypoints = [drA drB drC drD drE drF drG drH];

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
omegaFH = -dgam;
omegaDF = -(dgam + dth2);
omegaEG = -(dgam + dth2);
omegaCE = -dgam;
omegaBC = -dgam;
omegaAB = -dgam;

% Kinetic energy
T_FH = (1/2) * m_FH * dot(drcmFH, drcmFH) + (1/2) * I_FH * omegaFH^2;
T_DF = (1/2) * m_DF * dot(drcmDF, drcmDF) + (1/2) * I_DF * omegaDF^2;
T_EG = (1/2) * m_EG * dot(drcmEG, drcmEG) + (1/2) * I_EG * omegaEG^2;
T_CE = (1/2) * m_CE * dot(drcmCE, drcmCE) + (1/2) * I_CE * omegaCE^2;
T_BC = (1/2) * m_BC * dot(drcmBC, drcmBC) + (1/2) * I_BC * omegaBC^2;
T_AB = (1/2) * m_AB * dot(drcmAB, drcmAB) + (1/2) * I_AB * omegaAB^2;
T_m1 = (1/2) * m_motor * dot(drC, drC) + (1/2) * I_motor * omegaBC^2;
T_m2 = (1/2) * m_motor * dot(drE, drE) + (1/2) * I_motor * omegaCE^2;
% T_r1 = (1/2) * I_rm1 * ((dgam) + 0)^2;
% T_r2 = (1/2) * I_rm2 * (dgam + Nm2*dth2)^2;
T_r1 = 0;
T_r2 = 0;

%%

% Define potential energies. See Lecture 6 formulas for gravitational 
% potential energy of rigid bodies and elastic potential energies of
% energy storage elements.
V_FH = m_FH * g * dot(rcmFH, jhat);
V_DF = m_DF * g * dot(rcmDF, jhat);
V_EG = m_EG * g * dot(rcmEG, jhat);
V_CE = m_CE * g * dot(rcmCE, jhat);
V_BC = m_BC * g * dot(rcmBC, jhat);
V_AB = m_AB * g * dot(rcmAB, jhat);
V_m1 = m_motor * g * dot(rC, jhat);
V_m2 = m_motor * g * dot(rE, jhat);


% Sum KE and PE terms
T = simplify(T_FH + T_DF + T_EG + T_CE + T_BC + T_AB + T_r1 + T_r2 + T_m1 + T_m2);
V = simplify(V_FH + V_DF + V_EG + V_CE + V_BC + V_AB + V_m1 + V_m2);

% Define contributions to generalized forces.  See Lecture 6 formulas for
% contributions to generalized forces.
% QF1 = F2Q(Fy_foot*jhat,rH);              % y-dir contact force at foot
% QF2 = F2Q(Fx_hand*ihat,rA);              % x-dir contact force at hand

% Qtau1 = M2Q(tau1*khat, omegaBC*khat);  % motor 1 @ point C
Qtau2 = M2Q(tau2*khat, omegaBC*khat);  % motor 2 @ point E

Qtau = Qtau2;

% Sum kinetic energy terms, potential energy terms, and generalized force
% contributions.
% Q = QF1 + QF2 + Qtau;
Q = Qtau;

% Calculate rcm, the location of the center of mass
rcm = (m_AB*rcmAB + m_BC*rcmBC + m_CE*rcmCE + m_EG*rcmEG + m_DF*rcmDF + m_FH*rcmFH ...
        + m_motor*rC + m_motor*rE)/...
        (m_AB + m_BC + m_CE + m_EG + m_DF + m_FH + 2*m_motor);


% Assemble C, the set of constraints
C  = y;  % When y = 0, the constraint is satisfied because foot is on the ground
dC = ddt(C);

%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                          % total system energy
L = T-V;                                          % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

size(eom)

%%% Rearrange Equations of Motion. 
A = simplify(jacobian(eom,ddq));
b = simplify(A*ddq - eom);

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_Joint_Sp = A;
Grav_Joint_Sp = simplify(jacobian(V, q)');
Corr_Joint_Sp = simplify( eom + Q - Grav_Joint_Sp - A*ddq);

%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:3,1) = q;  
z(4:6,1) = dq;

%%

% Write functions to a separate folder because we don't usually have to see them
directory = './AutoDerived/';  % changed to run from parent dir

% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});
matlabFunction(dkeypoints,'file',[directory 'dkeypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm = simplify(ddt(rcm));             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});

% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% %%
% % Write a function to evaluate phi and dphi with the hand constraint x=0
% phi_sol = [phi_solved dphi_solved]';
% matlabFunction(phi_sol,'file',[directory 'phi_solved_' name],'vars',{z p});

% Write a function to evaluate gam and dgam with the hand constraint x=0
gam_sol = [gam_solved dgam_solved]';
matlabFunction(gam_sol,'file',[directory 'gam_solved_' name],'vars',{z p});

% Write a function to eval jacobian at the hand (point A)
jA = jacobian(rA, q);
matlabFunction(jA, 'file',[directory, 'jacobian_hand_', name],'vars',{z,p});

% Write a function to eval jacobian at the foot (point H)
jH = jacobian(rH, q);
matlabFunction(jH, 'file',[directory, 'jacobian_foot_', name],'vars',{z,p});

% Compute ddt( jH ) and write function
djH = reshape( ddt(jH(:)) , size(jH) );
matlabFunction(djH, 'file',[directory, 'jacobian_dot_foot_', name],'vars',{z,p});

% Write a function to eval jacobian at point C
jC = jacobian(rC, q);
matlabFunction(jC, 'file',[directory, 'jacobian_C_', name],'vars',{z,p});

% Functions for joint space conversion
matlabFunction(Grav_Joint_Sp, 'file',[directory, 'grav_', name],'vars',{z,p});
matlabFunction(Corr_Joint_Sp, 'file',[directory, 'corr_', name],'vars',{z,p});

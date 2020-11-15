function p = parameters() 

% World
g = 9.81;

% System
stance_width = 0.04;      % separation between legs in the z-plane 
rope_diam = 0.010;
c_fric_hand = 0.2;        % estimated friction coefficient (single side)
c_fric_foot = 0.2;        % estimated friction coefficient (single side)
delta = 2.3562;           % fixed shoulder angle, 3*pi/4
% delta = 0;

% Lengths
L_AB = 0.150;
L_BC = 0.083;
L_CD = 0.011;
L_DE = 0.031;
L_CE = 0.042;
L_EG = 0.096;
L_DF = 0.096;
L_FG = 0.031;
L_FH = 0.122;
L_GH = 0.091;


% COM - All COM measured from “bottom to top” of member as shown
% in diagram above. Ex c_AB is distance from point B to COM
c_AB = L_AB/2;
c_BC = L_BC/2;
c_CE = L_CE/2;
c_EG = L_EG/2;
c_DF = L_DF/2;
c_FH = L_FH/2;

% Masses
m_AB = .4*L_AB;
m_BC = .066;
m_CE = .4*L_CE;
m_EG = .4*L_EG;
m_DF = .4*L_DF;
m_FH = .4*L_FH;
m_tibloc = 0.035+0.010;  % 35g manuf spec; includes tibloc and anything separate from original leg
m_motor = 0.098;

% Moments of Inertia
I_AB = m_AB * L_AB^2/12;
I_BC = m_BC * L_BC^2/12;
I_CE = m_CE * L_CE^2/12;
I_EG = m_EG * L_EG^2/12;
I_DF = m_DF * L_DF^2/12;
I_FH = m_FH * L_FG^2/12;
I_motor = 0;      % motor moment of inertia
I_rm1 = 0;        % motor 1 rotor inertia
I_rm2 = 0;        % motor 2 rotor inertia
Nm1   = 1;        % motor 1 gear ratio
Nm2   = 1;        % motor 2 gear ratio

p = [g; stance_width; rope_diam; c_fric_hand; c_fric_foot; delta; ...  % 
    L_AB; L_BC; L_CD; L_DE; L_CE; L_EG; L_DF; L_FG; L_FH; L_GH; ...    % 
    c_AB; c_BC; c_CE; c_EG; c_DF; c_FH; ...                            % 
    m_AB; m_BC; m_CE; m_EG; m_DF; m_FH; ...                            % 
    m_tibloc; m_motor; ...                                             % 
    I_AB; I_BC; I_CE; I_EG; I_DF; I_FH; I_motor; ...                   %
    I_rm1; I_rm2; Nm1; Nm2];                                           %
end
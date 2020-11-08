function E = energy_climber(in1,in2)
%ENERGY_CLIMBER
%    E = ENERGY_CLIMBER(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    08-Nov-2020 06:12:03

I_AB = in2(31,:);
I_BC = in2(32,:);
I_CE = in2(33,:);
I_DF = in2(35,:);
I_EG = in2(34,:);
I_FH = in2(36,:);
I_motor = in2(37,:);
I_rm1 = in2(38,:);
I_rm2 = in2(39,:);
L_BC = in2(8,:);
L_CE = in2(11,:);
L_EG = in2(12,:);
L_FH = in2(15,:);
L_GH = in2(16,:);
Nm1 = in2(40,:);
Nm2 = in2(41,:);
c_AB = in2(17,:);
c_BC = in2(18,:);
c_CE = in2(19,:);
c_DF = in2(21,:);
c_EG = in2(20,:);
c_FH = in2(22,:);
delta = in2(6,:);
dphi = in1(8,:);
dth1 = in1(6,:);
dth2 = in1(7,:);
dy = in1(5,:);
g = in2(1,:);
m_AB = in2(23,:);
m_BC = in2(24,:);
m_CE = in2(25,:);
m_DF = in2(27,:);
m_EG = in2(26,:);
m_FH = in2(28,:);
m_motor = in2(30,:);
phi = in1(4,:);
th1 = in1(2,:);
th2 = in1(3,:);
y = in1(1,:);
t2 = cos(phi);
t3 = sin(phi);
t4 = dphi+dth1;
t5 = delta+phi;
t6 = phi+th1;
t7 = dphi.^2;
t13 = -dy;
t8 = dth2+t4;
t9 = cos(t5);
t10 = cos(t6);
t11 = sin(t6);
t12 = t6+th2;
t16 = t4.^2;
t14 = cos(t12);
t15 = sin(t12);
t17 = L_CE.*t10;
t18 = L_FH.*t10;
t19 = L_GH.*t10;
t20 = L_CE.*t11;
t21 = L_FH.*t11;
t22 = L_GH.*t11;
t23 = c_CE.*t10;
t24 = c_CE.*t11;
t25 = t8.^2;
t26 = L_EG.*t14;
t27 = L_EG.*t15;
t28 = c_DF.*t14;
t29 = c_EG.*t14;
t30 = c_DF.*t15;
t31 = c_EG.*t15;
t32 = dth2.*t26;
t33 = dth2.*t27;
t34 = t19+t26;
t35 = t18+t28;
t36 = t19+t29;
t37 = t22+t27;
t38 = t21+t30;
t39 = t22+t31;
t40 = t24+t37;
t41 = t17+t34;
t42 = t23+t34;
t43 = t20+t37;
t44 = dth1.*t41;
t45 = dth1.*t43;
E = (I_AB.*t7)./2.0+(I_BC.*t7)./2.0+(I_CE.*t16)./2.0+(I_DF.*t25)./2.0+(I_EG.*t25)./2.0+(I_FH.*t16)./2.0+(I_motor.*t7)./2.0+(I_motor.*t16)./2.0+(m_AB.*((t13+t33+t45+dphi.*(t43+L_BC.*t3-c_AB.*sin(t5))).^2+(t32+t44+dphi.*(t41+L_BC.*t2-c_AB.*t9)).^2))./2.0+(m_motor.*((t13+t33+t45+dphi.*t43).^2+(t32+t44+dphi.*t41).^2))./2.0+(m_DF.*((dphi.*t35+dth2.*t28+dth1.*t35).^2+(t13+dphi.*t38+dth2.*t30+dth1.*t38).^2))./2.0+(m_EG.*((dphi.*t36+dth2.*t29+dth1.*t36).^2+(t13+dphi.*t39+dth2.*t31+dth1.*t39).^2))./2.0+(m_BC.*((t32+t44+dphi.*(t41+c_BC.*t2)).^2+(t13+t33+t45+dphi.*(t43+c_BC.*t3)).^2))./2.0+(m_FH.*((t13+c_FH.*dphi.*t11+c_FH.*dth1.*t11).^2+(c_FH.*dphi.*t10+c_FH.*dth1.*t10).^2))./2.0-g.*(m_AB.*t17+m_AB.*t19+m_AB.*t26+m_BC.*t17+m_BC.*t19+m_BC.*t26+m_CE.*t19+m_CE.*t23+m_CE.*t26+m_DF.*t18+m_DF.*t28+m_EG.*t19+m_EG.*t29+m_motor.*t17+m_motor.*t19.*2.0+m_motor.*t26.*2.0+m_AB.*y+m_BC.*y+m_CE.*y+m_DF.*y+m_EG.*y+m_FH.*y+m_motor.*y.*2.0+L_BC.*m_AB.*t2-c_AB.*m_AB.*t9+c_BC.*m_BC.*t2+c_FH.*m_FH.*t10)+(I_rm1.*(dphi+Nm1.*dth1).^2)./2.0+(m_CE.*((t13+t33+dphi.*t40+dth1.*t40).^2+(t32+dphi.*t42+dth1.*t42).^2))./2.0+(m_motor.*((t13+t33+dphi.*t37+dth1.*t37).^2+(t32+dphi.*t34+dth1.*t34).^2))./2.0+(I_rm2.*(t4+Nm2.*dth2).^2)./2.0;

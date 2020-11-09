function E = energy_climber(in1,in2)
%ENERGY_CLIMBER
%    E = ENERGY_CLIMBER(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    09-Nov-2020 09:58:40

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
dgam = in1(8,:);
dth1 = in1(6,:);
dth2 = in1(7,:);
dy = in1(5,:);
g = in2(1,:);
gam = in1(4,:);
m_AB = in2(23,:);
m_BC = in2(24,:);
m_CE = in2(25,:);
m_DF = in2(27,:);
m_EG = in2(26,:);
m_FH = in2(28,:);
m_motor = in2(30,:);
th1 = in1(2,:);
th2 = in1(3,:);
y = in1(1,:);
t2 = cos(gam);
t3 = sin(gam);
t4 = dgam+dth2;
t5 = gam+th2;
t6 = dgam.^2;
t13 = -dth1;
t14 = -dy;
t15 = -th1;
t7 = L_CE.*t2;
t8 = L_GH.*t2;
t9 = L_CE.*t3;
t10 = L_GH.*t3;
t11 = cos(t5);
t12 = sin(t5);
t16 = t4.^2;
t19 = dgam+t13;
t20 = gam+t15;
t17 = L_EG.*t11;
t18 = L_EG.*t12;
t21 = sin(t20);
t24 = cos(t20);
t25 = delta+t20;
t28 = t19.^2;
t22 = dth2.*t17;
t23 = dth2.*t18;
t26 = cos(t25);
t27 = sin(t25);
t29 = L_BC.*t24;
t30 = L_BC.*t21;
t31 = c_AB.*t26;
t32 = c_AB.*t27;
E = (m_motor.*((t14+t23+dgam.*(t9+t10+t18)).^2+(t22+dgam.*(t7+t8+t17)).^2))./2.0+(m_BC.*((dy-t23-dgam.*(t9+t10+t18+c_BC.*t21)+c_BC.*dth1.*t21).^2+(t22+dgam.*(t7+t8+t17+c_BC.*t24)+c_BC.*t13.*t24).^2))./2.0+(m_motor.*((t22+dgam.*(t8+t17)).^2+(t14+t23+dgam.*(t10+t18)).^2))./2.0+(I_AB.*t28)./2.0+(I_BC.*t28)./2.0+(I_CE.*t6)./2.0+(I_DF.*t16)./2.0+(I_EG.*t16)./2.0+(I_FH.*t6)./2.0+(I_motor.*t6)./2.0+(I_motor.*t28)./2.0+(m_DF.*((t14+dgam.*(L_FH.*t3+c_DF.*t12)+c_DF.*dth2.*t12).^2+(dgam.*(L_FH.*t2+c_DF.*t11)+c_DF.*dth2.*t11).^2))./2.0+(m_FH.*((dy-c_FH.*dgam.*t3).^2+c_FH.^2.*t2.^2.*t6))./2.0-g.*(m_AB.*t7+m_AB.*t8+m_AB.*t17+m_AB.*t29-m_AB.*t31+m_BC.*t7+m_BC.*t8+m_BC.*t17+m_CE.*t8+m_CE.*t17+m_EG.*t8+m_motor.*t7+m_motor.*t8.*2.0+m_motor.*t17.*2.0+m_AB.*y+m_BC.*y+m_CE.*y+m_DF.*y+m_EG.*y+m_FH.*y+m_motor.*y.*2.0+L_FH.*m_DF.*t2+c_BC.*m_BC.*t24+c_CE.*m_CE.*t2+c_DF.*m_DF.*t11+c_EG.*m_EG.*t11+c_FH.*m_FH.*t2)+(m_AB.*((t22-dth1.*(t29-t31)+dgam.*(t7+t8+t17+t29-t31)).^2+(t14+t23-dth1.*(t30-t32)+dgam.*(t9+t10+t18+t30-t32)).^2))./2.0+(m_CE.*((t22+dgam.*(t8+t17+c_CE.*t2)).^2+(t14+t23+dgam.*(t10+t18+c_CE.*t3)).^2))./2.0+(I_rm2.*(dgam+Nm2.*dth2).^2)./2.0+(I_rm1.*(t19+Nm1.*dth1).^2)./2.0+(m_EG.*((dgam.*(t8+c_EG.*t11)+c_EG.*dth2.*t11).^2+(t14+dgam.*(t10+c_EG.*t12)+c_EG.*dth2.*t12).^2))./2.0;

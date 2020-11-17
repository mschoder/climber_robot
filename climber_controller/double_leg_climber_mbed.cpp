#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#include <vector>

#define BEZIER_ORDER_FOOT    7
#define NUM_INPUTS (15 + 2*(BEZIER_ORDER_FOOT+1) + 1) // 32 total (last one is phase flag
#define NUM_OUTPUTS 37

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(12000); //initialize the motor shield with a period of 12000 ticks or ~20kHZ
Ticker currentLoop;

// Variables for q1 - R
float current1R;
float current_des1R = 0;
float prev_current_des1R = 0;
float current_int1R = 0;
float angle1R;
float angle_des1R;
float velocity1R;
float velocity_des1R;
float duty_cycle1R;
float angle1R_init;

// Variables for q2 - R
float current2R;
float current_des2R = 0;
float prev_current_des2R = 0;
float current_int2R = 0;
float angle2R;
float angle_des2R;
float velocity2R;
float velocity_des2R;
float duty_cycle2R;
float angle2R_init;

// Variables for q1 - L
float current1L;
float current_des1L = 0;
float prev_current_des1L = 0;
float current_int1L = 0;
float angle1L;
float angle_des1L;
float velocity1L;
float velocity_des1L;
float duty_cycle1L;
float angle1L_init;

// Variables for q2 - L
float current2L;
float current_des2L = 0;
float prev_current_des2L = 0;
float current_int2L = 0;
float angle2L;
float angle_des2L;
float velocity2L;
float velocity_des2L;
float duty_cycle2L;
float angle2L_init;

// Fixed kinematic parameters
const float l_OA=.011; 
const float l_OB=.042; 
const float l_AC=.096; 
const float l_DE=.091;

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = 1000.0f;  // 1kHz impedance control loop
float start_period, traj_period, end_period;
int num_cycles, cycle_ct, phase_flag;

// Control parameters
float current_Kp = 4.0f;         
float current_Ki = 0.4f;           
float current_int_max = 3.0f;       
float duty_max;      
float K_xx;
float K_yy;
float K_xy;
float D_xx;
float D_xy;
float D_yy;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

// Current control interrupt function
void CurrentLoop()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
    //~~~~~~~~~~~~~ MOTOR A - R1 ~~~~~~~~~~~~~~ 
    current1R = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1R = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1R = current_des1R - current1R;                                             // current errror
    current_int1R += err_c1R;                                                             // integrate error
    current_int1R = fmaxf( fminf(current_int1R, current_int_max), -current_int_max);      // anti-windup
    float ff1R = R*current_des1R + k_t*velocity1R;                                         // feedforward terms
    duty_cycle1R = (ff1R + current_Kp*err_c1R + current_Ki*current_int1R)/supply_voltage;   // PI current controller
    
    float absDuty1R = abs(duty_cycle1R);
    if (absDuty1R > duty_max) {
        duty_cycle1R *= duty_max / absDuty1R;
        absDuty1R = duty_max;
    }    
    if (duty_cycle1R < 0) { // backwards
        motorShield.motorAWrite(absDuty1R, 1);
    } else { // forwards
        motorShield.motorAWrite(absDuty1R, 0);
    }             
    prev_current_des1R = current_des1R; 
    //~~~~~~~~~~~~~ MOTOR B - R2 ~~~~~~~~~~~~~~ 
    current2R     = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity2R = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2R = current_des2R - current2R;                                             // current error
    current_int2R += err_c2R;                                                             // integrate error
    current_int2R = fmaxf( fminf(current_int2R, current_int_max), -current_int_max);      // anti-windup   
    float ff2R = R*current_des2R + k_t*velocity2R;                                         // feedforward terms
    duty_cycle2R = (ff2R + current_Kp*err_c2R + current_Ki*current_int2R)/supply_voltage;   // PI current controller
    
    float absDuty2R = abs(duty_cycle2R);
    if (absDuty2R > duty_max) {
        duty_cycle2R *= duty_max / absDuty2R;
        absDuty2R = duty_max;
    }    
    if (duty_cycle2R < 0) { // backwards
        motorShield.motorBWrite(absDuty2R, 1);
    } else { // forwards
        motorShield.motorBWrite(absDuty2R, 0);
    }             
    prev_current_des2R = current_des2R; 
    
    //~~~~~~~~~~~~~ MOTOR C - L1 ~~~~~~~~~~~~~~ 
    current1L = -(((float(motorShield.readCurrentC())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1L = encoderC.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1L = current_des1L - current1L;                                             // current errror
    current_int1L += err_c1L;                                                             // integrate error
    current_int1L = fmaxf( fminf(current_int1L, current_int_max), -current_int_max);      // anti-windup
    float ff1L = R*current_des1L + k_t*velocity1L;                                         // feedforward terms
    duty_cycle1L = (ff1L + current_Kp*err_c1L + current_Ki*current_int1L)/supply_voltage;   // PI current controller
    
    float absDuty1L = abs(duty_cycle1L);
    if (absDuty1L > duty_max) {
        duty_cycle1L *= duty_max / absDuty1L;
        absDuty1L = duty_max;
    }    
    if (duty_cycle1L < 0) { // backwards
        motorShield.motorCWrite(absDuty1L, 1);
    } else { // forwards
        motorShield.motorCWrite(absDuty1L, 0);
    }             
    prev_current_des1L = current_des1L; 
    //~~~~~~~~~~~~~ MOTOR D - L2 ~~~~~~~~~~~~~~ 
    current2L     = -(((float(motorShield.readCurrentD())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity2L = encoderD.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2L = current_des2L - current2L;                                             // current error
    current_int2L += err_c2L;                                                             // integrate error
    current_int2L = fmaxf( fminf(current_int2L, current_int_max), -current_int_max);      // anti-windup   
    float ff2L = R*current_des2L + k_t*velocity2L;                                         // feedforward terms
    duty_cycle2L = (ff2L + current_Kp*err_c2L + current_Ki*current_int2L)/supply_voltage;   // PI current controller
    
    float absDuty2L = abs(duty_cycle2L);
    if (absDuty2L > duty_max) {
        duty_cycle2L *= duty_max / absDuty2L;
        absDuty2L = duty_max;
    }    
    if (duty_cycle2L < 0) { // backwards
        motorShield.motorDWrite(absDuty2L, 1);
    } else { // forwards
        motorShield.motorDWrite(absDuty2L, 0);
    }             
    prev_current_des2L = current_des2L; 
}

int main (void)
{
    
    // Object for 7th order Cartesian foot trajectory
    BezierCurve rDesFoot_bezR(2,BEZIER_ORDER_FOOT);
    BezierCurve rDesFoot_bezL(2,BEZIER_ORDER_FOOT);
    
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
//    pc.printf("%f",input_params[0]);
    
    while(1) {
        
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)) {
            
                        
            // Get inputs from MATLAB          
            start_period                = input_params[0];    // First buffer time, before trajectory
            traj_period                 = input_params[1];    // Trajectory time/length
            end_period                  = input_params[2];    // Second buffer time, after trajectory
    
            angle1R_init                = input_params[3];    // Initial angle for q1 right (rad)
            angle2R_init                = input_params[4];    // Initial angle for q2 right (rad)
            angle1L_init                = input_params[5];    // Initial angle for q1 left  (rad)
            angle2L_init                = input_params[6];    // Initial angle for q2 left  (rad)

            K_xx                        = input_params[7];    // Foot stiffness N/m
            K_yy                        = input_params[8];    // Foot stiffness N/m
            K_xy                        = input_params[9];    // Foot stiffness N/m
            D_xx                        = input_params[10];    // Foot damping N/(m/s)
            D_yy                        = input_params[11];    // Foot damping N/(m/s)
            D_xy                        = input_params[12];   // Foot damping N/(m/s)
            duty_max                    = input_params[13];   // Maximum duty factor
            num_cycles                  = input_params[14];   // Number of cycles to run for 
            phase_flag                  = input_params[31];   // 0 if in phase, 1 if out of phase
            
            cycle_ct = 0;
          
            // Get foot trajectory points
            float foot_ptsR[2*(BEZIER_ORDER_FOOT+1)];
            float foot_ptsL[2*(BEZIER_ORDER_FOOT+1)];
            for(int i = 0; i<2*(BEZIER_ORDER_FOOT+1);i++) {  // Right foot
              foot_ptsR[i] = input_params[15+i];    
            }
            
            std::vector<float> bpts_y;
            for(int i = 0; i<2*(BEZIER_ORDER_FOOT+1);i++) {  // Left foot
              float bpt = input_params[15+i];
              if (i%2 == 0) {  // Flip x-value signs for the left side (even indices)
                bpt *= -1.0;
              } 
              else{
                bpts_y.push_back(bpt); // add y-value to vector
              } 
              foot_ptsL[i] = bpt;    
            }
            if (phase_flag) { // if out of phase, reverse order of odd-indexed (y) values
              for (int i=0; i < 2*(BEZIER_ORDER_FOOT+1); i++) {
                if (i%2 == 1) { 
                    foot_ptsL[i] = bpts_y.back();
                    bpts_y.pop_back();
                }
              }
            }
            
            // Create the bezier curves from control points
            rDesFoot_bezR.setPoints(foot_ptsR);
            rDesFoot_bezL.setPoints(foot_ptsL);
            
            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor C off
            motorShield.motorDWrite(0, 0); //turn motor D off
                         
            // Run experiment
            while( t.read() < start_period + num_cycles*traj_period + end_period) { 
            
                if (t - start_period - cycle_ct * traj_period > traj_period) {
                  cycle_ct++;
//                  pc.printf("Cycle Count %d \n\r",cycle_ct);
                }
                 
                // Read encoders to get motor states
                angle1R = encoderA.getPulses() * PULSE_TO_RAD + angle1R_init;       
                velocity1R = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle2R = encoderB.getPulses() * PULSE_TO_RAD + angle2R_init;       
                velocity2R = encoderB.getVelocity() * PULSE_TO_RAD; 
                
                angle1L = encoderC.getPulses() * PULSE_TO_RAD + angle1L_init;       
                velocity1L = encoderC.getVelocity() * PULSE_TO_RAD;
                 
                angle2L = encoderD.getPulses() * PULSE_TO_RAD + angle2L_init;       
                velocity2L = encoderD.getVelocity() * PULSE_TO_RAD;              
                
                const float th1R = angle1R;
                const float th2R = angle2R;
                const float dth1R= velocity1R;
                const float dth2R= velocity2R;
                const float th1L = angle1L;
                const float th2L = angle2L;
                const float dth1L= velocity1L;
                const float dth2L= velocity2L;
 
                // Calculate the Jacobian
                float Jx_th1R = l_AC*cos(th1R + th2R) + l_DE*cos(th1R) + l_OB*cos(th1R);
                float Jx_th2R = l_AC*cos(th1R + th2R);
                float Jy_th1R = l_AC*sin(th1R + th2R) + l_DE*sin(th1R) + l_OB*sin(th1R);
                float Jy_th2R = l_AC*sin(th1R + th2R);
                
                float Jx_th1L = l_AC*cos(th1L + th2L) + l_DE*cos(th1L) + l_OB*cos(th1L);
                float Jx_th2L = l_AC*cos(th1L + th2L);
                float Jy_th1L = l_AC*sin(th1L + th2L) + l_DE*sin(th1L) + l_OB*sin(th1L);
                float Jy_th2L = l_AC*sin(th1L + th2L);
                                
                                
                // Calculate the forward kinematics (position and velocity)
                float xFootR = l_AC*sin(th1R + th2R) + l_DE*sin(th1R) + l_OB*sin(th1R);
                float yFootR = -l_AC*cos(th1R + th2R) - l_DE*cos(th1R) - l_OB*cos(th1R);
                float dxFootR = dth1R*Jx_th1R + dth2R*Jx_th2R;
                float dyFootR = dth1R*Jy_th1R + dth2R*Jy_th2R;
                
                float xFootL = l_AC*sin(th1L + th2L) + l_DE*sin(th1L) + l_OB*sin(th1L);
                float yFootL = -l_AC*cos(th1L + th2L) - l_DE*cos(th1L) - l_OB*cos(th1L);
                float dxFootL = dth1L*Jx_th1L + dth2L*Jx_th2L;
                float dyFootL = dth1L*Jy_th1L + dth2R*Jy_th2L;

                // Set gains based on buffer and traj times, then calculate desired x,y from Bezier trajectory at current time if necessary
                float teff  = 0;
                float vMult = 0;
                if( t < start_period) {
                    if (K_xx > 0 || K_yy > 0) {
                        K_xx = 50; // for joint space control, set this to 1
                        K_yy = 50; // for joint space control, set this to 1
                        D_xx = 2;  // for joint space control, set this to 0.1
                        D_yy = 2;  // for joint space control, set this to 0.1
                        K_xy = 0;
                        D_xy = 0;
                    }
                    teff = 0;
                }
//                else if (t < start_period + traj_period)
                else if ( t < start_period + num_cycles * traj_period )
                {
                    K_xx = input_params[7];  // Foot stiffness N/m
                    K_yy = input_params[8];  // Foot stiffness N/m
                    K_xy = input_params[9];  // Foot stiffness N/m
                    D_xx = input_params[10];  // Foot damping N/(m/s)
                    D_yy = input_params[11];  // Foot damping N/(m/s)
                    D_xy = input_params[12]; // Foot damping N/(m/s)
//                    teff = (t-start_period);
                    teff = t - start_period - cycle_ct * traj_period;
                    vMult = 1;
//                    pc.printf("Teff: %f \n\r", teff);
                }
                else
                {
                    teff = traj_period;
                    vMult = 0;
                }
                
                // Create Bezier curves
                float rDesFootR[2] , vDesFootR[2];
                rDesFoot_bezR.evaluate(teff/traj_period,rDesFootR);
                rDesFoot_bezR.evaluateDerivative(teff/traj_period,vDesFootR);
                vDesFootR[0]/=traj_period;
                vDesFootR[1]/=traj_period;
                vDesFootR[0]*=vMult;
                vDesFootR[1]*=vMult;
                
                float rDesFootL[2] , vDesFootL[2];
                rDesFoot_bezL.evaluate(teff/traj_period,rDesFootL);
                rDesFoot_bezL.evaluateDerivative(teff/traj_period,vDesFootL);
                vDesFootL[0]/=traj_period;
                vDesFootL[1]/=traj_period;
                vDesFootL[0]*=vMult;
                vDesFootL[1]*=vMult;
                
                // Calculate the inverse kinematics (joint positions and velocities) for desired joint angles              
                float xFootdR = -rDesFootR[0];
                float yFootdR = rDesFootR[1];                
                float l_OER = sqrt( (pow(xFootdR,2) + pow(yFootdR,2)) );
                float alphaR = abs(acos( (pow(l_OER,2) - pow(l_AC,2) - pow((l_OB+l_DE),2))/(-2.0f*l_AC*(l_OB+l_DE)) ));
                float th2_desR = -(3.14159f - alphaR); 
                float th1_desR = -((3.14159f/2.0f) + atan2(yFootdR,xFootdR) - abs(asin( (l_AC/l_OER)*sin(alphaR) )));
                
                float xFootdL = -rDesFootL[0];
                float yFootdL = rDesFootL[1];                
                float l_OEL = sqrt( (pow(xFootdL,2) + pow(yFootdL,2)) );
                float alphaL = abs(acos( (pow(l_OEL,2) - pow(l_AC,2) - pow((l_OB+l_DE),2))/(-2.0f*l_AC*(l_OB+l_DE)) ));
                float th2_desL = -(3.14159f - alphaR); 
                float th1_desL = -((3.14159f/2.0f) + atan2(yFootdL,xFootdL) - abs(asin( (l_AC/l_OEL)*sin(alphaL) )));
                
                
                float ddR = (Jx_th1R*Jy_th2R - Jx_th2R*Jy_th1R);
                float dth1_desR = (1.0f/ddR) * (  Jy_th2R*vDesFootR[0] - Jx_th2R*vDesFootR[1] );
                float dth2_desR = (1.0f/ddR) * ( -Jy_th1R*vDesFootR[0] + Jx_th1R*vDesFootR[1] );
                
                float ddL = (Jx_th1L*Jy_th2L - Jx_th2L*Jy_th1L);
                float dth1_desL = (1.0f/ddL) * (  Jy_th2L*vDesFootL[0] - Jx_th2L*vDesFootL[1] );
                float dth2_desL = (1.0f/ddL) * ( -Jy_th1L*vDesFootL[0] + Jx_th1L*vDesFootL[1] );
        
                // Calculate error variables
                float e_xR = rDesFootR[0] - xFootR;
                float e_yR = yFootdR - yFootR;
                float de_xR = vDesFootR[0] - dxFootR;
                float de_yR = vDesFootR[1] - dyFootR;
                
                float e_xL = rDesFootL[0] - xFootL;
                float e_yL = yFootdL - yFootL;
                float de_xL = vDesFootL[0] - dxFootL;
                float de_yL = vDesFootL[1] - dyFootL;
        
                // Calculate virtual force on foot
                float fxR = K_xx*(rDesFootR[0] - xFootR) + K_xy*(yFootdR - yFootR) + D_xx*(vDesFootR[0] - dxFootR) + D_xy*(vDesFootR[1] - dyFootR);
                float fyR = K_xy*(rDesFootR[0] - xFootR) + K_yy*(yFootdR - yFootR) + D_xy*(vDesFootR[0] - dxFootR) + D_yy*(vDesFootR[1] - dyFootR);
                
                float fxL = K_xx*(rDesFootL[0] - xFootL) + K_xy*(yFootdL - yFootL) + D_xx*(vDesFootL[0] - dxFootL) + D_xy*(vDesFootL[1] - dyFootL);
                float fyL = K_xy*(rDesFootL[0] - xFootL) + K_yy*(yFootdL - yFootL) + D_xy*(vDesFootL[0] - dxFootL) + D_yy*(vDesFootL[1] - dyFootL);
                
                //Part 1
                //current_des1 = 0;     
                //current_des2 = 0;  
                
                //Part 2                
                // Set desired currents    
                // Joint-space impedance
                // sub Kxx for K1, Dxx for D1, Kyy for K2, Dyy for D2
                // Note: Be careful with signs now that you have non-zero desired angles!
                // Your equations should be of the form i_d = K1*(q1_d - q1) + D1*(dq1_d - dq1)         
                //current_des1 = (K_xx*(th1_des - angle1) + D_xx*(dth1_des - velocity1)) / k_t;      
                //current_des2 = (K_yy*(th2_des - angle2) + D_yy*(dth2_des - velocity2)) / k_t;
                
                //Part 3                         
                // Cartesian impedance  
                // Note: As with the joint space laws, be careful with signs! -- Take jacobian transpse x F! / k_t        
                current_des1R = (Jx_th1R*fxR + Jy_th1R*fyR) / k_t;          
                current_des2R = (Jx_th2R*fxR + Jy_th2R*fyR) / k_t;   
                
                current_des1L = (Jx_th1L*fxL + Jy_th1L*fyL) / k_t;          
                current_des2L = (Jx_th2L*fxL + Jy_th2L*fyL) / k_t;   
                
                
                // Form output to send to MATLAB     // TOOD - Add Left side outputs
                float output_data[NUM_OUTPUTS];
                // current time
                output_data[0] = t.read();
                // motor 1 state
                output_data[1] = angle1R;
                output_data[2] = velocity1R;  
                output_data[3] = current1R;
                output_data[4] = current_des1R;
                output_data[5] = duty_cycle1R;
                // motor 2 state
                output_data[6] = angle2R;
                output_data[7] = velocity2R;
                output_data[8] = current2R;
                output_data[9] = current_des2R;
                output_data[10]= duty_cycle2R;
                // foot state
                output_data[11] = xFootR;
                output_data[12] = yFootR;
                output_data[13] = dxFootR;
                output_data[14] = dyFootR;
                output_data[15] = rDesFootR[0];
                output_data[16] = rDesFootR[1];
                output_data[17] = vDesFootR[0];
                output_data[18] = vDesFootR[1];
                
                // LEFT side outputs
                // motor 1 state
                output_data[19] = angle1L;
                output_data[20] = velocity1L;  
                output_data[21] = current1L;
                output_data[22] = current_des1L;
                output_data[23] = duty_cycle1L;
                // motor 2 state
                output_data[24] = angle2L;
                output_data[25] = velocity2L;
                output_data[26] = current2L;
                output_data[27] = current_des2L;
                output_data[28]= duty_cycle2L;
                // foot state
                output_data[29] = xFootL;
                output_data[30] = yFootL;
                output_data[31] = dxFootL;
                output_data[32] = dyFootL;
                output_data[33] = rDesFootL[0];
                output_data[34] = rDesFootL[1];
                output_data[35] = vDesFootL[0];
                output_data[36] = vDesFootL[1];
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(impedance_control_period_us);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoop.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor C off
            motorShield.motorDWrite(0, 0); //turn motor D off
        
        } // end if
        
    } // end while
    
} // end main
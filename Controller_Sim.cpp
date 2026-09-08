#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>

using namespace std;




int main(){
    ofstream csv("controller_sim.csv");
    if (!csv.is_open()){
        cout << "ERROR: Could not open CSV file!" << endl;
        return 1;
    }

    csv << "time,target_N,target_E,target_alt,target_vN,target_vE,x0_phi,x1_theta,x2_psi,x3_p,x4_q,x5_r,x6_pN,x7_pE,x8_pD,x9_vN,x10_vE,x11_vD,x12_M1,x13_M2,x14_M3,x15_M4,vN_cmd,vE_cmd,vH_cmd\n";

    double pi = 3.1415926;
    int n = 16, k = 0; //x = [phi, theta, psi, p, q, r, px, py, pz, vx, vy, vz, m1, m2, m3, m4] ----- (m1, m2, m3, and m4 are motor throttle)
    float dt = .01, tfinal = 60;
    float Ixx = .0033, Iyy = .0033, Izz = .005, mass = .77, g = 9.81, t = 0, alt = 0, h_cruiz = 0;
    float Kt = 4, Kq = 2.35;
    float d_arm = .11;
    float tc = .028; //motor time contant 
    float hoverThrottle = mass*g/(4*Kt);
    float delta_x = 0, delta_y = 0, dist = 0, rel_speed = 0;
    float future_tgt_angle = 0;

    //command variables
    float h_cmd = 0, pN_cmd = 0, pE_cmd = 0, vN_cmd = 0, vE_cmd = 0, aN_cmd = 0, aE_cmd = 0, aD_cmd = 0, phi_cmd = 0, theta_cmd = 0, psi_cmd = 0, vD_cmd = 0;
    float M1_cmd = 0, M2_cmd = 0, M3_cmd = 0, M4_cmd = 0;
    float L_cmd = 0, M_cmd = 0, N_cmd = 0;


    //normalized moment commands
    float s1 = 0, sL = 0, sM = 0, sN = 0;

    //true state variables
    float L = 0, M = 0, N = 0, Thrust = 0;

   
    float p_dot = 0, q_dot = 0, r_dot = 0, aN_dot = 0, aE_dot = 0, aD_dot = 0, phi_dot = 0, theta_dot = 0, psi_dot = 0;

    //integrators, derivative, and proportional terms
    float ix = 0, iy = 0, iz = 0, ex = 0, ey = 0, ez = 0;
    float kp_h = 0, kd_h = 0, ki_h = 0;
    float kp_xy = 0, kd_xy = 0, ki_xy = 0;
    float kp_phi = 0, kd_phi = 0, ki_phi = 0;
    float kp_theta = 0, kd_theta = 0, ki_theta = 0;
    float kp_psi = 0, kd_psi = 0, ki_psi = 0;

    float e_phi = 0, e_theta = 0, e_psi = 0;

    bool near_landing, mid_landing, target_acquired = false, landing_started = false;


    float blend = 0.5; //blend factor for the landing controller, 0 is pure landing controller, 1 is pure tracking controller

    //misc variables
    float max_tilt_allowed = 20*pi/180, phi_base = 0, theta_base = 0;

    //Planar gain terms
    float Kd_xy = 0, Kp_xy = 0, Ki_xy = 0;

    //thrust and throttle variables
    float T_total = 0;
    float M1_dot = 0, M2_dot = 0, M3_dot = 0, M4_dot = 0;

    //target variables
    vector<float> target_center = {0, 0};
    float target_radius = 50.0, target_speed = 1, target_angle = 0, target_vx = 0, target_vy = 0, target_heading = 0, tau = .011; //in meters and meters/second
    //drone starts at a hover (but zero meters off the ground so its just weightless)
    vector<float> x = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, hoverThrottle, hoverThrottle, hoverThrottle, hoverThrottle};
    vector<float> target_xy = {target_radius, 0}; //target starts on the 0 degree point on the circle


    vector<float> quat = {1, 0, 0, 0};

    //main loop
    while (t < tfinal){


        //-------------------------computing target position and velocity-------------------------//
        target_angle += target_speed*dt/target_radius; //angle = speed*time/radius = w*t b/c we're keeping the radius and linear speed constant and v/r is angular speed
        target_vx = -target_speed*sin(target_angle);
        target_vy = target_speed*cos(target_angle);
        target_xy[0] = target_center[0] + target_radius*cosf(target_angle);
        target_xy[1] = target_center[1] + target_radius*sinf(target_angle);
        target_heading = atan2(target_vy, target_vx); //tan(psi) = vy/vx b/c vx = vcos(psi) and vy = vsin(psi) hence vy/vx = sin(psi)/cos(psi) = tan(psi)
        

        //getting distance from target
        delta_x = target_xy[0] - x[6];
        delta_y = target_xy[1] - x[7];

        dist = sqrt(delta_x*delta_x + delta_y*delta_y);
        rel_speed = sqrt((target_vx - x[9])*(target_vx - x[9]) + (target_vy - x[10])*(target_vy - x[10]));


        //-------------------------computing future target position and velocity-------------------------//
        //tau is basically like the target will be at the future angle tau seconds from now
        //the following set up basically says make tau large while far from target, that is look ahead a lot while far away
        //then as you get closer decrease how far ahead you are looking, and tau can even become negative to make the drone look backwards to correct for overshoot 
        if(target_speed > .3){
            tau = dist/target_speed; //tau propotional to distance and thus decreases as you get closer to the target
        }
        else{
            tau = dist/.3;
        }

        //tau phases
        alt = -x[8];
        if(dist < 5 && alt < 4){
            tau = tau - .5;
        }
        if(dist < 2 && alt < 2){
            tau = tau - .7;
        }

        if(tau > 3){
            tau = 3;
        }
        if(tau < -2){
            tau = -2;
        }

        if(dist < 4 && rel_speed < .5){
            target_acquired = true;
        }
        if(target_acquired){
            tau = 0;
        }

        future_tgt_angle = target_angle + target_speed/target_radius*tau; //predicting next angular position of target



        //-----------------------cruiz altitude commands-------------------------//
        h_cruiz = 8;
        if(t >= 20 && dist < .5 && rel_speed < .15){
            landing_started = true;
        }
        if(!landing_started){
            h_cmd = h_cruiz;
        }
        else if(h_cmd > 1.5){
            h_cmd = h_cmd - .4*dt;
        }
        else if(h_cmd > .3){
            h_cmd = h_cmd - .2*dt;
        }
        else if(h_cmd > 0){
            h_cmd = h_cmd - .1*dt;
            if(h_cmd < 0){
                h_cmd = 0;
            }
        }


        //-----------------------cruiz altitude commands-------------------------//
        pN_cmd = target_center[0] + target_radius*cos(future_tgt_angle);
        pE_cmd = target_center[1] + target_radius*sin(future_tgt_angle);

        //-----------------------computing planar velocity commands-------------------------//
        vN_cmd = -target_speed*sin(future_tgt_angle);
        vE_cmd = target_speed*cos(future_tgt_angle);

        //compute planar error terms
        ex = pN_cmd - x[6];
        ey = pE_cmd - x[7];

        //planar integrator terms
        ix += ex*dt;
        iy += ey*dt;
        

        if(ix > 8){
            ix = 8;
        }
        if(iy > 8){
            iy = 8;
        }
        if(ix < -8){
            ix = -8;
        }
        if(iy < -8){
            iy = -8;
        }

        //-----------------------landing conditions-------------------------//
        near_landing = alt < 1 && dist < 2;
        mid_landing = alt < 2.5 && dist < 4;

        if (near_landing){
            Kp_xy = 0.35;
            Kd_xy = 1.05;
            Ki_xy = 0.0;
        }
        else if (mid_landing){
            Kp_xy = 0.30;
            Kd_xy = 0.90;
            Ki_xy = 0.0;
        }
        else{
            Kp_xy = 0.20;
            Kd_xy = 0.60;
            Ki_xy = 0.005;
        }

        //-----------------------computing planar acceleration commands-------------------------//
        //X[9] and x[10] are vN and vE respectively
        aN_cmd = Kp_xy*ex + Kd_xy*(vN_cmd - x[9]) + Ki_xy*ix;
        aE_cmd = Kp_xy*ey + Kd_xy*(vE_cmd - x[10]) + Ki_xy*iy;

        //limit acceleration magnitude (keep under 3 m/s^2)
        if(aN_cmd > 3){
            aN_cmd = 3;
        }
        else if(aN_cmd < -3){
            aN_cmd = -3;
        }
        if(aE_cmd > 3){
            aE_cmd = 3;
        }
        else if(aE_cmd < -3){
            aE_cmd = -3;
        }



        //--------------------Setting angle commands based on acceleration commands--------------------//
        //assuming small angle approximation for theta and phi and yaw ~ 0 (so theta and phi should be below 20 degrees)
        theta_base = -aN_cmd / g;   // pitch forward -> north accel
        phi_base   =  aE_cmd / g;   // roll right   -> east  accel

        if(theta_base > max_tilt_allowed){
            theta_base = max_tilt_allowed;
        }
        else if(theta_base < -max_tilt_allowed){
            theta_base = -max_tilt_allowed;
        }
        if(phi_base > max_tilt_allowed){
            phi_base = max_tilt_allowed;
        }
        else if(phi_base < -max_tilt_allowed){
            phi_base = -max_tilt_allowed;
        }

        if(near_landing){

            blend = 1; //keep horizontal control while landing

            theta_cmd = blend*theta_base;
            phi_cmd = blend*phi_base;


        }
        else{
            theta_cmd = theta_base;
            phi_cmd = phi_base;
        }

        psi_cmd = 0; //keep yaw at 0 for now, but could be used to make the drone face the target or something else


        //-------------------setting altitude and vertical velocity commands-------------------//

        ez = h_cmd - alt;
        iz = max(min(iz + ez*dt, 3.0f), -3.0f); //limit integrator to prevent windup

        //--------setting PID coefficients for altitude control, these could be tuned further if needed--------//
        kp_h = 1;
        kd_h = 2;
        ki_h = 0; 

        //---------setting vertical velocity command conditions---------//
        //vD_cmd is positive upward to match altitude
        if(!landing_started || h_cmd <= 0){
            vD_cmd = 0;
        }
        else if (h_cmd > 1.5){
            vD_cmd = -.4;
        }
        else if(h_cmd > .3){
            vD_cmd = -.2;
        }
        else{
            vD_cmd = -.1;
        }

        //---------computing vertical acceleration command based on desired velocity---------//
        aD_cmd = kp_h*ez + kd_h*(vD_cmd + x[11]) + ki_h*iz;

        aD_cmd = max(min(aD_cmd, 2.0f), -3.0f); //limit vertical acceleration command to prevent excessive thrust

        //---------computing total thrust command based on vertical acceleration command---------//
        T_total = mass*(g + aD_cmd);
        T_total = max(min(T_total, 4*Kt), 0.0f); //limit total thrust command to prevent excessive thrust or negative thrust

        //angualr error terms
        e_phi = phi_cmd - x[0];
        e_theta = theta_cmd - x[1];
        e_psi = psi_cmd - x[2];

        kp_phi = 10; kd_phi = 3;
        kp_theta = 10; kd_theta = 3;
        kp_psi = 4; kd_psi = 1.5;


        L_cmd = Ixx*(kp_phi*e_phi + kd_phi*(0 - x[3]));
        M_cmd = Iyy*(kp_theta*e_theta + kd_theta*(0 - x[4]));
        N_cmd = Izz*(kp_psi*e_psi + kd_psi*(0 - x[5]));

    
        s1 = T_total / Kt;
        sL = L_cmd / (Kt*d_arm);
        sM = M_cmd / (Kt*d_arm);
        sN = N_cmd / Kq;

        M1_cmd = (s1 - sL + sM + sN) / 4;
        M2_cmd = (s1 - sL - sM - sN) / 4;
        M3_cmd = (s1 + sL - sM + sN) / 4;
        M4_cmd = (s1 + sL + sM - sN) / 4;


        //Clamps throttle between 0 and 1, which is the range of the normalized microseconds motor command input-------------//
        M1_cmd = max(min(M1_cmd, 1.0f), 0.0f); 
        M2_cmd = max(min(M2_cmd, 1.0f), 0.0f);
        M3_cmd = max(min(M3_cmd, 1.0f), 0.0f);
        M4_cmd = max(min(M4_cmd, 1.0f), 0.0f);



        //----------Model of aircraft motion------------------------------//
        


     


        Thrust = Kt*(x[12] + x[13] + x[14] + x[15]);
        L = Kt*d_arm*(x[14] + x[15] - x[12] - x[13]);
        M = Kt*d_arm*(x[12] + x[15] - x[13] - x[14]);
        N = Kq*(x[12] + x[14] - x[13] - x[15]);


        //computing derivatives to propagate the state
        M1_dot = (M1_cmd - x[12]) / tc;
        M2_dot = (M2_cmd - x[13]) / tc;
        M3_dot = (M3_cmd - x[14]) / tc;
        M4_dot = (M4_cmd - x[15]) / tc;

        p_dot = ((Iyy - Izz)*x[4]*x[5] + L)/Ixx;
        q_dot = ((Izz - Ixx)*x[3]*x[5] + M)/Iyy;
        r_dot = ((Ixx - Iyy)*x[3]*x[4] + N)/Izz;

        phi_dot = x[3] + sinf(x[0])*tanf(x[1])*x[4] + cosf(x[0])*tanf(x[1])*x[5];
        theta_dot = cosf(x[0])*x[4] - sinf(x[0])*x[5];
        psi_dot = sinf(x[0])/cosf(x[1])*x[4] + cosf(x[0])/cosf(x[1])*x[5];

        aN_dot = -Thrust/mass*(cosf(x[0])*sinf(x[1])*cosf(x[2]) + sinf(x[0])*sinf(x[2]));
        aE_dot = -Thrust/mass*(cosf(x[0])*sinf(x[1])*sinf(x[2]) - sinf(x[0])*cosf(x[2]));
        aD_dot = -Thrust/mass*(cosf(x[0])*cosf(x[1])) + g;


        //propagating the state
        x[0] += phi_dot*dt;
        x[1] += theta_dot*dt;
        x[2] += psi_dot*dt;
        x[3] += p_dot*dt;
        x[4] += q_dot*dt;
        x[5] += r_dot*dt;
        x[6] += x[9]*dt;
        x[7] += x[10]*dt;
        x[8] += x[11]*dt;
        x[9] += aN_dot*dt;
        x[10] += aE_dot*dt;
        x[11] += aD_dot*dt;
        x[12] += M1_dot*dt;
        x[13] += M2_dot*dt;
        x[14] += M3_dot*dt;
        x[15] += M4_dot*dt;

        //prevent the quadcopter from going below the ground
        if(x[8] > 0){
            x[8] = 0;
            if(x[11] > 0){
                x[11] = 0;
            }
        }


        k++;
        t = k*dt;

        csv << t << ',' << target_xy[0] << ',' << target_xy[1] << ',' << 0 << ',' << target_vx << ',' << target_vy;
        for (int element = 0; element < n; element++){
            csv << ',' << x[element];
        }
        csv << ',' << vN_cmd << ',' << vE_cmd << ',' << vD_cmd << "\n";
    }





    
    







    


    return 0;

}


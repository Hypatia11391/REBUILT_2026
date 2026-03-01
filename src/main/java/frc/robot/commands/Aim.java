package frc.robot.commands;

public final class Aim {
    double elevation = Math.PI/3;
    double g = 9.80665;
    double pos_r_x;
    double pos_r_y;
    double pos_r_z;
    double pos_t_x;
    double pos_t_y;
    double pos_t_z;
    double vel_r_x;
    double vel_r_y;
    double req_theta;
    double req_exit_v;

    public static void update_aim(Aim AimObj, int newtonsMethod) {
        double t = 5.0;
        double H = AimObj.pos_t_z - AimObj.pos_r_z;
        double cot_alpha = 1/Math.tan(AimObj.elevation);
        double r_h_x = AimObj.pos_t_x - AimObj.pos_r_x;
        double r_h_y = AimObj.pos_t_y - AimObj.pos_r_y;
        double r_h_z = AimObj.pos_t_z - AimObj.pos_r_z;

        for(int i = 0; i < newtonsMethod; i++) {
            double dist_eff_x = r_h_x - AimObj.vel_r_x * t;
            double dist_eff_y = r_h_y - AimObj.vel_r_y * t;
            double dist_eff_mag = Math.sqrt(Math.pow(dist_eff_x,2 ) + Math.pow(dist_eff_y, 2));
            double dist_eff_x_hat = dist_eff_x/dist_eff_mag;
            double dist_eff_y_hat = dist_eff_y/dist_eff_mag;

            double f = cot_alpha * (H + AimObj.g/2 * t * t) - dist_eff_mag;
            double f_prime = cot_alpha * AimObj.g * t + (AimObj.vel_r_x*dist_eff_x_hat + AimObj.vel_r_y*dist_eff_y_hat);

            t = t - f/f_prime;
        }

        double dist_eff_x = r_h_x - AimObj.vel_r_x * t;
        double dist_eff_y = r_h_y - AimObj.vel_r_y * t;
        double dist_eff_mag = Math.sqrt(Math.pow(dist_eff_x, 2) + Math.pow(dist_eff_y, 2));
        double dist_eff_x_hat = dist_eff_x/dist_eff_mag;
        double dist_eff_y_hat = dist_eff_y/dist_eff_mag;

        AimObj.req_theta = Math.atan2(dist_eff_y_hat, dist_eff_x_hat);
        AimObj.req_exit_v = (r_h_z + AimObj.g/2*t)/(t * Math.sin(AimObj.elevation));
    }
}
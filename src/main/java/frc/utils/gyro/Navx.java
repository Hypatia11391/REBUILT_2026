package frc.utils.gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;

public class NavX extends SubsystemBase{
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public NavX(){
        // Optional: give the sensor a moment to start calibrating on boot.
        // Don't block here; just be aware calibration happens.
    }


    // Yaw heading in degrees [-180, 180]
    public double getYawDeg(){
        return navx.getYaw();
    }

    // Continuous angle (total accumulated yaw angle)
    // Can grow beyond 360. Great for odometry turning
    public double getAngleDeg(){
        return navx.getAngle(); 
    }

    // rate in deg/sec
    public double getRateDegPerSec(){
        return navx.getRate();
    }

    // Pitch from the IMU in deg
    public double getPitchDeg(){
        return navx.getPitch();
    }

    // Roll from the IMU in deg
    public double getRollDeg(){
        return navx.getRoll();
    }

    // Raw-ish linear accel in G. 
    // Useful for debugging, not for precise motion.
    public double getWorldLinearAccelX(){
        return navx.getWorldLinearAccelX();
    }
    public double getWorldLinearAccelY(){
        return navx.getWorldLinearAccelY();
    }

    // Use to zero the Yaw values at the start of the match
    public void zeroYaw(){
        navx.zeroYaw();
    }

    // If you use getAngle(), reset it to 0.
    public void resetAngle(){
        navx.reset();
    }

    public boolean isCalibrating(){
        return navx.isCalibrating();
    }

    public boolean isConnected(){
        return navx.isConnected();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("navX/Connected", isConnected());
        SmartDashboard.putBoolean("navX/Calibrating", isCalibrating());
        SmartDashboard.putNumber("navX/YawDeg", getYawDeg());
        SmartDashboard.putNumber("navX/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("navX/RateDegPerSec", getRateDegPerSec());
        SmartDashboard.putNumber("navX/PitchDeg", getPitchDeg());
        SmartDashboard.putNumber("navX/RollDeg", getRollDeg());
    }
}

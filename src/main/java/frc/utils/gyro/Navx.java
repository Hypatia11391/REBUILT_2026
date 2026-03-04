package frc.utils.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;

public class Navx extends SubsystemBase{
    public final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // private double yawOffsetDeg = 0.0;
    // private boolean fieldCalibrated = false;

    public Navx(){
    }

    // public void calibrateFieldOrientationFromCompass(){
    //     double compassDeg = navx.getFusedHeading(); // [0, 360]
    //     double yawDeg = navx.getYaw(); // [-180, 180]

    //     double compassSigned = wrapTo180(compassDeg);

    //     yawOffsetDeg = compassSigned-yawDeg;
    //     yawOffsetDeg = wrapTo180(yawOffsetDeg);

    //     fieldCalibrated = true;
    // }

    // public boolean isFieldCalibrated(){
    //     return fieldCalibrated;
    // }

    public double getFieldHeadingDeg(){
    
        SmartDashboard.putBoolean("magnometerCalibrated", navx.isMagnetometerCalibrated());
        SmartDashboard.putNumber("navX/fusedHead", wrapTo180(navx.getFusedHeading()));
        return wrapTo180(navx.getFusedHeading());
    }


    private static double wrapTo180(double deg){
        deg = deg % 360.0;
        if (deg >= 180.0) deg -=360.0;
        if (deg<-180.0) deg+=360.0;
        return deg;
    }

    public double getHeadingDeg(){
        return -navx.getYaw();
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
    public void resetNavX(){
        navx.reset();
    }

    public boolean isCalibrating(){
        return navx.isCalibrating();
    }

    public boolean isConnected(){
        return navx.isConnected();
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(navx.getYaw());
    }
  
    public Rotation3d getFullRotation() {
        return new Rotation3d(
            Math.toRadians(getRollDeg()),
            Math.toRadians(getPitchDeg()),
            getHeading().getRadians()); // possibly getYawDeg();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("navX/Connected", isConnected());
        // SmartDashboard.putBoolean("navX/Calibrating", isCalibrating());
        // SmartDashboard.putBoolean("navX/isFieldCalibrated", fieldCalibrated);
        SmartDashboard.putNumber("navX/YawDeg", getHeadingDeg());
        // SmartDashboard.putNumber("navX/AngleDeg", getAngleDeg());
        // SmartDashboard.putNumber("navX/RateDegPerSec", getRateDegPerSec());
        SmartDashboard.putNumber("navX/PitchDeg", getPitchDeg());
        SmartDashboard.putNumber("navX/RollDeg", getRollDeg());
        SmartDashboard.putNumber("navX/AngleVal", getAngleDeg());
        // SmartDashboard.putNumber("navx/CompassDeg", getCompassDeg());
    }
}

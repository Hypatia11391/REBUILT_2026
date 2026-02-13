/*
 * DriveBase.java
 * 
 * Low-level control of the mecanum drive
 * 
 * - Contains motor controller object, 
 * - Contains WPILib mecanum drive object, 
 * - Runs methods (ie, drive, stop, etc)
 * 
 * This class should NOT read controller input or handle logic
 */

package frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleConsumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase { 

  // tune this to cap max output for testing
  private static final double MAX_SPEED = 1.0;
  
  // CAN IDs (spark max)
  private static final int FRONT_LEFT_ID = 4;
  private static final int FRONT_RIGHT_ID  = 1;
  private static final int REAR_LEFT_ID = 3;
  private static final int REAR_RIGHT_ID = 2;

 
  private static final int SHOOTER_NEO_LOW_ID = 0;
  private static final int SHOOTER_NEO_TOP_ID = 0;
  /*  
  *   brushed - CIS for the rollers
  *   brushed - non CIS lifting
  *   brushed - CIS for the kicker
  *   brushless - NEO x2
  */  
  

  private final SparkMax frontRight;
  private final SparkMax frontLeft;
  private final SparkMax rearRight;
  private final SparkMax rearLeft;

  private final SparkMax shooterNeoLow;
  private final SparkMax shooterNeoTop;

  private final MecanumDrive m_Drive; 

  public DriveBase() { 

    // wheel motors
    frontLeft = new SparkMax(FRONT_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    frontRight = new SparkMax(FRONT_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
    rearLeft = new SparkMax(REAR_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    rearRight = new SparkMax(REAR_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);

    // shooter motors
    shooterNeoLow = new SparkMax(SHOOTER_NEO_LOW_ID, SparkLowLevel.MotorType.kBrushless);
    shooterNeoTop = new SparkMax(SHOOTER_NEO_TOP_ID, SparkLowLevel.MotorType.kBrushless);

    

    
    

    configureMotor(frontLeft, true);
    configureMotor(frontRight, false);
    configureMotor(rearLeft, true);
    configureMotor(rearRight, false);

    configureShooterMotor(shooterNeoLow, false); 
    configureShooterMotor(shooterNeoTop, false);
    
    

    m_Drive = new MecanumDrive(
      cappedSetter(frontLeft, MAX_SPEED),
      cappedSetter(rearLeft, MAX_SPEED),
      cappedSetter(frontRight, MAX_SPEED),
      cappedSetter(rearRight, MAX_SPEED)
    );

    // motors visible in shuffleboard
    SendableRegistry.addChild(m_Drive, frontLeft); 
    SendableRegistry.addChild(m_Drive, rearLeft); 
    SendableRegistry.addChild(m_Drive, frontRight);
    SendableRegistry.addChild(m_Drive, rearRight);
    }
    
    /** robot-oriented if gyroAngle is zero. field-oriented if real gyro angle is passed. */
    public void driveCartesian(double xSpeed, double ySpeed, double zRot, Rotation2d gyroAngle){
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("zRot", zRot);
      m_Drive.driveCartesian(xSpeed, ySpeed, zRot, gyroAngle); 
    }
    public void driveCartesian(double xSpeed, double ySpeed, double zRot){
      m_Drive.driveCartesian(xSpeed, ySpeed, zRot, new Rotation2d()); 
    }

    public void stop() {
      m_Drive.stopMotor();
    }

    public void configureMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted);
      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void configureShooterMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50)
            .voltageCompensation(12)
            .openLoopRampRate(0.25);

      // TODO: figure out the PID stuff
      // config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //   .p(0.0001, ClosedLoopSlot.kSlot0)
      //   .i(0, ClosedLoopSlot.kSlot0)
      //   .d(0, ClosedLoopSlot.kSlot0)
      //   .outputRange(0, 1, ClosedLoopSlot.kSlot0);
      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }
    


    private static DoubleConsumer cappedSetter(SparkMax controller, double maxSpeed) {
        return speed -> controller.set(maxSpeed * speed);
    }

    
}
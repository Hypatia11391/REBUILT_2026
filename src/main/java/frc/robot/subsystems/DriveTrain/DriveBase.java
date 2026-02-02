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
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableRegistry; // for motors to show up in shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // later can switch to the shuffleboard

//import edu.wpi.first.wpilibj.TimedRobot; //TODO: check whether or we can use it, cuz it seems to be off
import edu.wpi.first.wpilibj.drive.MecanumDrive; // mecanum drive math
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveWithJoystick;

public class DriveBase extends SubsystemBase { // main class that extend TimedRobot
  private final MecanumDrive m_Drive; // mecanum drive object

  private static final double MAX_SPEED = 0.1;
  
  private static final int kFrontLeftChannel = 4; // front left port
  private static final int kFrontRightChannel = 1; // front right port
  private static final int kRearLeftChannel = 3; // rear left port
  private static final int kRearRightChannel = 2; // rear right port

  // rear right and joystick port are on different devices, so the ports can be the same

  /**it  Called once at the beginning of the robot program. */
  public DriveBase() { // constructor
    // PWMSparkMax frontLeft = new PWMSparkMax(kFrontLeftChannel); // front left motor controller
    // PWMSparkMax rearLeft = new PWMSparkMax(kRearLeftChannel); // rear left motor controller
    // PWMSparkMax frontRight = new PWMSparkMax(kFrontRightChannel); // front right motor controller
    // PWMSparkMax rearRight = new PWMSparkMax(kRearRightChannel); // rear right motor controller

    SparkMax frontRight = new SparkMax(kFrontRightChannel, SparkLowLevel.MotorType.kBrushless); // front right motor controller
    SparkMax frontLeft = new SparkMax(kFrontLeftChannel, SparkLowLevel.MotorType.kBrushless); // front right motor controller
    SparkMax rearRight = new SparkMax(kRearRightChannel, SparkLowLevel.MotorType.kBrushless); // front right motor controller
    SparkMax rearLeft = new SparkMax(kRearLeftChannel, SparkLowLevel.MotorType.kBrushless); // front right motor controller
    
    frontLeft.configureAsync(
      new SparkMaxConfig().inverted(true),
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    frontRight.configureAsync(
      new SparkMaxConfig().inverted(false),
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    rearLeft.configureAsync(
      new SparkMaxConfig().inverted(true),
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    rearRight.configureAsync(
      new SparkMaxConfig().inverted(false),
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    // TODO: look if method reference can be removed
    m_Drive = new MecanumDrive(
      createCappedSpeedSetter(frontLeft, MAX_SPEED),
      createCappedSpeedSetter(rearLeft, MAX_SPEED),
      createCappedSpeedSetter(frontRight, MAX_SPEED),
      createCappedSpeedSetter(rearRight, MAX_SPEED)
    );
        // for debugging
        SendableRegistry.addChild(m_Drive, frontLeft); // front left motor shows up on the shuffleboard
        SendableRegistry.addChild(m_Drive, rearLeft); // rear left motor shows up on the shuffleboard
        SendableRegistry.addChild(m_Drive, frontRight); // front right motor shows up on the shuffleboard
        SendableRegistry.addChild(m_Drive, rearRight); // rear right motor shows up on the shuffleboard
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRot){
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("zRot", zRot);
      m_Drive.driveCartesian(xSpeed, ySpeed, zRot); 
    }

    public DoubleConsumer createCappedSpeedSetter(SparkMax controller, double maxSpeed) {
      return (speed) -> {
        controller.set(maxSpeed * speed);
      };
    }

    public void stop() {
      m_Drive.stopMotor();
    }
}
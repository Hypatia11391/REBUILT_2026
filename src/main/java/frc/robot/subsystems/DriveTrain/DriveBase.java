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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.poseEstimation.OdometryManager;
import frc.utils.gyro.Navx;

import java.util.function.DoubleConsumer;

public class DriveBase extends SubsystemBase { // main class that extend TimedRobot
  private final MecanumDrive m_Drive; // mecanum drive object

    // tune this to cap max output for testing
  private static final double MAX_SPEED = 1.0;
  
  // CAN IDs (spark max)
  private static final int FRONT_LEFT_ID = 4;
  private static final int FRONT_RIGHT_ID  = 1;
  private static final int REAR_LEFT_ID = 3;
  private static final int REAR_RIGHT_ID = 2;

  private final OdometryManager odometryManager;

  private static final double MAGICAL_CONSTANT_I_DONT_WANT_TO_FIGURE_OUT = 1; // TODO: JUST... FIX THIS
  private static final double OTHER_MAGICAL_CONSTANT_I_DONT_WANT_TO_FIGURE_OUT = 1; // TODO: FIX THIS TOO

  /**it  Called once at the beginning of the robot program. */
  public DriveBase(Navx navx) { // constructor

      // wheel motors
    SparkMax frontLeft = new SparkMax(FRONT_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMax frontRight = new SparkMax(FRONT_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMax rearLeft = new SparkMax(REAR_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMax rearRight = new SparkMax(REAR_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
    
    AbsoluteEncoderConfig conf = new AbsoluteEncoderConfig();
    conf = conf.positionConversionFactor(OdometryManager.WHEEL_CIRCUMFERENCE);
    conf = conf.velocityConversionFactor(OdometryManager.WHEEL_CIRCUMFERENCE);

    configureMotor(frontLeft, true,conf);
    configureMotor(frontRight, false, conf);
    configureMotor(rearLeft, true, conf);
    configureMotor(rearRight, false, conf);
    
    SparkAbsoluteEncoder frEncoder = frontRight.getAbsoluteEncoder();
    SparkAbsoluteEncoder flEncoder = frontLeft.getAbsoluteEncoder();
    SparkAbsoluteEncoder rrEncoder = rearRight.getAbsoluteEncoder();
    SparkAbsoluteEncoder rlEncoder = rearLeft.getAbsoluteEncoder();

    odometryManager = new OdometryManager(frEncoder,flEncoder,rrEncoder,rlEncoder,navx);

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
  
    initPathPlanner();
    }
    
    private void initPathPlanner() {
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
      odometryManager::getPose,
      odometryManager::resetPose,
      odometryManager::getRelativeSpeeds,
      (speeds, ff) -> {
        m_Drive.driveCartesian(
            speeds.vyMetersPerSecond * MAGICAL_CONSTANT_I_DONT_WANT_TO_FIGURE_OUT,
            speeds.vxMetersPerSecond * MAGICAL_CONSTANT_I_DONT_WANT_TO_FIGURE_OUT,
            speeds.omegaRadiansPerSecond * OTHER_MAGICAL_CONSTANT_I_DONT_WANT_TO_FIGURE_OUT
        );
      },
      new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants TODO: THIS IS PROBABLY WRONG
          new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants TODO: THIS IS PROBABLY WRONG
      ),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
      },
      this
    );
  }

  public OdometryManager getOdometryManager() {
    return odometryManager;
  }

  @Override
  public void periodic() {
    odometryManager.update();

    super.periodic();
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

  public void configureMotor(SparkMax motor, boolean isInverted, AbsoluteEncoderConfig absEncoderConf){
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(isInverted);
    motor.configureAsync(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  private static DoubleConsumer cappedSetter(SparkMax controller, double maxSpeed) {
      return speed -> controller.set(maxSpeed * speed);
  }
}
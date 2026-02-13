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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableRegistry; // for motors to show up in shuffleboard
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // later can switch to the shuffleboard

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.gyro.Navx;
// import frc.robot.commands.DriveWithJoystick;

public class DriveBase extends SubsystemBase { // main class that extend TimedRobot
  private final MecanumDrive m_Drive; // mecanum drive object
  private final Navx navx;

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
   
  //     -/  \
  // ^+x
  // |   -\  /
  // '-> +y

  private static final double ROOT_TWO = Math.sqrt(2);
  private static final double MAGICAL_CONSTANT_I_DONT_WANT_TO_FIGURE_OUT = 1; // TODO: JUST... FIX THIS
  private static final double OTHER_MAGICAL_CONSTANT_I_DONT_WANT_TO_FIGURE_OUT = 1; // TODO: FIX THIS TOO

  private static final Translation2d FRONT_LEFT_MOVE = new Translation2d(ROOT_TWO,ROOT_TWO);
  private static final Translation2d FRONT_RIGHT_MOVE = new Translation2d(ROOT_TWO,-ROOT_TWO);
  private static final Translation2d REAR_LEFT_MOVE = new Translation2d(ROOT_TWO,-ROOT_TWO);
  private static final Translation2d REAR_RIGHT_MOVE = new Translation2d(ROOT_TWO,ROOT_TWO);

  // TODO: MEASURE
  private static final double WHEEL_DIAMETER = 0.25;
  private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

  private static final Pose2d STARTING_POSE = Pose2d.kZero; // TODO: Correct to be the actual starting position!
  private final SparkMax shooterNeoLow;
  private final SparkMax shooterNeoTop;


  private Pose2d currentPose = STARTING_POSE;

  private final SparkAbsoluteEncoder frEncoder;
  private final SparkAbsoluteEncoder flEncoder;
  private final SparkAbsoluteEncoder rrEncoder;
  private final SparkAbsoluteEncoder rlEncoder;

  /**it  Called once at the beginning of the robot program. */
  public DriveBase(Navx navx) { // constructor
    this.navx = navx;

    // wheel motors
    frontLeft = new SparkMax(FRONT_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    frontRight = new SparkMax(FRONT_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
    rearLeft = new SparkMax(REAR_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    rearRight = new SparkMax(REAR_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
    
    AbsoluteEncoderConfig conf = new AbsoluteEncoderConfig();
    conf = conf.positionConversionFactor(WHEEL_CIRCUMFERENCE);
    conf = conf.velocityConversionFactor(WHEEL_CIRCUMFERENCE);

    // shooter motors
    shooterNeoLow = new SparkMax(SHOOTER_NEO_LOW_ID, SparkLowLevel.MotorType.kBrushless);
    shooterNeoTop = new SparkMax(SHOOTER_NEO_TOP_ID, SparkLowLevel.MotorType.kBrushless);


    configureMotor(frontLeft, true);
    configureMotor(frontRight, false);
    configureMotor(rearLeft, true);
    configureMotor(rearRight, false);
    
    frEncoder = frontRight.getAbsoluteEncoder();
    flEncoder = frontLeft.getAbsoluteEncoder();
    rrEncoder = rearRight.getAbsoluteEncoder();
    rlEncoder = rearLeft.getAbsoluteEncoder();

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
      this::getPose,
      this::resetPose,
      this::getRelativeSpeeds,
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

  @Override
  public void periodic() {
    Translation2d vec = FRONT_RIGHT_MOVE.times(frEncoder.getPosition())
      .plus(
        FRONT_LEFT_MOVE.times(flEncoder.getPosition())
      ).plus(
        REAR_LEFT_MOVE.times(rlEncoder.getPosition())
      ).plus(
        REAR_RIGHT_MOVE.times(rrEncoder.getPosition())
      );

    Rotation2d newRotation = new Rotation2d(Angle.ofBaseUnits(navx.getYawDeg(), Units.Degrees));

    vec.rotateBy(newRotation);

    this.currentPose = new Pose2d(
        this.currentPose.getTranslation()
            .plus(vec),
        newRotation
    );

    super.periodic();
  }

  public ChassisSpeeds getRelativeSpeeds() {
    Translation2d velocity = FRONT_RIGHT_MOVE.times(frEncoder.getVelocity())
        .plus(
            FRONT_LEFT_MOVE.times(flEncoder.getVelocity())
        ).plus(
            REAR_LEFT_MOVE.times(rlEncoder.getVelocity())
        ).plus(
            REAR_RIGHT_MOVE.times(rrEncoder.getVelocity())
        );

    return new ChassisSpeeds(
        velocity.getX(),
        velocity.getY(),
        navx.getRateDegPerSec() * Math.PI / 180
    );
  }

  public void resetPose(Pose2d pose) {
    this.currentPose = pose;
  }

  public Pose2d getPose() {
    return currentPose;
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
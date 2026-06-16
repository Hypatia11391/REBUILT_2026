/*
 * DriveBase.java
 *
 * Low-level control of the mecanum drive
 *
 * - Contains motor controller object,
 * - Contains WPILib mecanum drive object,
 * - Runs methods (ie, drive, stop, etc.)
 *
 * This class should NOT read controller input or handle logic
 */

package frc.robot.subsystems.DriveTrain;

// import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // later can switch to the shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.utils.gyro.Navx;
import java.time.Instant;
import java.util.function.DoubleConsumer;

//import frc.robot.commands.Autos;

public class DriveBase extends SubsystemBase {

    // main class that extend TimedRobot
    private final MecanumDrive m_Drive; // mecanum drive object

    private final RelativeEncoder frEncoder;
    private final RelativeEncoder flEncoder;
    private final RelativeEncoder rrEncoder;
    private final RelativeEncoder rlEncoder;

    private final Field2d m_field = new Field2d();
    private final SparkMax frSparkMax;
    private final SparkMax flSparkMax;
    private final SparkMax rrSparkMax;
    private final SparkMax rlSparkMax;

    private final MecanumDrivePoseEstimator3d poseEstimator;
    private final Navx navx;
    private final MecanumDriveKinematics driveKinematics;

    private static Pose2d currentPose;

    /** Called once at the beginning of the robot program. */
    public DriveBase(
        Navx navx,
        MecanumDrivePoseEstimator3d poseEstimator,
        MecanumDriveKinematics kinematics
    ) {
        // constructor
        this.poseEstimator = poseEstimator;
        this.navx = navx;
        this.driveKinematics = kinematics;

        // wheel motors
        flSparkMax = new SparkMax(
            DriveBaseConstants.FRONT_LEFT_ID,
            SparkLowLevel.MotorType.kBrushless
        );
        frSparkMax = new SparkMax(
            DriveBaseConstants.FRONT_RIGHT_ID,
            SparkLowLevel.MotorType.kBrushless
        );
        rlSparkMax = new SparkMax(
            DriveBaseConstants.REAR_LEFT_ID,
            SparkLowLevel.MotorType.kBrushless
        );
        rrSparkMax = new SparkMax(
            DriveBaseConstants.REAR_RIGHT_ID,
            SparkLowLevel.MotorType.kBrushless
        );

        AbsoluteEncoderConfig conf = new AbsoluteEncoderConfig();
        conf = conf.positionConversionFactor(
            DriveBaseConstants.WHEEL_CIRCUMFERENCE
        );
        conf = conf.velocityConversionFactor(
            DriveBaseConstants.WHEEL_CIRCUMFERENCE
        );

        configureMotor(flSparkMax, true, conf);
        configureMotor(frSparkMax, false, conf);
        configureMotor(rlSparkMax, true, conf);
        configureMotor(rrSparkMax, false, conf);

        this.flEncoder = flSparkMax.getEncoder();
        this.frEncoder = frSparkMax.getEncoder();
        this.rlEncoder = rlSparkMax.getEncoder();
        this.rrEncoder = rrSparkMax.getEncoder();

        m_Drive = new MecanumDrive(
            cappedSetter(flSparkMax, DriveBaseConstants.MAX_SPEED),
            cappedSetter(rlSparkMax, DriveBaseConstants.MAX_SPEED),
            cappedSetter(frSparkMax, DriveBaseConstants.MAX_SPEED),
            cappedSetter(rrSparkMax, DriveBaseConstants.MAX_SPEED)
        );

        // motors visible in shuffleboard
        SendableRegistry.addChild(m_Drive, flSparkMax);
        SendableRegistry.addChild(m_Drive, frSparkMax);
        SendableRegistry.addChild(m_Drive, rlSparkMax);
        SendableRegistry.addChild(m_Drive, rrSparkMax);

        SmartDashboard.putData("Field", m_field);

        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            DriveBase::getPose2D,
            this::resetPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        Pose3d temp = poseEstimator.getEstimatedPosition();
        currentPose = temp.toPose2d();
        m_field.setRobotPose(currentPose);

        super.periodic();

        return new MecanumDriveWheelSpeeds(
            flEncoder.getVelocity(),
            frEncoder.getVelocity(),
            rlEncoder.getVelocity(),
            rrEncoder.getVelocity()
        );
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
            flEncoder.getPosition(),
            frEncoder.getPosition(),
            rlEncoder.getPosition(),
            rrEncoder.getPosition()
        );
    }

    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return driveKinematics.toChassisSpeeds(getWheelSpeeds());
    }

    @Override
    public void periodic() {
        this.poseEstimator.updateWithTime(
            //Instant.now().toEpochMilli() / 1000.0,
            Timer.getFPGATimestamp(),
            navx.getFullRotation(),
            this.getWheelPositions()
        );

        updateAimInstance();

        SmartDashboard.putNumber("frontLeftVel", flEncoder.getVelocity());
        SmartDashboard.putNumber("frontRightVel", frEncoder.getVelocity());
        SmartDashboard.putNumber("rearLeftVel", rlEncoder.getVelocity());
        SmartDashboard.putNumber("rearRightVel", rrEncoder.getVelocity());
    }

    // public void driveAtSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    //   flSparkMax.set(wheelSpeeds.frontLeftMetersPerSecond);
    //   frSparkMax.set(wheelSpeeds.frontRightMetersPerSecond);
    //   rlSparkMax.set(wheelSpeeds.rearLeftMetersPerSecond);
    //   rrSparkMax.set( wheelSpeeds.rearRightMetersPerSecond);
    // }

    /**
     * Drives the robot using robot-relative ChassisSpeeds.
     * Required by PathPlanner's AutoBuilder.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        double xPercent =
            speeds.vxMetersPerSecond / DriveBaseConstants.MAX_SPEED;
        double yPercent =
            speeds.vyMetersPerSecond / DriveBaseConstants.MAX_SPEED;

        double rotPercent =
            speeds.omegaRadiansPerSecond / DriveBaseConstants.MAX_ANGULAR_SPEED;

        xPercent = Math.max(-1.0, Math.min(1.0, xPercent));
        yPercent = Math.max(-1.0, Math.min(1.0, yPercent));
        rotPercent = Math.max(-1.0, Math.min(1.0, rotPercent));

        driveCartesian(xPercent, yPercent, rotPercent, new Rotation2d());
    }

    /* robot-oriented if gyroAngle is zero. field-oriented if real gyro angle is passed. */
    public void driveCartesian(
        double xSpeed,
        double ySpeed,
        double zRot,
        Rotation2d gyroAngle
    ) {
        SmartDashboard.putNumber("xSpeed", xSpeed);
<<<<<<< HEAD
        SmartDashboard.putNumber("zRot", zRot);
=======

        SmartDashboard.putNumber("zRot", zRot);

>>>>>>> 8cef887c1b21f3973a803de1ae97ecf86b03f52f
        SmartDashboard.putNumber("ySpeed", ySpeed);

        m_Drive.driveCartesian(ySpeed, xSpeed, zRot, gyroAngle);
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRot) {
        m_Drive.driveCartesian(ySpeed, xSpeed, zRot, new Rotation2d());
    }

    public void stop() {
        m_Drive.stopMotor();
    }

    public void configureMotor(
        SparkMax motor,
        boolean isInverted,
        AbsoluteEncoderConfig absEncoderConf
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(isInverted);
        config.absoluteEncoder.apply(absEncoderConf);
        motor.configureAsync(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    private static DoubleConsumer cappedSetter(
        SparkMax controller,
        double maxSpeed
    ) {
        return speed -> controller.set(maxSpeed * speed);
    }

    public static Pose2d getPose2D() {
        return currentPose;
    }

    public void updateAimInstance() {
        RobotContainer.aimInstance.updateRobotState(
            currentPose,
            getChassisSpeeds()
        );
    }

    public Command driveRobotRelativeCommand(ChassisSpeeds speeds) {
        return this.run(() -> driveRobotRelative(speeds)).finallyDo(() ->
            stop()
        );
    }

    public Command driveCartesianCommand(
        double xSpeed,
        double ySpeed,
        double zRot
    ) {
        return this.run(() -> driveCartesian(xSpeed, ySpeed, zRot)).finallyDo(
            () -> stop()
        );
    }

    public Command stopCommand() {
        return this.run(() -> stop());
    }
}

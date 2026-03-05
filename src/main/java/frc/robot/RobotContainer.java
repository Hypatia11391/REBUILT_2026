/**
 * RobotContainer
 * <p>
 * Defines the overall structure of the robot.
 * - Creates subsystems (e.g. DriveBase)
 * - Creates controllers and joysticks
 * - Sets default commands
 * RobotContainer handles all wiring between subsystems and commands.
 */

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Aim;
import frc.robot.commands.Autos;
import frc.robot.commands.Buttons;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.OperateWithJoystick;
import frc.robot.poseEstimation.VisionManager;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Mechanisms.Feed;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.Kicker;
import frc.robot.subsystems.Mechanisms.Shooter;
import frc.utils.gyro.Navx;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick m_operatorController =
      new Joystick(OperatorConstants.kOperateControllerPort);
  private final Navx navx = 
      new Navx();
  private final Shooter shooter =
      new Shooter();
  private final Intake intake =
      new Intake();
  private final Kicker kicker =
      new Kicker();
  private final Feed feed =
      new Feed();

    // TODO!!!!: MEASUREMENTS!!!!! MEASURE THE POSITIONS OF EACH WHEEL RELATIVE TO ROBOT ORIGIN!!!
    private static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
        new Translation2d(0,0),
        new Translation2d(0,0),
        new Translation2d(0,0),
        new Translation2d(0,0)
    );

    private static final Matrix<N4, N1> VISION_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1, 0.1);
    private static final Matrix<N4, N1> STATE_STD_DEVS = VecBuilder.fill(0.45, 0.45, 0.45, 0.45);

    private static final Pose3d STARTING_POSE = Pose3d.kZero; // TODO: Correct to be the actual starting position!

    private final MecanumDrivePoseEstimator3d poseEstimator = new MecanumDrivePoseEstimator3d(
        DRIVE_KINEMATICS,
        navx.getFullRotation(),
        new MecanumDriveWheelPositions(), // This could have some problem if the default position for the wheels are not "zero"
        STARTING_POSE,
        VISION_STD_DEVS,
        STATE_STD_DEVS
    );

    
  /* ???????????????
    public MecanumDrivePoseEstimator3d getPositionFromPoseEstimator() {
      return poseEstimator;
    };
    */

    private final VisionManager visionManager = new VisionManager(poseEstimator);

  private final DriveBase m_driveBase =
      new DriveBase(navx,poseEstimator,DRIVE_KINEMATICS);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveBase.setDefaultCommand(
      new DriveWithJoystick(m_driveBase, m_driverController, navx));
    shooter.setDefaultCommand(
      new OperateWithJoystick(shooter, m_driverController, intake, kicker, feed)); // TODO: change to operator
    configureBindings();
  }

  public DriveBase getDriveBase() {
    return m_driveBase;
  }
  public Navx getNavx(){
    return navx;
  }

  public void update() {
    visionManager.update();
  }

  /**
   * Maps driver inputs (buttons/triggers) to commands.
   * This is where controller buttons are bound to robot actions.
   * Called once during robot initialization.
   */


  private void configureBindings() {
    
    // new JoystickButton(m_driverController, Buttons.LS.ordinal()).onTrue(new InstantCommand(navx::zeroYaw, navx));

    new JoystickButton(m_driverController, Buttons.X.ordinal()).whileTrue(new InstantCommand(m_driveBase::aimingFunction, m_driveBase)).onChange(new InstantCommand(Aim::automaticAimControl));
    
    // new JoystickButton(m_driverController, Buttons.X.ordinal() +1).onTrue(new InstantCommand(navx::calibrateFieldOrientation, navx));
    new JoystickButton(m_driverController, Buttons.B.ordinal() + 1).onTrue(new InstantCommand(intake::zeroLift, intake)); // TODO: change to operator

}

  /**
   * Returns the command that will run during autonomous mode.
   * Called by {@link Robot} when autonomous starts.
   */
  public Command getAutonomousCommand() {
    return Autos.autonomousFull("pathplanner/paths.txt");
  }

}

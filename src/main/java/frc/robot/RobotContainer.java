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

import frc.utils.gyro.Navx;
import frc.robot.commands.Autos;
import frc.robot.commands.Buttons;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.OperateWithJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Mechanisms.Feed;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.Mechanisms.Kicker;
import frc.robot.subsystems.Mechanisms.Shooter;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



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
  
  private final DriveBase m_driveBase =
      new DriveBase(navx);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveBase.setDefaultCommand(
      new DriveWithJoystick(m_driveBase, m_driverController, navx));
    shooter.setDefaultCommand(
      new OperateWithJoystick(shooter, m_operatorController, intake, kicker, feed));
    configureBindings();
  }

  /**
   * Maps driver inputs (buttons/triggers) to commands.
   * This is where controller buttons are bound to robot actions.
   * Called once during robot initialization.
   */

  private void configureBindings() {
    new JoystickButton(m_driverController, Buttons.LS.ordinal()).onTrue(new InstantCommand(navx::zeroYaw, navx));
  }

  /**
   * Returns the command that will run during autonomous mode.
   * Called by {@link Robot} when autonomous starts.
   */
  public Command getAutonomousCommand() {
    return Autos.autonomousFull("pathplanner/paths.txt");
  }
}

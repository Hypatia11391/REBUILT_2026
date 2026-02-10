/**
 * RobotContainer
 *
 * Defines the overall structure of the robot.
 * - Creates subsystems (e.g. DriveBase)
 * - Creates controllers and joysticks
 * - Sets default commands
 * RobotContainer handles all wiring between subsystems and commands.
 */

package frc.robot;

import frc.utils.gyro.Navx;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain.DriveBase;


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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);

  private final Navx navx = 
      new Navx();
  
  private final DriveBase m_driveBase =
      new DriveBase();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveBase.setDefaultCommand(
      new DriveWithJoystick(m_driveBase, m_driverController, navx));
      configureBindings();
  }

  /**
   * Maps driver inputs (buttons/triggers) to commands.
   * This is where controller buttons are bound to robot actions.
   * Called once during robot initialization.
   */

  private void configureBindings() {
    new JoystickButton(m_driverController, 1).onTrue(new InstantCommand(navx::zeroYaw, navx));
  }

  /**
   * Returns the command that will run during autonomous mode.
   * Called by {@link Robot} when autonomous starts.
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

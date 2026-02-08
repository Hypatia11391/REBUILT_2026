package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    // RobotContainer wires subsystems + default commands + button bindings.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Required for command-based: runs commands + polls buttons each loop.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}


  @Override
  public void autonomousInit() {
    /** runs the autonomous command selected by your {@link RobotContainer} class. */
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // This function is called periodically during autonomous. 
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Stop auto when teleop starts.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during teleop. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Clean slate for test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

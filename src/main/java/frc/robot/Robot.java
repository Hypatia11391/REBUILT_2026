package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain.DriveBase;
//Logging
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;


  public Robot() {
    // robotContainer wires subsystems + default commands + button bindings
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    

  }

  @Override
  public void robotPeriodic() {
    // required for command-based: runs commands + polls buttons each loop
    CommandScheduler.getInstance().run();
    this.m_robotContainer.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // runs the autonomous command selected by your {@link RobotContainer} class.
  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
    m_robotContainer.getNavx().zeroYaw();

    // if (!navx.isCalibrating() && !navx.isFieldCalibrated()){
    //   navx.calibrateFieldOrientationFromCompass();
    // }
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

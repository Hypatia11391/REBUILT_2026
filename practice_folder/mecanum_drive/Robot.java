
// This code is taken from WPILib's official examples.

// ASSIGNMENT:
// Work in pairs: one experienced programmer + one less-experienced programmer.
// 1. Comment each line of this code (explain what each line does).
// 2. Create your own branch off of the `practice-branch` (one branch per group).
// 3. Create a new file in the `tester_files/mecanum_drive` folder with your commented code.
// 4. Push your changes to GitHub on your new branch.
// 5. Create a Pull Request asking one of the leaders (Pavel, Owen, Maxime) to review and approve your work.

// You are allowed to:
// - Look at WPILib's documentation.
// - Look at the slides in Google Drive about GitHub and the drivetrain.

// You are NOT allowed to:
// - Look at the comments provided in WPILib's GitHub examples.
// - Use AI to generate comments for this code.

// We do not have a way to directly check whether you cheated, but we trust you and expect integrity from everyone on the team.

// If you have any questions: talk to one of the leaders.


package edu.wpi.first.wpilibj.examples.mecanumdrive;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 0;

  private static final int kJoystickChannel = 0;

  private final MecanumDrive m_robotDrive;
  private final Joystick m_stick;

  public Robot() {
    PWMSparkMax frontLeft = new PWMSparkMax(kFrontLeftChannel);
    PWMSparkMax rearLeft = new PWMSparkMax(kRearLeftChannel);
    PWMSparkMax frontRight = new PWMSparkMax(kFrontRightChannel);
    PWMSparkMax rearRight = new PWMSparkMax(kRearRightChannel);

    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);

    m_stick = new Joystick(kJoystickChannel);

    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.driveCartesian(-m_stick.getY(), -m_stick.getX(), -m_stick.getZ());
  }
}
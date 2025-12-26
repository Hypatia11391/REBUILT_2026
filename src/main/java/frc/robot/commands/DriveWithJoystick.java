/**
 * DrivWithJoystick.java
 * 
 * Teleop for driving the robot
 * 
 * - Reads controller inpurt
 * - Applies deadbands and scaling
 * - Calls drive method from DriveBase.java // if someone knows how to do {@link} properly then do it for this line
 * 
 * This file should NOT create/configure hardware
 **/

package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;

public class DriveWithJoystick extends Command{
  private static final double MAX_SPEED = 0.1;
  private static final double DEADZONE = 0.1;
  private final DriveBase m_drive;
  private final Joystick m_stick; // joystick object

  private double currX = 0;
  private double currY = 0;

  public DriveWithJoystick(DriveBase drive, Joystick stick){
      m_drive = drive;
      m_stick = stick;
      addRequirements(m_drive);
  }
  @Override
  public void initialize(){}

  @Override
  public void execute(){
    double xSpeed = m_stick.getRawAxis(JoystickAxes.LEFT_X.ordinal());
    double ySpeed = -m_stick.getRawAxis(JoystickAxes.LEFT_Y.ordinal());
    double zRot = m_stick.getRawAxis(JoystickAxes.RIGHT_X.ordinal());

    if(xSpeed * xSpeed + ySpeed * ySpeed > DEADZONE * DEADZONE) { // deadzone
      if(xSpeed * xSpeed + ySpeed * ySpeed > 1) {
        double length = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        xSpeed /= length;
        ySpeed /= length;
      }

      xSpeed = MAX_SPEED * xSpeed;
      ySpeed = MAX_SPEED * ySpeed;

      currX = currX * 0.95 + xSpeed * 0.05;
      currY = currY * 0.95 + ySpeed * 0.05;

      m_drive.drive(currX,currY,zRot);
    } else {
      currX = currX * 0.95;
      currY = currY * 0.95;
      m_drive.drive(currX,currY,0);
    }

    System.out.printf("%f,%f,%f, len: %f\n",xSpeed, ySpeed, zRot, Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed));
  }
  @Override
  public void end(boolean interrupted){
    m_drive.drive(0,0,0);
  }
  @Override
  public boolean isFinished(){
    return false;
  }
}

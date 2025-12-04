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

    private final DriveBase m_drive;
    private final Joystick m_stick; // joystick object

    public DriveWithJoystick(DriveBase drive, Joystick stick){
        m_drive = drive;
        m_stick = stick;
        addRequirements(m_drive);
  }
  @Override
  public void initialize(){}

  @Override
  public void execute(){
    double ySpeed = -m_stick.getY();
    double xSpeed = -m_stick.getX();
    double zRot = -m_stick.getZ();
    m_drive.drive(xSpeed, ySpeed, zRot);
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

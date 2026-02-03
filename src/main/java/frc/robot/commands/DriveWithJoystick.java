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
  private static final double DEADZONE = 0.1;
  private static final double ROT_DEADZONE = 0.1;
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
    double wishX = m_stick.getRawAxis(JoystickAxes.LEFT_X.ordinal());
    double wishY = -m_stick.getRawAxis(JoystickAxes.LEFT_Y.ordinal());
    double zRot = m_stick.getRawAxis(JoystickAxes.RIGHT_X.ordinal());
    if(wishX * wishX + wishY * wishY < DEADZONE * DEADZONE) {
      wishX = 0;
      wishY = 0;
    }

    if(zRot * zRot < ROT_DEADZONE * ROT_DEADZONE) {
      zRot = 0;
    }

    if(wishX * wishX + wishY * wishY > 1) {
      double length = Math.sqrt(wishX * wishX + wishY * wishY);
      wishX /= length;
      wishY /= length;
    }

    // for (int i = 1; i <=12; i++){
    //   if (m_stick.getRawButton(i)){
    //     System.out.println("Button " + i + "pressed");
    //   }
    // }
// m_stick.getTrigger() ? 0.35 :
    double scale = 0.35;

    currX = currX * 0.95 + wishX * 0.05;
    currY = currY * 0.95 + wishY * 0.05;

    m_drive.driveCartesian(currX * scale,currY * scale,-zRot * scale); //times the scale

  }

  @Override
  public void end(boolean interrupted){
    m_drive.driveCartesian(0,0,0);
  }
  @Override
  public boolean isFinished(){
    return false;
  }
}

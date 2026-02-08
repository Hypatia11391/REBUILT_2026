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
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.utils.gyro.NavX;


public class DriveWithJoystick extends Command{
  private static final double DEADZONE = 0.1;
  private static final double ROT_DEADZONE = 0.1;

  private static final double ALPHA_XY = 0.05;
  private static final double ALPHA_Z = 0.08;

  private final DriveBase m_drive;
  private final Joystick m_stick; // joystick object
  private final NavX navX; // navx object

  private double currX = 0;
  private double currY = 0;
  private double currZ = 0;

  public DriveWithJoystick(DriveBase drive, Joystick stick, NavX navx){
      m_drive = drive;
      m_stick = stick;
      navX = navx;
      addRequirements(m_drive);
  }
  @Override
  public void initialize(){}

  @Override
  public void execute(){
    double wishX = m_stick.getRawAxis(JoystickAxes.LEFT_X.ordinal());
    double wishY = -m_stick.getRawAxis(JoystickAxes.LEFT_Y.ordinal());
    double wishZ = m_stick.getRawAxis(JoystickAxes.RIGHT_X.ordinal());
    if(wishX * wishX + wishY * wishY < DEADZONE * DEADZONE) {
      wishX = 0;
      wishY = 0;
    }

    if(wishZ * wishZ < ROT_DEADZONE * ROT_DEADZONE) {
      wishZ = 0;
    }

    double mag2 = wishX * wishX + wishY * wishY;

    if(mag2 > 1) {
      double mag = Math.sqrt(mag2);
      wishX /= mag;
      wishY /= mag;
    }
  
  // speed scaling TODO: hook to button for slow-fast
    double scale = 0.40; // % speed

    currX = currX * (1.0 - ALPHA_XY) + wishX * ALPHA_XY;
    currY = currY * (1.0 - ALPHA_XY) + wishY * ALPHA_XY;
    currZ = currZ * (1.0 - ALPHA_Z) + wishZ * ALPHA_Z;
    
    double outX = currX * scale;
    double outY = currY * scale;
    double outZ = -currZ * scale;

    //Field-oriented, navx yaw as heading
    Rotation2d heading = Rotation2d.fromDegrees(-navX.getYawDeg());

    m_drive.driveCartesian(outX, outY, outZ, heading); // scales

  }

  @Override
  public void end(boolean interrupted){
    currX = 0;
    currY = 0;
    currZ = 0;
    m_drive.stop();
  }
  @Override
  public boolean isFinished(){
    return false;
  }
}

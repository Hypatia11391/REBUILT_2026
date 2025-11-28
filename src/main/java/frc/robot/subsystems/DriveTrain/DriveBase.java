/*
 * DriveBase.java
 * 
 * Low-level control of the mecanum drive
 * 
 * - Contains motor controller object, 
 * - Contains WPILib mecanum drive object, 
 * - Runs methods (ie, drive, stop, etc)
 * 
 * This class should NOT read controller input or handle logic
 */

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.util.sendable.SendableRegistry; // for motors to show up in shuffleboard

// import edu.wpi.first.wpilibj.TimedRobot; //TODO: check whether or we can use it, cuz it seems to be off
import edu.wpi.first.wpilibj.drive.MecanumDrive; // mecanum drive math
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax; // motor controller (spark max)
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase { // main class that extend TimedRobot
  private static final int kFrontLeftChannel = 2; // front left port
  private static final int kRearLeftChannel = 3; // rear left port
  private static final int kFrontRightChannel = 1; // front right port
  private static final int kRearRightChannel = 0; // rear right port

  // private static final int kJoystickChannel = 0; // joystick port

  // rear right and joystick port are on different devices, so the ports can be the same

  private final MecanumDrive m_Drive; // mecanum drive object
  // private final Joystick m_stick; // joystick object

  /**it  Called once at the beginning of the robot program. */
  @SuppressWarnings("unused") // there's often a warning about not using some line or some variable, which is kinda annoying
  public DriveBase() { // constructor
    PWMSparkMax frontLeft = new PWMSparkMax(kFrontLeftChannel); // front left motor controller
    PWMSparkMax rearLeft = new PWMSparkMax(kRearLeftChannel); // rear left motor controller
    PWMSparkMax frontRight = new PWMSparkMax(kFrontRightChannel); // front right motor controller
    PWMSparkMax rearRight = new PWMSparkMax(kRearRightChannel); // rear right motor controller

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    // TODO: look if method reference can be removed
    m_Drive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set); 
    
        // m_stick = new Joystick(kJoystickChannel); // opens joystick
    
        // for debugging
        SendableRegistry.addChild(m_Drive, frontLeft); // front left motor shows up on the shuffleboard
        SendableRegistry.addChild(m_Drive, rearLeft); // rear left motor shows up on the shuffleboard
        SendableRegistry.addChild(m_Drive, frontRight); // front right motor shows up on the shuffleboard
        SendableRegistry.addChild(m_Drive, rearRight); // rear right motor shows up on the shuffleboard
      }
    
    
  public void driveCartesian(double xSpeed, double ySpeed, double zRotation) { // every 20 sec during teleop mode
    m_Drive.driveCartesian(xSpeed, ySpeed, zRotation);
  }
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
  /**  
     * -m_stick.getY() : forward/backword (joystick y is reversed, so we multiply but -1) 
     * -m_stick.getX() : left/right (left: positive, right: negative) 
     * -m_stick.getZ() : twist axis or sm 
    */ 

  public void stop(){
      m_Drive.stopMotor();
    }
}
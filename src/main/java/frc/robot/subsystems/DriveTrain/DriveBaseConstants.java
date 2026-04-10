package frc.robot.subsystems.DriveTrain;

public class DriveBaseConstants {

    // tune this to cap max output for testing
    public static final double MAX_SPEED = 1.0;

    // CAN IDs (spark max)
    public static final int FRONT_LEFT_ID = 9;
    public static final int FRONT_RIGHT_ID = 1;
    public static final int REAR_LEFT_ID = 10;
    public static final int REAR_RIGHT_ID = 2;

    // In meters
    private static final double WHEEL_DIAMETER = 0.1588;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
}

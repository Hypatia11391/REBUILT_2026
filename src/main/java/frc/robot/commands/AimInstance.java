package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AimInstance {

    private Pose2d robotPose;
    private ChassisSpeeds fieldRelativeSpeeds;
    private boolean redTeam;

    public AimInstance(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, boolean redTeam) {
        this.robotPose = robotPose;
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
        this.redTeam = redTeam;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return fieldRelativeSpeeds;
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
    }

    public boolean isRedTeam() {
        return redTeam;
    }    

    //------------------------------------------------------------------//

    private static final double shooterAngle = Math.PI/3;
    private static final double gravity = 9.80665;

    //private static double[] shooterPosition = new double[3];
    private static Pose3d shooterPosition;
    //private static double[] targetPosition = new double[3];
    private static Pose3d targetPosition;
    private static final Pose3d targetPositionRed = new Pose3d(4.611624, 4.034536, 1.8288, new Rotation3d(0,0,0)); 
    private static final Pose3d targetPositionBlue = new Pose3d(11.901424, 4.034536, 1.8288, new Rotation3d(0,0,0));
    private static double[] robotVelocities = new double[2];

    private static final Rotation3d defaultRotation = new Rotation3d(0, 0, 0);

    private static final double shooterSetHeight = 0.381;
    private static double height;

    private static double lastTimeGuess = 1;
    private static final int newtonsMethodIterations = 4;

    public static double rotateBy;
    public static double exitVelocity;

    public static boolean automaticAimControl = false;

    public void updateAim() {
        
        targetPosition = targetPositionBlue;
        if (redTeam)
            targetPosition = targetPositionRed;

        //double[] distanceToTarget = new double[3];
        Translation3d distanceToTarget;
        double time = lastTimeGuess;
        double cotAlpha = 1/Math.tan(shooterAngle);

        updateRobotState(robotPose, fieldRelativeSpeeds);
        // distanceToTarget[0] = targetPosition[0] - shooterPosition[0];
        // distanceToTarget[1] = targetPosition[1] - shooterPosition[1];
        // distanceToTarget[2] = targetPosition[2] - shooterPosition[2];

        distanceToTarget = targetPosition.getTranslation().minus(shooterPosition.getTranslation());

        //distanceToTarget = new Transform3d(targetPosition.getMeasureX(), targetPosition.getMeasureY(), targetPosition.getMeasureZ(), defaultRotation);
        
        
        time = newtonsMethodFunc(distanceToTarget, robotVelocities, time, height, cotAlpha, newtonsMethodIterations);

        double effectiveDistanceX = distanceToTarget.getX() - robotVelocities[0] * time;
        double effectiveDistanceY = distanceToTarget.getY() - robotVelocities[1] * time;
        
        rotateBy = Math.atan2(effectiveDistanceY, effectiveDistanceX);
        exitVelocity = (height + 0.5 * gravity * Math.pow(time, 2)) / (time * Math.sin(shooterAngle));

        lastTimeGuess = time;

    }

    // Newtons method is an iterative function that calculates the time that an object will be in the air. 
    private double newtonsMethodFunc(Translation3d distanceToTarget, double[] robotVelocities, double time, double height, double cotAlpha, int repetitions) {
            for (int i = 0; i < repetitions; i++) {
                double dx = distanceToTarget.getX() - robotVelocities[0] * time;
                double dy = distanceToTarget.getY() - robotVelocities[1] * time;
                double effectiveDistanceMagnitude = Math.sqrt(dx * dx + dy * dy);
                
                double HATeffectiveDistanceX = dx/effectiveDistanceMagnitude;
                double HATeffectiveDistanceY = dy/effectiveDistanceMagnitude;

                double func = cotAlpha * (height + gravity/2 * time * time) - effectiveDistanceMagnitude;
                double funcPrime = cotAlpha * gravity * time - (robotVelocities[0]*HATeffectiveDistanceX + robotVelocities[1]*HATeffectiveDistanceY);

                time = time - func/funcPrime;
            }
            return time;
    }

    private static void updateRobotState(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

        // shooterPosition[0] = robotPose.getX();
        // shooterPosition[1] = robotPose.getY();
        // shooterPosition[2] = shooterSetHeight;

        shooterPosition = new Pose3d(robotPose.getX(), robotPose.getY(), shooterSetHeight, defaultRotation);

        robotVelocities[0] = fieldRelativeSpeeds.vxMetersPerSecond;
        robotVelocities[1] = fieldRelativeSpeeds.vyMetersPerSecond;

        height = targetPosition.getZ() - shooterPosition.getZ();
    }

    public void toggleAutomaticAimControl() {
        automaticAimControl = !automaticAimControl;
    }

    public static double getExitVelocity() {
        return exitVelocity;
    }

    public static double getRequiredRotation() {
        return rotateBy;
    }
    
}

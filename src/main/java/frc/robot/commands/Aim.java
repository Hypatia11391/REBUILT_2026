package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class Aim {

    private static final double shooterAngle = Math.PI/3;
    private static final double gravity = 9.80665;

    private static double[] shooterPosition = new double[3];
    private static double[] targetPosition = new double[3];
    private static final double[] targetPositionRed = {4.611624, 4.034536,1.8288}; 
    private static final double[] targetPositionBlue = {11.901424, 4.034536,1.8288};
    private static final double shooterSetHeight = 0.381;

    private static double[] robotVelocities = new double[2];

    private static double height;

    // Feed this into Newtons method to calculate flight time (Seconds)
    private static double lastTimeGuess = 1;
    private static final int newtonsMethodIterations = 4;

    public static double rotateBy;
    public static double exitVelocity;

    public static boolean automaticAimControl = false;

    public static void updateAim(AimPacket aimPacket) {
        
        targetPosition = targetPositionBlue;
        if (aimPacket.isRedTeam())
            targetPosition = targetPositionRed;

        double[] distanceToTarget = new double[3];
        double time = lastTimeGuess;
        double cotAlpha = 1/Math.tan(shooterAngle);

        updateStuff(aimPacket.getRobotPose(), aimPacket.getFieldRelativeSpeeds());
        distanceToTarget[0] = targetPosition[0] - shooterPosition[0];
        distanceToTarget[1] = targetPosition[1] - shooterPosition[1];
        distanceToTarget[2] = targetPosition[2] - shooterPosition[2];
        
        time = newtonsMethodFunc(distanceToTarget, robotVelocities, time, height, cotAlpha, newtonsMethodIterations);

        double effectiveDistanceX = distanceToTarget[0] - robotVelocities[0] * time;
        double effectiveDistanceY = distanceToTarget[1] - robotVelocities[1] * time;
        
        rotateBy = Math.atan2(effectiveDistanceY, effectiveDistanceX);
        exitVelocity = (height + 0.5 * gravity * Math.pow(time, 2)) / (time * Math.sin(shooterAngle));

        lastTimeGuess = time;    


    }

    private static double newtonsMethodFunc(double[] distanceToTarget, double[] robotVelocities, double time, double height, double cotAlpha, int repetitions) {
            for (int i = 0; i < repetitions; i++) {
                final double gravity = Aim.gravity;
                double dx = distanceToTarget[0] - robotVelocities[0] * time;
                double dy = distanceToTarget[1] - robotVelocities[1] * time;
                double effectiveDistanceMagnitude = Math.sqrt(dx * dx + dy * dy);
                
                double HATeffectiveDistanceX = dx/effectiveDistanceMagnitude;
                double HATeffectiveDistanceY = dy/effectiveDistanceMagnitude;

                double func = cotAlpha * (height + gravity/2 * time * time) - effectiveDistanceMagnitude;
                double funcPrime = cotAlpha * gravity * time - (robotVelocities[0]*HATeffectiveDistanceX + robotVelocities[1]*HATeffectiveDistanceY);

                time = time - func/funcPrime;
            }
            return time;
    }

    private static void updateStuff(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

        shooterPosition[0] = robotPose.getX();
        shooterPosition[1] = robotPose.getY();
        shooterPosition[2] = shooterSetHeight;

        robotVelocities[0] = fieldRelativeSpeeds.vxMetersPerSecond;
        robotVelocities[1] = fieldRelativeSpeeds.vyMetersPerSecond;

        height = targetPosition[2] - shooterPosition[2];
    }

    public static void toggleAutomaticAimControl() {
        automaticAimControl = !automaticAimControl;
    }


}
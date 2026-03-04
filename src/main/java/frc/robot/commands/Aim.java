package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class Aim {
    private static double shooterAngle = Math.PI/3;
    private static double gravity = 9.80665;

    private static double[] shooterPosition = new double[3];
    private static double[] targetPosition = new double[3];
    private static double[] targetPositionRed = {4.611624, 4.034536,1.8288}; //TODO Replace targetPosition with this and measure gameday
    private static double[] targetPositionBlue = {11.901424, 4.034536,1.8288}; //TODO Replace targetPosition with this and measure gameday

    private static double[] robotVelocities = new double[2];


    private static double height;

    public static double rotateBy;
    public static double exitVelocity;

    public static void updateAim(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, double timeGuess, boolean redTeam) {
        targetPosition = targetPositionBlue;
        if (redTeam)
            targetPosition = targetPositionRed;

        double[] distanceToTarget = new double[3];
        double time = timeGuess;
        double cotAlpha = 1/Math.tan(shooterAngle);

        //Functions that do important stuff. Move the order and it all goes to sh!t.
        updateStuff(robotPose, fieldRelativeSpeeds);
        distanceToTarget[0] = targetPosition[0] - shooterPosition[0];
        distanceToTarget[1] = targetPosition[1] - shooterPosition[1];
        distanceToTarget[2] = targetPosition[2] - shooterPosition[2];
        
        time = newtonsMethodFunc(distanceToTarget, robotVelocities, time, height, cotAlpha, 4);

        double effectiveDistanceX = distanceToTarget[0] - robotVelocities[0] * time;
        double effectiveDistanceY = distanceToTarget[1] - robotVelocities[1] * time;
        
        rotateBy = Math.atan2(effectiveDistanceY, effectiveDistanceX);
        exitVelocity = (height + 0.5 * gravity * Math.pow(time, 2)) / (time * Math.sin(shooterAngle));
    }

    private static double newtonsMethodFunc(double[] distanceToTarget, double[] robotVelocities, double time, double height, double cotAlpha, int repetitions) {
            for (int i = repetitions; i > 0; i--) {
                final double gravity = 9.80665;
                double dx = distanceToTarget[0] - robotVelocities[0] * time;
                double dy = distanceToTarget[1] - robotVelocities[1] * time;
                double effectiveDistanceMagnitude = Math.sqrt(dx * dx + dy * dy);
                
                double HATeffectiveDistanceX = dx/effectiveDistanceMagnitude;
                double HATeffectiveDistanceY = dy/effectiveDistanceMagnitude;

                double func = cotAlpha * (height + gravity/2 * time * time) - effectiveDistanceMagnitude;
                double funcPrime = cotAlpha * gravity * time + (robotVelocities[0]*HATeffectiveDistanceX + robotVelocities[1]*HATeffectiveDistanceY);

                time = time - func/funcPrime;
            }
            return time;
    }

    private static void updateStuff(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

        shooterPosition[0] = robotPose.getX();
        shooterPosition[1] = robotPose.getY();
        shooterPosition[2] = 0.381;  //TODO Should be right prob double check

        robotVelocities[0] = fieldRelativeSpeeds.vxMetersPerSecond;
        robotVelocities[1] = fieldRelativeSpeeds.vyMetersPerSecond;

        height = targetPosition[2] - shooterPosition[2];
    }
}
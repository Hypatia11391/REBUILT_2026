package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class Aim {
    double shooterAngle = Math.PI/3;
    double gravity = 9.80665;

    double[] shooterPosition = new double[3];
    double[] targetPosition = new double[3];
    double[] robotVelocities = new double[2];

    double height;

    double rotateBy;
    double exitVelocity;

    public static void updateAim(Aim AimObj, Pose2d robotPose, Translation3d goalLocation, ChassisSpeeds fieldRelativeSpeeds, double timeGuess) {

        double[] distanceToTarget = new double[3];
        double time = timeGuess;
        double cotAlpha = 1/Math.tan(AimObj.shooterAngle);

        //Functions that do important stuff. Move the order and it all goes to sh!t.
        AimObj.updateStuff(robotPose, goalLocation, fieldRelativeSpeeds);
        distanceToTarget[0] = AimObj.targetPosition[0] - AimObj.shooterPosition[0];
        distanceToTarget[1] = AimObj.targetPosition[1] - AimObj.shooterPosition[1];
        distanceToTarget[2] = AimObj.targetPosition[2] - AimObj.shooterPosition[2];
        
        time = newtonsMethodFunc(distanceToTarget, AimObj.robotVelocities, time, AimObj.height, cotAlpha, 4);

        double effectiveDistanceX = distanceToTarget[0] - AimObj.robotVelocities[0] * time;
        double effectiveDistanceY = distanceToTarget[1] - AimObj.robotVelocities[1] * time;
        
        AimObj.rotateBy = Math.atan2(effectiveDistanceY, effectiveDistanceX);
        AimObj.exitVelocity = (AimObj.height + 0.5 * AimObj.gravity * Math.pow(time, 2)) / (time * Math.sin(AimObj.shooterAngle));
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

    private void updateStuff(Pose2d robotPose, Translation3d goalLocation, ChassisSpeeds fieldRelativeSpeeds) {

        this.shooterPosition[0] = robotPose.getX();
        this.shooterPosition[1] = robotPose.getY();
        this.shooterPosition[2] = 0.5;

        this.targetPosition[0] = goalLocation.getX();
        this.targetPosition[1] = goalLocation.getY();
        this.targetPosition[2] = goalLocation.getZ();

        this.robotVelocities[0] = fieldRelativeSpeeds.vxMetersPerSecond;
        this.robotVelocities[1] = fieldRelativeSpeeds.vyMetersPerSecond;

        this.height = this.targetPosition[2] - this.shooterPosition[2];
    }
}
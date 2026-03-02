package frc.robot.commands;

public final class Aim {
    double shooterAngle = Math.PI/3;
    double gravity = 9.80665;

    double[] shooterPosition = new double[3];
    double[] targetPosition = new double[3];
    double[] robotVelocities = new double[2];

    double height = targetPosition[2] - shooterPosition[2];

    double rotateBy;
    double exitVelocity;

    public static void update_aim(Aim AimObj, double timeGuess) {
        double[] distanceToTarget = new double[3];
        double time = timeGuess;
        double cotAlpha = 1/Math.tan(AimObj.shooterAngle);
        distanceToTarget[0] = AimObj.targetPosition[0] - AimObj.shooterPosition[0];
        distanceToTarget[1] = AimObj.targetPosition[1] - AimObj.shooterPosition[1];
        distanceToTarget[2] = AimObj.targetPosition[2] - AimObj.shooterPosition[2];

        time = newtonsMethodFunc(distanceToTarget, AimObj.robotVelocities, time, AimObj.height, cotAlpha, 4);

        double effectiveDistanceX = distanceToTarget[0] - AimObj.robotVelocities[0] * time;
        double effectiveDistanceY = distanceToTarget[1] - AimObj.robotVelocities[1] * time;
        double effectiveDistanceMagnitude = Math.sqrt(Math.pow(effectiveDistanceX, 2) + Math.pow(effectiveDistanceY, 2));
        double HATeffectiveDistanceX = effectiveDistanceX/effectiveDistanceMagnitude;
        double HATeffectiveDistanceY = effectiveDistanceY/effectiveDistanceMagnitude;

        AimObj.rotateBy = Math.atan2(HATeffectiveDistanceY, HATeffectiveDistanceX);
        AimObj.exitVelocity = (AimObj.height + 0.5 * AimObj.gravity * Math.pow(time, 2)) / (time * Math.sin(AimObj.shooterAngle));
        
        AimObj.updateStuff();
    }

    public static double newtonsMethodFunc(double[] distanceToTarget, double[] robotVelocities, double time, double height, double cotAlpha, int repetitions) {
            for (int i = repetitions; i > 0; i--) {
                final double gravity = 9.80665;
                double[] effectiveDistance = new double[2];
                effectiveDistance[0] = distanceToTarget[0] - robotVelocities[0] * time;
                effectiveDistance[1] = distanceToTarget[1] - robotVelocities[1] * time;
                double effectiveDistanceMagnitude = Math.sqrt(Math.pow(effectiveDistance[0],2 ) + Math.pow(effectiveDistance[1], 2));
                double HATeffectiveDistanceX = effectiveDistance[0]/effectiveDistanceMagnitude;
                double HATeffectiveDistanceY = effectiveDistance[1]/effectiveDistanceMagnitude;

                double func = cotAlpha * (height + gravity/2 * time * time) - effectiveDistanceMagnitude;
                double funcPrime = cotAlpha * gravity * time + (robotVelocities[0]*HATeffectiveDistanceX + robotVelocities[1]*HATeffectiveDistanceY);

                time = time - func/funcPrime;
            }
            return time;
    };

    public void updateStuff() {
        /*
        shooterPosition[0] = Do.Stuff.To.Get.Pos;
        shooterPosition[1] = Do.Stuff.To.Get.Pos;
        shooterPosition[2] = Do.Stuff.To.Get.Pos;

        targetPosition[0] = Do.Stuff.To.Get.Pos;
        targetPosition[1] = Do.Stuff.To.Get.Pos;
        targetPosition[2] = Do.Stuff.To.Get.Pos;

        robotVelocities[0] = Do.Stuff.To.Get.Vel;
        robotVelocities[1] = Do.Stuff.To.Get.Vel;

        height = Do.Stuff.To.Get.Height;
        */


    }
}
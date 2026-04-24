package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain.DriveBase;

public class AimInstance {

    private Pose2d robotPose;
    private ChassisSpeeds robotVelocities;
    private boolean redTeam;

    private final double shooterAngle = Math.PI/3;
    private final double gravity = 9.80665;
    private final Rotation3d defaultRotation = new Rotation3d(0, 0, 0);
    private final double shooterSetHeight = 0.381;
    private double lastTimeGuess = 1;
    private final int newtonsMethodIterations = 4;
    private final float overShootConstant = 0.5f;
    private double height;


    private Pose3d shooterPosition;
    private Pose3d targetPosition;
    private final Pose3d targetPositionRed = new Pose3d(4.611624, 4.034536, 1.8288, defaultRotation); 
    private final Pose3d targetPositionBlue = new Pose3d(11.901424, 4.034536, 1.8288, defaultRotation);

    private double rotateBy;
    private double exitVelocity;

    private boolean automaticAimControl = false;

    public AimInstance(Pose2d robotPose, ChassisSpeeds robotVelocities, boolean redTeam) {
        this.robotPose = robotPose;
        this.robotVelocities = robotVelocities;
        this.redTeam = redTeam;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public ChassisSpeeds getRobotVelocities() {
        return robotVelocities;
    }

    public boolean isRedTeam() {
        return redTeam;
    }    

    //------------------------------------------------------------------//

    public void updateAim() {
        
        targetPosition = targetPositionBlue;
        if (redTeam)
            targetPosition = targetPositionRed;

        Translation3d distanceToTarget;
        double time = lastTimeGuess;
        double cotAlpha = 1/Math.tan(shooterAngle);

        updateRobotState(robotPose, robotVelocities);


        distanceToTarget = targetPosition.getTranslation().minus(shooterPosition.getTranslation());
        
        time = newtonsMethodFunc(distanceToTarget, robotVelocities, time, height, cotAlpha, newtonsMethodIterations);

        double effectiveDistanceX = distanceToTarget.getX() - robotVelocities.vxMetersPerSecond * time;
        double effectiveDistanceY = distanceToTarget.getY() - robotVelocities.vyMetersPerSecond * time;
        
        
        rotateBy = Math.atan2(effectiveDistanceY, effectiveDistanceX);
        exitVelocity = (height + 0.5 * gravity * Math.pow(time, 2)) / (time * Math.sin(shooterAngle));
        
        lastTimeGuess = time;

    }

    // Newtons method is an iterative function that calculates the time that an object will be in the air. 
    private double newtonsMethodFunc(Translation3d distanceToTarget, ChassisSpeeds robotVelocities, double time, double height, double cotAlpha, int repetitions) {
            for (int i = 0; i < repetitions; i++) {
                double dx = distanceToTarget.getX() - robotVelocities.vxMetersPerSecond * time;
                double dy = distanceToTarget.getY() - robotVelocities.vyMetersPerSecond * time;
                double effectiveDistanceMagnitude = Math.sqrt(dx * dx + dy * dy);
                
                double HATeffectiveDistanceX = dx/effectiveDistanceMagnitude;
                double HATeffectiveDistanceY = dy/effectiveDistanceMagnitude;

                double func = cotAlpha * (height + gravity/2 * time * time) - effectiveDistanceMagnitude;
                double funcPrime = cotAlpha * gravity * time - (robotVelocities.vxMetersPerSecond*HATeffectiveDistanceX + robotVelocities.vyMetersPerSecond*HATeffectiveDistanceY);

                if (effectiveDistanceMagnitude < 1e-6 || Math.abs(funcPrime) < 1e-9) break; // safegaurd against div by 0

                time = time - func/funcPrime;
            }
            return time;
    }

    public void updateRobotState(Pose2d robotPose, ChassisSpeeds robotVelocities) {

        shooterPosition = new Pose3d(robotPose.getX(), robotPose.getY(), shooterSetHeight, defaultRotation);

        this.robotVelocities = robotVelocities;

        height = targetPosition.getZ() - shooterPosition.getZ();
    }

    public void updateRobotState(DriveBase driveBase) {

        shooterPosition = new Pose3d(DriveBase.getPose2D().getX(), DriveBase.getPose2D().getY(), shooterSetHeight, defaultRotation);

        this.robotVelocities = driveBase.getChassisSpeeds();
    }

    public void toggleAutomaticAimControl() {
        automaticAimControl = !automaticAimControl;
    }

    public double getExitVelocity() {
        return exitVelocity;
    }

    public boolean automaticAimControl() {
        return automaticAimControl;
    }

    public double getRequiredRotation() {
        return rotateBy;
    }
    
    public float getOverShootConstant() {
        return overShootConstant;
    }
}

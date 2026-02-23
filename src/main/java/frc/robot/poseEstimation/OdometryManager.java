package frc.robot.poseEstimation;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.utils.gyro.Navx;

import java.util.logging.Level;
import java.util.logging.Logger;

public class OdometryManager {

    //     -/  \
    // ^+x
    // |   -\  /
    // '-> +y

    private static final double ROOT_TWO = Math.sqrt(2);

    private static final Translation2d FRONT_LEFT_MOVE = new Translation2d(ROOT_TWO,ROOT_TWO);
    private static final Translation2d FRONT_RIGHT_MOVE = new Translation2d(ROOT_TWO,-ROOT_TWO);
    private static final Translation2d REAR_LEFT_MOVE = new Translation2d(ROOT_TWO,-ROOT_TWO);
    private static final Translation2d REAR_RIGHT_MOVE = new Translation2d(ROOT_TWO,ROOT_TWO);

    // TODO: MEASURE
    private static final double WHEEL_DIAMETER = 0.25;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    private static final Pose2d STARTING_POSE = Pose2d.kZero; // TODO: Correct to be the actual starting position!

    private Pose2d currentPose = STARTING_POSE;

    private final Navx navx;

    private final SparkAbsoluteEncoder frEncoder;
    private final SparkAbsoluteEncoder flEncoder;
    private final SparkAbsoluteEncoder rrEncoder;
    private final SparkAbsoluteEncoder rlEncoder;

    public OdometryManager(
        SparkAbsoluteEncoder frEncoder,
        SparkAbsoluteEncoder flEncoder,
        SparkAbsoluteEncoder rrEncoder,
        SparkAbsoluteEncoder rlEncoder,
        Navx navx
    ) {
        this.frEncoder = frEncoder;
        this.flEncoder = flEncoder;
        this.rrEncoder = rrEncoder;
        this.rlEncoder = rlEncoder;
        this.navx = navx;
    }


    public void update() {
        Translation2d vec = FRONT_RIGHT_MOVE.times(frEncoder.getPosition())
            .plus(
                FRONT_LEFT_MOVE.times(flEncoder.getPosition())
            ).plus(
                REAR_LEFT_MOVE.times(rlEncoder.getPosition())
            ).plus(
                REAR_RIGHT_MOVE.times(rrEncoder.getPosition())
            );

        Rotation2d newRotation = new Rotation2d(Angle.ofBaseUnits(navx.getYawDeg(), Units.Degrees));

        vec.rotateBy(newRotation);

        this.currentPose = new Pose2d(
            this.currentPose.getTranslation()
                .plus(vec),
            newRotation
        );
    }

    public ChassisSpeeds getRelativeSpeeds() {
        Translation2d velocity = FRONT_RIGHT_MOVE.times(frEncoder.getVelocity())
            .plus(
                FRONT_LEFT_MOVE.times(flEncoder.getVelocity())
            ).plus(
                REAR_LEFT_MOVE.times(rlEncoder.getVelocity())
            ).plus(
                REAR_RIGHT_MOVE.times(rrEncoder.getVelocity())
            );

        return new ChassisSpeeds(
            velocity.getX(),
            velocity.getY(),
            navx.getRateDegPerSec() * Math.PI / 180
        );
    }

    // in meters
    private static final double SCARY_DIFFERENCE_DISTANCE = 10.0/100.0;

    public void updateFromVision(Vector<N3> newVisionPosition) {
        double xDist = this.currentPose.getTranslation().getX()-newVisionPosition.get(0);
        double yDist = this.currentPose.getTranslation().getY()-newVisionPosition.get(1);
        if(xDist * xDist + yDist * yDist > SCARY_DIFFERENCE_DISTANCE * SCARY_DIFFERENCE_DISTANCE) {
            Logger.getLogger("OdometryManager")
                .log(
                    Level.SEVERE,
                    "Very large difference between odometry position and vision position: " +
                        Math.sqrt(xDist * xDist + yDist * yDist)
                    );
        } else {
            this.currentPose = new Pose2d(
                new Translation2d(
                    newVisionPosition.get(0),
                    newVisionPosition.get(1)
                ),
                this.currentPose.getRotation()
            );
        }
    }

    public void resetPose(Pose2d pose) {
        this.currentPose = pose;
    }

    public Pose2d getPose() {
        return currentPose;
    }
}

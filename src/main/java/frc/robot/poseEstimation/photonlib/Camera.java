package frc.robot.poseEstimation.photonlib;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {

    private final Transform3d cameraRelativeToRobot;
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private boolean hasTargets;
    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;
    private final AprilTagPositions aprilTagPositions = new AprilTagPositions();

    public Camera(Transform3d cameraRelativeToRobot, PhotonCamera camera) {
        this.cameraRelativeToRobot = cameraRelativeToRobot;
        this.camera = camera;
    }

    public void updateCameraResults() {
        latestResult = camera.getLatestResult();
        hasTargets = latestResult.hasTargets();
        targets = latestResult.getTargets();
        bestTarget = latestResult.getBestTarget();
    }

    public Pose3d getBestTargetRobotRelativePose() {
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();

        return new Pose3d().transformBy(cameraToTarget.plus(cameraRelativeToRobot.inverse()));
    }

    public int getBestTargetID() {
        return bestTarget.getFiducialId();
    }

    public double getBestTargetAmbiguity() {
        return bestTarget.getPoseAmbiguity();
    }

    public Pose3d getAprilTagFieldRelativePose(int ID) {
        return new Pose3d(
            aprilTagPositions.getAprilTagPosition(ID)[0],
            aprilTagPositions.getAprilTagPosition(ID)[1],
            aprilTagPositions.getAprilTagPosition(ID)[2],
            new Rotation3d(0,0,0)
        );
    }



}

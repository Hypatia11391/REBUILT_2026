package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {

    private final Transform3d cameraRelativeToRobot;
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private boolean hasTargets;
    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;

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

    public Pose3d getBestTargetPose() {
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();

        return new Pose3d().transformBy(cameraToTarget.plus(cameraRelativeToRobot.inverse()));
    }

}

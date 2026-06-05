package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;

public class PoseHandler {

    private Pose3d robotGlobal;
    private Pose3d backCameraRobot;
    private Pose3d leftCameraRobot;
    private Pose3d rightCameraRobot;
    private Pose3d turret;
    private Pose3d cameraTurret;

    public PoseHandler() {

    }


    public Pose3d getRobotGlobal() {
        return robotGlobal;
    }

    public void setRobotGlobal(Pose3d robotGlobal) {
        this.robotGlobal = robotGlobal;
    }

    public Pose3d getBackCameraRobot() {
        return backCameraRobot;
    }

    public void setBackCameraRobot(Pose3d backCameraRobot) {
        this.backCameraRobot = backCameraRobot;
    }

    public Pose3d getLeftCameraRobot() {
        return leftCameraRobot;
    }

    public void setLeftCameraRobot(Pose3d leftCameraRobot) {
        this.leftCameraRobot = leftCameraRobot;
    }

    public Pose3d getRightCameraRobot() {
        return rightCameraRobot;
    }

    public void setRightCameraRobot(Pose3d rightCameraRobot) {
        this.rightCameraRobot = rightCameraRobot;
    }

    public Pose3d getTurret() {
        return turret;
    }

    public void setTurret(Pose3d turret) {
        this.turret = turret;
    }

    public Pose3d getCameraTurret() {
        return cameraTurret;
    }

    public void setCameraTurret(Pose3d cameraTurret) {
        this.cameraTurret = cameraTurret;
    }

    public Pose3d convertPose(Pose3d relativePose, Pose3d finalPose) {
        return relativePose.relativeTo(finalPose);
    }
        
    
}

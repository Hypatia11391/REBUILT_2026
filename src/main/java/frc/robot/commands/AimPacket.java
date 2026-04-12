package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AimPacket {

    private Pose2d robotPose;
    private ChassisSpeeds fieldRelativeSpeeds;
    private boolean redTeam;

    public AimPacket(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, boolean redTeam) {
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
    
}

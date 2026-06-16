package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;
import frc.robot.subsystems.Mechanisms.Feed;
import frc.robot.subsystems.Mechanisms.Kicker;
import frc.robot.subsystems.Mechanisms.Shooter;
import frc.utils.gyro.Navx;

public final class Autos extends Command {

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command loadPath(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static Command trackTarget(
        DriveBase driveBase,
        Navx navx,
        AimInstance aimInstance
    ) {
        return Commands.run(() -> {
            driveBase.updateAimInstance();
            aimInstance.updateAim();

            double rotationError = MathUtil.angleModulus(
                aimInstance.getRequiredRotation() -
                    navx.getHeading().getRadians()
            );
            double rotationPower = MathUtil.clamp(
                rotationError * aimInstance.getOverShootConstant(),
                -DriveBaseConstants.MAX_SPEED,
                DriveBaseConstants.MAX_SPEED
            );

            driveBase.driveCartesian(
                0.0,
                0.0,
                rotationPower,
                navx.getHeading()
            );
        }, driveBase)
            .until(
                () ->
                    Math.abs(
                        MathUtil.angleModulus(
                            aimInstance.getRequiredRotation() -
                                navx.getHeading().getRadians()
                        )
                    ) < 0.05
            )
            .finallyDo(driveBase::stop)
            .withName("Track Target");
    }

    public static Command shootSequence(
        Shooter shooter,
        Kicker kicker,
        Feed feed,
        double shooterSpeed,
        double timeout
    ) {
        Command sequence = Commands.sequence(
            Commands.waitUntil(() -> shooter.atSpeed()),
            feed.setFeedSpeedCommand(1.0).withTimeout(0.5)
        ); //ignore magic numbers for now
        return sequence.deadlineFor(
            shooter
                .spinUpCommand(shooter.getMaxSpeed(), shooter.getMaxSpeed())
                .withName("Shoot Sequence")
        );
    }
}

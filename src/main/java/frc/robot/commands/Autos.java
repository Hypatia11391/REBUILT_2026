package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Feed;
import frc.robot.subsystems.Mechanisms.Kicker;
import frc.robot.subsystems.Mechanisms.Shooter;

public final class Autos extends Command{

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

  public static Command trackTarget(AimInstance aimInstance) {

    return Commands.none();
  }

  public static Command shootSequence(Shooter shooter, Kicker kicker, Feed feed, double shooterSpeed, double timeout) {
    Command sequence = Commands.sequence(Commands.waitUntil(() -> shooter.atSpeed()), feed.setFeedSpeedCommand(1.0).withTimeout(0.5)); //ignore magic numbers for now
    return sequence.deadlineWith(shooter.spinUpCommand(shooter.getMaxSpeed(), shooter.getMaxSpeed()).withName("Shoot Sequence"));
  }

} 
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

}
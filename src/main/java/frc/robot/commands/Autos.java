package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos extends Command{

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  @Override
  public void initialize() {

  }


  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  public Command loadPath() {

    try {

      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

      return AutoBuilder.followPath(path);

    } catch (Exception e) {
        e.printStackTrace();
        return Commands.none();
    }
  }






}
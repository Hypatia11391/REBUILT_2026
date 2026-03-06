/**
 * This class contains static methods for autonomous commands
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import java.io.File;

//All dis for getting paths from text file in /src/main/deploy/pathplanner
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;


public abstract class Autos {
  public static Command followPath(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);

    } catch (Exception e) {
        e.printStackTrace();
        return Commands.none();
    }
  }

  public static Command runAutoMode(String autoName) {
    return AutoBuilder.buildAuto(autoName);
  }

  public static String[] getPaths(String filePath) {
    try {

        File file = new File(Filesystem.getDeployDirectory(), filePath);
        
        return Files.readAllLines(file.toPath()).toArray(new String[0]);
        
    } catch (IOException e) {
        // System.out.println("Could not read file: " + filePath);
        SmartDashboard.putString("Could not read path file for Auto", filePath);
        return new String[0];
    }
  }

  public static Command autonomousFull(String filePath, RobotContainer container){


    String[] paths = getPaths(filePath);
    List<Command> commandList = new ArrayList<>();
    
    commandList.add(new InstantCommand(
      () -> {
        container.intake.setLiftMotorSpeed(OperateWithJoystick.INTAKE_LIFT_PWR_DOWN);
      },
      container.intake
    ));

    int pathsLength = paths.length;
    
    for (int i = 0; i < (pathsLength-1); i++){
      String pathToRun = paths[i].trim();
      if (i==2) {
        commandList.add(
          new InstantCommand(
            () -> {
              container.intake.setFeedMotorSpeed(OperateWithJoystick.INTAKE_FEED_PWR);
            }, container.intake
          )
        );
      } else if(i==3) {
        commandList.add(
          new InstantCommand(
            () -> {
              container.intake.setFeedMotorSpeed(0);
            }, container.intake
          )
        );
      }
      
      
      if (pathToRun.isEmpty())
        continue;

      try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathToRun);
            commandList.add(AutoBuilder.followPath(path));
        } catch (Exception e) {
            System.out.println("Error loading path: " + pathToRun);
            e.printStackTrace();
        }
    }

    return Commands.sequence(commandList.toArray(new Command[0])).withTimeout(20.0);
    }


}

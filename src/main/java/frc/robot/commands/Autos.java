/**
 * This class contains static methods for autonomous commands
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;


// import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator3d;
// import edu.wpi.first.wpilibj.Filesystem;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.DriveTrain.DriveBase;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerPath;

// import java.io.File;

// import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator3d;

// //All dis for getting paths from text file in /src/main/deploy/pathplanner
// import java.io.IOException;
// import java.nio.file.Files;
// import java.util.ArrayList;
// import java.util.List;

public final class Autos {
    private static final double DISTANCE_TO_MOVE = -0.2;
    private static final double MOVE_SPEED = -0.1;
    public static Pose2d initialPose;

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

//   public static Command followPath(String pathName) {
//     try {
//       PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
//       return AutoBuilder.followPath(path);

//     } catch (Exception e) {
//         e.printStackTrace();
//         return Commands.none();
//     }
//   }

//   public static Command runAutoMode(String autoName) {
//     return AutoBuilder.buildAuto(autoName);
//   }
// /*
// public static String[] getPaths(String filePath) {
//     try {

//         File file = new File(Filesystem.getDeployDirectory(), filePath);

//         return Files.readAllLines(file.toPath()).toArray(new String[0]);

//     } catch (IOException e) {
//         System.out.println("Could not read file: " + filePath);
//         e.printStackTrace();
//         return new String[0];
//     }
//   }
//   */

    public static Command autonomousFull(RobotContainer container) {
        initialPose = container.getEstimatedPose().toPose2d();
        return Commands.sequence(
            Commands.runOnce(() ->
                    container.intake.setFeedMotorSpeed(OperateWithJoystick.INTAKE_FEED_PWR),
                container.intake
            ),
            Commands.deadline(
                Commands.waitUntil(
                    () -> {
                        double targetX = initialPose.getX() + DISTANCE_TO_MOVE;
                        // We're moving backwards so x will get smaller & eventually hit target
                        double currentX = container.getEstimatedPose().getX();
                        SmartDashboard.putNumber("autoXPos",targetX);
                        return currentX - targetX < 0;
                    }
                ),
                Commands.run(
                    () -> {
                        container.getDriveBase().driveCartesian(MOVE_SPEED,0,0,container.getNavx().getHeading());
                    },
                    container.getDriveBase()
                )
            ),
            Commands.runOnce(() ->
                    container.intake.setFeedMotorSpeed(0),
                container.intake
            )
        );
    }






//     String[] paths = getPaths(filePath);
//     int pathsLength = paths.length;

//     List<Command> commandList = new ArrayList<>();

//     for (int i = 0; i < (pathsLength-1); i++){
//       String pathToRun = paths[i]\.trim();
//       if (i==2) {
//         commandList.add(
//           new InstantCommand(
//             () -> {
//               container.intake.setFeedMotorSpeed(OperateWithJoystick.INTAKE_FEED_PWR);
//             }, container.intake
//           )
//         );
//       } else if(i==3) {
//         commandList.add(
//           new InstantCommand(
//             () -> {
//               container.intake.setFeedMotorSpeed(0);
//             }, container.intake
//           )
//         );
//       }


//       if (pathToRun.isEmpty())
//         continue;

//       try {
//             PathPlannerPath path = PathPlannerPath.fromPathFile(pathToRun);
//             commandList.add(AutoBuilder.followPath(path));
//         } catch (Exception e) {
//             System.out.println("Error loading path: " + pathToRun);
//             e.printStackTrace();
//         }
    // }
    // return Commands.sequence(commandList.toArray(new Command[0])).withTimeout(20.0);

    // }

}



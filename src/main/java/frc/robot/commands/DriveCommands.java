package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveCommands {

  public DriveCommands() {
  }

  public Command drive(SuperStructure superStructure, SwerveSubsystem swerve, Pose3d branchPosition) {
    return Commands.run(() -> swerve.goToPoseWithPathfind(branchPosition), swerve);
  }
}

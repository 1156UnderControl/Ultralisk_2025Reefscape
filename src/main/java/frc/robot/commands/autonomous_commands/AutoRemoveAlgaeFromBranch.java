package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoRemoveAlgaeFromBranch extends SequentialCommandGroup {

  public AutoRemoveAlgaeFromBranch(SuperStructure superStructure, SwerveSubsystem swerve,
      TargetBranch branch) {
    addCommands(new InstantCommand(() -> swerve.forceReefPoseEstimation(true)),
        new AutoGoToFaceAndRaiseElevator(swerve, superStructure, branch),
        new InstantCommand(() -> swerve.forceReefPoseEstimation(false)));
  }
}

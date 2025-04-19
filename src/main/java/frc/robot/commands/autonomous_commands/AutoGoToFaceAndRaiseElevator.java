package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToRemovePosition;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoGoToFaceAndRaiseElevator extends SequentialCommandGroup {
  SwerveSubsystem swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public AutoGoToFaceAndRaiseElevator(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    this.targetBranch = branch;
    addCommands(
        new AutoGoToFace(swerve, superStructure, branch),
        new InstantCommand(() -> this.superStructure.scorer.setAutoAlgaeLevel(branch)),
        new MoveScorerToRemovePosition(superStructure)
            .alongWith(new AutoGoToPoseCollectAlgaeFromReef(swerve, superStructure, branch)));
  }
}

package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToRemovePosition;
import frc.robot.constants.SwerveConstants.TargetBranch;

public class AutoPrepareToRemoveAlgae extends SequentialCommandGroup {
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public AutoPrepareToRemoveAlgae(SuperStructure superStructure, TargetBranch branch) {
    this.superStructure = superStructure;
    this.targetBranch = branch;
    addCommands(
        new InstantCommand(() -> this.superStructure.scorer.setAutoAlgaeLevel(branch)),
        new MoveScorerToRemovePosition(superStructure));
  }
}

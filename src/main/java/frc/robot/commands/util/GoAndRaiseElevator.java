package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.OptimizedMoveScorerToScorePosition;
import frc.robot.commands.swerve.SwerveGoToBranch;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoAndRaiseElevator extends SequentialCommandGroup {

  SwerveSubsystem swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public GoAndRaiseElevator(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch,
      boolean forcingDriverController) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    this.targetBranch = branch;
    addCommands(new OptimizedMoveScorerToScorePosition(superStructure, swerve)
        .alongWith(new SwerveGoToBranch(swerve, branch)));
  }
}

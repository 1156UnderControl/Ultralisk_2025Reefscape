package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.OptimizedMoveScorerToScorePosition;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomousOptimizedDirect extends SequentialCommandGroup {

  public AutoScoreCoralAutonomousOptimizedDirect(SuperStructure superStructure, SwerveSubsystem swerve,
      TargetBranch branch) {
    addCommands(new InstantCommand(() -> swerve.forceReefPoseEstimation(true)),
        new SwerveGoToBranchFastAutonomous(swerve, branch, true)
            .alongWith(new OptimizedMoveScorerToScorePosition(superStructure, swerve)),
        Commands.run(() -> superStructure.scorer.placeCoral()).withTimeout(0.3),
        new InstantCommand(() -> swerve.forceReefPoseEstimation(false)));
  }
}

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
        new SwerveGoToBranchFastAutonomous(swerve, branch, superStructure, true)
            .alongWith(new CollectAutonomousOptimizedAfterAutoAlign(superStructure, swerve)
                .andThen(Commands.either(Commands.none(),
                    new OptimizedMoveScorerToScorePosition(superStructure, swerve)
                        .andThen(Commands.run(() -> superStructure.scorer.placeCoral()).withTimeout(0.3)),
                    () -> superStructure.scorer.isTimerAfterAutoAlignFinished()))),
        new InstantCommand(() -> swerve.forceReefPoseEstimation(false)));
  }
}

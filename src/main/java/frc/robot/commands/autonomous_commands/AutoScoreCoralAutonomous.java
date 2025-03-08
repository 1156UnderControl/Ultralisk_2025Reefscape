package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.commands.swerve.SwerveGoToBranch;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomous extends SequentialCommandGroup {

  public AutoScoreCoralAutonomous(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(
        new InstantCommand(() -> swerve.forceReefPoseEstimation(true)),
        new SwerveGoToBranch(swerve, branch, true),
        new MoveScorerToScorePosition(superStructure),
        new SwerveGoToBranch(swerve, branch, false),
        Commands.run(() -> superStructure.scorer.placeCoral()).withTimeout(0.3),
        new SwerveGoToBranch(swerve, branch, true),
        new InstantCommand(() -> swerve.forceReefPoseEstimation(false)));
  }
}

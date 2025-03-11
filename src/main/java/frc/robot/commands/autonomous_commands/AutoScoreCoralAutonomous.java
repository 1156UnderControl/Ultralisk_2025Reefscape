package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.swerve.SwerveGoToBranch;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomous extends SequentialCommandGroup {

  public AutoScoreCoralAutonomous(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(
        new InstantCommand(() -> swerve.forceReefPoseEstimation(true)),
        new SwerveGoToBranch(swerve, branch, true),
        new ConditionalCommand(new AutoScoreElevator(superStructure, swerve, branch),
            null, () -> superStructure.scorer.hasCoral()),
        new InstantCommand(() -> swerve.forceReefPoseEstimation(false)));
  }
}

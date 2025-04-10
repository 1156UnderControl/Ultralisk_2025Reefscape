package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToRemovePosition;
import frc.robot.commands.swerve.SwerveGoToFace;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToFaceAndRaiseElevator extends SequentialCommandGroup {
  SwerveSubsystem swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public GoToFaceAndRaiseElevator(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    this.targetBranch = branch;
    addCommands(
        new SwerveGoToFace(swerve, superStructure, branch), new MoveScorerToRemovePosition(superStructure)
            .alongWith(Commands.run(() -> swerve.driveAlignAngleJoystick(), swerve)));
  }
}

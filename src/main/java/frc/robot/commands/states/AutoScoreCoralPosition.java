package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.commands.swerve.SwerveGoToBranch;
import frc.robot.commands.util.JoystickInterruptible;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public AutoScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(new SwerveGoToBranch(swerve, branch, true), new MoveScorerToScorePosition(superStructure),
        new JoystickInterruptible(new SwerveGoToBranch(swerve, branch, false), 0.2),
        Commands.idle(superStructure)
            .until(operatorKeyboard.scoreCoral().or(() -> swerve.isAtTargetPosition())
                .and(() -> superStructure.scorer.isSecuredToPlaceCoral())),
        Commands.run(() -> superStructure.scorer.placeCoral()));
  }
}

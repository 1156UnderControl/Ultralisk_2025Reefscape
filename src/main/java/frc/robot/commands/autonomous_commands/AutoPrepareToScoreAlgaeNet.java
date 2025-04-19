package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;

public class AutoPrepareToScoreAlgaeNet extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public AutoPrepareToScoreAlgaeNet(SuperStructure superStructure) {
    addCommands(
        new MoveScorerToScorePosition(superStructure).until(() -> superStructure.scorer.isSecuredToScoreOnNet()),
        Commands.idle(superStructure));
  }
}

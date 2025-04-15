package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;

public class AutoScoreAlgae extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public AutoScoreAlgae(SuperStructure superStructure) {
    addCommands(
        Commands.run(() -> superStructure.scorer.placeAlgae(), superStructure).withTimeout(1));
  }
}

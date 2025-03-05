package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToDefaultPositionWithCoral;

public class DefaultPositionWithCoral extends SequentialCommandGroup {

  public DefaultPositionWithCoral(SuperStructure superStructure) {
    addCommands(new MoveScorerToDefaultPositionWithCoral(superStructure),
        Commands.idle(superStructure));
  }
}

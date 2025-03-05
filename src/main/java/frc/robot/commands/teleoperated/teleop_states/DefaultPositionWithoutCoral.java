package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToDefaultPositionWithoutCoral;

public class DefaultPositionWithoutCoral extends SequentialCommandGroup {

  public DefaultPositionWithoutCoral(SuperStructure superStructure) {
    addCommands(new MoveScorerToDefaultPositionWithoutCoral(superStructure),
        Commands.idle(superStructure));
  }
}

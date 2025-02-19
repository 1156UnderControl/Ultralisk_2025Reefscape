package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToDefaultPosition;

public class DefaultPosition extends SequentialCommandGroup {

  public DefaultPosition(SuperStructure superStructure) {
    addCommands(new InstantCommand(() -> superStructure.intake.stopIntake(), superStructure),
        new MoveScorerToDefaultPosition(superStructure),
        Commands.idle(superStructure));
  }
}

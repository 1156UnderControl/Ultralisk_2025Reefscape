package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.climber.StowClimber;
import frc.robot.commands.scorer.MoveScorerToDefaultPosition;

public class DefaultPosition extends SequentialCommandGroup {

  public DefaultPosition(SuperStructure superStructure) {
    addCommands(new StowClimber(superStructure),
        new MoveScorerToDefaultPosition(superStructure),
        new InstantCommand(() -> superStructure.led.setRainbow()),
        Commands.idle(superStructure));
  }
}

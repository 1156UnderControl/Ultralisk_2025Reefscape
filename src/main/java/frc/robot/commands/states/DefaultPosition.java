package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.commands.climber.StowClimber;
import frc.robot.commands.scorer.MoveScorerToDefaultPosition;

public class DefaultPosition extends SequentialCommandGroup {

  public DefaultPosition(SuperStructure superStructure) {
    addCommands(Commands.runOnce(() -> superStructure.led.setSolidColor(LedColor.PURPLE)),
        new StowClimber(superStructure),
        new MoveScorerToDefaultPosition(superStructure),
        Commands.idle(superStructure));
  }
}

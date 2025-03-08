package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;

public class DefaultPositionAutonomous extends SequentialCommandGroup {

  public DefaultPositionAutonomous(SuperStructure superStructure) {
    addCommands(new MoveScorerToDefaultPositionAutonomous(superStructure));
  }
}

package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveScorerToDefaultPositionAutonomous extends Command {
  SuperStructure superStructure;

  public MoveScorerToDefaultPositionAutonomous(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.moveScorerToDefaultPosition();
    ;
  }

  @Override
  public void execute() {
    superStructure.scorer.moveScorerToDefaultPosition();

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class CollectAutonomous extends Command {
  SuperStructure superStructure;

  public CollectAutonomous(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public void execute() {
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.hasCoral();
  }
}

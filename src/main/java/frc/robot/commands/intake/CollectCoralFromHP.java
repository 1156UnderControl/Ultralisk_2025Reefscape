package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class CollectCoralFromHP extends Command {
  SuperStructure superStructure;

  public CollectCoralFromHP(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.intake.intake();
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

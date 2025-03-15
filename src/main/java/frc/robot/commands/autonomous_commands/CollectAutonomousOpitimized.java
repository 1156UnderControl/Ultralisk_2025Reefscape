package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class CollectAutonomousOpitimized extends Command {
  SuperStructure superStructure;

  public CollectAutonomousOpitimized(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    this.superStructure.auto_State = "COLLECTING";
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

  @Override
  public void end(boolean interrupted) {
    superStructure.scorer.stopEndEffector();
  }
}

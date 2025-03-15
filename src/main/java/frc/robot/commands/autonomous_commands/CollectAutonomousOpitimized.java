package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class CollectAutonomousOpitimized extends Command {
  SuperStructure superStructure;
  Timer collectTimer;

  public CollectAutonomousOpitimized(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
    collectTimer = new Timer();
    collectTimer.reset();
  }

  @Override
  public void initialize() {
    this.superStructure.auto_State = "COLLECTING";
    superStructure.scorer.intakeFromHP();
    collectTimer.start();
  }

  @Override
  public void execute() {
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public boolean isFinished() {
    if (collectTimer.get() >= 1) {
      return true;
    }
    return superStructure.scorer.hasCoral();
  }

  @Override
  public void end(boolean interrupted) {
    superStructure.scorer.stopEndEffector();
    collectTimer.stop();
  }
}

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class StowClimber extends Command {
  SuperStructure superStructure;

  public StowClimber(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.climber.unlockClimber();
    superStructure.climber.stopIntakingCage();
  }

  @Override
  public void execute() {
    superStructure.climber.goToStowPosition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

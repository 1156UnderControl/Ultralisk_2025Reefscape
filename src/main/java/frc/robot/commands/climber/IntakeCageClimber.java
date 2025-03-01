package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class IntakeCageClimber extends Command {
  SuperStructure superStructure;

  public IntakeCageClimber(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    superStructure.climber.intakeCage();
  }

  @Override
  public boolean isFinished() {
    return superStructure.climber.isCageCollected();
  }
}

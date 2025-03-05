package frc.robot.commands.teleoperated.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveClimberToIntakePosition extends Command {
  SuperStructure superStructure;

  public MoveClimberToIntakePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.climber.unlockClimber();
  }

  @Override
  public void execute() {
    superStructure.climber.goToIntakeCagePosition();
  }

  @Override
  public boolean isFinished() {
    return superStructure.climber.isAtIntakeCagePosition();
  }
}

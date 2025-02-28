package frc.robot.commands.teleoperated.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveClimberToRaisedPosition extends Command {
  SuperStructure superStructure;

  public MoveClimberToRaisedPosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.climber.raiseClimber();
  }

  @Override
  public void execute() {
    superStructure.climber.raiseClimber();
  }

  @Override
  public boolean isFinished() {
    return superStructure.climber.isCageCollected() && superStructure.climber.isAtSetPoint();
  }
}

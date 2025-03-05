package frc.robot.commands.teleoperated.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class MoveClimberToClimbedPosition extends Command {
  SuperStructure superStructure;

  public MoveClimberToClimbedPosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.climber.lockClimber();
    superStructure.climber.stopIntakingCage();
    superStructure.climber.climb();
  }

  @Override
  public void execute() {
    superStructure.climber.climb();
  }

  @Override
  public boolean isFinished() {
    return superStructure.climber.isAtClimbPosition();
  }
}

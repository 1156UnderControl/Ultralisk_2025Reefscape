package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;

public class StopClimberMotor extends Command {
  SuperStructure superStructure;

  public StopClimberMotor(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.climber.setCoastClimber();
  }

  @Override
  public void execute() {
    superStructure.climber.stopClimberArm();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

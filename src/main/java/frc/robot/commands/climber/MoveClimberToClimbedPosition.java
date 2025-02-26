package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.SuperStructure;

public class MoveClimberToClimbedPosition extends Command {
  SuperStructure superStructure;
  LedSubsystem led = LedSubsystem.getInstance();

  public MoveClimberToClimbedPosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.climber.climb();
  }

  @Override
  public void execute() {
    superStructure.climber.climb();
  }

  @Override
  public void end(boolean interrupted) {
    led.setSolidColor(LedColor.ORANGE);
  }

  @Override
  public boolean isFinished() {
    return superStructure.climber.isAtSetPoint();
  }
}

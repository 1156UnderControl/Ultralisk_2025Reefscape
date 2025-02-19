package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveAlignWithCoralStation extends Command {
  ISwerve swerve;

  public SwerveAlignWithCoralStation(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

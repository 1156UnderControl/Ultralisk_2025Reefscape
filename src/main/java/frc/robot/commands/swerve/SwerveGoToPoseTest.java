package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToPoseTest extends Command {
  SwerveSubsystem swerve;

  public SwerveGoToPoseTest(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.swerve.driveToBranch(TargetBranch.B, false);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPosition();
  }
}

package frc.robot.commands.autonomous_commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoUpdateOdometry extends Command {
  SwerveSubsystem swerve;
  Translation2d defaultPostion;

  public AutoUpdateOdometry(SwerveSubsystem swerve, Translation2d defaultPosition) {
    this.swerve = swerve;
    this.defaultPostion = defaultPosition;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.swerve.resetOdometryLimelight(defaultPostion);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.defaultPositionChanged();
  }

  @Override
  public void end(boolean interrupted) {

  }
}

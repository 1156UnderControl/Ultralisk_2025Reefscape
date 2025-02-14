package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScore extends SequentialCommandGroup {
  SuperStructure superStructure;
  SwerveSubsystem swerve;

  public AutoScore(SuperStructure superStructure, SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.superStructure = superStructure;
  }

}

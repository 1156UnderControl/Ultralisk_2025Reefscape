package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScore extends SequentialCommandGroup {
  SuperStructure superStructure;
  SwerveSubsystem swerve;

  public AutoScore(SuperStructure superStructure, SwerveSubsystem swerve, ReefHeight reefHeight) {
    addCommands(new IntakeCommands().intake(superStructure, swerve),
        new DriveCommands().drive(superStructure, swerve, null),
        new ScorerCommands().score(superStructure, swerve, reefHeight));
  }
}

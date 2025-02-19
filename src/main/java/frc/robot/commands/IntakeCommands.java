package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class IntakeCommands {

  public IntakeCommands() {
  }

  public static Command intake(SuperStructure superStructure, SwerveSubsystem swerve) {
    return Commands.parallel(Commands.parallel(Commands.run(() -> superStructure.intake.intake(), superStructure)
        .andThen(Commands.run(() -> superStructure.scorer.intakeFromHP(), superStructure))),
        Commands.run(() -> swerve.driveAimingToNearestHP(), swerve)).until(() -> superStructure.scorer.hasCoral())
        .finallyDo(() -> superStructure.intake.stopIntake()).finallyDo(() -> superStructure.scorer.stopIntakeFromHP());
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperStructure;
import frc.robot.commands.swerve.SwerveAlignWithCoralStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class IntakeCommands {

  public IntakeCommands() {
  }

  public Command intake(SuperStructure superStructure, SwerveSubsystem swerve) {
    return Commands.parallel(Commands.run(() -> superStructure.intake.intake(), superStructure),
        Commands.run(() -> superStructure.scorer.intakeFromHP(), superStructure),
        new SwerveAlignWithCoralStation(swerve)).until(() -> superStructure.scorer.hasCoral())
        .finallyDo(() -> superStructure.intake.stopIntake()).finallyDo(() -> superStructure.scorer.stopIntakeFromHP());
  }
}

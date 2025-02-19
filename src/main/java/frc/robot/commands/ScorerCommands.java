package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperStructure;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ScorerCommands {

  public ScorerCommands() {
  }

  public static Command score(SuperStructure superStructure, SwerveSubsystem swerve, ReefHeight reefHeight) {
    return Commands.parallel(Commands.run(() -> superStructure.scorer.setTargetBranch(reefHeight), superStructure),
        Commands.run(() -> superStructure.scorer.setElevatorDutyCycle(-1), superStructure))
        .until(() -> !superStructure.scorer.hasCoral())
        .finallyDo(() -> superStructure.scorer.stopIntakeFromHP());
  }
}

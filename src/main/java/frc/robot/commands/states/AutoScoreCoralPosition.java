package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.commands.util.GoAndRaiseElevator;
import frc.robot.commands.util.GoToFaceAndRaiseElevator;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.IDriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();
  IDriverController driverController = DriverController.getInstance();
  boolean hasCancelledAutoMove = false;

  public AutoScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(new InstantCommand(() -> hasCancelledAutoMove = false),
        new GoAndRaiseElevator(swerve, superStructure, branch)
            .until(driverController.isForcingDriverControl().or(() -> {
              if (operatorKeyboard.scoreObject().getAsBoolean()) {
                hasCancelledAutoMove = true;
              }
              return hasCancelledAutoMove;
            })),
        new ParallelRaceGroup(new MoveScorerToScorePosition(superStructure)
            .until(operatorKeyboard.scoreObject()
                .or(() -> swerve.isAtTargetPositionWithoutHeading() && superStructure.scorer.isScorerAtPosition())
                .or(() -> hasCancelledAutoMove)),
            Commands.run(() -> swerve.driveAlignAngleJoystick(), swerve)),
        Commands.run(() -> superStructure.scorer.placeCoral()).withTimeout(Seconds.of(0.3)),
        Commands.idle(superStructure).alongWith(Commands.run(() -> swerve.driveAlignAngleJoystick(), swerve))
            .until(operatorKeyboard.removeAlgaeFromBranch()),
        new GoToFaceAndRaiseElevator(swerve, superStructure, branch)
            .onlyIf(() -> !superStructure.scorer.isAlgaeManualControl()));
  }
}

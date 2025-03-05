// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Java_Is_UnderControl.LEDs.ILed;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.commands.teleoperated.teleop_states.AutoScoreCoralPosition;
import frc.robot.commands.teleoperated.teleop_states.ClimbPosition;
import frc.robot.commands.teleoperated.teleop_states.CollectPosition;
import frc.robot.commands.teleoperated.teleop_states.DefaultPositionWithCoral;
import frc.robot.commands.teleoperated.teleop_states.DefaultPositionWithoutCoral;
import frc.robot.commands.teleoperated.teleop_states.RemoveAlgaePosition;
import frc.robot.commands.teleoperated.teleop_states.ScoreCoralPosition;
import frc.robot.constants.FieldConstants.AlgaeHeight;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private OperatorController keyBoard = OperatorController.getInstance();

  private DriverController driverController = DriverController.getInstance();

  public final SuperStructure superStructure = new SuperStructure();

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(() -> superStructure.scorer.isElevatorInHighPosition(),
      () -> superStructure.scorer.getTargetReefLevel(),
      TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public final ILed leds = LedSubsystem.getInstance();

  public RobotContainer() {
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
    NamedCommands.registerCommand("Intake Coral", new CollectPosition(superStructure, drivetrain));
    NamedCommands.registerCommand("Score Coral A",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.A));
    NamedCommands.registerCommand("Score Coral B",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.B));
    NamedCommands.registerCommand("Score Coral C",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.C));
    NamedCommands.registerCommand("Score Coral D",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.D));
    NamedCommands.registerCommand("Score Coral E",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.E));
    NamedCommands.registerCommand("Score Coral F",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.F));
    NamedCommands.registerCommand("Score Coral G",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.G));
    NamedCommands.registerCommand("Score Coral H",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.H));
    NamedCommands.registerCommand("Score Coral I",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.I));
    NamedCommands.registerCommand("Score Coral J",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.J));
    NamedCommands.registerCommand("Score Coral K",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.K));
    NamedCommands.registerCommand("Score Coral L",
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.L));

    NamedCommands.registerCommand("Set Coral Level L1",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L1)));
    NamedCommands.registerCommand("Set Coral Level L2",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L2)));
    NamedCommands.registerCommand("Set Coral Level L3",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L3)));
    NamedCommands.registerCommand("Set Coral Level L4",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L4)));
    superStructure
        .setDefaultCommand(Commands.either(new DefaultPositionWithCoral(superStructure),
            new DefaultPositionWithoutCoral(superStructure), () -> superStructure.scorer.hasCoral()));
    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoystick(), drivetrain)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));
  }

  private void configureBindings() {

    // driverController.rightBumper().onTrue(new
    // AutoIntakeCoralPosition(superStructure, drivetrain));

    keyBoard.collectCoral()
        .onTrue(new CollectPosition(superStructure, drivetrain));

    keyBoard.prepareToScoreCoral().onTrue(new ScoreCoralPosition(superStructure, drivetrain));

    driverController.x().whileTrue(
        Commands.runEnd(() -> superStructure.scorer.setCoastScorer(), () -> superStructure.scorer.setBrakeScorer())
            .ignoringDisable(true));

    keyBoard.reefL1()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L1);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeight.LOW);
        }));

    keyBoard.reefL2()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L2);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeight.MID);
        }));

    keyBoard.reefL3()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L3)));

    keyBoard.reefL4()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L4)));

    keyBoard.removeAlgaeFromBranch()
        .onTrue(new RemoveAlgaePosition(superStructure, drivetrain)
            .deadlineFor(Commands.run(() -> drivetrain.driveAlignAngleJoystickRemoveAlgae(), drivetrain)));

    keyBoard.alignToClimb().onTrue(new ClimbPosition(superStructure)
        .deadlineFor(Commands.runOnce(() -> drivetrain.setAngleForClimb())
            .andThen(Commands.run(() -> drivetrain.driveLockedAngleToClimb(), drivetrain)
                .until(() -> superStructure.robotIsClimbed)
                .andThen(Commands.run(() -> drivetrain.stopSwerve(), drivetrain)))));

    keyBoard.cancelAction().onTrue(new DefaultPositionWithCoral(superStructure));

    keyBoard.goToReefA().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.A));

    keyBoard.goToReefB().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.B));

    keyBoard.goToReefC().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.C));

    keyBoard.goToReefD().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.D));

    keyBoard.goToReefE().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.E));

    keyBoard.goToReefF().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.F));

    keyBoard.goToReefG().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.G));

    keyBoard.goToReefH().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.H));

    keyBoard.goToReefI().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.I));

    keyBoard.goToReefJ().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.J));

    keyBoard.goToReefK().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.K));

    keyBoard.goToReefL().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.L));

    driverController.x().and(() -> DriverStation.isDisabled())
        .whileTrue(Commands.runEnd(() -> superStructure.setCoastToRobot(), () -> superStructure.setBrakeToRobot())
            .ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

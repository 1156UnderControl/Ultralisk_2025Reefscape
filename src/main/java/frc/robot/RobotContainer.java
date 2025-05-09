// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.scorer.ScoreProcessor;
import frc.robot.commands.states.AutoScoreCoralPosition;
import frc.robot.commands.states.ClimbPosition;
import frc.robot.commands.states.CollectPosition;
import frc.robot.commands.states.DefaultPosition;
import frc.robot.commands.states.RemoveAlgaePosition;
import frc.robot.commands.states.ScoreObjectPosition;
import frc.robot.commands.swerve.SwerveAngleWithCoralStation;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
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

  private SwerveModuleConstants[] modulesArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(() -> superStructure.scorer.isElevatorInHighPosition(),
      () -> superStructure.scorer.getTargetReefLevel(), () -> superStructure.scorer.getTargetReefLevelAlgae(),
      TunerConstants.getSwerveDrivetrainConstants(),
      modulesArray[0], modulesArray[1], modulesArray[2], modulesArray[3]);

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  private NamedCommandsRegistry namedCommandsRegistry;

  public RobotContainer() {
    setNamedCommandsForAuto();
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
    superStructure
        .setDefaultCommand(new DefaultPosition(superStructure));
    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoystick(), drivetrain)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));
    this.namedCommandsRegistry = new NamedCommandsRegistry(drivetrain, superStructure);
  }

  private void setNamedCommandsForAuto() {
    this.namedCommandsRegistry.registerAllAutoCommands();
  }

  private void configureBindings() {

    keyBoard.collectCoral().onTrue(
        new CollectPosition(superStructure, drivetrain).deadlineFor(new SwerveAngleWithCoralStation(drivetrain)));

    keyBoard.prepareToScore().onTrue(new ScoreObjectPosition(superStructure));

    keyBoard.scoreObject().and(() -> superStructure.scorer.readyToScoreProcessor())
        .onTrue(new ScoreProcessor(superStructure));

    keyBoard.reefL1()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L1);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.GROUND);
          this.superStructure.scorer.setAlgaeManualControl(true);
        }));

    keyBoard.reefL2()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L2);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.LOW);
          this.superStructure.scorer.setAlgaeManualControl(true);
        }));

    keyBoard.reefL3()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L3);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.MID);
          this.superStructure.scorer.setAlgaeManualControl(true);
        }));

    keyBoard.reefL4()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L4)));

    keyBoard.removeAlgaeFromBranch().and(() -> this.superStructure.scorer.isAlgaeManualControl())
        .onTrue(new RemoveAlgaePosition(superStructure, drivetrain)
            .deadlineFor(Commands.run(() -> drivetrain.driveAlignAngleJoystickRemoveAlgae(), drivetrain)));

    keyBoard.alignToClimb().onTrue(new ClimbPosition(superStructure)
        .deadlineFor(Commands.runOnce(() -> drivetrain.setAngleForClimb())
            .andThen(Commands.run(() -> drivetrain.driveLockedAngleToClimb(), drivetrain)
                .until(() -> superStructure.robotIsClimbed)
                .andThen(Commands.run(() -> drivetrain.stopSwerve(), drivetrain)))));

    keyBoard.cancelAction().onTrue(new DefaultPosition(superStructure));

    keyBoard.goToReefA().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.A)
        .alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.A);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefB().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.B)
        .alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.B);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefC().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.C).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.C);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefD().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.D).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.D);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefE().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.E).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.E);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefF().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.F).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.F);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefG().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.G).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.G);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefH().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.H).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.H);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefI().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.I).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.I);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefJ().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.J).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.J);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefK().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.K).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.K);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefL().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.L).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.L);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    driverController.y().onTrue(Commands.runOnce(() -> superStructure.scorer.overrideHasAlgae()));

    driverController.a().onTrue(Commands.runOnce(() -> superStructure.scorer.overrideHasCoral()));

    driverController.b().onTrue(Commands.runOnce(() -> superStructure.scorer.overrideNoObject()));

    driverController.start().onTrue(
        new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0))).ignoringDisable(true));

    driverController.x().and(() -> DriverStation.isDisabled()).whileTrue(Commands
        .runEnd(() -> superStructure.setCoastToRobot(), () -> superStructure.setBrakeToRobot()).ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

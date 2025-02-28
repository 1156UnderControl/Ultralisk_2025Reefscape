// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.states.CollectPosition;
import frc.robot.commands.states.DefaultPosition;
import frc.robot.commands.states.RemoveAlgaePosition;
import frc.robot.commands.states.ScoreCoralPosition;
import frc.robot.constants.FieldConstants.AlgaeHeight;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private OperatorController keyBoard = OperatorController.getInstance();

  private DriverController driverController = DriverController.getInstance();

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public final SuperStructure superStructure = new SuperStructure();

  public RobotContainer() {
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
    superStructure.setDefaultCommand(new DefaultPosition(superStructure));
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoy(), drivetrain).onlyIf(() -> DriverStation.isTeleopEnabled()));

    driverController.rightBumper().whileTrue(drivetrain.wheelRadiusCharacterization());

    keyBoard.collectCoral().onTrue(new CollectPosition(superStructure, drivetrain));

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
        .onTrue(new RemoveAlgaePosition(superStructure, drivetrain));

    keyBoard.cancelAction().onTrue(new DefaultPosition(superStructure));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

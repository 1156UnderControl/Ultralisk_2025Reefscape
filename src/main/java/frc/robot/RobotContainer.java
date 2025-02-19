// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.robot.commands.states.DefaultPosition;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private OperatorController controller = OperatorController.getInstance();

  private OperatorController operatorPanel = OperatorController.getInstance();

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public final SuperStructure superStructure = new SuperStructure();

  private ReefHeight reefLevel = ReefHeight.L4;

  public RobotContainer() {
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
    superStructure.setDefaultCommand(new DefaultPosition(superStructure));
  }

  private void configureBindings() {
    Pose3d posebranch1Score = CoordinatesTransform
        .getRetreatPose(AllianceFlipUtil.apply(Reef.branchPositions.get(1).get(ReefHeight.L2)), 1.0);
    Pose3d posebranch7Score = CoordinatesTransform
        .getRetreatPose(AllianceFlipUtil.apply(Reef.branchPositions.get(7).get(ReefHeight.L2)), 1.0);

    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoy(), drivetrain).onlyIf(() -> DriverStation.isTeleopEnabled()));

    // Left Reef Positions
    controller.goToReefA()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(0).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/A", Commands.waitSeconds(1));

    controller.goToReefL()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(1).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/L", Commands.waitSeconds(1));

    controller.goToReefK()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(2).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/K", Commands.waitSeconds(1));

    controller.goToReefJ()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(3).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/J", Commands.waitSeconds(1));

    controller.goToReefI()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(4).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/I", Commands.waitSeconds(1));

    controller.goToReefH()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(5).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/H", Commands.waitSeconds(1));

    controller.goToReefB()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(0).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/B", Commands.waitSeconds(1));

    controller.goToReefC()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(1).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/C", Commands.waitSeconds(1));

    controller.goToReefD()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(2).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/D", Commands.waitSeconds(1));

    controller.goToReefE()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(3).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/E", Commands.waitSeconds(1));

    controller.goToReefF()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(4).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/F", Commands.waitSeconds(1));

    controller.goToReefG()
        .whileTrue(
            drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions.get(5).get(reefLevel).toPose2d()));
    NamedCommands.registerCommand("score/collect/G", Commands.waitSeconds(1));

    if (controller.reefL1().getAsBoolean()) {
      reefLevel = ReefHeight.L1;
    }

    if (controller.reefL2().getAsBoolean()) {
      reefLevel = ReefHeight.L2;
    }

    if (controller.reefL3().getAsBoolean()) {
      reefLevel = ReefHeight.L3;
    }

    if (controller.reefL4().getAsBoolean()) {
      reefLevel = ReefHeight.L4;
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

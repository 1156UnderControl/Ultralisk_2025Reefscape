// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private OperatorController controller = OperatorController.getInstance();

  // private final CommandXboxController joystick = new CommandXboxController(0);

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public RobotContainer() {
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoy(), drivetrain).onlyIf(() -> DriverStation.isTeleopEnabled()));

    // Left Reef Positions
    controller.goToReefA()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions[0]));
    NamedCommands.registerCommand("score/collect/A", Commands.waitSeconds(1));

    controller.goToReefL()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions[1]));
    NamedCommands.registerCommand("score/collect/L", Commands.waitSeconds(1));

    controller.goToReefK()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions[2]));
    NamedCommands.registerCommand("score/collect/K", Commands.waitSeconds(1));

    controller.goToReefJ()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions[3]));
    NamedCommands.registerCommand("score/collect/J", Commands.waitSeconds(1));

    controller.goToReefI()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions[4]));
    NamedCommands.registerCommand("score/collect/I", Commands.waitSeconds(1));

    controller.goToReefH()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.branchPositions[5]));
    NamedCommands.registerCommand("score/collect/H", Commands.waitSeconds(1));

    // Right Reef Positions
    controller.goToReefB()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.rightCenterFaces[0]));
    NamedCommands.registerCommand("score/collect/B", Commands.waitSeconds(1));

    controller.goToReefC()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.rightCenterFaces[1]));
    NamedCommands.registerCommand("score/collect/C", Commands.waitSeconds(1));

    controller.goToReefD()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.rightCenterFaces[2]));
    NamedCommands.registerCommand("score/collect/D", Commands.waitSeconds(1));

    controller.goToReefE()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.rightCenterFaces[3]));
    NamedCommands.registerCommand("score/collect/E", Commands.waitSeconds(1));

    controller.goToReefF()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.rightCenterFaces[4]));
    NamedCommands.registerCommand("score/collect/F", Commands.waitSeconds(1));

    controller.goToReefG()
        .whileTrue(drivetrain.goToPoseWithPathfind(FieldConstants.Reef.rightCenterFaces[5]));
    NamedCommands.registerCommand("score/collect/G", Commands.waitSeconds(1));
    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

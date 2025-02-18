// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.joysticks.ControlBoard;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private ControlBoard driverController = ControlBoard.getInstance();
  private OperatorController operatorPanel = OperatorController.getInstance();

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public final SuperStructure superStructure = new SuperStructure();

  public RobotContainer() {
    // superStructure.setDefaultCommand(Commands.run(() ->
    // superStructure.scorer.setElevatorDutyCycle(0), superStructure));
    // superStructure.setDefaultCommand(new DefaultPosition(superStructure));
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  private void configureBindings() {
    driverController.b().and(() -> !superStructure.scorer.hasCoral())
        .whileTrue(Commands.runEnd(() -> {
          superStructure.scorer.intakeFromHP();
          superStructure.intake.intake();
        },
            () -> {
              superStructure.scorer.stopIntakeFromHP();
              superStructure.intake.stopIntake();
            }, superStructure));

    driverController.a().and(() -> !superStructure.scorer.hasCoral())
        .onTrue(
            Commands.runEnd(
                () -> superStructure.scorer.setPivotTestPosition(12),
                () -> superStructure.scorer.setPivotDutyCycle(0)))
        .and(
            () -> superStructure.scorer.isAtCollectPosition());
    driverController.y()
        .whileTrue(Commands.runEnd(
            () -> superStructure.scorer.setPivotTestPosition(205),
            () -> superStructure.scorer.setPivotDutyCycle(0), superStructure));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

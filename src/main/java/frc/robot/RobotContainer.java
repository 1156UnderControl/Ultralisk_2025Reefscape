// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Java_Is_UnderControl.Joysticks.Types.TypeXboxController;
import frc.robot.commands.Go100Angle;
import frc.robot.commands.StopMotorCmd;
import frc.robot.test_subsystems.ArmSubsystem;

public class RobotContainer {

  TypeXboxController controllerType;
  ArmSubsystem armSubsystem;

  public RobotContainer() {
    armSubsystem = ArmSubsystem.getInstance();
    controllerType = new TypeXboxController(0);
    armSubsystem.setDefaultCommand(new StopMotorCmd(armSubsystem));
    configureBindings();
  }

  private void configureBindings() {
    controllerType.buttomUp().whileTrue(this.armSubsystem.sysIdQuasistaticForward().until(armSubsystem.atLimitForward()));
    controllerType.buttomDown().whileTrue(this.armSubsystem.sysIdQuasistaticReverse().until(armSubsystem.atLimitReverse()));
    controllerType.buttomRight().whileTrue(this.armSubsystem.sysIdDynamicForward().until(armSubsystem.atLimitForward()));
    controllerType.buttomLeft().whileTrue(this.armSubsystem.sysIdDynamicReverse().until(armSubsystem.atLimitReverse()));
    controllerType.dPadRight().onTrue(new Go100Angle(armSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

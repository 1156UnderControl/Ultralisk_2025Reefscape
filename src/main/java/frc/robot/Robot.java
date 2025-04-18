// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.robot.joysticks.OperatorController;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private boolean autoHasExecuted = false;

  OperatorController controller = OperatorController.getInstance();

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start("", "", 0.01);
    DriverStation.startDataLog(DataLogManager.getLog(), true);
  }

  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

    if (!this.autoHasExecuted) {
      if (LimelightHelpers.getTV("limelight-left")) {
        this.m_robotContainer.superStructure.led.setSolidColor(LedColor.GREEN);
        return;
      }
      this.m_robotContainer.superStructure.led.setSolidColor(LedColor.RED);
      return;
    }

    this.m_robotContainer.superStructure.led.setSolidColor(LedColor.BLUE);
    return;
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    this.autoHasExecuted = true;
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}

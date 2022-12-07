// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.lang.Math;
  
public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;

  public DriveDistance(double distance, Drivetrain drive) {
    m_distance = distance;
    m_drive = drive;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_drive.driveDistance(m_distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getLeftDistanceInch() - m_distance) <= 0.5 && Math.abs(m_drive.getRightDistanceInch() - m_distance) <= 0.5;
  }
}

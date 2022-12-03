// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_CAN_ID, MotorType.kBrushless);
  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_CAN_ID, MotorType.kBrushless);
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_CAN_ID, MotorType.kBrushless);
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_CAN_ID, MotorType.kBrushless);
  /** Creates a new Drivetrain. */
  public Drivetrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

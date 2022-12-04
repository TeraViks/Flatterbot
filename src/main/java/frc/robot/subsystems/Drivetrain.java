// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.lang.Math;

public class Drivetrain extends SubsystemBase {
  // Creates motor objects
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_CAN_ID, MotorType.kBrushless);
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_CAN_ID, MotorType.kBrushless);

  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_CAN_ID, MotorType.kBrushless);
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_CAN_ID, MotorType.kBrushless);
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    resetEncoders();
    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftFrontMotor.set(RobotContainer.getJoyX());
    leftBackMotor.set(RobotContainer.getJoyX());

    rightFrontMotor.set(RobotContainer.getJoyX());
    rightBackMotor.set(RobotContainer.getJoyX());

  }

  public final void resetEncoders() {
    leftFrontMotor.getEncoder().setPosition(0.0);
    leftBackMotor.getEncoder().setPosition(0.0);
    rightFrontMotor.getEncoder().setPosition(0.0);
    rightBackMotor.getEncoder().setPosition(0.0);
  }

  public final double getLeftEncoderPos() {
    return (leftFrontMotor.getEncoder().getPosition() + leftBackMotor.getEncoder().getPosition()) / 2;
  }
  
  public final double getRightEncoderPos() {
    return (rightFrontMotor.getEncoder().getPosition() + rightBackMotor.getEncoder().getPosition()) / 2;
  }

  public final double getAverageEncoderPos() {
    return (getRightEncoderPos() + getLeftEncoderPos()) / 2;
  }

  public final double getLeftEncoderVel() {
    return (leftFrontMotor.getEncoder().getVelocity() + leftBackMotor.getEncoder().getVelocity()) / 2;
  }

  public final double getRightEncoderVel() {
    return (rightFrontMotor.getEncoder().getVelocity() + rightBackMotor.getEncoder().getVelocity()) / 2;
  }

  public final double getAverageEncoderVel() {
    return (getRightEncoderVel() + getLeftEncoderVel()) / 2;
  }

  public final double getLeftDistanceInch() {
    return Constants.WHEEL_DIAMETER * Math.PI * getLeftEncoderPos() / Constants.TICK_P_ROT;
  }

  public final double getRightDistanceInch() {
    return Constants.WHEEL_DIAMETER * Math.PI * getRightEncoderPos() / Constants.TICK_P_ROT;
  }

  public final double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2;
  }
  
}


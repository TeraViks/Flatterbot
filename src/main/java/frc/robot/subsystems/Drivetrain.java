// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import java.lang.Math;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax.IdleMode;

public class Drivetrain extends SubsystemBase {
  // Creates Motor Objects
  CANSparkMax m_leftLeader = new CANSparkMax(Constants.LEFT_FRONT_CAN_ID, MotorType.kBrushless);
  CANSparkMax m_leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_CAN_ID, MotorType.kBrushless);
  CANSparkMax m_rightLeader = new CANSparkMax(Constants.RIGHT_FRONT_CAN_ID, MotorType.kBrushless);
  CANSparkMax m_rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_CAN_ID, MotorType.kBrushless);

  // Creates Encoder Objects
  RelativeEncoder m_leftFrontEncoder = m_leftLeader.getEncoder();
  RelativeEncoder m_leftBackEncoder = m_leftBackMotor.getEncoder();
  RelativeEncoder m_rightFrontEncoder = m_rightLeader.getEncoder();
  RelativeEncoder m_rightBackEncoder = m_rightBackMotor.getEncoder();

  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightLeader, m_rightBackMotor);
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftLeader, m_leftBackMotor);
  
  
  
  private final DifferentialDrive diffDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_rightLeader.setInverted(true);
    m_rightBackMotor.setInverted(true);

    m_leftLeader.setIdleMode(IdleMode.kCoast);
    m_leftBackMotor.setIdleMode(IdleMode.kCoast);
    m_rightLeader.setIdleMode(IdleMode.kCoast);
    m_rightBackMotor.setIdleMode(IdleMode.kCoast);
    resetEncoders();
    
    m_leftBackMotor.follow(m_leftLeader, false);
    m_rightBackMotor.follow(m_rightLeader, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    diffDrive.arcadeDrive(
      RobotContainer.getJoyX(),
      RobotContainer.getJoyY()*Constants.SPEED_FACTOR
      );
  }

  private final double revToInch(double revolutions){
    return Constants.WHEEL_DIAMETER * Math.PI * revolutions / Constants.MOTOR_WHEEL_GEAR_RATIO;
  }

  private final double inchToRev(double inches) {
    return Constants.WHEEL_DIAMETER * Math.PI / inches * Constants.MOTOR_WHEEL_GEAR_RATIO;
  }

  public final void resetEncoders() {
    m_leftFrontEncoder.setPosition(0.0);
    m_leftBackEncoder.setPosition(0.0);
    m_rightFrontEncoder.setPosition(0.0);
    m_rightBackEncoder.setPosition(0.0);
  }

  public final double getLeftEncoderPos() {
    return (m_leftFrontEncoder.getPosition() + m_leftBackEncoder.getPosition()) / 2;
  }
  
  public final double getRightEncoderPos() {
    return (m_rightFrontEncoder.getPosition() + m_rightBackEncoder.getPosition()) / 2;
  }

  public final double getAverageEncoderPos() {
    return (getRightEncoderPos() + getLeftEncoderPos()) / 2;
  }

  public final double getLeftEncoderVel() {
    return (m_leftFrontEncoder.getVelocity() + m_leftBackEncoder.getVelocity()) / 2;
  }

  public final double getRightEncoderVel() {
    return (m_rightFrontEncoder.getVelocity() + m_rightBackEncoder.getVelocity()) / 2;
  }

  public final double getAverageEncoderVel() {
    return (getRightEncoderVel() + getLeftEncoderVel()) / 2;
  }

  public final double getLeftDistanceInch() {
    return revToInch(getLeftEncoderPos());
  }

  public final double getRightDistanceInch() {
    return revToInch(getRightEncoderPos());
  }

  public final double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2;
  }

  public final void driveDistance(double inches) {
    double distance = inchToRev(inches);
    m_leftLeader.getPIDController().setReference(distance, ControlType.kPosition);
    m_rightLeader.getPIDController().setReference(distance, ControlType.kPosition);
  }
  
}


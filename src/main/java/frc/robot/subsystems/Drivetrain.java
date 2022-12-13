// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.lang.Math;

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

  // Creates the MotorControllerGroup objects used in teleop for arcadeDrive()
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightLeader, m_rightBackMotor);
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftLeader, m_leftBackMotor);
  
  
  private final DifferentialDrive diffDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  // Creates a new Drivetrain object and sets settings
  public Drivetrain() {
    diffDrive.setDeadband(Constants.DEADBAND_SIZE);
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

  // This method will be called once per teleop run
  @Override
  public void periodic() {
    double y = RobotContainer.getJoyY();
    double x = RobotContainer.getJoyX();
    diffDrive.arcadeDrive(
      -y * Constants.SPEED_FACTOR,
      x * Constants.SPEED_FACTOR,
      true
      );
  }

  private final double revToInch(double revolutions){
    double ret = revolutions * Constants.WHEEL_DIAMETER * Math.PI / Constants.MOTOR_WHEEL_GEAR_RATIO;
    return ret;
  }


  private final double inchToRev(double inches) {
    double ret = inches * Constants.MOTOR_WHEEL_GEAR_RATIO / (Constants.WHEEL_DIAMETER * Math.PI);
    // System.out.printf("inchToRev: (%f)->(%f)", inches, ret);
    return ret;
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
    double revs = inchToRev(inches);
    m_leftLeader.getPIDController().setOutputRange(-0.5, 0.5);
    m_rightLeader.getPIDController().setOutputRange(-0.5, 0.5);
    m_leftLeader.getPIDController().setP(0.5);
    m_rightLeader.getPIDController().setP(0.5);

    m_leftLeader.getPIDController().setReference(revs, ControlType.kPosition);
    m_rightLeader.getPIDController().setReference(revs, ControlType.kPosition);
    System.out.println("finished");
    System.out.println(inches);
    System.out.println(revs);
    System.out.println(revToInch(revs));
  }
  
}

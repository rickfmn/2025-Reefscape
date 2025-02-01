// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {

  public SparkMax m_angleMotor = new SparkMax(AlgaeIntakeConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetAngleMotor(double voltage){
    m_angleMotor.setVoltage(voltage);
  }

  
}

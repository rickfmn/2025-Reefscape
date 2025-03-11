// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.SignalLights.LightSignal;

public class AlgaeIntake extends SubsystemBase {

  public static enum IntakeState{
    Deployed,
    Deploying,
    Retracted,
    Retracting
  }

  private SparkFlex m_intakeMotor = new SparkFlex(AlgaeIntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private SparkMax m_angleMotor = new SparkMax(AlgaeIntakeConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
  private SparkLimitSwitch angleLimitSwitch = m_angleMotor.getReverseLimitSwitch();
  private RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();
  public IntakeState currentState = IntakeState.Retracted;
  public SignalLights signalLights;
  private boolean isIntaking = false;


  /** Creates a new AlgaeIntake. */
  public AlgaeIntake(SignalLights lights) {
    signalLights = lights;
    m_angleEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    signalLights.ReceiveIntakeData(hasAlgae());

    switch (currentState) {
      case Deployed:
        SetAngleMotor(0);

        break;
      case Deploying:
        SetAngleMotor(AlgaeIntakeConstants.kDEPLOY_SPEED);
        if(m_angleEncoder.getPosition() > AlgaeIntakeConstants.INTAKE_DEPLOY_ANGLE){
          currentState = IntakeState.Deployed;
        }
        break;
      case Retracted:
        SetAngleMotor(0);
        if(m_angleEncoder.getPosition() > 0.1){
          retractIntake();;
        }
        break;
      case Retracting:
        SetAngleMotor(AlgaeIntakeConstants.kRETRACT_SPEED);
        if(m_angleEncoder.getPosition() < 0.1){
          currentState = IntakeState.Retracted;
        }
        break;
      default:
        SetAngleMotor(0);
        break;

    }

    if(isIntaking){
      if(angleLimitSwitch.isPressed()){
        SetIntakeMotor(AlgaeIntakeConstants.kHOLD_SPEED);
      }
      else{
        SetIntakeMotor(AlgaeIntakeConstants.kINTAKE_SPEED);
      }
    }

    
  }

  public void SetAngleMotor(double voltage){
    m_angleMotor.setVoltage(voltage);
  }

  public void deployIntake(){
    currentState = IntakeState.Deploying;
    
  }

  public void retractIntake(){
    currentState = IntakeState.Retracting;
  }

  public void SetIntakeMotor(double voltage){
    m_intakeMotor.setVoltage(voltage);
  }

  public void StopIntake(){
    SetIntakeMotor(0);
    retractIntake();
    signalLights.SetSignal(LightSignal.databits);
    isIntaking = false;
  }

  public void Intake(){
    if(angleLimitSwitch.isPressed()){
      SetIntakeMotor(AlgaeIntakeConstants.kHOLD_SPEED);
    }
    else{
      SetIntakeMotor(AlgaeIntakeConstants.kINTAKE_SPEED);
    }
    isIntaking = true;
    
    deployIntake();
    signalLights.SetSignal(LightSignal.hasAlgae);
  }

  public void Outtake(){
    SetIntakeMotor(AlgaeIntakeConstants.kOUTTAKE_SPEED);
    signalLights.SetSignal(LightSignal.hasAlgae);
    isIntaking = false;
  }

  public boolean hasAlgae(){
    return false;
  }

  
}

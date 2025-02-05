// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.SignalLights.LightSignal;

public class Climber extends SubsystemBase {

  public static enum ClimbState{
    Best,//for most of the match before we climb
    Preparing,
    Prepared,
    Climbing,
    Climbed
  }

  private SparkMax m_winchLeader = new SparkMax(ClimberConstants.LEADER_MOTOR_ID, MotorType.kBrushless);
  private SparkMax m_winchFollower = new SparkMax(ClimberConstants.FOLLOWER_MOTOR_ID, MotorType.kBrushless);
  private SparkAbsoluteEncoder m_absEncoder = m_winchLeader.getAbsoluteEncoder();

  public ClimbState currentState = ClimbState.Best;

  public SignalLights signalLights;

  /** Creates a new Climber. */
  public Climber(SignalLights lights) {
    signalLights = lights;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch(currentState){
      case Best:

        if(m_absEncoder.getPosition() < ClimberConstants.CLIMB_BANGLE){
          setWinch(0.01);
        }
        else if(m_absEncoder.getPosition() > ClimberConstants.CLIMB_BANGLE){
          setWinch(-0.01);
        }
        else{
          setWinch(0);
        }

        break;
      case Climbed:
        setWinch(0);
        //TODO: lights party mode
        break;
      case Climbing:
        setWinch(ClimberConstants.kPREPARE_SPEED);

        if(m_absEncoder.getPosition() < ClimberConstants.CLIMB_PANGLE){
          setWinch(0);
          currentState = ClimbState.Prepared;
          signalLights.SetSignal(LightSignal.climbFinish);
        }

        break;
      case Prepared:

        setWinch(0);


        break;
      case Preparing:

        setWinch(ClimberConstants.kPREPARE_SPEED);

        if(m_absEncoder.getPosition() > ClimberConstants.CLIMB_PANGLE){
          setWinch(0);
          currentState = ClimbState.Prepared;
          signalLights.SetSignal(LightSignal.climbPrep);
        }

        break;
      default:

        setWinch(0);

        break;
      
    }
  }


  public void setWinch(double voltage){
    m_winchLeader.set(voltage);
    //other motor should follow
  }

  public void Prepare(){
    currentState = ClimbState.Preparing;
  }

  public void Climb(){
    currentState = ClimbState.Climbing;
  }

  //go to the position for the coral intake mode
  public void Best(){
    currentState = ClimbState.Best;
  }

}

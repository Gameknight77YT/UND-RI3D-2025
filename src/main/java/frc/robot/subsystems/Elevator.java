// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MiniPID;

public class Elevator extends SubsystemBase {


  private TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorID);

  private TalonFXConfiguration elevatorCfg = new TalonFXConfiguration();

  //private DigitalInput toplimitSwitch = new DigitalInput(Constants.toplimitSwitchID);
  //private DigitalInput bottomlimitSwitch = new DigitalInput(Constants.bottomlimitSwitchID);


  private double PIDSpeed = 0;

  private double kP = 0.0001; 
  private double kI = 0.0;
  private double kD = 0.0;

  private final MiniPID mPID = new MiniPID(kP, kI, kD);

  /** Creates a new elevator. */
  public Elevator() {

    elevatorCfg.CurrentLimits.SupplyCurrentLimit = 40;
    elevatorCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    elevatorCfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.3;

    elevatorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    elevatorMotor.getConfigurator().apply(elevatorCfg);

    elevatorMotor.clearStickyFaults(10);

    

  }

  public void ManualMoveElevator(double speed) {
    
    speed = MathUtil.applyDeadband(speed, 0.15);
    
    speed = Math.pow(speed, 3);
    
    //double curveValue = 1;

    
    double lowerError = Constants.lowerElevatorPosLimit - GetEncoderPosition();
    double upperError = Constants.upperElevatorPosLimit - GetEncoderPosition();

    double lowerErrorSpeed = lowerError * 0.0001;
    double upperErrorSpeed = upperError * 0.0001;

    

    // SmartDashboard.putNumber("lowerError", lowerError);
    // SmartDashboard.putNumber("upperError", upperError);
    // SmartDashboard.putNumber("lowerErrorSpeed", lowerErrorSpeed);
    // SmartDashboard.putNumber("upperErrorSpeed", upperErrorSpeed);

    if (speed < 0){
      if (speed < lowerErrorSpeed){
        speed = lowerErrorSpeed;
      }
    }

    else if (speed > 0){
      if (speed > upperErrorSpeed){
        speed = upperErrorSpeed;
      }
    }   

    // if (speed < 0 && !GetBottomLimitSwitch()){
    //   speed = 0;
    // }

    // if (speed > 0 && !GetTopLimitSwitch()){
    //   speed = 0;
    // }

    SmartDashboard.putNumber("CommandedElevatorSpeed", speed);

    if (speed > Constants.maxManualArmSpeed){
      speed = Constants.maxManualArmSpeed;
    }
    if (speed < -Constants.maxManualArmSpeed){
      speed = -Constants.maxManualArmSpeed;
    }

    // double p = SmartDashboard.getNumber("P Gain Arm", 0);
    // double i = SmartDashboard.getNumber("I Gain Arm", 0);
    // double d = SmartDashboard.getNumber("D Gain Arm", 0);
  
    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { mPID.setP(p); kP = p; }
    // if((i != kI)) { mPID.setI(i); kI = i; }
    // if((d != kD)) { mPID.setD(d); kD = d; }
    

    elevatorMotor.set(speed);


  }

  public void PIDMoveElevator(double position){
    if (position > Constants.upperElevatorPosLimit){
      position = Constants.upperElevatorPosLimit;
    }
    if (position < Constants.lowerElevatorPosLimit){
      position = Constants.lowerElevatorPosLimit;
    }
    PIDSpeed = 0;

    // double posCorrection = (0.08333*position) - 3;
    // if (position > 20 && position < 80){
    //   position = position - posCorrection;
    // }
    
    PIDSpeed = mPID.getOutput(GetEncoderPosition(), position);
    SmartDashboard.putNumber("elevator: PID commanded speed", PIDSpeed);
    SmartDashboard.putNumber("elevator: Commanded position", position);
    //System.out.println(PIDSpeed);

    if (PIDSpeed > 1){
      PIDSpeed = 1;
    }
    if (PIDSpeed < -1){
      PIDSpeed = -1;
    }

    

    double lowerError = Constants.lowerElevatorPosLimit - GetEncoderPosition();
    double upperError = Constants.upperElevatorPosLimit - GetEncoderPosition();

    double lowerErrorSpeed = lowerError * 0.0001;
    double upperErrorSpeed = upperError * 0.0001;



    if (PIDSpeed < 0){
      if (PIDSpeed < lowerErrorSpeed){
        PIDSpeed = lowerErrorSpeed;
      }
    }

    else if (PIDSpeed > 0){
      if (PIDSpeed > upperErrorSpeed){
        PIDSpeed = upperErrorSpeed;
      }
    }   


    // if (PIDSpeed < 0 && !GetBottomLimitSwitch()){
    //   PIDSpeed = 0;
    // }

    // if (PIDSpeed > 0 && !GetTopLimitSwitch()){
    //   PIDSpeed = 0;
    // }

    // if (PIDSpeed > 0.2){
    //   PIDSpeed = 0.2;
    // }
    // if (PIDSpeed < -0.2){
    //   PIDSpeed = -0.2;
    // }
    //mLeader.set(PIDSpeed);
    if (Math.abs(position - GetEncoderPosition()) > 0.1){
      if (Math.abs(PIDSpeed) < 0.02){
        //PIDSpeed = 0.08 * (PIDSpeed/PIDSpeed);
      }
    }

    //mLeader.setControl(turnDutyCycle);
    elevatorMotor.set(PIDSpeed);

    //System.out.println("PIDspeed" + PIDSpeed);

  }

  

  public double GetEncoderPosition(){
    var position = elevatorMotor.getPosition();
    double adjustedPos = (position.getValueAsDouble() * 360.0) + 79.7167;
    SmartDashboard.putNumber("elevator position", adjustedPos);
    
    return adjustedPos;
  }

  public boolean IsPIDFinished(double goalState){
    
    if (Math.abs(GetEncoderPosition() - goalState) < 1){
      return true;
    }
    return false;
  }

  public void zeroEncoder() {
    elevatorMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    GetEncoderPosition();
    
  }
}

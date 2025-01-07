// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MiniPID;

public class Wrist extends SubsystemBase {
  
  private TalonFX wristMotor = new TalonFX(Constants.wristMotorID);

  private TalonFXConfiguration wristCfg = new TalonFXConfiguration();

  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(Constants.wristEncoderID);

  private MiniPID mPID = new MiniPID(.01, 0, 0);

  private double PIDSpeed;

  /** Creates a new Wrist. */
  public Wrist() {

    wristEncoder.setInverted(false);

    wristCfg.CurrentLimits.SupplyCurrentLimit = 40;
    wristCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    wristCfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.3;

    wristCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    wristMotor.getConfigurator().apply(wristCfg);

    wristMotor.clearStickyFaults(10);

  }

  public void ManualMoveWrist(double speed) {
    
    speed = MathUtil.applyDeadband(speed, 0.15);
    
    speed = Math.pow(speed, 3);
    
    //double curveValue = 1;

    double lowerError = Constants.backWristPosLimit - getWristEncoder().in(Degree);
    double upperError = Constants.forwardWristPosLimit - getWristEncoder().in(Degree);

    double lowerErrorSpeed = lowerError * -0.025; //lower = more dampning
    double upperErrorSpeed = upperError * -0.025;

    

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

    

    SmartDashboard.putNumber("CommandedWristSpeed", speed);

    if (speed > Constants.wristSpeed){
      speed = Constants.wristSpeed;
    }
    if (speed < -Constants.wristSpeed){
      speed = -Constants.wristSpeed;
    }

    

    // double p = SmartDashboard.getNumber("P Gain Arm", 0);
    // double i = SmartDashboard.getNumber("I Gain Arm", 0);
    // double d = SmartDashboard.getNumber("D Gain Arm", 0);
  
    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { mPID.setP(p); kP = p; }
    // if((i != kI)) { mPID.setI(i); kI = i; }
    // if((d != kD)) { mPID.setD(d); kD = d; }

    wristMotor.set(speed);


  }

  public void PIDmoveWrist(double position) {
    if (position > Constants.forwardWristPosLimit){
      position = Constants.forwardWristPosLimit;
    }
    if (position < Constants.backWristPosLimit){
      position = Constants.backWristPosLimit;
    }
    PIDSpeed = 0;

    PIDSpeed = mPID.getOutput(getWristEncoder().in(Degree), position);
    SmartDashboard.putNumber("wrist: Commanded position", position);

    if (PIDSpeed > 1){
      PIDSpeed = 1;
    }
    if (PIDSpeed < -1){
      PIDSpeed = -1;
    }

    double lowerError = Constants.backWristPosLimit - getWristEncoder().in(Degree);
    double upperError = Constants.forwardWristPosLimit - getWristEncoder().in(Degree);

    double lowerErrorSpeed = lowerError * -0.025;
    double upperErrorSpeed = upperError * -0.025;

    SmartDashboard.putNumber("wrist: PID commanded speed", PIDSpeed);

    // if (PIDSpeed < 0){
    //   if (PIDSpeed < lowerErrorSpeed){
    //     PIDSpeed = lowerErrorSpeed;
    //   }
    // } else if (PIDSpeed > 0){
    //   if (PIDSpeed > upperErrorSpeed){
    //     PIDSpeed = upperErrorSpeed;
    //   }
    // }   
    

    if (PIDSpeed > Constants.wristSpeed){
      PIDSpeed = Constants.wristSpeed;
    }
    if (PIDSpeed < -Constants.wristSpeed){
      PIDSpeed = -Constants.wristSpeed;
    }
    // //mLeader.set(PIDSpeed);
    // if (Math.abs(position - getWristEncoder().in(Degree)) > 0.1){
    //   if (Math.abs(PIDSpeed) < 0.02){
    //     //PIDSpeed = 0.08 * (PIDSpeed/PIDSpeed);
    //   }
    // }
    
    

    //mLeader.setControl(turnDutyCycle);
    wristMotor.set(-PIDSpeed);

    //System.out.println("PIDspeed" + PIDSpeed);


  }

  public Angle getWristEncoder() {
    return Rotations.of(wristEncoder.get()).minus(Degrees.of(176));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist: Encoder position", getWristEncoder().in(Degree));
    
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MiniPID;

public class Wrist extends SubsystemBase {
  
  private SparkMax wristMotor = new SparkMax(Constants.wristMotorID, MotorType.kBrushless);

  private TalonSRX encoder = new TalonSRX(Constants.wristEncoderID);

  private SparkMaxConfig motorcfg = new SparkMaxConfig();
  private TalonSRXConfiguration encodercfg = new TalonSRXConfiguration();

  private MiniPID pidController = new MiniPID(.5, 0, 0);

  private double PIDSpeed;

  /** Creates a new Wrist. */
  public Wrist() {
    motorcfg.idleMode(IdleMode.kBrake).inverted(false);
    
    wristMotor.configure(motorcfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristMotor.clearFaults();

    encodercfg.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;

    encoder.configAllSettings(encodercfg);

  }

  public void ManualMoveWrist(double speed) {
    
    speed = MathUtil.applyDeadband(speed, 0.15);
    
    speed = Math.pow(speed, 3);
    
    //double curveValue = 1;

    double lowerError = Constants.backWristPosLimit - getWristEncoder().in(Degree);
    double upperError = Constants.forwardWristPosLimit - getWristEncoder().in(Degree);

    double lowerErrorSpeed = lowerError * 0.025;
    double upperErrorSpeed = upperError * 0.025;

    SmartDashboard.putNumber("Wrist: Encoder position", getWristEncoder().in(Degree));

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

    

    SmartDashboard.putNumber("CommandedArmSpeed", speed);

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

    PIDSpeed = pidController.getOutput(getWristEncoder().in(Degree), position);
    SmartDashboard.putNumber("wrist: PID commanded speed", PIDSpeed);
    SmartDashboard.putNumber("wrist: Commanded position", position);

    if (PIDSpeed > 1){
      PIDSpeed = 1;
    }
    if (PIDSpeed < -1){
      PIDSpeed = -1;
    }

    double lowerError = Constants.backWristPosLimit - getWristEncoder().in(Degree);
    double upperError = Constants.forwardWristPosLimit - getWristEncoder().in(Degree);

    double lowerErrorSpeed = lowerError * 0.025;
    double upperErrorSpeed = upperError * 0.025;

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

    // if (PIDSpeed > 0.2){
    //   PIDSpeed = 0.2;
    // }
    // if (PIDSpeed < -0.2){
    //   PIDSpeed = -0.2;
    // }
    //mLeader.set(PIDSpeed);
    if (Math.abs(position - getWristEncoder().in(Degree)) > 0.1){
      if (Math.abs(PIDSpeed) < 0.02){
        //PIDSpeed = 0.08 * (PIDSpeed/PIDSpeed);
      }
    }

    //mLeader.setControl(turnDutyCycle);
    wristMotor.set(PIDSpeed);

    //System.out.println("PIDspeed" + PIDSpeed);


  }

  public Angle getWristEncoder() {
    return Radians.of(encoder.getSelectedSensorPosition() / 4096);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder", getWristEncoder().in(Degrees));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private TalonFX clawMotor = new TalonFX(Constants.clawMotorID);

  private TalonFXConfiguration clawCFG = new TalonFXConfiguration();

  /** Creates a new Claw. */
  public Claw() {
    clawCFG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    clawCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    clawMotor.getConfigurator().apply(clawCFG);
  }

  public void intake() {
    clawMotor.set(Constants.clawSpeed);
  }

  public void outtake() {
    clawMotor.set(-Constants.clawSpeed);
  }

  public void stop() {
    clawMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

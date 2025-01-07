package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Wrist;

public class ManualWrist extends Command {
  private final Wrist wrist;
  //private final CommandJoystick mJoystick;
  private DoubleSupplier speed;

  private double lastPos;

  public ManualWrist(Wrist wrist, DoubleSupplier speed) {
    this.wrist = wrist;
    //mJoystick = joystick;
    this.speed = speed;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPos = wrist.getWristEncoder().in(Degrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(lastPos - wrist.getWristEncoder().in(Degrees)) > 3) {
      lastPos = wrist.getWristEncoder().in(Degrees);
    }

    if(Math.abs(speed.getAsDouble()) > .05) {
      wrist.ManualMoveWrist(speed.getAsDouble());
    } else {
      wrist.PIDmoveWrist(lastPos);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.ManualMoveWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
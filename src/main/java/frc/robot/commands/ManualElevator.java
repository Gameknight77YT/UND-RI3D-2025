package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command {
  private final Elevator elevator;
  //private final CommandJoystick mJoystick;
  private DoubleSupplier speed;

  private double lastPos;

  public ManualElevator(Elevator elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    //mJoystick = joystick;
    this.speed = speed;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPos = elevator.GetEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(lastPos - elevator.GetEncoderPosition()) > 500) {
      lastPos = elevator.GetEncoderPosition();
    }

    if(Math.abs(speed.getAsDouble()) > .05) {
      elevator.ManualMoveElevator(speed.getAsDouble());
    } else {
      elevator.PIDMoveElevator(lastPos);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.ManualMoveElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
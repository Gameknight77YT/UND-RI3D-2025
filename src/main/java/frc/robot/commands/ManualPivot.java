package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Pivot;

public class ManualPivot extends Command {
  private final Pivot pivot;
  //private final CommandJoystick mJoystick;
  private DoubleSupplier speed;

  public ManualPivot(Pivot pivot, DoubleSupplier speed) {
    this.pivot = pivot;
    //mJoystick = joystick;
    this.speed = speed;

    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double speed = mJoystick.getY();

    pivot.ManualMovePivot(speed.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Intake;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.ManualWrist;
import frc.robot.commands.Outtake;
import frc.robot.commands.PIDElevator;
import frc.robot.commands.PIDPivot;
import frc.robot.commands.PIDWrist;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    private Elevator elevator = new Elevator();
    private Pivot pivot = new Pivot();
    private Wrist wrist = new Wrist();
    private Claw claw = new Claw();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/3; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private SlewRateLimiter xLimiter = new SlewRateLimiter(40.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(40.0);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(36.0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    //private final Telemetry logger = new Telemetry(MaxSpeed);
 
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xLimiter.calculate(-Math.pow(MathUtil.applyDeadband(driverController.getLeftY(), 0.05),3) * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(yLimiter.calculate(-Math.pow(MathUtil.applyDeadband(driverController.getLeftX(), .05),3) * MaxSpeed)) // Drive left with negative X (left)
                    //.withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(driverController.getRawAxis(3), .05) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
                    .withRotationalRate(rotLimiter.calculate(-Math.pow(MathUtil.applyDeadband(driverController.getRightX(), .05),3) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        //elevator
        elevator.setDefaultCommand(new ManualElevator(elevator, () -> -manipulatorController.getLeftY()));
        //manipulatorController.button(5).whileTrue(new PIDElevator(elevator, 15000)); // y
        //manipulatorController.button(7).onTrue(elevator.runOnce(() -> elevator.zeroEncoder()));//left bumper

        //pivot
        pivot.setDefaultCommand(new ManualPivot(pivot, () -> -manipulatorController.getRawAxis(4)));//right y
        //manipulatorController.button(2).whileTrue(new PIDPivot(pivot, -45)); // b

        //wrist
        wrist.setDefaultCommand(new ManualWrist(wrist, () -> -manipulatorController.getRawAxis(3)));
        //manipulatorController.povUp().whileTrue(new ManualWrist(wrist, () -> (Constants.wristSpeed)));
        //manipulatorController.povDown().whileTrue(new ManualWrist(wrist, () -> (-Constants.wristSpeed)));
        //manipulatorController.button(1).whileTrue(new PIDWrist(wrist, -60));// a
        
        //claw
        manipulatorController.button(10).whileTrue(new Intake(claw));//right trigger
        manipulatorController.button(8).whileTrue(new Outtake(claw));//right bumper
        
        //combination
        manipulatorController.button(1).whileTrue( // a  -  zero
            new PIDElevator(elevator, Constants.lowerElevatorPosLimit)
            .alongWith(new PIDPivot(pivot, 0))
            .alongWith(new PIDWrist(wrist, 0))
            );

        manipulatorController.povLeft().whileTrue(  //coral human player station
            new PIDElevator(elevator, Constants.elevatorMotorID)
            .alongWith(new PIDPivot(pivot, -52))
            .alongWith(new PIDWrist(wrist, -85))
            );
        
        manipulatorController.button(7).whileTrue( //left bumper  -  upper alge reef pickup
            new PIDElevator(elevator, 9300)
            .alongWith(new PIDPivot(pivot, -28))
            .alongWith(new PIDWrist(wrist, 0))
            );

        manipulatorController.button(9).whileTrue( //left trigger  -  lower alge reef pickup
            new PIDElevator(elevator, 203) 
            .alongWith(new PIDPivot(pivot, -37))
            .alongWith(new PIDWrist(wrist, 3.5))
            );
        
        manipulatorController.povDown().whileTrue( //L1 reef
            new PIDElevator(elevator, Constants.lowerElevatorPosLimit) 
            .alongWith(new PIDPivot(pivot, -85))
            .alongWith(new PIDWrist(wrist, -82))
            );

        manipulatorController.povRight().whileTrue( //L2 reef
            new PIDElevator(elevator, Constants.lowerElevatorPosLimit) 
            .alongWith(new PIDPivot(pivot, -75))
            .alongWith(new PIDWrist(wrist, -72))
            );

        manipulatorController.povUp().whileTrue( //L3 reef
            new PIDElevator(elevator, 4700) 
            .alongWith(new PIDPivot(pivot, 14))
            .alongWith(new PIDWrist(wrist, -16))
            );

        manipulatorController.button(5).whileTrue( //y  - L4 reef
            new PIDElevator(elevator, Constants.upperElevatorPosLimit) 
            .alongWith(new PIDPivot(pivot, 5))
            .alongWith(new PIDWrist(wrist, -38))
            );

        
        
        
        /*manipulatorController.button(0).whileTrue(
            new PIDElevator(elevator, 0)
            .alongWith(new PIDPivot(pivot, 0))
            .alongWith(new PIDWrist(wrist, 0))
            );
        */
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //driverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
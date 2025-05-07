// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final CommandXboxController xbox = new CommandXboxController(0);
    private final CommandJoystick joystick = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        configureXbox();
    }

    //TODO add way to switch between xbox and joystick/2

    @SuppressWarnings("unused")
    private void configureXbox() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-xbox.getLeftY() * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((-xbox.getLeftX() * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        xbox.b().whileTrue(drivetrain.applyRequest(() -> brake));
        xbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))
        ));

        //Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xbox.back().and(xbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        xbox.back().and(xbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        xbox.start().and(xbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        xbox.start().and(xbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @SuppressWarnings("unused")
    private void configureJoystick() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-joystick.getY() * MaxSpeed) * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY((-joystick.getX() * MaxSpeed) * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.button(1).whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.button(11).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.button(10).and(joystick.button(3)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.button(10).and(joystick.button(3)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.button(9).and(joystick.button(4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.button(9).and(joystick.button(4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.button(12).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

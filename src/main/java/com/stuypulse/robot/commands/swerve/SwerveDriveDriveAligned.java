package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveAligned extends Command {

    private final SwerveDrive swerve;
    private final VStream drive;
    private final Odometry odometry;

    private final AngleController controller;

    private final Supplier<Rotation2d> targetAngle;

    public SwerveDriveDriveAligned(Gamepad driver, Supplier<Rotation2d> targetAngle) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        this.targetAngle = targetAngle;

        drive = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get()));

        controller = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD);
                
        addRequirements(swerve);
    }

    public SwerveDriveDriveAligned(Gamepad driver, Rotation2d targetAngle) {
        this(driver, () -> targetAngle);
    }

    @Override
    public void execute() {
        swerve.drive(
            drive.get(),
            controller.update(
                Angle.fromRotation2d(targetAngle.get()),
                Angle.fromRotation2d(odometry.getPose().getRotation())));
    }
}

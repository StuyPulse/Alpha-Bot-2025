package com.stuypulse.robot.commands.swerve;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePIDToPose extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final HolonomicDriveController controller;
    private final Supplier<Pose2d> poseSupplier;
    private final BStream isAligned;

    private final StopWatch stopWatch;

    private final FieldObject2d targetPose2d;
    private final FieldObject2d setpointPose2d;

    private Pose2d targetPose;
    private Trajectory trajectory;

    public SwerveDrivePIDToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwerveDrivePIDToPose(Supplier<Pose2d> poseSupplier) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        this.poseSupplier = poseSupplier;

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");
        setpointPose2d = Odometry.getInstance().getField().getObject("Setpoint Pose");

        controller = new HolonomicDriveController(
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD), 
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD), 
            new ProfiledPIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD, new Constraints(Settings.Swerve.Constraints.MAX_ANGULAR_VELOCITY.get(), Settings.Swerve.Constraints.MAX_ANGULAR_ACCELERATION.get()))
        );

        controller.setTolerance(new Pose2d(
            Settings.Swerve.Alignment.X_TOLERANCE.get(), 
            Settings.Swerve.Alignment.Y_TOLERANCE.get(), 
            Rotation2d.fromDegrees(Settings.Swerve.Alignment.THETA_TOLERANCE.get())
        ));

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Alignment.ALIGNMENT_DEBOUNCE));

        stopWatch = new StopWatch();

        addRequirements(swerve);
    }

    private boolean isAligned() {
        Pose2d pose = odometry.getPose();

        return Math.abs(targetPose.getX() - pose.getX()) < Settings.Swerve.Alignment.X_TOLERANCE.get()
            && Math.abs(targetPose.getY() - pose.getX()) < Settings.Swerve.Alignment.Y_TOLERANCE.get()
            && Math.abs(Angle.fromRotation2d(targetPose.getRotation()).sub(Angle.fromRotation2d(pose.getRotation())).toRadians()) < Settings.Swerve.Alignment.THETA_TOLERANCE.get();
    }

    @Override
    public void initialize() {
        targetPose = poseSupplier.get();

        trajectory = TrajectoryGenerator.generateTrajectory(
            odometry.getPose(), 
            new ArrayList<Translation2d>(), 
            targetPose, 
            new TrajectoryConfig(Settings.Swerve.Constraints.MAX_VELOCITY.get(), Settings.Swerve.Constraints.MAX_ACCELERATION.get())
        );

        stopWatch.reset();
    }

    @Override
    public void execute() {
        targetPose2d.setPose(Robot.isBlue() ? targetPose : Field.transformToOppositeAlliance(targetPose));

        SmartDashboard.putNumber("Alignment/Target x", targetPose.getX());
        SmartDashboard.putNumber("Alignment/Target y", targetPose.getY());
        SmartDashboard.putNumber("Alignment/Target angle", targetPose.getRotation().getDegrees());

        Trajectory.State goal = trajectory.sample(stopWatch.getTime());

        targetPose2d.setPose(Robot.isBlue() ? goal.poseMeters : Field.transformToOppositeAlliance(goal.poseMeters));

        SmartDashboard.putNumber("Alignment/Setpoint x", goal.poseMeters.getX());
        SmartDashboard.putNumber("Alignment/Setpoint y", goal.poseMeters.getY());
        SmartDashboard.putNumber("Alignment/Setpoint angle", goal.poseMeters.getRotation().getDegrees());
        
        ChassisSpeeds speeds = controller.calculate(odometry.getPose(), goal, targetPose.getRotation());
        speeds.omegaRadiansPerSecond *= -1;
        swerve.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());
        Field.clearFieldObject(setpointPose2d);
        Field.clearFieldObject(targetPose2d);
    }

}

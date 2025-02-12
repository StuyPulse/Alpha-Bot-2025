package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightVision extends AprilTagVision{

    private String[] names;
    private boolean enabled;
    private boolean[] camerasEnabled;

    public LimelightVision() {
        names = new String[Cameras.LimelightCameras.length];
        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            names[i] = Cameras.LimelightCameras[i].getName();
            Pose3d robotRelativePose = Cameras.LimelightCameras[i].getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                names[i], 
                robotRelativePose.getX(), 
                robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                robotRelativePose.getRotation().getX(), 
                robotRelativePose.getRotation().getY(), 
                robotRelativePose.getRotation().getZ()
            );
        }

        camerasEnabled = new boolean[Cameras.LimelightCameras.length];
        for (int i = 0; i < camerasEnabled.length; i++) {
            camerasEnabled[i] = true;
        }

        enabled = true;
    }

    @Override
    public void setTagWhitelist(int... ids) {
        for (String name : names) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        }
    }

    @Override
    public void enable() {
        enabled = true;
    }

    @Override
    public void disable() {
        enabled = false;
    }

    @Override
    public void setCameraEnabled(String name, boolean enabled) {
        for (int i = 0; i < names.length; i++) {
            if (names[i].equals(name)) {
                camerasEnabled[i] = enabled;
            }
        }
    }

    @Override
    public void periodic() {
        if (enabled) {
            for (int i = 0; i < names.length; i++) {
                if (camerasEnabled[i]) {
                    String limelightName = names[i];

                    LimelightHelpers.SetRobotOrientation(
                        limelightName, 
                        (Odometry.getInstance().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0
                    );

                    PoseEstimate poseEstimate = Robot.isBlue() 
                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
                        : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
                    
                    if (poseEstimate != null && poseEstimate.tagCount > 0) {
                        Pose2d robotPose = poseEstimate.pose;
                        double timestamp = poseEstimate.timestampSeconds;
                        Odometry.getInstance().addVisionData(robotPose, timestamp, Settings.Vision.MIN_STDDEVS.times(1 + poseEstimate.avgTagDist));
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", true);
                    }
                    else {
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
                    }
                }
            }
        }

        SmartDashboard.putBoolean("Vision/Is Enabled", enabled);
        for (int i = 0; i < names.length; i++) {
            SmartDashboard.putBoolean("Vision/" + names[i] + " Is Enabled", camerasEnabled[i]);
        }
    }
}
 
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TagPose2d {
    private Pose2d m_pose;
    public TagPose2d(Pose2d pose) {
        m_pose = pose;
    }
    public TagPose2d translate(Transform2d translation) {
        return new TagPose2d(m_pose.plus(translation));
    }
    public TagPose2d facing(Pose2d target) {
        SmartDashboard.putString("Target Pose: ", target.toString());
        SmartDashboard.putString("Origin Pose", m_pose.toString());
        double x = target.getX() - m_pose.getX();
        double y = target.getY() - m_pose.getY();
        double angle = Math.atan2(y, x);
        SmartDashboard.putNumber("Angle from .facing: ", angle);
        Rotation2d rotation = new Rotation2d(angle);
        return new TagPose2d(new Pose2d(m_pose.getX(), m_pose.getY(), rotation));
    }
    public TagPose2d facing(TagPose2d target) {
        return this.facing(target.toPose());
    }
    public Pose2d toPose() {
        return m_pose;
    }
}

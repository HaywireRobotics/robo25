// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Add your docs here. */
public class FieldLayout {
    private AprilTagFieldLayout m_fieldLayout;
    private boolean m_fieldLayoutReady = false;
    public FieldLayout(String pathToJSON) {
        try {
            m_fieldLayout = new AprilTagFieldLayout(pathToJSON);
            m_fieldLayoutReady = true;
        } catch (IOException err) {
            System.err.println("Failed to fetch " + pathToJSON + ". Does it exist or is it in the /deploy directory?");
        }
    }
    public TagPose2d getTag(int id) {
        return new TagPose2d(m_fieldLayout.getTagPose(id).get().toPose2d());
    }
}

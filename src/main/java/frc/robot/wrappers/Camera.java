// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;
import java.io.IOException;
import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.kConstants;

/** Add your docs here. */
public class Camera extends PhotonCamera {
    private List<PhotonPipelineResult> m_cameraData;
    private Transform3d m_cameraOffset;
    private AprilTagFieldLayout aprilTagFieldLayout;
    public Camera(String name, Transform3d cameraOffset) {
        super(name);
        m_cameraOffset = cameraOffset;
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(kConstants.kFieldAprilTagJSON);
        } catch (IOException error) {
            System.out.println("Couldn't find " + kConstants.kFieldAprilTagJSON + ". Does it exist?");
        }
    }
    public ArrayList<Integer> getVisibleAprilTagIds() {
        List<PhotonTrackedTarget> targets = this.getVisibleAprilTags();
        ArrayList<Integer> out = new ArrayList<Integer>();
        for (Integer i = 0; i < targets.size(); i++) {
            out.add(targets.get(i).fiducialId);
        }
        return out;
    }

    public List<PhotonTrackedTarget> getVisibleAprilTags() {
        updateVisible();
        return m_cameraData.get(0).getTargets();
    }

    public PhotonPipelineResult getTargets() {
        updateVisible();
        return m_cameraData.get(0);
    }

    public Optional<Pose2d> estimatePose(Pose2d robotPose) {
        updateVisible();
        if (m_cameraData.size() == 0 || m_cameraData.get(0) == null) {
            return Optional.empty();
        }
        if (!m_cameraData.get(0).hasTargets()) {
            return Optional.empty();
        }
        PhotonTrackedTarget target = m_cameraData.get(0).getBestTarget();
        if (target == null) {
            return Optional.empty();
        }
        if (target.poseAmbiguity > kConstants.kMaxPoseAmbiguity) {
            return Optional.empty();
        }
        if (target.bestCameraToTarget.getX() > kConstants.kMaxTagDistance) {
            return Optional.empty();
        }
        int target_id = target.fiducialId;
        Pose3d field_target_pose = aprilTagFieldLayout.getTagPose(target_id).get();
        Pose3d camera_pose = field_target_pose.plus(target.bestCameraToTarget.inverse());
        Pose3d calculatedRobotPose = camera_pose.plus(m_cameraOffset.inverse());
        return Optional.of(calculatedRobotPose.toPose2d().interpolate(robotPose, target.poseAmbiguity*10));
    }

    private void updateVisible() {
        List<PhotonPipelineResult> results = this.getAllUnreadResults();
        if (results.isEmpty()) {
            return;
        } else {
            m_cameraData = results;
        }
    }
}
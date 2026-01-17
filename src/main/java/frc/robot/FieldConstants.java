// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public static final Translation2d hubCenter = new Translation2d(4.6256194, 4.0346376);

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none");

    ArrayList<AprilTag> aprilTags = new ArrayList<AprilTag>();
    AprilTag tag1 = new AprilTag(10, new Pose3d());

    AprilTagLayoutType(String name) {
      aprilTags.add(tag1);

      layout = new AprilTagFieldLayout(
        aprilTags, 27, 54
      );

      layoutString = name;
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;

    public AprilTagFieldLayout getLayout() {
      return layout;
    }

    public String getLayoutString() {
      return layoutString;
    }
  }
}

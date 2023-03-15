package frc.robot.supportingClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class VisionMedianFilterTest {

    @Test
    public void getOdoPose() {
        VisionMedianFilter medianFilter = new VisionMedianFilter(5);
        Pose2d[] poses = new Pose2d[] {
                new Pose2d(1.09, 2.054, new Rotation2d(Math.toRadians(29))),
                new Pose2d(0.95, 3, new Rotation2d(Math.toRadians(24))),
                new Pose2d(1.23, 1.954, new Rotation2d(Math.toRadians(50))),
                new Pose2d(1.1, 2.091, new Rotation2d(Math.toRadians(35))),
                new Pose2d(0.975, 1.992, new Rotation2d(Math.toRadians(32))),
                new Pose2d(1.2, 2.2, new Rotation2d(Math.toRadians(37)))
        };
        for (Pose2d pose : poses) {
            System.out.println(medianFilter.getOdoPose(new OdoPosition(pose, 1)));
        }
    }
}
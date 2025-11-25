package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * AutoApriltag Subsystem
 *
 * This subsystem handles AprilTag detection using the Limelight 3A camera.
 */
public class AutoApriltag extends SubsystemBase {
    public Limelight3A limelight;

    /**
     * Constructor for AutoApriltag.
     * Initializes and starts the Limelight camera.
     *
     * @param hardwareMap The hardware map to get the Limelight.
     */
    public AutoApriltag(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
    }

    /**
     * Gets the list of detected AprilTag IDs.
     *
     * @return A list of integers representing the IDs of currently visible AprilTags.
     */
    public List getNumbers() {
        List<FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        List apriltagIDs = new ArrayList<>();
        for (FiducialResult fiducial: fiducials) {
            apriltagIDs.add(fiducial.getFiducialId()); // The ID number of the fiducial
        }
        return apriltagIDs;
    }
}

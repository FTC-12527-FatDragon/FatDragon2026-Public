package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)//Mass of Robot
            .forwardZeroPowerAcceleration(0)
            .lateralZeroPowerAcceleration(0)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.085,
                    0.01,
                    0,
                    0.03
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0,
                    0,
                    0,
                    0,
                    0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0,
                    0,
                    0,
                    0,
                    0
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontMotor")
            .rightRearMotorName("rightBackMotor")
            .leftRearMotorName("leftBackMotor")
            .leftFrontMotorName("leftFrontMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3)
            .strafePodX(-3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);




}

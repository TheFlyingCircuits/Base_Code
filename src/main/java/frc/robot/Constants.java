// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static boolean atCompetition = false;

    public final class UniversalConstants {
        public final static double gravityMetersPerSecondSquared = 9.81;
        public final static double defaultPeriodSeconds = 0.02;

        public final static String canivoreName = "CTRENetwork";

        public enum Direction {
            left,
            right
        }

        public static final double frameWidthMeters = Units.inchesToMeters(27);

        public static final double bumperWidthMeters = Units.inchesToMeters(27 + 7);

    }

    public final static class ControllerConstants {
        public static final double controllerDeadzone = 0.075;
        public static final double maxThrottle = 1.0;
    }


    public final static class GyroConstants {
        public static final int pigeonID = 0;


        //Follow the mount calibration process in Phoenix Tuner to determine these
        public static final double mountPoseYawDegrees = 92.07196807861328;
        public static final double mountPosePitchDegrees = -0.24960607290267944;
        public static final double mountPoseRollDegrees = -0.591957151889801;
    }

    public final static class VisionConstants {
        //Camera, IP, hostname
        //intakeCam, 10.17.87.50, ?
        //frontLeft, 10.17.87.54, Photon-RPi4-FL
        //frontRight, 10.17.87.51, Photon-RPi4-FR
        //backRight, 10.17.87.52, Photon-RPi4-BR
        //backLeft, 10.17.87.53, Photon-RPi4-BL
        public final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
                                                       

        public final static Transform3d robotToFrontLeft = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.248), Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, -Math.toRadians(8), -Math.toRadians(17.772))
        );

        public final static Transform3d robotToFrontLeft_calibrated = new Transform3d(
            new Translation3d(0.2, 0.274, 0.205),
            new Rotation3d(Units.degreesToRadians(0.08), Units.degreesToRadians(-7.264), Units.degreesToRadians(-18.2))
        );

        public final static Transform3d robotToFrontRight = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.248), -Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, -Math.toRadians(8), Math.toRadians(17.772))
        );

        public final static Transform3d robotToFrontRight_calibrated = new Transform3d(
            new Translation3d(0.201, -0.288, 0.207),
            new Rotation3d(Units.degreesToRadians(0.198), Units.degreesToRadians(-8.141), Units.degreesToRadians(18.319))
        );

        public final static Transform3d robotToBackLeft = new Transform3d(
            new Translation3d(-0.184, Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, Math.toRadians(-8), Math.toRadians(17.772-180))
        );


        public final static Transform3d robotToBackLeft_calibrated = new Transform3d(
            new Translation3d(-0.194, 0.293, 0.202),
            new Rotation3d(Units.degreesToRadians(0.32), Units.degreesToRadians(-6.124), Units.degreesToRadians(-161.518))
        );

        public final static Transform3d robotToBackRight = new Transform3d(
            new Translation3d(-Units.inchesToMeters(7.248), -Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, Math.toRadians(-8), Math.toRadians(-17.772+180))
        );

        public final static Transform3d robotToBackRight_calibrated = new Transform3d(
            new Translation3d(-0.201, -0.274, 0.203),
            new Rotation3d(Units.degreesToRadians(-0.277), Units.degreesToRadians(-7.105), Units.degreesToRadians(160.893))
        );

        public final static Transform3d robotToCoralCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(5.5), Units.inchesToMeters(26.)),
            new Rotation3d(0, Math.toRadians(19), Math.toRadians(-12))
        );

        public final static Transform3d robotToCoralCameraCalibrated = new Transform3d(
            -0.382, 0.161, 0.683,
            new Rotation3d(Math.toRadians(-0.91), Math.toRadians(21.76), Math.toRadians(-14.15))
        );

        public final static String[] tagCameraNames = {
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight"
        };

        public final static Transform3d[] tagCameraTransforms = {
            robotToFrontLeft_calibrated,
            robotToFrontRight_calibrated,
            robotToBackLeft_calibrated,
            robotToBackRight_calibrated
        };

        public final static Transform3d[] oldTagCameraTransforms = {
            robotToFrontLeft,
            robotToFrontRight,
            robotToBackLeft,
            robotToBackRight
        };

    }

}

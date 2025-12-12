package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.subsystems.vision.SingleTagCam;
import frc.robot.subsystems.vision.SingleTagPoseObservation;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputsLogged;

public class Drivetrain extends SubsystemBase {
    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private VisionIO visionIO;
    private VisionIOInputsLogged visionInputs;

    private SingleTagCam[] tagCams = {
        new SingleTagCam(VisionConstants.tagCameraNames[0], VisionConstants.tagCameraTransforms[0]), // front left
        new SingleTagCam(VisionConstants.tagCameraNames[1], VisionConstants.tagCameraTransforms[1]), // front right
        new SingleTagCam(VisionConstants.tagCameraNames[2], VisionConstants.tagCameraTransforms[2]), // back left
        new SingleTagCam(VisionConstants.tagCameraNames[3], VisionConstants.tagCameraTransforms[3])  // back right
    };

    private boolean fullyTrustVisionNextPoseUpdate = false;
    private boolean allowTeleportsNextPoseUpdate = false;
    private boolean hasAcceptablePoseObservationsThisLoop = false;

    private SwerveModule[] swerveModules;

    private SwerveDrivePoseEstimator fusedPoseEstimator;
    private SwerveDrivePoseEstimator wheelsOnlyPoseEstimator;

    /**
     * Distance between the center point of the left wheels and the center point of the right wheels.
     */
    double trackwidthMeters = Units.inchesToMeters(21.75);
    /**
     * Distance between the center point of the front wheels and the center point of the back wheels.
     */
    double wheelbaseMeters = Units.inchesToMeters(21.75);

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
 
    public Drivetrain(
        GyroIO gyroIO, 
        SwerveModuleIO flSwerveModuleIO, 
        SwerveModuleIO frSwerveModuleIO, 
        SwerveModuleIO blSwerveModuleIO, 
        SwerveModuleIO brSwerveModuleIO,
        VisionIO visionIO
    ) {
        
        this.gyroIO = gyroIO;
        gyroInputs = new GyroIOInputsAutoLogged();

        this.visionIO = visionIO;
        visionInputs = new VisionIOInputsLogged();

        swerveModules = new SwerveModule[] {
            new SwerveModule(flSwerveModuleIO, 0, "frontLeft"),
            new SwerveModule(frSwerveModuleIO, 1, "frontRight"),
            new SwerveModule(blSwerveModuleIO, 2, "backLeft"),
            new SwerveModule(brSwerveModuleIO, 3, "backRight")
        };

        gyroIO.setRobotYaw(0);

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.005);

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0., 0., 0.);


        fusedPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics, 
            gyroInputs.robotYawRotation2d,
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );

        wheelsOnlyPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics,
            gyroInputs.robotYawRotation2d,
            getModulePositions(), 
            new Pose2d());

        RobotConfig config;
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          config = null;
          e.printStackTrace();
        }

        setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getRobotRelativeVelocityMPS(); // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getModuleStates(); // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
    }
 
    
    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxDesiredTeleopVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleIndex]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveModules) {
            swervePositions[mod.moduleIndex] = mod.getPosition();
        }

        return swervePositions;
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveStates = new SwerveModuleState[4];

        for (SwerveModule mod : swerveModules) {
            swerveStates[mod.moduleIndex] = mod.getState();
        }

        return swerveStates;
    }

    //**************** DRIVING ****************//

    /**
     * Drives the robot based on a desired ChassisSpeeds.
     * <p>
     * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
     * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
     * @param closedLoop - Whether or not to used closed loop PID control to control the speed of the drive wheels.
    */
    public void robotOrientedDrive(ChassisSpeeds desiredChassisSpeeds) {
        // SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            desiredChassisSpeeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );
        setModuleStates(previousSetpoint.moduleStates());
    }

    /**
     * Drives the robot at a desired chassis speeds. The coordinate system
     * is the same as the one as the one for setPoseMeters().
     * 
     * @param desiredChassisSpeeds - Field relative chassis speeds, in m/s and rad/s. 
     * @param closedLoop - Whether or not to drive the drive wheels with using feedback control.
     */
    public void fieldOrientedDrive(ChassisSpeeds desiredChassisSpeeds) {
        Rotation2d currentOrientation = getPoseMeters().getRotation();
        ChassisSpeeds robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredChassisSpeeds, currentOrientation);
        this.robotOrientedDrive(robotOrientedSpeeds);
    }

    public ChassisSpeeds getFieldOrientedVelocity() {
        ChassisSpeeds robotOrientedSpeeds = DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotOrientedSpeeds, getPoseMeters().getRotation());
    }

    public ChassisSpeeds getRobotRelativeVelocityMPS() {
        return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public double getSpeedMetersPerSecond() {
        ChassisSpeeds v = getFieldOrientedVelocity();
        double s = Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
        return s;
    }

    //**************** ODOMETRY ****************//

    public void setPoseMeters(Pose2d pose) {
        fusedPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
        wheelsOnlyPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
    }
    public void setOrientation(Rotation2d orientation) {
        // keep location the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(currentPose.getTranslation(), orientation));
    }
    public void setLocation(Translation2d locationOnField) {
        // keep orientation the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(locationOnField, currentPose.getRotation()));
    }

     /**
     * Gets the current position of the robot on the field in meters, 
     * based off of our odometry and vision estimation.
     * This value considers the origin to be the right side of the blue alliance.
     * <p>
     * A positive X value brings the robot towards the red alliance, and a positive Y value
     * brings the robot towards the left side as viewed from the blue alliance.
     * <p>
     * Rotations are discontinuous counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * 
     * @return The current position of the robot on the field in meters.
     */ 
    public Pose2d getPoseMeters() {
        return fusedPoseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the rotation reported by the gyro.
     * This rotation is continuous and counterclockwise positive.
     * 
     * This is not necessarily equivalent to the one reported by getPoseMeters(), and it is recommended
     * to use that rotation in almost every case.
     * 
     * This is usable for calibrating the wheel radii, where a continuous angle is required.
     * @return
     */
    public Rotation2d getGyroRotation2d() {
        return gyroInputs.robotYawRotation2d;
    }

    /**
     * Sets the angle of the robot's pose so that it is facing forward, away from your alliance wall. 
     * This allows the driver to realign the drive direction and other calls to our angle.
     */
    public void setRobotFacingForward() {
        Rotation2d forwardOnRed = Rotation2d.k180deg;
        Rotation2d forwardOnBlue = Rotation2d.kZero;
        Rotation2d forwardNow = getPoseMeters().getRotation();
        this.setOrientation(FlyingCircuitUtils.getAllianceDependentValue(forwardOnRed, forwardOnBlue, forwardNow));
    }

    public void fullyTrustVisionNextPoseUpdate() {
        this.fullyTrustVisionNextPoseUpdate = true;
    }
    public void allowTeleportsNextPoseUpdate() {
        this.allowTeleportsNextPoseUpdate = true;
    }
    public boolean seesAcceptableTag() {
        return this.hasAcceptablePoseObservationsThisLoop;
    }
    private void updatePoseEstimator() {
        // log flags that were set in between last pose update and now
        Logger.recordOutput("drivetrain/fullyTrustingVision", this.fullyTrustVisionNextPoseUpdate);
        Logger.recordOutput("drivetrain/allowingPoseTeleports", this.allowTeleportsNextPoseUpdate);

        // update with wheel deltas
        fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());

        // get all pose observations from each camera
        List<SingleTagPoseObservation> allFreshPoseObservations = new ArrayList<>();
        for (SingleTagCam tagCam : tagCams) {
            allFreshPoseObservations.addAll(tagCam.getFreshPoseObservations());
        }

        // process pose obvervations in chronological order
        allFreshPoseObservations.sort(new Comparator<SingleTagPoseObservation>() {
            public int compare(SingleTagPoseObservation a, SingleTagPoseObservation b) {
                return Double.compare(a.timestampSeconds(), b.timestampSeconds());
            } 
        });

        // Filter tags
        List<Pose3d> acceptedTags = new ArrayList<>();
        List<Pose3d> rejectedTags = new ArrayList<>();
        for (SingleTagPoseObservation poseObservation : allFreshPoseObservations) {

            Translation2d observedLocation = poseObservation.robotPose().getTranslation().toTranslation2d();
            Translation2d locationNow = getPoseMeters().getTranslation();

            // reject tags that are too far away
            if (poseObservation.tagToCamMeters() > 5) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // reject tags that are too ambiguous
            if (poseObservation.ambiguity() > 0.2) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // Don't allow the robot to teleport. Disallowing teleports can cause problems when we get bumped
            // and experience lots of wheel slip, which is why we have the "allowTeleportsNextPoseUpdate" flag
            // (used at driver's discretion (typically via y-button)).
            double teleportToleranceMeters = 4.0;
            if (observedLocation.getDistance(locationNow) > teleportToleranceMeters && (!this.allowTeleportsNextPoseUpdate)) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // This measurment passes all our checks, so we add it to the fusedPoseEstimator
            acceptedTags.add(poseObservation.getTagPose());
            Matrix<N3, N1> stdDevs = this.fullyTrustVisionNextPoseUpdate ? VecBuilder.fill(0, 0, 0) : poseObservation.getStandardDeviations();

            fusedPoseEstimator.addVisionMeasurement(
                poseObservation.robotPose().toPose2d(), 
                poseObservation.timestampSeconds(), 
                stdDevs
            );
        }

        // reset flags for next time
        this.fullyTrustVisionNextPoseUpdate = false;
        this.allowTeleportsNextPoseUpdate = false;
        this.hasAcceptablePoseObservationsThisLoop = acceptedTags.size() > 0;

        // log the accepted and rejected tags
        Logger.recordOutput("drivetrain/acceptedTags", acceptedTags.toArray(new Pose3d[0]));
        Logger.recordOutput("drivetrain/rejectedTags", rejectedTags.toArray(new Pose3d[0]));
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        if (gyroIO instanceof GyroIOSim) //calculates sim gyro
            gyroIO.calculateYaw(getModulePositions());
          

        visionIO.updateInputs(visionInputs);
        for (SwerveModule mod : swerveModules)
            mod.periodic();
          
        Logger.processInputs("gyroInputs", gyroInputs);
        Logger.processInputs("visionInputs", visionInputs);

        updatePoseEstimator();

        Logger.recordOutput("drivetrain/fusedPose", fusedPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/wheelsOnlyPose", wheelsOnlyPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/speedMetersPerSecond", getSpeedMetersPerSecond());

        Logger.recordOutput("drivetrain/swerveModuleStates", getModuleStates());
        Logger.recordOutput("drivetrain/swerveModulePositions", getModulePositions());

        // this.compareCamPoses();
    }
}

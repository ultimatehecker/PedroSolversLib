package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.utilities.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utilities.geometry.Translation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.utilities.geometry.Rotation2d;

/** Utility class for flipping positions/rotations to the other side of the field */
public class FlippingUtil {
    /** The type of symmetry for the current field */
    public static FieldSymmetry symmetryType = FieldSymmetry.kRotational;
    /** The X size or length of the current field in meters */
    public static double fieldSizeX = Units.feetToMeters(57.573);
    /** The Y size or width of the current field in meters */
    public static double fieldSizeY = Units.feetToMeters(26.417);

    /** Enum representing the different types of field symmetry */
    public enum FieldSymmetry {
        /**
         * Field is rotationally symmetric. i.e. the red alliance side is the blue alliance side rotated
         * by 180 degrees
         */
        kRotational,
        /** Field is mirrored vertically over the center of the field */
        kMirrored
    }

    /**
     * Flip a field position to the other side of the field, maintaining a blue alliance origin
     *
     * @param pos The position to flip
     * @return The flipped position
     */
    public static Translation2d flipFieldPosition(Translation2d pos) {
        switch (symmetryType) {
            case kMirrored:
                pos = new Translation2d(fieldSizeX - pos.getX(), pos.getY());
                break;
            case kRotational:
                pos = new Translation2d(fieldSizeX - pos.getX(), fieldSizeY - pos.getY());
                break;
        };

        return pos;
    }

    /**
     * Flip a field rotation to the other side of the field, maintaining a blue alliance origin
     *
     * @param rotation The rotation to flip
     * @return The flipped rotation
     */
    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        switch (symmetryType) {
            case kMirrored:
                Rotation2d.kPi.minus(rotation);
                break;
            case kRotational:
                rotation.minus(Rotation2d.kPi);
                break;
        };

        return rotation;
    }

    /**
     * Flip a field pose to the other side of the field, maintaining a blue alliance origin
     *
     * @param pose The pose to flip
     * @return The flipped pose
     */
    public static Pose2d flipFieldPose(Pose2d pose) {
        return new Pose2d(flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
    }

    /**
     * Flip field relative chassis speeds for the other side of the field, maintaining a blue alliance
     * origin
     *
     * @param fieldSpeeds Field relative chassis speeds
     * @return Flipped speeds
     */
    public static ChassisSpeeds flipFieldSpeeds(ChassisSpeeds fieldSpeeds) {
        switch (symmetryType) {
            case kMirrored:
                fieldSpeeds = new ChassisSpeeds(-fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond, -fieldSpeeds.omegaRadiansPerSecond);
                break;
            case kRotational:
                fieldSpeeds = new ChassisSpeeds(-fieldSpeeds.vxMetersPerSecond, -fieldSpeeds.vyMetersPerSecond, fieldSpeeds.omegaRadiansPerSecond);
                break;
        };

        return fieldSpeeds;
    }

    /**
     * Flip an array of drive feedforwards for the other side of the field. Only does anything if
     * mirrored symmetry is used
     *
     * @param feedforwards Array of drive feedforwards
     * @return The flipped feedforwards
     */
    public static double[] flipFeedforwards(double[] feedforwards) {
        switch (symmetryType) {
            case kMirrored:
                if (feedforwards.length == 4) {
                    feedforwards = new double[] {feedforwards[1], feedforwards[0], feedforwards[3], feedforwards[2]};
                } else if (feedforwards.length == 2) {
                    feedforwards = new double[] {feedforwards[1], feedforwards[0]};
                }

                break;
            case kRotational:
                break;
        };

        return feedforwards;
    }

    /**
     * Flip an array of drive feedforward X components for the other side of the field. Only does
     * anything if mirrored symmetry is used
     *
     * @param feedforwardXs Array of drive feedforward X components
     * @return The flipped feedforward X components
     */
    public static double[] flipFeedforwardXs(double[] feedforwardXs) {
        return flipFeedforwards(feedforwardXs);
    }

    /**
     * Flip an array of drive feedforward Y components for the other side of the field. Only does
     * anything if mirrored symmetry is used
     *
     * @param feedforwardYs Array of drive feedforward Y components
     * @return The flipped feedforward Y components
     */
    public static double[] flipFeedforwardYs(double[] feedforwardYs) {
        double[] flippedFeedforwardYs = flipFeedforwards(feedforwardYs);
        switch (symmetryType) {
            case kMirrored:
                // Y directions also need to be inverted
                for (int i = 0; i < flippedFeedforwardYs.length; ++i) {
                    flippedFeedforwardYs[i] *= -1;
                }

                break;
            case kRotational:
                break;
        };

        return flippedFeedforwardYs;
    }
}
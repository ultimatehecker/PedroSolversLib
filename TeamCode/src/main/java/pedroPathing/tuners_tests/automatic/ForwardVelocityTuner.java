package pedroPathing.tuners_tests.automatic;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import pedroPathing.Constants;

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 * You can adjust the distance the robot will travel on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous(name = "Forward Velocity Tuner", group = "Automatic Tuners")
public class ForwardVelocityTuner extends OpMode {
    private ArrayList<Double> velocities = new ArrayList<>();
    private Follower follower;
    
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private boolean end;

    /**
     * This initializes the drive motors as well as the cache of velocities and the FTC Dashboard
     * telemetry.
     */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        telemetry.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
        telemetry.addLine("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetry.addLine("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
        telemetry.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetry.addData("pose", follower.getPose());
        telemetry.update();

    }

    /**
     * This starts the OpMode by setting the drive motors to run forward at full power.
     */
    @Override
    public void start() {
        follower.startTeleopDrive(false);
        end = false;
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(1,1,1,true);

        if (gamepad1.cross || gamepad1.a) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();

        if (!end) {
            if (Math.abs(follower.getPose().getX()) > DISTANCE) {
                end = true;
                stopRobot();
            } else {
                double currentVelocity = Math.abs(MathFunctions.dotProduct(follower.getVelocity(), new Vector(1, 0)));
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            stopRobot();
            double average = 0;
            for (Double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetry.addData("forward velocity:", average);
            telemetry.update();
        }
    }

    /**
     * This stops the OpMode by setting the drive motors to run at 0 power.
     */
    public void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }
}

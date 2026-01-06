package meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CloseBlueAutoTest {

    //Location for the robot to start
    //TODO Change if starting close/far and for red/blue
    static double startX = -56;
    static double startY = -43;
    static double startHeading = 45; //In degrees
    static Pose2d startPose = new Pose2d(startX,startY,Math.toRadians(startHeading));

    //Position and heading the robot needs to be to launch the artifact
    //TODO Find where bc right now the position is a complete guess
    static double launchX = -16;
    static double launchY = -16;
    static double launchHeading = Math.toRadians(360); //In degrees
    static Vector2d launchPose = new Vector2d(launchX,launchY);

    public static void main(String[] args) {
        runPath();
    }

    public static void runPath(){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .waitSeconds(0)
                .splineTo(launchPose,Math.toRadians(45))
                //Shoot (waitSeconds is placeholder for shooting)
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-12,-32),Math.toRadians(-90))
                //Pick up artifacts
                .strafeToLinearHeading(new Vector2d(-12,-52), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-12,-40), Math.toRadians(-90))
                .splineTo(launchPose,Math.toRadians(225))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(12,-32),Math.toRadians(-90))
                //Pick up artifacts
                .strafeToLinearHeading(new Vector2d(12,-52), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12,-40), Math.toRadians(-90))
                .splineTo(launchPose,Math.toRadians(225))
                .waitSeconds(4)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}
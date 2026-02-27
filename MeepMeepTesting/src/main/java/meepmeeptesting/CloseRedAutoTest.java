package meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class CloseRedAutoTest {

    //Location for the robot to start
    //TODO Change if starting close/far and for red/blue
    static double startX = -60;
    static double startY = 37;
    static double startHeading = 0; //In degrees
    static Pose2d startPose = new Pose2d(startX,startY,Math.toRadians(startHeading));

    //Position and heading the robot needs to be to launch the artifact
    //TODO Find where bc right now the position is a complete guess
    static double launchX = -24;
    static double launchY = 24;
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
                        .strafeToLinearHeading(launchPose,Math.toRadians(-225))
                        //Shoot (waitSeconds is placeholder for shooting)
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-7,20),Math.toRadians(90))
                        //Pick up artifacts
                        .strafeToLinearHeading(new Vector2d(-7,54), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-7,40), Math.toRadians(90))
                        .strafeToLinearHeading(launchPose,Math.toRadians(-225))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(15,20),Math.toRadians(90))
                        //Pick up artifacts
                        .strafeToLinearHeading(new Vector2d(15,58), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(15,40), Math.toRadians(90))
                        .strafeToLinearHeading(launchPose,Math.toRadians(-225))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-24,45),Math.toRadians(90))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}
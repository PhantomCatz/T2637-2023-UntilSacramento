package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

/*****************************************************************************************
*
* Autonomous selections
* 
*****************************************************************************************/
public class CatzAutonomousPaths
{  
    private final SendableChooser<Boolean> chosenAllianceColor = new SendableChooser<>();
    private final SendableChooser<Integer> chosenPath          = new SendableChooser<>();

    /*------------------------------------------------------------------------------------
    *  Field Relative angles when robot is TBD - Finish Comment  
    *-----------------------------------------------------------------------------------*/
    private final double FWD_OR_BWD = 0.0;
    private final double RIGHT      =   -90.0; 
    private final double LEFT       =   90.0;
    private final double BIAS_OFFSET = -2;
    

    private final double MIN_DIST        =  20.0;   //TBD - This doesn't make sense, if we want to do this, then define as waypoint to waypoint
    private final double MAX_DIST        = 100.0;

    private final double GP_TO_GP        = 48.0;    //TBD not sure what GP is referring to


    private final double INDEXER_EJECT_TIME = 0.5;  //TBD - Put in Indexer

    /*------------------------------------------------------------------------------------
    *  Path ID's
    *-----------------------------------------------------------------------------------*/
    private final int LEFT_SCORE_1                        = 1;
    private final int LEFT_SCORE_2                        = 2;
    private final int LEFT_SCORE_1_BALANCE                = 3;
    private final int LEFT_SCORE_2_BALANCE                = 4;

    private final int CENTER_SCORE_1                        = 20;
    private final int CENTER_SCORE_1_BALANCE                = 21;

    private final int RIGHT_SCORE_1                        = 40;
    private final int RIGHT_SCORE_2                        = 41;
    private final int RIGHT_SCORE_1_BALANCE                = 42;
    private final int RIGHT_SCORE_2_BALANCE                = 43;

    private final int TEST                  = 100;



    public static int pathID;

     /*  DRIVE STRAIGHT VALUES: 
     * if distance > 70, then FAST, else SLOW
     * 8 second maxTime is an arbitrary number, subject to change upon more testing 
     * only robot backwards movement has negative signs over distance and maxspeed
     * left and right distances and max speed aren't negative
     * TEMP_DECEL_DIST decelDistance is an arbitrary number, subject to change upon more testing
     * 
     *   *note* - autonomous is 15 seconds, meaning that all of this will have to finsih within that time
     *          - THIS CODE IS MADE FOR BLUE SIDE 
     *          - FOR RED, CHANGE LEFTS WITH RIGHTS AND RIGHTS WITH LEFTS (from blue)
     *          - movement similar to code.org level programming
    */

    /* PATH NAME:
     *    /CenterRightTunnel/
     * - CenterRight (Starting Position)
     * - Tunnel (type of movement/movement path)
     */

    /* Distances:          -______-
     * drive.DriveStraight(distance, decelDistance, maxSpeed, wheelPos, maxTime);
     *  - 224 = distance from grid to center pieces
     *                
     */
    // drive.DriveStraight(distance, decelDist, )


    public CatzAutonomousPaths()
    {
        chosenAllianceColor.setDefaultOption("Blue Alliance", Robot.constants.BLUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  Robot.constants.RED_ALLIANCE);
        SmartDashboard.putData              ("Alliance Color", chosenAllianceColor);

        chosenPath.setDefaultOption("Left Score 1",           LEFT_SCORE_1);   
        chosenPath.addOption       ("Left Score 2",           LEFT_SCORE_2);
        chosenPath.addOption       ("Left Score 1 Balance",   LEFT_SCORE_1_BALANCE);

        chosenPath.addOption       ("Center Score 1",         CENTER_SCORE_1);
        chosenPath.addOption       ("Center Score 1 Balance", CENTER_SCORE_1_BALANCE);

        chosenPath.addOption       ("Right Score 1",          RIGHT_SCORE_1);
        chosenPath.addOption       ("Right Score 2",          RIGHT_SCORE_2);
        chosenPath.addOption       ("Right Score 1 Balance",  RIGHT_SCORE_1_BALANCE);

        chosenPath.addOption       ("TEST PATH",  TEST);

        SmartDashboard.putData     ("Auton Path", chosenPath);
    }


    public void executeSelectedPath()
    {
        pathID = chosenPath.getSelected();

 
        switch (pathID)
        {
            case LEFT_SCORE_1: SideScore1();
            break;

            case LEFT_SCORE_2: LeftScore2();
            break;

            case LEFT_SCORE_1_BALANCE: LeftScore1Balance();
            break;

            case CENTER_SCORE_1: CenterScore1();
            break;

            case CENTER_SCORE_1_BALANCE: CenterScore1Balance();
            break;

            case RIGHT_SCORE_1: SideScore1();
            break;

            case RIGHT_SCORE_2: RightScore2();
            break;

            case RIGHT_SCORE_1_BALANCE: RightScore1Balance();
            break;

            case TEST: testPath();
            break;
        }

    }

    public void testPath()
    {
        scoreCubeIndexer();
        Robot.auton.DriveStraight(25, RIGHT, 5.0);
        
        

    }

    

    public void SideScore1()
    {
        double direction;

        scoreCubeIndexer();

        Robot.auton.DriveStraight(100, LEFT, 2.0);     //From Grid to area to do 180 deg turn
        Timer.delay(2.0);
        Robot.auton.DriveStraight(100, RIGHT, 2.0);     //From Grid to area to do 180 deg turn

    }

    public void sideScore2()
    {

        scoreCubeIndexer();

        Robot.auton.DriveStraight( 30, FWD_OR_BWD, 2.0);     //From Grid to area to do 180 deg turn

        Robot.auton.TurnInPlace(180, 2);

        deployAndRunIntake();
        Timer.delay(0.75);
        
        Robot.auton.DriveStraight(200, FWD_OR_BWD, 5.0);
        Timer.delay(0.50);
        stopAndStowIntake();
        Timer.delay(0.75);


        Robot.auton.TurnInPlace(180, 2);

        Robot.auton.DriveStraight(-210, FWD_OR_BWD, 5.0);
    }


    
    public void LeftScore2()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = RIGHT;
        }
        else
        {
            direction = LEFT;
        }

        sideScore2();

        Robot.auton.DriveStraight( 48,  direction, 2.0);    //Move in front of center node
        Robot.auton.DriveStraight(-25, FWD_OR_BWD, 2.0);    //Move up to center node

        scoreCubeIndexer();
    }

        
    public void RightScore2()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = LEFT;
        }
        else
        {
            direction = RIGHT;
        }

        sideScore2();

        Robot.auton.DriveStraight( 48,  direction, 2.0);    //Move in front of center node
        Robot.auton.DriveStraight(-25, FWD_OR_BWD, 2.0);    //Move up to center node

        scoreCubeIndexer();
    }




    public void LeftScore1Balance()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = RIGHT;
        }
        else
        {
            direction = LEFT;
        }

        scoreCubeIndexer();

        Robot.auton.DriveStraight(180, FWD_OR_BWD+BIAS_OFFSET, 7.0);     //From Grid to area to do 180 deg turn

        Robot.auton.TurnInPlace(180, 2);

        deployAndRunIntake();
        Timer.delay(0.75);
        Robot.auton.DriveStraight(50, FWD_OR_BWD+BIAS_OFFSET, 2.0);
        Timer.delay(0.5);
        stopAndStowIntake();
        Timer.delay(0.75);
        Robot.auton.TurnInPlace(180, 2);
        Robot.auton.DriveStraight(-70, FWD_OR_BWD+BIAS_OFFSET, 5.0);
        Robot.auton.DriveStraight(60, direction, 2.0);
        Robot.auton.DriveStraight(-80, FWD_OR_BWD+BIAS_OFFSET, 2.0);
        Balance();
        scoreCubeIndexer();

    }


    public void RightScore1Balance()
    {
        double direction;

        if(chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
        {
            direction = LEFT;
        }
        else
        {
            direction = RIGHT;
        }

        scoreCubeIndexer();

        Robot.auton.DriveStraight(30, FWD_OR_BWD, 2.0);     //From Grid to area to do 180 deg turn

        Robot.auton.TurnInPlace(180, 2);

        Robot.auton.DriveStraight(150, FWD_OR_BWD, 5.0);
        deployAndRunIntake();
        Robot.auton.DriveStraight(50, FWD_OR_BWD, 2.0);
        stopAndStowIntake();
        Robot.auton.TurnInPlace(180, 2);
        Robot.auton.DriveStraight(-70, FWD_OR_BWD, 5.0);
        Robot.auton.DriveStraight(48, direction, 2.0);
        Robot.auton.DriveStraight(-80, FWD_OR_BWD, 2.0);
        Balance();
        scoreCubeIndexer();

    }

    public void CenterScore1Balance() 
    {
        scoreCubeIndexer();
        Robot.auton.DriveStraight(100, FWD_OR_BWD-7, 4.0);
        Balance();
    }

    public void CenterScore1() 
    {
        scoreCubeIndexer();
        Robot.auton.DriveStraight(200, FWD_OR_BWD, 4.0);
    }

    public void Balance()
    {
        Robot.balance.StartBalancing();
    }

    public void deployAndRunIntake()
    {
        Robot.intake.intakeDeploy();
        Robot.intake.intakeRollerIn();
    }

    public void stopAndStowIntake() 
    {
        Robot.intake.intakeRollerOff();
        Robot.intake.intakeStow();
    }

    public void scoreCubeIndexer()
    {
        Robot.indexer.indexerFrntEject();
        Timer.delay(INDEXER_EJECT_TIME);
        Robot.indexer.indexerFrntStop();
    }
}
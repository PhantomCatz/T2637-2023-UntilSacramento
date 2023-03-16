package frc.DataLogger;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.String;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

public class DataCollection 
{	
    Date date;	
    SimpleDateFormat sdf;
    String dateFormatted;

    public boolean fileNotAppended = false;

    //decide the location and extender
    public final String logDataFilePath = "//media//sda1//RobotData";
    public final String logDataFileType = ".csv";

    private       Thread  dataThread;

    public        boolean logDataValues = false;   
    public static int     logDataID;

    public ArrayList<CatzLog> logData;

    StringBuilder sb = new StringBuilder();

    private final double LOG_SAMPLE_RATE = 0.1;

    public static final int LOG_ID_NONE            = 0;
    public static final int LOG_ID_SWERVE_STEERING = 1;
    public static final int LOG_ID_SWERVE_DRIVING  = 2;
    public static final int LOG_ID_BALANCE         = 3;
    public static final int LOG_ID_INTAKE          = 4;
    public static final int LOG_ID_INDEXER         = 5;
    public static final int LOG_ID_ELEVATOR        = 6;
    public static final int LOG_ID_CLAW            = 7;
    public static final int LOG_ID_DRV_STRAIGHT    = 8;
    public static final int LOG_ID_TURN_IN_PLACE   = 9;

    public boolean validLogID = true;

    private final String LOG_HDR_SWERVE_STEERING = "time,target,lf-angle,lf-err,lf-flip-err,lb-angle,lb-err,lb-flip-err,rf-angle,rf-err,rf-flip-err,rb-angle,rb-err,rb-flip-err";
    private final String LOG_HDR_SWERVE_DRIVING = "time,target,lf-angle,lf-dist,lf-vel,lb-angle,lb-dist,lb-vel,rf-angle,rf-dist,rf-vel,rb-angle,rb-dist,rb-vel,lf-error";
    private final String LOG_HDR_BALANCE_MOD = "time,pitch,rate,power,pitchTerm,rateTerm,";
    private final String LOG_HDR_INTAKE = "time,target-angle,current-angle,intake-stow-count,intake-motor-output-percent,target-angular-rate,angle-dot,final-motor-pwr,pwr-for-mtr,dt-ang,dt-time,state,targetAngAccel";
    private final String LOG_HDR_INDEXER = "";
    private final String LOG_HDR_ELEVATOR = "time,raw-enc-val,cur-piv-agl,piv-lop-err,stow-hold-err,piv-mtr-pwr,piv-abs-pos,talon-cnts,crg-pos-inch";
    private final String LOG_HDR_CLAW = "";
    private final String LOG_HDR_DRV_STRAIGHT = "time, delta-pos-cn,dis-rem-in,drv-pwr-kp,drv-pwr-clmp,drv-pwr,curr-angl,angl-err,angl-err-rate,turn-pwr-kp,turn-pwr-kd,turn-pwr,lf-mod-angl";
    private final String LOG_HDR_TURN_IN_PLACE = "time, cur-angle, cur-err, mtr-pwr";

    public String logStr;

    public static String mechanismName = "Not Set";

    public static final SendableChooser<Integer> chosenDataID = new SendableChooser<>();

    public static int boolData = 0;

    public static final int shift0 = 1 << 0;
    public static final int shift1 = 1 << 1;
    public static final int shift2 = 1 << 2;
    public static final int shift3 = 1 << 3;
    public static final int shift4 = 1 << 4;
    public static final int shift5 = 1 << 5;
    public static final int shift6 = 1 << 6;
    public static final int shift7 = 1 << 7;

    public DataCollection()
    {
        dataCollectionShuffleboard();
    }

    public void updateLogDataID()
    {
        if(chosenDataID.getSelected() == LOG_ID_NONE)
        {
            stopDataCollection();
        }
        else
        {
            startDataCollection();
        }
        setLogDataID(chosenDataID.getSelected());

    }

    public void setLogDataID(final int dataID)
    {
        logDataID = dataID;
    }

    
    public void dataCollectionInit(final ArrayList<CatzLog> list)
    {   
        date = Calendar.getInstance().getTime();
        sdf = new SimpleDateFormat("yyyy-MM-dd kk.mm.ss");	
        dateFormatted = sdf.format(date);

        logData = list;

        //dataCollectionShuffleboard();

        dataThread = new Thread( () ->
        {
            while(!Thread.interrupted())
            {   
                if(logDataValues == true)
                {
                    collectData(logDataID);
                } 
                else if (logDataValues == false) 
                {

                } 

                Timer.delay(LOG_SAMPLE_RATE);

            }

        } );

        dataThread.start();
    }

    /*-----------------------------------------------------------------------------------------
    *  Initialize drop down menu for data collection on Shuffleboard
    *----------------------------------------------------------------------------------------*/
    public void dataCollectionShuffleboard()
    {
        chosenDataID.setDefaultOption("None",        LOG_ID_NONE);
        
        chosenDataID.addOption("Swerve Steering", LOG_ID_SWERVE_STEERING);
        chosenDataID.addOption("Swerve Driving", LOG_ID_SWERVE_DRIVING);
        chosenDataID.addOption("Balance Data",       LOG_ID_BALANCE);
        chosenDataID.addOption("Intake", LOG_ID_INTAKE);
        chosenDataID.addOption("Indexer", LOG_ID_INDEXER);
        chosenDataID.addOption("Elevator", LOG_ID_ELEVATOR);
        chosenDataID.addOption("Claw", LOG_ID_CLAW);
        chosenDataID.addOption("Auton Drv Straight", LOG_ID_DRV_STRAIGHT);
        chosenDataID.addOption("Auton Turn In Place", LOG_ID_TURN_IN_PLACE);

        SmartDashboard.putData("Data Collection", chosenDataID);
    
    }

    public void startDataCollection() 
    {
        logDataValues = true;
    }

    public void stopDataCollection() 
    {
        logDataValues = false; 
    }

    public void collectData(final int dataID)
    {
        CatzLog data;
        double data1 = -999.0;
        double data2 = -999.0;
        double data3 = -999.0;
        double data4 = -999.0;
        double data5 = -999.0;
        double data6 = -999.0;
        double data7 = -999.0;
        double data8 = -999.0;
        double data9 = -999.0;
        double data10 = -999.0;
        double data11 = -999.0;
        double data12 = -999.0;
        double data13 = -999.0;
        double data14 = -999.0;
        int data15    = -999;
        //double data16 = -999.0;


        //define each data
        switch (dataID) 
        {
            case LOG_ID_SWERVE_STEERING :    
                break;
            case LOG_ID_SWERVE_DRIVING :    
                break;
            case LOG_ID_BALANCE: 
                break;
            case LOG_ID_INTAKE:
                break;
            case LOG_ID_INDEXER:
                break;
            case LOG_ID_ELEVATOR:
                break;
            case LOG_ID_CLAW:
                break;
            case LOG_ID_DRV_STRAIGHT:
                break;
            case LOG_ID_TURN_IN_PLACE:
                break;
            default :
                validLogID = false;

        }
    }

    public static void resetBooleanData()
    {
        boolData = 0;
    }

    public static void booleanDataLogging(boolean bool1, int bitPos)
    {
        if(bool1 == true)
        {
            boolData |= (1 << bitPos);
        }
    }
    
    public void writeHeader(PrintWriter pw) 
    {
        switch (logDataID)
        {
            case LOG_ID_SWERVE_STEERING:
                pw.printf(LOG_HDR_SWERVE_STEERING);
                break;
            case LOG_ID_SWERVE_DRIVING:
                pw.printf(LOG_HDR_SWERVE_DRIVING);
                break;
            case LOG_ID_BALANCE:
                pw.printf(LOG_HDR_BALANCE_MOD);
            break;
            case LOG_ID_INTAKE:
                pw.printf(LOG_HDR_INTAKE);
            break;
            case LOG_ID_INDEXER:
                pw.printf(LOG_HDR_INDEXER);
            break;
            case LOG_ID_ELEVATOR:
                pw.printf(LOG_HDR_ELEVATOR);
            break; 
            case LOG_ID_CLAW:
                pw.printf(LOG_HDR_CLAW);
            break;
            case LOG_ID_DRV_STRAIGHT:
                pw.printf(LOG_HDR_DRV_STRAIGHT);
            break;
            case LOG_ID_TURN_IN_PLACE:
                pw.printf(LOG_HDR_TURN_IN_PLACE);
            break;
            default :
                pw.printf("Invalid Log Data ID");            


        }
    }
    
    //create log file
    public String createFilePath()
    {
        String logDataFullFilePath = logDataFilePath + " " + setLogDataName() + " " + dateFormatted +  logDataFileType;
    	return logDataFullFilePath;
    }

    // print out data after fully updated
    public void exportData(ArrayList<CatzLog> data) throws IOException
    {   
        System.out.println("Export Data ///////////////");    
        try (
            
        FileWriter     fw = new FileWriter(createFilePath(), fileNotAppended);
        BufferedWriter bw = new BufferedWriter(fw);
        PrintWriter    pw = new PrintWriter(bw))

        {
            writeHeader(pw);
            pw.print("\n");

            // loop through arraylist and adds it to the StringBuilder
            int dataSize = data.size();
            for (int i = 0; i < dataSize; i++)
            {
                pw.print(data.get(i).toString() + "\n");
                pw.flush();
            }

            pw.close();
        }
    }

    public static int getLogDataID()
    {
        return logDataID;
    }

    public static String setLogDataName()
    {
        switch(getLogDataID())
        {
            case(LOG_ID_SWERVE_STEERING):
                mechanismName = "Swerve Steering Data";
            break;
            case(LOG_ID_SWERVE_DRIVING):
                mechanismName = "Swerve Driving Data";
            break; 
            case(LOG_ID_BALANCE):
                mechanismName = "Balancing Data";
            break;
            case(LOG_ID_INTAKE):
                mechanismName = "Intake Data";
            break;
            case(LOG_ID_INDEXER):
                mechanismName = "Indexer Data";
            break;
            case(LOG_ID_ELEVATOR):
                mechanismName = "Elevator Data";
            break;
            case(LOG_ID_CLAW):
                mechanismName = "Claw Data";
            break;
            case(LOG_ID_DRV_STRAIGHT):
                mechanismName = "Auton Drv Straight Data";
            break;
            case(LOG_ID_TURN_IN_PLACE):
                mechanismName = "Auton Turn In Place Data";
            break;
        }
        return mechanismName;
    }
}
package frc.Mechanisms;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.Robot;

public class CatzRGB 
{
    private final int LED_COUNT = 54;

    private final int LED_PWM_PORT = 0;
    
    private AddressableLED led;   

    private AddressableLEDBuffer ledBuffer;
    
    
    private Color RED    = Color.kRed;
    private Color ORANGE = Color.kOrange;
    private Color YELLOW = Color.kYellow;
    private Color GREEN  = Color.kGreen;
    private Color BLUE   = Color.kBlue;
    private Color PURPLE = Color.kPurple;
    private Color WHITE  = Color.kWhite;
    private Color TEAM_COLOR = Color.kDodgerBlue;
    private Color PINK  = Color.kHotPink;   

    private int FlashDelayCounter = 0;
    private int FLowDelayCounter = 0;
    private int PingpongDelayCounter = 0;

    private final int FLASH_DELAY   = 15;
    private final int FLOW_DELAY   = 3;
    private final int PINGPONG_DELAY   = 1;

    private final int PINGPONG_RADIUS = 5;
    private final int FLOW_WIDTH = 6;

    private int flowCounter = 0;
    private int PingpongCounter = 0;

    private boolean PingpongUp = true;

    private int nextFlashColor = 1;

    public CatzRGB()
    {
        led = new AddressableLED(LED_PWM_PORT);
       // ledBack = new AddressableLED(LED_PWM_PORT_BACK);

        ledBuffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
      

    }


    public void LEDWork()
    {
        if(Robot.isInDisabled() == true)
        {
            pingpong(TEAM_COLOR, WHITE);
        }
        else if(Robot.isInAuton() == true)
        {
            if(Robot.balance.startBalance == true)
            {
                solidColor(GREEN);
            }
            else
            {
                flowUp(TEAM_COLOR,WHITE);
            }
        }
        else if(Robot.indexer.isConeDetected == true)
        {
            solidColor(PURPLE);
        }
        else if(Robot.indexer.isConeDetected == true)
        {
            solidColor(YELLOW);
        }
        else if(Robot.cubeRequest == true)
        {
            flash(PURPLE,WHITE);
        }
        else if (Robot.coneRequest == true)
        {
            flash(YELLOW,WHITE);
        }
        else if(Robot.cubeScoringReady == true)
        {
            flowDown(PURPLE,WHITE);
        }
        else if(Robot.coneScoringReady == true)
        {
            flowDown(YELLOW,WHITE);
        }
        else if(Robot.noGamePiece == true)
        {
            flowDown(TEAM_COLOR,WHITE);
        }
        
        led.setData(ledBuffer);
        
        
        
    }

    public void flowUp(Color color1, Color color2)
    {
        FLowDelayCounter++;
        if(FLowDelayCounter > FLOW_DELAY)
        {
            for(int i = 0; i < LED_COUNT; i++)
        {
            if((i-flowCounter) % (FLOW_WIDTH*2) < FLOW_WIDTH)
            {
                ledBuffer.setLED(i, color1);
            }
            else
            {
                ledBuffer.setLED(i, color2);
            }
        }

        flowCounter++;
        if(flowCounter >= (FLOW_WIDTH*2))
        {
            flowCounter = 0;
        }

            FLowDelayCounter = 0;
        }
    }

    public void flowDown(Color color1, Color color2)
    {
        FLowDelayCounter++;
        if(FLowDelayCounter > FLOW_DELAY)
        {
            for(int i = 0; i < LED_COUNT; i++)
        {
            if((i+flowCounter) % (FLOW_WIDTH*2) < FLOW_WIDTH)
            {
                ledBuffer.setLED(i, color1);
            }
            else
            {
                ledBuffer.setLED(i, color2);
            }
        }

        flowCounter++;
        if(flowCounter >= (FLOW_WIDTH*2))
        {
            flowCounter = 0;
        }

            FLowDelayCounter = 0;
        }
    }

    public void flash(Color color1, Color color2)
    {
        FlashDelayCounter++;
        if(FlashDelayCounter > FLASH_DELAY)
        {
            if(nextFlashColor == 1)
            {
                solidColor(color1);
                nextFlashColor = 2;
            }
            else
            {
                solidColor(color2);
                nextFlashColor = 1;
            }

            FlashDelayCounter = 0;
        }
    }

    public void pingpong(Color color1, Color color2)
    {
        PingpongDelayCounter++;
        if(PingpongDelayCounter > PINGPONG_DELAY)
        {
            for(int i = 0; i < LED_COUNT; i++)
            {
                if(i<=(PingpongCounter+PINGPONG_RADIUS) && i>=(PingpongCounter-PINGPONG_RADIUS))
                {
                    ledBuffer.setLED(i, color1);
                }
                else
                {
                    ledBuffer.setLED(i, color2);
                }
            }

            if(PingpongUp)
            {
                PingpongCounter++;
            }
            else
            {
                PingpongCounter--;
            }

            if(PingpongCounter >= LED_COUNT)
            {
                PingpongUp = false;
            }
            else if(PingpongCounter < 0)
            {
                PingpongUp = true;
            }

            PingpongDelayCounter = 0;
        }
    }

    public void solidColor(Color color)
    {
        for(int i = 0; i < LED_COUNT; i++)
        {
            ledBuffer.setLED(i, color);
        }
    }

    
}
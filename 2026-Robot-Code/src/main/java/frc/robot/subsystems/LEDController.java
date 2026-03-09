/* package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import com.revrobotics.CIEColor;
import com.reduxrobotics.sensors.canandcolor.wpistruct.*;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.servohub.config.ServoChannelConfigAccessor;
import com.revrobotics.spark.config.SparkParameters;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SignalsConfigAccessor;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.;



import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ledColor;





public class LEDController extends SubsystemBase {
  private double m_Color = 0.0;
  private Spark m_ledController = new SparkMax(0);

  /**
   * Creates a new ledController.
   */



/*
  public LEDController() {
    SmartDashboard.putNumber("LEDController/color", m_Color);
  
  }

  @Override
  public void periodic() {
    //m_Color = SmartDashboard.getNumber("LEDController/color", m_Color);
    m_ledController.set(m_Color);
  }


  public void setColor(ledColor color) {
    switch(color) {
      case kIndexerFull:
        m_Color = 0.65; // ORANGE
        break;
      case kTargetFound:
        m_Color = 0.69; // YELLOW
        break;
      case kTargetNotFound:
        m_Color = 0.61; // RED
        break;
      case kOnTarget:
        m_Color = 0.77; // GREEN
        break;
      case kSlow:
        m_Color = 0.87; // BLUE
        break;
      case kNormal:
      default:
        m_Color = 0.91; // VIOLET
    }
  }

}








/*
//import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;


public class LedBlinkinFloor {

	Spark blinkin;

	public Blinkin() {
		blinkin = new Spark(ElectricalLayout.LIGHTS);
	}
		
	
	//if the robot is not in hatMode and in normal drive, the LED turns solid white (0.93)
	public void lightsWhite() {
		blinkin.set(0.93);
	}
	
	
	 //if the robot detects the cube, the LED blinks gold (-0.07)
	
	public void lightsGold() {
		blinkin.set(-0.07); 
	}

}
	*/

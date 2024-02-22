package frc.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

/*
 * Classe para abstrair os diversos modelos de Joystick que existem
 * não necessitando dessa forma, alterações no código quando pluga um Joystick diferente
 * 
 * @author  Marlon Andrei
 * 
 */
public class JoystickGeneric {
    //private final Dashboard log = new Dashboard();

    XboxController jXbox;
    Joystick jLogitech;
    PS4Controller jPs4;

    int jLogXboxPs4 = 0;

    public JoystickGeneric() {

        jLogitech = new Joystick(OIConstants.kDriverControllerPort);
        jXbox = new XboxController(OIConstants.kDriverControllerPort);  
        jPs4 = new PS4Controller(OIConstants.kDriverControllerPort);

        if (jLogitech.getName().toLowerCase().indexOf("logitech") != 0) {
            jLogXboxPs4 = 0;        
        } else if (jLogitech.getName().toLowerCase().indexOf("xbox") != 0) {
            jLogXboxPs4 = 1;        
        } else if (jLogitech.getName().toLowerCase().indexOf("ps4") != 0) {
            jLogXboxPs4 = 2;        
        }

    }

    public GenericHID getController() {
        GenericHID ret = new GenericHID(OIConstants.kDriverControllerPort);

        if (jLogXboxPs4 == 0) {
            ret = jLogitech;
        } else if (jLogXboxPs4 == 1) {
            ret = jXbox;
        } else if (jLogXboxPs4 == 2) {
            ret = jPs4;
        }

        return ret;
    }

    public double getLeftX() {
        double ret = -1;

        if (jLogXboxPs4 == 0) {
            ret = jLogitech.getRawAxis(1);
        } else if (jLogXboxPs4 == 1) {
            ret = jXbox.getLeftX();
        } else if (jLogXboxPs4 == 2) {
            ret = jPs4.getLeftX();
        }

        return ret;
    }

    public double getLeftY() {
        double ret = -1;

        if (jLogXboxPs4 == 0) {
            ret = jLogitech.getRawAxis(0);
        } else if (jLogXboxPs4 == 1) {
            ret = jXbox.getLeftY();
        } else if (jLogXboxPs4 == 2) {
            ret = jPs4.getLeftY();
        }

        return ret;
    }

    public double getRightX() {
        double ret = -1;

        if (jLogXboxPs4 == 0) {
            ret = jLogitech.getRawAxis(2);
        } else if (jLogXboxPs4 == 1) {
            ret = jXbox.getRightX();
        } else if (jLogXboxPs4 == 2) {
            ret = jPs4.getRightX();
        }

        return ret;
    }



}

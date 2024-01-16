// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Controller class for our guitar hero controller. */
public class GuitarController extends CommandGenericHID {
    public GuitarController(int port) {
        super(port);
        
    }

    /** @return A Trigger representing the green fret button. */
    public Trigger greenFret() { return button(Constants.Controllers.Guitar.GREEN_FRET); }

    /** @return A Trigger representing the red fret button. */
    public Trigger redFret() { return button(Constants.Controllers.Guitar.RED_FRET); }

    /** @return A Trigger representing the yellow fret button. */
    public Trigger yellowFret() { return button(Constants.Controllers.Guitar.YELLOW_FRET); }

    /** @return A Trigger representing the blue fret button. */
    public Trigger blueFret() { return button(Constants.Controllers.Guitar.BLUE_FRET); }

    /** @return A Trigger representing the orange fret button. */
    public Trigger orangeFret() { return button(Constants.Controllers.Guitar.ORANGE_FRET); }


    /** @return A Trigger representing the start button (plus) */
    public Trigger start() { return button(Constants.Controllers.Guitar.START_BUTTON ); }

    /** @return A Trigger representing the select button (minus) */
    public Trigger select() { return button(Constants.Controllers.Guitar.SELECT_BUTTON ); }
}

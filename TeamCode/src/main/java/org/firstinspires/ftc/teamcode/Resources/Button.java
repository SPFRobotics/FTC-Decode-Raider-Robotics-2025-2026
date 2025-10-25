package org.firstinspires.ftc.teamcode.Resources;


public class Button {
    private boolean stillPressed = false;
    private boolean activated = false;


    public boolean toggle(boolean button){
        if (!button){
            stillPressed = false;
        }
        if (button && !stillPressed && !activated){
            activated = true;
            stillPressed = true;
        }
        else if (button && !stillPressed && activated) {
            activated = false;
            stillPressed = true;
        }
        return activated;
    }

    public boolean toggle(int button) {
        if (button == 0) {
            stillPressed = false;
        }
        if (button == 1 && !stillPressed && !activated) {
            activated = true;
            stillPressed = true;
        } else if (button == 1 && !stillPressed && activated) {
            activated = false;
            stillPressed = true;
        }
        return activated;
    }

    public boolean press(boolean button){
        if (!button){
            stillPressed = false;
        }
        if (button && !stillPressed){
            stillPressed = true;
            return true;
        }
        else{
            return false;
        }
    }

    public boolean press(int button) {
        if (button == 0) {
            stillPressed = false;
        }
        if (button == 1 && !stillPressed && !activated) {
            activated = true;
            stillPressed = true;
        }
        return activated;
    }
    public void changeState(boolean x){
        activated = x;
    }

    public boolean getState(){
        return activated;
    }
}

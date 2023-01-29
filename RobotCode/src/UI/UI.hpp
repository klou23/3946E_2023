/**
 * File: UI.hpp
 * Author: Kevin Lou
 * Created On: Jan 3 2023
 * Last Updated: Jan 3 2023
 *
 * Copyright (c) 2023 3946E
 *
 * Summary of File:
 *  - Contains all functions for the UI
 */

#ifndef ROBOTCODE_UI_HPP
#define ROBOTCODE_UI_HPP

#include "UIVars.hpp"
#include "../Autonomous/Autonomous.hpp"

namespace UI{

    void setAuton(){
        if(progButton->isToggled()){
            auton1Button->setState(LouUI::ToggleButton::RELEASED);
            auton2Button->setState(LouUI::ToggleButton::RELEASED);
            auton3Button->setState(LouUI::ToggleButton::RELEASED);
            auton4Button->setState(LouUI::ToggleButton::RELEASED);
            Autonomous::autonSide = Autonomous::PROG;
            Autonomous::autonNumber = 0;
            autonDescription->setText(Autonomous::progDescription);
            return;
        }

        if(leftButton->isToggled()) Autonomous::autonSide = Autonomous::LEFT;
        else if(rightButton->isToggled()) Autonomous::autonSide = Autonomous::RIGHT;
        else Autonomous::autonSide = Autonomous::NONE;

        if(auton1Button->isToggled()) Autonomous::autonNumber = 1;
        else if(auton2Button->isToggled()) Autonomous::autonNumber = 2;
        else if(auton3Button->isToggled()) Autonomous::autonNumber = 3;
        else if(auton4Button->isToggled()) Autonomous::autonNumber = 4;
        else Autonomous::autonNumber = 0;

        if(Autonomous::autonSide == Autonomous::NONE){
            autonDescription->setText(Autonomous::noAutonDescription);
        }else if(Autonomous::autonSide == Autonomous::LEFT){
            switch (Autonomous::autonNumber) {
                case 1:
                    autonDescription->setText(Autonomous::left1Description);
                    break;
                case 2:
                    autonDescription->setText(Autonomous::left2Description);
                    break;
                case 3:
                    autonDescription->setText(Autonomous::left3Description);
                    break;
                case 4:
                    autonDescription->setText(Autonomous::left4Description);
                    break;
                default:
                    autonDescription->setText(Autonomous::noAutonDescription);
            }
        }else if(Autonomous::autonSide == Autonomous::RIGHT){
            switch (Autonomous::autonNumber) {
                case 1:
                    autonDescription->setText(Autonomous::right1Description);
                    break;
                case 2:
                    autonDescription->setText(Autonomous::right2Description);
                    break;
                case 3:
                    autonDescription->setText(Autonomous::right3Description);
                    break;
                case 4:
                    autonDescription->setText(Autonomous::right4Description);
                    break;
                default:
                    autonDescription->setText(Autonomous::noAutonDescription);
            }
        }
    }

    lv_res_t selectLeft(lv_obj_t *obj){
        rightButton->setState(LouUI::ToggleButton::RELEASED);
        progButton->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    lv_res_t selectRight(lv_obj_t *obj){
        leftButton->setState(LouUI::ToggleButton::RELEASED);
        progButton->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    lv_res_t selectProg(lv_obj_t *obj){
        leftButton->setState(LouUI::ToggleButton::RELEASED);
        rightButton->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    lv_res_t selectAuton1(lv_obj_t *obj){
        auton2Button->setState(LouUI::ToggleButton::RELEASED);
        auton3Button->setState(LouUI::ToggleButton::RELEASED);
        auton4Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    lv_res_t selectAuton2(lv_obj_t *obj){
        auton1Button->setState(LouUI::ToggleButton::RELEASED);
        auton3Button->setState(LouUI::ToggleButton::RELEASED);
        auton4Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    lv_res_t selectAuton3(lv_obj_t *obj){
        auton1Button->setState(LouUI::ToggleButton::RELEASED);
        auton2Button->setState(LouUI::ToggleButton::RELEASED);
        auton4Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    lv_res_t selectAuton4(lv_obj_t *obj){
        auton1Button->setState(LouUI::ToggleButton::RELEASED);
        auton2Button->setState(LouUI::ToggleButton::RELEASED);
        auton3Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    void initialize(){
        display.addScreen("Auton");
        display.addScreen("Drive");
        display.addScreen("Flywheel");

        /***Auton***/
        leftButton = (new LouUI::ToggleButton(display.getScreen("Auton")))
                ->setPosition(27, 0)
                ->setSize(120, 40)
                ->setMainColor(LouUI::Color(140,140,140), LouUI::ToggleButton::ALL_PRESSED)
                ->setGradientColor(LouUI::Color(140,140,140), LouUI::ToggleButton::ALL_PRESSED)
                ->setMainColor(LouUI::Color(125, 125, 125), LouUI::ToggleButton::ALL_RELEASED)
                ->setGradientColor(LouUI::Color(125, 125, 125), LouUI::ToggleButton::ALL_RELEASED)
                ->setBorderColor(LouUI::Color("WHITE"), LouUI::ToggleButton::ALL)
                ->setBorderOpacity(255, LouUI::ToggleButton::ALL)
                ->setBorderWidth(5, LouUI::ToggleButton::ALL_TOGGLED)
                ->setBorderWidth(0, LouUI::ToggleButton::ALL_UNTOGGLED);
        rightButton = (new LouUI::ToggleButton(display.getScreen("Auton"), *leftButton))
                ->align(leftButton->getObj(), LouUI::OUT_RIGHT_MID, 20, 0);
        progButton = (new LouUI::ToggleButton(display.getScreen("Auton")))
                ->setSize(120, 40)
                ->setMainColor(LouUI::Color(0,140,0), LouUI::ToggleButton::ALL_PRESSED)
                ->setGradientColor(LouUI::Color(0,140,0), LouUI::ToggleButton::ALL_PRESSED)
                ->setMainColor(LouUI::Color(0, 125, 0), LouUI::ToggleButton::ALL_RELEASED)
                ->setGradientColor(LouUI::Color(0, 125, 0), LouUI::ToggleButton::ALL_RELEASED)
                ->setBorderColor(LouUI::Color("WHITE"), LouUI::ToggleButton::ALL)
                ->setBorderOpacity(255, LouUI::ToggleButton::ALL)
                ->setBorderWidth(5, LouUI::ToggleButton::ALL_TOGGLED)
                ->setBorderWidth(0, LouUI::ToggleButton::ALL_UNTOGGLED)
                ->align(rightButton->getObj(), LouUI::OUT_RIGHT_MID, 20, 0);

        auton1Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *leftButton))
                ->setPosition(5, 60)
                ->setSize(100, 40);
        auton2Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *auton1Button))
                ->align(auton1Button->getObj(), LouUI::OUT_RIGHT_MID, 10, 0);
        auton3Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *auton1Button))
                ->align(auton2Button->getObj(), LouUI::OUT_RIGHT_MID, 10, 0);
        auton4Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *auton1Button))
                ->align(auton3Button->getObj(), LouUI::OUT_RIGHT_MID, 10, 0);

        leftButtonLabel = (new LouUI::Label(leftButton->getObj()))
                ->setText("LEFT");
        rightButtonLabel = (new LouUI::Label(rightButton->getObj()))
                ->setText("RIGHT");
        progButtonLabel = (new LouUI::Label(progButton->getObj()))
                ->setText("PROG");
        auton1ButtonLabel = (new LouUI::Label(auton1Button->getObj()))
                ->setText("1");
        auton2ButtonLabel = (new LouUI::Label(auton2Button->getObj()))
                ->setText("2");
        auton3ButtonLabel = (new LouUI::Label(auton3Button->getObj()))
                ->setText("3");
        auton4ButtonLabel = (new LouUI::Label(auton4Button->getObj()))
                ->setText("4");

        leftButton->setAction(selectLeft);
        rightButton->setAction(selectRight);
        progButton->setAction(selectProg);

        auton1Button->setAction(selectAuton1);
        auton2Button->setAction(selectAuton2);
        auton3Button->setAction(selectAuton3);
        auton4Button->setAction(selectAuton4);

        /***Drive***/

        flTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setPosition(20, 0)
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20);

        blTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(flTempGauge->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, -20);

        frTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(flTempGauge->getObj(), LouUI::Align::OUT_RIGHT_MID, 40, 0);

        brTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(frTempGauge->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, -20);


        flTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("FL")
                ->align(flTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        blTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("BL")
                ->align(blTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        frTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("FR")
                ->align(frTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        brTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("BR")
                ->align(brTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        odomLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("Odom")
                ->align(frTempGaugeLabel->getObj(), LouUI::Align::OUT_RIGHT_MID, 120, 15);

        odomXLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("X - 0")
                ->align(odomLabel->getObj(), LouUI::Align::OUT_BOTTOM_MID, 20, 15);

        odomYLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("Y - 0")
                ->align(odomXLabel->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, 10);

        odomThetaLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("T - 0")
                ->align(odomYLabel->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, 10);

        /***Flywheel***/
        flywheelSpeedGauge = (new LouUI::Gauge(display.getScreen("Flywheel")))
                ->setPosition(0, 0)
                ->setSize(190, 190)
                ->setScale(250, 31, 7)
                ->setRange(0, 3000)
                ->setCriticalValue(2800)
                ->setValue(20);

        flywheelSpeedLabel = (new LouUI::Label(display.getScreen("Flywheel")))
                ->setText("RPM")
                ->align(flywheelSpeedGauge->getObj(), LouUI::Align::IN_BOTTOM_MID, 0, -30);
    }
}

#endif //ROBOTCODE_UI_HPP

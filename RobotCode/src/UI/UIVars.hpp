/**
 * File: UIVars.hpp
 * Author: Kevin Lou
 * Created On: Jan 3 2023
 * Last Updated: Jan 3 2023
 *
 * Copyright (c) 2023 3946E
 *
 * Summary of File:
 *  - Contains Declarations for all UI elements
 */

#ifndef ROBOTCODE_UIVARS_HPP
#define ROBOTCODE_UIVARS_HPP

#include "../LouUI/LouUI.hpp"

extern LouUI::Display display;

/***Auton Selector***/
extern LouUI::ToggleButton* leftButton;
extern LouUI::Label* leftButtonLabel;
extern LouUI::ToggleButton* rightButton;
extern LouUI::Label* rightButtonLabel;
extern LouUI::ToggleButton* progButton;
extern LouUI::Label* progButtonLabel;
extern LouUI::ToggleButton* auton1Button;
extern LouUI::Label* auton1ButtonLabel;
extern LouUI::ToggleButton* auton2Button;
extern LouUI::Label* auton2ButtonLabel;
extern LouUI::ToggleButton* auton3Button;
extern LouUI::Label* auton3ButtonLabel;
extern LouUI::ToggleButton* auton4Button;
extern LouUI::Label* auton4ButtonLabel;
extern LouUI::Label* autonDescription;

/***Drive Data***/
extern LouUI::Gauge* flTempGauge;
extern LouUI::Label* flTempGaugeLabel;
extern LouUI::Gauge* frTempGauge;
extern LouUI::Label* frTempGaugeLabel;
extern LouUI::Gauge* blTempGauge;
extern LouUI::Label* blTempGaugeLabel;
extern LouUI::Gauge* brTempGauge;
extern LouUI::Label* brTempGaugeLabel;
extern LouUI::Label* odomLabel;
extern LouUI::Label* odomXLabel;
extern LouUI::Label* odomYLabel;
extern LouUI::Label* odomThetaLabel;

/***Flywheel Data***/
extern LouUI::Gauge* flywheelSpeedGauge;
extern LouUI::Label* flywheelSpeedLabel;
extern LouUI::Gauge* flywheel1TempGauge;
extern LouUI::Gauge* flywheel2TempGuage;

#endif //ROBOTCODE_UIVARS_HPP

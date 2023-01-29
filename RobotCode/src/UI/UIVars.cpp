/**
 * File: UIVars.cpp
 * Author: Kevin Lou
 * Created On: Jan 3 2023
 * Last Updated: Jan 3 2023
 *
 * Copyright (c) 2023 3946E
 *
 * Summary of File:
 *  - Initializes all UI elements as null
 */

#include "UIVars.hpp"

LouUI::Display display;

/***Auton Selector***/
LouUI::ToggleButton* leftButton = nullptr;
LouUI::Label* leftButtonLabel = nullptr;
LouUI::ToggleButton* rightButton = nullptr;
LouUI::Label* rightButtonLabel = nullptr;
LouUI::ToggleButton* progButton = nullptr;
LouUI::Label* progButtonLabel = nullptr;
LouUI::ToggleButton* auton1Button = nullptr;
LouUI::Label* auton1ButtonLabel = nullptr;
LouUI::ToggleButton* auton2Button = nullptr;
LouUI::Label* auton2ButtonLabel = nullptr;
LouUI::ToggleButton* auton3Button = nullptr;
LouUI::Label* auton3ButtonLabel = nullptr;
LouUI::ToggleButton* auton4Button = nullptr;
LouUI::Label* auton4ButtonLabel = nullptr;
LouUI::Label* autonDescription = nullptr;

/***Drive Data***/
LouUI::Gauge* flTempGauge = nullptr;
LouUI::Label* flTempGaugeLabel = nullptr;
LouUI::Gauge* frTempGauge = nullptr;
LouUI::Label* frTempGaugeLabel = nullptr;
LouUI::Gauge* blTempGauge = nullptr;
LouUI::Label* blTempGaugeLabel = nullptr;
LouUI::Gauge* brTempGauge = nullptr;
LouUI::Label* brTempGaugeLabel = nullptr;
LouUI::Label* odomLabel = nullptr;
LouUI::Label* odomXLabel = nullptr;
LouUI::Label* odomYLabel = nullptr;
LouUI::Label* odomThetaLabel = nullptr;

/***Flywheel Data***/
LouUI::Gauge* flywheelSpeedGauge = nullptr;
LouUI::Label* flywheelSpeedLabel = nullptr;
LouUI::Gauge* flywheel1TempGauge = nullptr;
LouUI::Gauge* flywheel2TempGuage = nullptr;
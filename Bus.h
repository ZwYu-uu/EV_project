//
// Created by ZwYu on 2022-06-27.
//

#ifndef EV_PROJECT_BUS_H
#define EV_PROJECT_BUS_H

#include "parameters.h"

class Bus
{
public:
    //The buses at both ATS and DL can be charged
    int CN; // Charger number
    bool prime; // false: b; true: b'
    int ATS; // Available time slot
    int DL; // deadline
    double ISoC; // Initial state of charge

    Bus();
    Bus(int CN, bool prime, int ATS, int DL, double ISoC);
};

class rate_sequence
{
public:
    int PR [number_time_slot]; // Power rates; rate: 0, 50, or 150
    double SoC[number_time_slot + 1]; // The state of charges; depend on the initial state and the rates array...
    double delta;

    rate_sequence();
    rate_sequence(double ISoC); // All zero power rates constructor
};

class column_information
{
public:
    int BN; // Bus number
    int ColN; // Column number (i.e., the index in the corresponding buses_columns[BN])
    double delta; // The corresponding delta

    column_information();
    column_information(int BN, int ColN, double delta);
};

#endif //EV_PROJECT_BUS_H

//
// Created by ZwYu on 2022-06-27.
//

#ifndef EV_PROJECT_BUS_H
#define EV_PROJECT_BUS_H

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


#endif //EV_PROJECT_BUS_H

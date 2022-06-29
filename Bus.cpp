//
// Created by ZwYu on 2022-06-27.
//

#include "Bus.h"

Bus::Bus() {};

Bus::Bus(int CN, bool prime, int ATS, int DL, double ISoC)
{
    this->CN = CN;
    this->prime = prime;
    this->ATS = ATS;
    this->DL = DL;
    this->ISoC = ISoC;
}

rate_sequence::rate_sequence() {}

rate_sequence::rate_sequence(double ISoC)
{
    for (int i = 0; i < number_time_slot; ++i)
    {
        this->PR[i] = 0;
        this->SoC[i] = ISoC;
    }
    this->SoC[number_time_slot] = ISoC;
    this->delta = target_state_of_charge - this->SoC[number_time_slot] > 0 ? target_state_of_charge - this->SoC[number_time_slot] : 0;
}

column_information::column_information() {}

column_information::column_information(int BN, int ColN, double delta)
{
    this->BN = BN;
    this->ColN = ColN;
    this->delta = delta;
}

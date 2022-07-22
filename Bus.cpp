//
// Created by ZwYu on 2022-06-27.
//

#include "Bus.h"

Bus::Bus() {};

Bus::Bus(int CN, bool prime, int ATS, int DL, double ISoC, double RSoC)
{
    this->CN = CN;
    this->prime = prime;
    this->ATS = ATS;
    this->DL = DL;
    this->ISoC = ISoC;
    this->RSoC = RSoC;
}

rate_sequence::rate_sequence() {}

rate_sequence::rate_sequence(double ISoC, double RSoC)
{
    for (int i = 0; i < number_time_slot; ++i)
    {
        this->PR[i] = 0;
        this->SoC[i] = ISoC;
    }
    this->SoC[number_time_slot] = ISoC;
    this->delta = RSoC - this->SoC[number_time_slot] > 0 ? RSoC - this->SoC[number_time_slot] : 0;
}

bool rate_sequence::isequal(rate_sequence A, rate_sequence B)
{
    for (int i = 0; i < number_time_slot; ++i)
        if (A.PR[i] != B.PR[i])
            return false;
    return true;
}

column_information::column_information() {}

column_information::column_information(int BN, int ColN, double delta)
{
    this->BN = BN;
    this->ColN = ColN;
    this->delta = delta;
}

round_indicator::round_indicator() {}

round_indicator::round_indicator(int BN, int t, int rate, double probability)
{
    this->BN = BN;
    this->t = t;
    this->rate = rate;
    this->probability = probability;
}
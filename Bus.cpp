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

// Use Column Generation with COIN-OR CLP to solve the Electric Vehicle Scheduling Problem
// 2022.06.15 Zhanwei Yu

#include <iostream>
#include <vector>
#include <iterator>
#include <chrono>
#include "CbcModel.hpp"
#include "OsiClpSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "parameters.h"
#include "Bus.h"
#include "SPFA.h"

int how_many_fixed = 0;

Bus total_buses[number_bus]; // List of busus; pre-input information
int chargers[number_charger][2]; // List of charger; pre-input information

std::vector<std::vector<rate_sequence>> bus_columns(number_bus);

std::vector<column_information> problem_columns; // give BN and ColN to record the columns

double chi[number_bus * 10000]; // Store chi-variables

double pi_1b[number_bus];
double pi_1c[number_charger][number_time_slot];
double pi_1d[number_charger][number_time_slot];
double pi_1e[number_time_slot];

double depo_power[number_time_slot]; // 6:00-22:00 is 700 kVA; 22:00-6:00 is 1400 kVA
// time slot starts at 18:00

int fixed_decisions[number_bus][number_time_slot];

void initialize_chargers()
{
    int bus_n = 0;
    for (int i = 0; i < number_charger; ++i)
    {
        chargers[i][0] = bus_n++;
        chargers[i][1] = bus_n++;
    }
}

void initialize_depo_power() // Needs to rewrite
{
    for (int t = 0; t < 15; ++t)
        depo_power[t] = 700;
    for (int t = 15; t < 47; ++t)
        depo_power[t] = 1400+150;
    for (int t = 47; t < number_time_slot; ++t)
        depo_power[t] = 700;
}

void initialize_total_buses() // Need to rewrite via reading file to initialize
{

//    total_buses[0] = Bus(0, false, 11, 41, 15.273191489361704, 91.6391489361702);
//    total_buses[1] = Bus(0, true, 3, 41, 14.37893617021277, 86.2736170212766);

//    // This average is 229 km. No solution
//    total_buses[0] = Bus(0, false, 11, 41, 15.273191489361704, 91.6391489361702);
//    total_buses[1] = Bus(0, true, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[12] = Bus(6, false, 4, 42, 14.401914893617024, 86.41148936170214);
//    total_buses[2] = Bus(1, false, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[3] = Bus(1, true, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[14] = Bus(7, false, 9, 45, 15.480000000000002, 92.88);
//    total_buses[4] = Bus(2, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[5] = Bus(2, true, 12, 46, 15.65617021276596, 93.93702127659574);
//    total_buses[16] = Bus(8, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[6] = Bus(3, false, 12, 47, 15.003829787234043, 90.02297872340425);
//    total_buses[7] = Bus(3, true, 10, 47, 15.333191489361703, 91.9991489361702);
//    total_buses[18] = Bus(9, false, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[8] = Bus(4, false, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[9] = Bus(4, true, 11, 48, 15.005106382978726, 90.03063829787233);
//    total_buses[20] = Bus(10, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[10] = Bus(5, false, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // The average distance is 224 km. 52
//    total_buses[0] = Bus(0, false, 11, 41, 15.273191489361704, 91.6391489361702);
//    total_buses[1] = Bus(0, true, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[12] = Bus(6, false, 4, 42, 14.401914893617024, 86.41148936170214);
//    total_buses[2] = Bus(1, false, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[3] = Bus(1, true, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[14] = Bus(7, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[4] = Bus(2, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[5] = Bus(2, true, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[16] = Bus(8, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[6] = Bus(3, false, 12, 47, 15.003829787234043, 90.02297872340425);
//    total_buses[7] = Bus(3, true, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[18] = Bus(9, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[8] = Bus(4, false, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[9] = Bus(4, true, 11, 48, 15.005106382978726, 90.03063829787233);
//    total_buses[20] = Bus(10, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[10] = Bus(5, false, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 222km.
//    total_buses[0] = Bus(0, false, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[1] = Bus(0, true, 4, 42, 14.401914893617024, 86.41148936170214);
//    total_buses[12] = Bus(6, false, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[2] = Bus(1, false, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[3] = Bus(1, true, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[14] = Bus(7, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[4] = Bus(2, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[5] = Bus(2, true, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[16] = Bus(8, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[6] = Bus(3, false, 12, 47, 15.003829787234043, 90.02297872340425);
//    total_buses[7] = Bus(3, true, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[18] = Bus(9, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[8] = Bus(4, false, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[9] = Bus(4, true, 11, 48, 15.005106382978726, 90.03063829787233);
//    total_buses[20] = Bus(10, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[10] = Bus(5, false, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 220 km. 51
//    total_buses[0] = Bus(0, false, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[1] = Bus(0, true, 10, 41, 16.314893617021276, 97.88936170212766);
//    total_buses[12] = Bus(6, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[2] = Bus(1, false, 4, 42, 14.401914893617024, 86.41148936170214);
//    total_buses[3] = Bus(1, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[14] = Bus(7, false, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[4] = Bus(2, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[5] = Bus(2, true, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[16] = Bus(8, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[6] = Bus(3, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[7] = Bus(3, true, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[18] = Bus(9, false, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[8] = Bus(4, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[9] = Bus(4, true, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[20] = Bus(10, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[10] = Bus(5, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 219 km. no solution
//    total_buses[0] = Bus(0, false, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[1] = Bus(0, true, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[12] = Bus(6, false, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[2] = Bus(1, false, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[3] = Bus(1, true, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[14] = Bus(7, false, 9, 45, 15.480000000000002, 92.88);
//    total_buses[4] = Bus(2, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[5] = Bus(2, true, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[16] = Bus(8, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[6] = Bus(3, false, 12, 46, 15.65617021276596, 93.93702127659574);
//    total_buses[7] = Bus(3, true, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[18] = Bus(9, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[8] = Bus(4, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[9] = Bus(4, true, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[20] = Bus(10, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[10] = Bus(5, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // 218.34 km. 46
//    total_buses[0] = Bus(0, false, 11, 41, 15.273191489361704, 91.6391489361702);
//    total_buses[1] = Bus(0, true, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[12] = Bus(6, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[2] = Bus(1, false, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[3] = Bus(1, true, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[14] = Bus(7, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[4] = Bus(2, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[5] = Bus(2, true, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[16] = Bus(8, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[6] = Bus(3, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[7] = Bus(3, true, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 11, 48, 15.005106382978726, 90.03063829787233);
//    total_buses[10] = Bus(5, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 217 km. 46
//    total_buses[0] = Bus(0, false, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[1] = Bus(0, true, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[12] = Bus(6, false, 4, 42, 14.401914893617024, 86.41148936170214);
//    total_buses[2] = Bus(1, false, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[3] = Bus(1, true, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[14] = Bus(7, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[4] = Bus(2, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[5] = Bus(2, true, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[16] = Bus(8, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[6] = Bus(3, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[7] = Bus(3, true, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[18] = Bus(9, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[8] = Bus(4, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[9] = Bus(4, true, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[20] = Bus(10, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[10] = Bus(5, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This data is from "omlopp 1.xlsx". The average distance is 214 km. 46
//    total_buses[0] = Bus(0, false, 3, 41, 14.37893617021277, 86.2736170212766);
//    total_buses[1] = Bus(0, true, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[12] = Bus(6, false, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[2] = Bus(1, false, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[3] = Bus(1, true, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[14] = Bus(7, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[4] = Bus(2, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[5] = Bus(2, true, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[16] = Bus(8, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[6] = Bus(3, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[7] = Bus(3, true, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[18] = Bus(9, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[8] = Bus(4, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[10] = Bus(5, false, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 211 km. 46
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[3] = Bus(1, true, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[14] = Bus(7, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[4] = Bus(2, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[5] = Bus(2, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[16] = Bus(8, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[6] = Bus(3, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[7] = Bus(3, true, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[18] = Bus(9, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[8] = Bus(4, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[9] = Bus(4, true, 13, 48, 14.37191489361702, 86.23148936170213);
//    total_buses[20] = Bus(10, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[10] = Bus(5, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 210 km. 46
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[3] = Bus(1, true, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[14] = Bus(7, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[4] = Bus(2, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[5] = Bus(2, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[16] = Bus(8, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[6] = Bus(3, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[7] = Bus(3, true, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[18] = Bus(9, false, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[8] = Bus(4, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[9] = Bus(4, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[20] = Bus(10, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[10] = Bus(5, false, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // The average distance is 208 km. 43
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[3] = Bus(1, true, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[14] = Bus(7, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[4] = Bus(2, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[5] = Bus(2, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[16] = Bus(8, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[6] = Bus(3, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[7] = Bus(3, true, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[18] = Bus(9, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[8] = Bus(4, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[9] = Bus(4, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[20] = Bus(10, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[10] = Bus(5, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // The average distance is 208 km. 18 EVs.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[3] = Bus(1, true, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[14] = Bus(7, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[4] = Bus(2, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[5] = Bus(2, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[16] = Bus(8, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[6] = Bus(3, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[7] = Bus(3, true, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[18] = Bus(9, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[8] = Bus(4, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[9] = Bus(4, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[20] = Bus(10, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[10] = Bus(5, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 13.27291615, 79.63749687);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 206 km.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[12] = Bus(6, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[2] = Bus(1, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[3] = Bus(1, true, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[14] = Bus(7, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[7] = Bus(3, true, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[9] = Bus(4, true, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 206 km. 18 EVs.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[12] = Bus(6, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[2] = Bus(1, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[3] = Bus(1, true, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[14] = Bus(7, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[7] = Bus(3, true, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[9] = Bus(4, true, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 13.16219024, 78.97314143);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 204 km. 45
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[3] = Bus(1, true, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[14] = Bus(7, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[4] = Bus(2, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[5] = Bus(2, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[16] = Bus(8, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[6] = Bus(3, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[7] = Bus(3, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[8] = Bus(4, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[9] = Bus(4, true, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[20] = Bus(10, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[10] = Bus(5, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[11] = Bus(5, true, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 204 km. 18 EVs.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[3] = Bus(1, true, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[14] = Bus(7, false, 4, 45, 13.777021276595743, 82.66212765957447);
//    total_buses[4] = Bus(2, false, 4, 46, 13.788510638297874, 82.73106382978723);
//    total_buses[5] = Bus(2, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[16] = Bus(8, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[6] = Bus(3, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[7] = Bus(3, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[8] = Bus(4, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[9] = Bus(4, true, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[20] = Bus(10, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[10] = Bus(5, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[11] = Bus(5, true, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 13.07882419, 78.47294513);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

    // 203.5629412 km. 43
    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
    total_buses[1] = Bus(0, true, 4, 42, 14.401914893617024, 86.41148936170214);
    total_buses[12] = Bus(6, false, 2, 42, 14.391702127659576, 86.35021276595744);
    total_buses[2] = Bus(1, false, 2, 45, 12.744255319148937, 76.46553191489362);
    total_buses[3] = Bus(1, true, 1, 46, 12.871276595744682, 77.2276595744681);
    total_buses[14] = Bus(7, false, 1, 46, 12.571914893617024, 75.43148936170213);
    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
    total_buses[5] = Bus(2, true, 2, 46, 12.972127659574467, 77.8327659574468);
    total_buses[16] = Bus(8, false, 3, 47, 12.017872340425534, 72.10723404255319);
    total_buses[6] = Bus(3, false, 4, 47, 12.744255319148937, 76.46553191489362);
    total_buses[7] = Bus(3, true, 8, 47, 14.588297872340428, 87.52978723404257);
    total_buses[18] = Bus(9, false, 2, 47, 12.57765957446809, 75.46595744680852);
    total_buses[8] = Bus(4, false, 4, 48, 12.972127659574467, 77.8327659574468);
    total_buses[9] = Bus(4, true, 2, 48, 12.571914893617024, 75.43148936170213);
    total_buses[20] = Bus(10, false, 10, 49, 14.588297872340428, 87.52978723404257);
    total_buses[10] = Bus(5, false, 1, 51, 11.678297872340426, 70.06978723404256);
    total_buses[11] = Bus(5, true, 2, 52, 11.40127659574468, 68.40765957446808);
    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // 203.5629412 km. 43. ISoC = 0
//    total_buses[0] = Bus(0, false, 2, 42, 0, 79.16170212765957 - 13.193617021276596);
//    total_buses[1] = Bus(0, true, 4, 42, 0, 86.41148936170214 - 14.401914893617024);
//    total_buses[12] = Bus(6, false, 2, 42, 0, 86.35021276595744 - 14.391702127659576);
//    total_buses[2] = Bus(1, false, 2, 45, 0, 76.46553191489362 - 12.744255319148937);
//    total_buses[3] = Bus(1, true, 1, 46, 0, 77.2276595744681 - 12.871276595744682);
//    total_buses[14] = Bus(7, false, 1, 46, 0, 75.43148936170213 - 12.571914893617024);
//    total_buses[4] = Bus(2, false, 1, 46, 0, 75.60382978723405 - 12.600638297872344);
//    total_buses[5] = Bus(2, true, 2, 46, 0, 77.8327659574468 - 12.972127659574467);
//    total_buses[16] = Bus(8, false, 3, 47, 0, 72.10723404255319 - 12.017872340425534);
//    total_buses[6] = Bus(3, false, 4, 47, 0, 76.46553191489362 - 12.744255319148937);
//    total_buses[7] = Bus(3, true, 8, 47, 0, 87.52978723404257 - 14.588297872340428);
//    total_buses[18] = Bus(9, false, 2, 47, 0, 75.46595744680852 - 12.57765957446809);
//    total_buses[8] = Bus(4, false, 4, 48, 0, 77.8327659574468 - 12.972127659574467);
//    total_buses[9] = Bus(4, true, 2, 48, 0, 75.43148936170213 - 12.571914893617024);
//    total_buses[20] = Bus(10, false, 10, 49, 0, 87.52978723404257 - 14.588297872340428);
//    total_buses[10] = Bus(5, false, 1, 51, 0, 70.06978723404256 - 11.678297872340426);
//    total_buses[11] = Bus(5, true, 2, 52, 0, 68.40765957446808 - 11.40127659574468);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // 203.5629412 km. 18 EVs.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 4, 42, 14.401914893617024, 86.41148936170214);
//    total_buses[12] = Bus(6, false, 2, 42, 14.391702127659576, 86.35021276595744);
//    total_buses[2] = Bus(1, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[3] = Bus(1, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[14] = Bus(7, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[5] = Bus(2, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[16] = Bus(8, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[6] = Bus(3, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[7] = Bus(3, true, 8, 47, 14.588297872340428, 87.52978723404257);
//    total_buses[18] = Bus(9, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[8] = Bus(4, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[9] = Bus(4, true, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[20] = Bus(10, false, 10, 49, 14.588297872340428, 87.52978723404257);
//    total_buses[10] = Bus(5, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[11] = Bus(5, true, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, 52, 12.99337922, 77.96027534);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 202 km. //42
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[12] = Bus(6, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[2] = Bus(1, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[3] = Bus(1, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[14] = Bus(7, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[5] = Bus(2, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[16] = Bus(8, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[6] = Bus(3, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[7] = Bus(3, true, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[18] = Bus(9, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[8] = Bus(4, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[9] = Bus(4, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[20] = Bus(10, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[10] = Bus(5, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[11] = Bus(5, true, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 202 km. 18 EVs.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[12] = Bus(6, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[2] = Bus(1, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[3] = Bus(1, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[14] = Bus(7, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[5] = Bus(2, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[16] = Bus(8, false, 11, 46, 14.37191489361702, 86.23148936170213);
//    total_buses[6] = Bus(3, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[7] = Bus(3, true, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[18] = Bus(9, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[8] = Bus(4, false, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[9] = Bus(4, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[20] = Bus(10, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[10] = Bus(5, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[11] = Bus(5, true, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 12.89590738, 77.37544431);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // 201 km.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[3] = Bus(1, true, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[7] = Bus(3, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[8] = Bus(4, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[9] = Bus(4, true, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // 201 km. 18EVs.
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[3] = Bus(1, true, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[7] = Bus(3, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[8] = Bus(4, false, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[9] = Bus(4, true, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 12.83346683, 77.000801);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // This data is from "omlopp 2.xlsx". The average distance is 199.6 km. 44
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[12] = Bus(6, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[2] = Bus(1, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[3] = Bus(1, true, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[7] = Bus(3, true, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // The average is 199.6 km. 18EVs. 48
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[12] = Bus(6, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[2] = Bus(1, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[3] = Bus(1, true, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[7] = Bus(3, true, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 12.74173967, 76.45043805);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 199km. 43
//    total_buses[0] = Bus(0, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[1] = Bus(0, true, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[12] = Bus(6, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[2] = Bus(1, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[3] = Bus(1, true, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[5] = Bus(2, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[7] = Bus(3, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[10] = Bus(5, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[11] = Bus(5, true, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 199km. 18 EVs. 45
//    total_buses[0] = Bus(0, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[1] = Bus(0, true, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[12] = Bus(6, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[2] = Bus(1, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[3] = Bus(1, true, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[5] = Bus(2, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[7] = Bus(3, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[10] = Bus(5, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[11] = Bus(5, true, 4, 47, 13.47191489361702, 80.83148936170213);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 12.72116395, 76.32698373);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 197.6376471 km. 41
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[3] = Bus(1, true, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[7] = Bus(3, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[8] = Bus(4, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[9] = Bus(4, true, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[20] = Bus(10, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[10] = Bus(5, false, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[11] = Bus(5, true, 8, 58, 9.485106382978724, 56.91063829787234);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // This average is 197.6376471 km. 18 EVs. 47
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 1, 42, 13.930212765957448, 83.5812765957447);
//    total_buses[12] = Bus(6, false, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[2] = Bus(1, false, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[3] = Bus(1, true, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[14] = Bus(7, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[4] = Bus(2, false, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[5] = Bus(2, true, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[16] = Bus(8, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[6] = Bus(3, false, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[7] = Bus(3, true, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[18] = Bus(9, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[8] = Bus(4, false, 10, 49, 14.303617021276596, 85.82170212765958);
//    total_buses[9] = Bus(4, true, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[20] = Bus(10, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[10] = Bus(5, false, 10, 53, 13.821702127659574, 82.93021276595745);
//    total_buses[11] = Bus(5, true, 8, 58, 9.485106382978724, 56.91063829787234);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 12.61516896, 75.69101377);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

//    // This data is from "omlopp 3.xlsx". The average distance is 195.6 km. 42
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[12] = Bus(6, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[2] = Bus(1, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[3] = Bus(1, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[14] = Bus(7, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[5] = Bus(2, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[16] = Bus(8, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[6] = Bus(3, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[7] = Bus(3, true, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 8, 58, 9.485106382978724, 56.91063829787234);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);

//    // The average is 195.6 km. 18 EVs. 49
//    total_buses[0] = Bus(0, false, 2, 42, 13.193617021276596, 79.16170212765957);
//    total_buses[1] = Bus(0, true, 2, 45, 12.744255319148937, 76.46553191489362);
//    total_buses[12] = Bus(6, false, 3, 45, 13.202553191489361, 79.21531914893617);
//    total_buses[2] = Bus(1, false, 3, 45, 13.47191489361702, 80.83148936170213);
//    total_buses[3] = Bus(1, true, 1, 46, 12.871276595744682, 77.2276595744681);
//    total_buses[14] = Bus(7, false, 1, 46, 12.571914893617024, 75.43148936170213);
//    total_buses[4] = Bus(2, false, 1, 46, 12.600638297872344, 75.60382978723405);
//    total_buses[5] = Bus(2, true, 2, 46, 12.972127659574467, 77.8327659574468);
//    total_buses[16] = Bus(8, false, 3, 47, 12.017872340425534, 72.10723404255319);
//    total_buses[6] = Bus(3, false, 4, 47, 12.744255319148937, 76.46553191489362);
//    total_buses[7] = Bus(3, true, 2, 47, 12.57765957446809, 75.46595744680852);
//    total_buses[18] = Bus(9, false, 4, 48, 12.972127659574467, 77.8327659574468);
//    total_buses[8] = Bus(4, false, 2, 48, 12.571914893617024, 75.43148936170213);
//    total_buses[9] = Bus(4, true, 9, 48, 13.19617021276596, 79.17702127659575);
//    total_buses[20] = Bus(10, false, 1, 51, 11.678297872340426, 70.06978723404256);
//    total_buses[10] = Bus(5, false, 2, 52, 11.40127659574468, 68.40765957446808);
//    total_buses[11] = Bus(5, true, 8, 58, 9.485106382978724, 56.91063829787234);
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
//    total_buses[22] = Bus(11, true, 0, number_time_slot - 1, 12.48664581, 74.91987484);
//    total_buses[23] = Bus(11, false, 0, number_time_slot - 1, 100.0, 100.0);

    memset(fixed_decisions, -1, sizeof(fixed_decisions));
}

void initialize_columns() // Initialize: to add all-zero columns
{
    for (int BN = 0; BN < number_bus; ++BN)
    {
        bus_columns[BN].push_back(rate_sequence(total_buses[BN].ISoC, total_buses[BN].RSoC));
        problem_columns.push_back(column_information(BN, bus_columns[BN].size() - 1, bus_columns[BN][bus_columns[BN].size() - 1].delta));
    }
}

void initialization()
{
    initialize_chargers();
    initialize_depo_power();
    initialize_total_buses();
    initialize_columns();
}

OsiClpSolverInterface model = OsiClpSolverInterface();

void solve_initial_LP()
{
    double objective[number_bus];
    double col_lb[number_bus];
    double col_ub[number_bus];
    for (int i = 0; i < number_bus; ++i)
    {
        objective[i] = problem_columns[i].delta;
        col_lb[i] = 0.0; // 0 <= chi <= 1
        col_ub[i] = 1.0;
    }
    double row_lb[number_bus + 2 * number_charger * number_time_slot + number_time_slot];
    double row_ub[number_bus + 2 * number_charger * number_time_slot + number_time_slot];
    // 0 ~ (number_bus - 1) are with respect to constraint (1b)
    // number_bus ~ (number_bus + number_charger * number_time_slot - 1) are with the respect to constraint (1c)
    // (number_bus + number_charger * number_time_slot) ~ (number_bus + 2 * number_charger * number_time_slot - 1) are with the respect to constraint (1d)
    // (number_bus + 2 * number_charger * number_time_slot - 1) ~ (number_bus + 2 * number_charger * number_time_slot + number_time_slot - 1) are with the respect to constraint (1e)
    for (int i = 0; i < number_bus; ++i)
    {
        row_lb[i] = 1.0;
        row_ub[i] = 1.0;
    }
    for (int i = number_bus; i < number_bus + 2 * number_charger * number_time_slot; ++i)
    {
        row_lb[i] = -1.0 * model.getInfinity();
        row_ub[i] = 1.0;
    }
    for (int i = number_bus + 2 * number_charger * number_time_slot; i < number_bus + 2 * number_charger * number_time_slot + number_time_slot; ++i)
    {
        row_lb[i] = -1.0 * model.getInfinity();
        row_ub[i] = std::floor((depo_power[i - number_bus - 2 * number_charger * number_time_slot] - base_load_power)/50); // q = 50
    }
    CoinPackedMatrix matrix = CoinPackedMatrix(false,0,0);
    matrix.setDimensions(0, number_bus);
    int inx[number_bus];
    for (int i = 0; i < number_bus; ++i)
        inx[i] = i;
    double el[number_bus] = {0.0};
    CoinPackedVector row(number_bus, inx, el);
    for (int i = 0; i < number_bus; ++i)
    {
        row.setElement(i,1.0);
        matrix.appendRow(row);
        row.setElement(i,0.0);
    }
    CoinPackedVector rows_after(number_bus, inx, el);
    for (int i = number_bus; i < number_bus + 2 * number_charger * number_time_slot + number_time_slot; ++i)
        matrix.appendRow(rows_after);
    model.loadProblem(matrix, col_lb, col_ub, objective, row_lb, row_ub);
    model.initialSolve();

    const double* solution;
    solution = model.getColSolution();
    for (int i = 0; i < number_bus; ++i)
        chi[i] = *(solution + i);
    const double* dualvariables;
    dualvariables = model.getRowPrice();
    for (int i = 0; i < number_bus; ++i)
        pi_1b[i] = *(dualvariables + i);
    for (int i = 0; i < number_charger; ++i)
        for (int j = 0; j < number_time_slot; ++j)
            pi_1c[i][j] = *(dualvariables + number_bus + i * number_time_slot + j);
    for (int i = 0; i < number_charger; ++i)
        for (int j = 0; j < number_time_slot; ++j)
            pi_1d[i][j] = *(dualvariables + number_bus + number_charger * number_time_slot + i * number_time_slot + j);
    for (int i = 0; i < number_time_slot; ++i)
        pi_1e[i] = *(dualvariables + number_bus + 2 * number_charger * number_time_slot + i);
}

void add_column_to_model(rate_sequence new_column)
{
    int inx[number_of_row];
    double el[number_of_row] = {0};
    for (int i = 0; i < number_of_row; ++i)
        inx[i] = i;
    el[problem_columns[problem_columns.size() - 1].BN] = 1.0; // For constraint (1b)
    if (!total_buses[problem_columns[problem_columns.size() - 1].BN].prime) // For constraints (1c) and (1d)
    {
        for (int t = 0; t < number_time_slot; ++t)
        {
            if (new_column.PR[t] == 150)
                el[number_bus + number_time_slot * total_buses[problem_columns[problem_columns.size() - 1].BN].CN + t] = 1.0;
            if (new_column.PR[t] == 50)
                el[number_bus + number_time_slot * number_charger + number_time_slot * total_buses[problem_columns[problem_columns.size() - 1].BN].CN + t] = 1.0;
        }
    }
    else
    {
        for (int t = 0; t < number_time_slot; ++t)  // For constraints (1c) and (1d)
        {
            if (new_column.PR[t] == 50 or new_column.PR[t] == 150)
                el[number_bus + number_time_slot * total_buses[problem_columns[problem_columns.size() - 1].BN].CN + t] = 1;
            if (new_column.PR[t] == 150)
                el[number_bus + number_time_slot * number_charger + number_time_slot * total_buses[problem_columns[problem_columns.size() - 1].BN].CN + t] = 1;
        }
    }
    for (int t = 0; t < number_time_slot; ++t) // For constraint (1e)
        el[number_bus + 2 * number_time_slot * number_charger + t] = new_column.PR[t]/50.0;
    CoinPackedVector col(number_of_row, inx, el);

    model.addCol(col, 0.0, 1.0, new_column.delta);
}

void get_min_reduced_cost_add_the_column(int BN, int t_star) // BN = bus number
{
//    int n = ceil((100 - total_buses[BN].ISoC) / energy_percent_every_time_slot); // The number of nodes in each time slot
    double node_corresponding_SoC[100];
    node_corresponding_SoC[0] = total_buses[BN].ISoC;
    int n = 1;
    double SoC = total_buses[BN].ISoC;
    while (SoC < 100)
    {
        if (SoC < 70)
            SoC += energy_percent_every_time_slot_1;
        else if (SoC < 80)
            SoC += energy_percent_every_time_slot_2;
        else
            SoC += energy_percent_every_time_slot_3;
        node_corresponding_SoC[n++] = SoC > 100? 100 : SoC;
    }

    int m = std::min(t_star, total_buses[BN].DL) - total_buses[BN].ATS + 1; // The number of time slots
    int number_of_all_nodes = n * m + 2; // Adding two is for source and tank
    SPFA graph(number_of_all_nodes);
    graph.SetSourceAndTank(0, n * m + 1);
    if (!total_buses[BN].prime) // b
    {
        graph.InputAdjMat(0, 1, 0);
        graph.InputAdjMat(0, 2, - pi_1d[total_buses[BN].CN][total_buses[BN].ATS] - pi_1e[total_buses[BN].ATS]);
        graph.InputAdjMat(0, 4, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - 3 * pi_1e[total_buses[BN].ATS]);
        for (int t = 1; t < m; ++t)
            for (int node = 1; node < n + 1; ++node)
            {
                graph.InputAdjMat(node + (t - 1) * n, node + t * n, 0);
                if (node < n)
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n + 1, - pi_1d[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1e[total_buses[BN].ATS + t]);
                if (node < n - 2)
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n + 3, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - 3 * pi_1e[total_buses[BN].ATS + t]);
            }
        for (int i = 0; i < n; ++i)
            graph.InputAdjMat((m - 1) * n + 1 + i, m * n + 1, (total_buses[BN].RSoC - node_corresponding_SoC[i] ) < 0? 0 : (total_buses[BN].RSoC - node_corresponding_SoC[i]));
    }
    else // b'
    {
        graph.InputAdjMat(0, 1, 0);
        graph.InputAdjMat(0, 2, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - pi_1e[total_buses[BN].ATS]);
        graph.InputAdjMat(0, 4, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - pi_1d[total_buses[BN].CN][total_buses[BN].ATS] - 3 * pi_1e[total_buses[BN].ATS]);
        for (int t = 1; t < m; ++t)
            for (int node = 1; node < n + 1; ++node)
            {
                graph.InputAdjMat(node + (t - 1) * n, node + t * n, 0);
                if (node < n)
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n + 1, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1e[total_buses[BN].ATS + t]);
                if (node < n - 2)
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n + 3, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1d[total_buses[BN].CN][total_buses[BN].ATS + t] - 3 * pi_1e[total_buses[BN].ATS + t]);
            }
        for (int i = 0; i < n; ++i)
            graph.InputAdjMat((m - 1) * n + 1 + i, m * n + 1, (total_buses[BN].RSoC - node_corresponding_SoC[i]) < 0? 0:(total_buses[BN].RSoC - node_corresponding_SoC[i]));
    }
    graph.Getpath();
    graph.Getdistance();
    if (graph.dis[number_of_all_nodes - 1] - pi_1b[BN] < 0 && abs(graph.dis[number_of_all_nodes - 1] - pi_1b[BN]) > 1e-6)
    {
        rate_sequence new_column;
        for (int t = 0; t < number_time_slot; ++t) // Initialize new_column
        {
            new_column.PR[t] = 0;
            new_column.SoC[t] = total_buses[BN].ISoC;
        }
        new_column.SoC[number_time_slot] = total_buses[BN].ISoC;

        int node_iter = graph.pre[number_of_all_nodes - 1];
        int t = std::min(t_star, total_buses[BN].DL);
        while (node_iter != 0)
        {
            if (node_iter > n)
            {
                if (node_iter - graph.pre[node_iter] - n == 3)
                    new_column.PR[t] = 150;
                else if (node_iter - graph.pre[node_iter] - n == 1)
                    new_column.PR[t] = 50;
                node_iter = graph.pre[node_iter];
                t--;
            }
            else
            {
                if (node_iter - graph.pre[node_iter] == 4)
                    new_column.PR[t] = 150;
                else if (node_iter - graph.pre[node_iter] == 2)
                    new_column.PR[t] = 50;
                node_iter = graph.pre[node_iter];
                t--;
            }
        }
        int cursor = 0;
        for(int t = 1; t < number_time_slot + 1; ++t)
        {
            if (new_column.PR[t - 1] == 50)
                cursor++;
            if (new_column.PR[t - 1] == 150)
                cursor += 3;
            new_column.SoC[t] = node_corresponding_SoC[cursor];
        }
        new_column.delta = total_buses[BN].RSoC - new_column.SoC[number_time_slot] > 0? total_buses[BN].RSoC - new_column.SoC[number_time_slot]:0;

        // add to bus_columns and problem_columns
        bus_columns[BN].push_back(new_column);
        problem_columns.push_back(column_information(BN, bus_columns[BN].size() - 1, new_column.delta));

        // add to the problem model
        add_column_to_model(new_column); // add column to the model
    }
}

void get_min_reduced_cost_add_the_column_fixed_version(int BN, int t_star) // BN = bus number
{
//    int n = ceil((100 - total_buses[BN].ISoC) / energy_percent_every_time_slot); // The number of nodes in each time slot
    double node_corresponding_SoC[100];
    node_corresponding_SoC[0] = total_buses[BN].ISoC;
    int n = 1;
    double SoC = total_buses[BN].ISoC;
    while (SoC < 100)
    {
        if (SoC < 70)
            SoC += energy_percent_every_time_slot_1;
        else if (SoC < 80)
            SoC += energy_percent_every_time_slot_2;
        else
            SoC += energy_percent_every_time_slot_3;
        node_corresponding_SoC[n++] = SoC > 100? 100 : SoC;
    }

    int m = std::min(t_star, total_buses[BN].DL) - total_buses[BN].ATS + 1; // The number of time slots
    int number_of_all_nodes = n * m + 2; // Adding two is for source and tank
    SPFA graph(number_of_all_nodes);
    graph.SetSourceAndTank(0, n * m + 1);
    if (!total_buses[BN].prime) // b
    {
        if (fixed_decisions[BN][0+total_buses[BN].ATS] == -1)
        {
            graph.InputAdjMat(0, 1, 0);
            graph.InputAdjMat(0, 2, - pi_1d[total_buses[BN].CN][total_buses[BN].ATS] - pi_1e[total_buses[BN].ATS]);
            graph.InputAdjMat(0, 4, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - 3 * pi_1e[total_buses[BN].ATS]);
        }
        else if (fixed_decisions[BN][0+total_buses[BN].ATS] == 0)
            graph.InputAdjMat(0, 1, 0);
        else if (fixed_decisions[BN][0+total_buses[BN].ATS] == 1)
            graph.InputAdjMat(0, 2, - pi_1d[total_buses[BN].CN][total_buses[BN].ATS] - pi_1e[total_buses[BN].ATS]);
        else if (fixed_decisions[BN][0+total_buses[BN].ATS] == 2)
            graph.InputAdjMat(0, 4, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - 3 * pi_1e[total_buses[BN].ATS]);

        for (int t = 1; t < m; ++t) // if m = 1, then skip this process
            for (int node = 1; node < n + 1; ++node)
            {
                if (fixed_decisions[BN][t+total_buses[BN].ATS] == -1)
                {
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n, 0);
                    if (node < n)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 1, - pi_1d[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1e[total_buses[BN].ATS + t]);
                    if (node < n - 2)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 3, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - 3 * pi_1e[total_buses[BN].ATS + t]);
                }
                else if (fixed_decisions[BN][t+total_buses[BN].ATS] == 0)
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n, 0);
                else if (fixed_decisions[BN][t+total_buses[BN].ATS] == 1)
                {
                    if (node < n)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 1, - pi_1d[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1e[total_buses[BN].ATS + t]);
                }
                else if (fixed_decisions[BN][t+total_buses[BN].ATS] == 2)
                {
                    if (node < n - 2)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 3, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - 3 * pi_1e[total_buses[BN].ATS + t]);
                }
            }
        for (int i = 0; i < n; ++i)
            graph.InputAdjMat((m - 1) * n + 1 + i, m * n + 1, (total_buses[BN].RSoC - node_corresponding_SoC[i] ) < 0? 0 : (total_buses[BN].RSoC - node_corresponding_SoC[i]));
    }
    else // b'
    {
        if (fixed_decisions[BN][0+total_buses[BN].ATS] == -1)
        {
            graph.InputAdjMat(0, 1, 0);
            graph.InputAdjMat(0, 2, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - pi_1e[total_buses[BN].ATS]);
            graph.InputAdjMat(0, 4, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - pi_1d[total_buses[BN].CN][total_buses[BN].ATS] - 3 * pi_1e[total_buses[BN].ATS]);
        }
        else if (fixed_decisions[BN][0+total_buses[BN].ATS] == 0)
            graph.InputAdjMat(0, 1, 0);
        else if (fixed_decisions[BN][0+total_buses[BN].ATS] == 1)
            graph.InputAdjMat(0, 2, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - pi_1e[total_buses[BN].ATS]);
        else if (fixed_decisions[BN][0+total_buses[BN].ATS] == 2)
            graph.InputAdjMat(0, 4, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS] - pi_1d[total_buses[BN].CN][total_buses[BN].ATS] - 3 * pi_1e[total_buses[BN].ATS]);

        for (int t = 1; t < m; ++t)
            for (int node = 1; node < n + 1; ++node)
            {
                if (fixed_decisions[BN][t+total_buses[BN].ATS] == -1)
                {
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n, 0);
                    if (node < n)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 1, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1e[total_buses[BN].ATS + t]);
                    if (node < n - 2)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 3, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1d[total_buses[BN].CN][total_buses[BN].ATS + t] - 3 * pi_1e[total_buses[BN].ATS + t]);
                }
                else if (fixed_decisions[BN][t+total_buses[BN].ATS] == 0)
                    graph.InputAdjMat(node + (t - 1) * n, node + t * n, 0);
                else if (fixed_decisions[BN][t+total_buses[BN].ATS] == 1)
                {
                    if (node < n)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 1, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1e[total_buses[BN].ATS + t]);
                }
                else if (fixed_decisions[BN][t+total_buses[BN].ATS] == 2)
                {
                    if (node < n - 2)
                        graph.InputAdjMat(node + (t - 1) * n, node + t * n + 3, - pi_1c[total_buses[BN].CN][total_buses[BN].ATS + t] - pi_1d[total_buses[BN].CN][total_buses[BN].ATS + t] - 3 * pi_1e[total_buses[BN].ATS + t]);
                }
            }
        for (int i = 0; i < n; ++i)
            graph.InputAdjMat((m - 1) * n + 1 + i, m * n + 1, (total_buses[BN].RSoC - node_corresponding_SoC[i]) < 0? 0:(total_buses[BN].RSoC - node_corresponding_SoC[i]));
    }
    graph.Getpath();
    graph.Getdistance();
    if (graph.dis[number_of_all_nodes - 1] - pi_1b[BN] < 0 && abs(graph.dis[number_of_all_nodes - 1] - pi_1b[BN]) > 1e-6)
    {
        rate_sequence new_column;
        for (int t = 0; t < number_time_slot; ++t) // Initialize new_column
        {
            new_column.PR[t] = 0;
            new_column.SoC[t] = total_buses[BN].ISoC;
        }
        new_column.SoC[number_time_slot] = total_buses[BN].ISoC;

        int node_iter = graph.pre[number_of_all_nodes - 1];
        int t = std::min(t_star, total_buses[BN].DL);
        while (node_iter != 0)
        {
            if (node_iter > n)
            {
                if (node_iter - graph.pre[node_iter] - n == 3)
                    new_column.PR[t] = 150;
                else if (node_iter - graph.pre[node_iter] - n == 1)
                    new_column.PR[t] = 50;
                node_iter = graph.pre[node_iter];
                t--;
            }
            else
            {
                if (node_iter - graph.pre[node_iter] == 4)
                    new_column.PR[t] = 150;
                else if (node_iter - graph.pre[node_iter] == 2)
                    new_column.PR[t] = 50;
                node_iter = graph.pre[node_iter];
                t--;
            }
        }
        int cursor = 0;
        for(int t = 1; t < number_time_slot + 1; ++t)
        {
            if (new_column.PR[t - 1] == 50)
                cursor++;
            if (new_column.PR[t - 1] == 150)
                cursor += 3;
            new_column.SoC[t] = node_corresponding_SoC[cursor];
        }
        new_column.delta = total_buses[BN].RSoC - new_column.SoC[number_time_slot] > 0? total_buses[BN].RSoC - new_column.SoC[number_time_slot]:0;

        // add to bus_columns and problem_columns
        bus_columns[BN].push_back(new_column);
        problem_columns.push_back(column_information(BN, bus_columns[BN].size() - 1, new_column.delta));

        // add to the problem model
        add_column_to_model(new_column); // add column to the model
    }
}

bool if_exist_the_same_column(int BN, rate_sequence new_column)
{
    for (int i = 0; i < bus_columns[BN].size(); ++i)
        if (rate_sequence::isequal(new_column, bus_columns[BN][i]))
            return false;
    return true;
}

void add_extra_column(int BN)
{
    double node_corresponding_SoC[100];
    node_corresponding_SoC[0] = total_buses[BN].ISoC;
    int n = 1;
    double SoC = total_buses[BN].ISoC;
    while (SoC < 100)
    {
        if (SoC < 70)
            SoC += energy_percent_every_time_slot_1;
        else if (SoC < 80)
            SoC += energy_percent_every_time_slot_2;
        else
            SoC += energy_percent_every_time_slot_3;
        node_corresponding_SoC[n++] = SoC > 100? 100 : SoC;
    }

    rate_sequence new_column;
    for (int t = 0; t < number_time_slot; ++t) // Initialize new_column
    {
        new_column.PR[t] = 0;
        new_column.SoC[t] = total_buses[BN].ISoC;
    }
    new_column.SoC[number_time_slot] = total_buses[BN].ISoC;

    for (int t = 0; t < number_time_slot; ++t)
    {
        if (fixed_decisions[BN][t] == 1)
            new_column.PR[t] = 50;
        if (fixed_decisions[BN][t] == 2)
            new_column.PR[t] = 150;
    }

    int cursor = 0;
    for (int t = 1; t < number_time_slot + 1; ++t)
    {
        if (new_column.PR[t - 1] == 50)
            cursor++;
        if (new_column.PR[t - 1] == 150)
            cursor += 3;
        new_column.SoC[t] = node_corresponding_SoC[cursor];
    }

    new_column.delta = total_buses[BN].RSoC - new_column.SoC[number_time_slot] > 0? total_buses[BN].RSoC - new_column.SoC[number_time_slot]:0;

    if (if_exist_the_same_column(BN, new_column))
    {
        // add to bus_columns and problem_columns
        bus_columns[BN].push_back(new_column);
        problem_columns.push_back(column_information(BN, bus_columns[BN].size() - 1, new_column.delta));

        // add to the problem model
        add_column_to_model(new_column); // add column to the model
    }
}

void resolve_LP()
{
    model.resolve();

    const double* solution;
    solution = model.getColSolution();
    for (int i = 0; i < problem_columns.size(); ++i)
        chi[i] = *(solution + i);
    const double* dualvariables;
    dualvariables = model.getRowPrice();
    for (int i = 0; i < number_bus; ++i)
        pi_1b[i] = *(dualvariables + i);
    for (int i = 0; i < number_charger; ++i)
        for (int j = 0; j < number_time_slot; ++j)
            pi_1c[i][j] = *(dualvariables + number_bus + i * number_time_slot + j);
    for (int i = 0; i < number_charger; ++i)
        for (int j = 0; j < number_time_slot; ++j)
            pi_1d[i][j] = *(dualvariables + number_bus + number_charger * number_time_slot + i * number_time_slot + j);
    for (int i = 0; i < number_time_slot; ++i)
        pi_1e[i] = *(dualvariables + number_bus + 2 * number_charger * number_time_slot + i);
}

//void use_BCB()
//{
//    for (int i = 0; i < problem_columns.size(); ++i)
//        model.setInteger(i);
//    CbcModel cbcmodel(model);
////    cbcmodel.setLogLevel(0); // Don't print the log
//    cbcmodel.branchAndBound();
//    const double* ILP_solution;
//    ILP_solution = cbcmodel.getCbcColSolution();
//    std::cout<<cbcmodel.getBestPossibleObjValue()<<std::endl;
//}

bool cmp(round_indicator A, round_indicator B)
{
    if (A.probability != B.probability)
        return A.probability > B.probability;
    else
        return false;
}

bool cmp_solution(column_information A, column_information B)
{
    if (A.BN != B.BN)
        return A.BN < B.BN;
    else
        return false;
}

bool if_satisfy_constraints(int BN, int t, int rate)
{
    int other_BN = BN % 2 ? BN - 1: BN + 1;
    if (rate == 1)
        if (fixed_decisions[other_BN][t] == 2)
            return false;
    if (rate == 2)
        if (fixed_decisions[other_BN][t] == 1 || fixed_decisions[other_BN][t] == 2)
            return false;
    int fixed_total_power = 0;
    for (int other_BN = 0; other_BN < number_bus; ++other_BN)
    {
        if (other_BN != BN)
        {
            if (fixed_decisions[other_BN][t] == 1)
                fixed_total_power += 50;
            if (fixed_decisions[other_BN][t] == 2)
                fixed_total_power += 150;
        }
    }
    if (rate == 1)
        rate = 50;
    if (rate == 2)
        rate = 150;
    if (fixed_total_power + rate > depo_power[t] - base_load_power)
        return false;
    return true;
}

void how_many_unfixed()
{
    int counter = 0;
    for (int i = 0; i < number_bus; ++i)
        for (int j = 0; j < number_time_slot; ++j)
            if (fixed_decisions[i][j] != -1)
                counter++;
    how_many_fixed = counter;
}

struct data_for_greedy
{
    double urgence;
    int BN;
};

bool cmp_for_greedy(data_for_greedy A, data_for_greedy B)
{
    if (A.urgence != B.urgence)
        return A.urgence > B.urgence;
    else
        return false;
}

int greedy()
{
    int if_OK[22] = {0};
    std::vector<data_for_greedy> vec_for_greedy;
    double SoC_ts[22][number_time_slot + 1];
    for (int b = 0; b < 22; ++b)
        SoC_ts[b][0] = total_buses[b].ISoC;
    double points[number_time_slot];
    for (int t = 0; t < number_time_slot; ++t)
    {
        points[t] = (depo_power[t] - base_load_power) / 50;
    }
    int powers_ts[22][number_time_slot] = {0};
    for (int t = 0; t < number_time_slot; ++t)
    {
        vec_for_greedy.clear();
        for (int b = 0; b < 22; ++b)
        {
            SoC_ts[b][t+1] = SoC_ts[b][t];
            if (total_buses[b].ATS <= t && total_buses[b].DL > t && SoC_ts[b][t] < total_buses[b].RSoC)
            {
                double urgence = (total_buses[b].RSoC - SoC_ts[b][t]) / (total_buses[b].DL - t + 1);
                data_for_greedy bus_data;
                bus_data.urgence = urgence;
                bus_data.BN = b;
                vec_for_greedy.push_back(bus_data);
            }
        }
        std::sort(vec_for_greedy.begin(), vec_for_greedy.end(), cmp_for_greedy);

        while (points[t] > 0 && vec_for_greedy.size() > 0)
        {
            int b = vec_for_greedy[0].BN;
            if (b % 2 == 0)
            {
                if (points[t] >= 3)
                {
                    if (powers_ts[b + 1][t] == 0)
                    {
                        powers_ts[b][t] = 3;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;

                        points[t] -= 3;
                    }
                }
                else if (points[t] >= 1)
                {
                    if (powers_ts[b + 1][t] <= 1)
                    {
                        powers_ts[b][t] = 1;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;

                        points[t] -= 1;
                    }
                }
            }
            else if (b % 2 == 1)
            {
                if (points[t] >= 3)
                {
                    if (powers_ts[b - 1][t] == 0)
                    {
                        powers_ts[b][t] = 3;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;
                        points[t] -= 3;
                    }
                }
                else if (points[t] >= 1)
                {
                    if (powers_ts[b - 1][t] <= 1)
                    {
                        powers_ts[b][t] = 1;

                        if (SoC_ts[b][t+1] <= 70)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_1;
                        else if (SoC_ts[b][t+1] <= 80)
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_2;
                        else
                            SoC_ts[b][t+1] += energy_percent_every_time_slot_3;

                        points[t] -= 1;
                    }
                }
            }
            vec_for_greedy.erase(vec_for_greedy.begin());
        }
        for (int b = 0; b < number_bus; ++b)
            if (SoC_ts[b][t] >= total_buses[b].RSoC)
                if_OK[b] = 1;
    }
    return 0;
}


void remove_violating_columns()
{
    for (std::vector<column_information>::iterator it = problem_columns.begin(); it != problem_columns.end();) // here all-zero columns always exist
    {
        int BN = it->BN;
        int ColN = it->ColN;
        for (int t = 0; t < number_time_slot; ++t)
        {
            if (fixed_decisions[BN][t] == 0 && bus_columns[BN][ColN].PR[t] !=0)
            {
                int index = std::distance(problem_columns.begin(), it);
                int col[1];
                col[0] = index;
                model.deleteCols(1, col);
                it = problem_columns.erase(it);
                it--;
                break;
            }
            if (fixed_decisions[BN][t] == 1 && bus_columns[BN][ColN].PR[t] !=50)
            {
                int index = std::distance(problem_columns.begin(), it);
                int col[1];
                col[0] = index;
                model.deleteCols(1, col);
                it = problem_columns.erase(it);
                it--;
                break;
            }
            if (fixed_decisions[BN][t] == 2 && bus_columns[BN][ColN].PR[t] !=150)
            {
                int index = std::distance(problem_columns.begin(), it);
                int col[1];
                col[0] = index;
                model.deleteCols(1, col);
                it = problem_columns.erase(it);
                it--;
                break;
            }
        }
        it++;
    }
}

int main()
{
    initialization();
//    greedy();

    auto start = std::chrono::high_resolution_clock::now();

/*************** find the minimal feasible t via bisection: start ***************/
    std::cout <<"Now finding the minimal t_star for LP via bi-section: "<< std::endl;

    double ObjValue;
    int t0 = 0;
    int tT = number_time_slot - 1;
    while (tT - t0 > 1)
    {
        model.reset();
        for (int i = 0; i < number_bus; ++i)
            bus_columns[i].clear();
        problem_columns.clear();

        model.setLogLevel(0);

        int t_star = (t0 + tT) / 2;

        initialization();
        solve_initial_LP();
        // First stage: add columns until that can not add
        bool flag_unchanged_columns = false;
        while (!flag_unchanged_columns) {
            int current_number_of_columns = problem_columns.size();
            for (int b = 0; b < number_bus; ++b)
                get_min_reduced_cost_add_the_column(b, t_star); //  t_star is included
            if (current_number_of_columns != problem_columns.size())
                resolve_LP();
            else
                flag_unchanged_columns = true;
        }

        std::cout << "When t* = " << t_star << ", LP's ObjValue is " << model.getObjValue() << std::endl;
        if (model.getObjValue() > 0)
            t0 = t_star;
        else
            tT = t_star;
    }

/*************** find the minimal feasible t for LP via bisection: end ***************/

    std::cout <<"\nFound the minimal t_star for LP, it is "<<tT<<"."<<std::endl;
    std::cout <<"\nNow finding the minimal t_star for RCGA via step-by-step: "<<std::endl;

    // step-by-step to find the feasible minimal t
    for (int t_star = tT; t_star < number_time_slot; t_star++)
    {
        model.reset();
        for (int i = 0; i < number_bus; ++i)
            bus_columns[i].clear();
        problem_columns.clear();

        model.setLogLevel(0);

        initialization();
        solve_initial_LP();

//        for (int i = 0; i < problem_columns.size(); ++i)
//            model.setInteger(i);
//        model.writeLp("/Users/ZwYu/test");

        // First stage: add columns until that can not add
        bool flag_unchanged_columns = false;
        while (!flag_unchanged_columns) {
            int current_number_of_columns = problem_columns.size();
            for (int b = 0; b < number_bus; ++b)
                get_min_reduced_cost_add_the_column(b, t_star); //  t_star is included
            if (current_number_of_columns != problem_columns.size())
                resolve_LP();
            else
                flag_unchanged_columns = true;
        }

        // Second stage: round (fix)
        double possibility_of_rates[number_bus][number_time_slot][3];// 3 = {0, 50, 150}
        std::vector<round_indicator> to_be_fixed_decisions; // to store decisions whose sigma_chi < 0.99

        int rounding_times = 0;
        while (rounding_times++ < 1000)
        {
            memset(possibility_of_rates, 0, sizeof(possibility_of_rates));
            to_be_fixed_decisions.clear();
            for (int i = 0; i < problem_columns.size(); ++i) // calculate the indicators
            {
                int BN = problem_columns[i].BN;
                int ColN = problem_columns[i].ColN;
                for (int t = 0; t < number_time_slot; ++t)
                {
                    if (bus_columns[BN][ColN].PR[t] == 0)
                        possibility_of_rates[BN][t][0] += chi[i];
                    if (bus_columns[BN][ColN].PR[t] == 50)
                        possibility_of_rates[BN][t][1] += chi[i];
                    if (bus_columns[BN][ColN].PR[t] == 150)
                        possibility_of_rates[BN][t][2] += chi[i];
                }
            }
            // fix the decisions that are one
            for (int BN = 0; BN < number_bus; ++BN)
                for (int t = 0; t < number_time_slot; ++t)
                    for (int rate = 0; rate < 3; ++rate) // 0:0, 1:50, 2:150.
                    {
                        double possibility = possibility_of_rates[BN][t][rate];
                        if (0.99 <= possibility && fixed_decisions[BN][t] == -1)
                            fixed_decisions[BN][t] = rate; // 0, 1, 2 = 0, 50, 150 needs to be unified later..
                        else if (possibility > 0 && fixed_decisions[BN][t] == -1)
                            to_be_fixed_decisions.push_back(round_indicator(BN, t, rate, possibility));
                    }
            // find the most possible variable to fix
            std::sort(to_be_fixed_decisions.begin(), to_be_fixed_decisions.end(), cmp);
            if (to_be_fixed_decisions.size() > 1) // fix process
            {
                for (int i = 0; i < to_be_fixed_decisions.size(); ++i)
                {
                    int BN = to_be_fixed_decisions[i].BN;
                    int t = to_be_fixed_decisions[i].t;
                    int rate = to_be_fixed_decisions[i].rate;
                    if (if_satisfy_constraints(BN, t, rate))
                    {
                        fixed_decisions[BN][t] = rate; // 0, 1, 2 (not 0, 50, 150) needs to be unified later...
                        break;
                    }
                }
            }
            else
            {
                remove_violating_columns();
                resolve_LP();
                std::cout<<"When t* = "<<t_star<<", RCGA gets ObjValue that is "<< model.getObjValue()<<std::endl;
                break;
            }

            // delete the violating columns
            remove_violating_columns();

            // add reparative columns to make the model feasible
            for (int BN = 0; BN < number_bus; ++BN)
                add_extra_column(BN);

            how_many_unfixed();
            // resolve the problem to get the values of the dual variables
            resolve_LP();

            flag_unchanged_columns = false;
            while (!flag_unchanged_columns)
            {
                int current_number_of_columns = problem_columns.size();
                for (int b = 0; b < number_bus; ++b)
                    get_min_reduced_cost_add_the_column_fixed_version(b, t_star); //  t_star is included
                if (current_number_of_columns != problem_columns.size())
                    resolve_LP();
                else
                    flag_unchanged_columns = true;
            }
            if (model.getObjValue() > 0)
            {
                std::cout<<"The semi-fixed LP's ObjVal is greater than zero, so skip this t ("<<t_star<<")."<<std::endl;
                break;
            }
        }
        if (model.getObjValue() == 0)
            break;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = (end - start).count();
    std::cout << "Running time: " <<std::setprecision(10)<< duration / 1000000.0 << "ms" << std::endl;

    int power_consumption[number_time_slot] = {0};
    for (int t = 0; t < number_time_slot; ++t)
    {
        for (int b = 0; b < number_bus; ++b)
            if (fixed_decisions[b][t] == 2)
                power_consumption[t] += 150;
            else if (fixed_decisions[b][t] == 1)
                power_consumption[t] += 50;
        power_consumption[t] += 500;
    }

    return 0;
}
// Use Column Generation with COIN-OR CLP to solve the Electric Vehicle Scheduling Problem
// 2022.06.15 Zhanwei Yu
#include <iostream>
#include <string>
#include "OsiClpSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "parameters.h"
#include "SPFA.h"

struct bus
{
    int CN; // Charger number
    int ATS; // Available time slot
    double ISoC; // Initial state of charge
};

struct rate_sequence
{
    int PR [number_time_slot]; // Power rates; rate: 0, 50, or 150
    double SoC[number_time_slot]; // The state of charges; depend on the initial state and the rates array...
    double delta;
};

struct column_information
{
    int BN; // Bus number
    int ColN; // Column number (i.e., the index in the corresponding buses_columns[BN])
    double delta; // The corresponding delta
};

bus total_buses[number_bus]; // List of busus; pre-input information
int chargers[number_charger][2];

rate_sequence buses_columns[number_bus][10000]; // Each bus has an array to store the columns (i.e., rate_sequence); Can use vector to automatically adapt
int buses_columns_cursor[number_bus] = {0}; // The cursor counts the number of columns of the buses; Initialization: All zero

// All the columns be added up by 1-dimension
column_information total_columns [number_bus * 10000]; // The total number of columns is the sum of buses_columns_cursor[number_bus]
double chi[number_bus * 10000]; // Store chi-variables
int total_columns_cursor = 0; // Counter the number of total_columns
double pi[number_bus + 2 * number_charger * number_time_slot + number_time_slot]; // Store the dual variables
std::string constraints_names[number_bus + 2 * number_charger * number_time_slot + number_time_slot]; // The constraints names for pi

double depo_power[number_time_slot]; // 6:00-22:00 is 700 kVA; 22:00-6:00 is 1400 kVA
// time slot starts at 18:00

void initialize_chargers()
{
    int bus_n = 0;
    for (int i = 0; i < number_charger; ++i)
    {
        chargers[i][0] = bus_n++;
        chargers[i][1] = bus_n++;
    }
}

void initialize_constraints_names()
{
    for (int i = 0; i < number_bus; ++i)
        constraints_names[i] = "(1b) b=" + std::to_string(i);
    for (int i = number_bus; i < number_bus + number_charger * number_time_slot; ++i)
        constraints_names[i] = "(1c) c=" + std::to_string((i - number_bus) / number_time_slot) + " t=" + std::to_string((i - number_bus) % number_time_slot);
    for (int i = number_bus + number_charger * number_time_slot; i < number_bus + 2 * number_charger * number_time_slot; ++i)
        constraints_names[i] = "(1d) c=" + std::to_string((i - number_bus - number_charger * number_time_slot) / number_time_slot) + " t=" + std::to_string((i - number_bus + number_charger * number_time_slot) % number_time_slot);
    for (int i = number_bus + 2 * number_charger * number_time_slot; i < number_bus + 2 * number_charger * number_time_slot + number_time_slot; ++i)
        constraints_names[i] = "(1e) t=" + std::to_string(i - number_bus - 2 * number_charger * number_time_slot);
}

void initialize_depo_power() // Need to rewrite
{
    for (int i = 0; i < 8; ++i)
        depo_power[i] = 700;
    for (int i = 8; i < number_time_slot; ++i)
        depo_power[i] = 1400;
}

void initialize_total_buses() // Need to rewrite via reading file to initialize
{
    total_buses[0].CN = 0; // Charger number is 0
    total_buses[0].ATS = 0; // Available time slot is 0
    total_buses[0].ISoC = 20.0; // Initial state of charge

    total_buses[1].CN = 0; // Charger number is 0
    total_buses[1].ATS = 1; // Available time slot is 0
    total_buses[1].ISoC = 30.0; // Initial state of charge

    total_buses[2].CN = 1; // Charger number is 0
    total_buses[2].ATS = 1; // Available time slot is 0
    total_buses[2].ISoC = 36.0; // Initial state of charge

    total_buses[3].CN = 1; // Charger number is 0
    total_buses[3].ATS = 2; // Available time slot is 0
    total_buses[3].ISoC = 28.0; // Initial state of charge
}

void initialize_columns() // Initialize: to add all-zero columns
{
    for (int i = 0; i < number_bus; ++i)
    {
        for (int j = 0; j < number_time_slot; ++j)
        {
            buses_columns[i][0].PR[j] = 0;
            buses_columns[i][0].SoC[j] = total_buses[i].ISoC;
        }
        buses_columns[i][0].delta = target_state_of_charge - buses_columns[i][0].SoC[number_time_slot - 1] > 0 ? target_state_of_charge - buses_columns[i][0].SoC[number_time_slot - 1] : 0;
        total_columns[total_columns_cursor].BN = i;
        total_columns[total_columns_cursor].ColN = 0;
        total_columns[total_columns_cursor].delta = buses_columns[i][0].delta;
        total_columns_cursor++;
        buses_columns_cursor[i]++;
    }
}

void initialization()
{
    initialize_chargers();
    initialize_constraints_names();
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
        objective[i] = total_columns[i].delta;
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
    for (int i = 0; i < number_bus + 2 * number_charger * number_time_slot + number_time_slot; ++i)
        pi[i] = *(dualvariables + i);

//    int n = model.getNumCols();
//    int m = model.getNumRows();
    // The follows are the solution and dual variables

//    if ( model.isProvenOptimal() )
//    {
//        std::cout << "Found optimal solution!" << std::endl;
//        std::cout << "Objective value is " << model.getObjValue() << std::endl;
//        const double* solution;
//        solution = model.getColSolution();
//        for (int i = 0; i < number_bus; ++i)
//            chi[i] = *(solution + i);
//        const double* dualvariables;
//        dualvariables = model.getRowPrice();
//        for (int i = 0; i < number_bus + 2 * number_charger * number_time_slot + number_time_slot; ++i)
//            pi[i] = *(dualvariables + i);
//    }
//    else
//    {
//        std::cout << "Didn’t find optimal solution." << std::endl;
//    }
}

int main()
{
    initialization();
    solve_initial_LP();
    return 0;
}

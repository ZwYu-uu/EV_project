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

int how_many = 0;

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
        depo_power[t] = 1400;
    for (int t = 47; t < number_time_slot; ++t)
        depo_power[t] = 700;
}

void initialize_total_buses() // Need to rewrite via reading file to initialize
{
//    total_buses[0] = Bus(0, false, 1, 44, 0, 69.7);
//    total_buses[1] = Bus(0, true, 3, 47, 0, 67.4);
//    total_buses[2] = Bus(1, false, 4, 48, 0, 68.9);
//    total_buses[3] = Bus(1, true, 4, 49, 0, 67.4);
//    total_buses[4] = Bus(2, false, 9, 50, 0, 66.0);
//    total_buses[5] = Bus(2, true, 10, 55, 0, 69.1);
//    total_buses[6] = Bus(3, false, 3, 43, 0, 71.9);
//    total_buses[7] = Bus(3, true, 2, 44, 0, 66.0);
//    total_buses[8] = Bus(4, false, 4, 47, 0, 68.9);
//    total_buses[9] = Bus(4, true, 1, 48, 0, 64.4);
//    total_buses[10] = Bus(5, false, 4, 50, 0, 64.9);
//    total_buses[11] = Bus(5, true, 11, 51, 0, 72.9);
//
//    total_buses[12] = Bus(6, false, 2, 44, 0, 72.0); //12
//    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);
//
//    total_buses[14] = Bus(7, false, 3, 47, 0, 66.0); //13
//    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);
//
//    total_buses[16] = Bus(8, false, 2, 48, 0, 64.9); //14
//    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);
//
//    total_buses[18] = Bus(9, false, 9, 49, 0, 72.9); //15
//    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);
//
//    total_buses[20] = Bus(10, false, 10, 51, 0, 71.5); //16
//    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
    total_buses[0] = Bus(0, false, 1, 44, 30.3, 69.7);
    total_buses[1] = Bus(0, true, 3, 47, 32.6, 67.4);
    total_buses[2] = Bus(1, false, 4, 48, 31.1, 68.9);
    total_buses[3] = Bus(1, true, 4, 49, 32.6, 67.4);
    total_buses[4] = Bus(2, false, 9, 50, 34.0, 66.0);
    total_buses[5] = Bus(2, true, 10, 55, 30.9, 69.1);
    total_buses[6] = Bus(3, false, 3, 43, 28.1, 71.9);
    total_buses[7] = Bus(3, true, 2, 44, 34.0, 66.0);
    total_buses[8] = Bus(4, false, 4, 47, 31.1, 68.9);
    total_buses[9] = Bus(4, true, 1, 48, 35.6, 64.4);
    total_buses[10] = Bus(5, false, 4, 50, 35.1, 64.9);
    total_buses[11] = Bus(5, true, 11, 51, 27.0, 72.9);

    total_buses[12] = Bus(6, false, 2, 44, 28.0, 72.0); //12
    total_buses[13] = Bus(6, true, 0, number_time_slot - 1, 100.0, 100.0);

    total_buses[14] = Bus(7, false, 3, 47, 34.0, 66.0); //13
    total_buses[15] = Bus(7, true, 0, number_time_slot - 1, 100.0, 100.0);

    total_buses[16] = Bus(8, false, 2, 48, 35.1, 64.9); //14
    total_buses[17] = Bus(8, true, 0, number_time_slot - 1, 100.0, 100.0);

    total_buses[18] = Bus(9, false, 9, 49, 27.1, 72.9); //15
    total_buses[19] = Bus(9, true, 0, number_time_slot - 1, 100.0, 100.0);

    total_buses[20] = Bus(10, false, 10, 51, 28.5, 71.5); //16
    total_buses[21] = Bus(10, true, 0, number_time_slot - 1, 100.0, 100.0);
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

    // add to bus_columns and problem_columns
    bus_columns[BN].push_back(new_column);
    problem_columns.push_back(column_information(BN, bus_columns[BN].size() - 1, new_column.delta));

    // add to the problem model
    add_column_to_model(new_column); // add column to the model

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
    how_many = counter;
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
    auto start = std::chrono::high_resolution_clock::now();

/*************** find the minimal feasible t via bisection: start ***************/

    double ObjValue;
    int t0 = 0;
    int tT = number_time_slot - 1;
    int t_star = 51;
//    while (tT - t0 > 1)
    while (tT - t_star > 1)
    {
        model.reset();
        for (int i = 0; i < number_bus; ++i)
            bus_columns[i].clear();
        problem_columns.clear();

//        model.setLogLevel(0);

//        int t_star = (t0 + tT) / 2;

        initialization();
        solve_initial_LP();
        // First stage: add columns until that can not add
        bool flag_unchanged_columns = false;
        while (!flag_unchanged_columns)
        {
            int current_number_of_columns = problem_columns.size();
            for (int b = 0; b < number_bus; ++b)
                get_min_reduced_cost_add_the_column(b, t_star); //  t_star is included
            if (current_number_of_columns != problem_columns.size())
                resolve_LP();
            else
                flag_unchanged_columns = true;
        }

        for (int i = 0; i < problem_columns.size(); ++i)
            model.setInteger(i);
        model.writeLp("/Users/ZwYu/Desktop/ILP");

        std::cout<<"When t* = "<<t_star<<", LP's ObjValue is "<<model.getObjValue()<<std::endl;

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
                std::cout<<"When t* = "<<t_star<<", RCGA gets ObjValue that is "<< model.getObjValue()<<std::endl; // in case...
                break;
            }
            how_many_unfixed();

            // delete the violating columns
            remove_violating_columns();

            // add reparative columns to make the model feasible
            for (int BN = 0; BN < number_bus; ++BN)
                add_extra_column(BN);

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
        }

//        // To store the solution: start //
//        std::vector<column_information> solution_columns;
//        int ccc = 0;
//        for (int i = 0; i < problem_columns.size(); ++i)
//        {
//            if (abs(chi[i] - 1) < 1e-6)
//            {
//                solution_columns.push_back(problem_columns[i]);
//                ccc++;
//            }
//        }
//        std::sort(solution_columns.begin(), solution_columns.end(), cmp_solution);
//        for (int i = 0; i < solution_columns.size(); ++i)
//            std::cout<<"BN: "<<solution_columns[i].BN<<", ColN: "<<solution_columns[i].ColN<<std::endl;
//        // To store the solution: end //

//        ObjValue = model.getObjValue();
//        if (ObjValue > 0)
//            t0 = t_star;
//        else
//            tT = t_star;
        t_star++;
    }

/*************** find the minimal feasible t via bisection: end ***************/

/*
//=============== resolve for minimal feasible t: start ===============//

    model.reset();
    for (int i = 0; i < number_bus; ++i)
        bus_columns[i].clear();
    problem_columns.clear();

    model.setLogLevel(0);

    int t_star = tT;

    initialization();
    solve_initial_LP();
    // First stage: add columns until that can not add
    bool flag_unchanged_columns = false;
    while (!flag_unchanged_columns)
    {
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

    int max_rounding_times = 1000;
    while (max_rounding_times--)
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
            std::cout<<"\n\nThe minimal t* = "<<t_star<<", RCGA gets ObjValue that is "<< model.getObjValue()<<std::endl; // in case...
            break;
        }
        // delete the violating columns
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

        for (int BN = 0; BN < number_bus; ++BN)
        {
            add_extra_column(BN);
        }

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

    }

// To store the solution: start //
    std::vector<column_information> solution_columns;
    int ccc = 0;
    for (int i = 0; i < problem_columns.size(); ++i)
    {
        if (chi[i] == 1)
        {
            solution_columns.push_back(problem_columns[i]);
            ccc++;
        }
    }
    std::sort(solution_columns.begin(), solution_columns.end(), cmp_solution);
    for (int i = 0; i < solution_columns.size(); ++i)
        std::cout<<"BN: "<<solution_columns[i].BN<<", ColN: "<<solution_columns[i].ColN<<std::endl;
// To store the solution: end //

//=============== resolve for minimal feasible t: end ===============//
*/
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = (end - start).count();
    std::cout << "Running time: " <<std::setprecision(10)<< duration / 1000000.0 << "ms" << std::endl;

    return 0;
}

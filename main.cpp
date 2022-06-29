// Use Column Generation with COIN-OR CLP to solve the Electric Vehicle Scheduling Problem
// 2022.06.15 Zhanwei Yu

#include <iostream>
#include <string>
#include <vector>
#include "CbcModel.hpp"
#include "OsiClpSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "parameters.h"
#include "Bus.h"
#include "SPFA.h"

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

void initialize_chargers()
{
    int bus_n = 0;
    for (int i = 0; i < number_charger; ++i)
    {
        chargers[i][0] = bus_n++;
        chargers[i][1] = bus_n++;
    }
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
    total_buses[0] = Bus(0, false, 0, number_time_slot - 1, 60.0);
    total_buses[1] = Bus(0, true, 2, number_time_slot - 1, 72.0);
    total_buses[2] = Bus(1, false, 3, number_time_slot - 1, 78.0);
    total_buses[3] = Bus(1, true, 4, number_time_slot - 1, 34.0);
}

void initialize_columns() // Initialize: to add all-zero columns
{
    for (int i = 0; i < number_bus; ++i)
    {
        bus_columns[i].push_back(rate_sequence(total_buses[i].ISoC));
        problem_columns.push_back(column_information(i, bus_columns[i].size() - 1, bus_columns[i][bus_columns[i].size() - 1].delta));
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
    int n = ceil((100 - total_buses[BN].ISoC) / energy_percent_every_time_slot); // The number of nodes in each time slot
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
            graph.InputAdjMat((m - 1) * n + 1 + i, m * n + 1, (target_state_of_charge - (total_buses[BN].ISoC + i * energy_percent_every_time_slot)) < 0? 0:(target_state_of_charge - (total_buses[BN].ISoC + i * energy_percent_every_time_slot)));
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
            graph.InputAdjMat((m - 1) * n + 1 + i, m * n + 1, (target_state_of_charge - (total_buses[BN].ISoC + i * energy_percent_every_time_slot)) < 0? 0:(target_state_of_charge - (total_buses[BN].ISoC + i * energy_percent_every_time_slot)));
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
        for(int t = 1; t < number_time_slot + 1; ++t)
            new_column.SoC[t] = new_column.SoC[t - 1] + energy_percent_every_time_slot * (new_column.PR[t - 1] / 50);
        new_column.delta = target_state_of_charge - new_column.SoC[number_time_slot] > 0? target_state_of_charge - new_column.SoC[number_time_slot]:0;

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
    for (int i = 0; i < problem_columns.size() - 1; ++i)
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

int main()
{
    initialization();
    solve_initial_LP();
    // t_star should be greater than any ATS!
    int t_star = 9;
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
    // Second stage: round (fix) chi
    for (int i = 0; i < problem_columns.size(); ++i)
        model.setInteger(i);
    CbcModel cbcmodel(model);
    cbcmodel.branchAndBound();
    const double* ILP_solution;
    ILP_solution = cbcmodel.getCbcColSolution();
    std::cout<<cbcmodel.getBestPossibleObjValue()<<std::endl;
    return 0;
}

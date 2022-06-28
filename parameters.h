//
// Created by ZwYu on 2022-06-20.
//

#ifndef EV_PROJECT_PARAMETERS_H
#define EV_PROJECT_PARAMETERS_H

#define number_bus 4
#define number_charger number_bus/2
#define number_time_slot 24 // 18:00 ~ 6:00; 24 = 12h / 30min
#define target_state_of_charge 85
#define base_load_power 500 // base load power is 500kW
#define bus_battery_capacity 564 // bus battery capacity is 564kWh
#define energy_percent_every_time_slot 4.4 // 50kW * 30min / 60min / 564KWh = 4.4%
#define number_of_row (number_bus + 2 * number_charger * number_time_slot + number_time_slot) // For construct a column


#endif //EV_PROJECT_PARAMETERS_H
//
// Created by ZwYu on 2022-06-20.
//

#ifndef EV_PROJECT_PARAMETERS_H
#define EV_PROJECT_PARAMETERS_H

#define number_bus 10
#define number_charger number_bus/2
#define number_time_slot 48 // 18:00 ~ 6:00; 48 = 12h / 15min
#define base_load_power 500 // base load power is 500kW
#define energy_percent_every_time_slot_1 2.2 // 50kW * 15min / 60min / 564KWh = 2.2% (15min/ts), or 4.4% (30min/ts)
#define energy_percent_every_time_slot_2 1.75 // 50kW * 80% * 15min / 60min / 564KWh = 1.75% (15min/ts), or 3.5% (30min/ts)
#define energy_percent_every_time_slot_3 1.35 // 50kW * 60% * 15min / 60min / 564KWh = 1.35% (15min/ts), or 2.7% (30min/ts)
#define number_of_row (number_bus + 2 * number_charger * number_time_slot + number_time_slot) // For construct a column

#define bus_battery_capacity 564 // bus battery capacity is 564kWh
#define energy_per_km 1.8 // 1.8kWh energy consumption per 1 km = 0.32%

#endif //EV_PROJECT_PARAMETERS_H
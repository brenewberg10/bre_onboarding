builtin_interfaces/Time stamp

uint8 track_flag_ack

uint8 vehicle_flag_ack

# Defines transitions for the Competition Team computer to communicate with Raptor.
uint8 ct_state

bool race_control_pit_summon # whether we have black flag

bool race_control_pass_allowed # whether we have attacker flag & waving green flag

bool race_control_attacker_commanded # whether we have attacker flag

bool race_control_defender_commanded # whether we have defender flag

float32 round_speed

# counter as a watchdog
uint8 rolling_counter

# vehicle number
uint8 vehicle_id

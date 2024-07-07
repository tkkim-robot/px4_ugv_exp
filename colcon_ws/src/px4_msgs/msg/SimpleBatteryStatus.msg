uint64 timestamp			# time since system start (microseconds)
float32 voltage_v			# Battery voltage in volts, 0 if unknown
float32 voltage_filtered_v	# Battery voltage in volts, filtered, 0 if unknown
float32 current_a			# Battery current in amperes, -1 if unknown
float32 current_filtered_a	# Battery current in amperes, filtered, 0 if unknown
float32 current_average_a	# Battery current average in amperes, -1 if unknown
float32 discharged_mah		# Discharged amount in mAh, -1 if unknown
float32 remaining			# From 1 to 0, -1 if unknown
float32 scale				# Power scaling factor, >= 1, or -1 if unknown
float32 time_remaining_s	# predicted time in seconds remaining until battery is empty under previous averaged load, NAN if unknown
uint8 cell_count			# Number of cells, 0 if unknown

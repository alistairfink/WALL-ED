# uint8_t representing terrain of tiles:
uint8 terrain 				
uint8 TERRAIN_UKNOWN	= 0				
uint8 TERRAIN_FLAT 		= 1			
uint8 TERRAIN_GRAVEL	= 2
uint8 TERRAIN_SAND 		= 3	
uint8 TERRAIN_WATER 	= 4	

# uint8_t representing target in tile:
uint8 target
uint8 TARGET_UKNOWN_UNDERTERMINED	= 0
uint8 TARGET_NONE 					= 1
uint8 TARGET_OCCUPIED				= 2
uint8 TARGET_MANSION				= 3
uint8 TARGET_CABIN					= 4
uint8 TARGET_SUPPLIES				= 5
uint8 TARGET_FIRE					= 6

# uint32_t count of number of occupied occupancy grid cells in tile:
uint32 occupancy_count

# Records if tile is visited
bool visited

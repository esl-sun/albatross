#ifndef MISC
#define MISC

void calibrateSensors(double* means);
char* createNewLog();
int gps_lock_status(struct gps_data_t* gps_data);
void get_gps_coord(struct gps_data_t* gps_data, int* error_code);
void decode_spektrum(uint16_t rx_length, uint16_t channel_id, uint32_t* control, uint8_t* message, uint16_t step);
void get_heading_distance(double curr_coord, double dest_coord, double distance, double bearing);

#endif

// ==============================================================
// ======================== SETTING =============================
// ==============================================================

#define F_CFU 16000000
#define BAUD  115200
#define UBRR0_SET ((F_CPU / (BAUD * 8UL)) - 1)

#define bit_value(bit) (1 << (bit))

#define HIGH 0x1
#define LOW  0x0

typedef enum {
	GetDist = 0x1,
	GetWater = 0x2,
	GetTemp = 0x3,
	GetAll = 0x4
} StructType;

typedef enum {
	DistFloat = 0x1,
	WaterFrom = 0x2,
	WaterTo = 0x3,
	TempFloat = 0x4,
	HumFloat = 0x5
} FloatType;

typedef struct DistSensor {
	char* desc;
	float data;
	uint8_t trig_bit;
	uint8_t echo_bit;
} DistSensor;

typedef struct WaterSensor {
	char* desc;
	float from;
	float to;
	uint8_t analog_bit;
} WaterSensor;

typedef struct TempSensor {
	char* temp_desc;
	char* hum_desc;
	float temp_data;
	float hum_data;
	uint8_t digital_bit;
} TempSensor;

typedef struct LogStruct {
	DistSensor* ds;
	WaterSensor* ws;
	TempSensor* ts;
	int write_count;
	int read_count;
	StructType type;
} LogStruct;

typedef struct Buffer {
    void* data;
    int next;
    int size;
} Buffer;

void struct_init(DistSensor* dist_sensor, WaterSensor* water_sensor, TempSensor* temp_sensor, LogStruct* log_struct, Buffer* buffer);
void reset_buffer(Buffer* b);
void reserve_space(Buffer* buffer, int bytes);
void serialize_string(char* str, Buffer* b);
void serialize_float(float x, FloatType float_type, Buffer* b);
void packet_serialize(LogStruct* ls, Buffer* b);

void UART_init();

void EEPROM_read(char* str, int offset, int size);
void EEPROM_write(char* str, int offset, int size);
void EEPROM_read_int(int* data, int offset);
void EEPROM_write_int(int data, int offset);
void EEPROM_erase(int n);
void EEPROM_init();

void dist_sensor_init();
void begin_trig();
int pulseIn(uint8_t state, int timeout);
void dist_sensor_active();

void analog_init();
void analog_sensor_init();
uint16_t analog_read();
void analog_sensor_active();

void temp_request();
void temp_response();
uint8_t temp_data();
void temp_sensor_active();

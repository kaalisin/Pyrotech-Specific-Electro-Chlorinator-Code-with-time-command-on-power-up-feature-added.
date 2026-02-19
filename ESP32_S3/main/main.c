// Header Files
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "arch/sys_arch.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lwip/stats.h"
#include "lwipopts.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include <time.h>
#include "esp_sntp.h"
#include "esp_netif_types.h"
#include "esp_netif_sntp.h"
#include "esp_vfs.h"
#include "freertos/timers.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_efuse.h"
#include "esp_mac.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include <math.h>
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_littlefs.h"


// REGENERATION SUSPEND AND RESUME IS REQUIRED OR NOT
#define REGENERATION_SUSPEND_RESUME_REQ 							(0u)

// Dosing Time Macros
#define DOSING_START_TIME			  (13) // 13:00
#define DOSING_END_TIME				  (14)//  14:00

#define ADC_TOTAL_SAMPLE			  (20)


// POLARITY TYPE MACROS
#define BATCH_TYPE                     (1u)
#define TIME_TYPE					   (0u)
#define POLARITY_TYPE				(BATCH_TYPE) //BATCH TYPE IS SELECTED 


// FLOW RATE MACROS
#define FLOW_RATE_DIFF_THRESHOLD       (0.04f)
#define FLOW_RATE_MIN_VALUE            (0.10f)
#define FLOW_RATE_MAX_VALUE            (3.0f)

// Additional Features MACROS
#define RS485_MODBUS_COMM_REQ			(0ul)
#define HIGH 							(1ul)
#define LOW								(0ul)
#define MAC_ID_CMD_LENGTH				(6ul)
#define DATA_CMD_LENGTH					(21ul)
#define LEDC_DUTY_RES                (LEDC_TIMER_10_BIT)

// Salt Dosing MACROS
#define SALT_DOSING_PUMP_TIME			(15)// 15 SEC
#define ELECTROLYSIS_PROCESS_TIME       (240)// 240 minute
#define RESET_HOLD_TIME_MS 				(2000) // 2 seconds
#define MAX_CRT_FILLING_TIME			(200)// 200 Second
#define MAX_CST_FILLING_TIME			(250)// 250 Second
#define POLARITY_DIVIDER				(60)// 60 minute (+VE) polarity and other 60 minutes (-VE) polarity

// System Check MACROS
#define SYS_OK                          (1ul)
#define SYS_NOK						    (0ul)
#define SYS_UNDEF                       (2ul)

// System Mode MACROS
#define STAND_BY_MODE					(0ul)
#define OPERATING_MODE					(1ul)
#define IDLE_MODE						(2ul)


// GPIO Numbers (raw numbers, not Bit-masks)
// Digital Input Signals
#define TOP_LEVEL_CRT_SENSOR_GPIO    	(40) // External Pull-Up Resistance   
#define BOTT_LEVEL_CRT_SENSOR_GPIO      (35) // External Pull-Up Resistance 
#define TOP_LEVEL_CST_SENSOR_GPIO       (41) // External Pull-Up Resistance   
#define BOTT_LEVEL_CST_SENSOR_GPIO      (36) // External Pull-Up Resistance 
#define OPERATING_MODE_SWITCH			(37) // External Pull-UP Resistance
#define REGENERATION_SIGNAL_GPIO        (42) // External Pull-UP Resistance
#define FLOATY_SENSOR_GPIO              (38) // External Pull -UP Resistance
#define RESET_BUTTON_GPIO       		(39) // External Pull-Down Resistance 

// Digital Output Signals
#define PUMP_P1_RELAY_GPIO              (14)
#define SALT_DOSING_PUMP_D1_GPIO        (13)
#define VALVE_V1_RELAY_GPIO             (11)
#define VALVE_V2_RELAY_GPIO             (12)
#define SSR_RELAY_GPIO                  (21)
#define ELECTRODE_POLARITY_GPIO			(5)
#define DOSING_PUMP_D2_GPIO             (16)

// LED GPIO
#define LED1_GPIO                       (8)
#define LED2_GPIO                       (9)
#define LED3_GPIO                       (10)
#define LED4_GPIO                       (48)




// Analog Input Signals
// Flow Meter       Pin 3 -  ADC1_2
// Current Sensor   Pin 18 - ADC2_7
// Voltage Sensor   Pin 17 - ADC2_6 

// Communication GPIO Pins
#define UART1_TXD_GPIO					(7)
#define UART1_RXD_GPIO					(6)
#define UART0_TXD_GPIO				    (17)
#define UART0_RXD_GPIO					(18)

// LEDS ON/OFF       	
#define LED1_ON()						gpio_set_level(LED1_GPIO,1)
#define LED2_ON()						gpio_set_level(LED2_GPIO,1)
#define LED3_ON()						gpio_set_level(LED3_GPIO,1)
#define LED4_ON()						gpio_set_level(LED4_GPIO,1)

#define LED1_OFF()						gpio_set_level(LED1_GPIO,0)
#define LED2_OFF()						gpio_set_level(LED2_GPIO,0)
#define LED3_OFF()						gpio_set_level(LED3_GPIO,0)
#define LED4_OFF()						gpio_set_level(LED4_GPIO,0)


// Input Component Bits 
#define FLOATY_SENSOR_BIT               (1<<0)
#define FLOATY_NO_SENSOR_BIT 			(1<<1)

#define FLOW_METER_SENSOR_BIT		    (1<<13)
#define FLOW_METER_NO_SENSOR_BIT		(1<<2)

#define CRT_TOP_SENSOR_BIT				(1<<3)
#define CRT_NO_TOP_SENSOR_BIT		    (1<<4)

#define CRT_BOTTOM_SENSOR_BIT			(1<<5)
#define CRT_NOT_BOTTOM_SENSOR_BIT		(1<<6)

#define CST_TOP_SENSOR_BIT				(1<<7)
#define CST_NO_TOP_SENSOR_BIT			(1<<8)

#define CST_BOTTOM_SENSOR_BIT			(1<<9)
#define CST_NO_BOTTOM_SENSOR_BIT		(1<<10)

#define REG_RELAY_SENSOR_BIT			(1<<11)
#define REG_NO_RELAY_SENSOR_BIT			(1<<12)

#define ELECTROLYSIS_DONE_BIT           (1<<14)
#define OPERATING_MODE_BIT				(1<<15)

#define CRT_FILLING_TIMER_BIT           (1<<16)
#define CST_FILLING_TIMER_BIT           (1<<20)

#define CST_BOTTOM_CHANGE_BIT		    (1<<17)

// Error Bits
#define CRT_FILLING_TIMER_NO_ERROR_BIT  (1<<18)
#define CST_FILLING_TIMER_NO_ERROR_BIT  (1<<19)

// Dosing Start Bits
#define DOSING_TIME_BIT					(1<<23)
#define DOSING_STOP_TIME_BIT			(1<<21)
#define DOSING_OFF_BIT					(1<<22)




// Output Component Bits
#define CRT_PUMP_P1_BIT					(1<<0)
#define SALT_DOSING_PUMP_BIT			(1<<1)
#define VALVE_V1_BIT					(1<<2)
#define VALVE_V2_BIT					(1<<3)
#define SSR_RELAY_BIT					(1<<4)
#define POLARITY_BIT					(1<<5)
#define DOSING_PUMP_BIT                 (1<<6)

// Status Bit 
#define SYSTEM_OK_BIT					(1<<0)
#define CHLORINE_SYS_BIT				(1<<1)
#define DOSING_SYS_BIT					(1<<2)
#define REG_SYS_BIT						(1<<3)
				
	


// Electro-Chlorinator Commands
#define MAC_ID_GET_COMMAND				(0x35)
#define STATUS_GET_COMMAND              (0x45)
#define DEBUG_COMMAND                   (0x55)



// ADC OFFSET
#define Volt_OFFSET						(0.1672f)
#define Curr_OFFSET						(1.61f)  // Sensor - 4



// ERROR CODES 
#define CRT_FILL_ERROR					(1<<0)
#define ELECTROLYSIS_ERROR				(1<<1)
#define CST_FILL_ERROR					(1<<2)
#define CRT_STOP_ERROR					(1<<3)
#define ELECTROLYSIS_STOP_ERROR			(1<<4)

#define REGENERATION_ERROR              (1<<8)
		
#define DOSING_ERROR					(1<<16)

#define TIME_SYNC_ERROR                 (1<<24)




// Structure , Enumeration  and UNION
typedef enum {
	CRT_FILLING_START_OPERATION = (1<<0),
	CRT_FILLING_STOP_OPERATION = (1<<1),
	
	ELECTROLYSIS_START_OPERATION = (1<<2),
	ELECTROLYSIS_STOP_OPERATION = (1<<3),
	
	CST_FILLING_START_OPERATION = (1<<4),
	CST_FILLING_STOP_OPERTAION = (1<<5),
	
	DOSING_OPERATION = (1<<6),
	
	REGENERATION_OPERATION = (1<<7),
	
}OPERATION;


typedef enum 
{
	REG_EVENT_GRP1 = 0,
	REG_EVENT_GRP2,
}REG_EVENT_GROUP_STATUS;


typedef enum
{
	DOSE_EVENT_GP1 = 0,
	DOSE_EVENT_GP2,
}DOSE_EVENT_GROUP_STATUS;


typedef enum {
	
	EVENT_GRP1 = 0,
	EVENT_GRP2,
	EVENT_GRP3,
	EVENT_GRP4,
	EVENT_GRP5,
	EVENT_GRP6,
	EVENT_GRP7,	
}EVENTGROUP_STATUS;




typedef struct {	
  OPERATION operating_mode;
  uint16_t timer_counter;
  uint16_t tank_filling_counter;
  uint16_t tank_draining_counter;  
  uint8_t electrolysis_done_f;	
  uint8_t Current_operating_status_f;
  uint8_t dosing_done_f;
  uint8_t last_polarity_status_f;
  uint8_t dosing_counter;
  uint8_t crt_filling_timer_error_f;
  uint8_t cst_filling_timer_error_f;
  uint8_t crt_reset_counter_done_f;
  #if POLARITY_TYPE == TIME_TYPE
  uint8_t switching_polarity;
  #endif
}NVS_DATA;

typedef union
{
	uint8_t All_Pin_status;   // 8-bit complete status
	struct
	{
	    uint8_t pump1    : 1;
	    uint8_t pump2    : 1;
	    uint8_t valve1   : 1;
	    uint8_t valve2   : 1;
	    uint8_t ssr      : 1;
	    uint8_t polarity : 1;
	    uint8_t pump3    : 1;
	    uint8_t reserved : 1;   // 1 unused bit to complete 8 bits
	} Output_level;

} output_status_t;



static const char *TAG = "esp_littlefs";
#define DATA_FILE      "/littlefs/data.bin"
#define META_FILE      "/littlefs/meta.bin"
#define RECORD_SIZE    sizeof(NVS_DATA)
#define MAX_RECORDS    (80000)   // adjust as per partition size
#define MAX_FILE_SIZE  (RECORD_SIZE * MAX_RECORDS)



//Global Variables
REG_EVENT_GROUP_STATUS reg_evet_grp_status = REG_EVENT_GRP1;
EVENTGROUP_STATUS event_grp_status = EVENT_GRP1;
DOSE_EVENT_GROUP_STATUS dose_event_grp_status = DOSE_EVENT_GP1;
uint32_t system_op_error_code;
char mac_str[20];
volatile uint8_t counter_start_f = 0;
static uint8_t data_Buffer[30];	
esp_err_t ret;					
esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;
NVS_DATA nvs_data;
nvs_handle_t my_handle;
BaseType_t xReturned;
TaskHandle_t DosingHandler = NULL;
TaskHandle_t reset_task_handler = NULL;
TaskHandle_t RegenerationHandler = NULL;
TaskHandle_t ChlorinationHandler = NULL;
TaskHandle_t NVS_data_saveHandler = NULL;
static volatile uint8_t System_status_f = SYS_OK;
static uint8_t prev_chlorine_status_f = SYS_UNDEF; 
static volatile	uint8_t chlorine_status_f = SYS_NOK;
static volatile	uint8_t regeneration_status_f = SYS_NOK;
static volatile	uint8_t dosing_status_f = SYS_NOK;
static volatile uint8_t System_mode = OPERATING_MODE;
static volatile uint8_t Prev_System_mode = IDLE_MODE;
EventGroupHandle_t sensor_event_group;
TimerHandle_t v1min_timer;
TimerHandle_t v1sec_timer;
TimerHandle_t v1sec_timer2;
const TickType_t xDelay = (10000 / portTICK_PERIOD_MS); // 10 SEC delay
const TickType_t CST_FILL_DELAY = (60000 / portTICK_PERIOD_MS); // 60 SEC delay
const TickType_t COMPLETE_OPERTAION_DELAY = (30000 / portTICK_PERIOD_MS); // 30 SEC delay
const uart_port_t uart_num1 = UART_NUM_2; // Communication to a IOT Reader 
#if RS485_MODBUS_COMM_REQ == 1
const uart_port_t uart_num = UART_NUM_1; // Communication to a RS485 Module 
static uint8_t MODBUS_READ_VOL_CURR_CMD[] = {0x01 , 0x03 , 0x10 ,0x01 , 0x00 , 0x02 , 0x91 , 0x0B};// {Slave Address + Function Code (0x03 : Read the Register) , Register Address (0x0000) , Number of Bytes to Read (2 byte : 0x0002) , CRC Value 2 Byte : (0xC40B)}  
static uint8_t ModBus_response[10]; // Mod-bus Data Response Buffer
TaskHandle_t MODBUS_task_handler = NULL;
static void v_Modbus_Read_task(void *pvParameters);
#endif


// Electrode Voltage and Current 
static volatile float Electrode_voltage , Electrode_current;
static volatile float temperature_value1 , temperature_value2;
// Flow Meter Value Dosing Value in %
static volatile float Flow_Rate , Dosing_speed;
// Debug Variables / Function
time_t now ;
struct tm current_time;
static void Print_Operation_status(void);

// ADC variables and  Functions
adc_oneshot_unit_handle_t adc1_handle;
#if RS485_MODBUS_COMM_REQ == 0
adc_oneshot_unit_handle_t adc2_handle;
static void read_electrode_parameter(void);
#endif
static void reset_CRT_counter(void *arg1, uint32_t arg2);

// Output Pin status Variable
output_status_t output_status;

// Function Declarations
static uint32_t pwm_from_percent(float percent);
static uint32_t load_write_index(void);
static void GPIOs_init(void);
static void save_write_index(uint32_t index);
static void save_log_data(NVS_DATA *data);
static bool read_latest_log(NVS_DATA *data);
//static void wifi_init(void);
//static void Wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);
//static void esp32_sntp_init(void);
static void read_nvs_data(void);
static void v_dosing_task(void * pvParameters);
static void v_reset_button_task(void * pvParameters);
static void v_regeneration_task(void * pvParameters);
static void v_chlorination_task(void * pvParameters);
static void v_NVS_data_save_task(void *pvParameters);
static void v1sec_callback(TimerHandle_t xTimer);
static void v1sec_callback2(TimerHandle_t xTimer);
static void v15sec_callback(TimerHandle_t xTimer);
static void v1min_callback(TimerHandle_t xTimer);
static void system_check(void);
static void read_System_Mode(void);
static void GPIOs_Status_check(void);
static void Salt_dosing_operation(void);
static void Uart_init(void);
static void data_transfer(void);
static void get_esp32_mac_id(uint8_t *mac_id);
static void send_status_data(void);
static void send_MAC_ID_data(void);
static void pwm_set_duty(uint32_t duty);
#if POLARITY_TYPE == TIME_TYPE
static void polarity_controller(void);
#endif
static void Get_Flow_rate(void);
static void monitor_task_stack(void);
static uint8_t time_sync_done_f = 0;

portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
// Comment for the future Functions
// 1. We need to create some timer threshold after which if the system is in operation mode and OK then we need to reset whole system
// By making all the counter value to zero and restart all the task


// External Interrupt ISR handler 
// Reset Button Interrupt ISR 
static void IRAM_ATTR reset_button_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Notify task that button was pressed
    if (reset_task_handler != NULL)
    {
	    xTaskNotifyFromISR(reset_task_handler, 1,
	                       eSetBits, &xHigherPriorityTaskWoken);
    }   
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  
}



// Main Program
void app_main(void)
{
	sensor_event_group = xEventGroupCreate();
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE("NVS","Storage is not successfully initialized");
    }
    //read_nvs_data();// Read Last Store NVS Data

	esp_vfs_littlefs_conf_t conf = {
	.base_path = "/littlefs",
	.partition_label = "storage",
	.format_if_mount_failed = true,
	.dont_mount = false,
};
     esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
        esp_littlefs_format(conf.partition_label);
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    if(read_latest_log(&nvs_data) != true)
	{
		memset(&nvs_data, 0, sizeof(nvs_data)); // if we are unable to read the Little FS then we reset the structure of NVS data 
	}
    if(nvs_data.tank_filling_counter >= MAX_CRT_FILLING_TIME) // For safety 
    {
		nvs_data.tank_filling_counter = 0;
	}
    if(nvs_data.tank_draining_counter >= MAX_CST_FILLING_TIME) // For safety 
    {
		nvs_data.tank_draining_counter = 0;
	}

    // Initialize GPIOs
    GPIOs_init();
    ESP_LOGI("All GPIOs","initialized successfully");

    // Initialize UART
	Uart_init();


#if RS485_MODBUS_COMM_REQ == 0
	// ADC INIT (One - Shot Mode)
	adc_oneshot_unit_init_cfg_t init_config2 = {
	    .unit_id = ADC_UNIT_2,
	    .ulp_mode = ADC_ULP_MODE_DISABLE, // Low Power Mode Disable
	};	
	ret = adc_oneshot_new_unit(&init_config2, &adc2_handle);
	if(ret != ESP_OK){
		ESP_LOGE("ONE SHOT" ,"MODE ADC NOT INIT");
	}

	adc_oneshot_chan_cfg_t config2 = {
	    .bitwidth = ADC_BITWIDTH_12, // Default Max is added 
	    .atten = ADC_ATTEN_DB_12, // 12 level Attenuation is added to increase the range of the ADC value calculation
	};	
	ret = adc_oneshot_config_channel(adc2_handle,ADC_CHANNEL_6,&config2); // Voltage Sensor ADC2_6 We take ADC2 Channel 6 For Voltage Sensor
	if(ret != ESP_OK){
		ESP_LOGE("ADC2_6","not able to initialize");
	}
	ret = adc_oneshot_config_channel(adc2_handle,ADC_CHANNEL_7,&config2); // Current Sensor ADC2_7 , We take ADC2 channel 7 for Current Sensor 
	if(ret != ESP_OK){
		ESP_LOGE("ADC2_6","not able to initialize");
	}
#endif		  	 	
   // Timer Creation
   TimerHandle_t v15sec_timer = xTimerCreate("Fifteen Second Timer", pdMS_TO_TICKS(15000), pdTRUE, NULL, v15sec_callback);
   if (v15sec_timer != NULL) {
    xTimerStart(v15sec_timer, 0);  // Start the 15 SEC timer
	}
    v1min_timer = xTimerCreate("One Minute Timer", pdMS_TO_TICKS(60000), pdTRUE, NULL, v1min_callback);  
    v1sec_timer = xTimerCreate("One Second Timer",pdMS_TO_TICKS(1000) , pdTRUE , NULL ,v1sec_callback); 
    v1sec_timer2 = xTimerCreate("One Second Timer 1",pdMS_TO_TICKS(1000) , pdTRUE , NULL ,v1sec_callback2); 
    	        	    
   
   // Task CREATION
   xTaskCreate(v_dosing_task,"Dosing Task",4096,NULL,5,&DosingHandler);
   xTaskCreate(v_regeneration_task,"Regeneration Task",4096,NULL,6,&RegenerationHandler); 
   xTaskCreate(v_chlorination_task,"Chlorination Task",4096,NULL,5,&ChlorinationHandler); 
   xTaskCreate(v_NVS_data_save_task,"NVS data Saving Task",4096,NULL,3,&NVS_data_saveHandler);
   xTaskCreate(v_reset_button_task, "reset_button_task", 4096, NULL, 7, &reset_task_handler);   // High Priority task   
   
#if RS485_MODBUS_COMM_REQ == 1
   xTaskCreate(v_Modbus_Read_task,"Mod-bus Task",4096,NULL,2,&MODBUS_task_handler); // Very Low Priority Task
#endif
   
   ESP_LOGI("TASK:","Created");
	// Main loops
    while (1) 
    {
		GPIOs_Status_check();// Read GPIO Pins Status
		read_System_Mode(); // Read the System Mode
	    system_check(); // Check the system Status (OK/NOK)
	    
	    #if POLARITY_TYPE == TIME_TYPE
	    polarity_controller(); // Electrode Polarity Controller
	    #endif
	    
	    #if RS485_MODBUS_COMM_REQ == 0
	    read_electrode_parameter(); // Read Electrode Voltage and Current by a Sensor
	    #endif
	    
	    data_transfer(); // getting the UART data and according to that Electro-Chlorinator Operate
	    Print_Operation_status(); // Debug Purpose 	
	  //monitor_task_stack(); // monitor stack size in every 1 SEC
		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// GPIO Initialization
static void GPIOs_init(void)
{
    // Input pins with Pull down configuration
    gpio_config_t i_conf = {
        .pin_bit_mask = ((1ULL<<TOP_LEVEL_CRT_SENSOR_GPIO) |
                         (1ULL<<TOP_LEVEL_CST_SENSOR_GPIO)|                         
                         (1ULL<<REGENERATION_SIGNAL_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&i_conf);
    
    
    // Input pins with Pull Up configuration  
    gpio_config_t ih_conf = {
        .pin_bit_mask = ((1ULL<<BOTT_LEVEL_CRT_SENSOR_GPIO) |
                         (1ULL<<BOTT_LEVEL_CST_SENSOR_GPIO) |
                         (1ULL<<FLOATY_SENSOR_GPIO) |
                         (1ULL<<OPERATING_MODE_SWITCH)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&ih_conf);    
 
    // Output pins configuration
    
    gpio_config_t o_conf = {
        .pin_bit_mask = ((1ULL<<PUMP_P1_RELAY_GPIO) |
						  (1ULL<<DOSING_PUMP_D2_GPIO)|
                         (1ULL<<SALT_DOSING_PUMP_D1_GPIO) |
                         (1ULL<<VALVE_V1_RELAY_GPIO) |
                         (1ULL<<VALVE_V2_RELAY_GPIO) |
                         (1ULL<<SSR_RELAY_GPIO) |
                         (1uLL<<ELECTRODE_POLARITY_GPIO)|
                         (1ULL << LED1_GPIO)|
                         (1ULL << LED2_GPIO)|
                         (1ULL << LED3_GPIO)|
                         (1ULL << LED4_GPIO)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&o_conf);
    
    
    // Reset Button Configuration
    gpio_config_t i_itrconf = {
        .pin_bit_mask = ((1ULL<<RESET_BUTTON_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE	
	};
    gpio_config(&i_itrconf);
    // Install ISR service
    gpio_install_isr_service(0);
    // Attach ISR handler
    gpio_isr_handler_add(RESET_BUTTON_GPIO, reset_button_isr_handler,NULL); // No argument to pass to the ISR     
    
}

/*static void wifi_init(void)
{

	ret = esp_netif_init();
	if(ret != ESP_OK )
	{
		ESP_LOGE("ESP","TCP/IP Stack Fail");
	}
	esp_event_loop_create_default();
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &Wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &Wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));	
                                                        
     wifi_config_t wifi_config ={
        .sta = {
            .ssid = "Airtel_adit_6830", // "Airtel_adit_6830"
            .password = "air48666", // air48666
        },		
		 
	 };                                                
	esp_wifi_set_mode(WIFI_MODE_STA); // Station Mode
	esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
	esp_wifi_start();
	ESP_LOGI("WI-FI","Started");
}*/

/*static void Wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI("WIFI","Trying to connect...");        
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI("WIFI","Connected to AP successfully!");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // Time Sync is Done using the SNTP Server 
        esp32_sntp_init(); // initialized SNTP Server for time sync
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW("WIFI", "Disconnected! Reconnecting...");
        esp_wifi_connect();
    }
}*/

/*static void esp32_sntp_init(void)
{
	esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
	esp_netif_sntp_init(&sntp_config);
	// wait for first sync
	if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(15000)) != ESP_OK) {
	    ESP_LOGI("SNTP:","sync failed");
	    return;
	}
	esp_netif_sntp_start();  // start periodic updates	
	ESP_LOGI("SNTP","Time Sync Done !!");
	// Set your timezone here (AFTER sync)
	setenv("TZ", "IST-5:30", 1);  
	tzset();
	time(&now);
	localtime_r(&now, &current_time);
}*/

static void read_nvs_data(void)
{
    esp_err_t ret;
    nvs_handle_t my_handle;

    ret = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open storage (%s)", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI("NVS", "Reading parameters");

    size_t required_size = sizeof(NVS_DATA);
    ret = nvs_get_blob(my_handle, "my_key", &nvs_data, &required_size);

    if (ret == ESP_ERR_NVS_NOT_FOUND || (ret == ESP_OK && required_size != sizeof(NVS_DATA))) {
        ESP_LOGW("NVS", "NVS data not found or size mismatch, initializing defaults");
        memset(&nvs_data, 0, sizeof(NVS_DATA));

        ret = nvs_set_blob(my_handle, "my_key", &nvs_data, sizeof(NVS_DATA));
        if (ret != ESP_OK) {
            ESP_LOGE("NVS", "Failed to set default blob (%s)", esp_err_to_name(ret));
        } else {
            ret = nvs_commit(my_handle);
            if (ret != ESP_OK) {
                ESP_LOGE("NVS", "Commit failed (%s)", esp_err_to_name(ret));
            }
        }
    } else if (ret != ESP_OK) {
        ESP_LOGE("NVS", "Failed to read blob (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI("NVS", "Data read successfully");
    }

    nvs_close(my_handle);
}




void v_reset_button_task(void *arg)
{
    uint32_t notifyValue;
    while (1)
    {
        xTaskNotifyWait(0, ULONG_MAX, &notifyValue, portMAX_DELAY);

        // Debounce 20–30 ms
        vTaskDelay(pdMS_TO_TICKS(30));

        // Check if button is still pressed (active LOW)
        if (gpio_get_level(RESET_BUTTON_GPIO) == HIGH)
        {
            int elapsed = 0;

            // Keep checking how long button stays pressed
            while (gpio_get_level(RESET_BUTTON_GPIO) == HIGH)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
                elapsed += 100;

                // If held long enough → factory reset
                if (elapsed >= RESET_HOLD_TIME_MS)
                {
                    ESP_LOGI("RESET","BUTTON PRESS");
                    // Erase flash safely
                    //nvs_flash_erase();
					memset(&nvs_data, 0, sizeof(nvs_data));
					save_log_data(&nvs_data);
                    vTaskDelay(pdMS_TO_TICKS(100)); 
                    esp_restart();
                }
            }
        }
    }
}

static void v_dosing_task(void * pvParameters)
{
	while(1)
	{
		if(gpio_get_level(OPERATING_MODE_SWITCH) == true)
		{

			switch(dose_event_grp_status)
	       {

			    case(DOSE_EVENT_GP1):  
					//Dosing Task Working 
					EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
											CST_BOTTOM_SENSOR_BIT | DOSING_TIME_BIT,
											pdTRUE,   
											pdTRUE,   // wait for both bit
											portMAX_DELAY);
											
					if(((bits & CST_BOTTOM_SENSOR_BIT) == CST_BOTTOM_SENSOR_BIT) && ((bits & DOSING_TIME_BIT) == DOSING_TIME_BIT))
					{
										ESP_LOGI("Chlorine Dosing","Started");
										gpio_set_level(DOSING_PUMP_D2_GPIO,1);
										LED2_ON();
										output_status.Output_level.pump3 = HIGH;
										nvs_data.operating_mode |= DOSING_OPERATION;	
										dose_event_grp_status = DOSE_EVENT_GP2;
					}

				break;
							
				case(DOSE_EVENT_GP2):
						EventBits_t bits1 = xEventGroupWaitBits(sensor_event_group,
												DOSING_STOP_TIME_BIT | CST_NO_BOTTOM_SENSOR_BIT,
												pdTRUE,   
												pdFALSE,   // wait for Any bit
												portMAX_DELAY);
												
						if(((bits1 & DOSING_STOP_TIME_BIT) == DOSING_STOP_TIME_BIT ))
						{
											ESP_LOGI("Chlorine Dosing","Stop");
											gpio_set_level(DOSING_PUMP_D2_GPIO,0);
											output_status.Output_level.pump3 = LOW;
											nvs_data.operating_mode &= ~(DOSING_OPERATION);
											dose_event_grp_status = DOSE_EVENT_GP1;	
						}
						else if (((bits1 & CST_NO_BOTTOM_SENSOR_BIT) == CST_NO_BOTTOM_SENSOR_BIT))
						{
											ESP_LOGI("Chlorine Dosing","Stop");
											gpio_set_level(DOSING_PUMP_D2_GPIO,0);
											output_status.Output_level.pump3 = LOW;
											dose_event_grp_status = DOSE_EVENT_GP1;								
						}
					break;
					
					
				default:
				  ESP_LOGI("Dosing","No Event Group is Executed");
				break;	
		   }		
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
 }
 
static void v_regeneration_task(void * pvParameters)
{
  while(1)
   {
	   	   
#if REGENERATION_SUSPEND_RESUME_REQ == 1
	  if((gpio_get_level(OPERATING_MODE_SWITCH) == true))
#endif
	    {	
			
		    switch (reg_evet_grp_status)
		     {
			
				case REG_EVENT_GRP1:    			    //Regeneration Task Working 
				EventBits_t bits2 = xEventGroupWaitBits(sensor_event_group,
	                                       FLOATY_SENSOR_BIT|REG_RELAY_SENSOR_BIT,
	                                       pdTRUE,   
	                                       pdTRUE,   // wait for all bit
	                                       portMAX_DELAY);
	                                                                                                     
		         if (((bits2 & REG_RELAY_SENSOR_BIT) == REG_RELAY_SENSOR_BIT)&&((bits2 & FLOATY_SENSOR_BIT) == FLOATY_SENSOR_BIT))
		         {				 				 
					 ESP_LOGI("Regeneration","Process Started");
					 ESP_LOGI("Pump P1:","OFF");
					 gpio_set_level(PUMP_P1_RELAY_GPIO,0);
					 output_status.Output_level.pump1 = LOW;
					 ESP_LOGI("Salt Dosing","Pump OFF");
					 gpio_set_level(SALT_DOSING_PUMP_D1_GPIO, 0); // OFF
					 output_status.Output_level.pump2 = LOW;
					 ESP_LOGI("Valve V1:","OFF");
					 gpio_set_level(VALVE_V1_RELAY_GPIO,0);	
					 output_status.Output_level.valve1 = LOW;			 			 
					 vTaskDelay(xDelay); // 10 SEC delay is provide to the valve because it take time to open and close 
					 ESP_LOGI("Pump P1:","Started");
					 gpio_set_level(PUMP_P1_RELAY_GPIO,1);
					 output_status.Output_level.pump1 = HIGH;
					 ESP_LOGI("RED:","LED OFF");
					 nvs_data.operating_mode |= (REGENERATION_OPERATION);
					 reg_evet_grp_status = REG_EVENT_GRP2;				 
					 
				 }
				 break;
				 
				case REG_EVENT_GRP2: 
				EventBits_t bits3 = xEventGroupWaitBits(sensor_event_group,
	                                       REG_NO_RELAY_SENSOR_BIT | FLOATY_NO_SENSOR_BIT,
	                                       pdTRUE,   
	                                       pdFALSE,   // wait for any bit
	                                       portMAX_DELAY);
	                                       
		         if (((bits3 & REG_NO_RELAY_SENSOR_BIT) == REG_NO_RELAY_SENSOR_BIT) && ((nvs_data.operating_mode & REGENERATION_OPERATION) == REGENERATION_OPERATION))
		         {				 
					 ESP_LOGI("Regeneration","Process Stop");
					 ESP_LOGI("Pump P1:","OFF");
					 output_status.Output_level.pump1 = LOW;
					 gpio_set_level(PUMP_P1_RELAY_GPIO,0);			 
					 nvs_data.operating_mode &= ~(REGENERATION_OPERATION);
					 reg_evet_grp_status = REG_EVENT_GRP1;
				 }
				 if(((bits3 & FLOATY_NO_SENSOR_BIT) == FLOATY_NO_SENSOR_BIT) && ((nvs_data.operating_mode & REGENERATION_OPERATION) == REGENERATION_OPERATION)) // If FLOATY is not present and Regeneration Signal Present
				 {					 			 
				     ESP_LOGI("Regeneration","Process Stop No Raw Water Present");
					 ESP_LOGI("Pump P1:","OFF");
					 gpio_set_level(PUMP_P1_RELAY_GPIO,0);
					 output_status.Output_level.pump1 = LOW;	
					 reg_evet_grp_status = REG_EVENT_GRP1;		 				 
				 }
				break;
				 
				default:
				ESP_LOGI("REGENERATION","DEFAULT CASE EXECUTED");
				break;
		    }
			 	
		 } 
		  vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

static void v_chlorination_task(void * pvParameters)
{
	while(1)
	{	
	  if((gpio_get_level(OPERATING_MODE_SWITCH) == true)) // Operating Mode - Operating should be present
	  {
		// CRT FILLING OPERATION 
		
		switch(event_grp_status)
		{
		
		
		
		  case(EVENT_GRP1):
			if((nvs_data.operating_mode & CRT_FILLING_START_OPERATION) == 0 && (gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO) == true))
			{
				EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
	                                       FLOATY_SENSOR_BIT | CRT_NO_TOP_SENSOR_BIT|CST_NO_TOP_SENSOR_BIT | REG_NO_RELAY_SENSOR_BIT|CRT_FILLING_TIMER_NO_ERROR_BIT,
	                                       pdTRUE,   
	                                       pdTRUE,   // wait for all bit
	                                       portMAX_DELAY);
	                                       
		         if (((bits & FLOATY_SENSOR_BIT) == FLOATY_SENSOR_BIT) && ((bits & CRT_NO_TOP_SENSOR_BIT) == CRT_NO_TOP_SENSOR_BIT)
		         && ((bits & CST_NO_TOP_SENSOR_BIT) == CST_NO_TOP_SENSOR_BIT) && ((bits & CRT_FILLING_TIMER_NO_ERROR_BIT) == CRT_FILLING_TIMER_NO_ERROR_BIT)
		         && ((bits & REG_NO_RELAY_SENSOR_BIT) == REG_NO_RELAY_SENSOR_BIT))
		         {
					 LED1_ON();
					 nvs_data.Current_operating_status_f = CRT_FILLING_START_OPERATION;						 			 
					 ESP_LOGI("CRT:","Filling Started");
					 ESP_LOGI("Valve V1:"," Started");
					 gpio_set_level(VALVE_V1_RELAY_GPIO,1);
					 output_status.Output_level.valve1 = HIGH;
					 vTaskDelay(xDelay); // 10 SEC delay is provide to the valve because it take time to open and close 			 
					 ESP_LOGI("PUMP P1 :","Started");
					 gpio_set_level(PUMP_P1_RELAY_GPIO,1);
					 if (v1sec_timer != NULL) {
					   	 xTimerStart(v1sec_timer, 0);  // Start the 1 SEC timer
					 }					 
					 output_status.Output_level.pump1 = HIGH;			 
					 Salt_dosing_operation(); // Salt Dosing OPeration taken place 
					 event_grp_status = EVENT_GRP2;
				 }
			}
			else
			{
				event_grp_status = EVENT_GRP2;
			} 
			break;
			
			
			
			
			
		 case(EVENT_GRP2):	
			if((nvs_data.operating_mode & CRT_FILLING_START_OPERATION) == 0 && (gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO) == true))			
			{ 	 
				 // We add the Event Group Because We want to start the Filling timer of the CRT tank Only When then Bottom CRT tank is Triggered
	 			EventBits_t bit = xEventGroupWaitBits(sensor_event_group,
	                           CRT_BOTTOM_SENSOR_BIT| CRT_TOP_SENSOR_BIT | CRT_FILLING_TIMER_BIT,
	                           pdTRUE,   
	                           pdFALSE,   // wait for any bit
	                           portMAX_DELAY);
	              
	             if((bit & CRT_BOTTOM_SENSOR_BIT) == CRT_BOTTOM_SENSOR_BIT)  // When CRT bottom is hit then 1 SEC timer is start
	             {
					event_grp_status = EVENT_GRP3; 
					if(nvs_data.crt_reset_counter_done_f !=1)
					{
					  xTimerPendFunctionCall(reset_CRT_counter, NULL, 0,portMAX_DELAY); // Reset the CRT Filling Counter To Start Again
					  nvs_data.crt_reset_counter_done_f = 1;
					}
				 }
				 else if((bit & CRT_TOP_SENSOR_BIT) == CRT_TOP_SENSOR_BIT)
				 {
					 // Some Error is there in the CRT Bottom Tank Sensor // Error Need to send to the IOT Reader Device that something is Wrong // Need to implement in the Future				 
					 event_grp_status = EVENT_GRP3;
					 ESP_LOGI("Bottom LEVEL CRT","TANK ISSUE");				 
				 }
				 else if((bit & CRT_FILLING_TIMER_BIT) == CRT_FILLING_TIMER_BIT)
				 {
					  event_grp_status = EVENT_GRP3;
					  ESP_LOGI("Both Top and Bottom LEVEL CRT","TANK ISSUE");	
				 } 	 			 	
			}
			else
			{
				event_grp_status = EVENT_GRP3;
			} 
			break; 
			 
			
			
			
			 
		
		 case(EVENT_GRP3):	  
			 if(((nvs_data.operating_mode & CRT_FILLING_STOP_OPERATION) == 0))
			{
				EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
	                                       CRT_TOP_SENSOR_BIT | CRT_FILLING_TIMER_BIT,
	                                       pdTRUE,    
	                                       pdFALSE,   // wait for any bit
	                                       portMAX_DELAY);
	                                       
		         if (((bits & CRT_TOP_SENSOR_BIT) == CRT_TOP_SENSOR_BIT))
		         {	
					 LED1_ON();			 
					 nvs_data.Current_operating_status_f =  CRT_FILLING_STOP_OPERATION;		
		            if (v1sec_timer != NULL) {  
	    				xTimerStop(v1sec_timer, 0);  // stop the 1 SEC timer
					  }				 		 
					 ESP_LOGI("CRT:","Filling Complete");
					 ESP_LOGI("PUMP P1 :","OFF");
					 gpio_set_level(PUMP_P1_RELAY_GPIO,0);
					 output_status.Output_level.pump1 = LOW;
					 ESP_LOGI("Dosing Pump D1:"," OFF");
					 gpio_set_level(SALT_DOSING_PUMP_D1_GPIO,0);
					 output_status.Output_level.pump2 = LOW;
					 ESP_LOGI("Valve V1:"," OFF");
					 gpio_set_level(VALVE_V1_RELAY_GPIO,0);
					 output_status.Output_level.valve1 = LOW;
					 vTaskDelay(xDelay); // 10 SEC delay is provide to the valve because it take time to open and close 
					 ESP_LOGI("BLUE LED :","OFF");
					 nvs_data.operating_mode |= (CRT_FILLING_START_OPERATION |CRT_FILLING_STOP_OPERATION);	
					 event_grp_status = EVENT_GRP4;		  
				 }else if((bits & CRT_FILLING_TIMER_BIT) == CRT_FILLING_TIMER_BIT) //If event group trigger due to the overflow of the time then it execute this 
				 {
		            if (v1sec_timer != NULL) {  
	    				xTimerStop(v1sec_timer, 0);  // stop the 1 SEC timer
					  }
					 ESP_LOGI("TOP LEVEL CRT","TANK ISSUE"); 				 
					 ESP_LOGI("CRT:","Filling time Exceeded");
					 ESP_LOGI("PUMP P1 :","OFF");
					 LED1_OFF();	
					 nvs_data.Current_operating_status_f = CRT_FILLING_START_OPERATION;	
					 nvs_data.crt_filling_timer_error_f = 1;
					 gpio_set_level(PUMP_P1_RELAY_GPIO,0);
					 output_status.Output_level.pump1 = LOW;
					 ESP_LOGI("Dosing Pump D1:"," OFF");
					 gpio_set_level(SALT_DOSING_PUMP_D1_GPIO,0);
					 output_status.Output_level.pump2 = LOW;
					 ESP_LOGI("Valve V1:"," OFF");
					 gpio_set_level(VALVE_V1_RELAY_GPIO,0);
					 output_status.Output_level.valve1 = LOW;
					 vTaskDelay(xDelay); // 10 SEC delay is provide to the valve because it take time to open and close 
					 event_grp_status = EVENT_GRP4;			 
				 }			 
			 }
			 else
			 {
				 event_grp_status = EVENT_GRP4;
			 }
			 break;	
			 
			
			
			
			 
			 
		 case(EVENT_GRP4): 	 
			 if(gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO) == false && (nvs_data.electrolysis_done_f == false && (nvs_data.dosing_done_f == true) //NEED TO DISCUSS - Because Without salt there should not be Electrolysis Process started 
			     &&  ((nvs_data.operating_mode & ELECTROLYSIS_START_OPERATION) == 0 )) && (nvs_data.crt_filling_timer_error_f == 0))
			   {
				EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
	                                       CRT_TOP_SENSOR_BIT|CRT_BOTTOM_SENSOR_BIT,
	                                       pdTRUE,    
	                                       pdTRUE,   // wait for all bit
	                                       portMAX_DELAY);	
	               if(((bits & CRT_TOP_SENSOR_BIT) == CRT_TOP_SENSOR_BIT)  && ((bits & CRT_BOTTOM_SENSOR_BIT) == CRT_BOTTOM_SENSOR_BIT)) 
	               { 
					  LED1_ON();
					  nvs_data.Current_operating_status_f = ELECTROLYSIS_START_OPERATION; 					                                                          
		              ESP_LOGI("SSR:","RELAY ON");
		              output_status.Output_level.ssr = HIGH;
		              
		              #if POLARITY_TYPE == BATCH_TYPE || POLARITY_TYPE == TIME_TYPE
		                  uint8_t temp_polarity = ((nvs_data.last_polarity_status_f == 0) ? 1 : 0);  
		                  output_status.Output_level.polarity = temp_polarity;
		              #endif    
		              	                    	               
		              gpio_set_level(SSR_RELAY_GPIO,0);	// SSR OFF
		              gpio_set_level(ELECTRODE_POLARITY_GPIO,temp_polarity); // LOW  -VE Polarity	              
		              gpio_set_level(SSR_RELAY_GPIO,1); // SSR ON 
		              ESP_LOGI("ELECTROLYSIS:","Started");	              
		                 if (v1min_timer != NULL) {
	    				xTimerStart(v1min_timer, 0);  // Start the 1 Minute timer
					   }
					  event_grp_status = EVENT_GRP5; 
	              }		   			   
			   }
			   else
			   {
				   event_grp_status = EVENT_GRP5;
			   }
			   break;
			   
			   
			   
			
			
			 
			case(EVENT_GRP5):   
			   if(gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO) == false &&  ((nvs_data.operating_mode & ELECTROLYSIS_STOP_OPERATION) == 0 )
			   		&& (nvs_data.electrolysis_done_f == false))
			   {
				  			EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
	                                       ELECTROLYSIS_DONE_BIT,
	                                       pdTRUE,    
	                                       pdTRUE,   // wait for all bit
	                                       portMAX_DELAY);
	                                       
	               if( (bits & ELECTROLYSIS_DONE_BIT) == ELECTROLYSIS_DONE_BIT ) 
	               {   
					  LED1_ON();
		              nvs_data.Current_operating_status_f =  ELECTROLYSIS_STOP_OPERATION;				                      
		              ESP_LOGI("SSR:","RELAY OFF");
		              output_status.Output_level.ssr = LOW;
		              gpio_set_level(SSR_RELAY_GPIO,0); // SSR OFF
		              
		              #if POLARITY_TYPE == BATCH_TYPE 
		                  nvs_data.last_polarity_status_f = ((nvs_data.last_polarity_status_f == 0) ? 1 : 0);
		              #endif 
		            
		              ESP_LOGI("ELECTROLYSIS:","STOP");                	               
		              if (v1min_timer != NULL) {  
	    				xTimerStop(v1min_timer, 0);  // Start the 1 Minute timer
					   }
		              nvs_data.operating_mode |= (ELECTROLYSIS_START_OPERATION | ELECTROLYSIS_STOP_OPERATION);	
		              event_grp_status = EVENT_GRP6;	              				    
	              }                        
			   }
			   else
			   {
				   event_grp_status = EVENT_GRP6;
			   }
			   break;
			   
			   
			 
			 
			   
			case(EVENT_GRP6):   
			   if(nvs_data.electrolysis_done_f == true && ((nvs_data.operating_mode & CST_FILLING_START_OPERATION) == 0 ))
			   {
				   EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
	                                       DOSING_OFF_BIT|CST_NO_TOP_SENSOR_BIT|CST_FILLING_TIMER_NO_ERROR_BIT,
	                                       pdTRUE,    
	                                       pdTRUE,   // wait for all bit
	                                       portMAX_DELAY);
	                  if(((bits & DOSING_OFF_BIT) == DOSING_OFF_BIT)  && ((bits & CST_NO_TOP_SENSOR_BIT) == CST_NO_TOP_SENSOR_BIT)
	                    && ((bits & CST_FILLING_TIMER_NO_ERROR_BIT) == CST_FILLING_TIMER_NO_ERROR_BIT)) 
	                  {						 
						  LED1_ON();
						  nvs_data.Current_operating_status_f = CST_FILLING_START_OPERATION;					  
						  ESP_LOGI("VALVE V2:","ON");
						  output_status.Output_level.valve2 = HIGH;
						  gpio_set_level(VALVE_V2_RELAY_GPIO,1);
						  ESP_LOGI("CST:","FILLING STARTED");
						  if (v1sec_timer2 != NULL) {
	    				   xTimerStart(v1sec_timer2, 0);  // Start the 1 second timer
					     }
					      event_grp_status = EVENT_GRP7;
					  }                                                                                               
			   }
			   else
			   {
				   event_grp_status = EVENT_GRP7;
			   }
			   break;
			 
			 
			 
			   
			   
			case(EVENT_GRP7):   
			   if(nvs_data.electrolysis_done_f == true && ((nvs_data.operating_mode & CST_FILLING_STOP_OPERTAION) == 0 ))
			   {
				  			EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
	                                       CRT_NOT_BOTTOM_SENSOR_BIT|CST_FILLING_TIMER_BIT,
	                                       pdTRUE,    
	                                       pdFALSE,   // wait for any bit
	                                       portMAX_DELAY);
	                  if(((bits & CRT_NOT_BOTTOM_SENSOR_BIT) == CRT_NOT_BOTTOM_SENSOR_BIT)) 
	                  {						  					  				  
						  vTaskDelay(CST_FILL_DELAY); // 15 SEC delay is provide to the valve because it take time to open and close  
						  ESP_LOGI("VALVE V2:","OFF");
						  LED1_ON();
						  if (v1sec_timer2 != NULL) {  
	    					xTimerStop(v1sec_timer2, 0);  // Start the 1 second timer
					  	 }
						  output_status.Output_level.valve2 = LOW;
						  gpio_set_level(VALVE_V2_RELAY_GPIO,0);
						  ESP_LOGI("CST:","FILLING STOP");			   				  
						  nvs_data.Current_operating_status_f =0;  
						  nvs_data.crt_filling_timer_error_f = 0;
						  nvs_data.cst_filling_timer_error_f = 0;
						  nvs_data.tank_draining_counter = 0;
						  nvs_data.electrolysis_done_f = 0;    
						  nvs_data.dosing_counter = 0;
						  nvs_data.timer_counter = 0;
						  nvs_data.crt_reset_counter_done_f = 0;
						  nvs_data.tank_filling_counter = 0;
						  nvs_data.dosing_done_f = 0;  
						  nvs_data.operating_mode &= (REGENERATION_OPERATION | DOSING_OPERATION);//I Need to discuss that after complete of one Batch should go with Reset all timers or Not.
						  #if POLARITY_TYPE == TIME_TYPE
						  nvs_data.switching_polarity = 0;
						  #endif
						  event_grp_status = EVENT_GRP1;				  
						  vTaskDelay(COMPLETE_OPERTAION_DELAY); // 30 SEC delay For the Next Operation to Execute					  
					  }
					  else if((bits & CST_FILLING_TIMER_BIT) == CST_FILLING_TIMER_BIT)
					  {
						  if (v1sec_timer2 != NULL) {  
	    					xTimerStop(v1sec_timer2, 0);  // Start the 1 second timer
					  	 }
						  ESP_LOGI("VALVE V2:","OFF");
						  nvs_data.Current_operating_status_f = CST_FILLING_START_OPERATION;
						  output_status.Output_level.valve2 = LOW;
						  gpio_set_level(VALVE_V2_RELAY_GPIO,0);
						  nvs_data.cst_filling_timer_error_f = 1;
						  event_grp_status = EVENT_GRP1;
					  }                                                                                     
			   }
			   else
			   {
				   event_grp_status = EVENT_GRP1;
			   }
			   break;
			   
			   
			   
		   
		   
			  default:
			  ESP_LOGI("DEFAULT" ,"EVENT GROUP IS NOT EXECUTED");
			  break; 
			  
		 }
		
		
	  }		   	   		   		 		 		 		 		 
		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void v_NVS_data_save_task(void *pvParameters)
{
    static NVS_DATA last_save_NVS_data = {0};
    while (1) 
    {
        // Wait until notified
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI("NVS","Save task triggered");

        // Take a snapshot safely
        NVS_DATA snapshot;
        snapshot = nvs_data;

        // Check if anything changed
        if (memcmp(&snapshot, &last_save_NVS_data, sizeof(NVS_DATA)) != 0) 
        {
            ESP_LOGI("NVS","Change detected, saving to NVS");
			save_log_data(&nvs_data);
			last_save_NVS_data = snapshot;
        }
        else
        {
            ESP_LOGI("NVS","No changes detected, skipping save");
        }


		if(time_sync_done_f == 1) // Time sync done 
		{
			time(&now);
			localtime_r(&now, &current_time);
		}
    }
}


static void v1sec_callback(TimerHandle_t xTimer)
{
    nvs_data.tank_filling_counter++;
    if (nvs_data.tank_filling_counter >= MAX_CRT_FILLING_TIME)
    {
        xEventGroupSetBits(sensor_event_group, CRT_FILLING_TIMER_BIT);
    }
}

static void reset_CRT_counter(void *arg1, uint32_t arg2)
{
	nvs_data.tank_filling_counter = 0;
}


static void v1sec_callback2(TimerHandle_t xTimer)
{
    nvs_data.tank_draining_counter++;
    if (nvs_data.tank_draining_counter >= MAX_CST_FILLING_TIME)
    {
        xEventGroupSetBits(sensor_event_group, CST_FILLING_TIMER_BIT);
    }
}

static void v15sec_callback(TimerHandle_t xTimer)
{
    if (NVS_data_saveHandler != NULL)
        xTaskNotifyGive(NVS_data_saveHandler);
}
static void v1min_callback(TimerHandle_t xTimer)
{
	nvs_data.timer_counter++;		
    if (nvs_data.timer_counter >= ELECTROLYSIS_PROCESS_TIME)
    {
        nvs_data.electrolysis_done_f = true;
        xEventGroupSetBits(sensor_event_group, ELECTROLYSIS_DONE_BIT);
    }
}
void system_check(void)
{
	dosing_status_f = SYS_OK;
	regeneration_status_f = SYS_OK;
	chlorine_status_f = SYS_NOK;
	System_status_f = SYS_OK;
	eTaskState state;
	system_op_error_code = 0;
	printf("System Checking started\n");
	switch(nvs_data.Current_operating_status_f)
	{		
	// System Check When last Mode is CRT Mode
	case  CRT_FILLING_START_OPERATION:
	 	if(gpio_get_level(FLOATY_SENSOR_GPIO) == true && (gpio_get_level(REGENERATION_SIGNAL_GPIO) == true) && (nvs_data.crt_filling_timer_error_f !=1))
		{
			chlorine_status_f = SYS_OK;
		}
		else
		{
		  system_op_error_code |= CRT_FILL_ERROR;	
		}
	break;
	
	case CRT_FILLING_STOP_OPERATION:
		if((gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO) == false) && (gpio_get_level(BOTT_LEVEL_CRT_SENSOR_GPIO) == true) && (nvs_data.dosing_done_f == true))
		   {
			 chlorine_status_f = SYS_OK; 
		   }
		   	else
		   {
		      system_op_error_code |= CRT_STOP_ERROR;	
		   }		
	break;
	
	
	// System Check When last Mode is Electrolysis Mode
	case ELECTROLYSIS_START_OPERATION:
		if((gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO) == false) && (gpio_get_level(BOTT_LEVEL_CRT_SENSOR_GPIO) == true)
		   && (nvs_data.electrolysis_done_f == false)) // In Future we should add these condition - current should non zero and voltage should be below 6.
		{
			chlorine_status_f = SYS_OK;
		}
		else
		{
		  system_op_error_code |= ELECTROLYSIS_ERROR;	
		}
	break;
	
	
	 case ELECTROLYSIS_STOP_OPERATION:
			if((gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO) == false) && (gpio_get_level(BOTT_LEVEL_CRT_SENSOR_GPIO) == true)
			   && (nvs_data.electrolysis_done_f == true))
			{
				chlorine_status_f = SYS_OK;
			}
			else
		   {
		       system_op_error_code |= ELECTROLYSIS_STOP_ERROR;	
		   }
	 break;
	// System Check When last Mode is CST FILLING Mode	
	  case CST_FILLING_START_OPERATION:
	  		if((gpio_get_level(TOP_LEVEL_CST_SENSOR_GPIO) == true) && (nvs_data.cst_filling_timer_error_f != 1)
		       &&((nvs_data.operating_mode & DOSING_OPERATION) != DOSING_OPERATION))
	  		{
				 chlorine_status_f = SYS_OK; 
			}
			else
		    {
		        system_op_error_code |= CST_FILL_ERROR;	
		    }	  
	  break;	
			
	// Need to work on the Stand-By Mode System Status Condition 	
	default:
	chlorine_status_f = SYS_OK;
	break;
	}
	
	if((chlorine_status_f == SYS_NOK && prev_chlorine_status_f != chlorine_status_f)
	    ||(gpio_get_level(OPERATING_MODE_SWITCH) == STAND_BY_MODE && Prev_System_mode != System_mode)) // Chlorine Process is having the issue
	{
		if (v1min_timer != NULL && xTimerIsTimerActive(v1min_timer) != pdFALSE) // Stop the timer only when it is running 
		{  
			xTimerStop(v1min_timer, 0);  // Stop the 1 Minute timer
			ESP_LOGI("1min:","Timer Stop");
		}	
		if (v1sec_timer != NULL && xTimerIsTimerActive(v1sec_timer) != pdFALSE) // Stop the timer only when it is running 
		{  
			xTimerStop(v1sec_timer, 0);  // Stop the 1 SEC timer
			ESP_LOGI("1sec:","Timer Stop");
		}	
		if (v1sec_timer2 != NULL && xTimerIsTimerActive(v1sec_timer2) != pdFALSE) // Stop the timer only when it is running 
		{  
			xTimerStop(v1sec_timer2, 0);  // Stop the 1 SEC timer
			ESP_LOGI("1sec2:","Timer Stop");
		}					
		if(ChlorinationHandler != NULL) //If system is in Stand-by Mode then the chlorine task is suspended  
		{
			vTaskSuspend(ChlorinationHandler);
			if((gpio_get_level(REGENERATION_SIGNAL_GPIO) == true))
			{
				gpio_set_level(PUMP_P1_RELAY_GPIO,0);
				output_status.Output_level.pump1 = LOW;
			}
			gpio_set_level(SALT_DOSING_PUMP_D1_GPIO,0);			
			gpio_set_level(VALVE_V1_RELAY_GPIO,0);			
			gpio_set_level(SSR_RELAY_GPIO,0);
			gpio_set_level(VALVE_V2_RELAY_GPIO,0);	// To be discuss may be 10 SEC delay is required			
			output_status.Output_level.pump2 = LOW;
			output_status.Output_level.valve1 = LOW;
			output_status.Output_level.valve2 = LOW;
			output_status.Output_level.ssr = LOW;
			ESP_LOGI("Chlorine Task","Suspended");
			ESP_LOGI("Chlorine:","Component in Default state");							
		}			
		if((gpio_get_level(OPERATING_MODE_SWITCH) == STAND_BY_MODE && Prev_System_mode != System_mode))
		{			
			  if((DosingHandler != NULL))
			  {
				  vTaskSuspend(DosingHandler);
				  ESP_LOGI("Dosing:"," STOP");
				  ESP_LOGI("Dosing:","Task Suspended");
				  ESP_LOGI("Dosing PUMP P2 :","Stop");
				  gpio_set_level(DOSING_PUMP_D2_GPIO,0); 	
				  output_status.Output_level.pump3 = LOW;			  
			  }
#if REGENERATION_SUSPEND_RESUME_REQ == 1		
			  if(RegenerationHandler != NULL)
			  {
				 vTaskSuspend(RegenerationHandler);
				 ESP_LOGI("Regeneration:","Task Suspended");
				 ESP_LOGI("Regeneration","Process Stop");
				 ESP_LOGI("Pump P1:","OFF");
				 gpio_set_level(PUMP_P1_RELAY_GPIO,0);
				 output_status.Output_level.pump1 = LOW;
			  }	
#endif	
		}		
		prev_chlorine_status_f = chlorine_status_f;	
		Prev_System_mode = System_mode;		
	}										
	// System Check When last Mode is Dosing Mode
	if((nvs_data.operating_mode & DOSING_OPERATION) == DOSING_OPERATION)
	{
	   float temp_time = (float)(current_time.tm_hour + (current_time.tm_min/60.0f));	
	   if((gpio_get_level(BOTT_LEVEL_CST_SENSOR_GPIO) == false) || (temp_time < DOSING_START_TIME || temp_time > DOSING_END_TIME))
	   {
	 	   	nvs_data.operating_mode &= ~(DOSING_OPERATION);
	   }	
	}
    // System Check When last Mode is Regeneration Mode
	if((nvs_data.operating_mode & REGENERATION_OPERATION) == REGENERATION_OPERATION)
	{
		if((gpio_get_level(FLOATY_SENSOR_GPIO) == true) && ((gpio_get_level(REGENERATION_SIGNAL_GPIO) == false)))	  
		   {
			 regeneration_status_f = SYS_OK;
		   } else
		   {
			   regeneration_status_f = SYS_NOK;
			   system_op_error_code |= REGENERATION_ERROR;
		   } 	
	}
	
	if((chlorine_status_f == SYS_OK && prev_chlorine_status_f != chlorine_status_f && gpio_get_level(OPERATING_MODE_SWITCH) == OPERATING_MODE)
	   ||(gpio_get_level(OPERATING_MODE_SWITCH) == OPERATING_MODE && Prev_System_mode != System_mode)) // If operating Mode is change from Stand By to Operating Mode 
	{																								   // If the Chlorine Process move from Not OK to OK 
			if(ChlorinationHandler !=NULL && chlorine_status_f == SYS_OK)
			{
				state = eTaskGetState(ChlorinationHandler);
				if(state == eSuspended)
				{				
					if(nvs_data.Current_operating_status_f == ELECTROLYSIS_START_OPERATION)
					{
			              ESP_LOGI("SSR:","RELAY ON");
			              LED1_ON();
			              ESP_LOGI("Electrolysis:","Started");			              
			              #if POLARITY_TYPE == TIME_TYPE || POLARITY_TYPE == BATCH_TYPE
			                gpio_set_level(SSR_RELAY_GPIO,0);	// SSR OFF 
			              	uint8_t temp_polarity = (((nvs_data.last_polarity_status_f == 0) ? 1 : 0)); 
           					gpio_set_level(ELECTRODE_POLARITY_GPIO,temp_polarity); 
	                        output_status.Output_level.polarity = temp_polarity;
			              #endif
			              gpio_set_level(SSR_RELAY_GPIO,1);	// SSR ON 
			              output_status.Output_level.ssr = HIGH;					
		                 if (v1min_timer != NULL) {
	    					xTimerStart(v1min_timer, 0);  // Start the 1 Minute timer
	    					ESP_LOGI("1min:","Timer Resume");
					   }						
					}
					else if(nvs_data.Current_operating_status_f == CRT_FILLING_START_OPERATION)
					{
						 ESP_LOGI("CRT:","Filling Started");
						 LED1_ON();
						 ESP_LOGI("Valve V1:"," Started");
						 gpio_set_level(VALVE_V1_RELAY_GPIO,1);
						 output_status.Output_level.valve1 = HIGH;
						 vTaskDelay(xDelay); // 10 SEC delay is provide to the valve because it take time to open and close 			 
						 ESP_LOGI("PUMP P1 :","Started");
						 gpio_set_level(PUMP_P1_RELAY_GPIO,1);
						 output_status.Output_level.pump1 = HIGH;
						 if (v1sec_timer != NULL) {
					   	 xTimerStart(v1sec_timer, 0);  // Start the 1 SEC timer
						}						 
					}
					else if(nvs_data.Current_operating_status_f == CST_FILLING_START_OPERATION && gpio_get_level(BOTT_LEVEL_CRT_SENSOR_GPIO) == true)
					{
					  ESP_LOGI("VALVE V2:","ON");
					  LED1_ON();
					  gpio_set_level(VALVE_V2_RELAY_GPIO,1);
					  output_status.Output_level.valve2 = HIGH;
					  ESP_LOGI("CST:","FILLING STARTED");
					  if (v1sec_timer2 != NULL) {
					   	 xTimerStart(v1sec_timer2, 0);  // Start the 1 SEC timer
						}	
					}
					ESP_LOGI("Chlorination:","Task Resume");
					vTaskResume(ChlorinationHandler);	
				}
			}
			
#if REGENERATION_SUSPEND_RESUME_REQ == 1			
			if(RegenerationHandler !=NULL)
			{
				state = eTaskGetState(RegenerationHandler);
				if(state == eSuspended)
				{
				  if ((nvs_data.operating_mode & REGENERATION_OPERATION) == REGENERATION_OPERATION && (regeneration_status_f == SYS_OK))
				  {		
				     ESP_LOGI("Salt Dosing","Pump OFF");
				     gpio_set_level(SALT_DOSING_PUMP_D1_GPIO, 0); // OFF	
				     output_status.Output_level.pump2 = LOW;
					 ESP_LOGI("Valve V1:","OFF");
					 gpio_set_level(VALVE_V1_RELAY_GPIO,0);	// OFF
					 output_status.Output_level.valve1 = LOW;					     				  			  
					 ESP_LOGI("Pump P1:","Started");
					 gpio_set_level(PUMP_P1_RELAY_GPIO,1);		
					 output_status.Output_level.pump1 = HIGH;					 						 							 							 				 			  
				  }	
				  ESP_LOGI("Regeneration:","Task Getting Resume");
				  vTaskResume(RegenerationHandler);			  			
				}		  		
		    }
#endif
		    			
			if(DosingHandler !=NULL) // to be discuss 
			{
				state = eTaskGetState(DosingHandler);
				if(state == eSuspended)
				{
					if(((nvs_data.operating_mode & DOSING_OPERATION) == DOSING_OPERATION) && dosing_status_f == SYS_OK)
					{
						ESP_LOGI("Dosing:","Started");
						LED2_ON();
						gpio_set_level(DOSING_PUMP_D2_GPIO,1);
						ESP_LOGI("Dosing PUMP P2 :","Started");
						output_status.Output_level.pump3 = HIGH;							 
					 }	
					ESP_LOGI("Dosing","Task getting Resume");					
					vTaskResume(DosingHandler);	 				 							 		
				}				 				
			}  		
		    			
			prev_chlorine_status_f = chlorine_status_f;
			Prev_System_mode = System_mode;	
		}						   
			
		if(regeneration_status_f == SYS_NOK || dosing_status_f == SYS_NOK || chlorine_status_f == SYS_NOK)
		{
			System_status_f = SYS_NOK;		
			LED1_OFF();
			LED2_OFF();
			LED3_ON();
		}
		else
		{
			LED3_OFF();
		}			
}

static void pwm_set_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void read_System_Mode(void)
{
	if(gpio_get_level(OPERATING_MODE_SWITCH) == true)
	{
		System_mode = OPERATING_MODE;
	}
	else 
	{
 		System_mode = STAND_BY_MODE;	
	}	
}
static void GPIOs_Status_check(void)
{
	EventBits_t set_bits = 0;
    EventBits_t clear_bits = 0;
	float temp_time = (float)(current_time.tm_hour + (current_time.tm_min/60.0f));	

    if (gpio_get_level(FLOATY_SENSOR_GPIO))
       {set_bits |= FLOATY_SENSOR_BIT;
        clear_bits |= FLOATY_NO_SENSOR_BIT;}
    else
       {set_bits |= FLOATY_NO_SENSOR_BIT;
        clear_bits |= FLOATY_SENSOR_BIT;}   
        
    if (!(gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO)))
        {set_bits |= CRT_TOP_SENSOR_BIT;
		clear_bits |= CRT_NO_TOP_SENSOR_BIT;}       
    else
        {set_bits |= CRT_NO_TOP_SENSOR_BIT;
        clear_bits |= CRT_TOP_SENSOR_BIT;}
       
    if (gpio_get_level(BOTT_LEVEL_CRT_SENSOR_GPIO))
        {set_bits |= CRT_BOTTOM_SENSOR_BIT;
        clear_bits |= CRT_NOT_BOTTOM_SENSOR_BIT;}        
    else
        {set_bits |= CRT_NOT_BOTTOM_SENSOR_BIT;
        clear_bits |= CRT_BOTTOM_SENSOR_BIT;}        

    if (!(gpio_get_level(TOP_LEVEL_CST_SENSOR_GPIO)))
        {set_bits |= CST_TOP_SENSOR_BIT;
        clear_bits |= CST_NO_TOP_SENSOR_BIT;}        
    else
        {set_bits |= CST_NO_TOP_SENSOR_BIT;
        clear_bits |= CST_TOP_SENSOR_BIT;}              
     
    if (gpio_get_level(BOTT_LEVEL_CST_SENSOR_GPIO))
        {set_bits |= CST_BOTTOM_SENSOR_BIT;
        clear_bits |= CST_NO_BOTTOM_SENSOR_BIT;}        
    else
        {set_bits |= CST_NO_BOTTOM_SENSOR_BIT;
        clear_bits |= CST_BOTTOM_SENSOR_BIT;}  
     
     
    if (!(gpio_get_level(REGENERATION_SIGNAL_GPIO)))
        {set_bits |= REG_RELAY_SENSOR_BIT;
        clear_bits |= REG_NO_RELAY_SENSOR_BIT;}        
    else
        {set_bits |= REG_NO_RELAY_SENSOR_BIT;
        clear_bits |= REG_RELAY_SENSOR_BIT;}        

	/* CRT filling timer */
	if (nvs_data.crt_filling_timer_error_f == 1){
	    clear_bits |= CRT_FILLING_TIMER_NO_ERROR_BIT;
	}
	else{
	    set_bits |= CRT_FILLING_TIMER_NO_ERROR_BIT;
	}
	
	/* CST filling timer */
	if (nvs_data.cst_filling_timer_error_f == 1){
	    clear_bits |= CST_FILLING_TIMER_NO_ERROR_BIT;
	}
	else{
	    set_bits |= CST_FILLING_TIMER_NO_ERROR_BIT;
	} 


	// Check the Dosing Time 
	if(temp_time >= DOSING_START_TIME && temp_time < DOSING_END_TIME)
	{
		set_bits |= DOSING_TIME_BIT;
		clear_bits |= DOSING_STOP_TIME_BIT;
	}
	else
	{
		set_bits |= DOSING_STOP_TIME_BIT;
		clear_bits |= DOSING_TIME_BIT;
	}

    if((nvs_data.operating_mode & DOSING_OPERATION) != DOSING_OPERATION)
	{
		set_bits |= DOSING_OFF_BIT;
	}
	else
	{
		clear_bits |= DOSING_OFF_BIT;
	}


    // Update all bits at once
    xEventGroupClearBits(sensor_event_group, clear_bits);
    xEventGroupSetBits(sensor_event_group, set_bits);
}

static void Print_Operation_status(void)
{
char chlorination[20];
char Dosing[5];
char Regeneration[5];
char SYSTEM_STAT[5];
char SYSTEM_MODE[5];


if (nvs_data.Current_operating_status_f & (1 << 5))
    strcpy(chlorination, "CST Filling STOP");
else if (nvs_data.Current_operating_status_f & (1 << 4))
    strcpy(chlorination, "CST Filling START");
else if (nvs_data.Current_operating_status_f & (1 << 3))
    strcpy(chlorination, "Electrolysis STOP");
else if (nvs_data.Current_operating_status_f & (1 << 2))
    strcpy(chlorination, "Electrolysis START");
else if (nvs_data.Current_operating_status_f & (1 << 1))
    strcpy(chlorination, "CRT Filling STOP");
else if (nvs_data.Current_operating_status_f & (1 << 0))
    strcpy(chlorination, "CRT Filling START");
else
    strcpy(chlorination, "Idle");

strcpy(Dosing, (nvs_data.operating_mode & (1 << 6)) ? "ON" : "OFF");
strcpy(Regeneration, (nvs_data.operating_mode & (1 << 7)) ? "ON" : "OFF");
strcpy(SYSTEM_STAT, (System_status_f & SYS_OK) ? "OK" : "NOK");
strcpy(SYSTEM_MODE, (System_mode == OPERATING_MODE) ? "OP" : "STB");

printf("Main task Process: %s Dosing:%s Regeneration:%s , System Status :%s , System Mode : %s  Electrolysis Duration : %u   Error Code : %ld   Event Group %d  duration %d , tank filling counter %d , tank draining counter %d , electrolysis_done_f %d , current operation status %d , dosing done flag %d , dosing counter %d , crt filling error flag %d and cst filling error flag %d , crt reset done flag %d , Operating Mode %d  \n", chlorination, Dosing, Regeneration,SYSTEM_STAT,SYSTEM_MODE,nvs_data.timer_counter,system_op_error_code
      , event_grp_status,nvs_data.timer_counter,nvs_data.tank_filling_counter , nvs_data.tank_draining_counter , nvs_data.electrolysis_done_f , nvs_data.Current_operating_status_f , nvs_data.dosing_done_f , nvs_data.dosing_counter , nvs_data.crt_filling_timer_error_f , nvs_data.cst_filling_timer_error_f , nvs_data.crt_reset_counter_done_f,nvs_data.operating_mode);	
}

void Salt_dosing_operation(void)
{
    if (nvs_data.dosing_done_f == true)
        return;  // already done

    ESP_LOGI("Dosing Pump D1", "Started");
    gpio_set_level(SALT_DOSING_PUMP_D1_GPIO, 1);
    output_status.Output_level.pump2 = HIGH;

    while (nvs_data.dosing_counter < SALT_DOSING_PUMP_TIME)
    {
        if (chlorine_status_f == SYS_OK)
        {
            ESP_LOGI("Dosing Pump D1", "Running... %d / %d sec",
                     nvs_data.dosing_counter, SALT_DOSING_PUMP_TIME);

            gpio_set_level(SALT_DOSING_PUMP_D1_GPIO, 1);
            output_status.Output_level.pump2 = HIGH;
            vTaskDelay(pdMS_TO_TICKS(1000)); // precise 1-sec timing
            nvs_data.dosing_counter++;
        }
        else
        {
            ESP_LOGW("Dosing Pump D1", "System not OK — Paused");
            gpio_set_level(SALT_DOSING_PUMP_D1_GPIO, 0);
            output_status.Output_level.pump2 = LOW;
            vTaskDelay(pdMS_TO_TICKS(500)); // normal delay (not sync)
        }
    }

    nvs_data.dosing_done_f = true;
    ESP_LOGI("Salt Dosing", "Complete");
    gpio_set_level(SALT_DOSING_PUMP_D1_GPIO, 0); // OFF
    output_status.Output_level.pump2 = LOW;
}
#if POLARITY_TYPE == TIME_TYPE
static void polarity_controller(void)
{
	if(chlorine_status_f == SYS_OK && nvs_data.Current_operating_status_f == ELECTROLYSIS_START_OPERATION && (gpio_get_level(OPERATING_MODE_SWITCH) == OPERATING_MODE))
	{
		int16_t polarity_cal = ((nvs_data.timer_counter % (2*POLARITY_DIVIDER)));
		if(polarity_cal >= POLARITY_DIVIDER && nvs_data.switching_polarity !=1)
		{
			gpio_set_level(SSR_RELAY_GPIO,0); // SSR OFF 
			vTaskDelay(pdMS_TO_TICKS(1000));     // 1 SEC delay 
			nvs_data.last_polarity_status_f = (((nvs_data.last_polarity_status_f == 0) ? 1 : 0)); 
			uint8_t temp_polarity = (((nvs_data.last_polarity_status_f == 0) ? 1 : 0));
            gpio_set_level(ELECTRODE_POLARITY_GPIO,temp_polarity); // LOW  -VE Polarity	                    
            gpio_set_level(SSR_RELAY_GPIO,1); // SSR ON 
			output_status.Output_level.polarity = temp_polarity;
			nvs_data.switching_polarity = 1;
		}
		else if(polarity_cal < POLARITY_DIVIDER && nvs_data.switching_polarity !=0)
		{
			gpio_set_level(SSR_RELAY_GPIO,0); // SSR OFF 
			vTaskDelay(pdMS_TO_TICKS(1000));     // 1 SEC delay  
			nvs_data.last_polarity_status_f = (((nvs_data.last_polarity_status_f == 0) ? 1 : 0)); 
			uint8_t temp_polarity = (((nvs_data.last_polarity_status_f == 0) ? 1 : 0)); 
            gpio_set_level(ELECTRODE_POLARITY_GPIO,temp_polarity); // LOW  -VE Polarity	                  
            gpio_set_level(SSR_RELAY_GPIO,1); // SSR ON 
			output_status.Output_level.polarity = temp_polarity;
			nvs_data.switching_polarity = 0;
		}
	}
}
#endif

static void Uart_init(void)
{
	  uart_config_t uart_config = {
	    .baud_rate = 9600,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // No hardware Flow control is using 
	    .rx_flow_ctrl_thresh = 0, // Signal of flow control is set to Zero (Not Required).
	    };
	    // 1️ Configure UART parameters
	    ESP_ERROR_CHECK(uart_param_config(uart_num1, &uart_config));	
	    // 2️ Set GPIO pins
	    ESP_ERROR_CHECK(uart_set_pin(uart_num1, UART1_TXD_GPIO, UART1_RXD_GPIO,
	                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));	
	    // 3️ Install UART driver
	    ESP_ERROR_CHECK(uart_driver_install(uart_num1, 1024, 1024, 0, NULL, 0));	
	    // 4️ Set mode (optional, only if you need RS485 or IRDA)
	    ESP_ERROR_CHECK(uart_set_mode(uart_num1, UART_MODE_UART));
    
	#if RS485_MODBUS_COMM_REQ == 1
	    uart_config_t uart_config1 = {
	    .baud_rate = 9600,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // No hardware Flow control is using 
	    .rx_flow_ctrl_thresh = 0, // Signal of flow control is set to Zero (Not Required).
	    };
	    // Configure UART parameters
	    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config1));	
	    // Set GPIO pins
	    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART0_TXD_GPIO, UART0_RXD_GPIO,
	                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 1024, 0, NULL, 0));
	    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_UART));
	#endif    
}

static void Get_Flow_rate(void)
{
    int adc_raw_data;
    static float previous_flow_rate = 0.0f;
    static uint8_t previous_cst_status = 0;
    EventBits_t set_bits = 0;

    if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw_data) != ESP_OK) {
        return;
    }

    Flow_Rate = ((float)adc_raw_data / 4095.0f) * 3.3f;  // voltage
    Flow_Rate = ((0.7 * Flow_Rate) + (0.3 * previous_flow_rate));
    
    ESP_LOGI("FLOW", "Rate = %.3f", Flow_Rate);
    float difference_rate = fabsf(Flow_Rate - previous_flow_rate);
    previous_flow_rate = Flow_Rate;
    if (difference_rate > FLOW_RATE_DIFF_THRESHOLD) {
        set_bits |= FLOW_METER_SENSOR_BIT;
    }

    uint8_t current_cst_status = gpio_get_level(BOTT_LEVEL_CST_SENSOR_GPIO);
    if (previous_cst_status != current_cst_status) {
        set_bits |= CST_BOTTOM_CHANGE_BIT;
        previous_cst_status = current_cst_status;
    }
    if (Flow_Rate < FLOW_RATE_MIN_VALUE) {
        xEventGroupSetBits(sensor_event_group, FLOW_METER_NO_SENSOR_BIT);
    } else {
        xEventGroupClearBits(sensor_event_group, FLOW_METER_NO_SENSOR_BIT);
    }
    
    if (set_bits) {
        xEventGroupSetBits(sensor_event_group, set_bits);
    }
}


static uint32_t pwm_from_percent(float percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    Dosing_speed = percent;
    return (uint32_t)((percent / 100.0f) * 1023); //1023 because we are using the 10 bit PWM 
}


#if RS485_MODBUS_COMM_REQ == 1
static void v_Modbus_Read_task(void *pvParameters)
{
	float Vol_output,Curr_output;
    while(1)
    {
        memset(ModBus_response, 0, sizeof(ModBus_response)); // Clear the Modbus Data Response
		Vol_output = 0;
		Curr_output = 0;
        // Send Modbus Read Command
        int bytes_sent = uart_write_bytes(uart_num, (const char*)MODBUS_READ_VOL_CURR_CMD, sizeof(MODBUS_READ_VOL_CURR_CMD));
        if (bytes_sent != sizeof(MODBUS_READ_VOL_CURR_CMD)) {
            ESP_LOGE("MODBUS", "Command send failed (%d bytes sent)", bytes_sent);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue; 
        }

        // Read available bytes from UART (up to buffer size)
        int len = uart_read_bytes(uart_num, ModBus_response, sizeof(ModBus_response), pdMS_TO_TICKS(5000));
        if (len > 0)
        {
            ESP_LOGI("MODBUS", "Received %d bytes:", len);
            char hex_str[3 * len + 1];
            char *p = hex_str;
            for (int i = 0; i < len; i++) {
                p += sprintf(p, "%02X ", ModBus_response[i]);
            }
            ESP_LOGI("MODBUS", "%s", hex_str);
            if(len == 9)
            {
				uint16_t raw_voltage = (ModBus_response[3] << 8) | ModBus_response[4];
				uint16_t raw_current = (ModBus_response[5] << 8) | ModBus_response[6];				
				Vol_output  = (raw_voltage / 100.0f);   // Convert to float
				Curr_output = (raw_current / 1000.0f);  // Convert to float
				Electrode_voltage = Vol_output;
				Electrode_current = Curr_output;
            	ESP_LOGI("MODBUS", "Voltage %f V  Current %f A", Vol_output, Curr_output);				
			}
        }
        else
        {
            ESP_LOGW("MODBUS", "No response received");
        }
        vTaskDelay(pdMS_TO_TICKS(3000)); // We change the time from 2 SEC to 3 SEC
    }  
}
#endif

#if RS485_MODBUS_COMM_REQ == 0
static void read_electrode_parameter(void)
{	
    float Vol_output = 0;
    float Curr_output =0; 
	float volt_sensor_output_v = 0;
	float curr_sensor_output_v = 0;
	int adc_raw_data=0;
	int32_t total_sample_value = 0;
	
	// Voltage Sensor Measurement
	for(uint8_t index = 0 ; index < ADC_TOTAL_SAMPLE ; index++)
	{	
	    adc_oneshot_read(adc2_handle, ADC_CHANNEL_6, &adc_raw_data); 
	   	vTaskDelay(pdMS_TO_TICKS(10)); 
		total_sample_value += adc_raw_data;  
	}
	total_sample_value = (total_sample_value/ADC_TOTAL_SAMPLE); 
	volt_sensor_output_v = (((float)total_sample_value / 4095.0f) *  3.3f); 
	if(volt_sensor_output_v != 0){
		Vol_output = ((volt_sensor_output_v * 5.0f) + Volt_OFFSET);   // Multiply by 5 as per voltage divider Rule 		
	}
	Electrode_voltage = Vol_output;
	ESP_LOGI("Electrode", "Actual Voltage = %.3f V", Vol_output);
		


	// Current Sensor Measurement
	total_sample_value = 0;
	for(uint8_t index = 0 ; index < ADC_TOTAL_SAMPLE ; index++)
	{
	    adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &adc_raw_data);
	   	vTaskDelay(pdMS_TO_TICKS(10)); 
		total_sample_value += adc_raw_data;  
	}
	total_sample_value = (total_sample_value/ADC_TOTAL_SAMPLE);	
	curr_sensor_output_v = (((float)total_sample_value / 4095.0f) *  3.3f);	
	Curr_output = ((curr_sensor_output_v- Curr_OFFSET)/0.0238f); 
	Electrode_current = Curr_output;  
	ESP_LOGI("Electrode", "Actual Current = %.3f A", Curr_output);	
}
#endif


static void monitor_task_stack(void)
{
    struct {
        const char *name;
        uint32_t stack_size;
    } tasks[] = {
        {"Dosing Task", 4096},
        {"Regeneration Task", 4096},
        {"Chlorination Task", 4096},
        {"NVS data Saving Task", 4096},
        {"reset_button_task", 4096},
        {"Tmr Svc", 2048}  // Timer task stack size from menuconfig
    };

    const int task_count = sizeof(tasks)/sizeof(tasks[0]);

    for (int i = 0; i < task_count; i++) {
        TaskHandle_t handle = xTaskGetHandle(tasks[i].name);
        if (handle == NULL) {
            ESP_LOGW("STACK_MON", "Task %s not found!", tasks[i].name);
            continue;
        }

        uint32_t free_bytes = uxTaskGetStackHighWaterMark(handle);  // bytes
        uint32_t total_bytes = tasks[i].stack_size;
        uint32_t used_bytes = total_bytes - free_bytes;
        uint32_t used_percent = (used_bytes * 100) / total_bytes;

        ESP_LOGI("STACK_MON",
                 "Task: %-20s | Used: %" PRIu32 " B (%" PRIu32 "%%) | Free: %" PRIu32 " B | Total: %" PRIu32 " B",
                 tasks[i].name, used_bytes, used_percent, free_bytes, total_bytes);
    }
}

static void get_esp32_mac_id(uint8_t *mac_id)
{
    if (mac_id == NULL) {
         printf("Invalid MAC ID buffer\n");
        return;
    }
    // Get the MAC address for the default Wi-Fi interface (station mode)
    esp_err_t err = esp_efuse_mac_get_default(mac_id);
    if (err == ESP_OK) {
		snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
                 mac_id[0], mac_id[1], mac_id[2], mac_id[3], mac_id[4], mac_id[5]);
         printf("MAC ID retrieved successfully\n");
         printf("ESP32 MAC ID: %s\n", mac_str);                                                      // Print MAC ID
    } else {
        printf("Failed to retrieve MAC ID, error: %d\n", err);
        memset(mac_id, 0, 6);                                                                               // Assign default zeros in case of failure
    }	
}

uint32_t load_write_index(void)
{
    uint32_t index = 0;
    FILE *f = fopen(META_FILE, "rb");
    if (f) {
        fread(&index, sizeof(index), 1, f);
        fclose(f);
    }
    return index;
}

void save_write_index(uint32_t index)
{
    FILE *f = fopen(META_FILE, "wb");
    if (f) {
        fwrite(&index, sizeof(index), 1, f);
        fclose(f);
    }
}


void save_log_data(NVS_DATA *data)
{
    static uint32_t write_index = 0;
    static bool initialized = false;

    if (!initialized) {
        write_index = load_write_index();
        initialized = true;
    }

    FILE *f = fopen(DATA_FILE, "r+b");
    if (!f) {
        f = fopen(DATA_FILE, "w+b");   // create if not exists
        if (!f) return;
    }

    uint32_t offset = write_index * RECORD_SIZE;

    // If file space over → wrap to beginning
    if (offset >= MAX_FILE_SIZE) {
        write_index = 0;
        offset = 0;
    }

    fseek(f, offset, SEEK_SET);
    fwrite(data, RECORD_SIZE, 1, f);
    fflush(f);
    fclose(f);

    write_index++;
    save_write_index(write_index);
}

bool read_latest_log(NVS_DATA *data)
{
    uint32_t write_index = load_write_index();
    if (write_index == 0) return false;

    uint32_t index = write_index - 1;
    uint32_t offset = index * RECORD_SIZE;

    FILE *f = fopen(DATA_FILE, "rb");
    if (!f) return false;

    fseek(f, offset, SEEK_SET);
    fread(data, RECORD_SIZE, 1, f);
    fclose(f);
    return true;
}




static void send_MAC_ID_data(void)
{
	uint8_t mac_id_local[6] = {0};
	
	memset(data_Buffer, 0, sizeof(data_Buffer));
	get_esp32_mac_id(mac_id_local);
	memcpy(data_Buffer, mac_id_local, 6);
	
	int bytes_sent = uart_write_bytes(uart_num1, (const char*)data_Buffer, MAC_ID_CMD_LENGTH); //6 byte MAC ID is send 
	if (bytes_sent != MAC_ID_CMD_LENGTH) {
	    ESP_LOGE("Error", "UART Send Failed (%d bytes sent)", bytes_sent);
	    return;
	}
	
	ESP_LOGI("MAC Command", "Packet sent successfully (%d bytes)", bytes_sent);			
}



static void send_status_data(void)
{
uint16_t IP_status = 0;
uint8_t OP_status = 0;
uint8_t system_status = 0;

memset(data_Buffer, 0, sizeof(data_Buffer));

IP_status |= gpio_get_level(FLOATY_SENSOR_GPIO)            ? FLOATY_SENSOR_BIT        : 0;
IP_status |= gpio_get_level(TOP_LEVEL_CRT_SENSOR_GPIO)     ?    0                     : CRT_TOP_SENSOR_BIT;
IP_status |= gpio_get_level(BOTT_LEVEL_CRT_SENSOR_GPIO)    ? CRT_BOTTOM_SENSOR_BIT    : 0;
IP_status |= gpio_get_level(TOP_LEVEL_CST_SENSOR_GPIO)     ?    0    				  : CST_TOP_SENSOR_BIT;
IP_status |= gpio_get_level(BOTT_LEVEL_CST_SENSOR_GPIO)    ? CST_BOTTOM_SENSOR_BIT    : 0;
IP_status |= gpio_get_level(REGENERATION_SIGNAL_GPIO)      ?    0  					  : REG_RELAY_SENSOR_BIT;
IP_status |= gpio_get_level(OPERATING_MODE_SWITCH)         ? OPERATING_MODE_BIT       : 0;

data_Buffer[0] = (IP_status >> 8);
data_Buffer[1] = (IP_status & 0xFF);

OP_status |= ((output_status.Output_level.pump1 == LOW)                       ? 0                       : CRT_PUMP_P1_BIT);
OP_status |= ((output_status.Output_level.pump2 == LOW)      				  ? 0                       : SALT_DOSING_PUMP_BIT);
OP_status |= ((output_status.Output_level.valve1 == LOW)                      ? 0                       : VALVE_V1_BIT);
OP_status |= ((output_status.Output_level.valve2 == LOW)                      ? 0                       : VALVE_V2_BIT);
OP_status |= ((output_status.Output_level.ssr == LOW)                         ? 0                       : SSR_RELAY_BIT);
OP_status |= ((output_status.Output_level.polarity == LOW)                    ? 0                       : POLARITY_BIT);
OP_status |= ((output_status.Output_level.pump3 == LOW)                       ? 0                       : DOSING_PUMP_BIT);


data_Buffer[2] = OP_status;

system_status |= (regeneration_status_f == SYS_OK) ? REG_SYS_BIT     : 0;
system_status |= (chlorine_status_f == SYS_OK)     ? CHLORINE_SYS_BIT : 0;
system_status |= (dosing_status_f == SYS_OK)       ? DOSING_SYS_BIT   : 0;

data_Buffer[3] = system_status;
data_Buffer[4] = nvs_data.Current_operating_status_f;
data_Buffer[5] = nvs_data.operating_mode;

memcpy(&data_Buffer[6],  (const void *)&Electrode_voltage, sizeof(float)); // 4 byte Voltage Value
memcpy(&data_Buffer[10],  (const void *)&Electrode_current, sizeof(float)); // 4 byte current Value

data_Buffer[14] = (nvs_data.timer_counter >> 8); // Electro-lysis Duration
data_Buffer[15] = (nvs_data.timer_counter & 0xFF);

memcpy(&data_Buffer[16], &system_op_error_code, sizeof(system_op_error_code)); // Error Code 

data_Buffer[20] = time_sync_done_f; // It tells that time sync is done or Not , 0: No Time Sync 1 : Time Sync 

Electrode_current =0; Electrode_voltage =0;
int bytes_sent = uart_write_bytes(uart_num1, (const char*)data_Buffer, DATA_CMD_LENGTH); // 21 Bytes is a Data Packet Length
if (bytes_sent != DATA_CMD_LENGTH) {
    ESP_LOGE("Error", "UART Send Failed (%d bytes sent)", bytes_sent);
    return;
}

ESP_LOGI("DATA Command", "Packet sent successfully (%d bytes)", bytes_sent);
}

static void data_transfer(void)
{
uint8_t response[12] = {0};
int len = uart_read_bytes(uart_num1, response, sizeof(response), pdMS_TO_TICKS(300));

if (len == 4 || len == 11)
{
    // Fast validation — single conditional
    if (len == 4 && response[0] == 0xAA && response[3] == 0xFF)
    {
        uint8_t action_cmd = response[1];

        switch (action_cmd)
        {
			case MAC_ID_GET_COMMAND:
				ESP_LOGI("MAC","GET COMMAND EXECUTE");
				send_MAC_ID_data();
			break;
			
            case STATUS_GET_COMMAND:
                ESP_LOGI("CMD", "Status command");
                send_status_data();
                break;

            case DEBUG_COMMAND:
                ESP_LOGI("CMD", "Debug command");
                // Debug function here
                break;

            default:
                ESP_LOGW("CMD", "Unknown command: %02X", action_cmd);
                break;
        }
    }
	else if(len == 11 && response[0] == 0xAA && response[10] == 0xFF)
	{
        uint8_t action_cmd = response[1];

        switch (action_cmd)
        {			
            case STATUS_GET_COMMAND:
                ESP_LOGI("TDCMD", "Status command");
				current_time.tm_year = ((response[3]*100 + response[4])- 1900);
				current_time.tm_mon = response[5];
				current_time.tm_mday = response[6];
				current_time.tm_hour = response[7];
				current_time.tm_min = response[8];
				current_time.tm_sec = response[9];	
				current_time.tm_isdst = -1;
				time_t epoch = mktime(&current_time);
				if (epoch != -1) {
					struct timeval tv = {
						.tv_sec = epoch,
						.tv_usec = 0
					};
					settimeofday(&tv, NULL);   //RTC starts auto-updating
					time_sync_done_f = 1;
				}
				ESP_LOGI("Time","RTC Set: %04d-%02d-%02d %02d:%02d:%02d",current_time.tm_year + 1900,current_time.tm_mon + 1,current_time.tm_mday,current_time.tm_hour,current_time.tm_min,current_time.tm_sec);							
                send_status_data();
                break;

            default:
                ESP_LOGW("CMD", "Unknown command: %02X", action_cmd);
                break;
        }
	}
    else
    {
        ESP_LOGW("CMD", "Invalid frame: %02X %02X %02X %02X",
                 response[0], response[1], response[2], response[3]);
    }
}	
}
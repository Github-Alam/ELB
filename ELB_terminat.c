#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "nvs_rw_val.h"
#include "esp_sleep.h"



 /*  Service Declaration UUID*/
#define   ESP_GATT_UUID_Environmental_SVC		0x181A		  	/*  Environmental Sensing service*/

/*  Characteristic Declaration UUID*/
// Temperature Measurement value
#define    ESP_GATT_TEM_MEAS                 	0x2A1C
//  Humidity Measurement
#define    ESP_GATT_HUM_MEAS                 	0x2A6F
//Time Update Control Point
#define    ESP_GATT_TIME_UPDATE_CP 				0x2A16
//Time Update State
#define    ESP_GATT_TIME_UPDATE_ST 				0x2A17

#define    ESP_GATT_TIME_SEC 		0x2B16


#define    C_PAIR_MAX_DEVICE  12

void ble_app_advertise(void);

char *TAG = "BLE-Server";
uint8_t ble_addr_type;
uint8_t temp =0;
uint8_t hum =0;
uint16_t ON_time =0;
uint16_t SLEEP_time =1;
uint16_t * turnONtime = 0;
uint16_t * SleepTimeS = 0;
uint16_t conn_handle;
bool FanStatus = true;
bool blueLED_ST = false;



#define BLUE_LED 26
#define RED_LED  25
#define SW  	 14


#define GPIO_BUTTON_PIN  GPIO_NUM_14 // Example GPIO pin for button input
#define GPIO_INTR_FLAG 0 // Interrupt flag for GPIO configuration

// Callback function for GPIO interrupt
void IRAM_ATTR gpio_isr_handler(void* arg) {
    // Terminate BLE connection
   // printf("Terminating BLE connection...\n");
   // ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM); // Replace conn_handle with actual connection handle
}

// Initialize GPIO pin for button input
void gpio_init() {
    gpio_config_t io_conf;
    // Configure button GPIO as input pin
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Trigger interrupt on falling edge (button press)
    io_conf.pin_bit_mask = (1ULL << GPIO_BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    // Install ISR service with default handler assigned
    gpio_install_isr_service(GPIO_INTR_FLAG);
    // Hook ISR handler for specific GPIO pin
    gpio_isr_handler_add(GPIO_BUTTON_PIN, gpio_isr_handler, (void*) GPIO_BUTTON_PIN);
}

// Write data to ESP32 defined as server
static int device_write_Interval(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	SleepTimeS =(uint16_t *)ctxt->om->om_data;
	SLEEP_time = *SleepTimeS;
	nvs_Write(my_handle, "INVtime", (uint16_t) SLEEP_time);
    printf("Sleep TimeSec from the client: %d sec\n", *SleepTimeS);
    FanStatus = true;
    return 0;
}

// Write data to ESP32 defined as server
static int device_write_ONduration(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);

	turnONtime =(uint16_t *)ctxt->om->om_data;
	ON_time = *turnONtime;
	nvs_Write(my_handle, "ONtime", (uint16_t) ON_time);
	printf("turn ON time from the client: %d sec\n", *turnONtime);
	FanStatus = true;

    return 0;
}

// Read data from ESP32 defined as server
static int devTemp_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, (uint8_t*)&temp, 1);
   // temp++;
    return 0;
}

static int devHum_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, (uint8_t*)&hum, 1);
    //hum++;
    return 0;
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
//     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .uuid = BLE_UUID16_DECLARE(ESP_GATT_UUID_Environmental_SVC),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
//         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
         {.uuid = BLE_UUID16_DECLARE(ESP_GATT_TEM_MEAS),           // Define UUID for Temperature reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = devTemp_read},
		 {.uuid = BLE_UUID16_DECLARE(ESP_GATT_HUM_MEAS),           // Define UUID for Humidity reading
		  .flags = BLE_GATT_CHR_F_READ,
		  .access_cb = devHum_read},
         {.uuid = BLE_UUID16_DECLARE(ESP_GATT_TIME_UPDATE_CP),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_Interval},
         {.uuid = BLE_UUID16_DECLARE(ESP_GATT_TIME_UPDATE_ST),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_ONduration},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
	//int rc;
	printf("GAP_EVT, event %d\n", event->type);
    switch (event->type)
    {
    // A new connection was established or a connection attempt failed
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        gpio_set_level(BLUE_LED, 1);
  		vTaskDelay(100/portTICK_PERIOD_MS);
  		gpio_set_level(BLUE_LED, 0);
  		vTaskDelay(100/portTICK_PERIOD_MS);

        // Extract the connection handle from the event parameters
         conn_handle = event->connect.conn_handle;

        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }

        else if (event->connect.status == 0)
	    {
        	blueLED_ST = true;
	    }
        break;

    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
    	gpio_set_level(BLUE_LED, 0);
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        if (event->connect.status != 0)
		  {
			  blueLED_ST = false;
			  ble_app_advertise();
		  }
        break;

        //Update the connection parameters
     case BLE_GAP_EVENT_CONN_UPDATE:
		  gpio_set_level(BLUE_LED, 1);
		  break;

		  //request send for update the connection parameters
     case BLE_GAP_EVENT_CONN_UPDATE_REQ:
   	      gpio_set_level(BLUE_LED, 1);
		  vTaskDelay(100/portTICK_PERIOD_MS);
		  gpio_set_level(BLUE_LED, 0);
		  vTaskDelay(100/portTICK_PERIOD_MS);
		  break;

		  //Complete advertisement
     case BLE_GAP_EVENT_ADV_COMPLETE:
			ESP_LOGI("GAP", "BLE GAP EVENT COMPLETE");
			ble_app_advertise();
			gpio_set_level(BLUE_LED, 1);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(BLUE_LED, 0);
           break;
    default:
    	printf("default");
    	 break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
	//struct ble_gap_event *event;

   for(uint8_t i = 0; i<2; i++)
	{
		gpio_set_level(BLUE_LED, 1);
		vTaskDelay(50/portTICK_PERIOD_MS);
		gpio_set_level(BLUE_LED, 0);
		vTaskDelay(50/portTICK_PERIOD_MS);
	}
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    adv_params.itvl_min = 0x100; 	 // 160 ms, Time = N * 0.625 msec
    adv_params.itvl_max = 0x800;	// 8*160ms
    ble_gap_adv_start(ble_addr_type, NULL, 1000, &adv_params, ble_gap_event, NULL);
    for(uint8_t i = 0; i<2; i++)
  	{
  		gpio_set_level(BLUE_LED, 1);
  		vTaskDelay(50/portTICK_PERIOD_MS);
  		gpio_set_level(BLUE_LED, 0);
  		vTaskDelay(50/portTICK_PERIOD_MS);
  	}
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

// Generates integer random number
// numbers in range lower, upper.
uint16_t creatRandoms(uint8_t lower, uint32_t upper, uint8_t count)
{
	uint16_t num=0;
	for( uint8_t i=0; i<count; i++)
	{
		num = ((rand()%(upper - lower + 1))+lower);
	}
	return num;
}

// Function to terminate a BLE connection
void terminate_connection(uint16_t conn_handle, uint8_t reason) {
    // Use NimBLE stack API to terminate the connection
    ble_gap_terminate(conn_handle, reason);
}

void app_main()
{
    nvs_flash_init();                          // 1 - Initialize NVS flash using
    esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set("Environmental_Sensor");   // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    gpio_init();								// Initialize GPIO

	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(SW, GPIO_MODE_INPUT);
	FanStatus = true;

    while(1)
      {
    	if(blueLED_ST!=true)
    	{
    		for(uint8_t i = 0; i<2; i++)
    		{
    			gpio_set_level(BLUE_LED, 1);
    			vTaskDelay(50/portTICK_PERIOD_MS);
    			gpio_set_level(BLUE_LED, 0);
    			vTaskDelay(50/portTICK_PERIOD_MS);
    		}
    	}

//    	if(gpio_get_level(SW)==0)
//    		vTaskDelay(1000/portTICK_PERIOD_MS);
//    	if(gpio_get_level(SW)==0)
//    	{
//    	    // Terminate the BLE connection
//    	    terminate_connection(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
//
//    	}

    	if(FanStatus!=false)
		{
			nvs_Read(my_handle, "ONtime", (uint16_t*) &ON_time); // time in sec
			nvs_Read(my_handle, "INVtime", (uint16_t*) &SLEEP_time); // time in SEC
			FanStatus=false;
		}
    	sleep(SLEEP_time); // time in sec

		gpio_set_level(RED_LED, 1);
		vTaskDelay(1000*ON_time/portTICK_PERIOD_MS);
		gpio_set_level(RED_LED, 0);
		temp = creatRandoms(25, 35, 2);
		hum = creatRandoms(40, 70, 2);
        nimble_port_freertos_init(host_task);      // 6 - Run the thread

        if(blueLED_ST!=true)
		{
		 for(uint8_t i = 0; i<2; i++)
			{
				gpio_set_level(BLUE_LED, 1);
				vTaskDelay(50/portTICK_PERIOD_MS);
				gpio_set_level(BLUE_LED, 0);
				vTaskDelay(50/portTICK_PERIOD_MS);
			}
		}
      }
}

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/reboot.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/launchapp_msg.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>


#include <bluetooth/services/lbs.h>
#include <zephyr/settings/settings.h>

#include "rotary.h"

// NFC
#define NDEF_MSG_BUF_SIZE	256
#define NFC_FIELD_LED		DK_LED1

// BLUETOOTH
#define DEVICE_NAME         CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED      DK_LED1
#define CON_STATUS_LED      DK_LED2
// #define RUN_LED_BLINK_INTERVAL 1000

#define USER_BUTTON         DK_BTN1_MSK

static bool app_button_state;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};


// NFC
/* URI nrf-toolbox://main/ */
static const uint8_t universal_link[] = {
	'n', 'r', 'f', '-', 't', 'o', 'o', 'l', 'b', 'o', 'x', ':', '/', '/', 'm', 'a', 'i', 'n',
	'/'};

/* Package: com.sec.android.app.camera */
/* If you want to use Camera App with NFC, Just use this code! */
// static const uint8_t camara_pkg_name[] = {
// 	'c', 'o', 'm', '.', 's', 'e', 'c', '.', 'a', 'n', 'd', 'r', 'o', 'i', 'd', '.', 'a', 'p', 'p', '.', 'c', 'a', 'm', 'e', 'r', 'a'
// };

/* Package: no.nordicsemi.android.mcp */
static const uint8_t nrf_connect_pkg_name[] = {
	'n', 'o', '.', 'n', 'o', 'r', 'd', 'i', 'c', 's', 'e', 'm', 'i', '.', 'a', 'n', 'd', 'r', 'o', 'i', 'd', '.', 'm', 'c', 'p'
};
/** .. include_endpoint_pkg_def_launchapp_rst */

/* Buffer used to hold an NFC NDEF message. */
static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];

// Rotary Sensor
#define SW_NODE DT_NODELABEL(gpiosw)
static const struct gpio_dt_spec sw = GPIO_DT_SPEC_GET(SW_NODE, gpios);

static bool sw_led_flag = false;
static int rotary_idx = 0;

static struct gpio_callback sw_cb_data;

// Bluetooth functions
static void bt_connected(struct bt_conn *conn, uint8_t err)
{
    if(err)
    {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    printk("Connected\n");

	led_off_all();
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	dk_set_led_off(CON_STATUS_LED);
}

#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
			err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = bt_connected,
	.disconnected     = bt_disconnected,
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;

#define USER_LED DK_LED3

static void app_led_cb(bool led_state)
{
	dk_set_led(USER_LED, led_state);
	led_all_set(led_state);
}

static bool app_button_cb(void)
{
	return app_button_state;
}

static struct bt_lbs_cb lbs_callbacs = {
	.led_cb    = app_led_cb,
	.button_cb = app_button_cb,
};

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & USER_BUTTON) {
		uint32_t user_button_state = button_state & USER_BUTTON;

		bt_lbs_send_button_state(user_button_state);
		app_button_state = user_button_state ? true : false;
	}
}

static int init_button(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}


static void nfc_callback(void *context,
			 nfc_t2t_event_t event,
			 const uint8_t *data,
			 size_t data_length)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(data_length);

	switch (event) {
	case NFC_T2T_EVENT_FIELD_ON:
		dk_set_led_on(NFC_FIELD_LED);
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		dk_set_led_off(NFC_FIELD_LED);
		break;
	default:
		break;
	}
}

void sw_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("SW pressed\n");
    sw_led_flag = !sw_led_flag;
}

int main(void)
{
	// LED(RICH SHIELD TWO )
	
    int blink_status = 0;

	// for error codes
	int err;

    struct sensor_value val;
    int rc;
    const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(qdec0));

	size_t len = sizeof(ndef_msg_buf);

    printk("Starting Bluetooth\n");
	// 버튼 초기화(DK 내부 버튼들 사용하려고 함)
	err = init_button();
	if(err) {
		printk("Button init failed (err %d)\n", err);
		return 0;
	}

	// BLE Security 설정 여부 확인하고, 설정되어있으면 실행하는 것.
    if (IS_ENABLED(CONFIG_BT_LBS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	// bluetooth 사용으로 설정하는 과정
    err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

    printk("Bluetooth initialized\n");

	// setting값이 있으면 load
    if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	// 버튼으로 LED 제어 준비
	err = bt_lbs_init(&lbs_callbacs);
	if (err) {
		printk("Failed to init LBS (err:%d)\n", err);
		return 0;
	}

	// 블루투스 Advertising
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("Advertising successfully started\n");


	// NFC 시작
	printk("Starting NFC\n");

    if(!device_is_ready(dev))
    {
        printk("Qdec device is not ready\n");
        return 0;
    }

    if(!gpio_is_ready_dt(&sw))
    {
		printk("SW GPIO is not ready\n");
		return 0;
	}

	/* Configure LED-pins as outputs */
	err = dk_leds_init();
	if (err) {
		printk("Cannot init LEDs!\n");
		goto fail;
	}

	/* Set up NFC */
	err = nfc_t2t_setup(nfc_callback, NULL);
	if (err) {
		printk("Cannot setup NFC T2T library!\n");
		goto fail;
	}

    err = gpio_pin_configure_dt(&sw, GPIO_INPUT);
    if(err < 0)
    {
        printk("Error configuring sw GPIO pin %d\n", err);
        return 0;
    }

    err = gpio_pin_interrupt_configure_dt(&sw, GPIO_INT_EDGE_RISING);
    if(err != 0)
    {
        printk("Error configuring SW GPIO interrupt %d\n", err);
        return 0;
    }

    gpio_init_callback(&sw_cb_data, sw_callback, BIT(sw.pin));
    gpio_add_callback(sw.port, &sw_cb_data);

    // led_on_idx(rotary_idx);
	led_off_all();

	/* Encode launch app data  */
	err = nfc_launchapp_msg_encode(nrf_connect_pkg_name,
				       sizeof(nrf_connect_pkg_name),
				       universal_link,
				       sizeof(universal_link),
				       ndef_msg_buf,
				       &len);
	if (err) {
		printk("Cannot encode message!\n");
		goto fail;
	}

	/* Set created message as the NFC payload */
	err = nfc_t2t_payload_set(ndef_msg_buf, len);
	if (err) {
		printk("Cannot set payload!\n");
		goto fail;
	}

	/* Start sensing NFC field */
	err = nfc_t2t_emulation_start();
	if (err) {
		printk("Cannot start emulation!\n");
		goto fail;
	}

	printk("NFC configuration done\n");

	int32_t rotary_value = 0;

    while(1)
    {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        
        rc = sensor_sample_fetch(dev);
        rc = sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &val);

		// if(app_button_state)
		rotary_value += (val.val1 / 72 * 5);

		if(rotary_value > 100)
			rotary_value = 100;
		else if(rotary_value < 0)
			rotary_value = 0;
		led_brightness(rotary_value);
		// printk("rotary value: %d\n", val.val1);	
		// else
		// 	led_off_all();

        k_msleep(100);
    }
    
	return 0;

fail:
#if CONFIG_REBOOT
	sys_reboot(SYS_REBOOT_COLD);
#endif /* CONFIG_REBOOT */

	return err;
}

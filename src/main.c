/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/reboot.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/launchapp_msg.h>

#include <dk_buttons_and_leds.h>

#include "rotary.h"

#define NDEF_MSG_BUF_SIZE	256
#define NFC_FIELD_LED		DK_LED1


// NFC
/* URI nrf-toolbox://main/ */
static const uint8_t universal_link[] = {
	'n', 'r', 'f', '-', 't', 'o', 'o', 'l', 'b', 'o', 'x', ':', '/', '/', 'm', 'a', 'i', 'n',
	'/'};

/* Package: com.sec.android.app.camera */
static const uint8_t camara_pkg_name[] = {
	'c', 'o', 'm', '.', 's', 'e', 'c', '.', 'a', 'n', 'd', 'r', 'o', 'i', 'd', '.', 'a', 'p', 'p', '.', 'c', 'a', 'm', 'e', 'r', 'a'
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

void display_rotary_led(int32_t rotary_val)
{
    if(rotary_val > 0)
        rotary_idx++;
    else if(rotary_val < 0)
        rotary_idx--;

    if(rotary_idx < 0)
        rotary_idx = MAX_ROTARY_IDX - 1;
    else if(rotary_idx > MAX_ROTARY_IDX - 1)
        rotary_idx = 0;

    led_on_idx(rotary_idx);
}

void sw_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("SW pressed\n");
    sw_led_flag = !sw_led_flag;
}

int main(void)
{
    struct sensor_value val;
    int rc;
    const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(qdec0));

	int nfc_err;
	size_t len = sizeof(ndef_msg_buf);

	printk("Starting NFC Launch app example\n");

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
	nfc_err = dk_leds_init();
	if (nfc_err) {
		printk("Cannot init LEDs!\n");
		goto fail;
	}

	/* Set up NFC */
	nfc_err = nfc_t2t_setup(nfc_callback, NULL);
	if (nfc_err) {
		printk("Cannot setup NFC T2T library!\n");
		goto fail;
	}

    int err = gpio_pin_configure_dt(&sw, GPIO_INPUT);
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

    led_on_idx(rotary_idx);

	/* Encode launch app data  */
	nfc_err = nfc_launchapp_msg_encode(camara_pkg_name,
				       sizeof(camara_pkg_name),
				       universal_link,
				       sizeof(universal_link),
				       ndef_msg_buf,
				       &len);
	if (nfc_err) {
		printk("Cannot encode message!\n");
		goto fail;
	}

	/* Set created message as the NFC payload */
	nfc_err = nfc_t2t_payload_set(ndef_msg_buf, len);
	if (nfc_err) {
		printk("Cannot set payload!\n");
		goto fail;
	}

	/* Start sensing NFC field */
	nfc_err = nfc_t2t_emulation_start();
	if (nfc_err) {
		printk("Cannot start emulation!\n");
		goto fail;
	}

	printk("NFC configuration done\n");

        while(1)
    {
        rc = sensor_sample_fetch(dev);
        rc = sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &val);

        if(!sw_led_flag)
            display_rotary_led(val.val1);
        else
            led_off_all();

        k_msleep(100);
    }
    
	return 0;

fail:
#if CONFIG_REBOOT
	sys_reboot(SYS_REBOOT_COLD);
#endif /* CONFIG_REBOOT */

	return nfc_err;
}

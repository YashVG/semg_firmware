/* main.c - sEMG BLE Peripheral */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

/* ── Custom sEMG Service ──────────────────────────────────────── */

/* 128-bit UUIDs with custom base (embeds 0x1800 / 0x1801 short IDs) */
#define BT_UUID_SEMG_SVC_VAL \
	BT_UUID_128_ENCODE(0x00001800, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define BT_UUID_SEMG_SVC  BT_UUID_DECLARE_128(BT_UUID_SEMG_SVC_VAL)

#define BT_UUID_SEMG_CHAR_VAL \
	BT_UUID_128_ENCODE(0x00001801, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define BT_UUID_SEMG_CHAR BT_UUID_DECLARE_128(BT_UUID_SEMG_CHAR_VAL)

/* sEMG packet layout:
 *   [0..1]   uint16  sequence number
 *   [2..5]   uint32  timestamp (ms since boot)
 *   [6]      uint8   sample count  (29)
 *   [7]      uint8   channel mask  (0x0F)
 *   [8..239] int16   sample data   (29 samples x 4 channels)
 */
#define SEMG_SAMPLE_COUNT   29
#define SEMG_CHANNEL_COUNT  4
#define SEMG_CHANNEL_MASK   0x0F
#define SEMG_HEADER_SIZE    8
#define SEMG_DATA_SIZE      (SEMG_SAMPLE_COUNT * SEMG_CHANNEL_COUNT * 2) /* 232 */
#define SEMG_PACKET_SIZE    (SEMG_HEADER_SIZE + SEMG_DATA_SIZE)          /* 240 */

static bool     semg_ntf_enabled;
static uint16_t semg_seq;

/* Double-buffered sample ring buffer */
static int16_t sample_bufs[2][SEMG_SAMPLE_COUNT * SEMG_CHANNEL_COUNT];
static uint8_t active_buf;   /* buffer index the timer ISR fills   */
static uint8_t ready_buf;    /* buffer index the work handler reads */
static uint8_t sample_idx;   /* next sample slot in active buffer   */
static int16_t fake_sample;  /* incrementing fake ADC value         */
static atomic_t send_pending; /* guards ready_buf during BLE send   */

static struct k_work  semg_send_work;
static struct k_timer sample_timer;

static void semg_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	semg_ntf_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("sEMG notifications %s\n", semg_ntf_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(semg_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SEMG_SVC),
	BT_GATT_CHARACTERISTIC(BT_UUID_SEMG_CHAR,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(semg_ccc_changed,
		     BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ── Advertising ──────────────────────────────────────────────── */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SEMG_SVC_VAL),
#if defined(CONFIG_BT_EXT_ADV)
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
#endif /* CONFIG_BT_EXT_ADV */
};

#if !defined(CONFIG_BT_EXT_ADV)
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
#endif /* !CONFIG_BT_EXT_ADV */

/* ── Connection handling ──────────────────────────────────────── */

enum {
	STATE_CONNECTED,
	STATE_DISCONNECTED,

	STATE_BITS,
};

static ATOMIC_DEFINE(state, STATE_BITS);

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");

		(void)atomic_set_bit(state, STATE_CONNECTED);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	(void)atomic_set_bit(state, STATE_DISCONNECTED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

/* ── Notifications ────────────────────────────────────────────── */

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

/* Timer ISR (2000 Hz): generates one fake sample per tick */
static void sample_timer_handler(struct k_timer *timer)
{
	int16_t *dst = &sample_bufs[active_buf][sample_idx * SEMG_CHANNEL_COUNT];

	for (int ch = 0; ch < SEMG_CHANNEL_COUNT; ch++) {
		dst[ch] = fake_sample++;
	}

	sample_idx++;

	if (sample_idx >= SEMG_SAMPLE_COUNT) {
		if (atomic_get(&send_pending)) {
			/* Work handler hasn't finished — drop this batch */
			sample_idx = 0;
			return;
		}
		ready_buf = active_buf;
		active_buf ^= 1;
		sample_idx = 0;
		atomic_set(&send_pending, 1);
		k_work_submit(&semg_send_work);
	}
}

/* Work handler: packs buffered samples into sEMG packet and sends via BLE */
static void semg_send_handler(struct k_work *work)
{
	static uint8_t buf[SEMG_PACKET_SIZE];
	static uint32_t last_send_ts;
	int16_t *src = sample_bufs[ready_buf];

	if (!semg_ntf_enabled) {
		atomic_set(&send_pending, 0);
		return;
	}

	uint32_t now = k_uptime_get_32();
	uint32_t delta = now - last_send_ts;

	sys_put_le16(semg_seq++, &buf[0]);
	sys_put_le32(now, &buf[2]);
	buf[6] = SEMG_SAMPLE_COUNT;
	buf[7] = SEMG_CHANNEL_MASK;

	for (int i = 0; i < SEMG_SAMPLE_COUNT * SEMG_CHANNEL_COUNT; i++) {
		sys_put_le16((uint16_t)src[i], &buf[SEMG_HEADER_SIZE + i * 2]);
	}

	bt_gatt_notify(NULL, &semg_svc.attrs[1], buf, sizeof(buf));

	/* Expected interval: 14 ms (29 samples / 2000 Hz = 14.5 ms) */
	if (last_send_ts != 0) {
		printk("sEMG pkt #%u  dt=%u ms (expect ~14)\n",
		       (unsigned int)(semg_seq - 1), delta);
	}
	last_send_ts = now;

	atomic_set(&send_pending, 0);
}

/* ── LED blink (optional) ─────────────────────────────────────── */

#if defined(CONFIG_GPIO)
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#include <zephyr/drivers/gpio.h>
#define HAS_LED     1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool                  led_is_on;

static void blink_timeout(struct k_work *work)
{
	led_is_on = !led_is_on;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);

	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static int blink_setup(void)
{
	int err;

	printk("Checking LED device...");
	if (!gpio_is_ready_dt(&led)) {
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	printk("Configuring GPIO pin...");
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err) {
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	k_work_init_delayable(&blink_work, blink_timeout);

	return 0;
}

static void blink_start(void)
{
	printk("Start blinking LED...\n");
	led_is_on = false;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static void blink_stop(void)
{
	struct k_work_sync work_sync;

	printk("Stop blinking LED.\n");
	k_work_cancel_delayable_sync(&blink_work, &work_sync);

	/* Keep LED on */
	led_is_on = true;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
}
#endif /* LED0_NODE */
#endif /* CONFIG_GPIO */

/* ── main ─────────────────────────────────────────────────────── */

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	bt_conn_auth_cb_register(&auth_cb_display);

	k_work_init(&semg_send_work, semg_send_handler);
	k_timer_init(&sample_timer, sample_timer_handler, NULL);
	k_timer_start(&sample_timer, K_USEC(500), K_USEC(500));

#if !defined(CONFIG_BT_EXT_ADV)
	printk("Starting Legacy Advertising (connectable and scannable)\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

#else /* CONFIG_BT_EXT_ADV */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U,
		.secondary_max_skip = 0U,
		.options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_CODED),
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};
	struct bt_le_ext_adv *adv;

	printk("Creating a Coded PHY connectable non-scannable advertising set\n");
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create Coded PHY extended advertising set (err %d)\n", err);

		printk("Creating a non-Coded PHY connectable non-scannable advertising set\n");
		adv_param.options &= ~BT_LE_ADV_OPT_CODED;
		err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
		if (err) {
			printk("Failed to create extended advertising set (err %d)\n", err);
			return 0;
		}
	}

	printk("Setting extended advertising data\n");
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set extended advertising data (err %d)\n", err);
		return 0;
	}

	printk("Starting Extended Advertising (connectable non-scannable)\n");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising set (err %d)\n", err);
		return 0;
	}
#endif /* CONFIG_BT_EXT_ADV */

	printk("Advertising successfully started\n");

#if defined(HAS_LED)
	err = blink_setup();
	if (err) {
		return 0;
	}

	blink_start();
#endif /* HAS_LED */

	while (1) {
		k_sleep(K_SECONDS(1));

		/* Battery level simulation */
		bas_notify();

		if (atomic_test_and_clear_bit(state, STATE_CONNECTED)) {
			/* Connected callback executed */

#if defined(HAS_LED)
			blink_stop();
#endif /* HAS_LED */
		} else if (atomic_test_and_clear_bit(state, STATE_DISCONNECTED)) {
#if !defined(CONFIG_BT_EXT_ADV)
			printk("Starting Legacy Advertising (connectable and scannable)\n");
			err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
					      ARRAY_SIZE(sd));
			if (err) {
				printk("Advertising failed to start (err %d)\n", err);
				return 0;
			}

#else /* CONFIG_BT_EXT_ADV */
			printk("Starting Extended Advertising (connectable and non-scannable)\n");
			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err) {
				printk("Failed to start extended advertising set (err %d)\n", err);
				return 0;
			}
#endif /* CONFIG_BT_EXT_ADV */

#if defined(HAS_LED)
			blink_start();
#endif /* HAS_LED */
		}
	}

	return 0;
}

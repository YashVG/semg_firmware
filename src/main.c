/* main.c - sEMG BLE Peripheral */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

#if defined(CONFIG_GPIO)
#include <zephyr/drivers/gpio.h>
#endif

/* ── Custom sEMG Service ──────────────────────────────────────── */

/* 128-bit UUIDs with custom base (embeds 0x1800 / 0x1801 short IDs) */
#define BT_UUID_SEMG_SVC_VAL \
	BT_UUID_128_ENCODE(0x00001800, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define BT_UUID_SEMG_SVC BT_UUID_DECLARE_128(BT_UUID_SEMG_SVC_VAL)

#define BT_UUID_SEMG_CHAR_VAL \
	BT_UUID_128_ENCODE(0x00001801, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define BT_UUID_SEMG_CHAR BT_UUID_DECLARE_128(BT_UUID_SEMG_CHAR_VAL)

/* sEMG packet layout:
 *   [0..1]   uint16  sequence number
 *   [2..5]   uint32  timestamp (ms since boot)
 *   [6]      uint8   sample count  (29)
 *   [7]      uint8   channel mask  (0x07)
 *   [8..181] int16   sample data   (29 samples x 3 channels)
 */
#define SEMG_SAMPLE_COUNT 29
#define SEMG_CHANNEL_COUNT 3
#define SEMG_CHANNEL_MASK 0x07
#define SEMG_HEADER_SIZE 8
#define SEMG_DATA_SIZE (SEMG_SAMPLE_COUNT * SEMG_CHANNEL_COUNT * 2) /* 174 */
#define SEMG_PACKET_SIZE (SEMG_HEADER_SIZE + SEMG_DATA_SIZE)		/* 182 */

#define SEMG_SAMPLE_RATE_HZ 2000
#define SIM_ENV_MAX 1024
#define SIM_REST_ENV 42
#define SIM_LIFT_ATTACK_STEP 1		/* ~1.0s from rest to full contraction */
#define SIM_FAST_LOWER_STEP 13		/* ~0.8s from full contraction to rest */
#define SIM_BRAKED_LOWER_STEP 5		/* ~2.0s from full contraction to rest */
#define SIM_TOP_HOLD_TICKS (SEMG_SAMPLE_RATE_HZ / 2)
#define SIM_FATIGUE_MAX 768
#define SIM_SIGNAL_GAIN_Q10 3072 /* 3.0x */
#define SIM_OUTPUT_LIMIT 22000
#define SIM_BUTTON_DEBOUNCE_MS 25
#define SEMG_PACKET_LOG_INTERVAL 64

static bool semg_ntf_enabled;
static uint16_t semg_seq;

/* Double-buffered sample ring buffer */
static int16_t sample_bufs[2][SEMG_SAMPLE_COUNT * SEMG_CHANNEL_COUNT];
static uint8_t active_buf;	  /* buffer index the timer ISR fills   */
static uint8_t ready_buf;	  /* buffer index the work handler reads */
static uint8_t sample_idx;	  /* next sample slot in active buffer   */
static atomic_t send_pending; /* guards ready_buf during BLE send   */

static struct k_work semg_send_work;
static struct k_timer sample_timer;

static atomic_t lift_button_pressed;
static atomic_t brake_button_pressed;

struct sim_state {
	uint16_t envelope;
	uint16_t hold_ticks;
	uint16_t fatigue;
	uint8_t fatigue_div;
	uint8_t recovery_div;
	uint32_t prng;
	uint16_t phase_low;
	uint16_t phase_mid;
	uint16_t phase_high;
	bool was_lifting;
};

static struct sim_state sim = {
	.envelope = SIM_REST_ENV,
	.prng = 0x1f2e3d4c,
};

static int16_t clamp_sim_sample(int32_t value)
{
	if (value > SIM_OUTPUT_LIMIT)
	{
		return SIM_OUTPUT_LIMIT;
	}
	if (value < -SIM_OUTPUT_LIMIT)
	{
		return -SIM_OUTPUT_LIMIT;
	}
	return (int16_t)value;
}

static int32_t tri_wave_q10(uint16_t phase)
{
	uint16_t folded = (phase & 0x8000) ? (uint16_t)(0xffff - phase) : phase;

	return ((int32_t)(folded >> 4)) - 1024;
}

static int32_t next_noise_q10(void)
{
	uint32_t x = sim.prng;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	sim.prng = x;

	return (int32_t)((x >> 22) & 0x3ff) - 512;
}

static void sim_update_movement(void)
{
	bool lifting = atomic_get(&lift_button_pressed) != 0;
	bool braking = atomic_get(&brake_button_pressed) != 0;

	if (lifting)
	{
		sim.was_lifting = true;

		if (sim.envelope < SIM_ENV_MAX)
		{
			uint16_t next = sim.envelope + SIM_LIFT_ATTACK_STEP;
			sim.envelope = (next > SIM_ENV_MAX) ? SIM_ENV_MAX : next;
		}

		if (sim.envelope >= (SIM_ENV_MAX - 12))
		{
			if (sim.hold_ticks < UINT16_MAX)
			{
				sim.hold_ticks++;
			}

			if (sim.hold_ticks > SIM_TOP_HOLD_TICKS)
			{
				sim.fatigue_div++;
				if (sim.fatigue_div >= 8)
				{
					sim.fatigue_div = 0;
					if (sim.fatigue < SIM_FATIGUE_MAX)
					{
						sim.fatigue++;
					}
				}
			}
		}
		else
		{
			sim.hold_ticks = 0;
			sim.fatigue_div = 0;
		}
	}
	else
	{
		uint16_t lower_step = braking ? SIM_BRAKED_LOWER_STEP : SIM_FAST_LOWER_STEP;

		sim.hold_ticks = 0;
		sim.fatigue_div = 0;

		if (sim.envelope > SIM_REST_ENV)
		{
			uint16_t next = sim.envelope - MIN(sim.envelope - SIM_REST_ENV, lower_step);
			sim.envelope = next;
		}
		else
		{
			sim.envelope = SIM_REST_ENV;
			sim.was_lifting = false;
		}

		if (sim.fatigue > 0)
		{
			sim.recovery_div++;
			if (sim.recovery_div >= 12)
			{
				sim.recovery_div = 0;
				sim.fatigue--;
			}
		}
	}

	sim.phase_low += 1278;  /* ~39 Hz at 2000 SPS */
	sim.phase_mid += 2687;  /* ~82 Hz at 2000 SPS */
	sim.phase_high += 4489; /* ~137 Hz at 2000 SPS */
}

static int16_t sim_generate_channel_sample(uint8_t channel)
{
	static const uint16_t channel_gain_q10[SEMG_CHANNEL_COUNT] = {1024, 740, 560};
	static const uint16_t channel_phase_offset[SEMG_CHANNEL_COUNT] = {0, 0x1555, 0x2aaa};
	uint16_t offset = channel_phase_offset[channel];
	int32_t fatigue = sim.fatigue;
	int32_t low = tri_wave_q10(sim.phase_low + offset);
	int32_t mid = tri_wave_q10(sim.phase_mid + (offset >> 1));
	int32_t high = tri_wave_q10(sim.phase_high + (offset << 1));
	int32_t noise = next_noise_q10();
	int32_t low_weight = 520 + (fatigue >> 2);
	int32_t mid_weight = 680;
	int32_t high_weight = 560 - (fatigue >> 1);
	int32_t mixed;
	int32_t scaled;

	if (high_weight < 150)
	{
		high_weight = 150;
	}

	mixed = (low * low_weight) + (mid * mid_weight) + (high * high_weight) + (noise * 220);
	mixed >>= 8;

	scaled = (mixed * (int32_t)sim.envelope) >> 10;
	scaled = (scaled * (int32_t)channel_gain_q10[channel]) >> 10;
	scaled = (scaled * SIM_SIGNAL_GAIN_Q10) >> 10;

	return clamp_sim_sample(scaled);
}

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
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

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

enum
{
	STATE_CONNECTED,
	STATE_DISCONNECTED,

	STATE_BITS,
};

static ATOMIC_DEFINE(state, STATE_BITS);

static struct bt_conn *current_conn;
static struct bt_gatt_exchange_params mtu_exchange_params;

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
							struct bt_gatt_exchange_params *params)
{
	if (err)
	{
		printk("MTU exchange failed (err %u)\n", err);
	}
	else
	{
		printk("MTU exchanged: %u\n", bt_gatt_get_mtu(conn));
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected\n");
	current_conn = bt_conn_ref(conn);
	(void)atomic_set_bit(state, STATE_CONNECTED);

	struct bt_le_conn_param param = {
		.interval_min = 6, /* 6 × 1.25ms = 7.5ms */
		.interval_max = 8, /* 8 × 1.25ms = 10ms  */
		.latency = 0,
		.timeout = 400, /* 4s supervision timeout */
	};

	int cerr = bt_conn_le_param_update(conn, &param);
	if (cerr)
	{
		printk("Connection param update failed (err %d)\n", cerr);
	}

	mtu_exchange_params.func = mtu_exchange_cb;
	int merr = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (merr)
	{
		printk("MTU exchange request failed (err %d)\n", merr);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

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

/* Timer ISR (2000 Hz): generates one simulated bicep sample per tick */
static void sample_timer_handler(struct k_timer *timer)
{
	int16_t *dst = &sample_bufs[active_buf][sample_idx * SEMG_CHANNEL_COUNT];

	sim_update_movement();

	for (int ch = 0; ch < SEMG_CHANNEL_COUNT; ch++)
	{
		dst[ch] = sim_generate_channel_sample((uint8_t)ch);
	}

	sample_idx++;

	if (sample_idx >= SEMG_SAMPLE_COUNT)
	{
		if (atomic_get(&send_pending))
		{
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

	if (!semg_ntf_enabled)
	{
		atomic_set(&send_pending, 0);
		return;
	}

	uint32_t now = k_uptime_get_32();
	uint32_t delta = now - last_send_ts;

	uint16_t packet_seq = semg_seq++;

	sys_put_le16(packet_seq, &buf[0]);
	sys_put_le32(now, &buf[2]);
	buf[6] = SEMG_SAMPLE_COUNT;
	buf[7] = SEMG_CHANNEL_MASK;

	for (int i = 0; i < SEMG_SAMPLE_COUNT * SEMG_CHANNEL_COUNT; i++)
	{
		sys_put_le16((uint16_t)src[i], &buf[SEMG_HEADER_SIZE + i * 2]);
	}

	if (current_conn)
	{
		int err = bt_gatt_notify(current_conn, &semg_svc.attrs[1], buf, sizeof(buf));
		if (err) {
			printk("bt_gatt_notify failed: %d\n", err);
		}
	}
	else
	{
		printk("current_conn is NULL — skipping notify\n");
	}

	/* Expected interval: 14 ms (29 samples / 2000 Hz = 14.5 ms) */
	if ((last_send_ts != 0) && ((packet_seq % SEMG_PACKET_LOG_INTERVAL) == 0))
	{
		printk("sEMG pkt #%u  dt=%u ms (expect ~14)\n",
			   (unsigned int)packet_seq, delta);
	}
	last_send_ts = now;

	atomic_set(&send_pending, 0);
}

/* ── Sim movement buttons ─────────────────────────────────────── */

#if defined(CONFIG_GPIO)
#define SW1_NODE DT_ALIAS(sw0)
#define SW2_NODE DT_ALIAS(sw1)

#if DT_NODE_HAS_STATUS_OKAY(SW1_NODE) && DT_NODE_HAS_STATUS_OKAY(SW2_NODE)
#define HAS_SIM_BUTTONS 1

static const struct gpio_dt_spec lift_button = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec brake_button = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static struct gpio_callback lift_button_cb;
static struct gpio_callback brake_button_cb;
static struct k_work sim_button_log_work;
static struct k_work_delayable sim_button_debounce_work;
static int lift_button_logged_state = -1;
static int brake_button_logged_state = -1;

static void sim_button_log_handler(struct k_work *work)
{
	(void)work;

	int lift = atomic_get(&lift_button_pressed) != 0;
	int brake = atomic_get(&brake_button_pressed) != 0;
	bool changed = false;

	if (lift != lift_button_logged_state)
	{
		printk("SIM BTN SW1 lift/curl %s  t=%u ms\n",
			   lift ? "pressed" : "released",
			   k_uptime_get_32());
		lift_button_logged_state = lift;
		changed = true;
	}

	if (brake != brake_button_logged_state)
	{
		printk("SIM BTN SW2 controlled-lowering %s  t=%u ms\n",
			   brake ? "pressed" : "released",
			   k_uptime_get_32());
		brake_button_logged_state = brake;
		changed = true;
	}

	if (changed)
	{
		printk("SIM BTN state: lift=%d brake=%d\n", lift, brake);
	}
}

static bool sim_buttons_update(void)
{
	int lift = gpio_pin_get_dt(&lift_button);
	int brake = gpio_pin_get_dt(&brake_button);
	bool changed = false;

	if (lift >= 0)
	{
		int pressed = lift ? 1 : 0;

		if (atomic_get(&lift_button_pressed) != pressed)
		{
			atomic_set(&lift_button_pressed, pressed);
			changed = true;
		}
	}
	if (brake >= 0)
	{
		int pressed = brake ? 1 : 0;

		if (atomic_get(&brake_button_pressed) != pressed)
		{
			atomic_set(&brake_button_pressed, pressed);
			changed = true;
		}
	}

	return changed;
}

static void sim_button_debounce_handler(struct k_work *work)
{
	(void)work;

	if (sim_buttons_update())
	{
		k_work_submit(&sim_button_log_work);
	}
}

static void sim_button_changed(const struct device *dev,
							   struct gpio_callback *cb,
							   uint32_t pins)
{
	(void)dev;
	(void)cb;
	(void)pins;

	k_work_reschedule(&sim_button_debounce_work, K_MSEC(SIM_BUTTON_DEBOUNCE_MS));
}

static int sim_buttons_setup(void)
{
	int err;

	if (!gpio_is_ready_dt(&lift_button) || !gpio_is_ready_dt(&brake_button))
	{
		printk("Simulation buttons are not ready\n");
		return -ENODEV;
	}

	k_work_init(&sim_button_log_work, sim_button_log_handler);
	k_work_init_delayable(&sim_button_debounce_work, sim_button_debounce_handler);

	err = gpio_pin_configure_dt(&lift_button, GPIO_INPUT);
	if (err)
	{
		printk("Failed to configure SW1 lift button (err %d)\n", err);
		return err;
	}

	err = gpio_pin_configure_dt(&brake_button, GPIO_INPUT);
	if (err)
	{
		printk("Failed to configure SW2 brake button (err %d)\n", err);
		return err;
	}

	gpio_init_callback(&lift_button_cb, sim_button_changed, BIT(lift_button.pin));
	err = gpio_add_callback(lift_button.port, &lift_button_cb);
	if (err)
	{
		printk("Failed to add SW1 lift callback (err %d)\n", err);
		return err;
	}

	gpio_init_callback(&brake_button_cb, sim_button_changed, BIT(brake_button.pin));
	err = gpio_add_callback(brake_button.port, &brake_button_cb);
	if (err)
	{
		printk("Failed to add SW2 brake callback (err %d)\n", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&lift_button, GPIO_INT_EDGE_BOTH);
	if (err)
	{
		printk("Failed to enable SW1 lift interrupt (err %d)\n", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&brake_button, GPIO_INT_EDGE_BOTH);
	if (err)
	{
		printk("Failed to enable SW2 brake interrupt (err %d)\n", err);
		return err;
	}

	sim_buttons_update();
	k_work_submit(&sim_button_log_work);
	printk("Simulation controls ready: SW1 lift/curl, SW2 controlled lowering\n");

	return 0;
}
#endif /* SW1_NODE && SW2_NODE */
#endif /* CONFIG_GPIO */

/* ── LED blink (optional) ─────────────────────────────────────── */

#if defined(CONFIG_GPIO)
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool led_is_on;

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
	if (!gpio_is_ready_dt(&led))
	{
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	printk("Configuring GPIO pin...");
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err)
	{
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
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	bt_conn_auth_cb_register(&auth_cb_display);

	k_work_init(&semg_send_work, semg_send_handler);

#if defined(HAS_SIM_BUTTONS)
	err = sim_buttons_setup();
	if (err)
	{
		printk("Simulation buttons disabled (err %d)\n", err);
	}
#else
	printk("Simulation buttons unavailable; streaming rest-noise simulation only\n");
#endif /* HAS_SIM_BUTTONS */

	k_timer_init(&sample_timer, sample_timer_handler, NULL);
	k_timer_start(&sample_timer, K_USEC(500), K_USEC(500));

#if !defined(CONFIG_BT_EXT_ADV)
	printk("Starting Legacy Advertising (connectable and scannable)\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

#else  /* CONFIG_BT_EXT_ADV */
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
	if (err)
	{
		printk("Failed to create Coded PHY extended advertising set (err %d)\n", err);

		printk("Creating a non-Coded PHY connectable non-scannable advertising set\n");
		adv_param.options &= ~BT_LE_ADV_OPT_CODED;
		err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
		if (err)
		{
			printk("Failed to create extended advertising set (err %d)\n", err);
			return 0;
		}
	}

	printk("Setting extended advertising data\n");
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		printk("Failed to set extended advertising data (err %d)\n", err);
		return 0;
	}

	printk("Starting Extended Advertising (connectable non-scannable)\n");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err)
	{
		printk("Failed to start extended advertising set (err %d)\n", err);
		return 0;
	}
#endif /* CONFIG_BT_EXT_ADV */

	printk("Advertising successfully started\n");

#if defined(HAS_LED)
	err = blink_setup();
	if (err)
	{
		return 0;
	}

	blink_start();
#endif /* HAS_LED */

	while (1)
	{
		k_sleep(K_SECONDS(1));

		if (atomic_test_and_clear_bit(state, STATE_CONNECTED))
		{
			/* Connected callback executed */

#if defined(HAS_LED)
			blink_stop();
#endif /* HAS_LED */
		}
		else if (atomic_test_and_clear_bit(state, STATE_DISCONNECTED))
		{
#if !defined(CONFIG_BT_EXT_ADV)
			printk("Starting Legacy Advertising (connectable and scannable)\n");
			err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
								  ARRAY_SIZE(sd));
			if (err)
			{
				printk("Advertising failed to start (err %d)\n", err);
				return 0;
			}

#else  /* CONFIG_BT_EXT_ADV */
			printk("Starting Extended Advertising (connectable and non-scannable)\n");
			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err)
			{
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

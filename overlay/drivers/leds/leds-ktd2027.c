/*
 *
 * Author: Petr Kalandra <kalandrap@gmail.com>
 *
 * Based on leds-pca955x.c
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>

#define NUM_LEDS          4
#define MAX_CURRENT       191
#define REG_CHANNEL_CTRL  0x04
#define REG_LED_IOUT(x)  (0x06 + x)

static const struct i2c_device_id ktd2027_id[] = {
	{ "ktd2027", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ktd2027_id);

struct ktd2027;

struct ktd2027_led {
	struct ktd2027      *chip;
	struct led_classdev  led_cdev;
	int                  led_num;
	char                 name[32];
};

struct ktd2027 {
	struct mutex       mutex;
	struct i2c_client *client;
	struct ktd2027_led leds[NUM_LEDS];
	u8                 leds_on;
};

static int ktd2027_brightness(struct ktd2027_led *ktd2027,
			       enum led_brightness brightness)
{
	return i2c_smbus_write_byte_data(
		ktd2027->chip->client,
		REG_LED_IOUT(ktd2027->led_num),
		(u8)brightness);
}

static int ktd2027_power_state(struct ktd2027_led *ktd2027)
{
	u8 leds_on = ktd2027->chip->leds_on;
	int ret;

	if (ktd2027->led_cdev.brightness)
		leds_on |=   1 << (2 * ktd2027->led_num);
	else
		leds_on &= ~(1 << (2 * ktd2027->led_num));

	ret = i2c_smbus_write_byte_data(ktd2027->chip->client,
			REG_CHANNEL_CTRL, leds_on);
	if (ret < 0)
		return ret;

	ktd2027->chip->leds_on = leds_on;
	return ret;
}

static int ktd2027_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct ktd2027_led *ktd2027;
	int ret;

	ktd2027 = container_of(led_cdev, struct ktd2027_led, led_cdev);

	mutex_lock(&ktd2027->chip->mutex);

	ret = ktd2027_brightness(ktd2027, value);
	if (ret < 0)
		goto unlock;
	ret = ktd2027_power_state(ktd2027);

unlock:
	mutex_unlock(&ktd2027->chip->mutex);
	return ret;
}

static struct led_platform_data *
ktd2027_dt_init(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node, *child;
	struct led_platform_data *pdata;
	struct led_info *ktd2027_leds;
	int count;

	count = of_get_child_count(np);
	if (!count || count > NUM_LEDS)
		return ERR_PTR(-ENODEV);

	ktd2027_leds = devm_kzalloc(&client->dev,
			sizeof(struct led_info) * NUM_LEDS, GFP_KERNEL);
	if (!ktd2027_leds)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		struct led_info led = {};
		u32 reg;
		int res;

		res = of_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= NUM_LEDS))
			continue;
		led.name =
			of_get_property(child, "label", NULL) ? : child->name;
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		ktd2027_leds[reg] = led;
	}
	pdata = devm_kzalloc(&client->dev,
			     sizeof(struct led_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->num_leds = NUM_LEDS;
	pdata->leds = ktd2027_leds;

	return pdata;
}

static const struct of_device_id of_ktd2027_match[] = {
	{ .compatible = "kinetic,ktd2027", },
	{},
};
MODULE_DEVICE_TABLE(of, of_ktd2027_match);

static int ktd2027_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct ktd2027 *ktd2027_chip;
	struct led_platform_data *pdata;
	int i, err;

	pdata = dev_get_platdata(&client->dev);

	if (!pdata) {
		pdata = ktd2027_dt_init(client);
		if (IS_ERR(pdata)) {
			dev_warn(&client->dev, "could not parse configuration\n");
			pdata = NULL;
		}
	}

	ktd2027_chip = devm_kzalloc(&client->dev, sizeof(*ktd2027_chip), GFP_KERNEL);
	if (!ktd2027_chip)
		return -ENOMEM;

	i2c_set_clientdata(client, ktd2027_chip);

	mutex_init(&ktd2027_chip->mutex);
	ktd2027_chip->client  = client;
	ktd2027_chip->leds_on = 0;

	/* Turn off LEDs by default*/
	i2c_smbus_write_byte_data(client, REG_CHANNEL_CTRL, 0x00);

	for (i = 0; i < NUM_LEDS; i++) {
		ktd2027_chip->leds[i].led_num = i;
		ktd2027_chip->leds[i].chip    = ktd2027_chip;

		/* Platform data can specify LED names and default triggers */
		if (pdata) {
			if (pdata->leds[i].name)
				snprintf(ktd2027_chip->leds[i].name,
					 sizeof(ktd2027_chip->leds[i].name), "ktd2027:%s",
					 pdata->leds[i].name);
			if (pdata->leds[i].default_trigger)
				ktd2027_chip->leds[i].led_cdev.default_trigger =
					pdata->leds[i].default_trigger;
		}

		if (!pdata || !pdata->leds[i].name)
			snprintf(ktd2027_chip->leds[i].name, sizeof(ktd2027_chip->leds[i].name),
				 "ktd2027:%d:%.2x:%d", client->adapter->nr,
				 client->addr, i);

		ktd2027_chip->leds[i].led_cdev.name = ktd2027_chip->leds[i].name;
		ktd2027_chip->leds[i].led_cdev.brightness_set_blocking = ktd2027_led_set;
		ktd2027_chip->leds[i].led_cdev.max_brightness = MAX_CURRENT;

		err = led_classdev_register(&client->dev, &ktd2027_chip->leds[i].led_cdev);
		if (err < 0)
			goto exit;
	}

	return 0;

exit:
	while (i--)
		led_classdev_unregister(&ktd2027_chip->leds[i].led_cdev);

	return err;
}

static int ktd2027_remove(struct i2c_client *client)
{
	struct ktd2027 *ktd2027 = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < NUM_LEDS; i++)
		led_classdev_unregister(&ktd2027->leds[i].led_cdev);

	return 0;
}

static struct i2c_driver ktd2027_driver = {
	.driver = {
		.name	= "leds-ktd2027",
		.of_match_table = of_match_ptr(of_ktd2027_match),
	},
	.probe	= ktd2027_probe,
	.remove	= ktd2027_remove,
	.id_table = ktd2027_id,
};

module_i2c_driver(ktd2027_driver);

MODULE_AUTHOR("Petr Kalandra <kalandrap@gmail.com>");
MODULE_DESCRIPTION("KTD2027 LED driver");
MODULE_LICENSE("GPL v2");

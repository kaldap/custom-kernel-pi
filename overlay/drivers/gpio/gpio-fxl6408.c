/*
 *  FXL6408 8bit I/O expander
 *
 *  Copyright (C) 2019 Petr Kalandra <kalandrap@gmail.com>
 *
 *  Derived from drivers/i2c/chips/pca953x.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_data/fxl6408.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <asm/unaligned.h>

#define REG_CONTROL    0x01
#define REG_DIRECTION  0x03
#define REG_OUTPUT     0x05
#define REG_OUTPUT_HIZ 0x07
#define REG_DEFAULT    0x09
#define REG_PULL_EN    0x0B
#define REG_PULL_SEL   0x0D
#define REG_INPUT      0x0F
#define REG_IRQ_MASK   0x11
#define REG_IRQ_STATUS 0x13

static const struct i2c_device_id fxl6408_id[] = {
	{ "fxl6408", 8 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fxl6408_id);

struct fxl6408_chip {
	unsigned gpio_start;
	u8 reg_output[1];
	u8 reg_direction[1];
	u8 reg_pull_en[1];
	u8 reg_pull_sel[1];
	struct mutex i2c_lock;

#ifdef CONFIG_GPIO_FXL6408_IRQ
	struct mutex irq_lock;
	u8 irq_mask[1];
	u8 irq_defv[1];
	u8 irq_both[1];
#endif

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	const char *const *names;
	unsigned long driver_data;
	struct regulator *regulator;

	int (*write_regs)(struct fxl6408_chip *, int, u8 *);
	int (*read_regs)(struct fxl6408_chip *, int, u8 *);
};

static int fxl6408_read_single(struct fxl6408_chip *chip, int reg, u32 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	*val = ret;

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	return 0;
}

static int fxl6408_write_single(struct fxl6408_chip *chip, int reg, u32 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int fxl6408_write_regs_8(struct fxl6408_chip *chip, int reg, u8 *val)
{
	return i2c_smbus_write_byte_data(chip->client, reg, *val);
}

static int fxl6408_write_regs(struct fxl6408_chip *chip, int reg, u8 *val)
{
	int ret = 0;

	ret = chip->write_regs(chip, reg, val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int fxl6408_read_regs_8(struct fxl6408_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	*val = ret;

	return ret;
}

static int fxl6408_read_regs(struct fxl6408_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = chip->read_regs(chip, reg, val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	return 0;
}

static int fxl6408_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	reg_val = chip->reg_direction[0] & ~(1u << off);

	ret = fxl6408_write_single(chip, REG_DIRECTION, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction[0] = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int fxl6408_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* set output level */
	if (val)
		reg_val = chip->reg_output[0] | (1u << off);
	else
		reg_val = chip->reg_output[0] & ~(1u << off);

	ret = fxl6408_write_single(chip, REG_OUTPUT, reg_val);
	if (ret)
		goto exit;

	chip->reg_output[0] = reg_val;

	/* then direction */
	reg_val = chip->reg_direction[0] | (1u << off);
	ret = fxl6408_write_single(chip, REG_DIRECTION, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction[0] = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int fxl6408_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u32 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = fxl6408_read_single(chip, REG_INPUT, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void fxl6408_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	if (val)
		reg_val = chip->reg_output[0] |  (1u << off);
	else
		reg_val = chip->reg_output[0] & ~(1u << off);

	ret = fxl6408_write_single(chip, REG_OUTPUT, reg_val);
	if (ret)
		goto exit;

	chip->reg_output[0] = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
}

static int fxl6408_gpio_get_direction(struct gpio_chip *gc, unsigned off)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u32 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = fxl6408_read_single(chip, REG_DIRECTION, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0)
		return ret;

	return !(reg_val & (1u << off));
}

static void fxl6408_gpio_set_multiple(struct gpio_chip *gc,
				      unsigned long *mask, unsigned long *bits)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 reg_val[1];
	int ret;

	mutex_lock(&chip->i2c_lock);
	memcpy(reg_val, chip->reg_output, 1);
	if (mask[0]) {
		reg_val[0] = (reg_val[0] & ~mask[0]) | (bits[0] & mask[0]);
	}

	ret = i2c_smbus_write_i2c_block_data(chip->client, REG_OUTPUT, 1, reg_val);
	if (ret)
		goto exit;

	memcpy(chip->reg_output, reg_val, 1);
exit:
	mutex_unlock(&chip->i2c_lock);
}

static int fxl6408_gpio_set_config(struct gpio_chip *gc, unsigned offset,
				  unsigned long config)
{
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 mask = 1u << offset;
	int ret;

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_PULL_UP:
		chip->reg_pull_en[0]  |= mask;
		chip->reg_pull_sel[0] |= mask;
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		chip->reg_pull_en[0]  |=  mask;
		chip->reg_pull_sel[0] &= ~mask;
		break;

	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
		chip->reg_pull_en[0]  &= ~mask;
		break;

	default:
		return -ENOTSUPP;
	}

	mutex_lock(&chip->i2c_lock);
	ret = fxl6408_write_single(chip, REG_PULL_SEL, chip->reg_pull_sel[0]);
	if (ret)
		goto exit;

	ret = fxl6408_write_single(chip, REG_PULL_EN, chip->reg_pull_en[0]);

exit:
	mutex_unlock(&chip->i2c_lock);

	return ret;
}

static void fxl6408_setup_gpio(struct fxl6408_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = fxl6408_gpio_direction_input;
	gc->direction_output = fxl6408_gpio_direction_output;
	gc->get = fxl6408_gpio_get_value;
	gc->set = fxl6408_gpio_set_value;
	gc->get_direction = fxl6408_gpio_get_direction;
	gc->set_multiple = fxl6408_gpio_set_multiple;
	gc->set_config = fxl6408_gpio_set_config;
	gc->can_sleep = true;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->parent = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

#ifdef CONFIG_GPIO_FXL6408_IRQ
static void fxl6408_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	chip->irq_mask[0] |= (1 << d->hwirq);
	fxl6408_write_regs(chip, REG_IRQ_MASK, chip->irq_mask);
}

static void fxl6408_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	chip->irq_mask[0] &= ~(1 << d->hwirq);
	fxl6408_write_regs(chip, REG_IRQ_MASK, chip->irq_mask);
}

static void fxl6408_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void fxl6408_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);

	mutex_unlock(&chip->irq_lock);
}

static int fxl6408_irq_update_both_def_states(struct fxl6408_chip *chip)
{
	u8 val[1];
	int ret;

	ret = fxl6408_read_regs(chip, REG_INPUT, val);
	if (ret)
		return ret;

	chip->irq_defv[0] = (val[0] & chip->irq_both[0]) | (chip->irq_defv[0] & ~chip->irq_both[0]);
	ret = fxl6408_write_regs(chip, REG_DEFAULT, chip->irq_defv);
	return ret;
}

static int fxl6408_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct fxl6408_chip *chip = gpiochip_get_data(gc);
	u8 mask = 1 << d->hwirq;
	int ret;

	if (type > IRQ_TYPE_EDGE_BOTH) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n", d->irq, type);
		return -EINVAL;
	}

	chip->irq_both[0] &= ~mask;
	if (type == IRQ_TYPE_EDGE_BOTH) {
		chip->irq_mask[0] &= ~mask;
		chip->irq_both[0] |=  mask;
		ret = fxl6408_irq_update_both_def_states(chip);
		if (ret)
			return ret;
	}
	else if (type & IRQ_TYPE_EDGE_FALLING) {
		chip->irq_mask[0] &= ~mask;
		chip->irq_defv[0] |=  mask;
	}
	else if (type & IRQ_TYPE_EDGE_RISING) {
		chip->irq_mask[0] &= ~mask;
		chip->irq_defv[0] &= ~mask;
	}
	else {
		chip->irq_mask[0] |=  mask;
	}

	/* Change the default level first ... */
	ret = fxl6408_write_regs(chip, REG_DEFAULT, chip->irq_defv);
	if (ret)
		return ret;

	/* ... then (un)mask the IRQ */
	ret = fxl6408_write_regs(chip, REG_IRQ_MASK, chip->irq_mask);
	if (ret)
		return ret;

	return 0;
}

static struct irq_chip fxl6408_irq_chip = {
	.name			= "fxl6408",
	.irq_mask		= fxl6408_irq_mask,
	.irq_unmask		= fxl6408_irq_unmask,
	.irq_bus_lock		= fxl6408_irq_bus_lock,
	.irq_bus_sync_unlock	= fxl6408_irq_bus_sync_unlock,
	.irq_set_type		= fxl6408_irq_set_type,
};

static bool fxl6408_irq_pending(struct fxl6408_chip *chip, u8 *pending)
{
	u8 cur_stat[1];
	u8 zero = 0;
	int ret;

	/* Read and clear the IRQ status register */
	ret = fxl6408_read_regs(chip, REG_IRQ_STATUS, cur_stat);
	if (ret)
		return ret;

	/* If some pins requested interrupt on both edges, update default states */
	if (chip->irq_both[0]) {
		ret = fxl6408_irq_update_both_def_states(chip);
		if (ret)
			return ret;
	}

	/* Clear the irq flags */
        ret = fxl6408_write_regs(chip, REG_IRQ_STATUS, &zero);
        if (ret)
                return ret;

	/* Remove output and masked pins from the equation */
	cur_stat[0] &= ~(chip->reg_direction[0] | chip->irq_mask[0]);
	if (!cur_stat[0])
		return false;

	pending[0] = cur_stat[0];
	return true;
}

static irqreturn_t fxl6408_irq_handler(int irq, void *devid)
{
	struct fxl6408_chip *chip = devid;
	u8 pending[0];
	u8 level;
	unsigned nhandled = 0;

	if (!fxl6408_irq_pending(chip, pending))
		return IRQ_NONE;

	while (pending[0]) {
			level = __ffs(pending[0]);
			handle_nested_irq(irq_find_mapping(chip->gpio_chip.irqdomain, level));
			pending[0] &= ~(1 << level);
			nhandled++;
	}

	return (nhandled > 0) ? IRQ_HANDLED : IRQ_NONE;
}

static int fxl6408_irq_setup(struct fxl6408_chip *chip,
			     int irq_base)
{
	struct i2c_client *client = chip->client;
	int ret;

	if (client->irq && irq_base != -1) {
		ret = fxl6408_read_regs(chip, REG_DEFAULT, chip->irq_defv);
		if (ret)
			return ret;

		ret = fxl6408_read_regs(chip, REG_IRQ_MASK, chip->irq_mask);
		if (ret)
			return ret;

		mutex_init(&chip->irq_lock);

		ret = devm_request_threaded_irq(&client->dev,
						 client->irq,
					   NULL,
					   fxl6408_irq_handler,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
					   dev_name(&client->dev), chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n", client->irq);
			return ret;
		}

		ret =  gpiochip_irqchip_add_nested(&chip->gpio_chip,
						   &fxl6408_irq_chip,
						   irq_base,
						   handle_simple_irq,
						   IRQ_TYPE_NONE);
		if (ret) {
			dev_err(&client->dev,
				"could not connect irqchip to gpiochip\n");
			return ret;
		}

		gpiochip_set_nested_irqchip(&chip->gpio_chip,
					    &fxl6408_irq_chip,
					    client->irq);
	}

	return 0;
}

#else /* CONFIG_GPIO_FXL6408_IRQ */
static int fxl6408_irq_setup(struct fxl6408_chip *chip,
			     int irq_base)
{
	struct i2c_client *client = chip->client;

	if (irq_base != -1)
		dev_warn(&client->dev, "interrupt support not compiled in\n");

	return 0;
}
#endif /* CONFIG_GPIO_FXL6408_IRQ */

static int device_fxl6408_init(struct i2c_client *client, struct fxl6408_chip *chip)
{
	int ret;
	u8 val[1] = { 0 };

	ret = fxl6408_read_regs(chip, REG_CONTROL, val);
	if (ret)
		goto out;

	dev_info(&client->dev, "found device by %d rev %d\n", val[0] >> 5, (val[0] >> 2) & 7);

	ret = fxl6408_read_regs(chip, REG_OUTPUT, chip->reg_output);
	if (ret)
		goto out;

	ret = fxl6408_read_regs(chip, REG_DIRECTION, chip->reg_direction);
	if (ret)
		goto out;

	ret = fxl6408_read_regs(chip, REG_PULL_EN, chip->reg_pull_en);
	if (ret)
		goto out;

	ret = fxl6408_read_regs(chip, REG_PULL_SEL, chip->reg_pull_sel);
	if (ret)
		goto out;

	// HiZ outputs are not supported, turn them off
	ret = fxl6408_write_regs(chip, REG_OUTPUT_HIZ, val);
out:
	return ret;
}

static const struct of_device_id fxl6408_dt_ids[];

static int fxl6408_probe(struct i2c_client *client,
				   const struct i2c_device_id *i2c_id)
{
	struct fxl6408_platform_data *pdata;
	struct fxl6408_chip *chip;
	int irq_base = 0;
	int ret;
	struct regulator *reg;

	chip = devm_kzalloc(&client->dev,
			sizeof(struct fxl6408_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	pdata = dev_get_platdata(&client->dev);
	if (pdata) {
		irq_base = pdata->irq_base;
		chip->gpio_start = pdata->gpio_base;
		chip->names = pdata->names;
	} else {
		struct gpio_desc *reset_gpio;

		chip->gpio_start = -1;
		irq_base = 0;

		/*
		 * See if we need to de-assert a reset pin.
		 *
		 * There is no known ACPI-enabled platforms that are
		 * using "reset" GPIO. Otherwise any of those platform
		 * must use _DSD method with corresponding property.
		 */
		reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						     GPIOD_OUT_LOW);
		if (IS_ERR(reset_gpio))
			return PTR_ERR(reset_gpio);
	}

	chip->client = client;

	reg = devm_regulator_get(&client->dev, "vcc");
	if (IS_ERR(reg)) {
		ret = PTR_ERR(reg);
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev, "reg get err: %d\n", ret);
		return ret;
	}
	ret = regulator_enable(reg);
	if (ret) {
		dev_err(&client->dev, "reg en err: %d\n", ret);
		return ret;
	}
	chip->regulator = reg;

	if (i2c_id) {
		chip->driver_data = i2c_id->driver_data;
	} else {
		ret = -ENODEV;
		goto err_exit;
	}

	mutex_init(&chip->i2c_lock);
	/*
	 * In case we have an i2c-mux controlled by a GPIO provided by an
	 * expander using the same driver higher on the device tree, read the
	 * i2c adapter nesting depth and use the retrieved value as lockdep
	 * subclass for chip->i2c_lock.
	 *
	 * REVISIT: This solution is not complete. It protects us from lockdep
	 * false positives when the expander controlling the i2c-mux is on
	 * a different level on the device tree, but not when it's on the same
	 * level on a different branch (in which case the subclass number
	 * would be the same).
	 *
	 * TODO: Once a correct solution is developed, a similar fix should be
	 * applied to all other i2c-controlled GPIO expanders (and potentially
	 * regmap-i2c).
	 */
	lockdep_set_subclass(&chip->i2c_lock,
			     i2c_adapter_depth(client->adapter));

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	fxl6408_setup_gpio(chip, chip->driver_data);

	if (chip->gpio_chip.ngpio <= 8) {
		chip->write_regs = fxl6408_write_regs_8;
		chip->read_regs = fxl6408_read_regs_8;
	} else {
	  dev_warn(&client->dev, "%d gpios not supported\n", chip->gpio_chip.ngpio);
	  ret = -EINVAL;
		goto err_exit;
	}

	ret = device_fxl6408_init(client, chip);
	if (ret)
		goto err_exit;

	ret = devm_gpiochip_add_data(&client->dev, &chip->gpio_chip, chip);
	if (ret)
		goto err_exit;

	ret = fxl6408_irq_setup(chip, irq_base);
	if (ret)
		goto err_exit;

	if (pdata && pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);
	return 0;

err_exit:
	regulator_disable(chip->regulator);
	return ret;
}

static int fxl6408_remove(struct i2c_client *client)
{
	struct fxl6408_platform_data *pdata = dev_get_platdata(&client->dev);
	struct fxl6408_chip *chip = i2c_get_clientdata(client);
	int ret;

	if (pdata && pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
	} else {
		ret = 0;
	}

	regulator_disable(chip->regulator);

	return ret;
}

static const struct of_device_id fxl6408_dt_ids[] = {
	{ .compatible = "onsemi,fxl6408", .data = (void*)8, },
	{ }
};

MODULE_DEVICE_TABLE(of, fxl6408_dt_ids);

static struct i2c_driver fxl6408_driver = {
	.driver = {
		.name	= "fxl6408",
		.of_match_table = fxl6408_dt_ids,
	},
	.probe		= fxl6408_probe,
	.remove		= fxl6408_remove,
	.id_table	= fxl6408_id,
};

static int __init fxl6408_init(void)
{
	return i2c_add_driver(&fxl6408_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(fxl6408_init);

static void __exit fxl6408_exit(void)
{
	i2c_del_driver(&fxl6408_driver);
}
module_exit(fxl6408_exit);

MODULE_AUTHOR("Petr Kalandra <kalandrap@gmail.com>");
MODULE_DESCRIPTION("GPIO expander driver for FXL6408");
MODULE_LICENSE("GPL");

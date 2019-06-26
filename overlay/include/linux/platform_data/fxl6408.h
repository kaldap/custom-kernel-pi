/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_FXL6408_H
#define _LINUX_FXL6408_H

#include <linux/types.h>
#include <linux/i2c.h>

/* platform data for the FXL6408 8-bit I/O expander driver */

struct fxl6408_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	u32		invert;

	/* interrupt base */
	int		irq_base;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	const char	*const *names;
};

#endif /* _LINUX_FXL6408_H */

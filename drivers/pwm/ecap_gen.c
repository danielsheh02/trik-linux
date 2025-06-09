/*
 * eCAP driver for PWM output generation
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/pwm/pwm.h>
#include <linux/module.h>

#define TIMER_CTR_REG 0x0

#define ECTRL2_SYNCOSEL_MASK (0x03 << 6)

#define ECTRL2_MDSL_ECAP BIT(9)
#define ECTRL2_CTRSTP_FREERUN BIT(4)
#define ECTRL2_PLSL_LOW BIT(10)
#define ECTRL2_SYNC_EN BIT(5)

#define CLK_DISABLE 0
#define CLK_ENABLE 1

static int ecap_pwm_stop(struct pwm_device *p) {
	unsigned long flags;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	if (ep->clk_enabled == CLK_DISABLE)
		return 0;

	spin_lock_irqsave(&ep->lock, flags);
	__raw_writew(__raw_readw(ep->mmio_base + ECCTL2) & ~BIT(4),
							 ep->mmio_base + ECCTL2);
	spin_unlock_irqrestore(&ep->lock, flags);

	ep->clk_enabled = CLK_DISABLE;
	clk_disable(ep->clk);
	clear_bit(FLAG_RUNNING, &p->flags);

	return 0;
}

static int ecap_pwm_start(struct pwm_device *p) {
	int ret = 0;
	unsigned long flags;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	if (ep->clk_enabled == CLK_ENABLE)
		return 0;

	clk_enable(ep->clk);
	ep->clk_enabled = CLK_ENABLE;
	spin_lock_irqsave(&ep->lock, flags);
	__raw_writew(__raw_readw(ep->mmio_base + ECCTL2) | BIT(4),
							 ep->mmio_base + ECCTL2);
	spin_unlock_irqrestore(&ep->lock, flags);
	set_bit(FLAG_RUNNING, &p->flags);

	return ret;
}

static int ecap_pwm_set_polarity(struct pwm_device *p, char pol) {
	unsigned long flags;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	clk_enable(ep->clk);

	spin_lock_irqsave(&ep->lock, flags);
	__raw_writew((__raw_readw(ep->mmio_base + ECCTL2) & ~BIT(10)) |
									 (!pol << 10),
							 ep->mmio_base + ECCTL2);
	spin_unlock_irqrestore(&ep->lock, flags);

	clk_disable(ep->clk);
	return 0;
}

static int ecap_pwm_config_period(struct pwm_device *p) {
	unsigned long flags;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	clk_enable(ep->clk);

	spin_lock_irqsave(&ep->lock, flags);
	__raw_writel((p->period_ticks) - 1, ep->mmio_base + CAP3);
	spin_unlock_irqrestore(&ep->lock, flags);

	clk_disable(ep->clk);
	return 0;
}

static int ecap_pwm_config_duty(struct pwm_device *p) {
	unsigned long flags;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	clk_enable(ep->clk);

	spin_lock_irqsave(&ep->lock, flags);
	if (p->duty_ticks > 0) {
		__raw_writel(p->duty_ticks, ep->mmio_base + CAP4);
	} else {
		__raw_writel(p->duty_ticks, ep->mmio_base + CAP2);
		__raw_writel(0, ep->mmio_base + TIMER_CTR_REG);
	}
	spin_unlock_irqrestore(&ep->lock, flags);

	clk_disable(ep->clk);
	return 0;
}

int init_ecap_gen(struct pwm_device *p) {
	unsigned long flags;
	struct ecap_pwm *ep = to_ecap_pwm(p);

	clk_enable(ep->clk);

	spin_lock_irqsave(&ep->lock, flags);
	__raw_writew(ECTRL2_MDSL_ECAP | ECTRL2_SYNCOSEL_MASK | ECTRL2_CTRSTP_FREERUN,
							 ep->mmio_base + ECCTL2);
	spin_unlock_irqrestore(&ep->lock, flags);

	clk_disable(ep->clk);
	return 0;
}
EXPORT_SYMBOL(init_ecap_gen);

int ecap_gen_config(struct pwm_device *p, struct pwm_config *c) {
	int ret = 0;
	switch (c->config_mask) {

	case BIT(PWM_CONFIG_DUTY_TICKS):
		p->duty_ticks = c->duty_ticks;
		ret = ecap_pwm_config_duty(p);
		break;

	case BIT(PWM_CONFIG_PERIOD_TICKS):
		p->period_ticks = c->period_ticks;
		ret = ecap_pwm_config_period(p);
		break;

	case BIT(PWM_CONFIG_POLARITY):
		ret = ecap_pwm_set_polarity(p, c->polarity);
		break;

	case BIT(PWM_CONFIG_START):
		ret = ecap_pwm_start(p);
		break;

	case BIT(PWM_CONFIG_STOP):
		ret = ecap_pwm_stop(p);
		break;
	}

	return ret;
}
EXPORT_SYMBOL(ecap_gen_config);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Driver for Davinci eCAP peripheral");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ecap");
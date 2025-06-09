/*
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#ifndef __LINUX_GENERATOR_H
#define __LINUX_GENERATOR_H

#include <linux/pwm/pwm.h>

int init_ecap_gen(struct pwm_device *p);
int ecap_gen_config(struct pwm_device *p, struct pwm_config *c);
#endif
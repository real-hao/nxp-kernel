/* SPDX-License-Identifier: (GPL-2.0 OR MIT)
 *
 * TSN support for Felix VSC9959 through tsntool
 *
 * Copyright 2020-2023 NXP
 */

#ifndef _MSCC_FELIX_SWITCH_TSN_H_
#define _MSCC_FELIX_SWITCH_TSN_H_

#include <soc/mscc/ocelot.h>
#include <net/dsa.h>

#if IS_ENABLED(CONFIG_TSN)

void felix_cbs_reset(struct ocelot *ocelot, int port, u32 speed);
void felix_tsn_enable(struct dsa_switch *ds);

#else

static inline void felix_cbs_reset(struct ocelot *ocelot, int port,
				   u32 speed)
{
}

static inline void felix_tsn_enable(struct dsa_switch *ds)
{
}

#endif

#endif
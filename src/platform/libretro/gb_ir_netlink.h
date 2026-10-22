/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef GB_IR_NETLINK_H
#define GB_IR_NETLINK_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/timing.h>
#include <mgba/gb/interface.h>

#include "libretro.h"

/* A Game Boy Color's infrared port carried by the frontend's link bus.
 *
 * The same bus the link cable rides, on a port of its own, because it is a
 * different thing joining different machines: a lead goes where it is pushed in,
 * and light goes wherever the top of the handheld is pointing. What the port can
 * see is the frontend's decision, and with nothing in view the bus grants
 * without bound and the receiver sees the dark.
 *
 * WHAT CROSSES. One message: this machine's LED changed, and when. There is no
 * byte, no clock and no handshake to carry, because the hardware has none -- RP
 * is a lamp and an eye, and every protocol over it is the game flashing the one
 * and timing the other against its own timer. Pokemon tells a 0 from a 1 by
 * whether the gap between two flashes is 183 or 366 microseconds, so the one
 * thing this must not do is move two edges by different amounts. Every edge is
 * therefore stamped the same distance ahead, which is the commit horizon the bus
 * needs anyway.
 *
 * The protocol id and the message layout are shared byte for byte with
 * gambatte's driver, so a unit run by this core and one run by gambatte see each
 * other.
 *
 * HOW MANY. Light is not a cable and nothing limits it to a pair: every unit on
 * the bus sees every other, and the receiver reads lit while any of them is.
 *
 * WHAT DOES NOT. The receiver's own behaviour -- it warms up, and it stops
 * seeing a light that stays on -- is not modelled, nor is a unit seeing its own
 * LED. Neither is anything that is not another Game Boy Color, nor the port a
 * HuC1 or HuC3 cartridge carries.
 */

/* Room for a peer toggling its LED as fast as a Game Boy can write a register,
 * for as long as an idle horizon lasts. */
enum { GBIR_EDGE_MAX = 1024 };

struct GBSIONetlink;

struct GBIRNetlinkEdge {
	uint64_t tick;
	uint8_t from;
	uint8_t on;
};

struct GBIRNetlink {
	struct GBInfraredDriver d;
	struct mTimingEvent event;

	const struct retro_link_interface* link;
	unsigned port;
	retro_link_port_t *handle;
	bool attached;

	/* This machine's link cable, if it has one, asked only whether it is
	 * carrying anybody.
	 *
	 * A machine that waits on two buses can be waited on through both, and the
	 * two buses line their machines up separately: the cable can want this one
	 * to hold for the other while the light wants the other to hold for this
	 * one, and with no timeout on either nothing would ever move again. So a
	 * machine waits on ONE, and the cable is the one, because a transfer over it
	 * cannot be a little late. A port made deaf by that still says where it is,
	 * so nobody waits on it, and still flashes; it only stops seeing. */
	const struct GBSIONetlink* cable;
	bool deaf;

	int selfId;
	unsigned peers;

	/* A monotonic 64-bit clock, accumulated from mGBA's 32-bit one, at the
	 * 8388608 ticks a second the Game Boy's timing runs at whatever speed the
	 * machine is in. See GBSIONetlink for why not mTimingGlobalTime. */
	int32_t lastRaw;
	bool haveRaw;
	uint64_t nowBase;

	/* How far ahead this machine promises not to flash, and the furthest tick it
	 * has promised that for.
	 *
	 * Short while the port is in use and long while it is not, because a horizon
	 * is paid for in rendezvous and two handhelds left facing each other on a
	 * shelf should cost nearly nothing. The promise outlives the horizon that
	 * made it: one given while the port was idle still binds the first flash
	 * after it wakes. */
	uint64_t horizon;
	uint64_t promised;
	uint64_t grant;
	bool anchored;
	uint64_t reannounceUntil;

	/* RP as last written, and the LED out of it. */
	uint8_t rp;
	bool led;

	/* How far ahead an edge is stamped, and the edges that decide it.
	 *
	 * The latency, unless a promise made while idle reaches further -- and then
	 * it STAYS further for the rest of that burst, because the next edge has to
	 * be moved by exactly as much as this one or the flash between them changes
	 * length. It goes back only across a silence no protocol times. */
	uint64_t delay;
	uint64_t lastEdge;
	uint64_t lastStamp;
	bool edgeSeen;

	/* Which peers' LEDs are lit, one bit a peer, as of this machine's clock. */
	uint32_t lit;

	/* Peers' edges, held until this machine's clock reaches the tick each was
	 * stamped with: WHEN one arrived is a wall-clock accident, when its tick
	 * comes round is not. */
	struct GBIRNetlinkEdge edges[GBIR_EDGE_MAX];
	unsigned edgeHead;
	unsigned edgeCount;
	bool overflowed;

	/* How many edges have come in, how many of them after the tick they were
	 * stamped for, and the worst of those. An edge should never be late: the
	 * horizon is what forbids it. Counted always, since it is one comparison an
	 * edge, and said once as the port leaves the bus if GBC_IR_STATS is set in
	 * the environment -- which is how to find out on a machine that cannot be
	 * watched any other way. Read by nothing that decides anything. */
	unsigned edgesIn;
	unsigned edgesLate;
	uint64_t worstLate;
};

void GBIRNetlinkCreate(struct GBIRNetlink*, const struct retro_link_interface* link, unsigned port,
                       const struct GBSIONetlink* cable);
void GBIRNetlinkDestroy(struct GBIRNetlink*);

/* Put the driver back on the schedule after a state has been loaded. A load
 * clears the schedule and restores only the events the state knows about, and
 * it replaces the clock this driver measures from. */
void GBIRNetlinkRearm(struct GBIRNetlink*);

CXX_GUARD_END

#endif

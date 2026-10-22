/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "gb_ir_netlink.h"

#include "gb_sio_netlink.h"

#include <mgba/internal/gb/gb.h>
#include <mgba/internal/gb/io.h>

/* Names the emulated wire, and keeps it apart from the link cable's: the two
 * share a bus implementation and nothing else. Byte for byte what gambatte's
 * driver calls it. */
#define GBIR_PROTOCOL "gbc-ir"

/* The Game Boy's timing rate: see GBNL_CLOCK_RATE. */
#define GBIR_CLOCK_RATE 8388608ull

/* How late a flash arrives, and how often two units in use rendezvous: 122
 * microseconds, which is 512 cycles of an undoubled CPU.
 *
 * Light takes no time and this is not light, so it is as small as it can be
 * afforded. Every edge is late by the same amount, which no receiver can see;
 * what a game CAN see is its partner's answers arriving twice this late, once
 * for each direction. The protocols leave room for that because the hardware
 * needed it -- Pokemon waits two milliseconds before it answers and eight before
 * it gives up -- and a game polling the port from a timer interrupt cannot
 * resolve much under thirty microseconds to begin with.
 *
 * Smaller is a rendezvous between two emulation threads that much more often. */
#define GBIR_LATENCY 1024ull

/* The horizon of a port nobody is using, about two milliseconds. A promise this
 * long is still standing when the port wakes, and is what the first flash after
 * that has to wait out -- once, at the head of a conversation, where every
 * protocol is waiting for a button to be pressed anyway. */
#define GBIR_IDLE_HORIZON 16384ull

/* How long after its last flash a port still counts as in use: an eighth of a
 * second. A unit that only ever transmits never switches its receiver on, so the
 * LED itself has to keep the horizon short between one flash and the next. Also
 * the silence across which an edge may go back to the ordinary latency. */
#define GBIR_HOLD 1048576ull

/* How long a lit LED keeps being announced after the view changes. A message to
 * a peer that has not rendezvoused since the bus was rebuilt has nowhere to land
 * and is dropped, and an LED left on says nothing further by itself. Counted on
 * this machine's own clock, never by whether an answer came: see GBSIONetlink. */
#define GBIR_REANNOUNCE (8 * GBIR_IDLE_HORIZON)

/* How soon to look again when a rendezvous came back with news and no room to
 * run: one cycle of an undoubled CPU. Never an odd tick -- a halted Game Boy is
 * moved from event to event, and one landing half way through a cycle would
 * leave the machine half a cycle out for good. */
#define GBIR_RETRY 2

/* Further than two readings of the clock can be apart, since one is taken at
 * every rendezvous. Anything longer is the clock having been replaced under
 * this driver by a loaded state, which is not time passing. */
#define GBIR_DISCONTINUITY (1 << 22)

enum {
	GBIR_LEVEL = 1
};

#define GBIR_MSG_SIZE 8

static bool GBIRNetlinkInit(struct GBInfraredDriver* driver);
static void GBIRNetlinkDeinit(struct GBInfraredDriver* driver);
static void GBIRNetlinkWriteRP(struct GBInfraredDriver* driver, uint8_t value);
static bool GBIRNetlinkReadLight(struct GBInfraredDriver* driver);
static void _irEvent(struct mTiming* timing, void* context, uint32_t cyclesLate);

/* The port is a Game Boy Color's. An original Game Boy has none, and neither has
 * a Game Boy Advance, which runs these games and leaves the window out. Asked of
 * the model as it stands, because a Game Boy Color that has dropped into its
 * compatibility mode for an older game has put the register out of reach. */
static bool _hasPort(const struct GB* gb) {
	return gb->model == GB_MODEL_CGB || gb->model == GB_MODEL_SCGB;
}

static uint64_t _now(struct GBIRNetlink* nl) {
	int32_t raw = mTimingCurrentTime(&nl->d.p->timing);
	int32_t delta;

	if (!nl->haveRaw) {
		nl->haveRaw = true;
		nl->lastRaw = raw;
		return nl->nowBase;
	}

	/* Signed difference, so the int32 wrapping round is just another step
	 * forward. */
	delta = raw - nl->lastRaw;
	nl->lastRaw = raw;
	if (delta > 0 && delta < GBIR_DISCONTINUITY) {
		nl->nowBase += (uint64_t) delta;
	}
	return nl->nowBase;
}

/* How long until this machine's own clock next reads a multiple of `period`.
 *
 * The driver wakes on that grid rather than a fixed step from wherever it last
 * ran, and the reason is a loaded state. The clock is part of the state and
 * this driver's place on the schedule is not, so a step counted from the load
 * would put the machine on a different beat from the one that never loaded --
 * and those two are exactly what rollback netplay runs side by side and
 * expects to agree. Counted off the clock, both land on the same ticks.
 *
 * `period` is a power of two, which is also what makes the clock wrapping
 * round harmless. */
static int32_t _untilGrid(const struct GBIRNetlink* nl, uint32_t period) {
	uint32_t raw = (uint32_t) mTimingCurrentTime(&nl->d.p->timing);
	return (int32_t) (period - (raw & (period - 1)));
}

/* A step that is short of the grid, made safe to schedule: never odd, for the
 * reason given at GBIR_RETRY, and never nothing. */
static int32_t _evenStep(int32_t step) {
	step &= ~1;
	return step < GBIR_RETRY ? GBIR_RETRY : step;
}

/* Either half of the port: the receiver switched on, or the LED lit. And for a
 * while after a flash, because a unit that only transmits writes nothing but
 * that one bit and is dark between flashes. */
static bool _hot(const struct GBIRNetlink* nl, uint64_t now) {
	if (nl->rp & 0xC1) {
		return true;
	}
	return nl->edgeSeen && now - nl->lastEdge < GBIR_HOLD;
}

static void _wire(struct GBIRNetlink* nl, uint64_t now, bool on) {
	uint8_t msg[GBIR_MSG_SIZE];
	uint64_t stamp;

	/* Nothing may be sent before this machine has rendezvoused since the bus was
	 * last rebuilt: see GBSIONetlink. Asking for nothing is granted at once,
	 * whatever anybody else is doing. */
	if (!nl->anchored) {
		uint64_t safe = now + nl->horizon;
		if (safe < nl->promised) {
			safe = nl->promised;
		}
		nl->grant = nl->link->advance(nl->handle, now, safe, 0, NULL);
		nl->promised = safe;
		nl->anchored = true;
	}

	/* The promise made to the bus has to be kept, and it may reach further than
	 * the latency if it was made while the port was idle. Then the delay grows to
	 * meet it and stays grown: this edge and the next must be moved by the same
	 * amount. */
	stamp = now + nl->delay;
	if (stamp < nl->promised) {
		nl->delay = nl->promised - now;
		stamp = nl->promised;
	}
	if (stamp < nl->lastStamp) {
		stamp = nl->lastStamp;
	}
	nl->lastStamp = stamp;

	memset(msg, 0, sizeof(msg));
	msg[0] = GBIR_LEVEL;
	msg[1] = (uint8_t) nl->selfId;
	msg[2] = on ? 1 : 0;
	nl->link->send(nl->handle, stamp, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

static void _setLed(struct GBIRNetlink* nl, uint64_t now, bool on) {
	if (on == nl->led) {
		return;
	}
	nl->led = on;

	/* A new burst, if the last edge was long enough ago that nothing can be
	 * timing the gap between the two. */
	if (!nl->edgeSeen || now - nl->lastEdge >= GBIR_HOLD) {
		nl->delay = GBIR_LATENCY;
	}
	nl->edgeSeen = true;
	nl->lastEdge = now;

	if (nl->attached && nl->peers >= 2) {
		_wire(nl, now, on);
	}
}

/* RP as the machine holds it, whoever put it there.
 *
 * A write comes through the driver; a reset and a loaded state do not, and both
 * replace the register. An LED left lit in another machine's eyes by a register
 * that no longer says so would stay lit for ever, so every rendezvous looks. */
static void _syncRegister(struct GBIRNetlink* nl, uint64_t now) {
	const struct GB* gb = nl->d.p;
	uint8_t rp = _hasPort(gb) ? (gb->memory.io[GB_REG_RP] & 0xC1) : 0;

	nl->rp = rp;
	_setLed(nl, now, rp & 1);
}

static void _apply(struct GBIRNetlink* nl, const struct GBIRNetlinkEdge* edge) {
	uint32_t bit = 1u << (edge->from < 31 ? edge->from : 31);

	if (edge->on) {
		nl->lit |= bit;
	} else {
		nl->lit &= ~bit;
	}
}

static void _queue(struct GBIRNetlink* nl, const struct GBIRNetlinkEdge* edge) {
	if (nl->edgeCount >= GBIR_EDGE_MAX) {
		/* A peer cannot have flashed this often inside one horizon, so this is
		 * the reasoning being wrong rather than the buffer being small. The
		 * oldest edge is acted on early to make room, which keeps the LED's final
		 * state right at the cost of when it got there, and it is said once. How
		 * many messages have arrived by a rendezvous is fixed by the grants, so
		 * even this is the same on two peers replaying it. */
		if (!nl->overflowed) {
			nl->overflowed = true;
			mLOG(GB_IO, WARN, "Infrared: more edges waiting than the horizon should allow;"
			     " acting on the oldest early");
		}
		_apply(nl, &nl->edges[nl->edgeHead]);
		nl->edgeHead = (nl->edgeHead + 1) % GBIR_EDGE_MAX;
		--nl->edgeCount;
	}
	nl->edges[(nl->edgeHead + nl->edgeCount) % GBIR_EDGE_MAX] = *edge;
	++nl->edgeCount;
}

static void _applyDue(struct GBIRNetlink* nl, uint64_t now) {
	while (nl->edgeCount > 0 && nl->edges[nl->edgeHead].tick <= now) {
		_apply(nl, &nl->edges[nl->edgeHead]);
		nl->edgeHead = (nl->edgeHead + 1) % GBIR_EDGE_MAX;
		--nl->edgeCount;
	}
}

static void _pump(struct GBIRNetlink* nl) {
	uint8_t buf[GBIR_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(buf);

	while (nl->link->recv(nl->handle, &tick, &from, buf, &len)) {
		if (len == GBIR_MSG_SIZE && buf[0] == GBIR_LEVEL) {
			struct GBIRNetlinkEdge edge;
			edge.tick = tick;
			edge.from = (uint8_t) (from < 255 ? from : 255);
			edge.on = buf[2] & 1;
			/* Not while deaf: a port that waits for nobody is told things when
			 * it is told them, and is not listening anyway. */
			++nl->edgesIn;
			if (!nl->deaf && tick < nl->nowBase) {
				++nl->edgesLate;
				if (nl->nowBase - tick > nl->worstLate) {
					nl->worstLate = nl->nowBase - tick;
				}
			}
			_queue(nl, &edge);
		}
		len = sizeof(buf);
	}
}

static void _forgetPeers(struct GBIRNetlink* nl) {
	nl->lit = 0;
	nl->edgeHead = 0;
	nl->edgeCount = 0;
}

static void _refreshPeers(struct GBIRNetlink* nl, uint64_t now) {
	unsigned was = nl->peers;
	unsigned count = 0;
	int id;

	id = nl->link->peers(nl->handle, &count);
	if (id < 0) {
		nl->selfId = 0;
		nl->peers = 0;
	} else {
		nl->selfId = id;
		nl->peers = count;
	}

	if (nl->peers != was) {
		/* Forget who was lit, and say so again if this machine is. What was in
		 * view a moment ago is not evidence about what is in view now, and a unit
		 * that came into view since has never been told anything. */
		_forgetPeers(nl);
		nl->anchored = false;
		nl->reannounceUntil = now + GBIR_REANNOUNCE;
		mLOG(GB_IO, DEBUG, "Infrared: %u unit(s) in view, this one is %i", nl->peers, nl->selfId);
	}
}

static void _detach(struct GBIRNetlink* nl) {
	if (!nl->attached) {
		return;
	}
	if (getenv("GBC_IR_STATS")) {
		mLOG(GB_IO, WARN, "Infrared: %u edges in, %u of them late, the worst by %u ticks",
		     nl->edgesIn, nl->edgesLate, (unsigned) nl->worstLate);
	}
	nl->link->detach(nl->handle);
	nl->attached = false;
	nl->anchored = false;
	nl->handle = NULL;
	nl->peers = 0;
	nl->selfId = 0;
	_forgetPeers(nl);
}

void GBIRNetlinkCreate(struct GBIRNetlink* nl, const struct retro_link_interface* link, unsigned port,
                       const struct GBSIONetlink* cable) {
	memset(nl, 0, sizeof(*nl));
	nl->d.init = GBIRNetlinkInit;
	nl->d.deinit = GBIRNetlinkDeinit;
	nl->d.writeRP = GBIRNetlinkWriteRP;
	nl->d.readLight = GBIRNetlinkReadLight;

	nl->event.context = nl;
	nl->event.name = "GB IR Netlink";
	nl->event.callback = _irEvent;
	nl->event.priority = 0x32;

	nl->link = link;
	nl->port = port;
	nl->cable = cable;
	nl->horizon = GBIR_IDLE_HORIZON;
	nl->delay = GBIR_LATENCY;
}

void GBIRNetlinkDestroy(struct GBIRNetlink* nl) {
	if (nl->link) {
		_detach(nl);
	}
}

void GBIRNetlinkRearm(struct GBIRNetlink* nl) {
	struct GB* gb = nl->d.p;
	int32_t step;

	if (!gb || !nl->attached) {
		return;
	}
	/* The clock has been replaced, so it is read afresh, and read HERE: whatever
	 * the machine runs from this instant has to be counted. */
	nl->lastRaw = mTimingCurrentTime(&gb->timing);
	nl->haveRaw = true;

	/* Back on the grid -- but with somebody in view, no further than this
	 * machine had been granted before the load. A grant is a distance the peer
	 * promised to stay quiet for, and a machine that ran a whole horizon on
	 * from wherever the state put it would run past the end of that promise and
	 * see whatever was flashed there late. */
	step = _untilGrid(nl, (uint32_t) nl->horizon);
	if (nl->peers >= 2 && !nl->deaf && nl->grant != RETRO_LINK_UNBOUNDED) {
		uint64_t left = nl->grant > nl->nowBase ? nl->grant - nl->nowBase : 0;
		if (left < (uint64_t) step) {
			step = _evenStep((int32_t) left);
		}
	}
	mTimingDeschedule(&gb->timing, &nl->event);
	mTimingSchedule(&gb->timing, &nl->event, step);
}

/* Called when the driver is installed and again at every reset, which is the
 * first moment the machine's model is known.
 *
 * The port joins the bus here rather than at load for that reason: whether there
 * is a port at all depends on what the machine turned out to be. Once joined it
 * stays joined across resets -- see GBSIONetlinkDeinit for why a reset must not
 * take a machine off its wire -- and comes off only if a reset turns the machine
 * into one that has no port. */
static bool GBIRNetlinkInit(struct GBInfraredDriver* driver) {
	struct GBIRNetlink* nl = (struct GBIRNetlink*) driver;
	struct GB* gb = driver->p;

	if (!nl->link) {
		return false;
	}
	if (gb->model == GB_MODEL_AUTODETECT) {
		return true;
	}
	if (!_hasPort(gb)) {
		_detach(nl);
		return true;
	}

	if (!nl->attached) {
		nl->handle = nl->link->attach(nl->port, GBIR_PROTOCOL, GBIR_CLOCK_RATE);
		if (!nl->handle) {
			mLOG(GB_IO, WARN, "Infrared: the frontend refused port %u", nl->port);
			return false;
		}
		nl->attached = true;
		nl->anchored = false;
		nl->reannounceUntil = 0;
		_forgetPeers(nl);
	}

	/* A reset puts mGBA's cycle counter back to zero. Re-anchor, keeping the
	 * accumulated total moving forward so the bus never sees this machine's
	 * clock go back. */
	nl->haveRaw = false;

	/* On the schedule. The reset has just replaced RP, and the first rendezvous
	 * is what notices. */
	nl->horizon = GBIR_IDLE_HORIZON;
	mTimingDeschedule(&gb->timing, &nl->event);
	mTimingSchedule(&gb->timing, &nl->event, _untilGrid(nl, GBIR_IDLE_HORIZON));
	return true;
}

/* Off the schedule, but NOT off the bus: see GBSIONetlinkDeinit. */
static void GBIRNetlinkDeinit(struct GBInfraredDriver* driver) {
	struct GBIRNetlink* nl = (struct GBIRNetlink*) driver;
	mTimingDeschedule(&driver->p->timing, &nl->event);
}

/* The value is passed in rather than read out of memory, because GBIOWrite calls
 * the driver before it stores it. */
static void GBIRNetlinkWriteRP(struct GBInfraredDriver* driver, uint8_t value) {
	struct GBIRNetlink* nl = (struct GBIRNetlink*) driver;
	uint64_t now;

	if (!nl->attached) {
		return;
	}
	now = _now(nl);
	nl->rp = value & 0xC1;
	_setLed(nl, now, value & 1);
}

static bool GBIRNetlinkReadLight(struct GBInfraredDriver* driver) {
	struct GBIRNetlink* nl = (struct GBIRNetlink*) driver;

	if (!nl->attached || nl->deaf || nl->peers < 2) {
		return false;
	}
	_applyDue(nl, _now(nl));
	return nl->lit != 0;
}

static void _irEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	struct GBIRNetlink* nl = context;
	uint32_t wakeFlags = RETRO_LINK_WAKE_NONE;
	uint64_t now;
	uint64_t safe;
	int32_t step;

	UNUSED(cyclesLate);
	if (!nl->attached) {
		return;
	}
	now = _now(nl);

	_refreshPeers(nl, now);
	_syncRegister(nl, now);
	nl->deaf = nl->cable && nl->cable->attached && nl->cable->cabled;

	/* The short horizon only with somebody in view. With nobody there is no
	 * promise to keep, and a machine alone must wake on one beat whatever its
	 * LED has been doing: when it last flashed is this driver's memory, not the
	 * machine's, and does not come back with a loaded state. */
	nl->horizon = (nl->peers >= 2 && _hot(nl, now)) ? GBIR_LATENCY : GBIR_IDLE_HORIZON;
	safe = now + nl->horizon;
	if (safe < nl->promised) {
		safe = nl->promised;
	}

	/* Never asks for more than its own horizon, which is what keeps two units
	 * from each waiting for the other to go first: whichever has promised the
	 * least is asking for no more than everybody has already promised it.
	 *
	 * A deaf port asks for nothing at all. That is granted whatever the others
	 * are doing, so it publishes where it is and waits for nobody. */
	nl->grant = nl->link->advance(nl->handle, now, safe, nl->deaf ? 0 : now + nl->horizon, &wakeFlags);
	nl->promised = safe;
	nl->anchored = true;

	if (wakeFlags & RETRO_LINK_WAKE_TOPOLOGY) {
		/* The bus was rebuilt under this machine. The count may not have changed
		 * -- one unit swapped for another leaves it at two -- so the count alone
		 * cannot be trusted to say so. */
		_forgetPeers(nl);
		nl->reannounceUntil = now + GBIR_REANNOUNCE;
	}

	_pump(nl);
	_applyDue(nl, now);

	/* A flash is an edge and is sent as one, but an LED that is simply ON when
	 * somebody comes into view has no edge left to send. */
	if (nl->led && nl->peers >= 2 && now < nl->reannounceUntil) {
		_wire(nl, now, true);
	}

	/* Never further than the horizon, and no further than the grant. Being late
	 * needs no allowance: the grid is read off the clock as it stands. */
	step = _untilGrid(nl, (uint32_t) nl->horizon);
	if (!nl->deaf && nl->grant != RETRO_LINK_UNBOUNDED) {
		if (wakeFlags != RETRO_LINK_WAKE_NONE && nl->grant <= now) {
			step = GBIR_RETRY;
		} else if (nl->grant > now && nl->grant - now < (uint64_t) step) {
			step = _evenStep((int32_t) (nl->grant - now));
		}
	}
	mTimingSchedule(timing, &nl->event, step);
}

/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "ereader_disk.h"
#include "accessory_interface.h"

#include <mgba/core/core.h>
#include <mgba/gba/interface.h>
#include <mgba/internal/gba/gba.h>
#include <mgba-util/threading.h>
#include <mgba-util/vfs.h>

#define EREADER_DISK_MAX 16

struct EReaderStrip {
	char* path;
	void* data;
	size_t size;
};

static struct mCore* eCore = NULL;
static struct EReaderStrip eStrips[EREADER_DISK_MAX];
static unsigned eCount = 1;
static unsigned eIndex = 0;
static bool eEjected = false;

/* A strip handed over through the accessory interface, waiting for the
 * next retro_run. The interface lets a frontend call from any thread, and the
 * reader is fed on the core's own. */
static Mutex ePendingMutex;
static bool ePendingMutexReady = false;
static void* ePendingData = NULL;
static size_t ePendingSize = 0;

/*
 * Sizes GBACartEReaderScan decodes. It returns silently on anything else, so a
 * strip of the wrong length would be a card that does nothing and says nothing;
 * refusing the replace puts the failure where a frontend can report it.
 */
static bool _sizeIsScannable(size_t size) {
	switch (size) {
	case 1308:
	case 1344:
	case 1872:
	case 2076:
	case 2112:
	case 2912:
	case 3520:
	case 5456:
		return true;
	default:
		return false;
	}
}

/* The strip file at `path`, whole, or false when it cannot be read or is not
 * a length the reader decodes. */
static bool _readStrip(const char* path, void** dataOut, size_t* sizeOut) {
	struct VFile* vf = VFileOpen(path, O_RDONLY);
	if (!vf) {
		return false;
	}
	ssize_t size = vf->size(vf);
	if (size <= 0 || !_sizeIsScannable((size_t) size)) {
		vf->close(vf);
		return false;
	}
	void* data = malloc((size_t) size);
	if (!data) {
		vf->close(vf);
		return false;
	}
	if (vf->read(vf, data, (size_t) size) != size) {
		free(data);
		vf->close(vf);
		return false;
	}
	vf->close(vf);
	*dataOut = data;
	*sizeOut = (size_t) size;
	return true;
}

static void _clearPending(void) {
	if (!ePendingMutexReady) {
		return;
	}
	MutexLock(&ePendingMutex);
	free(ePendingData);
	ePendingData = NULL;
	ePendingSize = 0;
	MutexUnlock(&ePendingMutex);
}

static void _clearStrip(struct EReaderStrip* strip) {
	free(strip->path);
	free(strip->data);
	strip->path = NULL;
	strip->data = NULL;
	strip->size = 0;
}

void EReaderDiskReset(void) {
	size_t i;
	for (i = 0; i < EREADER_DISK_MAX; ++i) {
		_clearStrip(&eStrips[i]);
	}
	_clearPending();
	eCore = NULL;
	eCount = 1;
	eIndex = 0;
	eEjected = false;
}

static bool _erSetEject(bool ejected) {
	/* Closing on a loaded strip IS the scan, and it happens now: the strip is
	 * decoded straight into the reader's dot buffer and read on the scan pass
	 * that follows. A card offered while the scanner is unpowered -- the game is
	 * not asking for one -- is dropped rather than held, so a swipe made at the
	 * wrong moment does nothing instead of firing later on its own. */
	if (!ejected && eEjected && eCore && eIndex < eCount) {
		struct EReaderStrip* strip = &eStrips[eIndex];
		if (strip->data && strip->size) {
			GBACartEReaderQueueCard((struct GBA*) eCore->board, strip->data, strip->size);
		}
	}
	eEjected = ejected;
	return true;
}

static bool _erGetEject(void) {
	return eEjected;
}

static unsigned _erGetImageIndex(void) {
	return eIndex;
}

static bool _erSetImageIndex(unsigned index) {
	if (index >= eCount) {
		return false;
	}
	eIndex = index;
	return true;
}

static unsigned _erGetNumImages(void) {
	return eCount;
}

static bool _erReplaceImage(unsigned index, const struct retro_game_info* info) {
	if (index >= eCount) {
		return false;
	}
	struct EReaderStrip* strip = &eStrips[index];
	if (!info || !info->path) {
		/* A null info is how a frontend removes an image. */
		_clearStrip(strip);
		return true;
	}

	void* data;
	size_t size;
	if (!_readStrip(info->path, &data, &size)) {
		return false;
	}

	_clearStrip(strip);
	strip->data = data;
	strip->size = size;
	strip->path = strdup(info->path);
	return true;
}

static bool _erAddImageIndex(void) {
	if (eCount >= EREADER_DISK_MAX) {
		return false;
	}
	++eCount;
	return true;
}

static bool _erGetImagePath(unsigned index, char* path, size_t len) {
	if (index >= eCount || !eStrips[index].path || !path || !len) {
		return false;
	}
	strncpy(path, eStrips[index].path, len - 1);
	path[len - 1] = '\0';
	return true;
}

static bool _erGetImageLabel(unsigned index, char* label, size_t len) {
	if (index >= eCount || !eStrips[index].path || !label || !len) {
		return false;
	}
	/* The file's own name: a card is identified by its title, and the set names
	 * each strip after the card it came off. */
	const char* base = strrchr(eStrips[index].path, '/');
	const char* alt = strrchr(eStrips[index].path, '\\');
	if (alt > base) {
		base = alt;
	}
	base = base ? base + 1 : eStrips[index].path;
	strncpy(label, base, len - 1);
	label[len - 1] = '\0';
	return true;
}

/* The same card slot through the accessory interface, which says what it
 * is: a reader that takes a dot-code strip for a moment. A strip handed over is
 * read and checked here and swiped in the next retro_run, with the same rule as
 * the tray-close edge above: a swipe the game is not asking for does nothing. */
static const struct retro_accessory_reader eAccessoryReaders[] = {
	{ RETRO_ACCESSORY_DOT_CODE, RETRO_ACCESSORY_PORT_CONSOLE, RETRO_ACCESSORY_READER_MOMENTARY, "e-Reader" },
};

static bool _amInsert(unsigned reader, const char* path) {
	if (reader != 0 || !path || !eCore || !ePendingMutexReady) {
		return false;
	}
	void* data;
	size_t size;
	if (!_readStrip(path, &data, &size)) {
		return false;
	}
	MutexLock(&ePendingMutex);
	free(ePendingData);
	ePendingData = data;
	ePendingSize = size;
	MutexUnlock(&ePendingMutex);
	return true;
}

static bool _amRemove(unsigned reader) {
	if (reader != 0) {
		return false;
	}
	/* A strip passes straight through; one not yet swiped is simply not. */
	_clearPending();
	return true;
}

static bool _amIsPresent(unsigned reader) {
	if (reader != 0 || !ePendingMutexReady) {
		return false;
	}
	MutexLock(&ePendingMutex);
	bool present = ePendingData != NULL;
	MutexUnlock(&ePendingMutex);
	return present;
}

void EReaderAccessoryPoll(void) {
	if (!eCore || !ePendingMutexReady) {
		return;
	}
	MutexLock(&ePendingMutex);
	void* data = ePendingData;
	size_t size = ePendingSize;
	ePendingData = NULL;
	ePendingSize = 0;
	MutexUnlock(&ePendingMutex);
	if (data) {
		GBACartEReaderQueueCard((struct GBA*) eCore->board, data, size);
		free(data);
	}
}

static void _publishAccessory(retro_environment_t environCallback, bool on) {
	static struct retro_accessory_interface iface;
	iface.interface_version = RETRO_ACCESSORY_INTERFACE_VERSION;
	iface.num_readers = on ? 1 : 0;
	iface.readers = on ? eAccessoryReaders : NULL;
	iface.insert = _amInsert;
	iface.remove = _amRemove;
	iface.is_present = _amIsPresent;
	environCallback(RETRO_ENVIRONMENT_SET_ACCESSORY_INTERFACE, &iface);
}

void EReaderAccessoryWithdraw(retro_environment_t environCallback) {
	if (eCore) {
		_publishAccessory(environCallback, false);
	}
}

void EReaderDiskPublish(struct mCore* core, retro_environment_t environCallback) {
	static const struct retro_disk_control_ext_callback callbacks = {
		.set_eject_state = _erSetEject,
		.get_eject_state = _erGetEject,
		.get_image_index = _erGetImageIndex,
		.set_image_index = _erSetImageIndex,
		.get_num_images = _erGetNumImages,
		.replace_image_index = _erReplaceImage,
		.add_image_index = _erAddImageIndex,
		.set_initial_image = NULL,
		.get_image_path = _erGetImagePath,
		.get_image_label = _erGetImageLabel,
	};

	if (!ePendingMutexReady) {
		MutexInit(&ePendingMutex);
		ePendingMutexReady = true;
	}
	EReaderDiskReset();
	eCore = core;

	/* Both: a frontend that knows the accessory interface gets a card
	 * reader rather than a disc drive, and RetroArch, which does not, keeps the
	 * disk control below. */
	_publishAccessory(environCallback, true);

	unsigned version = 0;
	if (environCallback(RETRO_ENVIRONMENT_GET_DISK_CONTROL_INTERFACE_VERSION, &version) && version >= 1) {
		environCallback(RETRO_ENVIRONMENT_SET_DISK_CONTROL_EXT_INTERFACE, (void*) &callbacks);
	} else {
		/* The v0 struct is the ext struct's leading members, so the same
		 * function pointers serve a frontend that only knows the old one. */
		environCallback(RETRO_ENVIRONMENT_SET_DISK_CONTROL_INTERFACE, (void*) &callbacks);
	}
}

/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/




#include "VariableLengthRingbuffer.hpp"
#include "assert.h"
#include "string.h"


VariableLengthRingbuffer::~VariableLengthRingbuffer()
{
	deallocate();
}

bool VariableLengthRingbuffer::allocate(size_t buffer_size)
{
	LockGuard lg{_mutex};

	assert(_ringbuffer == nullptr);

	_size = buffer_size;
	_ringbuffer = new uint8_t[_size];
	return _ringbuffer != nullptr;
}

void VariableLengthRingbuffer::deallocate()
{
	LockGuard lg{_mutex};

	delete[] _ringbuffer;
	_ringbuffer = nullptr;
	_size = 0;
}

bool VariableLengthRingbuffer::push_back(const uint8_t *packet, size_t packet_len)
{
	LockGuard lg{_mutex};

	if (packet_len == 0 || packet == nullptr) {
		// Nothing to add, we better don't try.
		return false;
	}

	if (_start <= _end && _size - _end < sizeof(Header)) {
		// We don't want to break up the header, and there is no space for it,
		// so we move to beginning of packetfer.
		_end = 0;
	}

	if (_start > _end) {
		// Add after end up to start, no wrap around.

		// Leave one byte free so that start don't end up the same
		// which signals empty.
		const size_t available = _start - _end - 1;

		if (sizeof(Header) + packet_len > available) {
			return false;
		}

		const Header header{(uint32_t)packet_len};
		memcpy(&_ringbuffer[_end], &header, sizeof(header));
		_end += sizeof(header);
		memcpy(&_ringbuffer[_end], packet, packet_len);
		_end += packet_len;

	} else {
		// Add after end, maybe wrap around.
		const size_t available = _start - _end - 1 + _size;

		if (sizeof(Header) + packet_len > available) {
			return false;
		}

		const Header header{(uint32_t)packet_len};
		memcpy(&_ringbuffer[_end], &header, sizeof(header));
		_end += sizeof(header);
		_end %= _size;

		if (_end == 0) {
			// We ended up at beginning after writing, next part is one chunk.
			memcpy(&_ringbuffer[_end], packet, packet_len);
			_end += packet_len;

		} else {

			const size_t remaining_packet_len = _size - _end;

			if (packet_len > remaining_packet_len) {
				memcpy(&_ringbuffer[_end], packet, remaining_packet_len);
				_end = 0;

				memcpy(&_ringbuffer[_end], packet + remaining_packet_len, packet_len - remaining_packet_len);
				_end += packet_len - remaining_packet_len;

			} else {
				memcpy(&_ringbuffer[_end], packet, packet_len);
				_end += packet_len;
			}
		}
	}

	return true;
}

size_t VariableLengthRingbuffer::pop_front(uint8_t *buf, size_t buf_max_len)
{
	LockGuard lg{_mutex};

	if (buf == nullptr) {
		// User needs to supply a valid pointer.
		return 0;
	}

	if (_start == _end) {
		// Empty
		return 0;
	}

	// If there isn't space for the next header, so we jump to the beginning.
	if (_start + sizeof(Header) > _size) {
		_start = 0;
	}

	// Check next header
	Header header;
	memcpy(&header, &_ringbuffer[_start], sizeof(header));

	if (header.len > buf_max_len) {
		// We can't it the packet into the user supplied buffer.
		return 0;
	}

	assert(header.len < _size);

	_start += sizeof(header);
	_start %= _size;

	if (_start < _end) {
		// No wrap around.
		memcpy(buf, &_ringbuffer[_start], header.len);
		_start += header.len;

	} else {
		const size_t remaining_buf_len = _size - _start;

		if (header.len > remaining_buf_len) {
			memcpy(buf, &_ringbuffer[_start], remaining_buf_len);
			_start = 0;
			memcpy(buf + remaining_buf_len, &_ringbuffer[_start], header.len - remaining_buf_len);
			_start += header.len - remaining_buf_len;

		} else {
			memcpy(buf, &_ringbuffer[_start], header.len);
			_start += header.len;
		}
	}

	return header.len;
}

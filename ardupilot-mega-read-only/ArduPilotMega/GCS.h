// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System protocols.

#ifndef __GCS_H
#define __GCS_H

#include <BetterStream.h>
#include <AP_Common.h>
#include <APM_BinComm.h>
#include <GCS_MAVLink.h>
#include <GPS.h>
#include <Stream.h>
#include <stdint.h>

///
/// @class	GCS
/// @brief	Class describing the interface between the APM code
///			proper and the GCS implementation.
///
/// GCS' are currently implemented inside the sketch and as such have
/// access to all global state.  The sketch should not, however, call GCS
/// internal functions - all calls to the GCS should be routed through
/// this interface (or functions explicitly exposed by a subclass).
///
class GCS_Class
{
public:

	/// Startup initialisation.
	///
	/// This routine performs any one-off initialisation required before
	/// GCS messages are exchanged.
	///
	/// @note The stream is expected to be set up and configured for the
	///       correct bitrate before ::init is called.
	///
	/// @note The stream is currently BetterStream so that we can use the _P
	///	      methods; this may change if Arduino adds them to Print.
	///
	/// @param	port		The stream over which messages are exchanged.
	///
	void		init(BetterStream *port) { _port = port; }

	/// Update GCS state.
	///
	/// This may involve checking for received bytes on the stream,
	/// or sending additional periodic messages.
	void		update(void) {}

	/// Send a message with a single numeric parameter.
	///
	/// This may be a standalone message, or the GCS driver may
	/// have its own way of locating additional parameters to send.
	///
	/// @param	id			ID of the message to send.
	/// @param	param		Explicit message parameter.
	///
	void		send_message(uint8_t id, int32_t param = 0) {}

	/// Send a text message.
	///
	/// @param	severity	A value describing the importance of the message.
	/// @param	str			The text to be sent.
	///
	void		send_text(uint8_t severity, const char *str) {}

	/// Send acknowledgement for a message.
	///
	/// @param	id			The message ID being acknowledged.
	/// @param	sum1		Checksum byte 1 from the message being acked.
	/// @param	sum2		Checksum byte 2 from the message being acked.
	///
	void		acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2) {}

	/// Emit an update of the "current" waypoints, often previous, current and
	/// next.
	///
	void		print_current_waypoints() {}

	//
	// The following interfaces are not currently implemented as their counterparts
	// are not called in the mainline code.  XXX ripe for re-specification.
	//

	/// Send a text message with printf-style formatting.
	///
	/// @param	severity	A value describing the importance of the message.
	/// @param	fmt			The format string to send.
	/// @param	...			Additional arguments to the format string.
	///
	//	void		send_message(uint8_t severity, const char *fmt, ...) {}

	/// Log a waypoint
	///
	/// @param	wp			The waypoint to log.
	/// @param	index		The index of the waypoint.
	//	void		print_waypoint(struct Location *wp, uint8_t index) {}

    // test if frequency within range requested for loop
    // used by data_stream_send
    static bool freqLoopMatch(uint16_t freq, uint16_t freqMin, uint16_t freqMax)
    {
        if (freq != 0 && freq >= freqMin && freq < freqMax) return true;
        else return false;
    }

    // send streams which match frequency range
    void data_stream_send(uint16_t freqMin, uint16_t freqMax);

protected:
	/// The stream we are communicating over
	BetterStream			*_port;
};

//
// GCS class definitions.
//
// These are here so that we can declare the GCS object early in the sketch
// and then reference it statically rather than via a pointer.
//

///
/// @class	GCS_MAVLINK
/// @brief	The mavlink protocol for qgroundcontrol
///
#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK || HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK
class GCS_MAVLINK : public GCS_Class
{
public:
	GCS_MAVLINK();
	void    update(void);
	void	init(BetterStream *port);
	void	send_message(uint8_t id, uint32_t param = 0);
	void	send_text(uint8_t severity, const char *str);
	void	acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2);
    void    data_stream_send(uint16_t freqMin, uint16_t freqMax);
private:
	void 	handleMessage(mavlink_message_t * msg);

	/// Perform queued sending operations
	///
	void    _queued_send();

	AP_Var      *_queued_parameter;                 ///< next parameter to be sent in queue
	uint16_t    _queued_parameter_index;            ///< next queued parameter's index

	/// Count the number of reportable parameters.
	///
	/// Not all parameters can be reported via MAVlink.  We count the number that are
	/// so that we can report to a GCS the number of parameters it should expect when it
	/// requests the full set.
	///
	/// @return         The number of reportable parameters.
	///
    uint16_t    _count_parameters();                ///< count reportable parameters

    uint16_t    _parameter_count;                   ///< cache of reportable parameters
    AP_Var      *_find_parameter(uint16_t index);   ///< find a reportable parameter by index


	mavlink_channel_t chan;
    uint16_t packet_drops;
    uint16_t rawSensorStreamRate;
    uint16_t attitudeStreamRate;
    uint16_t positionStreamRate;
    uint16_t rawControllerStreamRate;
    uint16_t rcStreamRate;
    uint16_t extraStreamRate[3];
};
#endif // GCS_PROTOCOL_MAVLINK

#endif // __GCS_H

/*
   PAKET( name,
          TYPE ( pn, pr, member )
          ...
   )

   You may never reissue one of the pn arguments
*/

#if !defined(NL_PACKET) || !defined(NL_STRING) || !defined(NL_INTEGER) || !defined(NL_BIT) || !defined(NL_INT64)
#error "The macros NL_PACKET, NL_STRING, NL_INTEGER, NL_INT64 and NL_BIT needs to be defined"
#endif

NL_PACKET(primary, 1,
       NL_BIT(		1,	T_MAY_IGNORE,	overwrite_peer)
)

NL_PACKET(secondary, 2, )

NL_PACKET(disk_conf, 3,
	NL_INT64(	2,	T_MAY_IGNORE,	disk_size)
	NL_STRING(	3,	T_MANDATORY,	backing_dev,	128)
	NL_STRING(	4,	T_MANDATORY,	meta_dev,	128)
	NL_INTEGER(	5,	T_MANDATORY,	meta_dev_idx)
	NL_INTEGER(	6,	T_MAY_IGNORE,	on_io_error)
	NL_INTEGER(	7,	T_MAY_IGNORE,	fencing)
	NL_BIT(		37,	T_MAY_IGNORE,	use_bmbv)
)

NL_PACKET(detach, 4,)

NL_PACKET(net_conf, 5,
	NL_STRING(	8,	T_MANDATORY,	my_addr,	128)
	NL_STRING(	9,	T_MANDATORY,	peer_addr,	128)
	NL_STRING(	10,	T_MAY_IGNORE,	shared_secret,	SHARED_SECRET_MAX)
	NL_STRING(	11,	T_MAY_IGNORE,	cram_hmac_alg,	SHARED_SECRET_MAX)
	NL_INTEGER(	14,	T_MAY_IGNORE,	timeout)
	NL_INTEGER(	15,	T_MANDATORY,	wire_protocol)
	NL_INTEGER(	16,	T_MAY_IGNORE,	try_connect_int)
	NL_INTEGER(	17,	T_MAY_IGNORE,	ping_int)
	NL_INTEGER(	18,	T_MAY_IGNORE,	max_epoch_size)
	NL_INTEGER(	19,	T_MAY_IGNORE,	max_buffers)
	NL_INTEGER(	20,	T_MAY_IGNORE,	unplug_watermark)
	NL_INTEGER(	21,	T_MAY_IGNORE,	sndbuf_size)
	NL_INTEGER(	22,	T_MAY_IGNORE,	ko_count)
	NL_INTEGER(	24,	T_MAY_IGNORE,	after_sb_0p)
	NL_INTEGER(	25,	T_MAY_IGNORE,	after_sb_1p)
	NL_INTEGER(	26,	T_MAY_IGNORE,	after_sb_2p)
	NL_INTEGER(	39,	T_MAY_IGNORE,	rr_conflict)
	NL_INTEGER(	40,	T_MAY_IGNORE,	ping_timeo)
	NL_BIT(		27,	T_MAY_IGNORE,	want_lose)
	NL_BIT(		28,	T_MAY_IGNORE,	two_primaries)
	NL_BIT(		41,	T_MAY_IGNORE,	always_asbp)
)

NL_PACKET(disconnect, 6, )

NL_PACKET(resize, 7,
	NL_INT64(		29,	T_MAY_IGNORE,	resize_size)
)

NL_PACKET(syncer_conf, 8,
	NL_INTEGER(	30,	T_MAY_IGNORE,	rate)
	NL_INTEGER(	31,	T_MAY_IGNORE,	after)
	NL_INTEGER(	32,	T_MAY_IGNORE,	al_extents)
)

NL_PACKET(invalidate, 9, )
NL_PACKET(invalidate_peer, 10, )
NL_PACKET(pause_sync, 11, )
NL_PACKET(resume_sync, 12, )
NL_PACKET(suspend_io, 13, )
NL_PACKET(resume_io, 14, )
NL_PACKET(outdate, 15, )
NL_PACKET(get_config, 16, )
NL_PACKET(get_state, 17,
	NL_INTEGER(	33,	T_MAY_IGNORE,	state_i)
)

NL_PACKET(get_uuids, 18,
	NL_STRING(	34,	T_MAY_IGNORE,	uuids,	(UUID_SIZE*sizeof(__u64)))
	NL_INTEGER(	35,	T_MAY_IGNORE,	uuids_flags)
)

NL_PACKET(get_timeout_flag, 19,
	NL_BIT(		36,	T_MAY_IGNORE,	use_degraded)
)

NL_PACKET(call_helper, 20,
	NL_STRING(	38,	T_MAY_IGNORE,	helper,		32)
)

// Tag nr 42 already allocated in drbd-8.1 development.
// Packet numbers 21 and 22 already in drbd-8.1 development.

NL_PACKET(sync_progress, 23,
	NL_INTEGER(	43,	T_MAY_IGNORE,	sync_progress)
)

#undef NL_PACKET
#undef NL_INTEGER
#undef NL_INT64
#undef NL_BIT
#undef NL_STRING


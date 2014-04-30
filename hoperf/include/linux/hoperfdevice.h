#ifndef _LINUX_HOPERFDEVICE_H
#define _LINUX_HOPERFDEVICE_H

#include <linux/if_hoperf.h>
#include <linux/netdevice.h>
#include <linux/random.h>

extern const struct header_ops hoperf_header_ops;

extern void hoperf_setup( struct net_device *net_dev );
extern __be16 hoperf_type_trans( struct sk_buff *skb, struct net_device *net_dev );
extern int hoperf_header(struct sk_buff *skb, struct net_device *net_dev,
	unsigned short type,
	const void *daddr, const void *saddr, unsigned int len);
extern int hoperf_rebuild_header( struct sk_buff *skb );
extern int hoperf_header_parse( const struct sk_buff *skb, unsigned char *haddr );
//extern int hoperf_header_cache( const struct neighbour *neigh, struct hh_cache *hh, __be16 type );
//extern void hoperf_header_cache_update( struct hh_cache *hh,
//	const struct net_device *net_dev,
//	const unsigned char *haddr);
extern int hoperf_prepare_mac_addr_change( struct net_device *net_dev, void *p );
extern void hoperf_commit_mac_addr_change( struct net_device *net_dev, void *p );
extern int hoperf_mac_addr( struct net_device *net_dev, void *p );
extern int hoperf_change_mtu( struct net_device *net_dev, int new_mtu );
extern int hoperf_validate_addr(struct net_device *net_dev );

extern struct net_device *alloc_hoperfdev_mqs( int sizeof_priv, unsigned int txqs,
	unsigned int rxqs );
#define alloc_hoperfdev( sizeof_priv ) alloc_hoperfdev_mq( sizeof_priv, 1 )
#define alloc_hoperfdev_mq( sizeof_priv, count ) alloc_hoperfdev_mqs( sizeof_priv, count, count )

static inline bool is_zero_hoperf_addr( const u8 *addr )
{
	return !addr[0];
}

static inline bool is_broadcast_hoperf_addr( const u8 *addr )
{
	return addr[0] == 0xFF;
}

static inline bool is_valid_hoperf_addr( const u8 *addr )
{
	/* FF is a broadcast address so we don't need to
	 * explicitly check for it here. */
	return !is_broadcast_hoperf_addr( addr ) && !is_zero_hoperf_addr( addr );
}

static inline void hoperf_random_addr( u8 *addr )
{
	get_random_bytes(addr, HOPERF_ALEN);
	if( is_zero_hoperf_addr( addr ) )
		addr[0] += 1;
	else if( is_broadcast_hoperf_addr( addr ) )
		addr[0] -= 1;
}

#define random_hoperf_addr( addr ) hoperf_random_addr( addr )

static inline void hoperf_broadcast_addr( u8 *addr )
{
	memset( addr, 0xFF, HOPERF_ALEN );
}

static inline void hoperf_zero_addr( u8 *addr )
{
	memset( addr, 0x00, HOPERF_ALEN );
}

static inline void hoperf_hw_addr_random( struct net_device *net_dev )
{
	net_dev->addr_assign_type = NET_ADDR_RANDOM;
	hoperf_random_addr( net_dev->dev_addr );
}

static inline unsigned compare_hoperf_addr( const u8 *addr1, const u8 *addr2 )
{
	BUILD_BUG_ON( HOPERF_ALEN != 1 );
	return (addr1[0] ^ addr2[0]) != 0;
}

static inline bool hoperf_addr_equal( const u8 *addr1, const u8 *addr2 )
{
	return !compare_hoperf_addr( addr1, addr2 );
}

static inline bool is_hoperfdev_addr( const struct net_device *net_dev,
	const u8 *addr )
{
	struct netdev_hw_addr *ha;
	bool res = false;

	rcu_read_lock( );
	for_each_dev_addr( net_dev, ha )
	{
		res = hoperf_addr_equal( addr, ha->addr );
		if( res )
			break;
	}
	rcu_read_unlock();
	return res;
}

static inline unsigned long compare_hoperf_header( const void *a, const void *b )
{
	u32 *a32 = (u32 *)( (u8 *)a + 1 );
	u32 *b32 = (u32 *)( (u8 *)b + 1 );

	return ( *(u8 *)a ^ *(u8 *)b ) | ( a32[0] ^ b32[0] );
}

#endif /* _LINUX_HOPERFDEVICE_H */

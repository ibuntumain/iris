//! The in-core NFS server as its own host on a bridged LAN (PCAP mode only).
//!
//! The in-process NFS server normally lives inside the NAT engine and answers
//! at the gateway IP. In PCAP mode the NAT engine isn't running and the guest
//! sits directly on the real LAN, so the server is presented as its own L2
//! host at a configured virtual IP. `PcapEngine` feeds each guest TX frame to
//! [`NfsVirtualHost::maybe_handle`]: an ARP for, or UDP to, our IP (portmap /
//! NFS / mountd) is answered and the reply frames go straight back to the guest
//! (still zero host sockets). Anything else is bridged to the wire unchanged.
//!
//! Inbound fragments (an NFS WRITE whose wsize exceeds the MTU) are reassembled
//! before dispatch; replies are fragmented to fit the MTU.

use std::net::Ipv4Addr;

use super::frag::Reassembler;
use super::packet::{self, Ether, Ipv4Route, MacAddr};
use super::portmap::{self, MOUNTD_PORT, NFS_PORT, PORTMAP_PORT};
use crate::config::NfsConfig;

pub(crate) struct NfsVirtualHost {
    ip: Ipv4Addr,
    mac: MacAddr,
    server: super::nfsudp::NfsServer,
    ip_id: u16,
    reasm: Reassembler,
}

impl NfsVirtualHost {
    pub(crate) fn new(ip: Ipv4Addr, nfs_cfg: NfsConfig) -> Self {
        // Locally administered, distinct from the NAT gateway's, so the guest
        // keeps the virtual NFS host as its own ARP entry. Fold all four
        // octets into the last two so two bridged instances with different
        // NFS addresses never share an L2 address — a fixed suffix collided.
        let o = ip.octets();
        let fold = (u16::from(o[0]) << 8 | u16::from(o[1]))
            ^ (u16::from(o[2]) << 8 | u16::from(o[3]));
        let mac = [0x02, 0x00, 0xDE, 0xAD, (fold >> 8) as u8, fold as u8];
        let server = super::nfsudp::NfsServer::new(nfs_cfg.shared_dir.clone(), nfs_cfg.version);
        Self { ip, mac, server, ip_id: 1, reasm: Reassembler::default() }
    }

    /// Handle a guest frame if it is addressed to the virtual NFS host. Returns
    /// the reply frames to inject back to the guest (possibly none, for a
    /// fragment still being reassembled), or `None` if the frame is not ours
    /// and should be bridged.
    pub(crate) fn maybe_handle(&mut self, frame: &[u8]) -> Option<Vec<Vec<u8>>> {
        let eth = Ether::parse(frame)?;
        if let Some(a) = eth.arp() {
            if a.op() != packet::ARP_REQUEST || a.target_ip() != self.ip {
                return None;
            }
            return Some(vec![packet::arp(packet::ARP_REPLY, self.mac, self.ip, a.sender_mac(), a.sender_ip())]);
        }
        let ip = eth.ipv4()?;
        if ip.proto() != packet::PROTO_UDP || ip.dst() != self.ip {
            return None;
        }
        let whole;
        let udp_bytes = if ip.is_fragment() {
            whole = match self.reasm.add(&ip) {
                Some(w) => w,
                None => return Some(Vec::new()), // ours, still waiting on fragments
            };
            &whole[..]
        } else {
            ip.payload()
        };
        let udp = packet::Udp::parse(udp_bytes)?;
        let reply = match udp.dport() {
            PORTMAP_PORT => portmap::reply(udp.payload()),
            NFS_PORT | MOUNTD_PORT => self.server.handle(udp.payload())?,
            _ => return None,
        };
        self.ip_id = self.ip_id.wrapping_add(1);
        let route = Ipv4Route::new(eth.src(), self.mac, self.ip, ip.src());
        Some(route.udp_fragments(self.ip_id, udp.dport(), udp.sport(), &reply))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::net::portmap::{getport_call, PROG_MOUNTD, PROG_NFS};
    /// Two virtual NFS hosts must not share an L2 address: bridged instances
    /// with distinct NFS subnets used to collide on one fixed MAC. The
    /// derivation is stable for the same address and separates subnets even
    /// when only the first two octets differ.
    #[test]
    fn distinct_nfs_hosts_get_distinct_macs() {
        let a = host(Ipv4Addr::new(192, 168, 10, 1));
        let b = host(Ipv4Addr::new(192, 168, 11, 1));
        let c = host(Ipv4Addr::new(10, 0, 1, 1));
        let d = host(Ipv4Addr::new(10, 1, 1, 1));
        assert_ne!(a.mac, b.mac);
        assert_ne!(c.mac, d.mac);
        assert_eq!(a.mac, host(Ipv4Addr::new(192, 168, 10, 1)).mac, "the same address derives the same MAC");
        assert_eq!(&a.mac[..4], &[0x02, 0x00, 0xDE, 0xAD]);
        assert_eq!(a.mac[0] & 0x03, 0x02, "locally administered, unicast");
    }


    fn host(ip: Ipv4Addr) -> NfsVirtualHost {
        NfsVirtualHost::new(ip, NfsConfig { shared_dir: "/nonexistent".into(),
                                            version: crate::net::nfsudp::NfsVersion::Auto })
    }

    fn port_in(reply: &[u8]) -> u32 {
        let ip = Ether::parse(reply).unwrap().ipv4().unwrap();
        let p = ip.udp().unwrap().payload();
        u32::from_be_bytes(p[24..28].try_into().unwrap())
    }

    /// A fragmented request (large NFS WRITEs arrive the same way) is
    /// reassembled: the first fragment is consumed (`Some([])`, so it is not
    /// bridged onto the wire) and the last yields the reply.
    #[test]
    fn reassembles_fragmented_request() {
        let nfs_ip = Ipv4Addr::new(192, 168, 1, 213);
        let guest = Ipv4Route::new([0x02, 0, 0xde, 0xad, 0xbf, 0x53], [0x08, 0, 0x69, 1, 2, 3],
                                   Ipv4Addr::new(192, 168, 1, 50), nfs_ip);
        let mut h = host(nfs_ip);
        // A GETPORT padded past one MTU so it has to fragment.
        let mut call = getport_call(0x1234, PROG_NFS);
        call.resize(3000, 0);
        let frags = guest.udp_fragments(0x42, 0x9abc, PORTMAP_PORT, &call);
        assert_eq!(frags.len(), 3);
        assert_eq!(h.maybe_handle(&frags[0]), Some(Vec::new()));
        assert_eq!(h.maybe_handle(&frags[1]), Some(Vec::new()));
        let replies = h.maybe_handle(&frags[2]).expect("reply after reassembly");
        assert_eq!(replies.len(), 1);
        assert_eq!(port_in(&replies[0]), NFS_PORT as u32);
    }

    #[test]
    fn handles_unfragmented_request_and_arp() {
        let nfs_ip = Ipv4Addr::new(10, 0, 0, 9);
        let guest_ip = Ipv4Addr::new(10, 0, 0, 2);
        let guest_mac = [0x08, 0, 0x69, 0x0a, 0x0b, 0x0c];
        let mut h = host(nfs_ip);
        let route = Ipv4Route::new(h.mac, guest_mac, guest_ip, nfs_ip);
        let replies = h.maybe_handle(&route.udp(7, 0x9abc, PORTMAP_PORT, &getport_call(7, PROG_MOUNTD))).unwrap();
        assert_eq!(port_in(&replies[0]), MOUNTD_PORT as u32);

        let arp = packet::arp(packet::ARP_REQUEST, guest_mac, guest_ip, [0; 6], nfs_ip);
        let r = h.maybe_handle(&arp).expect("ARP reply");
        assert_eq!(Ether::parse(&r[0]).unwrap().arp().unwrap().sender_mac(), h.mac);
        // Not ours: bridged.
        let other = packet::arp(packet::ARP_REQUEST, guest_mac, guest_ip, [0; 6], Ipv4Addr::new(10, 0, 0, 1));
        assert!(h.maybe_handle(&other).is_none());
    }
}

#![no_main]
#![no_std]

use core::fmt::Write;

use libtock::runtime::{set_main, stack_size, TockSyscalls};
use libtock::console::Console;
use libtock::alarm::{Alarm, Milliseconds};

use libtock_platform::ErrorCode;
use libtock_platform::Syscalls;
use libtock_platform::DefaultConfig;
use libtock_platform::subscribe::Subscribe;
use libtock_platform::allow_rw::AllowRw;
use libtock_platform::share;

use smoltcp::phy::Device;

use core::cell::Cell;

set_main!{main}
stack_size!{0x8000}

trait Config: libtock_platform::allow_ro::Config +
              libtock_platform::allow_rw::Config +
              libtock_platform::subscribe::Config {}

impl<T: libtock_platform::allow_ro::Config +
        libtock_platform::allow_rw::Config +
        libtock_platform::subscribe::Config>
    Config for T {}

const ETH_MTU: usize = 1524;

struct TockTapRxToken<'a>(&'a mut [u8; ETH_MTU]);

impl smoltcp::phy::RxToken for TockTapRxToken<'_> {
    fn consume<R, F>(self, _timestamp: smoltcp::time::Instant, f: F) -> smoltcp::Result<R>
        where F: FnOnce(&mut [u8]) -> smoltcp::Result<R>
    {
        let result = f(self.0);
        writeln!(Console::writer(), "RX called").unwrap();
        result
    }
}

struct TockTapTxToken<'a>(&'a mut [u8; ETH_MTU]);

impl smoltcp::phy::TxToken for TockTapTxToken<'_> {
    fn consume<R, F>(self, _timestamp: smoltcp::time::Instant, len: usize, f: F) -> smoltcp::Result<R>
        where F: FnOnce(&mut [u8]) -> smoltcp::Result<R> {
        let result = f(&mut self.0[..len]);
        writeln!(Console::writer(), "TX called with len = {}", len).unwrap();
        result
    }
}

struct TockTapDevice {
    rx_buffer: [u8; ETH_MTU],
    tx_buffer: [u8; ETH_MTU],
}

const DRIVER_NUM: u32 = 0x30005;

// Commands
const DRIVER_LOCK_CMD: u32 = 1;
const ACK_RX_PKT_CMD: u32 = 7;

// Upcalls
const RX_UPCALL_NO: u32 = 1;

impl TockTapDevice {
    fn new() -> Self {
        // Acquire exclusive control of the Ethernet capsule
        TockSyscalls::command(DRIVER_NUM, DRIVER_LOCK_CMD, 0, 0).to_result::<(), ErrorCode>().unwrap();

        Self {
            rx_buffer: [0; ETH_MTU],
            tx_buffer: [0; ETH_MTU],
        }
    }

    fn try_receive(&mut self) -> Option<u16> {
        let upcall_called: Cell<Option<(u32,)>> = Cell::new(None);

        share::scope::<(AllowRw<TockSyscalls, DRIVER_NUM, 0>, Subscribe<TockSyscalls, DRIVER_NUM, RX_UPCALL_NO>), _, _>(
            |handle| -> Result<Option<u16>, ErrorCode> {
                let (allow_rw, subscribe) = handle.split();

                TockSyscalls::subscribe::<_, _, DefaultConfig, DRIVER_NUM, RX_UPCALL_NO>(subscribe, &upcall_called).unwrap();

                TockSyscalls::allow_rw::<DefaultConfig, DRIVER_NUM, 0>(allow_rw, &mut self.rx_buffer).unwrap();

                TockSyscalls::yield_wait();

                loop {
                    if let Some(packet_length) = upcall_called.get() {
                        TockSyscalls::command(DRIVER_NUM, ACK_RX_PKT_CMD, 0, 0)
                            .to_result::<(), ErrorCode>().unwrap();
                        return Ok(Some(packet_length.0 as u16));
                    }
                }
            }
        ).unwrap()
    }
}

impl<'a> smoltcp::phy::Device<'a> for TockTapDevice {
    type RxToken = TockTapRxToken<'a>;
    type TxToken = TockTapTxToken<'a>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let result = self.try_receive();

        if let Some(packet_length) = result {
            writeln!(Console::writer(), "Received packet with length = {}", packet_length).unwrap();
            Some((
                TockTapRxToken(&mut self.rx_buffer),
                TockTapTxToken(&mut self.tx_buffer),
            ))
        } else {
            writeln!(Console::writer(), "Empty packet received").unwrap();
            None
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(TockTapTxToken(&mut self.tx_buffer))
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let checksum_capabilities = smoltcp::phy::ChecksumCapabilities::ignored();

        let mut device_capabilities = smoltcp::phy::DeviceCapabilities::default();
        device_capabilities.medium = smoltcp::phy::Medium::Ethernet;
        device_capabilities.max_transmission_unit = ETH_MTU;
        device_capabilities.checksum = checksum_capabilities;

        device_capabilities
    }
}

fn smoltcp_now() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_millis
    (
        Alarm::current_instant::<Milliseconds>().unwrap().0
    )
}

#[allow(dead_code)]
fn send_icmp_ping_reply<'a>(
    identifier: u16,
    sequence_number: u16,
    data: &'a [u8],
    icmp_socket: &'a mut smoltcp::socket::IcmpSocket,
    remote_address: smoltcp::wire::IpAddress,
) -> (smoltcp::wire::Icmpv4Repr<'a>, smoltcp::wire::Icmpv4Packet<&'a mut [u8]>) {
    let icmp_repr = smoltcp::wire::Icmpv4Repr::EchoReply {
        ident: identifier,
        seq_no: sequence_number,
        data,
    };

    let icmp_payload = icmp_socket.send(icmp_repr.buffer_len(), remote_address).unwrap();

    let icmp_packet = smoltcp::wire::Icmpv4Packet::new_unchecked(icmp_payload);

    (icmp_repr, icmp_packet)
}

#[allow(dead_code)]
fn decode_icmp_ping_request(
    icmp_repr: smoltcp::wire::Icmpv4Repr,
) -> Option<(u16, u16, &[u8])> {
    if let smoltcp::wire::Icmpv4Repr::EchoRequest{ident, seq_no, data} = icmp_repr {
        return Some((ident, seq_no, data));
    }

    None
}

fn main() {
    let tock_tap_device = TockTapDevice::new();
    let _device_capabilities = tock_tap_device.capabilities();

    let ethernet_address = smoltcp::wire::EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
    let mut ip_addresses = [smoltcp::wire::IpCidr::new(smoltcp::wire::IpAddress::v4(192, 168, 1, 50), 24)];

    let mut icmp_rx_metadata = [smoltcp::socket::IcmpPacketMetadata::EMPTY; 1];
    let mut icmp_rx_payload = [0 as u8; ETH_MTU];
    let icmp_rx_buffer = smoltcp::socket::IcmpSocketBuffer::new(&mut icmp_rx_metadata[..], &mut icmp_rx_payload[..]);

    let mut icmp_tx_metadata = [smoltcp::socket::IcmpPacketMetadata::EMPTY; 1];
    let mut icmp_tx_payload = [0 as u8; ETH_MTU];
    let icmp_tx_buffer = smoltcp::socket::IcmpSocketBuffer::new(&mut icmp_tx_metadata[..], &mut icmp_tx_payload[..]);

    let icmp_socket = smoltcp::socket::IcmpSocket::new(icmp_rx_buffer, icmp_tx_buffer);

    const NUMBER_SOCKETS: usize = 1;
    let mut sockets = [smoltcp::iface::SocketStorage::EMPTY; NUMBER_SOCKETS];

    let mut neighbor_cache_storage = [None; 8];
    let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut neighbor_cache_storage[..]);

    let mut interface = smoltcp::iface::InterfaceBuilder::new(tock_tap_device, &mut sockets[..])
        .ip_addrs(&mut ip_addresses[..])
        .hardware_addr(ethernet_address.into())
        .neighbor_cache(neighbor_cache)
        .finalize();

    let icmp_socket_handle = interface.add_socket(icmp_socket);
    let mut _echo_payload = [0x0; 40];
    let identifier = 1234;
    let _remote_address = smoltcp::wire::IpAddress::v4(192, 168, 1, 1);

    let _reply_data: Option<(u16, u16, &[u8], smoltcp::wire::IpAddress)> = None;

    writeln!(Console::writer(), "Entering the main loop").unwrap();
    writeln!(Console::writer(), "ICMP Socket handle: {}", icmp_socket_handle).unwrap();

    loop {
        let timestamp = smoltcp_now();
        let result = interface.poll(timestamp);
        if result.is_err() {
            writeln!(Console::writer(), "Poll error").unwrap();
        }

        let icmp_socket: &mut smoltcp::socket::IcmpSocket = interface.get_socket(icmp_socket_handle);
        if !icmp_socket.is_open() {
            writeln!(Console::writer(), "The ICMP socket is closed").unwrap();
            icmp_socket.bind(smoltcp::socket::IcmpEndpoint::Ident(identifier)).unwrap();
        }

        //if reply_data.is_some() && icmp_socket.can_send() {
            //writeln!(Console::writer(), "Sending an ICMP echo reply").unwrap();

            //echo_payload[..8].copy_from_slice(&timestamp.secs().to_be_bytes());

            //let (icmp_repr, mut icmp_packet) = send_icmp_ping_reply(identifier, 0, &echo_payload, icmp_socket, remote_address);
            //icmp_repr.emit(&mut icmp_packet, &device_capabilities.checksum);
        //}

        if icmp_socket.can_recv() {
            writeln!(Console::writer(), "Received an ICMP echo request").unwrap();

            //let (payload, remote_address) = icmp_socket.recv().unwrap();

            //let icmp_packet = smoltcp::wire::Icmpv4Packet::new_unchecked(&payload);
            //let icmp_repr = smoltcp::wire::Icmpv4Repr::parse(&icmp_packet, &device_capabilities.checksum).unwrap();

            //if let Some((identifier, sequence_number, data)) = decode_icmp_ping_request(icmp_repr) {
                //reply_data = Some((identifier, sequence_number, data, remote_address));
            //}
        }

        Alarm::sleep_for(Milliseconds(2000)).unwrap();
    }
}

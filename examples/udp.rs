#![no_main]
#![no_std]

use libtock::alarm::{Alarm, Milliseconds};
use libtock::console::Console;
use libtock::runtime::{set_main, stack_size, TockSyscalls};

use libtock_platform::allow_ro::AllowRo;
use libtock_platform::allow_rw::AllowRw;
use libtock_platform::share;
use libtock_platform::subscribe::Subscribe;
use libtock_platform::DefaultConfig;
use libtock_platform::ErrorCode;
use libtock_platform::Syscalls;

use core::cell::Cell;
use core::cmp::Ordering;
use core::fmt::Write;

const ETH_MTU: usize = 1524;
const DRIVER_NUM: u32 = 0x30005;

// Commands
const DRIVER_LOCK_CMD: u32 = 1;
const SEND_PKT_CMD: u32 = 6;
const ACK_RX_PKT_CMD: u32 = 7;
const ACK_TX_PKT_CMD: u32 = 8;

// Upcalls
const RX_UPCALL_NO: u32 = 1;
const TX_UPCALL_NO: u32 = 2;

const BUFFER_NUM: u32 = 0;
const PACKET_IDENTIFIER: u32 = 0;

set_main! {main}
stack_size! {0x8000}

struct TockTapRxToken<'a> {
    buffer: &'a mut [u8; ETH_MTU],
    rx_upcall: Cell<Option<(u32,)>>,
}

impl<'a> TockTapRxToken<'a> {
    fn new(buffer: &'a mut [u8; ETH_MTU]) -> Self {
        Self {
            buffer,
            rx_upcall: Cell::default(),
        }
    }
}

impl smoltcp::phy::RxToken for TockTapRxToken<'_> {
    fn consume<R, F>(self, _timestamp: smoltcp::time::Instant, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        share::scope::<
            (
                AllowRw<TockSyscalls, DRIVER_NUM, BUFFER_NUM>,
                Subscribe<TockSyscalls, DRIVER_NUM, RX_UPCALL_NO>,
            ),
            _,
            _,
        >(|handle| {
            let (allow_rw, subscribe) = handle.split();
            TockSyscalls::subscribe::<_, _, DefaultConfig, DRIVER_NUM, RX_UPCALL_NO>(
                subscribe,
                &self.rx_upcall,
            )
            .unwrap();
            TockSyscalls::allow_rw::<DefaultConfig, DRIVER_NUM, BUFFER_NUM>(allow_rw, self.buffer)
                .unwrap();
            TockSyscalls::yield_wait();
            TockSyscalls::command(DRIVER_NUM, ACK_RX_PKT_CMD, 0, 0)
                .to_result::<(), ErrorCode>()
                .unwrap();
        });

        f(self.buffer)
    }
}

struct TockTapTxToken<'a> {
    buffer: &'a mut [u8; ETH_MTU],
    tx_upcall: Cell<Option<(u32, u32)>>,
}

impl<'a> TockTapTxToken<'a> {
    fn new(buffer: &'a mut [u8; ETH_MTU]) -> Self {
        Self {
            buffer,
            tx_upcall: Cell::default(),
        }
    }
}

impl smoltcp::phy::TxToken for TockTapTxToken<'_> {
    fn consume<R, F>(
        self,
        _timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        let result = f(&mut self.buffer[..len]);

        if result.is_ok() {
            share::scope::<
                (
                    AllowRo<TockSyscalls, DRIVER_NUM, BUFFER_NUM>,
                    Subscribe<TockSyscalls, DRIVER_NUM, TX_UPCALL_NO>,
                ),
                _,
                _,
            >(|handle| {
                let (allow_ro, subscribe) = handle.split();
                TockSyscalls::subscribe::<_, _, DefaultConfig, DRIVER_NUM, TX_UPCALL_NO>(
                    subscribe,
                    &self.tx_upcall,
                )
                .unwrap();
                TockSyscalls::allow_ro::<DefaultConfig, DRIVER_NUM, BUFFER_NUM>(
                    allow_ro,
                    self.buffer,
                )
                .unwrap();
                TockSyscalls::command(DRIVER_NUM, SEND_PKT_CMD, len as u32, PACKET_IDENTIFIER)
                    .to_result::<(), ErrorCode>()
                    .unwrap();
                TockSyscalls::yield_wait();
                //TockSyscalls::command(DRIVER_NUM, ACK_TX_PKT_CMD, 0, 0).to_result::<(), ErrorCode>().unwrap();
            })
        }

        result
    }
}

struct TockTapDevice {
    rx_buffer: [u8; ETH_MTU],
    tx_buffer: [u8; ETH_MTU],
    should_receive_packet: bool,
}

impl TockTapDevice {
    fn new() -> Self {
        Self {
            rx_buffer: [0; ETH_MTU],
            tx_buffer: [0; ETH_MTU],
            should_receive_packet: true,
        }
    }

    fn lock_driver(&self) {
        TockSyscalls::command(DRIVER_NUM, DRIVER_LOCK_CMD, 0, 0)
            .to_result::<(), ErrorCode>()
            .unwrap();
    }
}

impl<'a> smoltcp::phy::Device<'a> for TockTapDevice {
    type RxToken = TockTapRxToken<'a>;
    type TxToken = TockTapTxToken<'a>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        if self.should_receive_packet {
            self.should_receive_packet = false;
            Some((
                TockTapRxToken::new(&mut self.rx_buffer),
                TockTapTxToken::new(&mut self.tx_buffer),
            ))
        } else {
            self.should_receive_packet = true;
            None
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(TockTapTxToken::new(&mut self.tx_buffer))
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let checksum_capabilities = smoltcp::phy::ChecksumCapabilities::default();

        let mut device_capabilities = smoltcp::phy::DeviceCapabilities::default();
        device_capabilities.medium = smoltcp::phy::Medium::Ethernet;
        device_capabilities.max_transmission_unit = ETH_MTU;
        device_capabilities.checksum = checksum_capabilities;

        device_capabilities
    }
}

fn smoltcp_now() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_millis(Alarm::current_instant::<Milliseconds>().unwrap().0)
}

fn main() {
    let tock_tap_device = TockTapDevice::new();
    tock_tap_device.lock_driver();

    let ethernet_address = smoltcp::wire::EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
    let ip_address = smoltcp::wire::IpAddress::v4(192, 168, 1, 50);

    let mut ip_addresses = [smoltcp::wire::IpCidr::new(
        ip_address,
        24,
    )];

    let mut udp_rx_metadata = [smoltcp::socket::UdpPacketMetadata::EMPTY; 1];
    let mut udp_rx_payload = [0u8; ETH_MTU];
    let udp_rx_buffer =
        smoltcp::socket::UdpSocketBuffer::new(&mut udp_rx_metadata[..], &mut udp_rx_payload[..]);

    let mut udp_tx_metadata = [smoltcp::socket::UdpPacketMetadata::EMPTY; 1];
    let mut udp_tx_payload = [0u8; ETH_MTU];
    let udp_tx_buffer =
        smoltcp::socket::UdpSocketBuffer::new(&mut udp_tx_metadata[..], &mut udp_tx_payload[..]);

    let port = 1234u16;
    let mut udp_socket = smoltcp::socket::UdpSocket::new(udp_rx_buffer, udp_tx_buffer);
    udp_socket
        .bind(smoltcp::wire::IpEndpoint::new(ip_address, port))
        .unwrap();

    const NUMBER_SOCKETS: usize = 1;
    let mut sockets = [smoltcp::iface::SocketStorage::EMPTY; NUMBER_SOCKETS];

    let mut neighbor_cache_storage = [None; 8];
    let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut neighbor_cache_storage[..]);

    let mut interface = smoltcp::iface::InterfaceBuilder::new(tock_tap_device, &mut sockets[..])
        .ip_addrs(&mut ip_addresses[..])
        .hardware_addr(ethernet_address.into())
        .neighbor_cache(neighbor_cache)
        .finalize();

    let udp_socket_handle = interface.add_socket(udp_socket);

    #[derive(PartialEq)]
    enum State {
        WaitingForCommand,
        ShouldSayHello,
        ShouldSayBye,
    }

    let mut state = State::WaitingForCommand;
    let mut remote_ip_endpoint = None;

    loop {
        let timestamp = smoltcp_now();
        if false == interface.poll(timestamp).unwrap() {
            continue;
        }

        let udp_socket: &mut smoltcp::socket::UdpSocket = interface.get_socket(udp_socket_handle);
        if state == State::WaitingForCommand && udp_socket.can_recv() {
            let (data, ip_endpoint) = udp_socket.recv().unwrap();
            remote_ip_endpoint = Some(ip_endpoint);
            if data.cmp(b"hello\n") == Ordering::Equal {
                state = State::ShouldSayHello;
            } else if data.cmp(b"bye\n") == Ordering::Equal {
                state = State::ShouldSayBye;
            }
        }
        if state != State::WaitingForCommand && udp_socket.can_send() {
            if state == State::ShouldSayHello {
                udp_socket.send_slice(
                    "Hello from smoltcp".as_bytes(),
                    remote_ip_endpoint.unwrap()
                )
                .unwrap();
            } else if state == State::ShouldSayBye {
                udp_socket.send_slice(
                    "Bye from smoltcp".as_bytes(),
                    remote_ip_endpoint.unwrap()
                )
                .unwrap();
            }
            state = State::WaitingForCommand;
        }
    }
}

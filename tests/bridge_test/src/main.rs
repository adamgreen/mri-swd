/* Copyright (C) 2024  Adam Green (https://github.com/adamgreen)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
// Program which connects to the UART<->WiFi port on mri-swd and streams data to it for testing.
// The test expects the target to be running a simple UART loopback program so that all data sent over the WiFi bridge
// is sent back. It verifies this returned content.
use std::io::{Read, Write};

fn main() {
    println!("bridge_test - Testing mri-swd's UART<->WiFi Bridge");

    // Get the command line arguments and skip the first, which is just the executable name.
    let mut args = std::env::args();
    args.next();

    // Fetch the server address and port number string (ie. 10.0.0.201:2332) from the command line parameters.
    let server_address;
    if let Some(arg) = args.next() {
        server_address = arg;
    } else {
        panic!("Usage: bridge_test address:port");
    }

    println!("Attempting to connect to {server_address}");
    let tcp = std::net::TcpStream::connect(server_address);
    if let Err(err) = tcp {
        panic!("Failed TCP_IP connection - {err}");
    }
    println!("Connected!");

    // Fill in the buffer with known data to write it out to the stream.
    let mut write_buffer = [0u8; 256];
    for (i, elem) in write_buffer.iter_mut().enumerate() {
        *elem = i as u8;
    }

    let mut tcp = tcp.unwrap();
    tcp.set_read_timeout(Some(std::time::Duration::from_secs(5))).expect("Failed to set read timeout");

    for loop_number in 0..100 {
        // Send the buffer out over TCP/IP.
        tcp.write_all(&write_buffer).unwrap();

        // Read the return data.
        let mut read_buffer = [0u8; 256];
        tcp.read_exact(&mut read_buffer).expect("Failed to read bytes from mri-swd");

        // Verify the read data.
        if write_buffer != read_buffer {
            panic!("Unexpected data was read from socket. {write_buffer:?}");
        }
        println!("Loopback iteration #{} was valid", loop_number + 1);
    }

    println!("Test successful!");
}

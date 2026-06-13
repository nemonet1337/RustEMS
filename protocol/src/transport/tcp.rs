//! TCP transport for the rusEFI binary protocol.
//!
//! Connects to a TCP gateway (typically the Java console running in proxy mode
//! on port 29001) and wraps the connection in a [`FramedStream`].

use crate::io::FramedStream;
use anyhow::{Context, Result};
use tokio::net::TcpStream;

/// Default port used by the rusEFI Java console TCP bridge
pub const DEFAULT_PORT: u16 = 29001;

/// Connect to a rusEFI TCP gateway and return a [`FramedStream`] wrapping the
/// connection.
///
/// # Example
///
/// ```no_run
/// use rusefi_protocol::transport::tcp;
/// use rusefi_protocol::io::IoStream;
/// use rusefi_protocol::opcode::Command;
///
/// #[tokio::main(flavor = "current_thread")]
/// async fn main() -> anyhow::Result<()> {
///     let mut stream = tcp::connect("127.0.0.1", tcp::DEFAULT_PORT).await?;
///     stream.send_payload(&Command::Hello.to_payload()).await?;
///     let response = stream.recv_packet().await?;
///     println!("Response: {:?}", response);
///     Ok(())
/// }
/// ```
pub async fn connect(host: &str, port: u16) -> Result<FramedStream<TcpStream>> {
    let addr = format!("{}:{}", host, port);
    let tcp = TcpStream::connect(&addr)
        .await
        .with_context(|| format!("TCP connect to {addr} failed"))?;

    // Disable Nagle's algorithm for lower latency on small command packets
    tcp.set_nodelay(true)
        .with_context(|| "Failed to set TCP_NODELAY")?;

    tracing::info!("Connected to rusEFI gateway at {}", addr);
    Ok(FramedStream(tcp))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::IoStream;
    use crate::opcode::Command;
    use crate::packet::encode_packet_vec;
    use tokio::io::AsyncWriteExt;
    use tokio::net::TcpListener;

    /// Spin up a minimal echo server that sends back a fixed OK response,
    /// then verify the client can exchange a Hello packet.
    #[tokio::test]
    async fn tcp_hello_round_trip() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();

        // Server task: read one packet, echo back an OK response
        tokio::spawn(async move {
            let (mut sock, _) = listener.accept().await.unwrap();
            // Read length header
            let mut header = [0u8; 2];
            tokio::io::AsyncReadExt::read_exact(&mut sock, &mut header)
                .await
                .unwrap();
            let length = (u16::from(header[0]) << 8 | u16::from(header[1])) as usize;
            let mut rest = vec![0u8; length + 4];
            tokio::io::AsyncReadExt::read_exact(&mut sock, &mut rest)
                .await
                .unwrap();

            // Send back a one-byte OK response
            let response = encode_packet_vec(&[0x00]).unwrap();
            sock.write_all(&response).await.unwrap();
        });

        // Client
        let mut stream = connect("127.0.0.1", addr.port()).await.unwrap();
        stream
            .send_payload(&Command::Hello.to_payload())
            .await
            .unwrap();
        let response = stream.recv_packet().await.unwrap();
        assert_eq!(response, vec![0x00u8]);
    }
}

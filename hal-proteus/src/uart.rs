//! USART1 serial port for PC tuning (TunerStudio binary protocol).
//!
//! TX = PA9, RX = PA10, 115200 8N1, interrupt-driven (BufferedUart).

use embassy_stm32::usart::{BufferedUart, BufferedInterruptHandler, Config};
use embassy_stm32::{bind_interrupts, Peri};
use embassy_stm32::peripherals::{PA10, PA9, USART1};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USART1 => BufferedInterruptHandler<USART1>;
});

static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();

/// Initialise USART1 as a buffered UART for PC tuning.
///
/// Returns `None` if the UART configuration is invalid.
pub fn init(
    usart: Peri<'static, USART1>,
    rx: Peri<'static, PA10>,
    tx: Peri<'static, PA9>,
) -> Option<BufferedUart<'static>> {
    let tx_buf = TX_BUF.init([0u8; 256]);
    let rx_buf = RX_BUF.init([0u8; 256]);
    let mut config = Config::default();
    config.baudrate = 115_200;
    BufferedUart::new(usart, rx, tx, tx_buf, rx_buf, Irqs, config).ok()
}

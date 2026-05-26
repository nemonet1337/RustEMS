//! UAEFI board CAN bus driver using embassy-stm32 bxcan (CAN1).
//!
//! Uses a queue-based approach to bridge the async embassy CAN API
//! with the sync `CanBus` trait used by the engine-core control loop.

use rusefi_core::hal::{CanBus, CanFrame};
use embassy_stm32::can::{Can, Id, StandardId, ExtendedId};
use embassy_stm32::can::frame::Frame as EmbassyFrame;
use heapless::spsc::{Consumer, Producer, Queue};
use static_cell::StaticCell;

const CAN_QUEUE_SIZE: usize = 16;

static TX_QUEUE: StaticCell<Queue<CanFrame, CAN_QUEUE_SIZE>> = StaticCell::new();
static RX_QUEUE: StaticCell<Queue<CanFrame, CAN_QUEUE_SIZE>> = StaticCell::new();

pub struct Stm32CanDriver {
    tx_producer: Producer<'static, CanFrame>,
    rx_consumer: Consumer<'static, CanFrame>,
}

pub struct CanTaskResources {
    pub tx_consumer: Consumer<'static, CanFrame>,
    pub rx_producer: Producer<'static, CanFrame>,
}

impl Stm32CanDriver {
    pub fn init() -> (Self, CanTaskResources) {
        let tx_queue = TX_QUEUE.init(Queue::new());
        let rx_queue = RX_QUEUE.init(Queue::new());

        let (tx_prod, tx_cons) = tx_queue.split();
        let (rx_prod, rx_cons) = rx_queue.split();

        let driver = Self {
            tx_producer: tx_prod,
            rx_consumer: rx_cons,
        };
        let resources = CanTaskResources {
            tx_consumer: tx_cons,
            rx_producer: rx_prod,
        };
        (driver, resources)
    }
}

impl CanBus for Stm32CanDriver {
    fn transmit(&mut self, frame: &CanFrame) -> bool {
        self.tx_producer.enqueue(*frame).is_ok()
    }

    fn receive(&mut self) -> Option<CanFrame> {
        self.rx_consumer.dequeue()
    }
}

fn to_embassy_frame(frame: &CanFrame) -> Option<EmbassyFrame> {
    let id: Id = if frame.is_extended {
        let ext_id = ExtendedId::new(frame.id)?;
        Id::Extended(ext_id)
    } else {
        let std_id = StandardId::new(frame.id as u16)?;
        Id::Standard(std_id)
    };

    let dlc = frame.dlc as usize;
    EmbassyFrame::new_data(id, &frame.data[..dlc]).ok()
}

fn from_embassy_frame(frame: &EmbassyFrame) -> CanFrame {
    let (id, is_extended) = match *frame.id() {
        Id::Standard(sid) => (sid.as_raw() as u32, false),
        Id::Extended(eid) => (eid.as_raw(), true),
    };

    let data_slice = frame.data();
    let dlc = data_slice.len().min(8) as u8;
    let mut data = [0u8; 8];
    data[..dlc as usize].copy_from_slice(&data_slice[..dlc as usize]);

    CanFrame { id, is_extended, dlc, data }
}

pub async fn can_task(mut can: Can<'static>, mut resources: CanTaskResources) {
    can.set_bitrate(500_000);

    defmt::info!("UAEFI CAN1 task started @ 500 kbit/s");

    loop {
        while let Some(frame) = resources.tx_consumer.dequeue() {
            if let Some(embassy_frame) = to_embassy_frame(&frame) {
                can.write(&embassy_frame).await;
            } else {
                defmt::warn!("CAN TX: invalid frame dropped (id=0x{:X})", frame.id);
            }
        }

        match can.read().await {
            Ok(envelope) => {
                let core_frame = from_embassy_frame(&envelope.frame);
                if resources.rx_producer.enqueue(core_frame).is_err() {
                    defmt::warn!("CAN RX queue full, frame dropped");
                }
            }
            Err(e) => {
                defmt::warn!("CAN RX error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }
}

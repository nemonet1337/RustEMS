//! UAEFI Trigger Input Implementation
//!
//! Uses EXTI interrupts for crank and cam sensor inputs.
//! PA8 = Crank (VR/Hall), PA5 = Cam (Hall)

use rusefi_core::hal::TriggerInput;
use embassy_stm32::{Peri, bind_interrupts, exti, gpio::Pull};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::{PA5, PA8, EXTI5, EXTI8};
use heapless::spsc::{Consumer, Producer, Queue};

/// Trigger event timestamp (microseconds since boot).
pub type TriggerTimestamp = u64;

/// Queue capacity for trigger events.
const TRIGGER_QUEUE_SIZE: usize = 64;

/// Global queue for crank trigger events.
static CRANK_QUEUE: static_cell::StaticCell<Queue<TriggerTimestamp, TRIGGER_QUEUE_SIZE>> =
    static_cell::StaticCell::new();

/// Global queue for cam trigger events.
static CAM_QUEUE: static_cell::StaticCell<Queue<TriggerTimestamp, TRIGGER_QUEUE_SIZE>> =
    static_cell::StaticCell::new();

// PA5 (EXTI5) and PA8 (EXTI8) both use the shared EXTI9_5 interrupt on STM32F4.
bind_interrupts!(struct ExtiIrqs {
    EXTI9_5 => exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
});

/// UAEFI Trigger Input using EXTI interrupts.
pub struct Stm32TriggerInput {
    crank_consumer: Consumer<'static, TriggerTimestamp>,
    cam_consumer: Consumer<'static, TriggerTimestamp>,
}

/// Producers returned from init(), to be passed to the EXTI tasks.
pub struct TriggerProducers {
    pub crank: Producer<'static, TriggerTimestamp>,
    pub cam: Producer<'static, TriggerTimestamp>,
}

impl Stm32TriggerInput {
    /// Initialize the trigger input system.
    pub fn init() -> (Self, TriggerProducers) {
        let crank_queue = CRANK_QUEUE.init(Queue::new());
        let cam_queue = CAM_QUEUE.init(Queue::new());

        let (crank_prod, crank_cons) = crank_queue.split();
        let (cam_prod, cam_cons) = cam_queue.split();

        let input = Self {
            crank_consumer: crank_cons,
            cam_consumer: cam_cons,
        };

        let producers = TriggerProducers {
            crank: crank_prod,
            cam: cam_prod,
        };

        (input, producers)
    }
}

impl TriggerInput for Stm32TriggerInput {
    fn read_crank_timestamp(&mut self) -> Option<u64> {
        self.crank_consumer.dequeue()
    }

    fn read_cam_timestamp(&mut self) -> Option<u64> {
        self.cam_consumer.dequeue()
    }
}

/// EXTI task for crank sensor (PA8).
pub async fn crank_exti_task(
    pa8:   Peri<'static, PA8>,
    exti8: Peri<'static, EXTI8>,
    mut tx: Producer<'static, TriggerTimestamp>,
) {
    let mut pin = ExtiInput::new(pa8, exti8, Pull::Up, ExtiIrqs);

    defmt::info!("UAEFI crank EXTI task started on PA8");

    loop {
        pin.wait_for_rising_edge().await;

        let timestamp = embassy_time::Instant::now().as_micros();

        if tx.enqueue(timestamp).is_err() {
            defmt::warn!("Crank queue full, dropped timestamp");
        }

        pin.wait_for_falling_edge().await;
    }
}

/// EXTI task for cam sensor (PA5).
pub async fn cam_exti_task(
    pa5:   Peri<'static, PA5>,
    exti5: Peri<'static, EXTI5>,
    mut tx: Producer<'static, TriggerTimestamp>,
) {
    let mut pin = ExtiInput::new(pa5, exti5, Pull::Up, ExtiIrqs);

    defmt::info!("UAEFI cam EXTI task started on PA5");

    loop {
        pin.wait_for_rising_edge().await;

        let timestamp = embassy_time::Instant::now().as_micros();

        if tx.enqueue(timestamp).is_err() {
            defmt::warn!("Cam queue full, dropped timestamp");
        }

        pin.wait_for_falling_edge().await;
    }
}

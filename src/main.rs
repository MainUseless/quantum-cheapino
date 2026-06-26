#![no_std]
#![no_main]

mod keymap;
#[macro_use]
mod macros;
mod vial;

use core::convert::Infallible;

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Flex, InputConfig, Pull};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rng::TrngSource;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use rmk::ble::{BleTransport, build_ble_stack};
use rmk::config::{BehaviorConfig, PositionalConfig, RmkConfig, StorageConfig, VialConfig};
use rmk::debounce::default_debouncer::DefaultDebouncer;
use rmk::driver::flex_pin::FlexPin;
use rmk::host::HostService;
use rmk::keyboard::Keyboard;
use rmk::matrix::bidirectional_matrix::{BidirectionalMatrix, ScanLocation};
use rmk::storage::async_flash_wrapper;
use rmk::{HostResources, KeymapData, initialize_keymap_and_storage, run_all};

use crate::keymap::*;
use crate::vial::{VIAL_KEYBOARD_DEF, VIAL_KEYBOARD_ID};

::esp_bootloader_esp_idf::esp_app_desc!();

// Newtype wrapper: esp-hal Flex pin -> rmk FlexPin trait (orphan rule)
struct EspFlexPin<'d>(Flex<'d>);

impl ErrorType for EspFlexPin<'_> {
    type Error = Infallible;
}

impl InputPin for EspFlexPin<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_low())
    }
}

impl OutputPin for EspFlexPin<'_> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }
}

impl FlexPin for EspFlexPin<'_> {
    fn set_as_input(&mut self) {
        self.0.set_output_enable(false);
        self.0.apply_input_config(&InputConfig::default().with_pull(Pull::Down));
        self.0.set_input_enable(true);
    }
    fn set_as_output(&mut self) {
        self.0.set_input_enable(false);
        self.0.set_low();
        self.0.set_output_enable(true);
    }
}

// =============================================================================
// Cheapino bidirectional matrix scan map
// =============================================================================
//
// The Cheapino uses 12 GPIO pins wired to a 6x6 logical matrix through
// alternating diode directions. Each logical key position maps to a specific
// (input_pin, output_pin) pair — some scan forward (row→col), some reverse
// (col→row).
//
// Pin index assignments:
//   0: GPIO4  (row0)     6: GPIO0  (col0)
//   1: GPIO5  (row1)     7: GPIO1  (col1)
//   2: GPIO6  (row2)     8: GPIO3  (col2)
//   3: GPIO2  (row3)     9: GPIO7  (col3)
//   4: GPIO8  (row4)    10: GPIO10 (col4)
//   5: GPIO9  (row5)    11: GPIO20 (col5)
//
// ScanLocation::Pins(in_pin_idx, out_pin_idx)
//   out_pin drives HIGH, in_pin reads (with pull-down)

use ScanLocation::Pins;

const SCAN_MAP: [[ScanLocation; COL]; ROW] = [
    // Row 0 (left): Q  W  E  R  T  Space
    [Pins(3,11), Pins(10,3), Pins(3,10), Pins(9,3), Pins(3,9), Pins(11,3)],
    // Row 1 (left): A  S  D  F  G  Tab
    [Pins(4,11), Pins(10,4), Pins(4,10), Pins(9,4), Pins(4,9), Pins(11,4)],
    // Row 2 (left): Z  X  C  V  B  LCtrl
    [Pins(5,11), Pins(10,5), Pins(5,10), Pins(9,5), Pins(5,9), Pins(11,5)],
    // Row 3 (right): Y  U  I  O  P  Enter
    [Pins(0,6),  Pins(6,0),  Pins(0,7),  Pins(7,0),  Pins(0,8),  Pins(8,0)],
    // Row 4 (right): H  J  K  L  ;  Backspace
    [Pins(1,6),  Pins(6,1),  Pins(1,7),  Pins(7,1),  Pins(1,8),  Pins(8,1)],
    // Row 5 (right): N  M  ,  .  /  LAlt
    [Pins(2,6),  Pins(6,2),  Pins(2,7),  Pins(7,2),  Pins(2,8),  Pins(8,2)],
];

#[esp_rtos::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);
    let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
    let mut rng = esp_hal::rng::Trng::try_new().unwrap();

    // BLE
    let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 64> = ExternalController::new(connector);
    let central_addr = [0x18, 0xe2, 0x21, 0x80, 0xc0, 0xc7];
    let mut host_resources = HostResources::new();
    let stack = build_ble_stack(controller, central_addr, &mut rng, &mut host_resources).await;

    // Flash
    let flash = FlashStorage::new(peripherals.FLASH);
    let flash = async_flash_wrapper(flash);

    // All 12 GPIO pins as flexible I/O for bidirectional scanning
    let pins: [EspFlexPin; 12] = [
        EspFlexPin(Flex::new(peripherals.GPIO4)),   // 0: row0
        EspFlexPin(Flex::new(peripherals.GPIO5)),   // 1: row1
        EspFlexPin(Flex::new(peripherals.GPIO6)),   // 2: row2
        EspFlexPin(Flex::new(peripherals.GPIO2)),   // 3: row3
        EspFlexPin(Flex::new(peripherals.GPIO8)),   // 4: row4
        EspFlexPin(Flex::new(peripherals.GPIO9)),   // 5: row5
        EspFlexPin(Flex::new(peripherals.GPIO0)),   // 6: col0
        EspFlexPin(Flex::new(peripherals.GPIO1)),   // 7: col1
        EspFlexPin(Flex::new(peripherals.GPIO3)),   // 8: col2
        EspFlexPin(Flex::new(peripherals.GPIO7)),   // 9: col3
        EspFlexPin(Flex::new(peripherals.GPIO10)),  // 10: col4
        EspFlexPin(Flex::new(peripherals.GPIO20)),  // 11: col5
    ];

    // RMK config
    let vial_config = VialConfig::new(VIAL_KEYBOARD_ID, VIAL_KEYBOARD_DEF, &[(0, 0), (1, 1)]);
    let storage_config = StorageConfig {
        start_addr: 0x3f0000,
        num_sectors: 16,
        ..Default::default()
    };
    let rmk_config = RmkConfig {
        vial_config,
        storage_config,
        ..Default::default()
    };

    // Keymap + storage
    let mut keymap_data = KeymapData::new(keymap::get_default_keymap());
    let mut behavior_config = BehaviorConfig::default();
    let per_key_config = PositionalConfig::default();
    let (keymap, mut storage) = initialize_keymap_and_storage(
        &mut keymap_data,
        flash,
        &storage_config,
        &mut behavior_config,
        &per_key_config,
    )
    .await;

    // Bidirectional matrix for Cheapino's alternating-diode layout
    let debouncer = DefaultDebouncer::new();
    let mut matrix = BidirectionalMatrix::<_, _, 12, ROW, COL>::new(pins, debouncer, SCAN_MAP);

    let mut keyboard = Keyboard::new(&keymap);
    let host_ctx = rmk::host::KeyboardContext::new(&keymap);
    let mut host_service = HostService::new(&host_ctx, &rmk_config);
    let mut ble_transport = BleTransport::new(&stack, rmk_config).await;

    run_all!(matrix, storage, ble_transport, keyboard, host_service).await;
}

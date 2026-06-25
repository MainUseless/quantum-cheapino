#![no_std]
#![no_main]

use esp_backtrace as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::{
    gpio::{Input, Output, Pull, Level, Speed, AnyPin, Pin},
    peripherals::Peripherals,
};
use rmk::{
    config::RmkConfig,
    initialize_keyboard_with_config,
    matrix::KeyState,
};

// Define matrix dimensions (Cheapino maps a physical 6x6 grid)
const ROWS: usize = 6;
const COLS: usize = 6;

// Define your keymap layers here. 
// Standard RMK layer array: [Layers][Rows][Columns]
// Replace the numeric placeholders with your actual RMK keycodes (e.g., KeyCode::A)
static KEYMAP: [[[u16; COLS]; ROWS]; 1] = [
    [
        [1, 2, 3, 4, 5, 6],
        [7, 8, 9, 10, 11, 12],
        [13, 14, 15, 16, 17, 18],
        [19, 20, 21, 22, 23, 24],
        [25, 26, 27, 28, 29, 30],
        [31, 32, 33, 34, 35, 36],
    ]
];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize peripherals using the Embassy HAL
    let peripherals = hal::init(hal::Config::default());

    // 1. Group the physical GPIO pins exactly from your configuration
    let row_gpios = [
        peripherals.GPIO4.degrade(),
        peripherals.GPIO5.degrade(),
        peripherals.GPIO6.degrade(),
        peripherals.GPIO2.degrade(),
        peripherals.GPIO8.degrade(),
        peripherals.GPIO9.degrade(),
    ];

    let col_gpios = [
        peripherals.GPIO0.degrade(),
        peripherals.GPIO1.degrade(),
        peripherals.GPIO3.degrade(),
        peripherals.GPIO7.degrade(),
        peripherals.GPIO10.degrade(),
        peripherals.GPIO20.degrade(),
    ];

    // 2. Setup baseline RMK Configurations for BLE
    let config = RmkConfig::default();

    // 3. Start the core RMK Engine 
    // We pass an empty matrix configuration because we are feeding it manually below
    let (mut keyboard, _) = initialize_keyboard_with_config(KEYMAP, config).await;

    // 4. Custom Alternating Scan Loop
    // This actively flips the pins between Inputs and Outputs to catch both diode directions
    let mut matrix_state = [[KeyState::default(); COLS]; ROWS];

    loop {
        // --- PASS 1: Scan Forward Diodes (Rows as Outputs, Cols as Inputs) ---
        let mut outputs: [Output<'_, AnyPin>; ROWS] = row_gpios.iter().map(|pin| {
            Output::new(pin.clone(), Level::High, Speed::Low)
        }).collect::<core::prelude::v1::Vec<_, ROWS>>().into_inner().unwrap();

        let inputs: [Input<'_, AnyPin>; COLS] = col_gpios.iter().map(|pin| {
            Input::new(pin.clone(), Pull::Up)
        }).collect::<core::prelude::v1::Vec<_, COLS>>().into_inner().unwrap();

        for r in 0..ROWS {
            outputs[r].set_low(); // Activate current row
            Timer::after_micros(5).await; // Settle electrical noise

            for c in 0..COLS {
                // If input is low, the forward diode switch is physically closed
                if inputs[c].is_low() {
                    matrix_state[r][c].pressed = true;
                }
            }
            outputs[r].set_high(); // Deactivate row
        }

        // Clean up Pass 1 configuration explicitly
        drop(outputs);
        drop(inputs);

        // --- PASS 2: Scan Reversed Diodes (Cols as Outputs, Rows as Inputs) ---
        let mut outputs: [Output<'_, AnyPin>; COLS] = col_gpios.iter().map(|pin| {
            Output::new(pin.clone(), Level::High, Speed::Low)
        }).collect::<core::prelude::v1::Vec<_, COLS>>().into_inner().unwrap();

        let inputs: [Input<'_, AnyPin>; ROWS] = row_gpios.iter().map(|pin| {
            Input::new(pin.clone(), Pull::Up)
        }).collect::<core::prelude::v1::Vec<_, ROWS>>().into_inner().unwrap();

        for c in 0..COLS {
            outputs[c].set_low(); // Activate current column as an output
            Timer::after_micros(5).await;

            for r in 0..ROWS {
                // If row input reads low, the reversed diode switch is physically closed
                if inputs[r].is_low() {
                    // Map it carefully into your secondary layout coordinates
                    matrix_state[r][c].pressed = true; 
                }
            }
            outputs[c].set_high();
        }

        // Clean up Pass 2 configuration explicitly
        drop(outputs);
        drop(inputs);

        // 5. Submit the combined matrix states straight into the RMK core handler
        keyboard.set_matrix_state(&matrix_state).await;

        // Reset tracking matrix states for the next loop execution
        for r in 0..ROWS {
            for c in 0..COLS {
                matrix_state[r][c].pressed = false;
            }
        }

        // Delay to prevent pinned CPU thrashing and stabilize keyboard polling rate (1000Hz)
        Timer::after_millis(1).await;
    }
}
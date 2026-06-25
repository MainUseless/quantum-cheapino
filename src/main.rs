#![no_std]
#![no_main]

use core::convert::Infallible;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Flex, Pull},
    init,
};
use rmk::{
    device::InputDevice,
    event::KeyEvent,
};

const ROWS: usize = 6;
const COLS: usize = 6;

struct CheapinoDuplexMatrix<'d> {
    row_pins: [Flex<'d>; ROWS],
    col_pins: [Flex<'d>; COLS],
    prev_state: [[bool; COLS]; ROWS],
    events: heapless::spsc::Queue<KeyEvent, 16>,
}

impl<'d> CheapinoDuplexMatrix<'d> {
    async fn scan_matrix(&mut self) {
        let mut current_state = [[false; COLS]; ROWS];

        // PASS 1: Rows as Outputs (Drive Low), Cols as Inputs
        for r in 0..ROWS {
            self.row_pins[r].set_as_output();
            self.row_pins[r].set_low();
            Timer::after_micros(5).await;

            for c in 0..COLS {
                self.col_pins[c].set_as_input(Pull::Up);
                if self.col_pins[c].is_low() {
                    current_state[r][c] = true;
                }
            }
            self.row_pins[r].set_high();
            self.row_pins[r].set_as_input(Pull::Up);
        }

        // PASS 2: Cols as Outputs (Drive Low), Rows as Inputs
        for c in 0..COLS {
            self.col_pins[c].set_as_output();
            self.col_pins[c].set_low();
            Timer::after_micros(5).await;

            for r in 0..ROWS {
                self.row_pins[r].set_as_input(Pull::Up);
                if self.row_pins[r].is_low() {
                    current_state[r][c] = true;
                }
            }
            self.col_pins[c].set_high();
            self.col_pins[c].set_as_input(Pull::Up);
        }

        for r in 0..ROWS {
            for c in 0..COLS {
                if current_state[r][c] != self.prev_state[r][c] {
                    let _ = self.events.enqueue(KeyEvent {
                        row: r as u8,
                        col: c as u8,
                        pressed: current_state[r][c],
                    });
                    self.prev_state[r][c] = current_state[r][c];
                }
            }
        }
    }
}

impl<'d> InputDevice for CheapinoDuplexMatrix<'d> {
    type Error = Infallible;

    async fn read(&mut self) -> Result<KeyEvent, Self::Error> {
        loop {
            if let Some(event) = self.events.dequeue() {
                return Ok(event);
            }
            self.scan_matrix().await;
            Timer::after_millis(1).await;
        }
    }
}

#[rmk::macros::rmk_keyboard]
mod keyboard {
    // Note: You will need to check the RMK 0.8.x documentation on how to 
    // specifically pass your custom `CheapinoDuplexMatrix` InputDevice into this macro.
}

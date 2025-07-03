#![no_std]

extern crate alloc;

pub use phytium_mci::{IoPad, Kernel, PAD_ADDRESS, sd::SdCard, set_impl};
// pub use phytium_mci::{set_dma_impl, DmaImpl, DmaDirection};

use log::trace;

use alloc::{format, vec::Vec};
use core::{cell::UnsafeCell, cmp};
use phytium_mci::mci_host::err::MCIHostError;
use rdrive::{DriverGeneric, block::*};

const BLOCK_SIZE: usize = 512;

pub struct SdCardDriver {
    sdcard: UnsafeCell<SdCard>,
}

impl SdCardDriver {
    pub fn new(sd_card: SdCard) -> Self {
        SdCardDriver {
            sdcard: UnsafeCell::new(sd_card),
        }
    }

    pub fn mbr_data(&mut self) -> Result<[u8; BLOCK_SIZE], io::Error> {
        let mut buffer = [0u8; BLOCK_SIZE];
        self.read_block(0, &mut buffer)?;

        Ok(buffer)
    }

    #[inline]
    fn validate_buffer_alignment(buffer: &[u8]) -> Result<(), io::Error> {
        if buffer.len() < BLOCK_SIZE {
            return Err(io::Error {
                kind: io::ErrorKind::InvalidParameter { name: "buffer" },
                success_pos: 0,
            });
        }

        let (prefix, _, suffix) = unsafe { buffer.align_to::<u32>() };
        if !prefix.is_empty() || !suffix.is_empty() {
            return Err(io::Error {
                kind: io::ErrorKind::InvalidParameter { name: "buffer" },
                success_pos: 0,
            });
        }

        Ok(())
    }
}

unsafe impl Send for SdCardDriver {}
unsafe impl Sync for SdCardDriver {}

impl DriverGeneric for SdCardDriver {
    fn open(&mut self) -> Result<(), KError> {
        Ok(())
    }

    fn close(&mut self) -> Result<(), KError> {
        Ok(())
    }
}

fn deal_mci_host_err(err: MCIHostError) -> io::Error {
    let kind = match err {
        MCIHostError::Timeout => io::ErrorKind::TimedOut,
        MCIHostError::OutOfRange | MCIHostError::InvalidArgument => {
            io::ErrorKind::InvalidParameter { name: "parameter" }
        }
        MCIHostError::NoTransferInProgress | MCIHostError::NoData => {
            io::ErrorKind::InvalidParameter { name: "state" }
        }
        MCIHostError::TransferFailed | MCIHostError::SetCardBlockSizeFailed => {
            io::ErrorKind::InvalidParameter { name: "transfer" }
        }
        MCIHostError::AllSendCidFailed
        | MCIHostError::SendRelativeAddressFailed
        | MCIHostError::SendCsdFailed
        | MCIHostError::SelectCardFailed
        | MCIHostError::SendScrFailed => io::ErrorKind::InvalidParameter {
            name: "card_command",
        },
        MCIHostError::SetDataBusWidthFailed | MCIHostError::SwitchBusTimingFailed => {
            io::ErrorKind::InvalidParameter { name: "bus_config" }
        }
        MCIHostError::GoIdleFailed | MCIHostError::HandShakeOperationConditionFailed => {
            io::ErrorKind::InvalidParameter { name: "card_init" }
        }
        MCIHostError::SendApplicationCommandFailed | MCIHostError::SwitchFailed => {
            io::ErrorKind::InvalidParameter { name: "command" }
        }
        MCIHostError::StopTransmissionFailed | MCIHostError::WaitWriteCompleteFailed => {
            io::ErrorKind::InvalidParameter {
                name: "transmission",
            }
        }
        MCIHostError::InvalidVoltage
        | MCIHostError::SwitchVoltageFail
        | MCIHostError::SwitchVoltage18VFail33VSuccess => {
            io::ErrorKind::InvalidParameter { name: "voltage" }
        }
        MCIHostError::SdioResponseError
        | MCIHostError::SdioInvalidArgument
        | MCIHostError::SdioSendOperationConditionFail
        | MCIHostError::SdioSwitchHighSpeedFail
        | MCIHostError::SdioReadCISFail
        | MCIHostError::SdioInvalidCard => io::ErrorKind::InvalidParameter { name: "sdio" },
        MCIHostError::TuningFail | MCIHostError::ReTuningRequest => {
            io::ErrorKind::InvalidParameter { name: "tuning" }
        }
        MCIHostError::CardDetectFailed | MCIHostError::CardInitFailed => {
            io::ErrorKind::InvalidParameter {
                name: "card_detect",
            }
        }
        MCIHostError::CardStatusIdle => io::ErrorKind::InvalidParameter { name: "card_idle" },
        MCIHostError::PollingCardIdleFailed | MCIHostError::DeselectCardFailed => {
            io::ErrorKind::InvalidParameter {
                name: "card_control",
            }
        }

        _ => io::ErrorKind::Other(format!("{:?}", err).into()),
    };

    io::Error {
        kind,
        success_pos: 0,
    }
}

impl Interface for SdCardDriver {
    /// Reads a block of data from the SD card.
    ///
    /// # Arguments
    /// * `block_id` - The block identifier to read from
    /// * `buffer` - Mutable buffer to store the read data (must be >= BLOCK_SIZE and u32-aligned)
    ///
    /// # Returns
    /// * `Ok(())` on success
    /// * `Err(io::Error)` if buffer is invalid or read operation fails
    fn read_block(&mut self, block_id: usize, buffer: &mut [u8]) -> Result<(), io::Error> {
        trace!("read block {}", block_id);
        let actual_block_id = block_id;

        Self::validate_buffer_alignment(buffer)?;

        let (_, aligned_buf, _) = unsafe { buffer.align_to_mut::<u32>() };

        let mut temp_buf: Vec<u32> = Vec::with_capacity(aligned_buf.len());

        let sd_card = unsafe { &mut *self.sdcard.get() };
        sd_card
            .read_blocks(&mut temp_buf, actual_block_id as u32, 1)
            .map_err(deal_mci_host_err)?;

        let copy_len = cmp::min(temp_buf.len(), aligned_buf.len());
        aligned_buf[..copy_len].copy_from_slice(&temp_buf[..copy_len]);

        Ok(())
    }

    /// Writes a block of data to the SD card.
    ///
    /// # Arguments
    /// * `block_id` - The block identifier to write to
    /// * `buffer` - Buffer containing data to write (must be >= BLOCK_SIZE and u32-aligned)
    ///
    /// # Returns
    /// * `Ok(())` on success
    /// * `Err(io::Error)` if buffer is invalid or write operation fails
    fn write_block(&mut self, block_id: usize, buffer: &[u8]) -> Result<(), io::Error> {
        trace!("write block {}", block_id);
        let actual_block_id = block_id;

        Self::validate_buffer_alignment(buffer)?;

        let (_, aligned_buf, _) = unsafe { buffer.align_to::<u32>() };

        let sd_card = unsafe { &mut *self.sdcard.get() };

        let mut write_buf: Vec<u32> = aligned_buf.to_vec();
        sd_card
            .write_blocks(&mut write_buf, actual_block_id as u32, 1)
            .map_err(deal_mci_host_err)?;

        Ok(())
    }

    /// flushes any buffered data to the SD card.
    fn flush(&mut self) -> Result<(), io::Error> {
        Ok(())
    }

    /// Returns the number of blocks on the SD card.
    #[inline]
    fn num_blocks(&self) -> usize {
        let sd_card = unsafe { &*self.sdcard.get() };
        sd_card.block_count() as usize
    }

    /// Returns the block size in bytes.
    #[inline]
    fn block_size(&self) -> usize {
        let sd_card = unsafe { &*self.sdcard.get() };
        sd_card.block_size() as usize
    }
}

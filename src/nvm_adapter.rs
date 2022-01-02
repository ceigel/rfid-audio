use crate::hal::flash::{self, WriteErase};
use crate::playing_memento::{PlayingMemento, PLAYING_MEMENTO_DATA_SIZE};
use log::{error, info};

pub const PAGE_SIZE: usize = 2048;
const FLASH_PAGE_ID: usize = 127;
const MAGIC: u8 = 42;
const PLAYLIST_FLASH_MEM_SIZE: usize = PLAYING_MEMENTO_DATA_SIZE + 1; // 1 byte magic, 23 bytes payload
const FLASH_MEM_QUAD: usize = PLAYLIST_FLASH_MEM_SIZE / 8;

pub struct NvmAdapter {
    sr: flash::SR,
    cr: flash::CR,
    keyr: flash::KEYR,
}

impl NvmAdapter {
    pub fn new(keyr: flash::KEYR, sr: flash::SR, cr: flash::CR) -> Self {
        Self { sr, cr, keyr }
    }

    pub fn read_memento() -> (Option<PlayingMemento>, Option<usize>) {
        let page = flash::FlashPage(127);
        let mut addr = page.to_address();
        let orig_addr = addr;
        let end_addr = page.to_address() + PAGE_SIZE;
        let mut last_good_addr = addr;
        let mut has_memento: bool = false;
        while addr < end_addr {
            let byte_val = unsafe { core::ptr::read(addr as *const u8) };
            if byte_val != MAGIC {
                break;
            }
            has_memento = true;
            last_good_addr = addr;
            addr += PLAYLIST_FLASH_MEM_SIZE;
        }
        if has_memento {
            let buffer: [u8; PLAYING_MEMENTO_DATA_SIZE] = unsafe { core::ptr::read((last_good_addr + 1) as *const [u8; PLAYING_MEMENTO_DATA_SIZE]) };
            let memento_offset = (addr + PLAYING_MEMENTO_DATA_SIZE < end_addr).then(|| addr - orig_addr);
            let playing_memento = PlayingMemento::deserialize(&buffer)
                .map_err(|err| error!("Error from_flash: {:?}", err))
                .ok();
            (playing_memento, memento_offset)
        } else {
            (None, None)
        }
    }

    pub fn write_memento(&mut self, memento: &PlayingMemento, write_offset: Option<usize>) -> Option<usize> {
        let mut data: [u64; 3] = [0; 3];
        Self::memento_to_flash(memento, &mut data);

        let mut writer = || -> Result<Option<usize>, flash::Error> {
            let mut prog = self.keyr.unlock_flash(&mut self.sr, &mut self.cr)?;
            let page = flash::FlashPage(FLASH_PAGE_ID);
            let write_offset = match write_offset {
                Some(write_offset) => write_offset,
                None => {
                    prog.erase_page(page)?;
                    0usize
                }
            };
            let write_address = page.to_address() + write_offset;
            prog.write_native(write_address, &data)?;
            let next_write_offset = write_offset + PLAYLIST_FLASH_MEM_SIZE;
            Ok((next_write_offset + PLAYLIST_FLASH_MEM_SIZE < PAGE_SIZE).then(|| next_write_offset))
        };

        match writer() {
            Ok(next_offset) => {
                info!("Memento successfully written");
                next_offset
            }
            Err(err) => {
                error!("Error saving memento: {:?}", err);
                None
            }
        }
    }

    fn memento_to_flash(memento: &PlayingMemento, buffer: &mut [u64; FLASH_MEM_QUAD]) {
        let slice_size = buffer.len() * 8; // quad = 8 bytes
        info!("buffer: {:?}", buffer.as_ptr());
        let buf_slice: &mut [u8] = unsafe { &mut *core::ptr::slice_from_raw_parts_mut(buffer.as_mut_ptr() as *mut u8, slice_size) };
        info!("buf_slice: {:?}", buf_slice.as_ptr());
        buf_slice[0] = MAGIC;
        let mut memento_arr: [u8; PLAYING_MEMENTO_DATA_SIZE] = [0; PLAYING_MEMENTO_DATA_SIZE];
        memento.serialize(&mut memento_arr);
        buf_slice[1..].copy_from_slice(&memento_arr);
    }
}

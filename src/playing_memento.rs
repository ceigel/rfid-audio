use crate::flash;
use crate::playlist::{PlaylistName, PLAYLIST_NAME_LEN};
use core::convert::TryInto;
use core::str;
use embedded_sdmmc::{FilenameError, ShortFileName};

pub struct PlayingMemento {
    pub playlist: PlaylistName, // 8 bytes
    pub file: ShortFileName,    // 11 bytes
    pub offset: u32,            // 4 bytes
}

const MAGIC: u8 = 42;
const PLAYING_MEMENTO_DATA_SIZE: usize = 23;
const PLAYLIST_FLASH_MEM_SIZE: usize = PLAYING_MEMENTO_DATA_SIZE + 1; // 1 byte magic, 23 bytes payload
const PAGE_SIZE: usize = 2048;
const FILE_MEM_OFFSET: usize = PLAYLIST_NAME_LEN;
const OFFSET_MEM_OFFSET: usize = FILE_MEM_OFFSET + core::mem::size_of::<ShortFileName>();

impl PlayingMemento {
    pub fn new(playlist: PlaylistName, file: ShortFileName, offset: u32) -> Self {
        Self { playlist, file, offset }
    }

    pub fn from_flash(page: flash::FlashPage) -> Option<(PlayingMemento, usize)> {
        let mut addr = page.to_address();
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
            let playlist = PlaylistName::from_bytes(&buffer[0..PLAYLIST_NAME_LEN]);
            let file_name = str::from_utf8(&buffer[FILE_MEM_OFFSET..OFFSET_MEM_OFFSET]);
            let offset_arr: [u8; 4] = *&buffer[OFFSET_MEM_OFFSET..].try_into().unwrap();
            let offset = u32::from_be_bytes(offset_arr);
            file_name
                .ok()
                .and_then(|fnm| ShortFileName::create_from_str(fnm).ok())
                .map(|file| (PlayingMemento { playlist, file, offset }, last_good_addr))
        } else {
            None
        }
    }

    pub fn same_file(&self, other: &PlayingMemento) -> bool {
        self.playlist == other.playlist && self.file == other.file
    }

    pub fn update_offset(&mut self, new_offset: u32) {
        self.offset = new_offset;
    }

    pub fn serialize(&self, bytes: &mut [u8; PLAYLIST_NAME_LEN + PLAYLIST_NAME_LEN + (u32::BITS as usize) / 8]) {
        bytes[0..PLAYLIST_NAME_LEN].copy_from_slice(&self.playlist.name[0..PLAYLIST_NAME_LEN]);
        bytes[PLAYLIST_NAME_LEN..PLAYLIST_NAME_LEN * 2].copy_from_slice(&self.file.base_name()[0..PLAYLIST_NAME_LEN]);
        bytes[PLAYLIST_NAME_LEN * 2..].copy_from_slice(&self.offset.to_be_bytes()[..]);
    }

    pub fn deserialize(&mut self, bytes: &[u8; PLAYLIST_NAME_LEN + PLAYLIST_NAME_LEN + (u32::BITS as usize) / 8]) -> Result<(), FilenameError> {
        self.playlist.name[0..PLAYLIST_NAME_LEN].copy_from_slice(&bytes[0..PLAYLIST_NAME_LEN]);
        let name = str::from_utf8(&bytes[PLAYLIST_NAME_LEN..PLAYLIST_NAME_LEN * 2]).map_err(|_| FilenameError::Utf8Error)?;
        self.file = ShortFileName::create_from_str(name)?;
        let u32_bytes = bytes[PLAYLIST_NAME_LEN * 2..].try_into();
        self.offset = u32_bytes.map(|bytes| u32::from_be_bytes(bytes)).unwrap_or(0);
        Ok(())
    }
}

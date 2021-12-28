use crate::playlist::{PlaylistName, PLAYLIST_NAME_LEN};
use core::convert::TryInto;
use core::str;
use embedded_sdmmc::{FilenameError, ShortFileName};

pub struct PlayingMemento {
    pub playlist: PlaylistName, // 8 bytes
    pub file: ShortFileName,    // 11 bytes
    pub offset: u32,            // 4 bytes
}

pub const PLAYING_MEMENTO_DATA_SIZE: usize = 23;
// Memory can be written only 8 bytes at a time 3x8 = 24
const FILE_MEM_OFFSET: usize = PLAYLIST_NAME_LEN;
const OFFSET_MEM_OFFSET: usize = FILE_MEM_OFFSET + core::mem::size_of::<ShortFileName>();

impl PlayingMemento {
    pub fn new(playlist: PlaylistName, file: ShortFileName, offset: u32) -> Self {
        Self { playlist, file, offset }
    }

    pub fn same_file(&self, other: &PlayingMemento) -> bool {
        self.playlist == other.playlist && self.file == other.file
    }

    pub fn update_offset(&mut self, new_offset: u32) {
        self.offset = new_offset;
    }

    pub fn serialize(&self, bytes: &mut [u8; PLAYING_MEMENTO_DATA_SIZE]) {
        bytes[0..PLAYLIST_NAME_LEN].copy_from_slice(&self.playlist.name[..]);
        bytes[FILE_MEM_OFFSET..OFFSET_MEM_OFFSET].copy_from_slice(&self.file.base_name()[..]);
        bytes[OFFSET_MEM_OFFSET..].copy_from_slice(&self.offset.to_be_bytes()[..]);
    }

    pub fn deserialize(buffer: &[u8; PLAYING_MEMENTO_DATA_SIZE]) -> Result<Self, FilenameError> {
        let playlist = PlaylistName::from_bytes(&buffer[0..PLAYLIST_NAME_LEN]);
        let file_name = str::from_utf8(&buffer[FILE_MEM_OFFSET..OFFSET_MEM_OFFSET]).map_err(|_| FilenameError::Utf8Error);
        let offset_bytes = *&buffer[OFFSET_MEM_OFFSET..].try_into();
        let offset = offset_bytes.map(|bytes| u32::from_be_bytes(bytes)).unwrap_or(0);
        let playing_memento = file_name
            .and_then(|fnm| ShortFileName::create_from_str(fnm))
            .map(|file| PlayingMemento { playlist, file, offset });

        playing_memento
    }
}

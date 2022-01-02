use crate::playlist::{PlaylistName, PLAYLIST_NAME_LEN};
use core::convert::TryInto;
use core::str;
use embedded_sdmmc::{FilenameError, ShortFileName};
use log::info;

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
        info!("serialize bytes: {:?}", bytes[..].as_ptr());
        bytes[0..PLAYLIST_NAME_LEN].copy_from_slice(&self.playlist.name[..]);
        let base_name = self.file.base_name();
        let extension = self.file.extension();
        let base_name_end = FILE_MEM_OFFSET + base_name.len();
        bytes[FILE_MEM_OFFSET..base_name_end].copy_from_slice(base_name);
        let extension_begin = OFFSET_MEM_OFFSET - extension.len();
        bytes[extension_begin..OFFSET_MEM_OFFSET].copy_from_slice(extension);
        bytes[OFFSET_MEM_OFFSET..].copy_from_slice(&self.offset.to_be_bytes()[..]);
    }

    pub fn deserialize(buffer: &[u8; PLAYING_MEMENTO_DATA_SIZE]) -> Result<Self, FilenameError> {
        let playlist = PlaylistName::from_bytes(&buffer[0..PLAYLIST_NAME_LEN]);
        let mut file_name_buffer = [0u8; 12];
        file_name_buffer[0..8].copy_from_slice(
            buffer[FILE_MEM_OFFSET..(OFFSET_MEM_OFFSET - 3)]
                .split(|x| *x == 0u8)
                .next()
                .unwrap_or_default(),
        );
        file_name_buffer[8] = b'.';
        file_name_buffer[9..].copy_from_slice(
            buffer[(OFFSET_MEM_OFFSET - 3)..OFFSET_MEM_OFFSET]
                .split(|x| *x == 0u8)
                .next()
                .unwrap_or_default(),
        );
        let file_name = str::from_utf8(&file_name_buffer).map_err(|_| FilenameError::Utf8Error);
        let offset_bytes = buffer[OFFSET_MEM_OFFSET..].try_into();
        let offset = offset_bytes.map(u32::from_be_bytes).unwrap_or(0);
        file_name
            .and_then(|fnm| ShortFileName::create_from_str(fnm))
            .map(|file| PlayingMemento { playlist, file, offset })
    }
}

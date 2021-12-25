use crate::playlist::{PlaylistName, PLAYLIST_NAME_LEN};
use core::convert::TryInto;
use core::str;
use embedded_sdmmc::{FilenameError, ShortFileName};

pub struct PlayingMemento {
    pub playlist: PlaylistName,
    pub file: ShortFileName,
    pub offset: u32,
}

impl PlayingMemento {
    pub fn new(playlist: PlaylistName, file: ShortFileName, offset: u32) -> Self {
        Self {
            playlist,
            file,
            offset,
        }
    }

    pub fn same_file(&self, other: &PlayingMemento) -> bool {
        self.playlist == other.playlist && self.file == other.file
    }

    pub fn update_offset(&mut self, new_offset: u32) {
        self.offset = new_offset;
    }

    pub fn serialize(
        &self,
        bytes: &mut [u8; PLAYLIST_NAME_LEN + PLAYLIST_NAME_LEN + (u32::BITS as usize) / 8],
    ) {
        bytes[0..PLAYLIST_NAME_LEN].copy_from_slice(&self.playlist.name[0..PLAYLIST_NAME_LEN]);
        bytes[PLAYLIST_NAME_LEN..PLAYLIST_NAME_LEN * 2]
            .copy_from_slice(&self.file.base_name()[0..PLAYLIST_NAME_LEN]);
        bytes[PLAYLIST_NAME_LEN * 2..].copy_from_slice(&self.offset.to_be_bytes()[..]);
    }

    pub fn deserialize(
        &mut self,
        bytes: &[u8; PLAYLIST_NAME_LEN + PLAYLIST_NAME_LEN + (u32::BITS as usize) / 8],
    ) -> Result<(), FilenameError> {
        self.playlist.name[0..PLAYLIST_NAME_LEN].copy_from_slice(&bytes[0..PLAYLIST_NAME_LEN]);
        let name = str::from_utf8(&bytes[PLAYLIST_NAME_LEN..PLAYLIST_NAME_LEN * 2])
            .map_err(|_| FilenameError::Utf8Error)?;
        self.file = ShortFileName::create_from_str(name)?;
        let u32_bytes = bytes[PLAYLIST_NAME_LEN * 2..].try_into();
        self.offset = u32_bytes
            .map(|bytes| u32::from_be_bytes(bytes))
            .unwrap_or(0);
        Ok(())
    }
}

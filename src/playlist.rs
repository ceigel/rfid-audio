use crate::data_reader::{DataReader, DirectoryNavigator, FileError};
use embedded_sdmmc as sdmmc;

#[derive(PartialEq)]
pub enum PlaylistMoveDirection {
    Next,
    Previous,
}

#[derive(Clone, Debug)]
pub struct PlaylistName {
    pub name: [u8; 8],
    pub name_len: usize,
}

impl core::fmt::Display for PlaylistName {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        for c in self.name.iter().take(self.name_len) {
            write!(f, "{}", *c as char)?;
        }
        Ok(())
    }
}

impl PlaylistName {
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let name_len = bytes.len();
        let mut slf = Self {
            name: [0; 8],
            name_len,
        };
        slf.name[0..name_len].clone_from_slice(bytes);
        slf
    }
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.name[..self.name_len])
            .expect("To be able to convert playlist name to str")
    }
}

impl PartialEq<sdmmc::ShortFileName> for PlaylistName {
    fn eq(&self, other: &sdmmc::ShortFileName) -> bool {
        self.name == other.base_name()
    }
}

pub struct Playlist {
    directory: sdmmc::Directory,
    directory_name: sdmmc::ShortFileName,
    current_file: Option<DataReader>,
}

impl Playlist {
    pub fn new(directory: sdmmc::Directory, directory_name: sdmmc::ShortFileName) -> Self {
        Self {
            directory,
            directory_name,
            current_file: None,
        }
    }

    pub fn move_next<'a>(
        &mut self,
        dir: PlaylistMoveDirection,
        directory_navigator: &mut impl DirectoryNavigator,
    ) -> Result<Option<&mut DataReader>, FileError> {
        let comp = |de1: &sdmmc::DirEntry, de2: &sdmmc::DirEntry| {
            let (den1, den2) = (de1.name.base_name(), de2.name.base_name());
            if dir == PlaylistMoveDirection::Next {
                den1 < den2
            } else {
                den1 > den2
            }
        };
        let current_dir_entry = self.current_file.as_ref().map(|f| f.dir_entry.clone());
        self.current_file
            .take()
            .map(|f| f.close_file(directory_navigator))
            .transpose()?;
        let next_dir_entry = directory_navigator.next_file(
            &self.directory,
            current_dir_entry.as_ref(),
            "MP3",
            comp,
        )?;
        let current_file = match next_dir_entry {
            Some(ref de) => Some(DataReader::open_direntry(directory_navigator, de)?),
            None => None,
        };
        self.current_file = current_file;
        Ok(self.current_song())
    }

    pub fn current_song(&mut self) -> Option<&mut DataReader> {
        self.current_file.as_mut()
    }

    pub fn name(&self) -> sdmmc::ShortFileName {
        return self.directory_name.clone();
    }

    pub fn close(self, directory_navigator: &mut impl DirectoryNavigator) {
        self.current_file
            .map(|cf| cf.close_file(directory_navigator));
        directory_navigator.close_dir(self.directory);
    }
}

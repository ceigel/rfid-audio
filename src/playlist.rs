use crate::data_reader::{DataReader, FileError};
use embedded_sdmmc as sdmmc;

pub struct Playlist {
    directory: sdmmc::Directory,
    directory_name: sdmmc::ShortFileName,
    current_file: Option<sdmmc::DirEntry>,
}

impl Playlist {
    pub fn new(directory: sdmmc::Directory, directory_name: sdmmc::ShortFileName) -> Self {
        Self {
            directory,
            directory_name,
            current_file: None,
        }
    }

    pub fn get_current_file(
        &self,
        directory_navigator: &mut impl DirectoryNavigator,
    ) -> Result<Option<DataReader>, FileError> {
        let current_file = self.current_file.as_ref();
        if current_file.is_none() {
            return Ok(None);
        }
        let current_file = current_file.unwrap();
        let file = directory_navigator.open_file_read(&current_file)?;
        Ok(Some(DataReader::new(file, current_file.name.clone())))
    }

    pub fn move_next(
        &mut self,
        directory_navigator: &mut impl DirectoryNavigator,
    ) -> Result<Option<DataReader>, FileError> {
        let comp = |de1: &sdmmc::DirEntry, de2: &sdmmc::DirEntry| {
            if let (Ok(den1), Ok(den2)) = (de1.name.base_name(), de2.name.base_name()) {
                den1 < den2
            } else {
                false
            }
        };
        self.current_file = directory_navigator.next_file(
            &self.directory,
            self.current_file.as_ref(),
            "MP3",
            comp,
        )?;
        self.get_current_file(directory_navigator)
    }

    pub fn name(&self) -> sdmmc::ShortFileName {
        return self.directory_name.clone();
    }

    pub fn close(self, directory_navigator: &mut impl DirectoryNavigator) {
        directory_navigator.close_dir(self.directory);
    }
}

pub trait DirectoryNavigator {
    fn open_file_read(&mut self, file: &sdmmc::DirEntry) -> Result<sdmmc::File, FileError>;
    fn next_file(
        &mut self,
        dir: &sdmmc::Directory,
        current_file: Option<&sdmmc::DirEntry>,
        extension: &str,
        comp: impl Fn(&sdmmc::DirEntry, &sdmmc::DirEntry) -> bool,
    ) -> Result<Option<sdmmc::DirEntry>, FileError>;
    fn close_dir(&mut self, dir: sdmmc::Directory);
}

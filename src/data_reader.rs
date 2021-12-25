use crate::hal::hal as embedded_hal;
use crate::hal::rcc::Clocks;
use crate::hal::time::Hertz;
use crate::playlist::{Playlist, PLAYLIST_NAME_LEN};
use crate::Spi1Type;
use embedded_sdmmc as sdmmc;
use log::error;

const LOGGING_FILE: &str = "cards.txt";
const END_LINE: &str = "\r\n";
const LOG_LINE_LEN: usize = END_LINE.len() + PLAYLIST_NAME_LEN;
const MAX_UNKNOWN: usize = 10;
const BOM: [u8; 3] = [0xEF, 0xBB, 0xBF];
struct DummyTimeSource {}
impl sdmmc::TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> sdmmc::Timestamp {
        sdmmc::Timestamp::from_calendar(2020, 3, 7, 13, 23, 0).expect("To create date")
    }
}

pub(crate) type FileError = sdmmc::Error<sdmmc::SdMmcError>;

pub struct DataReader {
    file: sdmmc::File,
    pub dir_entry: sdmmc::DirEntry,
}

impl DataReader {
    pub fn open_direntry(
        directory_navigator: &mut impl DirectoryNavigator,
        dir_entry: &sdmmc::DirEntry,
    ) -> Result<DataReader, FileError> {
        let file = directory_navigator.open_direntry(dir_entry)?;
        Ok(DataReader {
            file,
            dir_entry: dir_entry.clone(),
        })
    }

    pub fn open_file_in_dir(
        directory_navigator: &mut impl DirectoryNavigator,
        file_name: &sdmmc::ShortFileName,
        directory: &sdmmc::Directory,
    ) -> Result<DataReader, FileError> {
        directory_navigator.open_file_in_dir(file_name, directory)
    }

    pub fn read_data(
        &mut self,
        directory_navigator: &mut impl DirectoryNavigator,
        out: &mut [u8],
    ) -> Result<usize, FileError> {
        directory_navigator.read_data(&mut self.file, out)
    }

    pub fn file_name(&self) -> sdmmc::ShortFileName {
        self.dir_entry.name.clone()
    }

    pub fn done(&self) -> bool {
        self.file.eof()
    }

    pub fn remaining(&self) -> u32 {
        self.file.left()
    }

    pub fn offset(&self) -> u32 {
        self.file.length() - self.file.left()
    }

    pub fn seek_from_start(&mut self, offset: u32) -> Result<(), ()> {
        self.file.seek_from_start(offset)
    }

    pub fn close_file(
        self,
        directory_navigator: &mut impl DirectoryNavigator,
    ) -> Result<(), FileError> {
        directory_navigator.close_file(self.file)
    }
}

pub struct SdCardReader<CS>
where
    CS: embedded_hal::digital::v2::OutputPin,
{
    controller: sdmmc::Controller<sdmmc::SdMmcSpi<Spi1Type, CS>, DummyTimeSource>,
    volume: sdmmc::Volume,
    root_dir: sdmmc::Directory,
}

impl<CS> SdCardReader<CS>
where
    CS: embedded_hal::digital::v2::OutputPin,
{
    pub fn new(
        spi: Spi1Type,
        cs: CS,
        freq: impl Into<Hertz>,
        clocks: Clocks,
    ) -> Result<Self, FileError> {
        let ts = DummyTimeSource {};
        let mut controller = sdmmc::Controller::new(sdmmc::SdMmcSpi::new(spi, cs), ts);
        controller
            .device()
            .init()
            .map_err(sdmmc::Error::DeviceError)?;
        controller.device().spi().reclock(freq, clocks);
        let volume = controller.get_volume(sdmmc::VolumeIdx(0))?;
        let root_dir = controller.open_root_dir(&volume)?;
        Ok(Self {
            controller,
            volume,
            root_dir,
        })
    }

    pub fn open_directory(&mut self, directory_name: &str) -> Result<Playlist, FileError> {
        match self
            .controller
            .find_directory_entry(&self.volume, &self.root_dir, directory_name)
        {
            Ok(_) => self.open_existing_directory(directory_name),
            Err(FileError::FileNotFound) => {
                error!("directory {} not found", directory_name);
                self.directory_not_found(directory_name)?;
                Err(FileError::FileNotFound)
            }
            Err(e) => Err(e),
        }
    }

    fn open_existing_directory(&mut self, directory_name: &str) -> Result<Playlist, FileError> {
        let dir = self
            .controller
            .open_dir(&self.volume, &self.root_dir, directory_name)?;
        let dir_name = sdmmc::ShortFileName::create_from_str(directory_name)
            .map_err(FileError::FilenameError)?;
        Ok(Playlist::new(dir, dir_name))
    }

    fn directory_not_found(&mut self, directory_name: &str) -> Result<(), FileError> {
        let mut buf: [u8; sdmmc::Block::LEN] = [0; sdmmc::Block::LEN];
        let sz_read =
            match self
                .controller
                .find_directory_entry(&self.volume, &self.root_dir, LOGGING_FILE)
            {
                Ok(dir_entry) => {
                    let mut log_file = self.controller.open_dir_entry(
                        &mut self.volume,
                        dir_entry,
                        sdmmc::Mode::ReadOnly,
                    )?;
                    if log_file.length() as usize > buf.len() {
                        log_file
                            .seek_from_end(buf.len() as u32)
                            .expect("To be able to seek inside log file");
                    } else if log_file.length() > 0 {
                        log_file
                            .seek_from_start(BOM.len() as u32)
                            .expect("To be able to seek inside log file");
                    }
                    let sz_read = self
                        .controller
                        .read(&self.volume, &mut log_file, &mut buf)
                        .unwrap_or_default();
                    self.controller.close_file(&self.volume, log_file).ok();
                    sz_read
                }
                Err(FileError::FileNotFound) => 0,
                Err(e) => return Err(e),
            };

        const WRITE_BUF_LEN: usize = MAX_UNKNOWN * LOG_LINE_LEN + BOM.len();
        let mut write_buf: [u8; WRITE_BUF_LEN] = [0; WRITE_BUF_LEN];

        let directory_name_bytes = directory_name.as_bytes();
        let cards = buf[..sz_read]
            .rsplit(|c| *c == b'\r' || *c == b'\n')
            .filter(|s| !s.is_empty())
            .take(MAX_UNKNOWN)
            .filter(|n| *n != directory_name_bytes)
            .chain(core::iter::once(directory_name_bytes));
        write_buf[0..BOM.len()].copy_from_slice(&BOM);
        let mut write_idx = BOM.len();
        for card in cards {
            write_buf[write_idx..(write_idx + PLAYLIST_NAME_LEN)].copy_from_slice(card);
            write_idx += PLAYLIST_NAME_LEN;
            write_buf[write_idx..(write_idx + END_LINE.len())].copy_from_slice(END_LINE.as_bytes());
            write_idx += END_LINE.len();
        }

        let mut log_file = self.controller.open_file_in_dir(
            &mut self.volume,
            &self.root_dir,
            LOGGING_FILE,
            sdmmc::Mode::ReadWriteCreateOrTruncate,
        )?;
        self.controller
            .write(&mut self.volume, &mut log_file, &write_buf[..write_idx])?;
        self.controller.close_file(&self.volume, log_file)?;
        Ok(())
    }
}

pub trait DirectoryNavigator {
    fn read_data(&mut self, file: &mut sdmmc::File, out: &mut [u8]) -> Result<usize, FileError>;
    fn close_file(&mut self, file: sdmmc::File) -> Result<(), FileError>;
    fn open_direntry(&mut self, file: &sdmmc::DirEntry) -> Result<sdmmc::File, FileError>;
    fn next_file(
        &mut self,
        dir: &sdmmc::Directory,
        current_file: Option<&sdmmc::DirEntry>,
        extension: &str,
        comp: impl Fn(&sdmmc::DirEntry, &sdmmc::DirEntry) -> bool,
    ) -> Result<Option<sdmmc::DirEntry>, FileError>;
    fn open_file_in_dir(
        &mut self,
        file_name: &sdmmc::ShortFileName,
        directory: &sdmmc::Directory,
    ) -> Result<DataReader, FileError>;
    fn close_dir(&mut self, dir: sdmmc::Directory);
}

impl<CS> DirectoryNavigator for SdCardReader<CS>
where
    CS: embedded_hal::digital::v2::OutputPin,
{
    fn read_data(&mut self, file: &mut sdmmc::File, out: &mut [u8]) -> Result<usize, FileError> {
        self.controller.read(&self.volume, file, out)
    }

    fn close_file(&mut self, file: sdmmc::File) -> Result<(), FileError> {
        self.controller.close_file(&self.volume, file)
    }

    fn open_direntry(&mut self, file: &sdmmc::DirEntry) -> Result<sdmmc::File, FileError> {
        self.controller
            .open_dir_entry(&mut self.volume, file.clone(), sdmmc::Mode::ReadOnly)
    }
    fn next_file(
        &mut self,
        dir: &sdmmc::Directory,
        current_file: Option<&sdmmc::DirEntry>,
        extension: &str,
        comp: impl Fn(&sdmmc::DirEntry, &sdmmc::DirEntry) -> bool,
    ) -> Result<Option<sdmmc::DirEntry>, FileError> {
        let mut next_dir_entry: Option<sdmmc::DirEntry> = None;
        self.controller
            .iterate_dir(&self.volume, dir, |dir_entry: &sdmmc::DirEntry| {
                if dir_entry.attributes.is_hidden() || dir_entry.attributes.is_directory() {
                    return;
                }
                if dir_entry.name.extension() != extension.as_bytes() {
                    return;
                }
                match (current_file, next_dir_entry.as_ref()) {
                    (Some(current_file), Some(nde)) => {
                        if comp(current_file, dir_entry) && comp(dir_entry, nde) {
                            next_dir_entry.replace(dir_entry.clone());
                        }
                    }
                    (Some(current_file), None) => {
                        if comp(current_file, dir_entry) {
                            next_dir_entry.replace(dir_entry.clone());
                        }
                    }
                    (None, Some(nde)) => {
                        if comp(dir_entry, nde) {
                            next_dir_entry.replace(dir_entry.clone());
                        }
                    }
                    (None, None) => {
                        next_dir_entry.replace(dir_entry.clone());
                    }
                };
            })?;
        Ok(next_dir_entry)
    }

    fn close_dir(&mut self, directory: sdmmc::Directory) {
        self.controller.close_dir(&self.volume, directory);
    }

    fn open_file_in_dir(
        &mut self,
        file_name: &sdmmc::ShortFileName,
        directory: &sdmmc::Directory,
    ) -> Result<DataReader, FileError> {
        let mut open_file: Option<sdmmc::DirEntry> = None;
        self.controller
            .iterate_dir(&self.volume, directory, |dir_entry: &sdmmc::DirEntry| {
                if dir_entry.name == *file_name {
                    open_file.replace(dir_entry.clone());
                }
            })?;
        match open_file {
            Some(de) => self
                .controller
                .open_dir_entry(&mut self.volume, de.clone(), sdmmc::Mode::ReadOnly)
                .map(|file| DataReader {
                    file: file,
                    dir_entry: de,
                }),
            None => Err(FileError::FileNotFound),
        }
    }
}

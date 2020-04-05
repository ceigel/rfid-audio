use crate::hal::hal as embedded_hal;
use embedded_sdmmc as sdmmc;

struct DummyTimeSource {}
impl sdmmc::TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> sdmmc::Timestamp {
        sdmmc::Timestamp::from_calendar(2020, 3, 7, 13, 23, 0).expect("To create date")
    }
}

pub(crate) type FileError = sdmmc::Error<sdmmc::SdMmcError>;
pub struct DataReader {
    file: sdmmc::File,
    file_name: sdmmc::ShortFileName,
}

impl DataReader {
    pub fn new(file: sdmmc::File, file_name: sdmmc::ShortFileName) -> Self {
        DataReader { file, file_name }
    }

    pub fn read_data(
        &mut self,
        file_reader: &mut impl FileReader,
        out: &mut [u8],
    ) -> Result<usize, FileError> {
        file_reader.read_data(&mut self.file, out)
    }

    pub fn file_name(&self) -> sdmmc::ShortFileName {
        self.file_name.clone()
    }

    pub fn done(&self) -> bool {
        self.file.eof()
    }
    pub fn remaining(&self) -> u32 {
        self.file.left()
    }
    pub fn seek_from_start(&mut self, offset: u32) -> Result<(), ()> {
        self.file.seek_from_start(offset)
    }
}

pub trait FileReader {
    fn read_data(&mut self, file: &mut sdmmc::File, out: &mut [u8]) -> Result<usize, FileError>;
}

pub struct SdCardReader<SPI, CS>
where
    SPI: embedded_hal::spi::FullDuplex<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::spi::FullDuplex<u8>>::Error: core::fmt::Debug,
{
    controller: sdmmc::Controller<sdmmc::SdMmcSpi<SPI, CS>, DummyTimeSource>,
    volume: sdmmc::Volume,
    root_dir: sdmmc::Directory,
}

impl<SPI, CS> SdCardReader<SPI, CS>
where
    SPI: embedded_hal::spi::FullDuplex<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::spi::FullDuplex<u8>>::Error: core::fmt::Debug,
{
    pub fn new(spi: SPI, cs: CS) -> Result<Self, FileError> {
        let ts = DummyTimeSource {};
        let mut controller = sdmmc::Controller::new(sdmmc::SdMmcSpi::new(spi, cs), ts);
        controller
            .device()
            .init()
            .map_err(|e| sdmmc::Error::DeviceError(e))?;
        let volume = controller.get_volume(sdmmc::VolumeIdx(0))?;
        let root_dir = controller.open_root_dir(&volume)?;
        Ok(Self {
            controller,
            volume,
            root_dir,
        })
    }

    pub fn open_file(&mut self, file_name: &str) -> Result<DataReader, FileError> {
        let file = self.controller.open_file_in_dir(
            &mut self.volume,
            &self.root_dir,
            file_name,
            sdmmc::Mode::ReadOnly,
        )?;
        let file_name = sdmmc::ShortFileName::create_from_str(file_name)
            .map_err(|e| FileError::FilenameError(e))?;
        Ok(DataReader::new(file, file_name))
    }
}

impl<SPI, CS> FileReader for SdCardReader<SPI, CS>
where
    SPI: embedded_hal::spi::FullDuplex<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::spi::FullDuplex<u8>>::Error: core::fmt::Debug,
{
    fn read_data(&mut self, file: &mut sdmmc::File, out: &mut [u8]) -> Result<usize, FileError> {
        self.controller.read(&self.volume, file, out)
    }
}

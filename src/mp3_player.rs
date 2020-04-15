use crate::data_reader::{DataReader, FileError, FileReader};
use crate::sound_device::SoundDevice;
use crate::MP3_DATA_LENGTH;
use embedded_mp3 as mp3;
use log::{debug, error, info};
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::time;

const MP3_TRIGGER_MOVE: usize = 2 * 1024;
#[derive(Debug)]
pub enum PlayError {
    FileError(FileError),
    NotAnMp3,
    NoValidMp3Frame,
    PlaylistFinished,
}

impl From<FileError> for PlayError {
    fn from(fe: FileError) -> Self {
        PlayError::FileError(fe)
    }
}

pub struct Mp3Player<'a> {
    read_index: usize,
    write_index: usize,
    mp3_data: &'a mut [u8; MP3_DATA_LENGTH],
    decoder: mp3::Decoder<'a>,
    pub(crate) last_frame_rate: Option<time::Hertz>,
    pcm_buffer: &'a mut [i16; mp3::MAX_SAMPLES_PER_FRAME],
    current_song: Option<DataReader>,
}

impl<'a> Mp3Player<'a> {
    pub fn new(
        mp3_data: &'a mut [u8; MP3_DATA_LENGTH],
        decoder_data: &'a mut mp3::DecoderData,
        pcm_buffer: &'a mut [i16; mp3::MAX_SAMPLES_PER_FRAME],
    ) -> Self {
        Self {
            read_index: 0,
            write_index: 0,
            mp3_data,
            decoder: mp3::Decoder::new(decoder_data),
            pcm_buffer: pcm_buffer,
            last_frame_rate: None,
            current_song: None,
        }
    }

    pub fn play_song(
        &mut self,
        data_reader: DataReader,
        file_reader: &mut impl FileReader,
        sound_device: &mut SoundDevice,
    ) -> Result<(), PlayError> {
        info!(
            "Playing song {}, size {}",
            data_reader.file_name(),
            data_reader.remaining()
        );
        self.current_song.replace(data_reader);
        self.skip_id3v2_header(file_reader)?;
        self.init_buffer(file_reader)?;
        sound_device.start_playing(self, file_reader)
    }

    fn skip_id3v2_header(&mut self, file_reader: &mut impl FileReader) -> Result<(), PlayError> {
        const MP3_DETECT_SIZE: usize = 10;
        let data_reader = self
            .current_song
            .as_mut()
            .expect("prepare_read to have returned");
        let buf = &mut self.mp3_data[0..MP3_DETECT_SIZE];
        let bytes_red = data_reader.read_data(file_reader, buf)?;
        if bytes_red != MP3_DETECT_SIZE
            || &buf[0..3] != "ID3".as_bytes()
            || (buf[5] & 0x0f != 0/* only first 4 bits of flags are available */)
            || (buf[6] & 0x80 != 0/* size(synchInteger), first bit 0 */)
            || (buf[7] & 0x80 != 0/* size(synchInteger), first bit 0 */)
            || (buf[8] & 0x80 != 0/* size(synchInteger), first bit 0 */)
            || (buf[9] & 0x80 != 0/* size(synchInteger), first bit 0 */)
        {
            return Err(PlayError::NotAnMp3);
        }
        let mut id3v2size: u32 = (((buf[6] as u32 & 0x7f) << 21)
            | ((buf[7] as u32 & 0x7f) << 14)
            | ((buf[8] as u32 & 0x7f) << 7)
            | (buf[9] as u32 & 0x7f))
            + 10;
        if buf[5] & 0x10 != 0 {
            id3v2size += 10;
        }
        debug!("ID3V2 size is {}", id3v2size);
        data_reader
            .seek_from_start(id3v2size)
            .map_err(|_| PlayError::NotAnMp3)
    }

    pub fn fill_buffer(&mut self, file_reader: &mut impl FileReader) -> Result<(), PlayError> {
        self.fill_buffer_intern(file_reader)
    }

    fn init_buffer(&mut self, file_reader: &mut impl FileReader) -> Result<(), PlayError> {
        self.fill_buffer_intern(file_reader)
    }

    pub fn fill_buffer_intern(
        &mut self,
        file_reader: &mut impl FileReader,
    ) -> Result<(), PlayError> {
        if self.current_song.as_ref().map_or(true, |song| song.done()) {
            return Ok(());
        }
        if self.read_index < MP3_TRIGGER_MOVE && self.write_index == self.mp3_data.len() {
            return Ok(()); //not enough space to read
        }
        if self.write_index == self.mp3_data.len() {
            let copy_len = self.mp3_data.len() - self.read_index;
            let from = self.mp3_data[self.read_index..].as_ptr();
            let into = self.mp3_data[..].as_mut_ptr();
            unsafe {
                core::ptr::copy(from, into, copy_len);
            }
            self.read_index = 0;
            self.write_index = copy_len;
        } else {
            let data_reader = self
                .current_song
                .as_mut()
                .expect("prepare_read to have returned");
            debug!(
                "Will read from card: {}..{}",
                self.write_index,
                self.mp3_data.len()
            );
            let out_slice = &mut self.mp3_data[self.write_index..];
            let bytes_red = data_reader.read_data(file_reader, out_slice)?;
            debug!(
                "Read {}, Remaining {} bytes",
                bytes_red,
                data_reader.remaining()
            );
            self.write_index += bytes_red;
        }
        Ok(())
    }

    pub fn next_frame(&mut self, dma_buffer: &mut [u16]) -> usize {
        debug!(
            "Next frame: read_index: {}, write_index: {}",
            self.read_index, self.write_index
        );
        let mp3: &[u8] = &self.mp3_data[self.read_index..self.write_index];
        if mp3.len() == 0 {
            return 0;
        }
        let pcm_buffer = &mut self.pcm_buffer;
        let decode_result = self.decoder.decode(mp3, pcm_buffer);
        loop {
            match decode_result {
                mp3::DecodeResult::Successful(bytes_read, frame) => {
                    debug!(
                        "Decoding successful: {} read_index: {}",
                        bytes_read, self.read_index
                    );
                    self.last_frame_rate.replace(frame.sample_rate.hz());
                    self.read_index += bytes_read;
                    let samples = pcm_buffer
                        .iter()
                        .step_by(frame.channels as usize)
                        .take(frame.sample_count as usize)
                        .map(|sample| ((*sample as i32) - (core::i16::MIN as i32)) as u16 >> 4);
                    let mut index: usize = 0;
                    for val in samples {
                        dma_buffer[index] = val;
                        index += 1;
                    }
                    return index;
                }
                mp3::DecodeResult::SkippedData(skipped_data) => {
                    debug!("Skipping: {} read_index: {}", skipped_data, self.read_index);
                    match self.read_index.checked_add(skipped_data) {
                        Some(new_val) => {
                            if new_val >= self.write_index {
                                error!("Skipped data outside buffer. read_index: {}, write_index: {}, skipped_data: {}", self.read_index, self.write_index, skipped_data);
                                return 0;
                            }
                            self.read_index = new_val;
                        }
                        None => {
                            error!("Skipped data overflowed read_index add. read_index: {}, skipped_data: {}", self.read_index, skipped_data);
                            return 0;
                        }
                    }
                }
                mp3::DecodeResult::InsufficientData => {
                    return 0;
                }
            }
        }
    }
}

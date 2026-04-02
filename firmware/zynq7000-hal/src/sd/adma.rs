use crate::dma::CacheAligned;

pub const ADMA2_DESCRIPTOR_COUNT: usize = 32;
const DESC_MAX_LENGTH: usize = 65_536;
const ATTR_TRANSFER: u16 = 1 << 5;
const ATTR_END: u16 = 1 << 1;
const ATTR_VALID: u16 = 1 << 0;

#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, Default)]
pub struct Adma2Descriptor32 {
    attribute: u16,
    length: u16,
    address: u32,
}

impl Adma2Descriptor32 {
    const fn new() -> Self {
        Self {
            attribute: 0,
            length: 0,
            address: 0,
        }
    }

    fn configure(&mut self, address: u32, len: usize, end: bool) {
        self.address = address;
        self.length = if len == DESC_MAX_LENGTH {
            0
        } else {
            len as u16
        };
        self.attribute = ATTR_TRANSFER | ATTR_VALID | if end { ATTR_END } else { 0 };
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Adma2DescriptorTable {
    entries: [Adma2Descriptor32; ADMA2_DESCRIPTOR_COUNT],
}

impl Adma2DescriptorTable {
    pub const MAX_TRANSFER_BYTES: usize = ADMA2_DESCRIPTOR_COUNT * DESC_MAX_LENGTH;

    pub const fn new() -> Self {
        Self {
            entries: [const { Adma2Descriptor32::new() }; ADMA2_DESCRIPTOR_COUNT],
        }
    }

    pub fn configure(&mut self, buffer_addr: u32, len: usize) {
        self.entries.fill(Adma2Descriptor32::new());
        let descriptor_count = len.div_ceil(DESC_MAX_LENGTH);
        for desc_idx in 0..descriptor_count {
            let offset = desc_idx * DESC_MAX_LENGTH;
            let desc_len = core::cmp::min(DESC_MAX_LENGTH, len - offset);
            self.entries[desc_idx].configure(
                buffer_addr + offset as u32,
                desc_len,
                desc_idx + 1 == descriptor_count,
            );
        }
    }

    pub fn as_ptr(&self) -> *const Adma2Descriptor32 {
        self.entries.as_ptr()
    }
}

pub type CacheAlignedAdma2DescriptorTable = CacheAligned<Adma2DescriptorTable>;

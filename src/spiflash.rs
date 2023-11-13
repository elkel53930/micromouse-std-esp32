use esp_idf_sys::{esp_err_to_name, esp_vfs_fat_mount_config_t, esp_vfs_fat_spiflash_mount};

const MAX_FILES: i32 = 16;
const ALLOCATION_UNIT_SIZE: usize = 512;

use lazy_static::lazy_static;
use std::ffi::CString;

pub const BASE_PATH_RAW: &'static str = "/sf";
pub const PARTITION_RAW: &'static str = "storage0";
lazy_static! {
    static ref PARTITION: CString =
        CString::new(PARTITION_RAW).expect("Failed to convert to CString");
    static ref BASE_PATH: CString =
        CString::new(BASE_PATH_RAW).expect("Failed to convert to CString");
}

pub fn mount() {
    // Mount FAT filesystem
    // file system trial
    unsafe {
        let mount_config = esp_vfs_fat_mount_config_t {
            max_files: MAX_FILES,
            format_if_mount_failed: true,
            allocation_unit_size: ALLOCATION_UNIT_SIZE,
        };
        let mut handle: esp_idf_sys::wl_handle_t = esp_idf_sys::WL_INVALID_HANDLE;
        let mount_result = esp_vfs_fat_spiflash_mount(
            BASE_PATH.as_ptr(),
            PARTITION.as_ptr(),
            &mount_config,
            &mut handle,
        );
        println!(
            "mount result: {:?} = {:?}",
            mount_result,
            std::ffi::CStr::from_ptr(esp_err_to_name(mount_result))
                .to_str()
                .unwrap()
        );
    }
}

/*
usage

fn test(){
    uprintln!("write file ");
    match std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .append(true)
        .open(fs::path("test.txt"))
    {
        Ok(mut file) => {
            match file.write_all(b"Hello, world!\n") {
                Ok(_) => uprintln!("write file done"),
                Err(e) => uprintln!("write_all failed: {:?}", e),
            }
        }
        Err(e) => uprintln!("OpenOptions::new failed: {:?}", e),
    }

    // Read & print file
    uprintln!("read file");
    match std::fs::File::open(fs::path("test.txt")) {
        Ok(mut file) => {
            let mut buf = String::new();
            file.read_to_string(&mut buf)?;
            uprintln!("file content: {}", buf);
        }
        Err(e) => uprintln!("read file failed: {:?}", e),
    }
}
*/

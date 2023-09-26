pub const CONTROL_CYCLE: u32 = 1; // ms
pub const WHEEL_DIAMETER: f32 = 13.0; // mm
pub const GEAR_RATIO: f32 = 1.5;

use yaml_rust::YamlLoader;

struct Config {
    f32_example: f32,
}

static mut CONFIG: Config = Config { f32_example: 0.123 };

fn load_yaml(path: &str) -> anyhow::Result<Vec<yaml_rust::Yaml>> {
    let y_str = std::fs::read_to_string(path)?.to_string();
    let docs = YamlLoader::load_from_str(&y_str)?;
    Ok(docs)
}

fn load<T>(config: &mut T, item: Option<T>)
where
    T: std::fmt::Display,
{
    match item {
        Some(x) => {
            *config = x;
            uprintln!("{}", *config);
        }
        None => uprintln!("Cannot find the entry."),
    }
}

macro_rules! load_config {
    ($doc:expr, $field:ident, f64) => {
        uprint!(concat!(stringify!($field), ": "));
        unsafe {
            load(
                &mut CONFIG.$field,
                $doc[stringify!($field)].as_f64().map(|x| x as f32),
            );
        }
    };
    ($doc:expr, $field:ident, i64) => {
        uprint!(concat!(stringify!($field), ": "));
        unsafe {
            load(
                &mut CONFIG.$field,
                $doc[stringify!($field)].as_i64().map(|x| x as i32),
            );
        }
    };
    ($doc:expr, $field:ident, u64) => {
        uprint!(concat!(stringify!($field), ": "));
        unsafe {
            load(
                &mut CONFIG.$field,
                $doc[stringify!($field)].as_u64().map(|x| x as u32),
            );
        }
    };
}

pub fn init() {
    let path = "/sf/config.yaml";
    uprintln!("Loading config from {}", path);
    let vec_yaml = match load_yaml(&path) {
        Ok(x) => x,
        Err(e) => {
            uprintln!("Config read error!!: {}", e);
            return;
        }
    };
    let yaml = &vec_yaml[0];

    load_config!(yaml, f32_example, f64);
}

pub fn f32_example() -> f32 {
    unsafe { CONFIG.f32_example }
}

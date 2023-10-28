pub const CONTROL_CYCLE: u32 = 1; // ms
pub const WHEEL_DIAMETER: f32 = 13.0; // mm
pub const GEAR_RATIO: f32 = 1.5;

use yaml_rust::{Yaml, YamlLoader};

const YAML_FILENAME: &str = "/sf/config.yaml";
pub struct YamlConfig {
    yaml: Yaml,
}

impl YamlConfig {
    // new: Load yaml file
    pub fn new() -> anyhow::Result<YamlConfig> {
        let y_str = std::fs::read_to_string(YAML_FILENAME)?.to_string();
        let docs = YamlLoader::load_from_str(&y_str)?;
        Ok(YamlConfig {
            yaml: docs[0].clone(),
        })
    }

    pub fn load(&self, field: &str) -> anyhow::Result<Yaml> {
        Ok(self.yaml[field].clone())
    }

    #[allow(dead_code)]
    pub fn show(&self) {
        println!("Configuration:");
        for item in self.yaml.as_hash().unwrap() {
            println!("  {}: {:?}", item.0.as_str().unwrap(), item.1);
        }
    }

    #[allow(dead_code)]
    pub fn ushow(&self) {
        uprintln!("Configuration:");
        for item in self.yaml.as_hash().unwrap() {
            uprintln!("  {}: {:?}", item.0.as_str().unwrap(), item.1);
        }
    }
}

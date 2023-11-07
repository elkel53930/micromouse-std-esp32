pub const CONTROL_CYCLE: u32 = 1; // ms

use yaml_rust::{Yaml, YamlLoader};

pub struct YamlConfig {
    filename: String,
    yaml: Option<Yaml>,
}

impl YamlConfig {
    pub fn new(filename: String) -> anyhow::Result<YamlConfig> {
        fn read_yaml(filename: &String) -> anyhow::Result<Vec<Yaml>> {
            let y_str = std::fs::read_to_string(&filename)?.to_string();
            let docs = YamlLoader::load_from_str(&y_str)?;
            Ok(docs)
        }

        let docs = match read_yaml(&filename) {
            Ok(docs) => docs,
            Err(e) => {
                println!("Error reading {}: {}", filename, e);
                return Ok(YamlConfig {
                    filename: filename,
                    yaml: None,
                });
            }
        };

        Ok(YamlConfig {
            filename: filename,
            yaml: Some(docs[0].clone()),
        })
    }

    #[allow(dead_code)]
    pub fn load(&self, field: &str) -> anyhow::Result<Yaml> {
        if self.yaml.is_none() {
            return Err(anyhow::anyhow!("{} is not opened.", self.filename));
        }
        Ok(self.yaml.as_ref().unwrap()[field].clone())
    }

    #[allow(dead_code)]
    pub fn load_i64(&self, field: &str, default: i64) -> i64 {
        if self.yaml.is_none() {
            println!(
                "{} is not opened, using default value {} for {}.",
                self.filename, default, field
            );
            return default;
        }

        let result = self.yaml.as_ref().unwrap()[field].as_i64();
        match result {
            Some(value) => value,
            None => {
                println!(
                    "Warning at {} in {}, using default value {}",
                    field, self.filename, default
                );
                default
            }
        }
    }

    #[allow(dead_code)]
    pub fn load_f64(&self, field: &str, default: f64) -> f64 {
        if self.yaml.is_none() {
            println!(
                "{} is not opened, using default value {} for {}.",
                self.filename, default, field
            );
            return default;
        }
        let result = self.yaml.as_ref().unwrap()[field].as_f64();
        match result {
            Some(value) => value,
            None => {
                println!(
                    "Warning at {} in {}, using default value {}",
                    field, self.filename, default
                );
                default
            }
        }
    }

    #[allow(dead_code)]
    pub fn load_bool(&self, field: &str, default: bool) -> bool {
        if self.yaml.is_none() {
            println!(
                "{} is not opened, using default value {} for {}.",
                self.filename, default, field
            );
            return default;
        }
        let result = self.yaml.as_ref().unwrap()[field].as_bool();
        match result {
            Some(value) => value,
            None => {
                println!(
                    "Warning at {} in {}, using default value {}",
                    field, self.filename, default
                );
                default
            }
        }
    }

    #[allow(dead_code)]
    pub fn load_vec_i16_f32(&self, field: &str, default: Vec<(i16, f32)>) -> Vec<(i16, f32)> {
        if self.yaml.is_none() {
            println!(
                "{} is not opened, using default value {:?} for {}.",
                self.filename, default, field
            );
            return default;
        }
        let result = self.yaml.as_ref().unwrap()[field].as_vec();
        match result {
            Some(value) => {
                let mut vec = Vec::new();
                for item in value {
                    let pair = match item.as_vec() {
                        Some(pair) => pair,
                        None => {
                            println!(
                                "Warning at {} in {}, using default value {:?}",
                                field, self.filename, default
                            );
                            return default;
                        }
                    };
                    let first = match pair[0].as_i64() {
                        Some(first) => first,
                        None => {
                            println!(
                                "Warning at {} in {}, using default value {:?}",
                                field, self.filename, default
                            );
                            return default;
                        }
                    };
                    let second = match pair[1].as_f64() {
                        Some(second) => second,
                        None => {
                            println!(
                                "Warning at {} in {}, using default value {:?}",
                                field, self.filename, default
                            );
                            return default;
                        }
                    };
                    vec.push((first as i16, second as f32));
                }
                vec
            }
            None => {
                println!(
                    "Warning at {} in {}, using default value {:?}",
                    field, self.filename, default
                );
                return default;
            }
        }
    }

    #[allow(dead_code)]
    pub fn load_vec_f32(&self, field: &str, default: Vec<f32>) -> Vec<f32> {
        if self.yaml.is_none() {
            println!(
                "{} is not opened, using default value {:?} for {}.",
                self.filename, default, field
            );
            return default;
        }
        let result = self.yaml.as_ref().unwrap()[field].as_vec();
        match result {
            Some(value) => {
                let mut vec = Vec::new();
                for item in value {
                    let item = match item.as_f64() {
                        Some(item) => item,
                        None => {
                            println!(
                                "Warning at {} in {}, using default value {:?}",
                                field, self.filename, default
                            );
                            return default;
                        }
                    };
                    vec.push(item as f32);
                }
                vec
            }
            None => {
                println!(
                    "Warning at {} in {}, using default value {:?}",
                    field, self.filename, default
                );
                return default;
            }
        }
    }

    #[allow(dead_code)]
    pub fn show(&self) {
        println!("Configuration:");
        if self.yaml.is_none() {
            println!("  {} is not opened.", self.filename);
            return;
        }
        for item in self.yaml.as_ref().unwrap().as_hash().unwrap() {
            println!("  {}: {:?}", item.0.as_str().unwrap(), item.1);
        }
    }

    #[allow(dead_code)]
    pub fn ushow(&self) {
        uprintln!("Configuration:");
        if self.yaml.is_none() {
            uprintln!("  {} is not opened.", self.filename);
            return;
        }
        for item in self.yaml.as_ref().unwrap().as_hash().unwrap() {
            uprintln!("  {}: {:?}", item.0.as_str().unwrap(), item.1);
        }
    }
}

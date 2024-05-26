use emcode::*;

pub struct CmdEm {}

imple COnsoleCommand for CmdEm {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() == 0{
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        let filename = args[0];


        let parser = parser::p_statement();
        let mut s = parser::ParserState::new(test_script);
        let result = parser(&mut s);
        match result {
            Ok(statement) => {
                let mut env = interpreter::Environment::new();
                let _ = interpreter::evaluate_statement(&mut env, &statement);
            }
            Err(e) => {
                println!("Error at {} : {}", s.pos_string(), e);
            }
        }
    }
    
}


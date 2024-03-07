use std::collections::HashMap;

type VariableName = String;
type Int = i32;
type Float = f32;
type Str = String;

#[derive(Debug)]
pub enum BinaryOperator {
    Add,
    Sub,
    Mul,
    Div,
    Mod,
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Neq,
    And,
    Or,
}

#[derive(Debug)]
pub enum Expression {
    IntLiteral(Int),
    FloatLiteral(Float),
    StringLiteral(Str),
    Variable(VariableName),
    FunctionCall(String, Vec<Expression>),
    BinaryOperation(BinaryOperator, Box<Expression>, Box<Expression>),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Int(Int),
    Float(Float),
    Str(Str),
}

pub type Environment = HashMap<VariableName, Value>;
type Function = dyn Fn(&Vec<Value>) -> Result<Value, String>;

fn fn_print(args: &Vec<Value>) -> Result<Value, String> {
    for arg in args {
        match arg {
            Value::Int(i) => print!("{} ", i),
            Value::Float(f) => print!("{} ", f),
            Value::Str(s) => print!("{} ", s),
        }
    }
    println!("");
    Ok(Value::Int(1))
}

fn fn_input_str(args: &Vec<Value>) -> Result<Value, String> {
    if args.len() > 0 {
        return Err("input() takes no arguments".to_string());
    }
    println!("input_str is not implemented yet.");
    Ok(Value::Str("".to_string()))
}

fn fn_input_int(args: &Vec<Value>) -> Result<Value, String> {
    if args.len() > 0 {
        return Err("input() takes no arguments".to_string());
    }
    println!("input_int is not implemented yet.");
    Ok(Value::Int(0))
}

fn fn_input_float(args: &Vec<Value>) -> Result<Value, String> {
    if args.len() > 0 {
        return Err("input() takes no arguments".to_string());
    }
    println!("input_float is not implemented yet.");
    Ok(Value::Float(0.0))
}

fn get_function(name: &str) -> Option<Box<Function>> {
    match name {
        "print" => Some(Box::new(fn_print)),
        "input" => Some(Box::new(fn_input_str)),
        "input_int" => Some(Box::new(fn_input_int)),
        "input_float" => Some(Box::new(fn_input_float)),
        _ => None,
    }
}

pub fn evaluate_expression(
    environment: &Environment,
    expression: &Expression,
) -> Result<Value, String> {
    match expression {
        Expression::IntLiteral(i) => Ok(Value::Int(*i)),
        Expression::FloatLiteral(f) => Ok(Value::Float(*f)),
        Expression::StringLiteral(s) => Ok(Value::Str((*s.clone()).to_string())),
        Expression::Variable(v) => {
            // if the variable is not found, return an error
            environment
                .get(v)
                .ok_or(format!("Variable not found: {}", v))?;
            Ok(environment[v].clone())
        }
        Expression::FunctionCall(name, args) => {
            let mut evaluated_args = Vec::new();
            for arg in args {
                evaluated_args.push(evaluate_expression(environment, arg)?);
            }
            let function = get_function(&name).ok_or(format!("Function not found: {}", name))?;
            let mut values = Vec::new();
            for arg in evaluated_args {
                match arg {
                    Value::Int(i) => values.push(Value::Int(i)),
                    Value::Float(f) => values.push(Value::Float(f)),
                    Value::Str(s) => values.push(Value::Str(s)),
                }
            }
            Ok(function(&values)?)
        }
        Expression::BinaryOperation(op, lhs, rhs) => {
            let lhs = evaluate_expression(environment, lhs)?;
            let rhs = evaluate_expression(environment, rhs)?;
            match (lhs, rhs) {
                (Value::Int(lhs), Value::Int(rhs)) => match op {
                    BinaryOperator::Add => Ok(Value::Int(lhs + rhs)),
                    BinaryOperator::Sub => Ok(Value::Int(lhs - rhs)),
                    BinaryOperator::Mul => Ok(Value::Int(lhs * rhs)),
                    BinaryOperator::Div => Ok(Value::Int(lhs / rhs)),
                    BinaryOperator::Mod => Ok(Value::Int(lhs % rhs)),
                    BinaryOperator::Lt => Ok(Value::Int((lhs < rhs) as i32)),
                    BinaryOperator::Le => Ok(Value::Int((lhs <= rhs) as i32)),
                    BinaryOperator::Gt => Ok(Value::Int((lhs > rhs) as i32)),
                    BinaryOperator::Ge => Ok(Value::Int((lhs >= rhs) as i32)),
                    BinaryOperator::Eq => Ok(Value::Int((lhs == rhs) as i32)),
                    BinaryOperator::Neq => Ok(Value::Int((lhs != rhs) as i32)),
                    BinaryOperator::And => Ok(Value::Int((lhs != 0 && rhs != 0) as i32)),
                    BinaryOperator::Or => Ok(Value::Int((lhs != 0 || rhs != 0) as i32)),
                },
                (Value::Float(lhs), Value::Float(rhs)) => match op {
                    BinaryOperator::Add => Ok(Value::Float(lhs + rhs)),
                    BinaryOperator::Sub => Ok(Value::Float(lhs - rhs)),
                    BinaryOperator::Mul => Ok(Value::Float(lhs * rhs)),
                    BinaryOperator::Div => Ok(Value::Float(lhs / rhs)),
                    BinaryOperator::Lt => Ok(Value::Int((lhs < rhs) as i32)),
                    BinaryOperator::Le => Ok(Value::Int((lhs <= rhs) as i32)),
                    BinaryOperator::Gt => Ok(Value::Int((lhs > rhs) as i32)),
                    BinaryOperator::Ge => Ok(Value::Int((lhs >= rhs) as i32)),
                    BinaryOperator::Eq => Ok(Value::Int((lhs == rhs) as i32)),
                    BinaryOperator::Neq => Ok(Value::Int((lhs != rhs) as i32)),
                    BinaryOperator::And => Ok(Value::Int((lhs != 0.0 && rhs != 0.0) as i32)),
                    BinaryOperator::Or => Ok(Value::Int((lhs != 0.0 || rhs != 0.0) as i32)),
                    _ => Err(format!("Invalid operation for float: {:?}", op)),
                },
                (Value::Str(lhs), Value::Str(rhs)) => match op {
                    BinaryOperator::Add => {
                        let mut result = lhs.clone();
                        result.push_str(&rhs);
                        Ok(Value::Str(result))
                    }
                    _ => Err(format!("Invalid operation for string: {:?}", op)),
                },
                _ => Err(format!("Invalid operation: {:?}", op)),
            }
        }
    }
}

// statement

#[derive(Debug)]
pub enum Statement {
    Expression(Expression),
    VariableDeclaration(VariableName, Expression),
    Sequence(Vec<Statement>),
    Assign(VariableName, Expression),
    If(Expression, Box<Statement>, Box<Statement>),
    While(Expression, Box<Statement>),
    Pass,
}

pub fn evaluate_statement(
    environment: &mut Environment,
    statement: &Statement,
) -> Result<(), String> {
    match statement {
        Statement::Expression(expression) => {
            evaluate_expression(environment, expression)?;
            Ok(())
        }
        Statement::VariableDeclaration(name, expression) => {
            let value = evaluate_expression(environment, expression)?;
            environment.insert(name.clone(), value);
            Ok(())
        }
        Statement::Sequence(statements) => {
            for statement in statements {
                evaluate_statement(environment, statement)?;
            }
            Ok(())
        }
        Statement::Assign(name, expression) => {
            environment
                .get(name)
                .ok_or(format!("Variable not found: {}", name))?;
            let value = evaluate_expression(environment, expression)?;
            environment.insert(name.clone(), value);
            Ok(())
        }
        Statement::If(condition, then_branch, else_branch) => {
            let condition = evaluate_expression(environment, condition)?;
            match condition {
                Value::Int(0) => evaluate_statement(environment, else_branch),
                _ => evaluate_statement(environment, then_branch),
            }
        }
        Statement::While(condition, body) => {
            let mut result = Ok(());
            while evaluate_expression(environment, condition)?.clone() != Value::Int(0) {
                result = evaluate_statement(environment, body);
            }
            result
        }
        Statement::Pass => Ok(()),
    }
}

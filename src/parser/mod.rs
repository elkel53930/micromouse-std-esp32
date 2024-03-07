pub struct ParserState<'a> {
    input: &'a str,
    offset: usize,
}

impl ParserState<'_> {
    pub fn new(input: &str) -> ParserState {
        ParserState { input, offset: 0 }
    }

    pub fn get(&self) -> &str {
        &self.input[self.offset..]
    }

    pub fn consume<'a>(&mut self, n: usize) {
        self.offset += n;
        if self.offset > self.input.len() {
            self.offset = self.input.len();
        }
    }

    pub fn consume_while<'a>(&mut self, f: impl Fn(char) -> bool) {
        let mut offset = self.offset;
        while let Some(c) = self.get().chars().next() {
            if f(c) {
                offset += c.len_utf8();
            } else {
                break;
            };
        }
        self.offset = offset;
    }

    pub fn pos_info(&self) -> (usize, usize) {
        let mut line = 1;
        let mut col = 1;
        let input = &self.input[..self.offset];
        for c in input.chars() {
            if c == '\n' {
                line += 1;
                col = 1;
            } else {
                col += 1;
            }
        }
        (line, col)
    }

    pub fn pos_string(&self) -> String {
        let (line, col) = self.pos_info();
        format!("{}:{}", line, col)
    }
}

pub trait Parser<T>: Fn(&mut ParserState) -> Result<T, String> {}
impl<T, F> Parser<T> for F where F: Fn(&mut ParserState) -> Result<T, String> {}
use crate::interpreter::{BinaryOperator, Expression, Statement};

// Basic parsers

pub fn digits<'a>() -> impl Parser<String> + 'a {
    move |s| {
        let end = s
            .get()
            .find(|c: char| !c.is_ascii_digit())
            .unwrap_or(s.get().len());
        if end == 0 {
            return Err("Expected digits".to_string());
        }
        let result = s.get()[..end].to_string();
        s.consume(end);
        Ok(result)
    }
}

pub fn character(c: char) -> impl Parser<char> {
    move |s| {
        let mut chars = s.get().chars();
        if chars.next() == Some(c) {
            {
                s.consume(c.len_utf8());
                Ok(c)
            }
        } else {
            Err(format!("Expected '{}'", c))
        }
    }
}

pub fn string<'a>(target: &'a str) -> impl Parser<String> + 'a {
    move |s| {
        if s.get().starts_with(target) {
            {
                s.consume(target.len());
                Ok(target.to_string())
            }
        } else {
            Err(format!("Expected '{}'", target))
        }
    }
}

pub fn map<T, U>(parser: Box<impl Parser<T>>, f: impl Fn(T) -> U) -> impl Parser<U> {
    move |s| parser(s).map(|result| f(result))
}

pub fn filter(f: impl Fn(char) -> bool) -> impl Parser<char> {
    move |s| {
        let mut chars = s.get().chars();
        if let Some(c) = chars.next() {
            if f(c) {
                {
                    s.consume(c.len_utf8());
                    Ok(c)
                }
            } else {
                Err(format!("Unexpected character: '{}'", c))
            }
        } else {
            Err("Unexpected EOF".to_string())
        }
    }
}

pub fn space<T>(parser: Box<impl Parser<T>>) -> impl Parser<T> {
    move |s| {
        many0(filter(|c| c.is_ascii_whitespace()).into())(s)?;
        parser(s)
    }
}

pub fn s_character(c: char) -> impl Parser<char> {
    space(Box::new(character(c)))
}

pub fn s_string<'a>(target: &'a str) -> impl Parser<String> + 'a {
    space(Box::new(string(target)))
}

pub fn s_digits() -> impl Parser<String> {
    space(Box::new(digits()))
}

fn alta<T>(p1: Box<impl Parser<T>>, p2: Box<impl Parser<T>>) -> impl Parser<T> {
    move |s| {
        if let Ok(result) = p1(s) {
            Ok(result)
        } else {
            p2(s)
        }
    }
}

#[macro_export]
macro_rules! alta {
    ($parser0:expr, $($parser:expr),*) => {{
        let p = $parser0;
        $(
            let p = crate::parser::alta(Box::new(p), Box::new($parser));
        )*
        p
    }};
}

pub fn many0<T>(parser: Box<impl Parser<T>>) -> impl Parser<Vec<T>> {
    move |s| {
        let mut result = Vec::new();
        while let Ok(r) = parser(s) {
            result.push(r);
        }
        Ok(result)
    }
}

pub fn many1<T>(parser: Box<impl Parser<T>>) -> impl Parser<Vec<T>> {
    move |s| {
        let r = parser(s)?;
        let mut result = Vec::new();
        while let Ok(r) = parser(s) {
            result.push(r);
        }
        result.insert(0, r);
        Ok(result)
    }
}

pub fn sep_by<T, U>(
    parser: Box<impl Parser<T>>,
    separator: Box<impl Parser<U>>,
) -> impl Parser<Vec<T>> {
    move |s| {
        let r = parser(s)?;
        let mut result = vec![r];
        while let Ok(_) = separator(s) {
            let r = parser(s)?;
            result.push(r);
        }
        Ok(result)
    }
}

pub fn sep_by0<T, U>(
    parser: Box<impl Parser<T>>,
    separator: Box<impl Parser<U>>,
) -> impl Parser<Vec<T>> {
    move |s| {
        let r = parser(s);
        if r.is_err() {
            return Ok(Vec::new());
        }
        let r = r.unwrap();
        let mut result = vec![r];
        while let Ok(_) = separator(s) {
            let r = parser(s)?;
            result.push(r);
        }
        Ok(result)
    }
}

// Combined parsers

pub fn p_ident() -> impl Parser<String> {
    move |s| {
        // 先頭はアルファベット
        // ２文字目以降は数字とアンダースコアも許容
        let head = space(Box::new(filter(|c| c.is_ascii_alphabetic())))(s)?;
        let tail = many0(Box::new(filter(|c| c.is_ascii_alphanumeric() || c == '_')))(s)?;
        let name = format!("{}{}", head, tail.into_iter().collect::<String>());
        Ok(name)
    }
}

pub fn p_expression() -> impl Parser<Expression> {
    move |s| {
        alta![
            p_function_call(),
            p_variable(),
            p_int_float_literal(),
            p_string_literal(),
            p_binary_operation()
        ](s)
    }
}

pub fn p_int_float_literal() -> impl Parser<Expression> {
    move |s| {
        let n = s_digits()(s)?;
        if s.get().chars().next() == Some('.') {
            let _ = character('.')(s)?;
            let m = digits()(s)?;
            let f = format!("{}.{}", n, m);
            let f = f.parse().unwrap();
            Ok(Expression::FloatLiteral(f))
        } else {
            Ok(Expression::IntLiteral(n.parse().unwrap()))
        }
    }
}

pub fn p_string_literal() -> impl Parser<Expression> {
    move |s| {
        let _ = s_string("\"")(s)?;
        let str = many0(filter(|c| c != '"').into())(s)?;
        let _ = string("\"")(s)?;
        // vector to s_string
        let str: String = str.into_iter().collect();
        Ok(Expression::StringLiteral(str))
    }
}

pub fn p_binary_operation() -> impl Parser<Expression> {
    move |s| {
        let _ = s_character('(')(s)?;
        let lhs = p_expression()(s)?;
        let op = alta![
            s_string("+"),
            s_string("-"),
            s_string("*"),
            s_string("/"),
            s_string("%"),
            s_string("=="),
            s_string("!="),
            s_string("<="),
            s_string(">="),
            s_string("<"),
            s_string(">"),
            s_string("&&"),
            s_string("||")
        ](s)?;
        let rhs = p_expression()(s)?;
        let _ = s_character(')')(s)?;
        let op = match op.as_str() {
            "+" => BinaryOperator::Add,
            "-" => BinaryOperator::Sub,
            "*" => BinaryOperator::Mul,
            "/" => BinaryOperator::Div,
            "%" => BinaryOperator::Mod,
            "==" => BinaryOperator::Eq,
            "!=" => BinaryOperator::Neq,
            "<=" => BinaryOperator::Le,
            ">=" => BinaryOperator::Ge,
            "<" => BinaryOperator::Lt,
            ">" => BinaryOperator::Gt,
            "&&" => BinaryOperator::And,
            "||" => BinaryOperator::Or,
            _ => unreachable!(),
        };
        Ok(Expression::BinaryOperation(
            op,
            Box::new(lhs),
            Box::new(rhs),
        ))
    }
}

pub fn p_function_call() -> impl Parser<Expression> {
    move |s| {
        let name = p_ident()(s)?;
        let _ = character('!')(s)?;
        let _ = s_character('(')(s)?;
        let args = sep_by0(p_expression().into(), Box::new(s_character(',')))(s)?;
        let _ = s_character(')')(s)?;
        Ok(Expression::FunctionCall(name, args))
    }
}

pub fn p_variable() -> impl Parser<Expression> {
    move |s| {
        let _ = s_character('$')(s)?;
        let name = p_ident()(s)?;
        Ok(Expression::Variable(name))
    }
}

pub fn p_statement() -> impl Parser<Statement> {
    move |s| alta![p_sequence(), p_without_sequence()](s)
}

pub fn p_without_sequence() -> impl Parser<Statement> {
    move |s| {
        alta![
            p_assign(),
            p_variable_declaration(),
            p_if(),
            p_while(),
            p_pass(),
            p_expression_statement()
        ](s)
    }
}

pub fn p_variable_declaration() -> impl Parser<Statement> {
    move |s| {
        let _ = s_string("let ")(s)?;
        let _ = space(Box::new(s_character('$')))(s)?;
        let name = p_ident()(s)?;
        let _ = s_string("=")(s)?;
        let expression = p_expression()(s)?;
        Ok(Statement::VariableDeclaration(name, expression))
    }
}

pub fn p_expression_statement() -> impl Parser<Statement> {
    move |s| {
        let expression = p_expression()(s)?;
        Ok(Statement::Expression(expression))
    }
}

pub fn p_sequence() -> impl Parser<Statement> {
    move |s| {
        let seq = sep_by(Box::new(p_without_sequence()), Box::new(s_character(';')))(s)?;
        Ok(Statement::Sequence(seq))
    }
}

pub fn p_assign() -> impl Parser<Statement> {
    move |s| {
        let _ = s_character('$')(s)?;
        let name = p_ident()(s)?;
        let _ = s_string("=")(s)?;
        let expression = p_expression()(s)?;
        Ok(Statement::Assign(name, expression))
    }
}

pub fn p_if() -> impl Parser<Statement> {
    move |s| {
        let _ = s_string("if ")(s)?;
        let condition = p_expression()(s)?;
        let _ = s_string("{")(s)?;
        let then_branch = p_statement()(s)?;
        let _ = s_string("}")(s)?;
        let _ = s_string("else {")(s)?;
        let else_branch = p_statement()(s)?;
        let _ = s_string("}")(s)?;
        Ok(Statement::If(
            condition,
            Box::new(then_branch),
            Box::new(else_branch),
        ))
    }
}

pub fn p_while() -> impl Parser<Statement> {
    move |s| {
        let _ = s_string("while")(s)?;
        let condition = p_expression()(s)?;
        let _ = s_string("{")(s)?;
        let body = p_statement()(s)?;
        let _ = s_string("}")(s)?;
        Ok(Statement::While(condition, Box::new(body)))
    }
}

pub fn p_pass() -> impl Parser<Statement> {
    move |s| {
        let _ = s_string("pass")(s)?;
        Ok(Statement::Pass)
    }
}
